/*
 * Copyright (c) 2026 Hake Huang
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * Quadrature encoder waveform simulator for STM32F103C8T6 (Blue Pill / stm32_min_dev).
 *
 * Features:
 * - Up to 12 independent quadrature output channels (A/B pair per channel)
 * - Shell control over UART1 console
 * - Per-channel start/stop, direction, and rate changes
 * - Stop drives both GPIOs to high impedance using GPIO_DISCONNECTED
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define CH_COUNT 12
#define DEFAULT_CYCLE_HZ 100
#define MAX_CYCLE_HZ 20000U
#define IDLE_SLEEP_MS 20

struct encoder_channel {
	const char *name;
	struct gpio_dt_spec a;
	struct gpio_dt_spec b;
	bool running;
	int8_t dir;
	uint8_t phase;
	uint32_t cycle_hz;
	uint32_t phase_period_us;
	uint64_t next_edge_us;
};

static struct encoder_channel channels[CH_COUNT] = {
	{
		.name = "ch0",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc0a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc0b), gpios),
	},
	{
		.name = "ch1",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc1a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc1b), gpios),
	},
	{
		.name = "ch2",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc2a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc2b), gpios),
	},
	{
		.name = "ch3",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc3a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc3b), gpios),
	},
	{
		.name = "ch4",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc4a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc4b), gpios),
	},
	{
		.name = "ch5",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc5a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc5b), gpios),
	},
	{
		.name = "ch6",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc6a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc6b), gpios),
	},
	{
		.name = "ch7",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc7a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc7b), gpios),
	},
	{
		.name = "ch8",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc8a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc8b), gpios),
	},
	{
		.name = "ch9",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc9a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc9b), gpios),
	},
	{
		.name = "ch10",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc10a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc10b), gpios),
	},
	{
		.name = "ch11",
		.a = GPIO_DT_SPEC_GET(DT_ALIAS(enc11a), gpios),
		.b = GPIO_DT_SPEC_GET(DT_ALIAS(enc11b), gpios),
	},
};

/* Forward sequence: A leads B => 00 -> 10 -> 11 -> 01 -> 00 */
static const uint8_t phase_lut[4][2] = {
	{0U, 0U},
	{1U, 0U},
	{1U, 1U},
	{0U, 1U},
};

static struct k_spinlock channels_lock;
static K_THREAD_STACK_DEFINE(generator_stack, 1536);
static struct k_thread generator_thread_data;

static uint64_t now_us(void)
{
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

static int channel_check_ready(size_t idx)
{
	if (idx >= ARRAY_SIZE(channels)) {
		return -EINVAL;
	}

	if (!gpio_is_ready_dt(&channels[idx].a) || !gpio_is_ready_dt(&channels[idx].b)) {
		return -ENODEV;
	}

	return 0;
}

static int channel_drive_phase(size_t idx, uint8_t phase)
{
	int ret;

	ret = gpio_pin_set_dt(&channels[idx].a, phase_lut[phase & 0x3U][0]);
	if (ret < 0) {
		return ret;
	}

	return gpio_pin_set_dt(&channels[idx].b, phase_lut[phase & 0x3U][1]);
}

static int channel_to_hiz(size_t idx)
{
	int ret_a = gpio_pin_configure_dt(&channels[idx].a, GPIO_DISCONNECTED);
	int ret_b = gpio_pin_configure_dt(&channels[idx].b, GPIO_DISCONNECTED);

	if (ret_a < 0) {
		return ret_a;
	}

	return ret_b;
}

static int channel_prepare_output(size_t idx)
{
	int ret;

	ret = gpio_pin_configure_dt(&channels[idx].a, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&channels[idx].b, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		(void)gpio_pin_configure_dt(&channels[idx].a, GPIO_DISCONNECTED);
		return ret;
	}

	return 0;
}

static int channel_start(size_t idx, uint32_t cycle_hz, int dir)
{
	uint64_t phase_period_us;
	k_spinlock_key_t key;
	int ret;

	ret = channel_check_ready(idx);
	if (ret < 0) {
		return ret;
	}

	if ((cycle_hz == 0U) || (cycle_hz > MAX_CYCLE_HZ)) {
		return -EINVAL;
	}

	phase_period_us = DIV_ROUND_CLOSEST((uint64_t)1000000U, (uint64_t)cycle_hz * 4U);
	if (phase_period_us == 0U) {
		phase_period_us = 1U;
	}

	ret = channel_prepare_output(idx);
	if (ret < 0) {
		return ret;
	}

	ret = channel_drive_phase(idx, 0U);
	if (ret < 0) {
		(void)channel_to_hiz(idx);
		return ret;
	}

	key = k_spin_lock(&channels_lock);
	channels[idx].running = true;
	channels[idx].dir = (dir >= 0) ? 1 : -1;
	channels[idx].phase = 0U;
	channels[idx].cycle_hz = cycle_hz;
	channels[idx].phase_period_us = (uint32_t)phase_period_us;
	channels[idx].next_edge_us = now_us() + phase_period_us;
	k_spin_unlock(&channels_lock, key);

	return 0;
}

static int channel_stop(size_t idx)
{
	k_spinlock_key_t key;
	int ret;

	ret = channel_check_ready(idx);
	if (ret < 0) {
		return ret;
	}

	key = k_spin_lock(&channels_lock);
	channels[idx].running = false;
	channels[idx].cycle_hz = 0U;
	channels[idx].phase_period_us = 0U;
	k_spin_unlock(&channels_lock, key);

	return channel_to_hiz(idx);
}

static int parse_channel(const char *arg, size_t *idx)
{
	char *end;
	unsigned long value;

	value = strtoul(arg, &end, 0);
	if ((end == arg) || (*end != '\0') || (value >= ARRAY_SIZE(channels))) {
		return -EINVAL;
	}

	*idx = (size_t)value;
	return 0;
}

static int parse_direction(const char *arg, int *dir)
{
	if ((strcmp(arg, "f") == 0) || (strcmp(arg, "forward") == 0) ||
	    (strcmp(arg, "+") == 0) || (strcmp(arg, "cw") == 0)) {
		*dir = 1;
		return 0;
	}

	if ((strcmp(arg, "r") == 0) || (strcmp(arg, "reverse") == 0) ||
	    (strcmp(arg, "-") == 0) || (strcmp(arg, "ccw") == 0)) {
		*dir = -1;
		return 0;
	}

	return -EINVAL;
}

static int parse_rate(const char *arg, uint32_t *rate)
{
	char *end;
	unsigned long value;

	value = strtoul(arg, &end, 0);
	if ((end == arg) || (*end != '\0') || (value == 0U) || (value > MAX_CYCLE_HZ)) {
		return -EINVAL;
	}

	*rate = (uint32_t)value;
	return 0;
}

static void print_channel_status(const struct shell *shell, size_t idx)
{
	bool running;
	int dir;
	uint8_t phase;
	uint32_t cycle_hz;
	uint32_t phase_period_us;
	k_spinlock_key_t key;

	key = k_spin_lock(&channels_lock);
	running = channels[idx].running;
	dir = channels[idx].dir;
	phase = channels[idx].phase;
	cycle_hz = channels[idx].cycle_hz;
	phase_period_us = channels[idx].phase_period_us;
	k_spin_unlock(&channels_lock, key);

	shell_print(shell,
		    "%s idx=%u A=%s.%u B=%s.%u state=%s dir=%s cycle_hz=%u phase=%u edge_us=%u",
		    channels[idx].name, (unsigned int)idx,
		    channels[idx].a.port->name, channels[idx].a.pin,
		    channels[idx].b.port->name, channels[idx].b.pin,
		    running ? "running" : "hi-z",
		    (dir >= 0) ? "forward" : "reverse",
		    cycle_hz, phase, phase_period_us);
}

static void generator_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		struct {
			size_t idx;
			uint8_t phase;
		} updates[CH_COUNT];
		size_t update_count = 0U;
		uint64_t now = now_us();
		uint64_t nearest = now + 1000U;
		bool any_running = false;
		k_spinlock_key_t key;

		key = k_spin_lock(&channels_lock);
		for (size_t i = 0; i < ARRAY_SIZE(channels); ++i) {
			if (!channels[i].running) {
				continue;
			}

			any_running = true;

			if ((int64_t)(now - channels[i].next_edge_us) >= 0) {
				channels[i].phase = (uint8_t)((channels[i].phase +
					((channels[i].dir >= 0) ? 1U : 3U)) & 0x3U);
				updates[update_count].idx = i;
				updates[update_count].phase = channels[i].phase;
				update_count++;

				do {
					channels[i].next_edge_us += channels[i].phase_period_us;
				} while ((int64_t)(now - channels[i].next_edge_us) >= 0);
			}

			if (channels[i].next_edge_us < nearest) {
				nearest = channels[i].next_edge_us;
			}
		}
		k_spin_unlock(&channels_lock, key);

		for (size_t i = 0; i < update_count; ++i) {
			(void)channel_drive_phase(updates[i].idx, updates[i].phase);
		}

		if (!any_running) {
			k_sleep(K_MSEC(IDLE_SLEEP_MS));
			continue;
		}

		now = now_us();
		if ((int64_t)(nearest - now) > 1500) {
			k_sleep(K_MSEC(1));
		} else if ((int64_t)(nearest - now) > 100) {
			k_usleep((int32_t)(nearest - now - 50));
		} else if ((int64_t)(nearest - now) > 10) {
			k_busy_wait((uint32_t)(nearest - now - 5));
		} else {
			k_yield();
		}
	}
}

static int cmd_enc_list(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (size_t i = 0; i < ARRAY_SIZE(channels); ++i) {
		print_channel_status(shell, i);
	}

	return 0;
}

static int cmd_enc_enable(const struct shell *shell, size_t argc, char **argv)
{
	size_t idx;
	uint32_t rate;
	int dir = 1;
	int ret;

	ret = parse_channel(argv[1], &idx);
	if (ret < 0) {
		shell_error(shell, "invalid channel, expected 0..%u", (unsigned int)(CH_COUNT - 1));
		return ret;
	}

	ret = parse_rate(argv[2], &rate);
	if (ret < 0) {
		shell_error(shell, "invalid cycle_hz, expected 1..%u", MAX_CYCLE_HZ);
		return ret;
	}

	if (argc >= 4) {
		ret = parse_direction(argv[3], &dir);
		if (ret < 0) {
			shell_error(shell, "invalid direction, use forward|reverse");
			return ret;
		}
	}

	ret = channel_start(idx, rate, dir);
	if (ret < 0) {
		shell_error(shell, "enable failed: %d", ret);
		return ret;
	}

	print_channel_status(shell, idx);
	return 0;
}

static int cmd_enc_disable(const struct shell *shell, size_t argc, char **argv)
{
	size_t idx;
	int ret;

	ret = parse_channel(argv[1], &idx);
	if (ret < 0) {
		shell_error(shell, "invalid channel, expected 0..%u", (unsigned int)(CH_COUNT - 1));
		return ret;
	}

	ret = channel_stop(idx);
	if (ret < 0) {
		shell_error(shell, "disable failed: %d", ret);
		return ret;
	}

	print_channel_status(shell, idx);
	return 0;
}

static int cmd_enc_rate(const struct shell *shell, size_t argc, char **argv)
{
	size_t idx;
	uint32_t rate;
	k_spinlock_key_t key;
	int ret;
	uint64_t phase_period_us;

	ret = parse_channel(argv[1], &idx);
	if (ret < 0) {
		shell_error(shell, "invalid channel, expected 0..%u", (unsigned int)(CH_COUNT - 1));
		return ret;
	}

	ret = parse_rate(argv[2], &rate);
	if (ret < 0) {
		shell_error(shell, "invalid cycle_hz, expected 1..%u", MAX_CYCLE_HZ);
		return ret;
	}

	phase_period_us = DIV_ROUND_CLOSEST((uint64_t)1000000U, (uint64_t)rate * 4U);
	if (phase_period_us == 0U) {
		phase_period_us = 1U;
	}

	key = k_spin_lock(&channels_lock);
	channels[idx].cycle_hz = rate;
	channels[idx].phase_period_us = (uint32_t)phase_period_us;
	if (channels[idx].running) {
		channels[idx].next_edge_us = now_us() + phase_period_us;
	}
	k_spin_unlock(&channels_lock, key);

	print_channel_status(shell, idx);
	return 0;
}

static int cmd_enc_dir(const struct shell *shell, size_t argc, char **argv)
{
	size_t idx;
	int dir;
	k_spinlock_key_t key;
	int ret;

	ret = parse_channel(argv[1], &idx);
	if (ret < 0) {
		shell_error(shell, "invalid channel, expected 0..%u", (unsigned int)(CH_COUNT - 1));
		return ret;
	}

	ret = parse_direction(argv[2], &dir);
	if (ret < 0) {
		shell_error(shell, "invalid direction, use forward|reverse");
		return ret;
	}

	key = k_spin_lock(&channels_lock);
	channels[idx].dir = (dir >= 0) ? 1 : -1;
	k_spin_unlock(&channels_lock, key);

	print_channel_status(shell, idx);
	return 0;
}

static int cmd_enc_stopall(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (size_t i = 0; i < ARRAY_SIZE(channels); ++i) {
		int ret = channel_stop(i);

		if (ret < 0) {
			shell_error(shell, "disable ch%u failed: %d", (unsigned int)i, ret);
			return ret;
		}
	}

	shell_print(shell, "all channels stopped and set to high impedance");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	enc_subcmds,
	SHELL_CMD(list, NULL, "Show all channels and GPIO assignments", cmd_enc_list),
	SHELL_CMD_ARG(enable, NULL, "enable <ch> <cycle_hz> [forward|reverse]", cmd_enc_enable, 3, 1),
	SHELL_CMD_ARG(disable, NULL, "disable <ch> and set both GPIOs to high-Z", cmd_enc_disable, 2, 0),
	SHELL_CMD_ARG(rate, NULL, "rate <ch> <cycle_hz>", cmd_enc_rate, 3, 0),
	SHELL_CMD_ARG(dir, NULL, "dir <ch> <forward|reverse>", cmd_enc_dir, 3, 0),
	SHELL_CMD(stopall, NULL, "Stop all channels and set all pins to high-Z", cmd_enc_stopall),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(enc, &enc_subcmds, "Quadrature encoder waveform generator", NULL);

int main(void)
{
	printk("Encoder simulator for STM32F103C8T6\n");
	printk("UART shell is on USART1 @ 115200 bps\n");
	printk("Command example: enc enable 0 %u forward\n", DEFAULT_CYCLE_HZ);

	for (size_t i = 0; i < ARRAY_SIZE(channels); ++i) {
		int ret = channel_check_ready(i);

		if (ret < 0) {
			printk("ch%u gpio not ready: %d\n", (unsigned int)i, ret);
			continue;
		}

		ret = channel_stop(i);
		if (ret < 0) {
			printk("ch%u initial hi-z failed: %d\n", (unsigned int)i, ret);
		}
	}

	(void)k_thread_create(&generator_thread_data, generator_stack,
		      K_THREAD_STACK_SIZEOF(generator_stack),
		      generator_thread, NULL, NULL, NULL,
		      2, 0, K_NO_WAIT);
		k_thread_name_set(&generator_thread_data, "enc_gen");

	return 0;
}
