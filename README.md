# Encoder Simulator for Zephyr

This repository contains a Zephyr application that turns an **STM32F103C8T6**
Blue Pill style board into a **multi-channel quadrature encoder waveform simulator**.

## Target board

- Zephyr board: `stm32_min_dev`
- MCU: `STM32F103C8T6`

## Application location

- `app/`

## Features

- 12 independent encoder channels
- Each channel outputs one quadrature **A/B** pair
- Per-channel enable/disable from the serial console
- Per-channel direction and rate control
- Stopping a channel sets both GPIOs to **high impedance**

## Build

From a Zephyr workspace shell:

```powershell
west build -p always -b stm32_min_dev app
```

## Flash

```powershell
west flash
```

## Serial console

- UART: **USART1**
- Default baudrate: **115200**
- Pins: **PA9/PA10**

## Shell commands

```text
enc list
enc enable <channel> <cycle_hz> [forward|reverse]
enc disable <channel>
enc rate <channel> <cycle_hz>
enc dir <channel> <forward|reverse>
enc stopall
```

Examples:

```text
enc enable 0 100 forward
enc enable 1 250 reverse
enc rate 1 1000
enc disable 0
```

## Channel pin map

| Channel | A pin | B pin |
|---|---|---|
| 0 | PA0  | PA1  |
| 1 | PA2  | PA3  |
| 2 | PA4  | PA5  |
| 3 | PA6  | PA7  |
| 4 | PA8  | PB0  |
| 5 | PB1  | PB5  |
| 6 | PB6  | PB7  |
| 7 | PB8  | PB9  |
| 8 | PB10 | PB11 |
| 9 | PB12 | PB13 |
| 10 | PB14 | PB15 |
| 11 | PA11 | PA12 |

## Notes

- Keep **PA13/PA14** reserved for SWD debug.
- Channel 11 uses **PA11/PA12**. Leave it disabled if you need USB.
- This is a **software-generated** waveform source, so maximum reliable rate
  depends on the number of active channels and system load.
