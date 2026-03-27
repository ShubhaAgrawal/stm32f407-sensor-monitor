# STM32F407 Real-Time Sensor Acquisition System

## Overview

FreeRTOS-based multi-task sensor platform on the STM32F407 Discovery board (ARM Cortex-M4 with hardware FPU). Reads the onboard LIS3DSH 3-axis accelerometer via SPI and internal temperature via DMA-driven ADC, with inter-task communication through osMessageQueue and formatted JSON output over UART.

## Hardware

- STM32F407 Discovery Board (STM32F407VG)
- Onboard LIS3DSH 3-axis accelerometer (SPI1, CS on PE3)
- Internal temperature sensor (ADC1, Channel 18)

## Architecture

```
┌─────────────┐     osMessageQueue     ┌──────────────┐
│ SensorTask  │ ───────────────────►   │ DisplayTask  │
│ (high prio) │                        │ (low prio)   │
│             │                        │              │
│ • SPI read  │                        │ • JSON format│
│   (accel)   │                        │   (cJSON)    │
│ • DMA ADC   │                        │ • UART tx    │
│   (temp)    │                        │              │
└─────────────┘                        └──────────────┘
```

- **SensorTask**: Periodic acquisition — reads X/Y/Z acceleration from LIS3DSH over SPI and internal temperature through DMA-driven ADC. Packages data into a struct and pushes to osMessageQueue.
- **DisplayTask**: Blocks on the message queue, formats received sensor data as JSON using the cJSON library, and transmits over UART at 115200 baud.

This separation ensures sensor acquisition timing is never affected by string formatting or serial transmission overhead.

## Peripherals

| Peripheral | Configuration | Purpose |
|-----------|---------------|---------|
| SPI1 | Full-duplex master, 5.25 MHz (84 MHz / 16), CPOL=Low, CPHA=1Edge, software CS (PE3) | LIS3DSH accelerometer communication |
| ADC1 | Channel 18, 480-cycle sampling time, DMA-driven | Internal temperature sensor |
| TIM2 | Periodic interrupt (1 Hz) | Timed sensor acquisition trigger |
| USART2 | 115200 baud, 8N1 | Formatted data output to host |
| GPIO | PE3 (SPI CS), LEDs (PD12–PD15), EXTI (PA0 user button) | Chip select, status indicators, external interrupt |

## Key Implementation Details

- **SPI chip select**: Managed manually via GPIO (software NSS) — CS pulled LOW before each transaction, HIGH after
- **LIS3DSH register access**: Read operations set bit 7 of the address byte (`addr | 0x80`); write operations leave bit 7 clear
- **WHO_AM_I self-test**: Reads register 0x0F at startup; expects 0x3F to confirm accelerometer is alive
- **ADC calibration**: Uses factory calibration values from OTP area rather than datasheet typical coefficients for improved temperature accuracy
- **FreeRTOS**: CMSIS-RTOS v2 API, priority-based preemptive scheduling

## Build

Built with STM32CubeIDE. The `.ioc` file contains the full peripheral configuration and can be opened in STM32CubeMX to inspect or regenerate initialization code.

## Status

Core functionality complete — SPI accelerometer acquisition, DMA-driven ADC temperature reading, FreeRTOS task architecture with message queue, and UART JSON output all working. Ongoing refinements planned.

## Tools

STM32CubeIDE, FreeRTOS (CMSIS-RTOS v2), STM32 HAL, cJSON
