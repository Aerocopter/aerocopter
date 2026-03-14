# Aerocopter Flight Controller

Aerocopter is an open-source flight controller firmware for STM32F405-based autopilot systems. It provides robust drivers for sensors (including MS5611 barometer) and supports real-time data visualization via VOFA+ protocol. The project is built with ARM GCC toolchain and a custom Makefile, featuring a clean modular architecture for easy customization and expansion.

## Features

- **MCU:** STM32F405 (Cortex-M4 with FPU, 168 MHz)
- **Sensors:**
  - MS5611 barometer (I2C) with temperature compensation
  - Planned support for ICM-42688 ICM-20689 IST8310 IMU
- **Communication:**
  - UART with `printf` redirection for debugging
  - VOFA+ FireWater & JustFloat protocols for 3D visualization
- **Build System:** Makefile-based, using `arm-none-eabi-gcc` and newlib-nano
- **Modular Architecture:** Clean separation of BSP, drivers (`mod/`), and application layers

## Hardware Requirements

- STM32F405 development board (e.g., F405RGT6)
- MS5611 barometer module (I2C interface, address 0x77)
- USB-to-TTL adapter (for UART debugging)
- Optional: ICM-42688 ICM-20689 IST8310

## Software Requirements

- **Toolchain:** GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`)
- **Build Tool:** GNU Make
- **Programming:** ST-Link utility or `dfu-util`
- **Serial Monitor:** `screen`, `minicom`, or VOFA+ (for visualization)

## Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/Aerocopter/aerocopter.git
cd aerocopter
