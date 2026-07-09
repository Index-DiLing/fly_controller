# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Quadcopter/drone flight controller firmware targeting **STM32F407ZG** (Cortex-M4, 168MHz). Written in **C++11** using the STM32F4xx **Standard Peripheral Library** (not HAL). Communication with a ground station over USART using a custom binary message protocol.

## Build System

This project uses **EIDE** (Embedded IDE VS Code extension) with **Keil ARMCLANG (AC6)** as the toolchain. No command-line build scripts exist — builds are triggered through the EIDE VS Code extension or Keil uVision.

- **VS Code workspace:** `dl_fly.code-workspace` — open this in VS Code with the EIDE extension installed
- **EIDE config:** `.eide/eide.yml` — defines the `FLY` target, all source files, compiler flags, include paths, linker script, and uploader config
- **Build output:** `build/FLY/` — contains the ELF, HEX, MAP, and `compile_commands.json` (for clangd)
- **Preprocessor defines:** `STM32F40_41xxx`, `USE_STDPERIPH_DRIVER`, `STM32F407xx`
- **Compiler:** ARMCLANG from Keil MDK at `d:\Keil_arm\Keil_v5\ARM\ARMCLANG\bin\armclang.exe`
- **Optimization:** `-O0` (no optimization)
- **Standard library:** STM32F4xx_DSP_StdPeriph_Lib_V1.9.0 at `E:\STM32F407Files\`

### Memory Layout

| Region | Start        | Size     |
|--------|--------------|----------|
| Flash  | `0x08000000` | `0x100000` (1MB) |
| RAM    | `0x20000000` | `0x20000` (128KB) |

### Build from VS Code

Open `dl_fly.code-workspace` in VS Code, then use the EIDE extension sidebar to build the `FLY` target. The build produces `build/FLY/dl_fly.hex` for flashing.

### Flash

Flashing uses **STLink** via SWD at `0x08000000`. Configured in `.eide/eide.yml` under `targets.FLY.uploadConfigMap.STLink`. The EIDE extension handles this from VS Code.

## Architecture

### Entry Point and Main Loop

- `main.cpp:76` — `main()` initializes all peripherals, queries the ground station for configuration params, then enters the control loop
- The control loop runs at ~100Hz (10ms period), reading IMU data → running Madgwick AHRS sensor fusion → cascade PID control → sending DShot motor commands → streaming telemetry back

### Custom `dl::` Library (header-only C++ wrappers)

Located in `DL_LIB/`. Wraps STM32 standard peripheral drivers in ergonomic C++ classes. Key modules:

| File | Purpose |
|------|---------|
| `dl_gpio.hpp` | GPIO pin initialization and manipulation |
| `dl_usart.hpp` | USART with DMA-based async reads |
| `dl_socket.hpp` | Higher-level socket-like abstraction over USART for message passing |
| `dl_iic.hpp` | I2C bus abstraction |
| `dl_imu.hpp` | IMU data types (`IMUGData` struct) and MPU9250 driver |
| `dl_dshot.hpp` | DShot ESC protocol (TIM1, DSHOT300) |
| `dl_pid.hpp` | Cascade PID controllers — rate and angle loops for roll/pitch/yaw + altitude |
| `dl_bme280.hpp` | BME280 barometer/humidity/temperature driver |
| `dl_message.hpp` | Binary message protocol builder (`MessageManager`) for ground station comms |
| `dl_bytebuffer.hpp` | Serialization buffer with typed read/write |
| `dl_log.h` / `dl_log.cpp` | Logger that sends formatted strings through the ground station socket |
| `dl_nvic_it.h` | NVIC interrupt callback registration system |
| `dl_dma.hpp` | DMA abstraction |
| `dl_spi.hpp` | SPI bus abstraction |
| `dl_timer.hpp` | Timer peripheral abstraction |

### Interrupt System

- `stm32f4xx_it.cpp` — Cortex-M4 fault handlers, SysTick (increments `SystemClockMilliseconds`), USART1 and DMA2_Stream5 IRQ handlers
- `dl::DL_IT_set_callback_plus()` / `dl::DL_IT_invoke_callback_plus()` — dynamically register and dispatch interrupt callbacks via `DL_IT_Handlers[81]` array
- SysTick configured at 1kHz → `SystemClockMilliseconds` provides millisecond timing
- `dl::delay_ms()` is a busy-wait loop based on SysTick

### Sensor Fusion & Orientation

- **Madgwick AHRS** in `DL_LIB/DL_AHRS/` — quaternion-based sensor fusion (gyro + accel + mag)
- `MPU9250` on I2C2 (PB10/PB11) provides gyro, accel, and magnetometer data
- `BME280` on the same I2C bus provides temperature, humidity, pressure
- `convertQuantToEuler()` macro in `global.h` converts quaternions to Euler angles (roll/pitch/yaw in degrees, 0–360 range with offset)
- Quaternion is initialized at startup: `q0=0, q1=-0.77, q2=-0.63, q3=0`

### PID Control Architecture

Cascade PID defined in `dl_pid.hpp` (all `inline` functions):

1. **Angle loop** (outer): `pid_roll()`, `pid_pitch()`, `pid_yaw()` — set target angles, output target angular rates
2. **Rate loop** (inner): `pid_roll_rate()`, `pid_pitch_rate()`, `pid_yaw_rate()` — set target angular rates, output motor throttle values
3. **Altitude loop:** `pid_z()` — target Z position → direct throttle adjustment
4. **Orchestrators:** `PIDAngleControl(ds)` → `PIDRateControl()` called each iteration
5. **Motor mixer:** throttle adjustments applied per the X-quad layout (see diagram in `global.h:77-84`)
6. All PID gains are `#define` macros in `global.h` (lines 126-183)

### Communication Protocol

Binary message protocol between flight controller and ground station over USART1 (PA9/PA10) at 1,520,000 baud:

- **Message types** defined in `dl_message.hpp` via `MessageManager` — typed messages with ID byte prefix (msg IDs: 0=b'\0'bug, 1=log, 2=pose, 4=BME280, 5=motor, 6=sync, 9=reqFloat, 10=reqBool, 11=reqInt, 19=init, 37=error)
- **Startup handshake:** FC requests 4 boolean params (unlock, testSensor, delayStart, enableDshot), then requests 4 int params (motor start values, timing config)
- **Telemetry stream:** Each loop iteration sends pose (quaternion + raw IMU) and motor throttle values
- Socket class provides both blocking (`read`) and async (`ASyncRead`/`ASyncWait`) I/O

### Sensor Testing Mode

If `testSensor` param is `true` at startup, the main loop runs for >10 seconds with verbose YPR logging before entering the normal control loop. This is for bench testing sensor data.

## Pin Assignments

| Pin  | Function          |
|------|-------------------|
| PF3  | IMU status LED    |
| PF4  | Unlock status LED |
| PF5  | General output    |
| PA9  | USART1 TX (ground station) |
| PA10 | USART1 RX (ground station) |
| PB10 | I2C2 SCL (IMU/BME280) |
| PB11 | I2C2 SDA (IMU/BME280) |

## Global State

**All** global variables are declared in `global.h` and defined across `global.cpp`, `stm32f4xx_it.cpp`, `main.cpp`, and `dl_pid.hpp`. Key globals:

- `SystemClockMilliseconds` — 1kHz tick counter (defined in `stm32f4xx_it.cpp`)
- `throttleValue[4]` / `throttleValueF[4]` — motor throttle outputs
- `q0..q3` / `roll, pitch, yaw` — orientation quaternion and Euler angles
- `mpu9250_data` — latest IMU readings
- `bmeData` — latest BME280 readings
- `sampleFreq` — measured loop frequency in Hz
- PID state variables (`target_roll`, `integral_delta_roll`, `last_lose_roll`, etc.)
- `globalErrorCode` — system error state

## Third-Party Components

- **FatFs** (`DL_LIB/Fatfs/`) — FAT filesystem for SD card
- **SDIO driver** (`DL_LIB/SDIO/`) — STM324x7I-EVAL SDIO driver adapted for this board
- **Madgwick AHRS** (`DL_LIB/DL_AHRS/`) — attitude estimation algorithm (C port of Sebastian Madgwick's algorithm)
- **MPU6050/MPU9250 drivers** (`DL_LIB/mpu/`) — alternative driver implementations
- All third-party code is vendored directly under `DL_LIB/`

## Project Conventions

- **Header-only libraries:** Most `dl_*.hpp` files define functions as `inline` in headers — no separate `.cpp` files
- **Include style:** `#include "dl_xxx.hpp"` for local libs, angle brackets are not used for project includes
- **Formatting:** `.clang-format` uses Microsoft base style, 4-space indent, Linux brace style, no column limit
- **clangd config:** `.clangd` points to `build/FLY/compile_commands.json` and maps Keil ARMCLANG include paths
- **.gitignore:** Ignores Keil uVision files, build outputs, EIDE temp files; `DL_LIB` source is tracked, `mains/` and `DebugConfig/` are excluded
- The `COMPILE_MPU` macro in `main.cpp` gates MPU9250 code — always defined currently
- C++ exception handling is disabled (`-fno-rtti` is set, likely `-fno-exceptions` too)
- Dynamic memory (`new`/`delete`) is used for buffers but never freed — acceptable in embedded firmware that runs forever
