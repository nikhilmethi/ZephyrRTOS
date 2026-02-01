# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Duke BME 554 (Embedded Medical Devices) lab repository for ECG, temperature sensing, and BLE on the **nRF52833 Development Kit**. Built on Zephyr RTOS using the nRF Connect SDK. Students progressively enable features (GPIO, timers, ADC, PWM, I2C, BLE) through lab exercises.

## Build Commands

### Zephyr Application (requires nRF Connect SDK / west)

```bash
# Initialize west workspace (first time only, run from repo root)
west init -l application
west update

# Build the application
west build application --pristine -b nrf52833dk/nrf52833 --no-sysbuild

# Flash to board
west flash
```

The CMake preset in `application/CMakePresets.json` configures the board target and overlay automatically for IDE builds (nRF Connect for VS Code extension).

### Python Tests

```bash
# Install dependencies
conda env create -f environment.yml   # or: pip install -r requirements.txt

# Run all tests
pytest -v testing/

# Run a single test
pytest -v testing/test_bme554.py::test_read_hex_data
```

## Architecture

### `application/` — Zephyr Embedded Firmware

- **`src/main.c`**: Single-file application implementing GPIO-driven heartbeat LED, button callbacks (sleep/reset), and a 5-state state machine (INIT → AWAKE_ENTRY → AWAKE_RUN → AWAKE_EXIT → SLEEP).
- **`prj.conf`**: Kconfig options. Logging and debug are enabled; SMF, events, ADC, PWM options are commented out for students to enable per lab.
- **`boards/nrf52833dk_nrf52833.overlay`**: Devicetree aliases mapping `heartbeat` → LED0, `sleepbutton` → button0, `resetbutton` → button3.

### `testing/` — Python Analysis & Test Suite

- **`bme554.py`**: Utility module for hex dump parsing, 12-bit two's complement unwrapping (ADC data), linear regression plotting, and confidence interval calculation.
- **`test_bme554.py`**: pytest tests for the utility functions.
- **`conftest.py`**: pytest fixtures loading CSV test data.
- **`technical_report.ipynb`**: Jupyter notebook template for lab reports.

### CI/CD (`.gitlab-ci.yml`)

Two stages: **build** (Zephyr firmware compilation using `zephyrprojectrtos/ci` image) and **test** (Python pytest). Additional CI jobs validate git tags and state diagram presence.

## Key Conventions

- Devicetree aliases (not raw node paths) are used throughout `main.c` for hardware references.
- New peripherals/features are enabled by uncommenting the corresponding `CONFIG_*` lines in `prj.conf`.
- The Python `testing/` directory is independent from the embedded code — it analyzes data collected from the device.
