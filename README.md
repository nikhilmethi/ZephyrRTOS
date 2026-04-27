# Zephyr RTOS Real-Time Health Monitoring System

Embedded system built on the nRF52833 using Zephyr RTOS to simulate a real-time physiological monitoring device.

## Overview
This project implements a full embedded pipeline including:
- State machine–based system control (sleep, awake, error states)
- Multi-threaded scheduling (heartbeat, action signals)
- Timer-driven LED signaling (1–5 Hz physiological range)
- ADC pipelines (single-sample + differential)
- Real-time signal processing (frequency extraction)
- Hardware integration (GPIO, PWM, ADC)

## Architecture
- Zephyr RTOS (threads, timers, interrupts)
- Event-driven design using atomic flags
- Modular drivers for ADC, LED control, and input handling

## Key Features
- Real-time frequency control via user input
- Out-of-phase LED signaling for simulated actuation
- ADC-based input processing and validation
- Oscilloscope-based validation of timing + signals

## My Contribution
Designed and implemented the full embedded firmware, including:
- State machine architecture
- Threading model and timing logic
- ADC data acquisition and analysis pipeline
- Hardware-software integration and debugging

## Notes
Development was done iteratively across multiple branches (ADC, PWM testing, ECG signal processing), then consolidated into this final implementation.
