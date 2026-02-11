---
title: Home
hide: footer
---

# Astra-Rocket Documentation

Astra-Rocket is the Terrapin Rocket Team's high-level flight computer framework built on top of TRT-Astra. It provides a batteries-included workflow for rockets: sensor initialization, state estimation, flight-stage logic, and logging are wired for you so you can focus on mission-specific behavior.

---

## What Astra-Rocket Gives You

- **Simple flight computer setup**: minimal `setup()` and `loop()` with one `AstraRocket` object
- **Sensor auto-detection**: use supported IMU, barometer, GPS, magnetometer, and high-G accelerometers without extra glue
- **Flight-stage tracking**: PAD -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDING
- **Logging out of the box**: preflight, flight, and postflight logging rates
- **HITL ready**: easy Hardware-In-The-Loop mode for simulation and testing

---

## Quick Start

1. Read [Installation](user-guide/installation.md)
2. Follow [Basic Usage](user-guide/basic-use.md)
3. Explore the Interfaces and Utilities for advanced customization

---

## Core Architecture

**AstraRocket**
- `AstraRocket` wraps Astra's `Astra` system and provides rocket-specific defaults
- `AstraRocketConfig` extends `AstraConfig` with flight-specific settings
- `RocketState` runs Kalman + Mahony filters for position/velocity/orientation

**Sensors**
- Uses Astra's sensor interfaces (`Accel`, `Gyro`, `Mag`, `Barometer`, `GPS`, `IMU`)
- Custom hardware is supported via thin wrapper classes

**Logging**
- Event logs and telemetry logs are configured automatically
- Logging rates change based on detected flight stage

---

## Where To Go Next

- [Installation](user-guide/installation.md)
- [Basic Usage](user-guide/basic-use.md)
- [Utilities](user-guide/utils)
- [Interfaces](user-guide/ifaces)
