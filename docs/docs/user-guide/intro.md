---
title: User Manual - Introduction
---
# Astra-Rocket User Manual

!!! info
    This documentation focuses on Astra-Rocket, which is built on top of TRT-Astra. Many of the Interfaces and Utilities described here come directly from Astra.

---

Astra-Rocket is a high-level flight computer framework tailored for rocketry. It provides a streamlined workflow while still giving you full access to Astra's underlying interfaces.

The library is organized into two major categories:

---

## Utilities

Utilities are ready-to-use systems you typically do not need to modify.

- **BlinkBuzz**: Non-blocking buzzer and LED patterns
- **Logger**: Event logs and CSV telemetry
- **Math**: Vectors, matrices, and quaternions
- **AstraSystem**: Core system that coordinates sensors and state estimation

---

## Interfaces

Interfaces are base classes you extend for custom behavior or hardware.

- **State**: The rocket state estimator and flight logic interface
- **DataReporter**: Sources of telemetry data
- **Sensor** and sensor types (Barometer, IMU, GPS, etc.)
- **Filters**: Kalman and AHRS filters

For most users, the [Basic Usage](basic-use.md) guide is the fastest way to get started.
