---
title: Home
hide: footer
---

# Astra-Rocket Documentation

Astra-Rocket is the Terrapin Rocket Team's rocket-specific layer built on top
of [Astra](https://terrapin-rocket-team.github.io/Astra/). It combines Astra's
sensor and state-estimation APIs with rocket flight-stage detection,
flight-oriented logging defaults, and optional ARC command handling.

!!! warning "Recovery detection is not deployment control"
    Astra-Rocket reports expected and observed recovery phases. It does not
    energize pyrotechnic or actuator outputs. Mission firmware must implement
    and validate deployment safety separately.

---

## What Astra-Rocket Gives You

- **Simple flight computer setup**: minimal `setup()` and `loop()` with one `AstraRocket` object
- **Astra integration**: configure Astra IMU, barometer, GPS, magnetometer, and
  miscellaneous sensor objects through one config
- **Flight-stage tracking**: pad, boost, coast, apogee, expected/observed
  drogue, expected/observed main, and landed
- **Logging defaults**: platform-appropriate serial and storage sinks, plus configurable logging rates
- **HITL ready**: easy Hardware-In-The-Loop mode for simulation and testing

---

## Quick Start

1. Read [Installation](user-guide/installation.md)
2. Follow [Basic Usage](user-guide/basic-use.md)
3. Review [Configuration](user-guide/configuration.md)
4. Understand [Flight Stages](user-guide/flight-stages.md)

---

## Core Architecture

**AstraRocket**
- `AstraRocket` wraps Astra's `Astra` system and provides rocket-specific defaults
- `AstraRocketConfig` extends `AstraConfig` with flight-specific settings
- `RocketState` runs Kalman + Mahony filters for position/velocity/orientation

**Astra**
- Owns sensor interfaces, hardware wrappers, state estimation, logging, and
  HITL/SITL transport
- Its documentation is authoritative for those APIs

**Logging**
- Event logs and telemetry logs are configured automatically
- Logging rates change based on detected flight stage

---

## Where To Go Next

- [Installation](user-guide/installation.md)
- [Basic Usage](user-guide/basic-use.md)
- [Configuration](user-guide/configuration.md)
- [Flight Stages](user-guide/flight-stages.md)
- [Astra user guide](https://terrapin-rocket-team.github.io/Astra/)
