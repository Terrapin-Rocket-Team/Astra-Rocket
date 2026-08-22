---
title: User Manual - Introduction
---
# Astra-Rocket User Manual

!!! info
    This documentation covers the rocket-specific layer. Use the
    [Astra user guide](https://terrapin-rocket-team.github.io/Astra/) for
    sensors, logging sinks, filters, math types, HITL/SITL transport, and
    other base-library APIs.

---

Astra-Rocket is a high-level flight-computer framework tailored for rocketry.
It supplies `AstraRocket`, `AstraRocketConfig`, `RocketState`, the
`FlightStage` enum, and an optional ARC command bridge.

The normal application flow is:

1. Create an `AstraRocketConfig`.
2. Attach Astra sensor objects and optional logging sinks.
3. Construct one `AstraRocket` with that config.
4. Call `init()` once and `update()` continuously.
5. Read `RocketState` for estimated state and the current flight phase.

Start with [Installation](installation.md) and [Basic Usage](basic-use.md).
Then review [Configuration](configuration.md) and
[Flight Stages](flight-stages.md) before connecting the library to mission
outputs.
