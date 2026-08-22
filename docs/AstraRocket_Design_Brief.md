# AstraRocket: Maintainer's Design Brief

## What It Does

AstraRocket is a high-level wrapper around Astra that provides rocket-specific abstractions. It handles:

1. **Flight stage detection** - 9-stage state machine from pad idle through landing
2. **Configured sensor wiring** - Uses sensor objects supplied through `AstraRocketConfig`
3. **Default logging setup** - Creates platform-appropriate serial and storage sinks when custom sinks are not supplied
4. **Rocket state estimation** - Extends Astra's State with vertical velocity, AGL altitude, apogee prediction

Target users can build a flight computer with ~10 lines of code instead of ~500.

## Design Philosophy

**Convention over configuration**: Provide sensible defaults, allow selective overrides. The minimal control loop is:

```cpp
AstraRocket rocket;
rocket.init();
while(1) rocket.update();
```

For hardware use, configure the required sensor objects before calling `init()`; the default constructor does not scan the buses for them.

Advanced users can override anything via `AstraRocketConfig`.

## Core Components

### 1. RocketState (extends Astra's State)

Implements flight stage detection state machine and tracks rocket-specific telemetry:

**Flight stages** (9 total):
- PAD_IDLE → BOOST (accel > 3G for 100ms)
- BOOST → COAST (signed vertical accel ≤ +1.5G for 200ms)
- COAST → APOGEE (vertical velocity ≤ -2 m/s, or fallback after very long coast)
- APOGEE → EXPECTING_DROGUE (descending)
- EXPECTING_DROGUE → UNDER_DROGUE (after 5 seconds, descent rate is 5-120 m/s for 2 seconds)
- UNDER_DROGUE → EXPECTING_MAIN (altitude < 400m AGL)
- EXPECTING_MAIN → UNDER_MAIN (descent rate slows to 2-10 m/s)
- UNDER_MAIN → LANDED (velocity < 1 m/s for 3s)

**State variables added:**
- Altitude AGL, vertical velocity, vertical acceleration
- Max acceleration, max velocity
- Apogee estimate, time to apogee
- Off-vertical angle
- Flight stage, time in current stage

**Orientation handling:**
- On pad, the system continuously snaps to the closest body axis to "up" (with hysteresis) to infer mounting orientation.
- At liftoff, the frame is locked and the orientation filter runs gyro-only for the remainder of flight.

Liftoff, burnout, apogee, and landing thresholds are configurable through `AstraRocketConfig`. Recovery-stage timing and descent-rate limits remain constants in `RocketState`.

### 2. AstraRocket (main wrapper class)

Coordinates sensor initialization, state updates, and logging. Key methods:
- `init()` - Initializes configured sensors and sets up logging
- `update()` - Updates sensors → state → handles stage transitions → logs data

There is no runtime sensor bus scan in `AstraRocket`; missions must supply their sensor objects through configuration. Hardware initialization fails if a required configured source, such as the barometer, is unavailable.

### 3. AstraRocketConfig (builder pattern)

Provides fluent API for configuration. Currently supports:
- Sensor overrides (barometer, GPS, IMU, high-G accel)
- Flight detection thresholds (liftoff, burnout, apogee, landing)
- Logging rates (preflight: 1Hz, flight: 50Hz, postflight: 1Hz)
- Status indicators (buzzer, LED)

See [AstraRocketConfig.h](src/AstraRocketConfig.h) for full API.

## What's Not Implemented Yet

These were in the original design but aren't critical for flight:

1. **Pyro channel control** - Design doc mentions `PyroController` but it's not in the codebase
2. **Advanced file management** - No automatic flight numbering or file rollover
3. **Flash backup logging** - Config exists but not wired up
4. **Continuity checking** - No active monitoring of e-match continuity
5. **I2C/SPI bus scanning** - Not implemented

Treat these as mission-dependent gaps to evaluate before flight use.

## Extending RocketState

To add custom telemetry columns:

1. Add member variable to RocketState
2. Calculate value in `updateVariables()` override
3. Register with DataReporter in constructor using `registerColumn()`

See [RocketState.cpp](src/RocketState.cpp) for examples.

## Dependencies

- **Astra** (base framework) - handles sensor abstraction, logging, state estimation
- **Arduino ecosystem** - Wire, SPI, Serial for hardware interfaces
- **PlatformIO** - build system

The maintained build matrix covers Teensy 4.1, STM32H723, ESP32-S3, and native host tests.
