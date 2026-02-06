# AstraRocket: Maintainer's Design Brief

## What It Does

AstraRocket is a high-level wrapper around TRT-Astra that provides rocket-specific abstractions. It handles:

1. **Flight stage detection** - 9-stage state machine from pad idle through landing
2. **Sensor auto-detection** - Tries common sensors if not explicitly configured
3. **Flight-phase logging** - Switches log rates based on flight stage
4. **Rocket state estimation** - Extends Astra's State with vertical velocity, AGL altitude, apogee prediction

Target users can build a flight computer with ~10 lines of code instead of ~500.

## Design Philosophy

**Convention over configuration**: Provide sensible defaults, allow selective overrides. The minimal example works out-of-box:

```cpp
AstraRocket rocket;
rocket.init();
while(1) rocket.update();
```

Advanced users can override anything via `AstraRocketConfig`.

## Core Components

### 1. RocketState (extends Astra's State)

Implements flight stage detection state machine and tracks rocket-specific telemetry:

**Flight stages** (9 total):
- PAD_IDLE → BOOST (accel > 3G for 100ms)
- BOOST → COAST (accel < 1.5G for 200ms)
- COAST → APOGEE (vertical velocity < 2 m/s)
- APOGEE → EXPECTING_DROGUE (descending)
- EXPECTING_DROGUE → UNDER_DROGUE (descent rate slows to 5-40 m/s)
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

All thresholds are hardcoded in [RocketState.h:134-147](src/RocketState.h#L134-L147). Future: make configurable via AstraRocketConfig.

### 2. AstraRocket (main wrapper class)

Coordinates sensor initialization, state updates, and logging. Key methods:
- `init()` - Auto-detects/initializes sensors, sets up logging
- `update()` - Updates sensors → state → handles stage transitions → logs data

Sensor auto-detection tries common TRT sensors in priority order (see [AstraRocket.h:147-150](src/AstraRocket.h#L147-L150)). Falls back to nullptr if none found.

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
5. **I2C/SPI bus scanning** - Auto-detect just tries `sensor.begin()` instead

The library is flight-ready without these. Add them if needed for your mission.

## Extending RocketState

To add custom telemetry columns:

1. Add member variable to RocketState
2. Calculate value in `updateVariables()` override
3. Register with DataReporter in constructor using `registerColumn()`

See [RocketState.cpp](src/RocketState.cpp) for examples.

## Dependencies

- **TRT-Astra** (base framework) - handles sensor abstraction, logging, state estimation
- **Arduino ecosystem** - Wire, SPI, Serial for hardware interfaces
- **PlatformIO** - build system

Targets Teensy 4.x but should work on any Arduino-compatible board with enough RAM.
