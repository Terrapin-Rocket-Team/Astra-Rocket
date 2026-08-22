# Configuration

`AstraRocketConfig` extends Astra's `AstraConfig`. Keep the config alive for as
long as its `AstraRocket`; the rocket stores a reference rather than a copy.

```cpp
AstraRocketConfig config;

config.withLiftoffAccelThreshold(3.0)
      .withLiftoffDetectDuration(100)
      .withBurnoutAccelThreshold(1.5)
      .withApogeeVelocityThreshold(2.0)
      .withLandingVelocityThreshold(1.0)
      .withLandingDetectDuration(3000)
      .withPreflightLogRate(1)
      .withFlightLogRate(50)
      .withPostflightLogRate(1);

config.with6DoFIMU(&imu)
      .withBaro(&baro)
      .withGPS(&gps);

AstraRocket rocket(config);
```

## Flight-detection settings

| Method | Unit | Default | Purpose |
| --- | --- | ---: | --- |
| `withLiftoffAccelThreshold()` | g | 3.0 | Signed vertical acceleration threshold |
| `withLiftoffDetectDuration()` | ms | 100 | Required sustained liftoff acceleration |
| `withBurnoutAccelThreshold()` | g | 1.5 | Upper acceleration threshold for burnout |
| `withApogeeVelocityThreshold()` | m/s | 2.0 | Downward vertical-speed threshold used near apogee |
| `withLandingVelocityThreshold()` | m/s | 1.0 | Maximum absolute vertical speed for landing detection |
| `withLandingDetectDuration()` | ms | 3000 | Required sustained low vertical speed |

Defaults are software defaults, not mission qualification. Review them against
the vehicle, sensors, sampling rates, and recovery plan before flight.

## Logging settings

`withPreflightLogRate()`, `withFlightLogRate()`, and
`withPostflightLogRate()` select phase-specific rates in hertz. Use
`withDataLogs()` and `withEventLogs()` from Astra to provide explicit sinks.

The default storage backend is EMMC for STM32, SD card for Teensy and ESP32,
and none for native builds. Confirm the actual board wiring and filesystem
before relying on a default.

## ARC command handling

`withArcProtocol(address, stream)` enables the optional ARC command bridge on a
transport. ARC setup is independent from HITL. The flight computer integration
must choose the correct node address and transport.

## Inherited Astra settings

Sensor selection, status indicators, logging sinks, HITL/SITL mode, system
name, and additional reporters come from `AstraConfig`. Their authoritative
documentation is in the [Astra user guide](https://terrapin-rocket-team.github.io/Astra/).
