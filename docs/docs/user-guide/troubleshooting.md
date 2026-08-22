# Troubleshooting

## `AstraRocket::init()` returns false

- Read the `LOG/` messages emitted during initialization.
- Confirm that every configured sensor is powered, wired, and on the expected
  bus/address.
- Configure at least the sensors needed by your mission; a default-constructed
  rocket does not invent hardware.
- Verify that mounting transforms are set before `init()`.

## The config chain does not compile

Construct `AstraRocketConfig` first, then call builder methods:

```cpp
AstraRocketConfig config;
config.withFlightLogRate(50);
config.with6DoFIMU(&imu).withBaro(&baro);
```

Inherited methods return `AstraConfig&`, so do not initialize a derived config
from the result of a chained inherited call.

## Flight stages do not advance

- Confirm that the vertical axis and sensor mounting transform are correct.
- Check acceleration and vertical-velocity units against the thresholds in
  [Configuration](configuration.md).
- Recovery phases are inferred from estimated descent rate. They do not prove
  that a physical parachute deployed.

## No telemetry or event logs appear

- Configure sinks with `withDataLogs()` and `withEventLogs()`, or rely on a
  platform default only after verifying that the platform supports it.
- Confirm the selected storage backend is valid for the board.
- Use the Astra logging guide for sink-specific details.

## The repository builds but hardware does not work

The automated matrix compiles firmware and runs native tests. Upload, storage,
sensor, radio, HITL, actuator, and deployment behavior require separate bench
and system tests.
