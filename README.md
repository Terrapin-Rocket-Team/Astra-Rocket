# Astra-Rocket

An Arduino library that adds rocket flight-state detection, flight-oriented
logging defaults, and ARC command handling to
[Astra](https://github.com/Terrapin-Rocket-Team/Astra).
Astra-Rocket is the foundation consumed by the team's current flight-computer
projects.

It detects recovery phases; it does **not** fire deployment hardware. Pyro and
actuator safety logic remains the responsibility of mission firmware.

## Install in a PlatformIO project

Add the library to the environment's `lib_deps`; Astra is installed
transitively:

```ini
lib_deps =
  https://github.com/Terrapin-Rocket-Team/Astra-Rocket.git#main
```

See the [installation guide](docs/docs/user-guide/installation.md) for complete
Teensy 4.1, STM32H723, and ESP32-S3 examples.

## Development setup

Install the shared support CLI and a native C++ compiler by following the
[Astra-Support setup instructions](https://github.com/Terrapin-Rocket-Team/Astra-Support#install-cli).
Then, from this repository root, run:

```bash
astra-support doctor --project .
astra-support test --project . --clean --no-progress
```

The test command downloads the PlatformIO dependencies, builds both Teensy
environments plus STM32H723, ESP32-S3, and native, and runs the native Unity
tests. The initial run can take several minutes. Hardware upload and HITL tests
require the corresponding device and are not included.

## Documentation

See the [Astra-Rocket user guide](https://terrapin-rocket-team.github.io/Astra-Rocket/)
for installation, configuration, flight stages, and troubleshooting. Use the
[Astra documentation](https://terrapin-rocket-team.github.io/Astra/) for
sensor, logging, filter, and math APIs inherited from Astra.

The current end-to-end firmware integration is documented in
[SRAD-Avionics](https://github.com/Terrapin-Rocket-Team/SRAD-Avionics).
