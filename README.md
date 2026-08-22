# Astra-Rocket

An Arduino library that adds rocket flight states, events, deployment control,
and ARC command handling to [Astra](https://github.com/Terrapin-Rocket-Team/Astra).
Astra-Rocket is the foundation consumed by the team's current flight-computer
projects.

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

See our [documentation](https://terrapin-rocket-team.github.io/Astra-Rocket/) for information on how to install and use.
