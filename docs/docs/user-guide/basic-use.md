---
title: Basic Usage
---

# Basic Usage

This guide walks through the full Astra-Rocket setup: PlatformIO project, library install, sensor wrappers, sensor integration, mounting orientation, and the required `setup()` / `loop()` code.

---

## 1. Install PlatformIO

If you have not already, install the PlatformIO extension in VS Code. See [Installation](installation.md) for the full setup.

---

## 2. Create a Project and Add Astra-Rocket

Create a PlatformIO project and add Astra-Rocket to `lib_deps` as shown in [Installation](installation.md). Astra-Rocket pulls in Astra automatically.

---

## 3. Choose Sensors

### Use a Supported Sensor

If your sensor already exists in Astra's hardware wrappers, include and instantiate it directly:

```cpp title="src/main.cpp"
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/DPS368.h>
#include <Sensors/HW/GPS/MAX_M10S.h>

using namespace astra;

BMI088 imu;
DPS368 baro;
MAX_M10S gps;
```

If hardware is not already supported, follow Astra's
[Sensor Interface](https://terrapin-rocket-team.github.io/Astra/user-guide/ifaces/sensor/)
guide. Astra owns custom sensor wrappers; Astra-Rocket only consumes them.

---

## 4. Set Mounting Orientation (Important)

If a sensor board is rotated relative to the rocket body frame, you **must** set its mounting orientation before initialization. This is critical for correct orientation and flight-stage logic.

```cpp title="src/main.cpp"
#include <Sensors/MountingTransform.h>

imu.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
```

You can set orientation for IMUs, accelerometers, gyros, and magnetometers. If you are unsure, start with `MountingOrientation::IDENTITY` and verify axes on the bench.

---

## 5. Integrate Sensors into Astra-Rocket

Create an `AstraRocketConfig`, then configure it in a separate statement. This
is important because inherited Astra builder methods return `AstraConfig&`.

```cpp title="src/main.cpp"
#include <Arduino.h>
#include <AstraRocket.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/DPS368.h>
#include <Sensors/HW/GPS/MAX_M10S.h>

using namespace astra;
using namespace astra_rocket;

BMI088 imu;
DPS368 baro;
MAX_M10S gps;

AstraRocketConfig config;

// Configure rocket-specific values before or separately from Astra values.
config.withFlightLogRate(50);
config.with6DoFIMU(&imu)
    .withBaro(&baro)
    .withGPS(&gps)
    .withStatusLED(25)
    .withGPSFixLED(26);

AstraRocket rocket(config);
```

If you have a 9-DoF IMU, use `.with9DoFIMU(&imu)` instead. For sensors that should be logged but not fused, use `.withMiscSensor(sensorPtr)`.

---

## 6. Setup and Loop

Astra-Rocket requires only `init()` in `setup()` and `update()` in `loop()`.

```cpp title="src/main.cpp"
void setup() {
    Serial.begin(115200);

    // Set sensor orientation before init()
    imu.setMountingOrientation(MountingOrientation::ROTATE_90_Z);

if (!rocket.init()) {
        while (1) {
            delay(1000);
        }
    }
}

void loop() {
    rocket.update();
}
```

---

## 7. Read Rocket State

```cpp
RocketState* state = rocket.getRocketState();
if (state != nullptr) {
    FlightStage stage = state->getFlightStage();
    double altitudeAGL = state->getAltitudeAGL();
    Vector<3> velocity = state->getVelocity();
}
```

See [Flight Stages](flight-stages.md) before using stage transitions in mission
logic.

## Notes on Examples

Astra's `Sensors/HW` wrappers and documentation are kept more current than Astra-Rocket's example sketches. When in doubt, use Astra's sensor wrappers and the latest Astra docs as your reference.

---

## Next Steps

- [Configuration](configuration.md)
- [Flight Stages](flight-stages.md)
- [Astra user guide](https://terrapin-rocket-team.github.io/Astra/)
