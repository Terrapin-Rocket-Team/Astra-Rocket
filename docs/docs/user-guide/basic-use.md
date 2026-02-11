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

## 3. Choose or Implement Sensors

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

### Write a Sensor Wrapper (Custom Hardware)

If your sensor is not supported, create a thin wrapper by inheriting from the correct Astra interface. The existing wrappers in Astra's `src/Sensors/HW/` folder are the best reference. Astra has the most up-to-date examples; some Astra-Rocket examples lag behind.

**Example: IMU6DoF wrapper**

```cpp title="lib/MyIMU/MyIMU.h"
#pragma once

#include <Sensors/IMU/IMU6DoF.h>
#include <Wire.h>
#include <MyVendorIMU.h>

using namespace astra;

class MyIMU : public IMU6DoF {
public:
    explicit MyIMU(TwoWire* bus = &Wire)
        : IMU6DoF("MyIMU"), driver(*bus) {}

    int init() override {
        if (!driver.begin()) {
            return -1;
        }
        return 0;
    }

    int read() override {
        driver.read();
        acc = Vector<3>(driver.ax_mss(), driver.ay_mss(), driver.az_mss());
        angVel = Vector<3>(driver.gx_rads(), driver.gy_rads(), driver.gz_rads());
        return 0;
    }

private:
    MyVendorIMU driver;
};
```

**Example: Barometer wrapper**

```cpp title="lib/MyBaro/MyBaro.h"
#pragma once

#include <Sensors/Baro/Barometer.h>
#include <MyVendorBaro.h>

using namespace astra;

class MyBaro : public Barometer {
public:
    MyBaro() : Barometer("MyBaro") {}

    int init() override {
        return driver.begin() ? 0 : -1;
    }

    int read() override {
        driver.read();
        altitudeM = driver.altitude_m();
        pressurePa = driver.pressure_pa();
        temperatureC = driver.temperature_c();
        return 0;
    }

private:
    MyVendorBaro driver;
};
```

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

Create an `AstraRocketConfig` and pass your sensors in. Use the IMU convenience helpers when possible:

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

AstraRocketConfig config = AstraRocketConfig()
    .with6DoFIMU(&imu)
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

## Notes on Examples

Astra's `Sensors/HW` wrappers and documentation are kept more current than Astra-Rocket's example sketches. When in doubt, use Astra's sensor wrappers and the latest Astra docs as your reference.

---

## Next Steps

- [Utilities](utils)
- [Interfaces](ifaces)
