---
title: Installation
---

# Installation

This guide assumes PlatformIO with the Arduino framework.

!!! warning "Important"
    PlatformIO only initializes when VS Code is opened at the **project root** (the folder containing `platformio.ini`). Open that folder directly to avoid missing build tasks and Intellisense.

---

## Prerequisites

- Git
- PlatformIO Core, or VS Code with the PlatformIO extension
- A supported board (Teensy 4.1, STM32, ESP32)
- Basic C++ familiarity

---

## 1. Create a PlatformIO Project

Use the PlatformIO Home screen to create a new project. Choose your board and the Arduino framework.

---

## 2. Add Astra-Rocket to `lib_deps`

Add Astra-Rocket to your `platformio.ini`. Astra-Rocket will pull in Astra automatically.

```ini title="platformio.ini"
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
lib_deps =
  https://github.com/Terrapin-Rocket-Team/Astra-Rocket.git
build_flags =
  -D ENV_TEENSY
```

```ini title="platformio.ini"
[env:stm32h723vehx]
platform = ststm32
board = stm32h723vehx
framework = arduino
lib_deps =
  https://github.com/Terrapin-Rocket-Team/Astra-Rocket.git
  stm32duino/STM32duino STM32SD
  https://github.com/stm32duino/FatFs.git
build_flags =
  -D ENV_STM
```

```ini title="platformio.ini"
[env:esp32]
platform = espressif32
board = esp32-s3-devkitm-1
framework = arduino
lib_deps =
  https://github.com/Terrapin-Rocket-Team/Astra-Rocket.git
build_flags =
  -D ENV_ESP
```

!!! tip
    Pin a specific release by appending a tag, e.g. `https://github.com/Terrapin-Rocket-Team/Astra-Rocket.git#v0.2.0`.

---

## 3. Verify the Build

Create a basic `main.cpp` and build the project. This verifies dependency and
header resolution only; a flight-ready configuration requires real sensors.

```cpp title="src/main.cpp"
#include <Arduino.h>
#include <AstraRocket.h>

using namespace astra_rocket;

AstraRocketConfig config;
AstraRocket rocket(config);

void setup() {
    Serial.begin(115200);
}

void loop() {
    rocket.update();
}
```

If the build succeeds, Astra-Rocket is installed correctly. Continue to
[Basic Usage](basic-use.md) before calling `init()`.

For a checkout of Astra-Rocket itself, use the shared CLI to run the maintained
matrix:

```bash
astra-support doctor --project .
astra-support test --project . --clean --no-progress
```

The result covers compilation and native tests, not hardware upload or flight
validation.
