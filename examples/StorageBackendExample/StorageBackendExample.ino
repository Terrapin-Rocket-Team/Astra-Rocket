/**
 * Storage Backend Configuration Example
 *
 * Demonstrates how to configure storage backends for different platforms
 * and how to override the default platform-specific storage backend.
 *
 * Platform Defaults:
 * - STM32: EMMC
 * - Teensy: SD_SDIO (built-in SD card)
 * - ESP32: SD_SPI
 *
 * Users can override these defaults using the withStorageBackend() method.
 */

#include <AstraRocket.h>
#include <RecordData/Storage/StorageFactory.h>

using namespace astra;
using namespace astra_rocket;

// Example 1: Using platform defaults (no configuration needed)
void example_defaultStorage() {
    AstraRocket rocket;

    // The storage backend is automatically set based on platform:
    // - STM32: StorageBackend::EMMC
    // - Teensy: StorageBackend::SD_SDIO
    // - ESP32: StorageBackend::SD_SPI

    if (!rocket.init()) {
        Serial.println("Initialization failed!");
        return;
    }

    Serial.println("Using platform default storage backend");
}

// Example 2: Override storage backend for your specific hardware
void example_customStorage() {
    AstraRocketConfig config;

    // Override the default storage backend
    #if defined(ENV_TEENSY)
    // Use SPI SD card instead of SDIO on Teensy
    config.withStorageBackend(StorageBackend::SD_SPI);
    #elif defined(ENV_STM)
    // Use SD card via SDIO instead of EMMC on STM32
    config.withStorageBackend(StorageBackend::SD_SDIO);
    #elif defined(ENV_ESP)
    // ESP32 only has SD_SPI available, but you can still set it explicitly
    config.withStorageBackend(StorageBackend::SD_SPI);
    #endif

    AstraRocket rocket(config);

    if (!rocket.init()) {
        Serial.println("Initialization failed!");
        return;
    }

    Serial.println("Using custom storage backend");
}

// Example 3: Direct storage backend usage (advanced)
void example_directStorageUse() {
    // You can also use storage backends directly without AstraRocket
    #if defined(ENV_STM)
    IStorage* storage = StorageFactory::create(StorageBackend::EMMC);
    #elif defined(ENV_TEENSY)
    IStorage* storage = StorageFactory::create(StorageBackend::SD_SDIO);
    #elif defined(ENV_ESP)
    IStorage* storage = StorageFactory::create(StorageBackend::SD_SPI);
    #endif

    if (!storage) {
        Serial.println("Failed to create storage backend!");
        return;
    }

    if (!storage->begin()) {
        Serial.println("Failed to initialize storage!");
        delete storage;
        return;
    }

    Serial.println("Storage backend initialized successfully!");

    // Use the storage backend for file operations
    IFile* file = storage->openWrite("test.txt", false);
    if (file) {
        const char* message = "Hello from custom storage!\n";
        file->write((const uint8_t*)message, strlen(message));
        file->close();
        delete file;
        Serial.println("File written successfully");
    }

    // Cleanup
    storage->end();
    delete storage;
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("=== Storage Backend Configuration Examples ===");
    Serial.println();

    // Run one of the examples:
    example_defaultStorage();
    // example_customStorage();
    // example_directStorageUse();
}

void loop() {
    // Your flight computer code here
    delay(1000);
}
