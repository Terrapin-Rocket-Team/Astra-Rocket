Import("env")
import os

# Filter out ESP32-specific files from TRT-Astra library
def skip_esp32_files(node):
    path = node.get_path()
    # Skip ESP32-specific files
    if any(pattern in path for pattern in [
        "RecordData/Logging/SDCard",
        "RecordData\\Logging\\SDCard",
        "StorageFactory.cpp",
        "RetrieveData",
        "SdFatFs.cpp"  # STM32SD has issues
    ]):
        return None
    return node

env.AddBuildMiddleware(skip_esp32_files)
