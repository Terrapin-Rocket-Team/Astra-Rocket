#ifndef LIS3MDL_WRAPPER_H
#define LIS3MDL_WRAPPER_H

#include <LIS3MDL.h>
#include <Wire.h>
#include "../../.pio/libdeps/native/TRT-Astra/src/Sensors/Mag/Mag.h"

namespace astra
{
    /**
     * LIS3MDL Magnetometer Wrapper for Astra Framework
     *
     * This wraps the Pololu LIS3MDL library to work with the Astra sensor framework.
     * The LIS3MDL is a 3-axis magnetometer that can be used with the BMI088 IMU
     * to create a full 9-DoF sensor system.
     *
     * Specifications:
     * - Range: ±4/±8/±12/±16 gauss (default: ±4 gauss)
     * - Resolution: 16-bit
     * - Output data rate: 0.625 to 80 Hz
     * - I2C Address: 0x1C (SA1=LOW) or 0x1E (SA1=HIGH)
     *
     * Gauss to µT conversion: 1 gauss = 100 µT
     */
    class LIS3MDL_Wrapper : public Mag
    {
    public:
        LIS3MDL_Wrapper(const char *name = "LIS3MDL",
                       LIS3MDL::sa1State sa1 = LIS3MDL::sa1_auto)
            : Mag(name), sa1_state(sa1)
        {
        }

        bool init() override
        {
            // Initialize the LIS3MDL
            if (!magnetometer.init(LIS3MDL::device_LIS3MDL, sa1_state))
            {
                return false;
            }

            // Configure for high-performance mode
            // Ultra-high-performance mode for X, Y, Z axes
            // 80 Hz ODR for fast updates
            // ±4 gauss full scale
            // Continuous conversion mode

            // CTRL_REG1: 0x7C = 0b01111100
            // OM = 11 (ultra-high-performance XY), DO = 111 (80 Hz ODR)
            magnetometer.writeReg(LIS3MDL::CTRL_REG1, 0x7C);

            // CTRL_REG2: 0x00 = 0b00000000
            // FS = 00 (+/- 4 gauss full scale)
            magnetometer.writeReg(LIS3MDL::CTRL_REG2, 0x00);

            // CTRL_REG3: 0x00 = 0b00000000
            // MD = 00 (continuous-conversion mode)
            magnetometer.writeReg(LIS3MDL::CTRL_REG3, 0x00);

            // CTRL_REG4: 0x0C = 0b00001100
            // OMZ = 11 (ultra-high-performance mode for Z)
            magnetometer.writeReg(LIS3MDL::CTRL_REG4, 0x0C);

            // CTRL_REG5: 0x40 = 0b01000000
            // BDU = 1 (block data update - prevents reading during update)
            magnetometer.writeReg(LIS3MDL::CTRL_REG5, 0x40);

            // Read a test sample to verify communication
            magnetometer.read();

            return true;
        }

        bool read() override
        {
            // Read raw magnetometer data
            magnetometer.read();

            // Convert from raw LSB to µT (microteslas)
            // For ±4 gauss range: sensitivity = 6842 LSB/gauss
            // 1 gauss = 100 µT
            // Therefore: µT = (raw / 6842) * 100 = raw / 68.42
            const float GAUSS_TO_UT = 100.0f;         // 1 gauss = 100 µT
            const float SENSITIVITY = 6842.0f;         // LSB per gauss for ±4 gauss range
            const float SCALE = GAUSS_TO_UT / SENSITIVITY;

            mag.x() = magnetometer.m.x * SCALE;
            mag.y() = magnetometer.m.y * SCALE;
            mag.z() = magnetometer.m.z * SCALE;

            return true;
        }

        /**
         * Get raw magnetometer values (for debugging)
         */
        void getRaw(int16_t &x, int16_t &y, int16_t &z)
        {
            x = magnetometer.m.x;
            y = magnetometer.m.y;
            z = magnetometer.m.z;
        }

        /**
         * Get the underlying LIS3MDL object for advanced configuration
         */
        LIS3MDL& getLIS3MDL()
        {
            return magnetometer;
        }

    private:
        LIS3MDL magnetometer;
        LIS3MDL::sa1State sa1_state;
    };
}

#endif // LIS3MDL_WRAPPER_H
