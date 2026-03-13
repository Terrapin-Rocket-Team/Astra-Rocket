#ifndef UNIT_TEST_SENSORS_COMPAT_H
#define UNIT_TEST_SENSORS_COMPAT_H

// Pull in the upstream native test mocks, but rename FakeBarometer so we can
// supply a local version that matches the newer zero-arg Sensor::update() API.
#define FakeBarometer VendorFakeBarometer
#define override
#include "../../.pio/libdeps/native/NativeTestMocks/UnitTestSensors.h"
#undef override
#undef FakeBarometer

class FakeBarometer : public Barometer
{
public:
    bool _healthy = true;
    double _altitude = 0.0;
    bool _shouldFailInit = false;

    FakeBarometer() : Barometer(), fakeAlt(0), fakeAltSet(false)
    {
        setName("FakeBarometer");
    }
    ~FakeBarometer() {}

    void reset()
    {
        initialized = false;
    }

    int read() override
    {
        pressure = fakeP;
        temp = fakeT;
        healthy = _healthy;
        return 0;
    }

    // Prevent Barometer::update() from recomputing altitude when a test sets it directly.
    int update() override
    {
        if (read() != 0)
            return -1;
        if (!fakeAltSet)
            altitudeASL = calcAltitude(pressure);
        return 0;
    }

    void setAltitude(double altM)
    {
        fakeAlt = altM;
        _altitude = altM;
        fakeAltSet = true;
        fakeP = 101325.0 * pow(1.0 - altM / 44330.0, 5.255);
        fakeT = 15.0 - altM * 0.0065;
        pressure = fakeP;
        temp = fakeT;
        altitudeASL = altM;
    }

    void set(double p, double t)
    {
        pressure = fakeP = p;
        temp = fakeT = t;
        fakeAltSet = false;
    }

    int init() override
    {
        if (_shouldFailInit)
            return -1;
        initialized = true;
        healthy = true;
        return 0;
    }

    bool isHealthy() const override { return _healthy; }

    double fakeP = 101325.0;
    double fakeT = 20.0;
    double fakeAlt = 0.0;
    int fakeAltSet = false;
};

#endif // UNIT_TEST_SENSORS_COMPAT_H
