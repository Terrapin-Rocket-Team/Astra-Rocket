#include <Sensors/Baro/Barometer.h>
#include <Sensors/GPS/GPS.h>
#include <Sensors/IMU/IMU.h>
#include <Math/Vector.h>
#include <Math/Quaternion.h>

using namespace astra;

class FakeBarometer : public Barometer
{
public:
    FakeBarometer() : Barometer(), fakeAltSet(false), fakeAlt(0)
    {
        initialized = true;
        setName("FakeBarometer");
    }
    ~FakeBarometer() {}

    bool read() override
    {
        pressure = fakeP;
        temp = fakeT;
        return true;
    }

    void set(double p, double t)
    {
        pressure = fakeP = p;
        temp = fakeT = t;
        fakeAltSet = false;  // Using pressure, not direct altitude
    }

    // Helper to set altitude directly (calculates pressure automatically)
    void setAltitude(double altM)
    {
        fakeAlt = altM;
        fakeAltSet = true;
        // Calculate corresponding pressure using standard atmosphere formula
        // P = P0 * (1 - L*h/T0)^(g*M/R*L)
        // Simplified: P = 101325 * (1 - h/44330)^5.255
        pressure = fakeP = 101325.0 * pow(1.0 - altM / 44330.0, 5.255);
        temp = fakeT = 15.0 - altM * 0.0065;  // Standard lapse rate
    }

    double getASLAltM()
    {
        if (fakeAltSet) {
            return fakeAlt;
        }
        // Use base class calculation from pressure
        return Barometer::getASLAltM();
    }

    bool init() override
    {
        return initialized;
    }

    double fakeP = 101325.0;  // Default to sea level
    double fakeT = 20.0;      // Default to 20C
    double fakeAlt = 0.0;
    bool fakeAltSet = false;
};

class FakeGPS : public GPS
{
public:
    FakeGPS() : GPS()
    {
        initialized = true;
        setName("FakeGPS");
    }
    ~FakeGPS() {}

    bool read() override {
        return true;
    }
    void set(double lat, double lon, double alt)
    {
        position.x() = lat;
        position.y() = lon;
        position.z() = alt;
    }
    void setDateTime(int y, int m, int d, int h, int mm, int s)
    {
        year = y;
        month = m;
        day = d;
        hr = h;
        min = mm;
        sec = s;
        snprintf(tod, 12, "%02d:%02d:%02d", hr, min, sec); // size is really 9 but 12 ignores warnings about truncation. IRL it will never truncate
    }

    bool init() override
    {
        return initialized;
    }

    void setHasFirstFix(bool fix)
    {
        hasFix = fix;
        if (fix)
            fixQual = 4;
        else
            fixQual = 0;
    }
    void setFixQual(int qual)
    {
        fixQual = qual;
    }
};

class FakeIMU : public IMU
{
public:
    FakeIMU() : IMU()
    {
        initialized = true;
        setName("FakeIMU");
    }
    ~FakeIMU() {}

    bool read() override
    {
        return true;
    }
    void set(Vector<3> acc, Vector<3> gyro, Vector<3> mag)
    {
        measuredAcc = acc;
        measuredGyro = gyro;
        measuredMag = mag;
    }

    bool init() override
    {
        measuredAcc = Vector<3>{0, 0, -9.8};
        measuredGyro = Vector<3>{0, 0, 0};
        measuredMag = Vector<3>{0, 0, 0};
        orientation = Quaternion{1, 0, 0, 0};
        return initialized;
    }
};