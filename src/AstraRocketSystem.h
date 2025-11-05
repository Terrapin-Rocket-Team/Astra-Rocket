#include <Utils/Astra.h>
#include <Sensors/Baro/Barometer.h>
#include <Sensors/Accel/Accel.h>
#include <Sensors/GPS/GPS.h>
#include <Sensors/Gyro/Gyro.h>
#include <Sensors/Mag/Mag.h>
#include <RecordData/Logging/DataLogger.h>
#include <RecordData/Logging/EventLogger.h>
using namespace astra;

class AstraRocket
{

public:
    AstraRocket(){
        USBLog *log = new USBLog(Serial, 9600, true);
        dataSinks = new ILogSink *[1];
        eventSinks = new ILogSink *[1];
        dataSinks[0] = log;
        eventSinks[0] = log;

        b = nullptr;

        *cfg = (new AstraConfig())->withBBAsync(true).withBuzzerPin(33).withDataLogs(dataSinks, 1);
    }

    bool init();
    bool update();

    State *getState() { return s; }

private:
    AstraConfig *cfg;
    Astra *sys;
    Barometer *b;
    GPS *n;
    Gyro *g;
    Accel *a;
    Mag *m;
    State *s;
    ILogSink **dataSinks;
    ILogSink **eventSinks;
};