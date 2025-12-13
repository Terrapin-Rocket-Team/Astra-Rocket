#include "RadioLog.h"

using namespace astra_rocket;

RadioLog::RadioLog(SerialUART_t &s) : UARTLog(s, 115200, true)
{
}

bool RadioLog::begin()
{
    if (!rdy)
    {
        UARTLog::begin();
        s.println("RAD/PING");
        delay(100);
        char buf[150];
        auto timeout = millis();
        int i;
        while (millis() - timeout < 1000)
        {
            while(s.available())
            {
                i = s.readBytesUntil('\n', buf, sizeof(buf));
                Serial.println(i);
                buf[i] = '\0';
                Serial.println(buf);
                if (!strncmp("RAD/PONG", buf, 8))
                    return rdy = true;
            }
            s.println("RAD/PING");
            delay(100);
        }
        return rdy = false;
    }
    return rdy;
}