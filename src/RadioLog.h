#ifndef RADIOLOG_H
#define RADIOLOG_H

#include <RecordData/Logging/LoggingBackend/ILogSink.h>

namespace astra_rocket
{
    using namespace astra;
    class RadioLog : public UARTLog
    {
    public:
        RadioLog(SerialUART_t &s);
        bool begin() override;
    };
}

#endif