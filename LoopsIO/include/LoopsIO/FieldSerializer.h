#ifndef LOOPSIO_FIELDSERIALIZER_H
#define LOOPSIO_FIELDSERIALIZER_H
#include <string>
#include "ISerializer.h"
#include "LoopsLib/Models/FlowField.h"

namespace LoopsIO
{
    using FieldSerializer = SerializerFactory<LoopsLib::Models::FlowField>;
    /*using AllFieldProviders = std::tuple<
        LoopsIO::TrajectorySetProviders::WktCsvTimestampedTrajectorySetProvider,
        LoopsIO::TrajectorySetProviders::WkbCsvTimestampedTrajectorySetProvider>;*/
}
#endif