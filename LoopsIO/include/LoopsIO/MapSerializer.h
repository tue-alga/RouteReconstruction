#ifndef LOOPSIO_MAPSERIALIZER_H
#define LOOPSIO_MAPSERIALIZER_H
#include <string>
#include "ISerializer.h"
#include <LoopsLib/DS/EmbeddedGraph.h>

namespace LoopsIO
{
    using MapSerializer = SerializerFactory<LoopsLib::DS::EmbeddedGraph>;
    /*using AllFieldProviders = std::tuple<
        LoopsIO::TrajectorySetProviders::WktCsvTimestampedTrajectorySetProvider,
        LoopsIO::TrajectorySetProviders::WkbCsvTimestampedTrajectorySetProvider>;*/
}
#endif