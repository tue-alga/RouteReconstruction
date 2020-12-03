#ifndef LOOPSIO_TRAJECTORYSETSERIALIZER_H
#define LOOPSIO_TRAJECTORYSETSERIALIZER_H
#include <string>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Models/ProblemInstance.h>
#include "ISerializer.h"
#include <LoopsIO/TrajectorySetProviders/WkbCsvTrajectorySetProvider.h>
#include <LoopsIO/TrajectorySetProviders/WktCsvTrajectorySetProvider.h>
#include <LoopsIO/TrajectorySetProviders/BoostProviders.h>

namespace LoopsIO
{
    using TrajectorySetSerializer = SerializerFactory<LoopsLib::MovetkGeometryKernel::TrajectorySet>;
    using TimestampedTrajectorySetSerializer = SerializerFactory<LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet>;
    using GraphTrajectorySetSerializer = SerializerFactory<LoopsLib::MovetkGeometryKernel::GraphTrajectorySet>;

    using AllTrajectorySetProviders = std::tuple<
        LoopsIO::TrajectorySetProviders::WktCsvTrajectorySetProvider,
        LoopsIO::TrajectorySetProviders::WkbCsvTrajectorySetProvider>;

    using AllTimestampedTrajectorySetProviders = std::tuple<
        LoopsIO::TrajectorySetProviders::WktCsvTimestampedTrajectorySetProvider,
        LoopsIO::TrajectorySetProviders::WkbCsvTimestampedTrajectorySetProvider>;
    
    using AllGraphTrajectorySetProviders = std::tuple<
        LoopsIO::TrajectorySetProviders::BoostBinGraphTrajectorySetProvider,
        LoopsIO::TrajectorySetProviders::BoostTxtGraphTrajectorySetProvider>;
}
#endif