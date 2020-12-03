#ifndef LOOPSIO_TRAJECTORYSERIALIZER_H
#define LOOPSIO_TRAJECTORYSERIALIZER_H
#include <string>
#include <LoopsLib/Models/DecompositionResult.h>
#include "ISerializer.h"

namespace LoopsIO
{
    using TrajectorSerializer = SerializerFactory<LoopsLib::MovetkGeometryKernel::Trajectory>;
    using TimestampedTrajectorSerializer = SerializerFactory<LoopsLib::MovetkGeometryKernel::TimstampedTrajectory>;
}
#endif