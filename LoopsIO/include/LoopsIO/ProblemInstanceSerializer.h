#ifndef LOOPSIO_PROBLEMINSTANCESERIALIZER_H
#define LOOPSIO_PROBLEMINSTANCESERIALIZER_H
#include <string>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Models/ProblemInstance.h>
#include <LoopsIO/IMapProvider.h>
#include "ISerializer.h"

namespace LoopsIO
{
    using ProblemInstanceSerializer = SerializerFactory<LoopsLib::Models::ProblemInstance>;
}
#endif