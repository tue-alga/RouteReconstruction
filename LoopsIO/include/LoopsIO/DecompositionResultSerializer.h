#ifndef LOOPSIO_DECOMPOSITIONRESULTSERIALIZER_H
#define LOOPSIO_DECOMPOSITIONRESULTSERIALIZER_H
#include <string>
#include <LoopsLib/Models/DecompositionResult.h>
#include "ISerializer.h"

namespace LoopsIO
{
    using DecompositionResultSerializer = SerializerFactory<LoopsLib::Models::DecompositionResult>;
}
#endif