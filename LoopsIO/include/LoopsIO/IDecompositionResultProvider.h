#ifndef LOOPSIO_IDECOMPOSITIONRESULTPROVIDER_H
#define LOOPSIO_IDECOMPOSITIONRESULTPROVIDER_H
#include "ISerializer.h"
#include <LoopsLib/Models/DecompositionResult.h>
namespace LoopsIO
{
    class IDecompositionResultProvider : public ISerializer
    {
    public:
        IDecompositionResultProvider(const std::string& name, const std::string& fileFilter,
            const std::string& extension)
            : ISerializer(name, fileFilter, extension)
        {
        }

        virtual void write(const std::string& filePath, const LoopsLib::Models::DecompositionResult& result) = 0;
        virtual void read(const std::string& filePath, LoopsLib::Models::DecompositionResult& result) = 0;
    };
}
#endif