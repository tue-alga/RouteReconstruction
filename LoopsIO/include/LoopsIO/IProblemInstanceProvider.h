#ifndef LOOPSIO_IPROBLEMINSTANCEPROVIDER_H
#define LOOPSIO_IPROBLEMINSTANCEPROVIDER_H
#include "ISerializer.h"
#include <LoopsLib/Models/ProblemInstance.h>

namespace LoopsLib {
    namespace Models {
        struct DecompositionResult;
    }
}

namespace LoopsIO
{
    class IProblemInstanceProvider : public ISerializer
    {
    public:
        IProblemInstanceProvider(const std::string& name, const std::string& fileFilter,
            const std::string& extension)
            : ISerializer(name, fileFilter, extension)
        {
        }

        virtual void write(const std::string& filePath, const LoopsLib::Models::DecompositionResult& result) = 0;
        virtual void read(const std::string& filePath, LoopsLib::Models::DecompositionResult& result) = 0;
    };
}
#endif