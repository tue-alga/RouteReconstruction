#ifndef LOOPSIO_DECOMPOSITIONPROVIDERS_JSONDECOMPOSITIONPROVIDER_H
#define LOOPSIO_DECOMPOSITIONPROVIDERS_JSONDECOMPOSITIONPROVIDER_H
#include <LoopsIO/IDecompositionProvider.h>

namespace LoopsIO::DecompositionProviders
{
    class JsonDecompositionProvider : public IDecompositionProvider
    {
    public:
        JsonDecompositionProvider();

        void read(const std::string& filePath, LoopsLib::Helpers::DecompositionObject& decompObj,
            std::function<bool(const std::string&)> dependentMapCb,
            std::function<bool(const std::string&)> dependentFieldCb) override;
        void write(const std::string& filePath, const LoopsLib::Helpers::DecompositionObject& decompObj) override;
    };
}
#endif
