#ifndef IO_DECOMPOSITIONPROVIDERS_TXTDECOMPOSITIONPROVIDER_H
#define IO_DECOMPOSITIONPROVIDERS_TXTDECOMPOSITIONPROVIDER_H
#include <LoopsIO/IDecompositionProvider.h>

namespace LoopsIO::DecompositionProviders
{
    class TxtDecompositionProvider : public IDecompositionProvider
    {
    public:
        TxtDecompositionProvider()
            : IDecompositionProvider("DecompositionTxt", "DecompositionTxt(*.decomptxt)", "decomptxt")
        {
        }

        void read(const std::string& filePath, LoopsLib::Helpers::DecompositionObject& decompObj,
            std::function<bool(const std::string&)> dependentMapCb,
            std::function<bool(const std::string&)> dependentFieldCb) override;
        void write(const std::string& filePath, const LoopsLib::Helpers::DecompositionObject& decompObj) override;
    };
}
#endif
