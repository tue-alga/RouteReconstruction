#include <LoopsIO/DecompositionProviders/JsonDecompositionProvider.h>

LoopsIO::DecompositionProviders::JsonDecompositionProvider::JsonDecompositionProvider(): IDecompositionProvider(
    "JSON Decomposition", "JSON Decomposition(*.decompjson)", "decompjson")
{
}

void LoopsIO::DecompositionProviders::JsonDecompositionProvider::read(const std::string& filePath,
    LoopsLib::Helpers::DecompositionObject& decompObj, std::function<bool(const std::string&)> dependentMapCb,
    std::function<bool(const std::string&)> dependentFieldCb)
{
}

void LoopsIO::DecompositionProviders::JsonDecompositionProvider::write(const std::string& filePath,
    const LoopsLib::Helpers::DecompositionObject& decompObj)
{
}
