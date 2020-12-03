#include <LoopsIO/DecompositionProviders/TxtDecompositionProvider.h>
#include <LoopsIO/Helpers/InStreamWrapper.h>
#include <LoopsIO/Helpers/OutStreamWrapper.h>
using namespace LoopsLib;
const char* DECOMPTXT_TAG = "[DecompTxt]";
const char* BASIS_TAG = "[Basis]";
const char* COEFFICIENTS_TAG = "[Coefficients]";

void LoopsIO::DecompositionProviders::TxtDecompositionProvider::read(const std::string& filePath,
    LoopsLib::Helpers::DecompositionObject& decompObj, std::function<bool(const std::string&)> dependentMapCb,
    std::function<bool(const std::string&)> dependentFieldCb)
{
    std::ifstream stream(filePath);
    if(!stream.is_open())
    {
        throw std::runtime_error("Could not open decomp file");
    }
    Helpers::InStreamWrapper wrapper(stream);
    std::string line;
    std::size_t pathCount;

    // Read file paths
    wrapper.readLine(DECOMPTXT_TAG)
        .readLineTo(line);
    if (!dependentMapCb(line)) return;
    wrapper.readLineTo(line);
    if (!dependentFieldCb(line)) return;

    // Read decomp size.
    std::size_t decompSize;
    wrapper.read(decompSize);
    decompObj.m_basis.clear();
    decompObj.m_basis.reserve(decompSize);
    decompObj.m_decompositionCoeffs.clear();
    decompObj.m_decompositionCoeffs.reserve(decompSize);
    
    wrapper.readLine(BASIS_TAG)
    .readMultipleTimes(decompSize, [&decompObj](std::size_t i, Helpers::InStreamWrapper& wrapper)
    {
        std::size_t elementSize = wrapper.read<std::size_t>();
        decompObj.m_basis.push_back(std::vector<LoopsLib::DS::BaseGraph::Edge*>{});
        auto& path = decompObj.m_basis.back();
        wrapper.readVector(elementSize, path, [&decompObj](std::size_t id) {return decompObj.m_graph->edge(id); }, " ");
    })
    .readLine(COEFFICIENTS_TAG)
    .readVector(decompSize, decompObj.m_decompositionCoeffs, " ");
}

void LoopsIO::DecompositionProviders::TxtDecompositionProvider::write(const std::string& filePath,
    const LoopsLib::Helpers::DecompositionObject& decompObj)
{
    std::ofstream stream(filePath);
    if (!stream.is_open())
    {
        throw std::runtime_error("Could not open file for writing");
    }

    Helpers::OutStreamWrapper wrapper(stream);
    wrapper.writeLn(DECOMPTXT_TAG)
        .writeLn(decompObj.m_graphPath)
        .writeLn(decompObj.m_fieldPath)
        .writeLn(decompObj.m_basis.size())
        .writeLn(BASIS_TAG)
        .writeFuncJoined(decompObj.m_basis, [](const std::vector<LoopsLib::DS::BaseGraph::Edge*>& data, Helpers::OutStreamWrapper& wrapper)
    {
        wrapper.write(data.size()).space().writeJoined(data, [](LoopsLib::DS::BaseGraph::Edge* e) {return e->id(); }, " ");
    }, "\n").nl()
        .writeLn(COEFFICIENTS_TAG)
        .writeJoined(decompObj.m_decompositionCoeffs, " ").nl();
}
