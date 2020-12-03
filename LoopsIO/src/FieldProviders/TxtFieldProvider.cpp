#include <LoopsIO/FieldProviders/TxtFieldProvider.h>
#include <LoopsIO/Helpers/InStreamWrapper.h>
#include <LoopsIO/Helpers/OutStreamWrapper.h>
#include <LoopsLib/Models/FlowField.h>
const char* FIELDTXT_TAG = "[FieldTxt]";
const char* PATHS_TAG = "[Paths]";
const char* PATHVALUES_TAG = "[PathValues]";

void LoopsIO::FieldProviders::TxtFieldProvider::read(const std::string& filePath, LoopsLib::Models::FlowField& flowField)
{   
    std::cout << "Reading txtfield " << std::endl;
    std::ifstream stream(filePath);
    if (!stream.is_open())
    {
        throw std::runtime_error("Could not open file for reading:"+filePath);
    }
    Helpers::InStreamWrapper wrapper(stream);
    std::string line;
    std::size_t pathCount;

    wrapper.readLine(FIELDTXT_TAG)
        .readLineTo(line);
    m_associatedGraphFile = line;
    flowField.m_graphFile = line;

    wrapper.readApply([&flowField,&pathCount](const std::size_t& val)
    {
        flowField.m_paths.clear();
        flowField.m_paths.reserve(val);
        pathCount = val;
    })
    .readLine(PATHS_TAG)
    .readMultipleTimes(pathCount, [&flowField](std::size_t i, Helpers::InStreamWrapper& wrapper)
    {
        const auto pathLength = wrapper.read<std::size_t>();
        flowField.m_paths.emplace_back();
        auto& path = flowField.m_paths.back();
        wrapper.readVector(pathLength, path, " ");
    })
    .readLine(PATHVALUES_TAG)
    .readVector(pathCount, flowField.m_pathValues, " ");
    
    // Reconstruct the actual field from the paths.
    std::cout << "Read " << pathCount << " paths " << std::endl;
}

void LoopsIO::FieldProviders::TxtFieldProvider::write(const std::string& filePath,
    const LoopsLib::Models::FlowField& decompObj)
{
    std::ofstream stream(filePath);
    if (!stream.is_open())
    {
        throw std::runtime_error("Could not open file for writing");
    }

    Helpers::OutStreamWrapper wrapper(stream);
    wrapper.writeLn(FIELDTXT_TAG)
        .writeLn(decompObj.m_graphFile)
        .writeLn(decompObj.m_paths.size())
        .writeLn(PATHS_TAG)
        .writeFuncJoined(decompObj.m_paths, [](const std::vector<LoopsLib::DS::BaseGraph::Id_t>& data, Helpers::OutStreamWrapper& wrapper)
    {
        wrapper.write(data.size()).space().writeJoined(data,  " ");
    }, "\n").nl()
        .writeLn(PATHVALUES_TAG)
        .writeJoined(decompObj.m_pathValues, " ").nl();
}
