#include <LoopsIO/MapProviders/JsonMapProvider.h>
#include <LoopsIO/Helpers/JsonStream.h>
using namespace LoopsLib;
LoopsIO::MapProviders::JsonMapProvider::JsonMapProvider(): IMapProvider("Map txt (*.mapjson)","mapjson")
{
}

void LoopsIO::MapProviders::JsonMapProvider::write(const std::string& outputPath,
    const LoopsLib::DS::EmbeddedGraph& graph)
{
    std::ofstream stream(outputPath);
    if (!stream.is_open()) throw std::runtime_error("Could not open file for writing");
    Helpers::JsonStream wrapper(stream);
    wrapper.beginObject()
    .writeValueField("numberOfVertices", graph.number_of_vertices()).comma()
    .writeValueField("numberOfEdges", graph.number_of_edges()).comma()
    .field("edges").fieldSep().beginArray()
    .commaSeparated(graph.edges(), [](LoopsLib::DS::BaseGraph::Edge* e)
    {
        std::stringstream ss;
        ss << e->m_source->id() << ',' << e->m_sink->id();
        return ss.str();
    })
    .endArray().comma()
    .field("locations").fieldSep().beginArray()
    .commaSeparated(graph.locations(), [](LoopsLib::MovetkGeometryKernel::MovetkPoint pnt)
    {
        std::stringstream ss;
        ss << pnt.get().x() << ',' << pnt.get().y();
        return ss.str();
    })
    .endArray()
    .endObject();
}

void LoopsIO::MapProviders::JsonMapProvider::read(const std::string& inputPath, DS::EmbeddedGraph& decompObj)
{

}
