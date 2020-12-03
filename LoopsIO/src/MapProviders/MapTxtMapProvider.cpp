#include <LoopsIO/MapProviders/MapTxtMapProvider.h>
#include "LoopsIO/Helpers/OutStreamWrapper.h"
#include <movetk/geom/GeometryInterface.h>
using namespace LoopsLib;
using namespace LoopsIO::MapProviders;


void MapTxtMapProvider::write(const std::string& outputPath, const LoopsLib::DS::EmbeddedGraph& graphIn)
{
    const auto* graph = &graphIn;
    std::ofstream stream(outputPath.c_str());
    if (!stream.is_open())
    {
        throw std::runtime_error("Failed to open output path");
    }
    Helpers::OutStreamWrapper str(stream);
    str.writeLn("[MapTxt]")
        .write(graph->number_of_vertices()).space().write(graph->number_of_edges()).nl()
        .writeLn("[Edges]")
        .writeFuncJoined(graph->edges(), [](LoopsLib::DS::BaseGraph::Edge* e, Helpers::OutStreamWrapper& wrap)
    {
        wrap.write(e->m_source->id()).space().write(e->m_sink->id());
    }, "\n").nl()
        .writeLn("[Locations]")
        .writeFuncJoined(graph->locations(),
            [](const LoopsLib::MovetkGeometryKernel::MovetkPoint& point, Helpers::OutStreamWrapper& wrap)
    {
        wrap.write(point.get().x()).space().write(point.get().y());
    }, "\n").nl();
}

void MapTxtMapProvider::read(const std::string& inputPath, LoopsLib::DS::EmbeddedGraph& graphIn)
{
    std::ifstream stream(inputPath.c_str());
    if (!stream.is_open())
    {
        throw std::runtime_error("Failed to open output path");
    }
    std::string line;
    std::stringstream str;
    auto getLine = [&stream, &str, &line]()
    {
        auto result = (bool)std::getline(stream, line);
        str.str(line);
        return result;
    };
    auto getLineOrFail = [&stream, &str, &line](const std::string& msg)
    {
        auto result = (bool)std::getline(stream, line);
        str.str(line);
        str.seekg(0);
        if (!result) throw std::runtime_error(msg);
    };

    getLineOrFail("No opening tag");
    assert(line == "[MapTxt]");
    //getLineOrFail("Missing line for vertex and edge count");
    getLine();
    std::size_t numVerts, numEdges;
    str >> numVerts >> numEdges;
    auto* graph = &graphIn;

    // Allocate in place.
    graph->clear();
    graph->allocateVertices(numVerts);
    graph->allocateEdges(numEdges);

    getLineOrFail("Missing [Edges] tag");
    assert(line == "[Edges]");

    for (std::size_t i = 0; i < numEdges; ++i)
    {
        getLineOrFail("Missing edge");
        LoopsLib::DS::BaseGraph::Id_t src, sink;
        str >> src >> sink;
        graph->initEdge(i, src, sink);
    }
    getLineOrFail("Missing [Locations] tag");
    assert(line == "[Locations]");
    auto mkPoint = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>();
    graph->locations().clear();
    for (std::size_t i = 0; i < numVerts; ++i)
    {
        getLineOrFail("Missing vert");
        LoopsLib::MovetkGeometryKernel::NT x, y;
        str >> x >> y;
        graph->locations().push_back(mkPoint({ x, y }));
    }
}