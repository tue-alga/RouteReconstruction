#include <LoopsIO/MapProviders/BoostBinaryProvider.h>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#ifdef WIN32
#define WIN32_LEAN_AND_MEAN      // Exclude rarely-used stuff from Windows headers

#include <windows.h>
#include <synchapi.h>
#endif
#include <movetk/geom/GeometryInterface.h>
using namespace LoopsLib;
LoopsIO::MapProviders::BoostBinaryProvider::BoostBinaryProvider(): IMapProvider("Boost binary (*.boostbin)", "boostbin")
{
}

void LoopsIO::MapProviders::BoostBinaryProvider::write(const std::string& outputPath,
    const LoopsLib::DS::EmbeddedGraph& graphIn)
{
    const auto* graph = &graphIn;
    std::ofstream stream(outputPath.c_str(), std::ios::binary);
    if (!stream.is_open())
    {
        throw std::runtime_error("Failed to open output path");
    }
    boost::archive::binary_oarchive out(stream);

    std::string spatRef;
    char* data;
    graphIn.spatialRef().exportToWkt(&data);
    spatRef = std::string(data);

    out & graph->number_of_vertices() & graph->number_of_edges() & spatRef;
    CPLFree(data);
    for(auto* e : graph->edges())
    {
        out & e->m_source->id() & e->m_sink->id();
    }
    for(const auto& v : graph->locations())
    {
        out & v.get().x() & v.get().y();
    }
}

void LoopsIO::MapProviders::BoostBinaryProvider::read(const std::string& inputPath,
    LoopsLib::DS::EmbeddedGraph& graphIn)
{
    auto* graph = &graphIn;
    std::ifstream stream(inputPath.c_str(), std::ios::binary);
    if (!stream.is_open())
    {
        throw std::runtime_error("Failed to open output path");
    }
    boost::archive::binary_iarchive in(stream);

    std::size_t numVerts, numEdges;
    std::string spatRefWkt;
    in & numVerts & numEdges & spatRefWkt;
    graphIn.spatialRef().importFromWkt(spatRefWkt.data());
    std::cout << "Verts-edges in graph: " << numVerts << "," << numEdges << std::endl;
    graph->allocateVertices(numVerts);
    std::cout << "Allocated vertices" << std::endl;
    graph->allocateEdges(numEdges);
    std::cout << "Allocated edges" << std::endl;
#ifdef WIN32
    Sleep(2000);
#endif
    for(std::size_t e = 0; e < numEdges; ++e)
    {
        LoopsLib::DS::BaseGraph::Id_t src, sink;
        in & src & sink;
        graph->initEdge(e, src, sink);
    }
    graph->locations().reserve(numVerts);
    auto mkPoint = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>();
    for(std::size_t v = 0; v < numVerts; ++v)
    {
        LoopsLib::NT x, y;
        in & x & y;
        graph->locations().push_back(mkPoint({x,y}));
    }
}