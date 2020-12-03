#include <PyLoops/Bindings/ds/PyGraph.h>
#include <LoopsIO/GraphIO.h>
#include <LoopsIO/MapProviders.h>
#include <movetk/geom/GeometryInterface.h>

void PyLoops::ds::PyGraph::registerPy(pybind11::module& mod)
{
    namespace py = pybind11;

    // Register all map providers
    LoopsIO::GraphIO::registerProvider(new LoopsIO::MapProviders::MapTxtMapProvider());
    LoopsIO::GraphIO::registerProvider(new LoopsIO::MapProviders::JsonMapProvider());
    LoopsIO::GraphIO::registerProvider(new LoopsIO::MapProviders::OsmXMLMapProvider());
    LoopsIO::GraphIO::registerProvider(new LoopsIO::MapProviders::BoostBinaryProvider());

    // Register edge and vertex
    py::class_<LoopsLib::DS::BaseGraph::Edge>(mod, "Edge")
        .def("id", &LoopsLib::DS::BaseGraph::Edge::id)
        .def("src", [](const LoopsLib::DS::BaseGraph::Edge& edge)
    {
        return edge.m_source->id();
    }).def("sink", [](const LoopsLib::DS::BaseGraph::Edge& edge)
    {
        return edge.m_sink->id();
    });
    py::class_<LoopsLib::DS::BaseGraph::Vertex>(mod, "Vertex")
        .def("id", &LoopsLib::DS::BaseGraph::Vertex::id);

    //Register graph class
    using Graph = LoopsLib::DS::EmbeddedGraph;
    py::class_<Graph>(mod, "Graph")
        .def(py::init([](long long vertexCount) { return new Graph(vertexCount); })) // Vertex allocated graph
        .def(py::init<>())
        .def("read", [](Graph& g, const std::string& filePath)
    {
        LoopsIO::GraphIO::read(filePath, g);
        g.getIndex().construct(g.locations());
        g.clearIndirectData();
    })
    .def("write",[](const Graph& g, const std::string& filePath)
    {
        LoopsIO::GraphIO::write(filePath, g);
    })
        .def("edge", py::overload_cast<long long>(&Graph::edge, py::const_),
             py::return_value_policy::reference)
        .def("edges", &Graph::edges)
        .def("setLocations", [](Graph& graph, const std::vector<std::pair<LoopsLib::NT, LoopsLib::NT>>& data)
        {
            auto mkPoint = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>();
            auto transformIt = LoopsLib::Helpers::Iterators::transform_iterator(data, [mkPoint](const std::pair<LoopsLib::NT, LoopsLib::NT>& el)
            {
                return mkPoint({el.first, el.second});
            });

            graph.setLocations(std::vector<Graph::Point>(transformIt, transformIt.associatedEnd()), true);
            graph.getIndex().construct(graph.locations());
        })

        .def("byteEstimate", [](Graph& graph)
        {
            std::size_t totalBytes = 0;
            totalBytes += sizeof(Graph);
            for(auto* v : graph.vertices())
            {
                totalBytes += sizeof v;
                totalBytes += sizeof *v;
                totalBytes += sizeof(Graph::Edge*) * (v->outEdges().size() + v->inEdges().size());
            }
            for (auto* e : graph.edges())
            {
                totalBytes += sizeof e;
                totalBytes += sizeof *e;
                totalBytes += sizeof(Graph::Vertex*) * 2;
            }
            totalBytes += sizeof graph.locations() + graph.locations().size() * sizeof graph.locations()[0];
            return totalBytes;
        })
        .def("allocateVertices", &Graph::allocateVertices)
        .def("addEdge", py::overload_cast<Graph::Id_t, Graph::Id_t>(&Graph::addEdge))
        .def("vertices", &Graph::vertices)
        .def("edgeCount", &Graph::number_of_edges)
        .def("vertexCount", &Graph::number_of_vertices)
        .def("vertexLocation", [](const Graph& g, std::size_t vertexId)
        {
            return g.locations()[vertexId];
        });
}
