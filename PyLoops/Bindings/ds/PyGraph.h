#ifndef PYLOOPS_BINDINGS_DS_GRAPH_H
#define PYLOOPS_BINDINGS_DS_GRAPH_H
#include <pybind11/pybind11.h>
#include <DS/EmbeddedGraph.h>
namespace PyLoops::ds
{
    class PyGraph
    {
        struct Vertex
        {
            LoopsLib::DS::EmbeddedGraph::Vertex* m_vert;
        };
    public:
        static void registerPy(pybind11::module& mod);
    };
}
#endif