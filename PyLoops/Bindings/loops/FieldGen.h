#ifndef PYLOOPS_LOOPS_PYFIELDGENERATOR_H
#define PYLOOPS_LOOPS_PYFIELDGENERATOR_H
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "DS/EmbeddedGraph.h"
namespace PyLoops::loops
{
    class PyFieldGenerator
    {
        LoopsLib::DS::EmbeddedGraph* m_graph;
        std::string m_graphFile;
        LoopsLib::NT m_noiseMaximum = 100;
    public:
        PyFieldGenerator(LoopsLib::DS::EmbeddedGraph* graph, const std::string& graphFile);

        void generateRandomUniform(std::size_t numberOfPaths, const std::string& outputFile);

        static void registerPy(pybind11::module& mod);
    };
}
#endif