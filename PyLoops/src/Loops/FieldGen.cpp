#include <PyLoops/Loops/FieldGen.h>
#include <LoopsLib/Helpers/RandomHelpers.h>
#include <LoopsLib/Models/FlowField.h>
#include <LoopsLib/GraphAlgs/ShortestPath.h>
#include <LoopsIO/FieldIO.h>
#include <pybind11/iostream.h>

PyLoops::loops::PyFieldGenerator::PyFieldGenerator(LoopsLib::DS::EmbeddedGraph* graph, const std::string& graphFile): m_graph(graph),
m_graphFile(graphFile)
{
}

void PyLoops::loops::PyFieldGenerator::generateRandomUniform(std::size_t numberOfPaths,
                                                                const std::string& outputFile)
{
    pybind11::scoped_ostream_redirect streamRedir(
        std::cout,                               // std::ostream&
        pybind11::module::import("sys").attr("stdout") // Python output
    );

    std::cout << "Generating " << numberOfPaths << " paths" << std::endl;
    LoopsLib::Models::FlowField field;
    field.m_graphFile = m_graphFile;
    auto randVert = [this]()
    {
        return LoopsLib::Helpers::RandomHelpers::randomLong(0, m_graph->number_of_vertices() - 1);
    };

    struct WeightProvider
    {
        std::vector<LoopsLib::NT> edgeLengths;
        std::vector<LoopsLib::NT> noise;

        WeightProvider(LoopsLib::DS::EmbeddedGraph* graph)
        {
            edgeLengths.resize(graph->numberOfEdges(),0);
            for(std::size_t i = 0; i < edgeLengths.size(); ++i)
            {
                edgeLengths[i] = graph->edgeLength(i);
            }
        }
        void generateNoise(LoopsLib::NT maximum)
        {
            noise.resize(edgeLengths.size(), 0);
            for (std::size_t i = 0; i < edgeLengths.size(); ++i)
            {
                noise[i] = LoopsLib::Helpers::RandomHelpers::randomFloatingPoint<LoopsLib::NT>(0, maximum);
            }
        }

        LoopsLib::NT operator[](LoopsLib::DS::BaseGraph::Id_t eId) const
        {
            return edgeLengths[eId] + noise[eId];
        }
    };
    WeightProvider weights(m_graph);
    for(std::size_t i = 0; i < numberOfPaths; ++i)
    {
        std::vector<LoopsLib::DS::BaseGraph::Edge*> path;
        int retries = 10;
        std::cout << "Generating " << (i+1) << "/" << numberOfPaths << std::endl;
        while(path.empty() && retries >= 0)
        {
            auto src = randVert();
            auto sink = randVert();
            std::cout << "\tSrc " << src << ", sink " << sink << std::endl;
            // Find unique other
            while (sink == src)
            {
                sink = randVert();
                std::cout << "\t\t New sink " << sink << std::endl;
            }

            LoopsLib::GraphAlgs::WeightedShortestPath<WeightProvider> shortestPath;
            weights.generateNoise(m_noiseMaximum);
            shortestPath.computeShortestPath(m_graph, weights, src, sink, path);

            --retries;
            if(path.empty()) std::cout << "\tRetrying" << std::endl;
        }
        if(!path.empty())
        {
            // Convert to edge Ids
            std::vector<LoopsLib::DS::BaseGraph::Id_t> eIdPath;
            eIdPath.reserve(path.size());
            std::transform(path.begin(), path.end(),std::back_inserter(eIdPath), [](auto* e) {return e->id(); });

            field.m_paths.push_back(eIdPath);
            field.m_pathValues.push_back(1);
        }
    }
    field.setFieldFromPaths(m_graph->numberOfEdges());
    LoopsIO::FieldIO::write(outputFile, field);
}

void PyLoops::loops::PyFieldGenerator::registerPy(pybind11::module& mod)
{
    namespace py = pybind11;
    using FG = PyFieldGenerator;
    py::class_<FG>(mod, "FieldGenerator")
        .def(py::init<LoopsLib::DS::EmbeddedGraph*,const std::string&>())
        .def_readwrite("noiseMaximum", &FG::m_noiseMaximum)
        .def("generateUniformRandomField", &FG::generateRandomUniform);
}
