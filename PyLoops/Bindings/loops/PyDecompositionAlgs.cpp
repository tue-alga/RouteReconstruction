#include "PyDecompositionAlgs.h"
#include "LoopsAlgs/Frechet/CleanAltSweepline.h"
#include <pybind11/stl.h>
//#include <pybind11/iostream.h>
PyLoops::Bindings::DecompositionException::DecompositionException(const std::string& msg): m_msg(msg)
{
}

char const* PyLoops::Bindings::DecompositionException::what() const noexcept
{
    return m_msg.c_str();
}

void PyLoops::Bindings::PyDecompositionAlgs::Tester::testReachability(
    const std::vector<LoopsLib::DS::BaseGraph::Id_t > & path, LoopsLib::NT epsilon)
{

    LoopsAlgs::Frechet::StrongFrechetGraphData data;
    data.setup(m_graph, path);

    LoopsAlgs::Frechet::FrechetGraphComputations precomputer(epsilon);

    precomputer.computeFDi(data);
    precomputer.computeLeftRightPointersBF(data);

    // Verify that all vertices of the path are in the view
    for (const auto& eId : path)
    {
        auto* e = data.m_graph->edge(eId);
        auto v0Ind = data.m_view.indexForVertex(e->m_source->id());
        auto v1Ind = data.m_view.indexForVertex(e->m_sink->id());
        if (data.m_view.availableVertices().find(*e->m_source) == data.m_view.availableVertices().end())
        {
            throw std::runtime_error("Could not find vertex of path in graph view");
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(2s);
        }
        if (data.m_view.availableVertices().find(*e->m_sink) == data.m_view.availableVertices().end())
        {
            throw std::runtime_error("Could not find vertex of path in graph view");
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(2s);
        }
        if (data.WhiteIntervals[v0Ind].empty())
        {
            throw std::runtime_error("Source vertex in path has no white interval");
        }
        if (data.WhiteIntervals[v1Ind].empty())
        {
            throw std::runtime_error("Source vertex in path has no white interval");
        }
    }
    {
        auto* src = data.m_graph->edge(*path.begin())->m_source;
        auto srcInd = data.m_view.indexForVertex(src->id());
        if (data.WhiteIntervals[srcInd].begin()->min != 0)
        {
            throw std::runtime_error("Path begin not potential start");
        }
        auto* sink = data.m_graph->edge(*path.rbegin())->m_sink;
        auto sinkInd = data.m_view.indexForVertex(sink->id());
        if (std::abs(data.WhiteIntervals[sinkInd].rbegin()->max - (LoopsLib::NT)path.size()) > 0.0001)
        {
            std::stringstream ss;
            ss << "Path end not potential end:" << data.WhiteIntervals[sinkInd].rbegin()->max << " vs size " << (LoopsLib::NT)path
                .size();
            ss << ", last cell:" << data.FDiWhiteIntervals[sinkInd].back().min << ',' << data.FDiWhiteIntervals[sinkInd]
                                                                                         .back().max;
            throw std::runtime_error(ss.str());
        }
    }
    // Setup the sweepline algorithm
    LoopsAlgs::Frechet::CleanAltSweepline<Tester> sweepLine(data);

    if (!sweepLine.testSinglePathNoPrinting(path))
    {
        sweepLine.testSinglePath(path);
        throw std::runtime_error("Self path is not consistent");
    }
}

LoopsLib::Models::DecompositionResult PyLoops::Bindings::PyDecompositionAlgs::runAlg(
    LoopsAlgs::FlowDecomposition::IFlowDecomposition* alg)
{

    verifyDecomposable();

    LoopsLib::Models::DecompositionResult res;
    res.m_relatedInstance = m_obj;
    alg->setDecompositionObject(&res);
    alg->decompose();
    return res;
}

PyLoops::Bindings::PyDecompositionAlgs::PyDecompositionAlgs(LoopsLib::Models::ProblemInstance* obj):
    m_obj(obj),
    m_frechetHitting(nullptr),
    m_hitting(nullptr),
    m_weightedFrechet(nullptr)
{
}

void PyLoops::Bindings::PyDecompositionAlgs::verifyDecomposable()
{
    if (m_obj->m_field->pathCount() == 0) throw DecompositionException("No field set");
    if (m_obj->m_availablePaths.empty()) throw DecompositionException("No available paths set");
}


void PyLoops::Bindings::PyDecompositionAlgs::registerPy(pybind11::module& mod)
{
    namespace py = pybind11;
    // Register custom exception class.
    py::register_exception<DecompositionException>(mod, "DecompositionException");

    using WFD = LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition;
    using FHD = LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition;
    bindDecompositionAlgorithm(detail::TypeHolder<LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition>{}, mod, "FrechetHittingPathsDecomposition",
        detail::Property<FHD, int>(std::string("pathsPerTrajectory"), &FHD::pathsPerTrajectory, &FHD::setPathsPerTrajectory),
        detail::Property<FHD, int>(std::string("numberOfThreads"), &FHD::numberOfThreads, &FHD::setNumberOfThreads)
    );
    bindDecompositionAlgorithm(detail::TypeHolder<LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition>{}, mod, "WeightedFrechetDecomposition",
        detail::Property<WFD,bool>(std::string("noDataCaching"), &WFD::noDataCaching, &WFD::setNoDataCaching),
        detail::Property<WFD, double>(std::string("maxSearchTime"), &WFD::maxSearchTime, &WFD::setMaxSearchTime),
        detail::Property<WFD, int>(std::string("maxNumPerPath"), &WFD::maximumPerPath, &WFD::setMaximumPerPath),
        detail::Property<WFD, int>(std::string("numberOfThreads"), &WFD::numberOfThreads, &WFD::setNumberOfThreads)
    );
    bindDecompositionAlgorithm(detail::TypeHolder<LoopsAlgs::FlowDecomposition::TrivialDecomposition>{}, mod, "TrivialDecomposition");

    py::class_<Tester>(mod, "FrechetPathTester")
        .def(py::init<LoopsLib::DS::EmbeddedGraph*>())
        .def("testPath", &Tester::testReachability);
}
