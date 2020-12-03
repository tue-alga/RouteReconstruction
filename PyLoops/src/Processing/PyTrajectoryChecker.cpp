#include <PyLoops/Processing/PyTrajectoryChecker.h>
#include <pybind11/stl.h>
#include <pybind11/iostream.h>
#include <fstream>

void PyLoops::processing::PyTrajectoryChecker::computeFrechetData(
    const std::vector<LoopsLib::DS::BaseGraph::Id_t>& traj, LoopsLib::DS::EmbeddedGraph* g, LoopsLib::NT epsilon,
    LoopsAlgs::Frechet::StrongFrechetGraphData& data)
{
    data.setup(g, traj);

    LoopsAlgs::Frechet::FrechetGraphComputations precomputer(epsilon);

    precomputer.computeFDi(data);
    precomputer.computeLeftRightPointersBF(data);
}

void PyLoops::processing::PyTrajectoryChecker::getPotentialEndpoints(
    const LoopsAlgs::Frechet::StrongFrechetGraphData& data, std::set<LoopsLib::DS::BaseGraph::Id_t>& potentialEndpoints)
{
    potentialEndpoints.clear();
    // Add all starting intervals as zero weight weight intervals
    for (std::size_t i = 0; i < data.number_of_vertices(); ++i)
    {
        if (data.WhiteIntervals[i].empty()) continue;

        // Collect potential endpoints
        if (data.isPotentialEndpoint(i, data.WhiteIntervals[i].size() - 1))
        {
            potentialEndpoints.insert(i);
        }
    }
}

LoopsAlgs::Frechet::WenkSweeplineResult PyLoops::processing::PyTrajectoryChecker::EndpointChecker::beforeEdgeProcess(
    LoopsLib::DS::BaseGraph::Edge* edge, const LoopsAlgs::Frechet::PrioQueueNode& node,
    const LoopsLib::Geometry::Interval& lrInterval, const LoopsAlgs::Frechet::StrongFrechetGraphData& data)
{
    if (lrInterval.max >= data.polylineEdgeCount() || (int)lrInterval.max == data.polylineEdgeCount() - 1)
    {
        auto id = data.m_view.indexForVertex(edge->m_sink->id());
        if (potEnd.find(id) != potEnd.end())
        {
            seenEnd.insert(id);
        }
    }
    ++counter;
    if(counter % 10)
    {
        if(!heap.verifyHeapProperty())
        {
            erroStream << "Heap property failed" << std::endl;
        }
    }
    return LoopsAlgs::Frechet::WenkSweeplineResult::ContinueIt;
}

bool PyLoops::processing::PyTrajectoryChecker::EndpointChecker::anySeen() const
{
    return !seenEnd.empty();
}

std::string PyLoops::processing::PyTrajectoryChecker::checkGraphData(
    const std::vector<LoopsLib::DS::BaseGraph::Id_t>& traj, LoopsLib::DS::EmbeddedGraph* g, LoopsLib::NT epsilon)
{
    LoopsAlgs::Frechet::StrongFrechetGraphData data;
    computeFrechetData(traj, g, epsilon, data);

    std::stringstream errorStr;

    const char nl = '\n';
    // Verify white pointers
    for (std::size_t i = 0; i < data.number_of_vertices(); ++i)
    {
        std::vector<bool> seenWis;
        seenWis.resize(data.WhiteIntervals[i].size(),false);

        for (std::size_t cell = 0; cell < data.FDiWhiteIntervals.at(i).size(); ++cell)
        {
            if(data.FDiWhiteIntervals[i][cell].isEmpty())
            {
                if(data.CellToIntervalId[i][cell] != -1)
                {
                    errorStr << "Vertex " << i << " cell " << cell << ": empty white interval has non-empty cell ref" << nl;
                }
            }
            else
            {
                const auto& cellInter = data.FDiWhiteIntervals[i][cell];
                if (data.CellToIntervalId[i][cell] == -1)
                {
                    errorStr << "Vertex " << i << " cell " << cell << ": non-empty white interval has empty cell ref" << nl;
                    continue;
                }
                auto inter = data.CellToIntervalId[i][cell];
                if(inter >= data.WhiteIntervals[i].size())
                {
                    errorStr << "Vertex " << i << " cell " << cell << ": inter out of bounds: " << inter << nl;
                }
                seenWis[inter] = true;
                if(!data.WhiteIntervals[i][inter].containsApprox(cell + cellInter.min, 0.0001))
                    errorStr << "Vertex " << i << " cell " << cell << ": FDI interval" << cellInter.min << ',' << cellInter.max << " not contained in associated white interval " 
                    << data.WhiteIntervals[i][inter].min << "," << data.WhiteIntervals[i][inter].max << nl;
                if (!data.WhiteIntervals[i][inter].containsApprox(cell + cellInter.max, 0.0001))
                    errorStr << "Vertex " << i << " cell " << cell << ": FDI interval" << cellInter.min << ',' << cellInter.max << " not contained in associated white interval "
                    << data.WhiteIntervals[i][inter].min << "," << data.WhiteIntervals[i][inter].max << nl;
            }
        }
    }

    // Verify that the lrpointers point to sensible locations
    for(std::size_t i = 0; i < data.number_of_vertices(); ++i)
    {
        for(std::size_t cell = 0; cell < data.lrPointers.at(i).size(); ++cell)
        {
            for(const auto& pair: data.lrPointers[i][cell])
            {
                auto sinkV = pair.first;

            }
        }
    }
    return errorStr.str();
}

std::string PyLoops::processing::PyTrajectoryChecker::checkEndspointsSeen(
    const std::vector<LoopsLib::DS::BaseGraph::Id_t>& traj, LoopsLib::DS::EmbeddedGraph* g, LoopsLib::NT epsilon, const std::string& deepLogOutput)
{

    pybind11::scoped_ostream_redirect streamRedir(
        std::cout,                               // std::ostream&
        pybind11::module::import("sys").attr("stdout") // Python output
    );
    // Construct polyline from points
    /*for (auto it = path.begin(), it2 = path.begin() + 1; it2 != path.end(); ++it, ++it2)
    {
        Segment s = MakeSegment()(locations[*it], locations[*it2]);
        auto v0 = s[0];
        auto v1 = s[1];
        data.polyline.push_back(s);
        v0 = data.polyline.back()[0];
        v1 = data.polyline.back()[1];
        logger.info("Distance seg ", (it - path.begin()), ": ", *it, "-", *it2, ":", std::sqrt(s.get().squared_length()));
    }*/
    LoopsAlgs::Frechet::StrongFrechetGraphData data;
    computeFrechetData(traj, g, epsilon, data);

    std::set<LoopsLib::DS::BaseGraph::Id_t> potEndpoints;
    getPotentialEndpoints(data, potEndpoints);

    if (potEndpoints.empty())
    {
        return "No potential endpoints found";
    }

    // Setup the sweepline algorithm
    LoopsAlgs::Frechet::CleanAltSweepline<EndpointChecker> sweepLine(data);

    if (!sweepLine.testSinglePathNoPrinting(traj))
    {
        return "The path does not give any feasible path by itself in the reachability graph";
    }


    // Verify that all vertices of the path are in the view
    for (const auto& eId : traj)
    {
        auto* e = data.m_graph->edge(eId);
        auto v0Ind = data.m_view.indexForVertex(e->m_source->id());
        auto v1Ind = data.m_view.indexForVertex(e->m_sink->id());
        if (data.m_view.availableVertices().find(*e->m_source) == data.m_view.availableVertices().end())
        {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(2s);
        }
        if (data.m_view.availableVertices().find(*e->m_sink) == data.m_view.availableVertices().end())
        {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(2s);
        }
        if (data.WhiteIntervals[v0Ind].empty())
        {
            return "Source vertex in path has no white interval";
        }
        if (data.WhiteIntervals[v1Ind].empty())
        {
            return "Source vertex in path has no white interval";
        }
    }
    {
        auto* src = data.m_graph->edge(*traj.begin())->m_source;
        auto srcInd = data.m_view.indexForVertex(src->id());
        if (data.WhiteIntervals[srcInd].begin()->min != 0)
        {
            return "Path begin not potential start";
        }
        auto* sink = data.m_graph->edge(*traj.rbegin())->m_sink;
        auto sinkInd = data.m_view.indexForVertex(sink->id());
        if(!data.isPotentialEndpoint(sinkInd,data.WhiteIntervals[sinkInd].size()-1))
        {
            std::stringstream ss;
            ss << "Path end not potential in isPotentialEndpoint():" << data.WhiteIntervals[sinkInd].rbegin()->max;
            return ss.str();
        }
        if (std::abs(data.WhiteIntervals[sinkInd].rbegin()->max - (LoopsLib::NT)traj.size()) > 0.0001)
        {
            std::stringstream ss;
            ss << "Path end not potential end:" << data.WhiteIntervals[sinkInd].rbegin()->max << " vs size " << (
                LoopsLib::NT)traj.size();
            ss << ", last cell:" << data.FDiWhiteIntervals[sinkInd].back().min << ',' << data.FDiWhiteIntervals[sinkInd]
                                                                                         .back().max;
            return ss.str();
        }
    }
    

    // Create the basic datastructures needed for the algorithm.
    std::vector<std::vector<LoopsAlgs::Frechet::PathPointer>> pathPointers;
    LoopsLib::DS::Heap<LoopsAlgs::Frechet::PrioQueueNode> prioQueue;
    std::vector<LoopsLib::Geometry::Interval> Cis;
    EndpointChecker checker(potEndpoints, prioQueue);

    sweepLine.initializeDefaultStart(prioQueue, Cis, pathPointers);
    if(prioQueue.empty())
    {
        return "No start points found!";
    }

    sweepLine.apply(prioQueue, Cis, checker, pathPointers);

    std::string res = checker.erroStream.str();
    if (!checker.anySeen())
    {
        pathPointers = {};
        prioQueue = {};
        Cis = {};
        sweepLine.initializeDefaultStart(prioQueue, Cis, pathPointers);
        EndpointChecker checker2(potEndpoints, prioQueue);
        std::ofstream stream(deepLogOutput);
        LoopsAlgs::Frechet::CleanAltSweepline<EndpointChecker,DeepLogger> sweepLineLogger(data);
        sweepLineLogger.apply(prioQueue, Cis, checker2, pathPointers);

        return "None of the potential " + std::to_string(potEndpoints.size()) + " endpoints seen, " + res;
    }
    return res;
}

std::string PyLoops::processing::PyTrajectoryChecker::checkEndpointsRandomizedStart(
    const std::vector<LoopsLib::DS::BaseGraph::Id_t>& traj, LoopsLib::DS::EmbeddedGraph* g, LoopsLib::NT epsilon)
{
    LoopsAlgs::Frechet::StrongFrechetGraphData data;
    computeFrechetData(traj, g, epsilon, data);

    std::set<LoopsLib::DS::BaseGraph::Id_t> potEndpoints;
    getPotentialEndpoints(data, potEndpoints);

    if (potEndpoints.empty())
    {
        return "No potential endpoints found";
    }

    // Setup the sweepline algorithm
    LoopsAlgs::Frechet::CleanAltSweepline<EndpointChecker> sweepLine(data);

    // Create the basic datastructures needed for the algorithm.
    std::vector<std::vector<LoopsAlgs::Frechet::PathPointer>> pathPointers;
    LoopsLib::DS::Heap<LoopsAlgs::Frechet::PrioQueueNode> prioQueue;
    std::vector<LoopsLib::Geometry::Interval> Cis;
    EndpointChecker checker(potEndpoints, prioQueue);

    sweepLine.initializeDefaultRandomized(prioQueue, Cis, pathPointers);

    sweepLine.apply(prioQueue, Cis, checker, pathPointers);

    if (!checker.anySeen())
    {
        return "None of the potential " + std::to_string(potEndpoints.size()) + " endpoints seen with random interval order at start";
    }
    return "";
}

std::string PyLoops::processing::PyTrajectoryChecker::checkEndpointsFullIntervalStart(
    const std::vector<LoopsLib::DS::BaseGraph::Id_t>& traj, LoopsLib::DS::EmbeddedGraph* g, LoopsLib::NT epsilon)
{
    LoopsAlgs::Frechet::StrongFrechetGraphData data;
    computeFrechetData(traj, g, epsilon, data);

    std::set<LoopsLib::DS::BaseGraph::Id_t> potEndpoints;
    getPotentialEndpoints(data, potEndpoints);

    if (potEndpoints.empty())
    {
        return "No potential endpoints found";
    }

    // Setup the sweepline algorithm
    LoopsAlgs::Frechet::CleanAltSweepline<EndpointChecker> sweepLine(data);


    // Create the basic datastructures needed for the algorithm.
    std::vector<std::vector<LoopsAlgs::Frechet::PathPointer>> pathPointers;
    LoopsLib::DS::Heap<LoopsAlgs::Frechet::PrioQueueNode> prioQueue;
    std::vector<LoopsLib::Geometry::Interval> Cis;
    EndpointChecker checker(potEndpoints, prioQueue);

    sweepLine.initializeDefaultStartFullIntervals(prioQueue, Cis, pathPointers);

    sweepLine.apply(prioQueue, Cis, checker, pathPointers);

    if (!checker.anySeen())
    {
        return "None of the potential " + std::to_string(potEndpoints.size()) + " endpoints seen with full intervals as start";
    }
    return "";
}

void PyLoops::processing::PyTrajectoryChecker::registerPy(pybind11::module& mod)
{
    pybind11::class_<PyTrajectoryChecker>(mod, "TrajectoryChecker")
        .def(pybind11::init<>())
        .def("fullCheck", &PyTrajectoryChecker::checkEndspointsSeen)
        .def("checkRandomizedStart", &PyTrajectoryChecker::checkEndpointsRandomizedStart)
        .def("checkFullIntervalStart", &PyTrajectoryChecker::checkEndpointsFullIntervalStart)
        .def("checkGraphData", &PyTrajectoryChecker::checkGraphData);
}
