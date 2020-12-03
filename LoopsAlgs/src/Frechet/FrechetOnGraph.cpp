#include <LoopsAlgs/Frechet/FrechetOnGraph.h>
#include "movetk/geom/CGALTraits.h"
#include "LoopsAlgs/Frechet/CleanAltSweepline.h"
using namespace LoopsLib;
using namespace LoopsAlgs::Frechet;

template<typename T>
bool approxGreaterEqual(const T& first, const T& second, const T& thresh = 0.000000001)
{
    auto diff = std::abs(first - second);
    if (diff < thresh) return true;
    return first >= second;
}

void FrechetOnGraph::reconstructPath(const StrongFrechetGraphData& data, DS::BaseGraph::Id_t vId, 
    const std::vector<std::vector<PathPointer>>& pathPointers,
    const Hooks& hooks,
                                           std::vector<DS::BaseGraph::Id_t>& edgePath)
{
    auto logger = Helpers::logFactory(this);
    logger.info("Reconstructing path");
    // Follow the pathpointers backwards until we reach L_k.
    // We are not interested in the matching yet, so just reconstruct the path.

    assert(data.IntervalInclusions[vId].back().rightIncluded);

    // Build in reverse order
    auto currentV = vId;
    auto currentInter = data.WhiteIntervals[vId].size()-1;
    if(hooks.fixedEdge >= 0)
    {
        edgePath = { hooks.fixedEdge };
        return;
    }
    logger.trace("Endpoint: potential start? ", data.isPotentialStartPoint(vId), ", and end? (should be) ", data.isPotentialEndpoint(vId,data.WhiteIntervals[vId].size()-1));
    
    while (true)
    {
        logger.trace("\t Current vertex: ", currentV);
        auto next = pathPointers[currentV][currentInter];
        edgePath.push_back(next.edge);
        logger.trace("\t\t Next edge: ", next.edge);
        // Find next interval
        currentV = data.m_view.indexForVertex(data.m_graph->edge(next.edge)->m_source);
        auto nextInterIndex = data.intervalForVertexLocation(currentV,next.srcInterval.min);
        currentInter = nextInterIndex;
        
        // Check if leftmost point is white: we are done.
        if (data.WhiteIntervals[currentV][currentInter].min < 1e-6)
        {
            logger.trace("\t End vertex: ", currentV);
            break;
        }
    }
    // Orient the path in the proper direction
    std::reverse(edgePath.begin(), edgePath.end());
}

WenkSweeplineResult FrechetOnGraph::Hooks::beforeEdgeProcess(LoopsLib::DS::BaseGraph::Edge* edge,
                                                             const LoopsAlgs::Frechet::PrioQueueNode& node,
                                                             const Interval& lrInterval,
                                                             const StrongFrechetGraphData& data,
                                                             const std::vector<std::vector<PathPointer>>& pathPointers)
{
    auto targetV = data.m_view.indexForVertex(edge->m_sink);
    // Should not happen, since no lrpointer would exist...
    if (data.WhiteIntervals[targetV].empty())
    {
        return WenkSweeplineResult::ContinueIt;
    }
    // Check if we found a potential endpoint
    if (data.isPotentialEndpoint(targetV, data.WhiteIntervals[targetV].size() - 1) && lrInterval.intersectsWith(
        data.WhiteIntervals[targetV].back()))
    {
        if(data.isPotentialStartPoint(data.m_view.indexForVertex(edge->m_source)) && node.inter.min < 1e-6)
        {
            endVertex = targetV;
            fixedEdge = edge->id();
            return WenkSweeplineResult::Done;
        }
        // Check if it is also start, then we may get a degenerate case
        //else if(data.isPotentialStartPoint(targetV))
        //{
        //    for(auto* inEdge : edge->m_sink->m_inEdges)
        //    {
        //        if (!data.m_view.isAvailable(inEdge->m_source->id())) continue;
        //        auto vToCheck = data.m_view.indexForVertex(inEdge->m_source);

        //        if(data.isPotentialStartPoint(vToCheck))
        //        {
        //            // Preset the edge to use for a single edge path.
        //            fixedEdge = inEdge->id();
        //            endVertex = targetV;
        //            return WenkSweeplineResult::Done;
        //        }
        //    }
        //    // No inedge found to make a single edge route. Try the reverse.
        //    for (auto* outEdge : edge->m_sink->m_outEdges)
        //    {
        //        if (!data.m_view.isAvailable(outEdge->m_sink->id())) continue;
        //        auto vToCheck = data.m_view.indexForVertex(outEdge->m_sink);

        //        if (data.isPotentialStartPoint(data.m_view.indexForVertex(outEdge->m_source)))
        //        {
        //            // Preset the edge to use for a single edge path.
        //            fixedEdge = outEdge->id();
        //            endVertex = targetV;
        //            return WenkSweeplineResult::Done;
        //        }
        //    }
        //    // We are going to ignore this endpoint
        //    return WenkSweeplineResult::ContinueIt;
        //}
        else
        {
            endVertex = targetV;
        }

        return WenkSweeplineResult::DoneAfterUpdates;
    }
    return WenkSweeplineResult::ContinueIt;
}

FrechetOnGraph::FrechetOnGraph(Graph* graph): m_graph(graph)
{
}

void FrechetOnGraph::setTargetGraph(Graph* graph)
{
    m_graph = graph;
}

void FrechetOnGraph::compute(const std::vector<DS::BaseGraph::Id_t>& path, 
                                   LoopsLib::NT epsilon, std::vector<DS::BaseGraph::Id_t>& outputPath)
{
    Trajectory trajectory;
    LoopsLib::DS::edgePathToLocations(*m_graph, path, trajectory);
    compute(trajectory, epsilon, outputPath);
}

void FrechetOnGraph::compute(const Trajectory& trajectory, LoopsLib::NT epsilon, std::vector<Graph::Id_t>& outputPath)
{
    auto logger = Helpers::logFactory(this);
    logger.info("Path size: ", trajectory.size());

    m_epsilon = epsilon;
    StrongFrechetGraphData data;
    data.setup(m_graph, trajectory);

    FrechetGraphComputations computer(epsilon);

    computer.computeFDi(data);
    computer.computeLeftRightPointersBF(data);

    CleanAltSweepline<Hooks, FrechetOnGraph> sweepline(data);

    // Hook into the sweepline
    Hooks hooks;

    decltype(sweepline)::AltHeap prioQueue;
    std::vector<Interval> Cis;

    std::vector<std::vector<PathPointer>> pathPointers;

    sweepline.initializeDefaultStart(prioQueue, Cis, pathPointers);

    sweepline.apply(prioQueue, Cis, hooks, pathPointers);
    // Succesfully found a path
    if (hooks.endVertex >= 0)
    {
        reconstructPath(data, hooks.endVertex, pathPointers,hooks, outputPath);
    }
    else
    {
        logger.info("No path found");
    }
}
