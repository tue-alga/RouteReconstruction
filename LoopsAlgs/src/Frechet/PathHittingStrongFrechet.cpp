#include <LoopsAlgs/Frechet/PathHittingStrongFrechet.h>
#include <movetk/geom/CGALTraits.h>
#include <LoopsLib/DS/Heap.h>
#include <LoopsLib/Helpers/Logger.h>
#include <LoopsAlgs/Frechet/CleanAltSweepline.h>
using namespace LoopsLib;
using Interval = Geometry::Interval;
using namespace LoopsAlgs::Frechet;

template<typename T>
bool approxGreaterEqual(const T& first, const T& second, const T& thresh = 0.000000001)
{
    auto diff = std::abs(first - second);
    if (diff < thresh) return true;
    return first >= second;
}

LoopsAlgs::Frechet::WenkSweeplineResult PathHittingStrongFrechet::PrefixHooks::beforeEdgeProcess(
    DS::BaseGraph::Edge* e, const PrioQueueNode& node, const Geometry::Interval& lrPointer, const StrongFrechetGraphData& data)
{
    auto logger = LoopsLib::Helpers::LogFactory<PathHittingStrongFrechet>{};

    if(node.inter.min > targetEdgeInterval.max || node.inter.min > lowestIntervalPoint)
    {
        return WenkSweeplineResult::Done;
    }

    // Index of the target vertex
    auto vId = data.m_view.indexForVertex(e->m_sink);


    if (lrPointer.min > targetEdgeInterval.max || lrPointer.min > lowestIntervalPoint)
    {
        logger.trace("\tPast target: ", targetEdgeInterval.max, ',', lowestIntervalPoint, " have: ",
            lrPointer.min);
        return WenkSweeplineResult::SkipRest;
    }

    // Check if we are at the target start vertex of the edge at the appropriate interval height
    if (vId == targetEdgeStartVert && targetEdgeInterval.intersectsWith(lrPointer))
    {
        Interval intersection = targetEdgeInterval.intersection(lrPointer);
        // Update the minimum that we can reach.
        if (intersection.min < lowestIntervalPoint)
        {
            lowestIntervalPoint = intersection.min;
            // TODO fix this?
            //data.pathPointers[vId][data.CellToIntervalId[vId][(int)targetEdgeInterval.min]] = Data::PathPointer{
                //node.vertex, node.inter,false };
            intervalFound = true;
            logger.trace("\t#### Updated end low to ", lowestIntervalPoint);
        }
        else
        {
            logger.trace("End was lower");
        }
        // Don't add interval belonging to the edge.
        return WenkSweeplineResult::ContinueIt;
    }
    return WenkSweeplineResult::ContinueIt;
}

LoopsAlgs::Frechet::WenkSweeplineResult PathHittingStrongFrechet::PostfixHooks::beforeEdgeProcess(
    DS::BaseGraph::Edge* e, const PrioQueueNode& node, const Geometry::Interval& lrPointer, const StrongFrechetGraphData& data)
{
    auto logger = LoopsLib::Helpers::LogFactory<PathHittingStrongFrechet>{};
    auto localV = data.m_view.indexForVertex(e->m_sink);

    if(node.vertex == endVert && node.interIndex == endIntervalOut)
    {
        return WenkSweeplineResult::Done;
    }

    //Check rpointer is at end of free surface, then we are done.
    if ((long long)lrPointer.max >= data.polyline.size() - 1 && data.isPotentialEndpoint(localV,data.WhiteIntervals[localV].size()-1)) //Right is included
        // Flag that rightmost point of FD_j is included.
    {
        logger.trace("\t\tFound endpoint: ", localV);
        endIntervalOut = data.WhiteIntervals[localV].size() - 1;
        endVert = localV;
        // Make sure to update the path pointer as well.
        //assert(!data.pathPointers[localV].empty());
        assert(!node.inter.isEmpty());
        logger.trace("\t\t Pp: ", node.vertex, "->", localV, "[end=", endIntervalOut,"]");

        // Flag that we want the path pointers to be updated, but are done after that 
        return WenkSweeplineResult::ContinueIt;
    }
    // Just continue
    return WenkSweeplineResult::ContinueIt;
}

bool PathHittingStrongFrechet::searchPrefixPath(const StrongFrechetGraphData& data, DS::BaseGraph::Id_t targetEdgeId, const Interval& targetEdgeInterval,
    NT& lowestInterval, int& lastInterval, std::vector<std::vector<PathPointer>>& prefixPathPointers)
{
    auto logger = LoopsLib::Helpers::logFactory(this);

    // Target edge data
    auto* targetEdge = m_graph->edge(targetEdgeId);
    const auto targetEdgeStartVert = data.m_view.indexForVertex(targetEdge->m_source);

    // Initialize output
    lowestInterval = std::numeric_limits<NT>::max();

    logger.trace("Interval to search: ", targetEdgeInterval.min, ",", targetEdgeInterval.max);
    logger.trace("At vert ", targetEdgeStartVert);

    PrefixHooks hooks;
    hooks.targetEdgeInterval = targetEdgeInterval;
    hooks.lowestIntervalPoint = std::numeric_limits<NT>::max();
    hooks.targetEdgeStartVert = targetEdgeStartVert;

    CleanAltSweepline<PrefixHooks, PathHittingStrongFrechet> sweepline(data);

    //The priority queue for the sweepline algorithm.
    DS::Heap<PrioQueueNode> prioQueue;
    std::vector<Interval> Cis;

    // Initialize with default start (all available left starting points)
    sweepline.initializeDefaultStart(prioQueue, Cis, prefixPathPointers);

    // Apply sweepline
    sweepline.apply(prioQueue, Cis, hooks, prefixPathPointers);

    if (hooks.intervalFound)
    {
        lowestInterval = hooks.lowestIntervalPoint;
        lastInterval = data.intervalForVertexLocation(targetEdgeStartVert,targetEdgeInterval.min);
        logger.trace("\tFound lowest ", lowestInterval, " for interval ", lastInterval);
    }
    return hooks.intervalFound;
}

bool PathHittingStrongFrechet::searchPostfixPath(const StrongFrechetGraphData& data, DS::BaseGraph::Id_t targetEdgeId, const Interval& sourceInterval, int& endIntervalOut, int& endVert,
    std::vector<std::vector<PathPointer>>& pathPointers)
{
    auto logger = LoopsLib::Helpers::logFactory(this);

    DS::Heap<PrioQueueNode> prioQueue;
    std::vector<Interval> Cis;
    Cis.resize(data.m_view.number_of_vertices(), Interval{}); //All are empty

    // Initialize with white starting points
    auto* targetEdge = m_graph->edge(targetEdgeId);
    const auto initialV = data.m_view.indexForVertex(targetEdge->m_sink);
    
    { //Initialize the first interval for the target edge.
        Interval initialInterval(sourceInterval.min, sourceInterval.max);
        prioQueue.insert(PrioQueueNode(initialInterval, initialV, data.intervalForVertexLocation(initialV, sourceInterval.min)), initialV);
        // Initialize Cis.
        // TODO: maybe intialize this to the initial interval.
        Cis[initialV] = sourceInterval;
    }

    logger.trace("Initial queue size: ", prioQueue.size());

    PostfixHooks hooks;
    CleanAltSweepline<PostfixHooks, PathHittingStrongFrechet> sweepline(data);
    // Preallocate path pointers
    sweepline.allocatePathPointers(pathPointers);

    sweepline.apply(prioQueue, Cis, hooks, pathPointers);
    if (hooks.endVert >= 0)
    {
        endIntervalOut = hooks.endIntervalOut;
        endVert = hooks.endVert;
        return true;
    }
    return false;
}

void PathHittingStrongFrechet::buildPrefixPath(const StrongFrechetGraphData& data, DS::BaseGraph::Id_t targetEdgeId, DS::BaseGraph::Id_t vId, int endInterval,
    const std::vector<std::vector<PathPointer>>& pathPointers,
    std::vector<DS::BaseGraph::Id_t>& edgePath)
{
    auto logger = LoopsLib::Helpers::logFactory(this);

    logger.debug("=== Building prefix path");
    auto* edge = m_graph->edge(targetEdgeId);
    //path.push_back(edge->m_source->id());
    // Build in reverse order
    int currentV = vId;
    int currentInter = endInterval;

    std::set<std::pair<int, int>> usedCombos;
    usedCombos.insert(std::make_pair(currentV, currentInter));

    while (true)
    {
        auto next = pathPointers[currentV][currentInter];
        logger.trace("\t Current vertex: ", currentV, " and inter ", currentInter, ", starting at ", data.WhiteIntervals[currentV][currentInter].min, ", moving via edge ", next.edge);
        
        edgePath.push_back(next.edge);
        // Find next interval
        currentV = data.m_view.indexForVertex(data.m_graph->edge(next.edge)->m_source);
        auto nextInterIndex = data.intervalForVertexLocation(currentV, next.srcInterval.min);
        currentInter = nextInterIndex;

        if (usedCombos.find(std::make_pair(currentV, currentInter)) != usedCombos.end())
        {
            logger.error("Detected cycle in prefix building");
            edgePath = {};
            return;
        }

        // Mark used
        usedCombos.insert(std::make_pair(currentV, currentInter));

        // Check if leftmost point is white: we are done.
        if (currentInter == 0 && data.IntervalInclusions[currentV][0].leftIncluded)
        {
            logger.trace("\t End vertex: ", currentV);
            break;
        }
    }
    // Orient the path in the proper direction
    std::reverse(edgePath.begin(), edgePath.end());
}

void PathHittingStrongFrechet::buildPostfixPath(const StrongFrechetGraphData& data, DS::BaseGraph::Id_t vId, int endInterval,
    const Interval& edgeInterval,
    DS::BaseGraph::Id_t targetVertexId,
    std::vector<std::vector<PathPointer>>& pathPointers,
    std::vector<DS::BaseGraph::Id_t>& edgePath)
{
    auto logger = LoopsLib::Helpers::logFactory(this);
    logger.debug("=== Building postfix path");

    std::set<std::pair<int, int>> usedCombos;

    // Build in reverse order
    int currentV = vId;
    int currentInter = endInterval;
    usedCombos.insert(std::make_pair(currentV, currentInter));
    while (true)
    {
        auto next = pathPointers[currentV][currentInter];
        logger.trace("\t Current vertex: ", currentV, " and inter ", currentInter, ", moving via edge ",next.edge);
        
        // Push the vertex to the path.
        edgePath.push_back(next.edge);

        currentV = data.m_view.indexForVertex(data.m_graph->edge(next.edge)->m_source);
        auto nextInterIndex = data.intervalForVertexLocation(currentV, next.srcInterval.min);
        currentInter = nextInterIndex;

        if(usedCombos.find(std::make_pair(currentV, currentInter)) != usedCombos.end())
        {
            logger.error("Detected cycle in postfix building");
            edgePath = {};
            return;
        }

        // Mark used
        usedCombos.insert(std::make_pair(currentV, currentInter));

        if (next.srcInterval.intersectsWith(edgeInterval) && currentV == targetVertexId)
        {
            logger.trace("\t Reached end: ", currentV);
            break;
        }
        // We should not go past the queried interval
        assert(next.srcInterval.max >= edgeInterval.min);
    }
    assert(currentV == targetVertexId);
    // Orient the path in the proper direction
    std::reverse(edgePath.begin(), edgePath.end());
}

bool PathHittingStrongFrechet::searchPath(const StrongFrechetGraphData& data, DS::BaseGraph::Id_t targetEdgeId, std::vector<DS::BaseGraph::Id_t>& path)
{
    auto logger = LoopsLib::Helpers::logFactory(this);
    logger.debug("Searching path for edge", targetEdgeId);

    auto* targetEdge = m_graph->edge(targetEdgeId);

    // IDs of src/sink vertices of target in the Frecehet view.
    const auto srcVertId = data.m_view.indexForVertex(targetEdge->m_source);

    const auto sinkVertId = data.m_view.indexForVertex(targetEdge->m_sink);

    logger.trace("-- Number of intervals: ", data.IntervalIdToCell[srcVertId].size());

    const auto& intervalStarts = data.IntervalIdToCell[srcVertId];

    // Search for intervals through which we may create a monotonous matching path in the freespace surface.
    for (int i = 0; i < intervalStarts.size(); ++i)
    {
        // Start cell of the interval
        const auto startCell = intervalStarts[i];
        auto currCell = startCell;

        // Check if the interval has a non-empty l/rpointer interval: existence in the pointer map should be enough.
        if (data.lrPointers[srcVertId][currCell].find(sinkVertId) == data.lrPointers[srcVertId][currCell].end())
        {
            continue;
        }
        //assert(data.leftPointers[srcVertId][currCell][sinkVertId].first >= currCell);
        
        Interval whiteInterval = data.WhiteIntervals[srcVertId][i];

        // Output paths
        std::vector<DS::BaseGraph::Id_t> prefixPath, postfixPath;

        //
        // Compute the prefix
        //
        NT lowestValue = 0; //Will be set in the below function
        int lastInterval = -1;
        logger.trace("=== Finding prefix path ");


        // Search for a prefix path if the source vertex of the edge is not actually already a feasible starting point
        if ((int)whiteInterval.min != 0 || !data.IntervalInclusions[srcVertId][0].leftIncluded)
        {
            std::vector<std::vector<PathPointer>> prefixPathPointers;
            if (!searchPrefixPath(data, targetEdgeId, whiteInterval, lowestValue, lastInterval, prefixPathPointers)) {

                logger.debug("== No prefix found");
                continue;
            }
            // Construct the path
            buildPrefixPath(data, targetEdgeId, srcVertId, lastInterval, prefixPathPointers, prefixPath);
            // Only happens on errors
            if (prefixPath.empty()) continue;
        }

        // Add connecting edge
        prefixPath.push_back(targetEdge->id());

        logger.debug("== Prefix found");

        // Get out interval
        Interval outInterval = data.lrPointers[srcVertId][startCell].at(sinkVertId).pointers;
        outInterval.min = std::max(outInterval.min, lowestValue);

        if (outInterval.isEmpty())
        {
            logger.debug("=== Prefix resulted in empty interval continuing");
            continue;
        }

        //
        // Find the postfix path, starting at the end of the target edge
        //
        logger.debug("=== Finding postfix path with interval: ", outInterval.min, ",", outInterval.max);

        if(!(data.isPotentialEndpoint(sinkVertId,data.WhiteIntervals[sinkVertId].size()-1) && data.WhiteIntervals[sinkVertId].back().intersectsWith(outInterval)))
        {

            int endInterval = -1, endVert = -1;
            std::vector<std::vector<PathPointer>> postfixPathPointers;
            if (!searchPostfixPath(data, targetEdgeId, outInterval, endInterval, endVert, postfixPathPointers))
            {
                logger.debug("== No postfix found");
                continue;
            }
            logger.debug("== Postfix found");
            // Reconstruct the path.
            // Prefix part

            buildPostfixPath(data, endVert, endInterval, outInterval, sinkVertId, postfixPathPointers, postfixPath);

            // Only happens on errors
            if (postfixPath.empty()) continue;
        }
        else
        {
            logger.debug("== Postfix found: was sink vertex");
        }

        // Attach prefix to postfix
        path.insert(path.begin(), prefixPath.begin(), prefixPath.end());
        path.insert(path.end(), postfixPath.begin(), postfixPath.end());

        return true;
    }

    // No path was found
    return false;
}

    PathHittingStrongFrechet::PathHittingStrongFrechet(const DS::EmbeddedGraph* graph) : m_graph(graph)
{
}

PathHittingStrongFrechet::PathHittingStrongFrechet(): m_graph(nullptr), m_epsilon(0)
{
}

void PathHittingStrongFrechet::setTargetGraph(DS::EmbeddedGraph* graph)
{
    m_graph = graph;
}

void PathHittingStrongFrechet::compute(const std::vector<DS::BaseGraph::Id_t>& path, LoopsLib::NT epsilon, DS::BaseGraph::Id_t targetEdge, std::vector<DS::BaseGraph::Id_t>& outputPath)
{

    auto logger = LoopsLib::Helpers::logFactory(this);

    logger.trace("Path size: ", path.size());

    m_epsilon = epsilon;
    // The 
    StrongFrechetGraphData data;
    precompute(path, epsilon, data);
    data.setup(m_graph, path);

    // Compute data
    FrechetGraphComputations computer(epsilon);
    computer.computeFDi(data);
    computer.computeLeftRightPointersBF(data);

    // Succesfully found a path
    if (searchPath(data, targetEdge, outputPath))
    {
        logger.info("Path found of size ", outputPath.size());
    }
    else
    {
        logger.info("No path found");
    }
}

void PathHittingStrongFrechet::precompute(const std::vector<DS::BaseGraph::Id_t>& path, 
    LoopsLib::NT epsilon, StrongFrechetGraphData& data)
{
    auto logger = LoopsLib::Helpers::logFactory(this);

    logger.trace("Path size: ", path.size());
    m_epsilon = epsilon;

    data.setup(m_graph, path);
    FrechetGraphComputations computer(m_epsilon);

    computer.computeFDi(data);
    computer.computeLeftRightPointersBF(data);
}

void PathHittingStrongFrechet::precompute(const Trajectory& trajectory, LoopsLib::NT epsilon,
    StrongFrechetGraphData& data)
{
    auto logger = LoopsLib::Helpers::logFactory(this);

    logger.trace("Path size: ", trajectory.size());
    m_epsilon = epsilon;

    data.setup(m_graph, trajectory);
    FrechetGraphComputations computer(m_epsilon);

    computer.computeFDi(data);
    computer.computeLeftRightPointersBF(data);
}

void PathHittingStrongFrechet::computeNextUsingPathPointers(const StrongFrechetGraphData& data,
    LoopsLib::DS::BaseGraph::Id_t targetEdgeId, const std::vector<std::vector<PathPointer>>& initialPathPointers,
    std::vector<LoopsLib::DS::BaseGraph::Id_t>& outputPath)
{
    auto logger = LoopsLib::Helpers::logFactory(this);
    logger.debug("Searching path for edge", targetEdgeId);

    auto* targetEdge = m_graph->edge(targetEdgeId);

    // IDs of src/sink vertices of target in the Frecehet view.
    const auto srcVertId = data.m_view.indexForVertex(targetEdge->m_source);

    const auto sinkVertId = data.m_view.indexForVertex(targetEdge->m_sink);

    logger.trace("-- Number of intervals: ", data.WhiteIntervals[srcVertId].size());

    const auto numWhiteIntervals = data.WhiteIntervals[srcVertId].size();

    // Quit when nothing to be found
    if (data.WhiteIntervals[srcVertId].empty()) return;
    if (data.WhiteIntervals[sinkVertId].empty()) return;


    const auto& intervalStarts = data.IntervalIdToCell[srcVertId];

    // Search for intervals through which we may create a monotonous matching path in the freespace surface.
    for (int i = 0; i < numWhiteIntervals; ++i)
    {
        // Start cell of the interval
        const auto startCell = intervalStarts[i];
        auto currCell = startCell;

        // Check if the interval has a non-empty l/rpointer interval: existence in the pointer map should be enough.
        if (data.lrPointers[srcVertId][currCell].find(sinkVertId) == data.lrPointers[srcVertId][currCell].end())
        {
            continue;
        }
        //assert(data.leftPointers[srcVertId][currCell][sinkVertId].first >= currCell);

        Interval whiteInterval = data.WhiteIntervals[srcVertId][i];

        // Output paths
        std::vector<DS::BaseGraph::Id_t> prefixPath, postfixPath;

        //
        // Compute the prefix
        //
        logger.trace("=== Finding prefix path ");


        // Search for a prefix path if the source vertex of the edge is not actually already a feasible starting point
        if (i != 0 || !data.isPotentialStartPoint(srcVertId))
        {
            // No path pointer set and interval is not start interval: continue.
            if (!initialPathPointers[srcVertId][i].isSet()) continue;

            // Construct the path
            buildPrefixPath(data, targetEdgeId, srcVertId, i, initialPathPointers, prefixPath);
        }

        // Add connecting edge
        prefixPath.push_back(targetEdge->id());

        logger.debug("== Prefix found");

        // Get out interval
        Interval outInterval = data.lrPointers[srcVertId][startCell].at(sinkVertId).pointers;
        outInterval.min = std::max(outInterval.min, initialPathPointers[srcVertId][i].lowestPointAtSelf);

        if (outInterval.isEmpty())
        {
            logger.debug("=== Prefix resulted in empty interval continuing");
            continue;
        }

        //
        // Find the postfix path, starting at the end of the target edge
        //
        logger.debug("=== Finding postfix path with interval: ", outInterval.min, ",", outInterval.max);

        if (!data.isPotentialEndpoint(sinkVertId, data.WhiteIntervals[sinkVertId].size() - 1) || !data.WhiteIntervals[sinkVertId].back().intersectsWith(outInterval))
        {

            int endInterval = -1, endVert = -1;
            std::vector<std::vector<PathPointer>> postfixPathPointers;
            if (!searchPostfixPath(data, targetEdgeId, outInterval, endInterval, endVert, postfixPathPointers))
            {
                logger.debug("== No postfix found");
                continue;
            }
            logger.debug("== Postfix found");
            // Reconstruct the path.
            // Prefix part

            buildPostfixPath(data, endVert, endInterval, outInterval, sinkVertId, postfixPathPointers, postfixPath);
        }
        else
        {
            logger.debug("== Postfix found: was sink vertex");
        }


        outputPath.insert(outputPath.begin(), prefixPath.begin(), prefixPath.end());
        outputPath.insert(outputPath.end(), postfixPath.begin(), postfixPath.end());
        return;
    }
}

void PathHittingStrongFrechet::precomputePathPointers(const StrongFrechetGraphData& data,
                                                      std::vector<std::vector<PathPointer>>& pathPointers)
{

    AltBasicHooks hooks;

    CleanAltSweepline<AltBasicHooks, PathHittingStrongFrechet> sweepline(data);

    //The priority queue for the sweepline algorithm.
    DS::Heap<PrioQueueNode> prioQueue;
    std::vector<Interval> Cis;

    // Initialize with default start (all available left starting points)
    sweepline.initializeDefaultStart(prioQueue, Cis, pathPointers);

    // Apply sweepline
    sweepline.apply(prioQueue, Cis, hooks, pathPointers);
}

void PathHittingStrongFrechet::computeNext(const StrongFrechetGraphData& data, DS::BaseGraph::Id_t targetEdge,
    std::vector<DS::BaseGraph::Id_t>& outputPath)
{
    auto logger = LoopsLib::Helpers::logFactory(this);
    auto* edge = m_graph->edge(targetEdge);
    logger.debug("====================");
    logger.debug("Starting search for edge ", targetEdge, " = [", *edge->m_source, ",", *edge->m_sink, "]");
    logger.debug("====================");
    try
    {
        // Succesfully found a path
        if (searchPath(data, targetEdge, outputPath))
        {
            logger.info("Path found");
        }
        else
        {
            logger.info("Path not found");
        }
    }
    catch(std::exception& e)
    {
        logger.error("Caught exception:'", e.what(),"' for target edge ", targetEdge);
    }
}
