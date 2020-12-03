#include <LoopsAlgs/Frechet/WeightedStrongFrechet.h>
#include <movetk/geom/CGALTraits.h>
#include <LoopsLib/Helpers/Iterators.h>
#include <LoopsLib/DS/Heap.h>
#include <LoopsAlgs/Frechet/CleanAltSweepline.h>
#include <thread>
#include <chrono>
#include <optional>
using namespace LoopsLib;
using namespace LoopsAlgs::Frechet;

auto WeightedStrongFrechet::WeightIntervalList::begin()
{
    return m_inters.begin();
}

bool WeightedStrongFrechet::WeightInterval::operator<(const WeightInterval& other) const
{
    if (lowEndpoint != other.lowEndpoint) return lowEndpoint < other.lowEndpoint;
    if (weight != other.weight)return weight < other.weight;
    return src < other.src;
}

bool WeightedStrongFrechet::WeightIntervalList::empty() const
{
    return m_inters.empty();
}

auto WeightedStrongFrechet::WeightIntervalList::end()
{
    return m_inters.end();
}

auto WeightedStrongFrechet::WeightIntervalList::rbegin()
{
    return m_inters.rbegin();
}

auto WeightedStrongFrechet::WeightIntervalList::rend()
{
    return m_inters.rend();
}

std::size_t WeightedStrongFrechet::WeightIntervalList::size() const
{
    return m_inters.size();
}

void WeightedStrongFrechet::WeightIntervalList::fixWeightMonotonicity()
{
    // Dumb linear reconstruction
    std::set<WeightInterval> newInters;
    auto logger = LoopsLib::Helpers::LogFactory<WeightedStrongFrechet>{};
    NT currMax = std::numeric_limits<NT>::lowest();
    for (const auto& el : m_inters)
    {
        if (el.weight >= currMax)
        {
            newInters.insert(el);
            currMax = el.weight;
            logger.trace("Retaining el: ", el.weight, ", endpoint:",el.lowEndpoint, ",src:",el.src);
        }
        else
        {
            logger.trace("Deleting el: ", el.weight, ", endpoint:", el.lowEndpoint, ",src:", el.src);
        }
    }
    m_inters = std::move(newInters);
}

void WeightedStrongFrechet::WeightIntervalList::insert(const WeightInterval& wi)
{
    m_inters.insert(wi);
    auto logger = LoopsLib::Helpers::LogFactory<WeightedStrongFrechet>{};
    logger.trace("Inserting el: ", wi.weight, ", endpoint:", wi.lowEndpoint, ",src:", wi.src);
    if(m_inters.size() != 1) fixWeightMonotonicity();
}

void WeightedStrongFrechet::WeightIntervalList::insertMultiple(const std::vector<WeightInterval>& wi)
{
    if (wi.empty()) return;
    auto logger = LoopsLib::Helpers::LogFactory<WeightedStrongFrechet>{};
    for(const auto& el: wi)
    {
        logger.trace("Inserting el: ", el.weight, ", endpoint:", el.lowEndpoint, ",src:", el.src);
    }
    m_inters.insert(wi.begin(), wi.end());
    fixWeightMonotonicity();
}

std::pair<bool, std::set<WeightedStrongFrechet::WeightInterval>::iterator> WeightedStrongFrechet::
findHighestOverlappingWeightInterval(std::size_t vertex, std::size_t intervalId, const NT& queryValue,
    const StrongFrechetGraphData& data, bool verbose)
{
    const auto& interval = data.WhiteIntervals[vertex][intervalId];
    if (queryValue < interval.min) {
        if(verbose)
        {
            m_log.info("Query value below min: q=", queryValue, " inter: ", interval.min, "-", interval.max);
        }
        return std::make_pair(false, std::set<WeightedStrongFrechet::WeightInterval>::iterator{});
    }
    const auto& weightIntervals = m_weightIntervals[vertex][intervalId];
    auto rIt = weightIntervals.m_inters.rbegin();
    auto rEnd = weightIntervals.m_inters.rend();
    if(verbose)
    {
        m_log.info("Query value q=", queryValue, " inter: ", interval.min, "-", interval.max);
        m_log.info("Weight interval count", weightIntervals.size());
    }
    for(auto it = std::prev(weightIntervals.m_inters.end()); rIt!= rEnd; ++rIt, --it)
    {
        if(verbose) m_log.info("Current weightinterval low endpoint: ", it->lowEndpoint);
        if (it->lowEndpoint <= queryValue) {
            if(verbose) m_log.info("Found weightinterval ");
            return std::make_pair(true, it);
        }
    }
    if (verbose) m_log.info("No weightinterval found");
    return std::make_pair(false, std::set<WeightedStrongFrechet::WeightInterval>::iterator{});
}

std::pair<std::set<WeightedStrongFrechet::WeightInterval>::iterator, std::set<WeightedStrongFrechet::
    WeightInterval>::iterator> WeightedStrongFrechet::findOverlappingWeightIntervals(std::size_t vertex,
        std::size_t
        intervalId,
        const Interval&
        interval, const StrongFrechetGraphData& data)
{
    using It = std::set<WeightedStrongFrechet::WeightInterval>::iterator;

    auto& target = this->m_weightIntervals[vertex][intervalId];
    if (target.empty())
    {
        m_log.debug("Empty weight interval collection at ", vertex);
        return std::make_pair(target.end(), target.end());
    }
    
    // "Difficult" overlap case.
    if (target.rbegin()->lowEndpoint <= interval.min)
    {
        m_log.debug("Only last weight interval at ", vertex);
        return std::make_pair(std::prev(target.end()), target.end());

    }
    m_log.debug("Searching for interval at ", vertex);
    std::optional<It> start, end;
    for (auto it = target.begin(),it2=std::next(it); it != target.end(); ++it,++it2)
    {
        Interval searchInter;
        searchInter.min = it->lowEndpoint;
        searchInter.max = it2 == target.end() ? data.WhiteIntervals[vertex][intervalId].max : it2->lowEndpoint;
        if(searchInter.isEmpty())
        {
            m_log.error("Weight interval incorrectly ordered!");
        }
        if(searchInter.contains(interval.min))
        {
            start = it;
        }
        if (interval.min < searchInter.min && !start.has_value())
        {
            start = it;
        }
        // Potentially ignores all degenerate intervals.
        if(searchInter.contains(interval.max))
        {
            if(interval.max == searchInter.max)
            {
                end = std::next(it2);
            }
            else
            {
                end = std::next(it);
            }
            break;
        }
    }
    if (!start.has_value()) return std::make_pair(target.end(), target.end());
    if (!end.has_value()) end = target.end();

    return std::make_pair(start.value(), end.value());
}

void WeightedStrongFrechet::reconstructPath(const FrechetData& data, LoopsLib::DS::BaseGraph::Id_t startVert, std::vector<LoopsLib::DS::BaseGraph::Id_t>& path)
{

    using Id = DS::BaseGraph::Id_t;

    struct UsedInterval
    {
        // Associated vertex
        Id vId;
        // Index of associated white interval
        std::size_t intervalId;
        // Distance of current iterator to lowest iterator
        std::size_t dist;

        using WiIt = std::set<WeightInterval>::iterator;

        UsedInterval(
            Id vId, std::size_t intervalId,
            WiIt wisBegin,
            WiIt used
        ) :vId(vId), intervalId(intervalId), dist(std::distance(wisBegin, used)) {}

        bool operator<(const UsedInterval& other) const
        {
            if (vId != other.vId) return vId < other.vId;
            if(intervalId != other.intervalId) return intervalId < other.intervalId;
            return dist < other.dist;
        }
    };

    // Simple function for creating a UsedInterval object
    auto createUsedInterval = [this](Id vId, std::size_t intervalId, UsedInterval::WiIt used)
    {
        return UsedInterval(vId, intervalId, m_weightIntervals[vId][intervalId].begin(), used);
    };

    // Keep track of used intervals to avoid infinite loops.
    std::set<UsedInterval> usedIntervals;


    // Search state
    struct SearchState {
        // Vertex of the state
        Id currentV;
        // Current free interval
        std::size_t currentInterval;
        // Reachable range on vertex free interval, given the current weight pointer
        // This is the range in the interavl from the lowest point of the weight interval upwards.
        Interval reachableRange;
        // Lowest attainable iterator
        WeightIntervalList::It m_lowestIt;
        // Current iterator
        WeightIntervalList::It m_it;
        // Is this state newly pushed.
        bool isNew = true;

    };
    std::stack<SearchState> pathState;

    // Lowest matching parameter seen
    LoopsLib::NT lowestPointReached = std::numeric_limits<LoopsLib::NT>::max();
    // Starting matching parameter
    LoopsLib::NT startLowPoint = std::numeric_limits<LoopsLib::NT>::max();

    // Initial state  
    {
        // Set current vertex
        const DS::BaseGraph::Id_t currentV = startVert;
        // Start at last free interval
        std::size_t intervalId = data.WhiteIntervals[currentV].size() - 1;
        // Set reachable space
        Interval activeInterval = data.WhiteIntervals[currentV].back();
        // Potential weighted path pointers to use, given as pair of iterators from start to exclusive end
        auto potentialWis = std::make_pair(m_weightIntervals[currentV][intervalId].begin(), m_weightIntervals[currentV][intervalId].end());

        // Both end?
        if(potentialWis.first == potentialWis.second)
        {
            return;
        }

        // The current weighted path pointer iterator to use
        const auto currentWiIt = std::prev(potentialWis.second);

        // Update the low endpoint that we could reach via the weighted path pointer
        activeInterval.min = currentWiIt->lowEndpoint;

        lowestPointReached = std::min(lowestPointReached, activeInterval.min);
        startLowPoint = activeInterval.min;
        m_log.trace("Initial lowpoint ", lowestPointReached);

        // Push as intial state
        pathState.push(
            SearchState
            {
                startVert,
                intervalId,
                activeInterval ,
                potentialWis.first,
                currentWiIt
            }
        );
    }

    // Use clock to limit search time
    using Clock = std::chrono::high_resolution_clock;
    auto timePoint = Clock::now();

    // Flags that we are back tracking.
    bool isBacktracking = false;


    // Reconstruct the path for the current potential endpoint in backwards manner
    // Similar to path pointers of Alt et al.
    m_log.trace("Start reconstruct for ", startVert);

    ///
    /// The approach:
    /// We save weight intervals, in increasing value and with increasing matching parameter.
    /// 1) We start at the end vertex and walk back in the FSD. 
    /// Loop 
    /// 2) We pick the top weight interval, see what intervals could reach this weight interval and pick the top weight interval at a previous vertex.
    /// 2) We mark the weight interval as used.
    /// 3) We continue searching, and ignore used weight intervals. If we get stuck, due to following the top weight intervals into a loop that closes
    /// on itself, we backtrack, picking a worse weight interval at the first previous state that has more available weight intervals. All intermediate
    /// used weight intervals will become available again.
    /// 
    /// In worst case, this is exponential in the maximum  number of weights on an interval.
    ///
    ///
    auto indentGuard = m_log.addGlobalPrefix("\t");
    while (!pathState.empty())
    {
        // Check search limit
        auto timeDiff = Clock::now() - timePoint;
        if (std::chrono::duration_cast<std::chrono::seconds>(timeDiff).count() > m_maxSearchTimeS)
        {
            m_log.warn("Exceeded search time limit for reconstructing path");
            break;
        }

        // Get current top
        auto& state = pathState.top();
        m_log.trace("Current vertex: ", state.currentV, "(orig=",data.m_view.vertexByIndex(state.currentV)->id(),"), whiteInter=", state.currentInterval, ", low endpoint=", state.reachableRange.min, " stack size ", pathState.size());
        auto indentGuard2 = m_log.addGlobalPrefix("\t");
        m_log.trace("Available weights length=", std::distance(state.m_lowestIt, state.m_it), ",isNew=",state.isNew,", total wis= ", m_weightIntervals[state.currentV][state.currentInterval].size());
        {
            auto indentGuard3 = m_log.addGlobalPrefix("\t");
            for(const auto& el: m_weightIntervals[state.currentV][state.currentInterval])
            {
                m_log.trace("Weight inter: src=", el.src, ", lowend=", el.lowEndpoint, ", weight=", el.weight, ", lowend at src=",el.srcEndpoint);
            }
        }


        // Get weighted path pointer
        auto wiIt = state.m_it;

        bool needBacktrack = false;
        if (isBacktracking)
        {
            // We are backtracking, but the used pointer was the last, so backtrack further
            if (wiIt == state.m_lowestIt)
                needBacktrack = true;
            else
            {
                // We need to go to the next interval, but this opens up the previous one, so delete it in the visited set.
                auto usedInter = createUsedInterval(state.currentV, state.currentInterval, state.m_it);
                m_log.trace("Erasing used interval at start: ", usedInter.vId, " ", usedInter.intervalId, " ", usedInter.dist);
                usedIntervals.erase(usedInter);
                // Remove the used interval, but also move one down.
                --wiIt;
            }
        }

        // Check if the weight pointer is already in use
        while (!needBacktrack)
        {
            // Not in use, use this one
            if (usedIntervals.find(createUsedInterval(state.currentV, state.currentInterval, wiIt)) == usedIntervals.end())
            {
                break;
            }
            // All in use, need to backtrack
            if (wiIt == state.m_lowestIt)
            {
                m_log.trace("Reached lowest, backtracking");
                needBacktrack = true;
                break;
            }
            m_log.trace("Skipping to next wiIt");
            --wiIt;
        }

        // Perform backtracking
        if (needBacktrack)
        {
            m_log.trace("Backtracking");
            auto backTrackIndent = m_log.addGlobalPrefix("\t");

            // Only erase if this state is not a freshly discovered state
            if(!state.isNew)
            {
                // Switched to a different iterator, so original will not be used anymore
                auto usedInter = createUsedInterval(state.currentV, state.currentInterval, state.m_it);
                m_log.trace("Erasing used interval: ", usedInter.vId, " ", usedInter.intervalId, " ", usedInter.dist);
                usedIntervals.erase(usedInter);
            }
            // Leave the path state on the stack that we need to process next.
            pathState.pop();
            if (!path.empty())
            {
                path.pop_back();
            }
            // Flag that we are backtracking. 
            isBacktracking = true;
            // The weight pointer in the previous state should already be in use, so automatically 
            // causes backtracking if needed.
            continue;
        }

        // Update usage
        if (wiIt != state.m_it && !state.isNew)
        {
            // Switched to a different iterator, so original will not be used anymore
            auto usedInter = createUsedInterval(state.currentV, state.currentInterval, state.m_it);
            m_log.trace("Erasing used interval: ", usedInter.vId, " ", usedInter.intervalId, " ", usedInter.dist);
            usedIntervals.erase(usedInter);
        }

        // Reached end marker
        if (wiIt->src == -1) {
            lowestPointReached = 0;
            break;
        }

        // We are not backtracking anymore
        isBacktracking = false;

        // Assign the used iterator
        state.m_it = wiIt;
        state.isNew = false;

        // Update used intervals
        auto usedInter = createUsedInterval(state.currentV, state.currentInterval, state.m_it);
        usedIntervals.insert(usedInter);
        
        m_log.trace("Inserting used interval: ", usedInter.vId, " ", usedInter.intervalId, " ", usedInter.dist);
        m_log.trace("It vert:", wiIt->src, ", src endpoint:", wiIt->srcEndpoint, ", dist to weight pointers start in parent:", usedInter.dist);

        // Get the white interval from which the current vertex-whiteinterval combo is reachable.
        auto srcWhiteInterval = data.getWhiteInterval(wiIt->src, wiIt->srcEndpoint);
        // Vertex to move to (backwards)
        auto srcVert = wiIt->src;
        // The interval associated with the path pointer
        std::size_t srcIntervalId = data.intervalForVertexLocation(srcVert, wiIt->srcEndpoint);

        // Check 

        // Extend the path
        path.push_back(data.m_view.vertexByIndex(state.currentV)->findInEdge(data.m_view.vertexByIndex(srcVert)->id())->id());

        // At (potential) end vertex
        if (srcIntervalId == 0 && data.IntervalInclusions[srcVert][srcIntervalId].leftIncluded)
        {
            lowestPointReached = 0;
            m_log.trace("End vertex: ", srcVert);
            break;
        }

        // Push new state
        Interval newReachableInterval(srcWhiteInterval.min, std::min(wiIt->srcEndpoint,state.reachableRange.max));

        // Returns pair of bool and weight interval iterator. First is false if none could be found (should not occur).
        // Second contains iterator to highest element.
        //auto inter = findHighestOverlappingWeightInterval(srcVert, srcIntervalId, wiIt->srcEndpoint, data);
        auto inter = findHighestOverlappingWeightInterval(srcVert, srcIntervalId, newReachableInterval.max, data);
        // No such weight intervals?
        if(!inter.first)
        {
            // Should not happen, then the lr pointer was not monotonous.
            m_log.error("Got empty overlap when reconstructing path");
            m_log.error("\tSrc vert and interval: ", srcVert," ", srcIntervalId);
            m_log.error("\tSrc endpoint to lookup with: ", wiIt->srcEndpoint);
            std::size_t cnt = 0;
            for(const auto& el: data.WhiteIntervals[srcVert])
            {
                m_log.error("\tWhite interval ", cnt, ":[", el.min, ",", el.max, "]");
                ++cnt;
            }
            cnt = 0;
            for(const auto& el: m_weightIntervals[srcVert][srcIntervalId])
            {
                m_log.error("\tWeight interval ", cnt, ": low endpoint ", el.lowEndpoint, ", src endpoint", el.srcEndpoint);
                ++cnt;
            }
            (void)findHighestOverlappingWeightInterval(srcVert, srcIntervalId, wiIt->srcEndpoint, data, true);
            // Empty the path
            path.clear();
            return;
        }

        auto potentialWis = std::make_pair(m_weightIntervals[srcVert][srcIntervalId].begin(), inter.second);

       
        m_log.trace("Pushed new edge: vert-interId= ", srcVert, ", ", srcIntervalId, ", range= ", newReachableInterval.min, ",", newReachableInterval.max, "\t",
            "progress=", (newReachableInterval.min - lowestPointReached), ", weight=", potentialWis.second->weight);

        // Keep track of lowest reached point
        lowestPointReached = std::min(lowestPointReached, newReachableInterval.min);

        // Push new state
        pathState.push(SearchState{
            srcVert,
            srcIntervalId,
            newReachableInterval ,
            potentialWis.first,
            potentialWis.second
            });
    }
    // Path is still in reverse order!

    if (path.empty()) {
        m_log.warn("No path found from startVert:", startVert, ", lowest endpoint seen: ", lowestPointReached, " initial low point: ", startLowPoint, " path size:", data.polylineEdgeCount());
        return;
    }

    assert(pathState.size() == path.size() );
    /*while(!pathState.empty())
    {
        auto state = pathState.top();
        pathState.pop();
        std::cout << "Reachable interval: " << state.reachableRange.min << "," << state.reachableRange.max << ", vert:" << data.m_view.vertexByIndex(state.currentV)->id() << std::endl;
    }*/

    // Greedily attach stuff at start of reconstructed 
    if(m_greedyAttachEnd)
    {
        NT totalWeight = std::accumulate(path.begin(), path.end(), (NT)0.0, [this](NT curr, auto e1)
        {
            return curr + this->m_weights->at(e1);
        });
        auto totalSize = path.size();

        // Location of first vertex
        auto vPos = data.polyline.front().m_p0;

        // Get all position within epsilon dist
        std::vector<long long> verts;
        m_graph->getIndex().containedInDisk(vPos.x(), vPos.y(), data.m_epsilon, verts);
        std::set<long long> vertSet(verts.begin(), verts.end());
        // Mark used vertices to not create a non-simple end
        std::set<long long> usedSet;
        usedSet.insert(m_graph->edge(path.back())->m_source->id());
        for(const auto& e: path)
        {
            usedSet.insert(m_graph->edge(e)->m_sink->id());
        }
        //Greedily add high weight edges
        auto currV = m_graph->edge(path.back())->m_source;
        while(true)
        {
            long long maxEdge = -1;
            NT maxWeight = std::numeric_limits<NT>::lowest();
            for(auto* e : currV->m_inEdges)
            {
                if (vertSet.find(e->m_source->id()) == vertSet.end()) continue;
                if (usedSet.find(e->m_source->id()) != usedSet.end()) continue;
                auto w = m_weights->at(e->id());
                if(w > maxWeight)
                {
                    maxWeight = w;
                    maxEdge = e->id();
                }
            }
            // Heuristic: only add if mean becomes better.
            if (maxEdge != -1 && (totalWeight + maxWeight) / (NT)(totalSize+1) > totalWeight/ (NT)(totalSize))
            {
                //Update size and weight
                totalWeight += maxWeight;
                ++totalSize;

                path.push_back(maxEdge);
                currV = m_graph->edge(maxEdge)->m_source;
                usedSet.insert(currV->id());
            }
            else
            {
                break;
            }
        }
    }

    // Reverse path to get right orientation
    std::reverse(path.begin(), path.end());

    /*for(const auto& el: path)
    {
        auto* e = m_graph->edge(el);
        std::cout << "Source:" << e->m_source->id() << ", sink: " << e->m_sink->id() << std::endl;
    }*/
}

void WeightedStrongFrechet::reconstructPaths(const FrechetData& data, const std::vector<std::vector<LoopsAlgs::Frechet::PathPointer>>& pathPointers, std::vector<std::vector<DS::BaseGraph::Id_t>>& paths, std::vector<NT>& weights)
{
    using Id = DS::BaseGraph::Id_t;

    // Get endpoint with maximum value
    std::vector<DS::BaseGraph::Id_t> potentialPoints;
    for(DS::BaseGraph::Id_t id : m_potentialEndpoints)
    {
        if (m_weightIntervals[id].back().empty()) {
            if(m_seenEndpoints.find(id) != m_seenEndpoints.end())
            {
                m_log.warn("Seen endpoint that now does not have weight intervals");
            }
            continue;
        }
        potentialPoints.push_back(id);
        
    }
    if(m_seenEndpoints.empty())
    {
        m_log.warn("No potential endpoints seen");
        m_log.flush();
        slowError();
    }
    if(potentialPoints.empty())
    {
        m_log.warn("No feasible endpoints found");
        return;
    }
    std::sort(potentialPoints.begin(), potentialPoints.end(), [this](Id i0, Id i1)
    {
        return m_weightIntervals[i0].back().rbegin()->weight > m_weightIntervals[i1].back().rbegin()->weight;
    });

    int reconstructed = 0;
    // We don't do negative weight paths.
    for(auto startVertId : potentialPoints)
    {
        // Verify that a path exists using the pathpointers and the weight intervals.
        

        // Build in reverse order
        paths.emplace_back(); // Add empty vector to paths
        auto& path = paths.back();
        reconstructPath(data, startVertId, path);

        if(path.empty())
        {
            paths.resize(paths.size() - 1);
            continue;
        }
        ++reconstructed;
        if (reconstructed >= m_maxNum) break;
    }

    if(paths.empty())
    {
        m_log.warn("Could not find any path!");
        /*using namespace std::chrono_literals;
        std::this_thread::sleep_for(2s);*/
    }
    else
    {
        m_log.info("Path finding done, found ", paths.size(), " paths ");
    }
}

void WeightedStrongFrechet::slowError()
{
    if(m_slowErrors)
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(2s);
    }
}

LoopsAlgs::Frechet::WenkSweeplineResult WeightedStrongFrechet::beforeEdgeProcess(DS::BaseGraph::Edge* edge,
    const LoopsAlgs::Frechet::PrioQueueNode& node, const Interval& lrInterval,
    const LoopsAlgs::Frechet::StrongFrechetGraphData& data)
{
    auto logger = LoopsLib::Helpers::logFactory(this);

    logger.debug("##Before edge process");

    // The vertex that is the sink of the edge. We will update the intervals there
    const auto targetVert = data.m_view.indexForVertex(edge->m_sink->id());

    // ID of the interval we are coming from.
    const auto srcIntervalId = node.interIndex;
    const auto& srcInterval = node.inter;// data.WhiteIntervals[node.vertex][srcIntervalId];

    // Weight of the edge
    const auto edgeWeight = (*m_weights)[edge->id()];
    logger.trace("Wis of current interval ", m_weightIntervals[node.vertex][node.interIndex].m_inters.size());

    // Affected range (inclusive)
    logger.trace("Updating range for interval ", lrInterval.min, ":", lrInterval.max);

    // Print white intervals
    /*for(const auto& inter: data.WhiteIntervals[targetVert])
    {
        logger.trace("\t\t Wi= ", inter.min, ":", inter.max);
    }*/

    // The intervals at the target that are reachable
    const auto affectedRange = data.intervalRangeForLrPointer(targetVert, lrInterval);

    logger.trace("Updating range", affectedRange.first, ":", affectedRange.second);

    // The maximum weight at the sink node, as currently known, in the best case scenario
    if(m_weightIntervals[node.vertex][srcIntervalId].empty() && node.inter.min != 0)
    {
        logger.warn("Empty wis for vertex ", node.vertex, " at interval ", srcIntervalId);
    }
    const auto maxWeight = m_weightIntervals[node.vertex][srcIntervalId].empty() ? edgeWeight : m_weightIntervals[node.vertex][srcIntervalId].rbegin()->weight + edgeWeight;


    // Found a potential endpoint
    if (m_potentialEndpoints.find(targetVert) != m_potentialEndpoints.end())
    {
        // White interval at the endpoint is reachable?
        if (data.WhiteIntervals[targetVert].back().contains(lrInterval.max))
        {
            m_seenEndpoints.insert(targetVert);

            logger.debug("Reachable endpoint");
            logger.debug("In range:", affectedRange.second == data.WhiteIntervals[targetVert].size() - 1);
            if(!(affectedRange.second == data.WhiteIntervals[targetVert].size() - 1))
            {
                //using namespace std::chrono_literals;
                //std::this_thread::sleep_for(2s);
            }
        }
    }

    auto indentGuard = logger.addGlobalPrefix("\t");
    // Go over all affected reachable intervals (= reachable from current white interval), insert a new weight interval.
    // The weight interval container automatically rectifies the weight interval property.
    for (std::size_t affected = affectedRange.first; affected <= affectedRange.second; ++affected)
    {
        const auto& targetWhiteInterval = data.WhiteIntervals[targetVert][affected];
        auto& affectedWeightIntervals = m_weightIntervals[targetVert][affected];

        // The (simpler) case where the lrInterval target white interval is ''above'' the source interval.
        if (node.inter.max < targetWhiteInterval.min)
        {
            // Fixes itself
            affectedWeightIntervals.insert(WeightInterval{ targetWhiteInterval.min, node.vertex, srcInterval.max, maxWeight });
        }
        // Came from start vertex
        else if(node.inter.min == 0 && m_weightIntervals[node.vertex][srcIntervalId].empty())
        {
            // Fixes itself
            affectedWeightIntervals.insert(WeightInterval{ targetWhiteInterval.min, node.vertex, srcInterval.max, maxWeight });
        }
        // The other case, where there is overlap between the src interval and the target interval.
        else
        {
            // Get the range of weight intervals at the source and try to insert them one by one.
            Interval overlap = srcInterval.intersection(targetWhiteInterval);
            // Find start and end weight intervals in the intersection at the src
            auto srcWIRange = findOverlappingWeightIntervals(node.vertex, srcIntervalId, overlap, data);

            if(srcWIRange.first == srcWIRange.second)
            {
                m_log.error("Empty weight interval range when propagating new weight intervals");
                return LoopsAlgs::Frechet::WenkSweeplineResult::ContinueIt;;
            }

            std::vector<WeightInterval> toInsert;
            //auto srcIntervalRange = data.intervalRangeForLrPointer(node.vertex, overlap);
            std::size_t its = 0;
            for(auto it = srcWIRange.first; it != srcWIRange.second; ++it)
            {
                const auto insertWeight = it->weight + edgeWeight;
                auto lowEndpoint = std::max(it->lowEndpoint, overlap.min);
                // Add tiny bit if required
                if (m_artificiallyInflate) lowEndpoint += 0.000001;

                // Determine srcEndpoint for the new weight interval
                const auto& srcWeightIntervalList = m_weightIntervals[node.vertex][srcIntervalId];
                NT srcEndpoint = -1;
                if(std::next(it) == srcWeightIntervalList.m_inters.end())
                {
                    srcEndpoint = data.WhiteIntervals[node.vertex][srcIntervalId].max;
                }
                else
                {
                    srcEndpoint = std::next(it)->lowEndpoint;
                }

                toInsert.push_back(WeightInterval{ lowEndpoint, node.vertex, srcEndpoint, insertWeight });
                ++its;
            }
            if(its == 0)
            {
                m_log.error("Source did not have weight intervals! V=", node.vertex, ", isStart=", data.WhiteIntervals[node.vertex].begin()->min == 0,
                    " Overlap:", overlap.min, "-", overlap.max," available wis:",m_weightIntervals[node.vertex][srcIntervalId].m_inters.size(),
                    " srcInterval ", srcIntervalId);
                for(const auto& el : m_weightIntervals[node.vertex][srcIntervalId].m_inters)
                {
                    m_log.error("Wi: endpoint=", el.lowEndpoint, " weight=", el.weight);
                }
                //using namespace std::chrono_literals;
                //std::this_thread::sleep_for(2s);
            }
            affectedWeightIntervals.insertMultiple(toInsert);
        }
    }
    indentGuard.restore();

    logger.debug("##Weight update done");
    // Continue with algorithm
    return LoopsAlgs::Frechet::WenkSweeplineResult::ContinueIt;
}

WeightedStrongFrechet::WeightedStrongFrechet(const LoopsLib::DS::EmbeddedGraph* graph) : m_graph(graph),m_epsilon(1.0)
{
}

const std::set<LoopsLib::DS::BaseGraph::Id_t>& WeightedStrongFrechet::potentialEndpoints() const
{
    return m_potentialEndpoints;
}

LoopsLib::NT WeightedStrongFrechet::epsilon() const
{
    return m_epsilon;
}

void WeightedStrongFrechet::setGreedyAttachEnd(bool value)
{
    m_greedyAttachEnd = value;
}

void WeightedStrongFrechet::setMaximumPerPath(int maxim)
{
    m_maxNum = maxim;
}

int WeightedStrongFrechet::setMaximumPerPath() const
{
    return m_maxNum;
}

const std::vector<std::vector<WeightedStrongFrechet::WeightIntervalList>>& WeightedStrongFrechet::
weightIntervals() const
{
    return m_weightIntervals;
}

void WeightedStrongFrechet::setMaxSearchTimeS(double maxSearchTime)
{
    m_maxSearchTimeS = maxSearchTime;
}

double WeightedStrongFrechet::maxSearchTimeS() const
{
    return m_maxSearchTimeS;
}

void WeightedStrongFrechet::precompute(const std::vector<DS::BaseGraph::Id_t>& path, NT epsilon, StrongFrechetGraphData& data)
{
    auto logger = LoopsLib::Helpers::logFactory(this);

    logger.info("Path size: ", path.size());

    m_epsilon = epsilon;

    // Setup the data for the current graph and path
    // Internal state is still uninitialized.
    data.setup(m_graph, path);

    // Compute elements of the data objects
    Frechet::FrechetGraphComputations precomputer(m_epsilon);

    precomputer.computeFDi(data);
    precomputer.computeLeftRightPointersBF(data);
}

void WeightedStrongFrechet::compute(const std::vector<DS::BaseGraph::Id_t>& path, 
    NT epsilon, const std::vector<NT>& weights, std::vector<std::vector<DS::BaseGraph::Id_t>>& outputPath)
{
    auto logger = LoopsLib::Helpers::logFactory(this);

    logger.info("Path size: ",path.size());
    
    m_weights = &weights;
    FrechetData data;

    precompute(path, epsilon, data);

    compute(data, weights, outputPath);
}

void WeightedStrongFrechet::precompute(const Trajectory& trajectory, NT epsilon, LoopsAlgs::Frechet::StrongFrechetGraphData& data)
{
    auto logger = LoopsLib::Helpers::logFactory(this);

    m_epsilon = epsilon;

    // Setup the data for the current graph and path
    // Internal state is still uninitialized.
    data.setup(m_graph, trajectory);

    // Compute elements of the data objects
    Frechet::FrechetGraphComputations precomputer(m_epsilon);

    precomputer.computeFDi(data);
    precomputer.computeLeftRightPointersBF(data);
}

void WeightedStrongFrechet::compute(const Trajectory& trajectory, NT epsilon, const std::vector<NT>& weights,
    std::vector<std::vector<LoopsLib::DS::BaseGraph::Id_t>>& outputPath)
{
    auto logger = LoopsLib::Helpers::logFactory(this);

    logger.info("Path size: ", trajectory.size());

    m_weights = &weights;
    FrechetData data;

    precompute(trajectory, epsilon, data);

    compute(data, weights, outputPath);
}

void WeightedStrongFrechet::compute(const LoopsAlgs::Frechet::StrongFrechetGraphData& data,
    const std::vector<NT>& weights, std::vector<std::vector<DS::BaseGraph::Id_t>>& outputPath)
{
    auto logger = LoopsLib::Helpers::logFactory(this);
    // Compute the pointers per interval per vertex
    using SelfType = std::decay_t<decltype(*this)>;

    m_weights = &weights;

    // Initialize the weight intervals structure.
    m_weightIntervals.clear();
    m_weightIntervals.resize(data.number_of_vertices(), {});

    // Don't initialize weight intervals
    for (std::size_t i = 0; i < data.number_of_vertices(); ++i)
    {
        m_weightIntervals[i].resize(data.WhiteIntervals[i].size(), {});
    }

    m_potentialEndpoints.clear();
    m_seenEndpoints.clear();
    // Add all starting intervals as zero weight weight intervals
    for (std::size_t i = 0; i < data.number_of_vertices(); ++i)
    {
        // Can not be anything without white intervals
        if (data.WhiteIntervals[i].empty()) continue;

        // Collect potential endpoints
        if (data.isPotentialEndpoint(i, data.WhiteIntervals[i].size() - 1))
        {
            m_potentialEndpoints.insert(i);
        }
    }
    if(m_potentialEndpoints.empty())
    {
        m_log.warn("Empty potential endpoints before processing, skipping");
        return;
    }

    // Setup the sweepline algorithm
    CleanAltSweepline<SelfType> sweepLine(data);

    // Create the basic datastructures needed for the algorithm.
    std::vector<std::vector<Frechet::PathPointer>> pathPointers;
    LoopsLib::DS::Heap<Frechet::PrioQueueNode> prioQueue;
    std::vector<Geometry::Interval> Cis;

    sweepLine.initializeDefaultStart(prioQueue, Cis, pathPointers);

    logger.info("Graph view size: ", data.m_view.availableVertices().size());

    sweepLine.apply(prioQueue, Cis, *this, pathPointers);

    // Reconstruct path
    std::vector<NT> outWeights; //TODO unused for now
    //std::vector<DS::BaseGraph::Edge*> edgePath;
    reconstructPaths(data, pathPointers, outputPath, outWeights);
    //std::transform(edgePath.begin(), edgePath.end(), std::back_inserter(outputPath), [](auto* e) {return e->id(); });
}
