#ifndef HELPERS_NEW_SWEEPLINE_H
#define HELPERS_NEW_SWEEPLINE_H
#include "FrechetHelpers.h"
#include <thread>
#include <chrono>

namespace LoopsAlgs::Frechet
{

    template<typename U, typename V>
    struct PairHasher
    {
        template<typename T>
        inline void hash_combine(std::size_t& seed, const T& val) const
        {
            std::hash<T> hasher;
            seed ^= hasher(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }

        std::size_t operator()(const std::pair<U, V>& pair) const
        {
            size_t seed = 0;
            hash_combine(seed, pair.first);
            hash_combine(seed, pair.second);
            return seed;
        }
    };
    struct LeftEndpointLessComparison
    {
        bool operator()(const Interval& inter, NT value) const
        {
            return inter.min < value;
        }
    };
    struct LeftEndpointGreaterComparison
    {
        bool operator()(const Interval& inter, NT value) const
        {
            return inter.min > value;
        }
    };
    struct RightEndpointLessComparison
    {
        bool operator()(const Interval& inter, NT value) const
        {
            return inter.max < value;
        }
    };
    struct RightEndpointGreaterComparison
    {
        bool operator()(const Interval& inter, NT value) const
        {
            return inter.max > value;
        }
    };

    class SortedIntervalList
    {
        std::vector<Interval> m_data;

        struct Iterator
        {
            const std::vector<Interval>& m_data;
            std::size_t index = 0;
            Iterator(const std::vector<Interval>& data):m_data(data){}

            NT operator*() const
            {
                auto ind = index >> 1;
                bool isEnd = index & 1;
                return isEnd ? m_data[ind].max : m_data[ind].min;
            }
            Iterator& operator++()
            {
                ++index;
                return *this;
            }
            Iterator& operator+(std::size_t value)
            {
                index += value;
                return *this;
            }
        };
    public:
        auto begin() const
        {
            return m_data.begin();
        }
        auto end() const
        {
            return m_data.end();
        }
        auto size() const
        {
            return m_data.size();
        }
        auto empty() const
        {
            return m_data.empty();
        }
        std::pair<std::vector<Interval>::const_iterator, std::vector<Interval>::const_iterator> findOverlapping(const Interval& queryInterval) const
        {
            
        }
        bool isSorted() const
        {
            if (m_data.size() <= 1) return true;

            for(std::size_t i =0; i < m_data.size()-1; ++i)
            {
                if (m_data[i].min > m_data[i + 1].min) return false;
                if (m_data[i].max > m_data[i + 1].min) return false;
            }
            return true;
        }

    };

    struct AltBasicHooks
    {
        LoopsLib::DS::BaseGraph::Id_t exploredEdges = 0;
        bool endWasSeen = false;
        WenkSweeplineResult beforeEdgeProcess(LoopsLib::DS::BaseGraph::Edge* edgeToExplore, const PrioQueueNode& currentNode, const Interval& lrPointer, const StrongFrechetGraphData& data)
        {
            const auto vId = data.m_view.indexForVertex(edgeToExplore->m_sink);
            if(data.WhiteIntervals[vId].back().intersectsWith(lrPointer))
            {
                endWasSeen = true;
            }

            exploredEdges += 1;
            return WenkSweeplineResult::ContinueIt;
        }
    };

    namespace Helpers
    {
        template<typename Graph_t>
        struct HighMemoryGraphData
        {
            using Id_t = typename Graph_t::Id_t;
            const Graph_t* graph;
            // Full white intervals
            std::vector<std::vector<Interval>> WhiteIntervals;
            // -1 means no white interval present
            std::vector<std::vector<long long>> CellToWhiteInterval;
            struct ReachabilityPointer
            {
                NT lowestPoint;
                std::size_t startWI;
                std::size_t endWI;
            };
            // Reachability pointers
            std::vector < std::vector<std::unordered_map<Id_t, ReachabilityPointer>>> ReachabilityPointers;

            ReachabilityPointer getReachablityPointers(Id_t vertex, NT offsetInVertexCi)
            {

            }
        };
        template<typename Graph_t>
        struct Graphdata
        {
            using Id_t = typename Graph_t::Id_t;
            const Graph_t* graph;
            // Full white intervals
            std::vector<std::vector<Interval>> WhiteIntervals;
            struct ReachabilityPointer
            {
                NT lowestPoint;
                std::size_t startWI;
                std::size_t endWI;
            };
            // Reachability pointers
            std::vector < std::vector<std::unordered_map<Id_t, ReachabilityPointer>>> ReachabilityPointers;

            ReachabilityPointer getReachable(Id_t vertex, NT offsetInVertexCi)
            {
                
            }
        };
    }

    template<typename IterationHooks, typename LoggerType = IterationHooks>
    struct NewSweepline {

        static_assert(detail::has_beforeEdgeProcess<IterationHooks>::value,
            "Hooks should have method beforeEdgeProcess(DS::BaseGraph::Edge*, const PrioQueueNode&, const Interval&, const StrongFrechetGraphData&)");
        
        //Logger.
        LoopsLib::Helpers::LogFactory<LoggerType> logger;
        // The graph
        const Graph* m_graph;
        // Data for strong frechet for a specific path
        const StrongFrechetGraphData* m_data;

        using AltHeap = LoopsLib::DS::Heap<PrioQueueNode>;

        using PathPointers = std::vector<std::vector<PathPointer>>;

        /**
         * \brief Create the sweepline algorithm
         * \param graph Target base graph to apply the algorithm to
         * \param data Constructed strong Frechet data for the graph.
         */
        NewSweepline(const StrongFrechetGraphData& data) :m_graph(data.m_graph),m_data(&data) {}

        /**
         * \brief 
         * \param targetVert The target vertex
         * \param node The priority queue node containing source vertex and interval information
         * \param usedEdge Edge to use when going from the source vertex to the target vertex
         * \param startInterval The start interval to assign to
         * \param endInterval INCLUSIVE end interval 
         * \param pathPointers Reference to path pointers object
         */
        void updatePathPointers(Graph::Id_t targetVert, const PrioQueueNode& node, const Graph::Id_t usedEdge, int startInterval, int endInterval, NT lrPointerLeft, PathPointers& pathPointers)
        {
            logger.trace("\t\t Pp: ", node.vertex, "->", targetVert, "[", startInterval, ",", endInterval, "]");

            auto& data = *m_data;
            
            // Update the path pointer(s) for modified intervals.
            for (int current = startInterval; current <= endInterval; ++current)
            {
                //TODO: HACK. Probably to fix this, we need a special case for the degenerate start intervals...
                //assert(!data.pathPointers[targetVert][current].wasProcessed);
                if (pathPointers.at(targetVert).at(current).wasProcessed) continue;

                // if(pathPointers[targetVert].find(current) != pathPointers[targetVert].end(){
                // check if lowets point is lower, then update
                // else{
                // just add, since doesn't have any yet

                // Update if it makes the reachable interval larger
                // When the pathpointer is not set, lowestPointAtSelf is the largest possible value and will always return true for below check.
                auto lowestToAssign = std::max(lrPointerLeft, data.WhiteIntervals[targetVert][current].min);
                if(pathPointers[targetVert][current].lowestPointAtSelf > lowestToAssign)
                {
                    assert(!node.inter.isEmpty());
                    pathPointers[targetVert][current] = PathPointer{
                        usedEdge,
                        node.inter,
                        false,
                        lowestToAssign
                    };
                    // Should be contained in the white interval
                    assert(data.WhiteIntervals[targetVert][current].contains(pathPointers[targetVert][current].lowestPointAtSelf));
                }
            }
        }

        /**
         * \brief Initializes the heap, Cis and path pointers for a regular sweepline application for full path finding in the graph.
         * The heap will contain intervals that have a white left endpoint. The Cis will be updated accordingly. The pathpointers are
         * resized to be able to contain all pointers for the intervals.
         * \param heap The heap to fill
         * \param Cis 
         * \param pathPointers 
         */
        void initializeDefaultStart(AltHeap& heap, std::vector<Interval>& Cis, std::vector<std::vector<PathPointer>>& pathPointers) const
        {
            // Number of vertices, represented in the data object
            const auto vNum = m_data->FDiWhiteIntervals.size();

            Cis.resize(vNum, Interval{}); //All are empty
            // Initialize with white starting points
            NT x = 0;
            for (auto i = 0; i < vNum; ++i)
            {
                //if (!m_data->IntervalInclusions[i][0].leftIncluded) continue;
                //// Leftmost coordinate should be white.
                //Interval initialInterval(0, 0);
                //heap.insert(PrioQueueNode(initialInterval, i, 0));
                //// Initialize Cis.
                //Cis.at(i).min = Cis[i].max = m_data->FDiWhiteIntervals.at(i).at(0).min;
                if (!m_data->IntervalInclusions[i][0].leftIncluded) continue;
                // Initialize Cis.
                Cis.at(i).min = Cis.at(i).max = 0;
                // Leftmost coordinate should be white.
                Interval initialInterval = Cis.at(i);
                heap.insert(PrioQueueNode(initialInterval, i, 0),i);
            }
            allocatePathPointers(pathPointers);
        }
         
        void allocatePathPointers(std::vector<std::vector<PathPointer>>& pathPointers) const
        {
            const auto vNum = m_data->number_of_vertices();
            pathPointers.resize(vNum, {});
            for (std::size_t i = 0; i < vNum; ++i)
            {
                pathPointers.at(i).resize(m_data->WhiteIntervals[i].size(), {});
            }
        }

        /**
         * \brief Insert the next interval in the priority queue
         * \param prioQueue The priority queue
         * \param vertex The vertex for which to insert the new interval
         * \param newIntervalIndex The index of the new interval (number of the interval in FD_i)
         * \param Ci The current 'reachable' interval. New interval should be clipped to the maximum of this interval.
         */
        inline void insertNextInterval(AltHeap& prioQueue, Graph::Id_t vertex, int newIntervalIndex, const Interval& Ci)
        {
            auto& data = *m_data;

            auto interToQueue = data.WhiteIntervals.at(vertex).at(newIntervalIndex);

            // Don't queue if the interval is outside the reachable region
            if (Ci.max < interToQueue.min) return;

            if (!interToQueue.isEmpty())
            {
                // Push to the queue.
                logger.trace("\t Interval ", newIntervalIndex, " [", interToQueue.min, ",", interToQueue.max, "] inserted ");
                prioQueue.insert(PrioQueueNode(interToQueue, vertex, newIntervalIndex), vertex);
            }
        }

        /**
         * \brief Checks if the given position is a whitepoint in the one-dimensional freespace for the given vertex
         * \param targetVert The vertex
         * \param pos The position
         * \returns Whether the position is a white point or not.
         */ 
        bool isWhitePoint(std::size_t targetVert, LoopsLib::NT pos) const
        {
            std::size_t cell = (std::size_t)pos;
            const auto& interval = m_data->FDiWhiteIntervals[targetVert][cell];
            return !interval.isEmpty() && interval.containsApprox(pos - (LoopsLib::NT)cell, 0.001);
        }

        /**
         * \brief Find the range of overlappingwhite intervals for the given parameter space range
         * \param targetVert The vertex for which we will inspect the white intervals
         * \param interval The interval to search with
         * \return Pair of inclusive start and end white intervals that overlap, or (-1,-1) if none overlap
         */
        std::pair<long long, long long> overlappingWhiteIntervals(std::size_t targetVert, const LoopsLib::Geometry::Interval& interval) const
        {
            if (m_data->WhiteIntervals[targetVert].empty()) return std::make_pair(-1, -1);
            const auto& wis = m_data->WhiteIntervals[targetVert];
            // First element that is >= the minimum of the provided interval
            auto lowerMin = std::lower_bound(wis.begin(), wis.end(), interval.min, LeftEndpointLessComparison());
            auto lowerMax = std::lower_bound(wis.begin(), wis.end(), interval.min, RightEndpointLessComparison());
            if(lowerMin == wis.end())
            {
                if (lowerMax != wis.end()) return std::make_pair(wis.size() - 1, wis.size() - 1);
                return std::make_pair(-1, -1);
            }
            auto startInterval = std::min(std::distance(wis.begin(), lowerMin), std::distance(wis.begin(), lowerMax));
            // Look for end by searching for max with right endpoints in reverse direction
            auto upperMax = std::lower_bound(wis.rbegin(), wis.rend(), interval.max, RightEndpointGreaterComparison());
            auto upperMin = std::lower_bound(wis.rbegin(), wis.rend(), interval.max, LeftEndpointGreaterComparison());
            if(upperMax == wis.rend())
            {
                if (upperMin != wis.rend()) return std::make_pair(0, 0);
                return std::make_pair(-1, -1);
            }
            auto endInterval= wis.size()-1 - std::min(std::distance(wis.rbegin(), upperMax), std::distance(wis.rbegin(), upperMin));
            
            return std::make_pair(startInterval, endInterval);
        }

        /**
         * \brief Acquires the left-right reachability pointers from the srcVertex to the targetVert at the current cell currCell.
         * takes into account the current offset x, that is the actual lower location of the white interval at the source vertex.
         * Makes sure that the reachbility pointers point to an white interval, or returns an empty pointer interval otherwis.
         * 
         */ 
        bool getReachabilityPointers(std::size_t srcVertex, std::size_t targetVert, LoopsLib::NT x, LoopsLib::Geometry::Interval& lrPointer, LeftRightInclusion& lrInclusion,
            std::pair<int,int>& reachableWis){

            const auto& lrPointers = m_data->lrPointers.at(srcVertex);
            std::size_t currCell = (std::size_t)x;

            // Only happens when x is exactly the end of the trajectory or larger...
            if(currCell >= lrPointers.size()){
                logger.trace("\t\tCell out of range", currCell," at vert ", srcVertex, " lrPoitners size: ", lrPointers.size());
                return false;
            }
            // Ignore edges for which no path exists.
            // Assume that both lpointer and rpointer are invalid in that case!
            if (lrPointers[currCell].find(targetVert) == lrPointers[currCell].end())
            {
                logger.trace("\t\tEdge with endpoint ", targetVert, " not reachable from vert ", srcVertex, " and  cell ", currCell);
                return false;
            }

            // Get left/right pointer and inclusion
            lrPointer = lrPointers[currCell].at(targetVert).pointers;
            lrInclusion = lrPointers[currCell].at(targetVert).inclusion;
            reachableWis = std::make_pair(lrPointers[currCell].at(targetVert).minWi, lrPointers[currCell].at(targetVert).maxWi);
            // Update the minimum according to the current sweepline value.
            lrPointer.min = std::max(lrPointer.min, x);
            logger.trace("\t\tChecking with lr pointer ", lrPointer.min, "-",lrPointer.max);

            // Update accordingly, we may have ended up in a non-white interval
            if(!isWhitePoint(targetVert, lrPointer.min))
            {
                // Has to be one higher then...
                int original = m_data->CellToIntervalId[targetVert][(std::size_t)lrPointer.min];
                if (original + 1 >= m_data->WhiteIntervals[targetVert].size()) {
                    logger.trace("\t\tBottom of src interval caused the vertex to be unreachable");
                    return false;
                }
                lrPointer.min = m_data->WhiteIntervals[targetVert][original + 1].min;
                reachableWis.first = original + 1;
            }

            // Finding a substitue may have caused the interval to actually become empty
            if (lrPointer.isEmpty()) {
                logger.trace("\t\t LR pointer empty");
                return false;
            }
            return true;
        }

        void updateQueue(AltHeap& prioQueue, std::size_t vId, const Interval& Ci)
        {
            const auto& data = *m_data;
            if (prioQueue.containsId(vId) && prioQueue.dataForId(vId).inter.max < Ci.min)
            {
                logger.error("Unprocessed entry in heap to be overwritten for vert ",vId,":",
                    prioQueue.dataForId(vId).inter.min, ",",
                    prioQueue.dataForId(vId).inter.max, " for inter ", prioQueue.dataForId(vId).interIndex,
                    " Ci: ", Ci.min, " : ", Ci.max);
                logger.error("WIs:");
                for (const auto& wi : m_data->WhiteIntervals[vId])
                {
                    logger.error("Wi ", wi.min, ":", wi.max);
                }
            }

            const auto queueInterval = data.intervalForVertexLocation(vId, Ci.min);

            auto intervalToQueue = data.WhiteIntervals[vId][queueInterval];
            intervalToQueue.min = std::max(Ci.min, intervalToQueue.min);

            PrioQueueNode prioNode(intervalToQueue, vId, queueInterval);
            assert(!Ci.isEmpty());
            // Find associated interval in prio queue.
            if (prioQueue.containsId(vId))
            {
                logger.trace("\t\t Updating key in prioqueue to ", Ci.min, " for vert ", vId);
                if (prioQueue.dataForId(vId).inter.min < intervalToQueue.min)
                {
                    logger.error("Updating interval to higher value: original ", prioQueue.dataForId(vId).inter.min, " new: ", intervalToQueue.min);
                }
                prioQueue.updateKey(vId, prioNode);
            }
            else
            {
                logger.trace("\t\t Inserting in prioqueue with value: ", Ci.min, ", with vertex: ", vId);
                prioQueue.insert(prioNode, vId);
            }
            assert(!prioQueue.dataForId(vId).inter.isEmpty());
        }
        /**
         * \brief Applies the Wenk et al. sweepline/dynamic program computation for Frechet mapmatching, adapted for directed graphs.
         * It is assumed that the prioqueue is initialized, the path pointers are initialized to empty pointers and the Cis are initialized
         * appropriately (empty intervals or degenerate intervals).
         * \param prioQueue The priority queue to use
         * \param Cis The Ci elements (reachable intervals per vertex)
         * \param hooks Hook object with beforeEdgeProcess() method that is called before processing any edge. Should return 
         * \param pathPointers 
         */
        void apply(AltHeap& prioQueue, std::vector<Interval>& Cis, IterationHooks& hooks, std::vector<std::vector<PathPointer>>& pathPointers)
        {
            auto& data = *m_data;
            LoopsLib::NT x = 0;
            
            // That vertex-whiteinterval combos already processed.
            std::unordered_set<std::pair<LoopsLib::DS::BaseGraph::Id_t, int>,PairHasher<LoopsLib::DS::BaseGraph::Id_t, int>> processedIntervals;

            bool done = false;

            while (!prioQueue.empty())
            {
                const auto prioEntry = prioQueue.extract();
                const auto node = prioEntry.value;

                // Advance x, the current sweepline value
                x = node.inter.min;

                logger.trace("Vertex from prio queue: ", node.vertex, " with val ", x, ", min-max:", node.inter.min," ",node.inter.max,". New prio size: ", prioQueue.size(),
                    ", associated Ci:", Cis[node.vertex].min, ",",Cis[node.vertex].max);

                // Fix up the reachable region for the vertex
                //Cis[node.vertex].min = x;

                // Insert next interval only if another interval exists
                if (node.interIndex + 1 < data.WhiteIntervals[node.vertex].size())// && processedIntervals.find(std::make_pair(node.vertex, node.interIndex+1)) == processedIntervals.end())
                {
                    insertNextInterval(prioQueue, node.vertex, node.interIndex+1, Cis[node.vertex]);
                }

                assert(data.CellToIntervalId[node.vertex][(int)node.inter.min] != -1);
                assert((int)node.inter.max == data.polyline.size() || data.CellToIntervalId[node.vertex][(int)node.inter.max] != -1);

                // Update adjacent vertices to comply with new interval
                for (auto* e : m_data->m_view.outEdgesByIndex(node.vertex))
                {
                    if (done) break;
                    
                    // Index to identify e with.
                    const auto vId = m_data->m_view.indexForVertex(e->m_sink->id());

                    // Reachability data for edge e and current vertex/white interval
                    Interval lrPointer;
                    LeftRightInclusion lrInclusion;
                    std::pair<int, int> reachableWis;
                    logger.trace("\tChecking edge with endpoint ", vId);

                    // No suitable reachability pointers were found.
                    if(!getReachabilityPointers(node.vertex, vId, x, lrPointer, lrInclusion, reachableWis)){
                        continue;
                    }
                    logger.trace("\t\tStarting edge process");

                    //Check rpointer is at end of free surface, then we are done.
                    auto result = hooks.beforeEdgeProcess(e, node, lrPointer, data);
                    switch(result)
                    {
                    case WenkSweeplineResult::Done:
                        return;
                    case WenkSweeplineResult::SkipRest:
                        continue;
                    case WenkSweeplineResult::DoneAfterUpdates:
                        // Flag that we are done, but continue with path pointer updates before quitting
                        done = true;
                        break;
                    default:
                            break;
                    }

                    // Replace Ci with interval when r(C_i) < l_{i,j}(I)
                    // This only happens when r(C_i) < x, since otherwise,
                    // the feasible chain of I should overlap with C_i, by convexity
                    // of the cells. In theory at least.
                    if (Cis[vId].isEmpty() || Cis[vId].max < lrPointer.min)
                    {
                        if (Cis[vId].isEmpty())
                        {
                            logger.trace("\t\t Updating Ci that was empty");
                        }
                        assert(data.WhiteIntervals[vId][data.CellToIntervalId[vId][(int)lrPointer.max]].max == lrPointer.max);

                        logger.trace("\t\t Updating full Ci for ", vId, ": from ",Cis[vId].min,",",Cis[vId].max, " to ", lrPointer.min, ",", lrPointer.max);
                        /*assert(Cis[vId].isEmpty() || (
                            data.CellToIntervalId[vId][(int)Cis[vId].max] != data.CellToIntervalId[vId][(int)lrPointer.min]
                            ));*/
                        const int startInterval = data.intervalForVertexLocation(vId, lrPointer.min);
                        const int endInterval = data.intervalForVertexLocation(vId, lrPointer.max);
                        if(startInterval != reachableWis.first || endInterval != reachableWis.second)
                        {
                            if(startInterval !=0 && endInterval != 0)
                            {
                                logger.error("Not same intervals for updating lrpointer to above:", startInterval, ":", endInterval, " vs ",
                                    reachableWis.first, ":", reachableWis.second);
                                logger.flush();
                                using namespace std::chrono_literals;
                                std::this_thread::sleep_for(2s);
                            }
                        }
                        updatePathPointers(vId, node, e->id(),
                            startInterval,
                            endInterval,
                            lrPointer.min,
                            pathPointers);

                        // Update Ci accordingly
                        Cis[vId] = lrPointer;

                        updateQueue(prioQueue, vId, Cis[vId]);
                        continue;
                    }
                    
                    // If we don't fully replace, update ends if necessary.
                    // Update the min value of the reachable chain
                    if (lrPointer.min < Cis[vId].min) // Lpointer guaranteed to point to an interval!
                    {
                        //Update interval in prio queue.
                        logger.trace("\t\t Updating lowerend Ci for ", vId, ": ", lrPointer.min,",",Cis[vId].min);

                        const int startInterval = data.intervalForVertexLocation(vId, lrPointer.min);
                        // Potentially coincides with a fully covered interval, will be caught in updatePathPointers()
                        const auto endInterval = data.intervalForVertexLocation(vId, Cis[vId].min);
                        updatePathPointers(vId, node, e->id(),
                            startInterval,
                            endInterval,
                            lrPointer.min,
                            pathPointers);

                        // Update lower end of Ci for the current vertex
                        Cis[vId].min = lrPointer.min;

                        // Update the white interval in the queue
                        updateQueue(prioQueue, vId, Cis[vId]);
                    }
                    // Update the max of the reachable chain
                    // Note: should make entirely new white intervals reachable, since Cis[vId].max should
                    // be the end of some white interval at vId (CONVEXITY!!)
                    if (lrPointer.max > Cis[vId].max)
                    {
                        Cis[vId].min = std::max(Cis[vId].min, x);
                        // Take diff with current
                        const int startInterval = data.intervalForVertexLocation(vId, Cis[vId].max);
                        const int endInterval = data.intervalForVertexLocation(vId, lrPointer.max);
                        logger.trace("\t\t Updating higherend Ci for ", vId, " from  ", Cis[vId].max, " to ", lrPointer.max, " inters: ", startInterval, "-", endInterval);
                        updatePathPointers(vId, node, e->id(),
                            startInterval,
                            endInterval,
                            lrPointer.min,
                            pathPointers);
                     
                        Cis[vId].max = lrPointer.max;

                        // NOTE: THIS SHOULDN'T BE NECESSARY?
                        if (!prioQueue.containsId(vId))
                        {
                            logger.warn("\t\t Inserting from higher in prioqueue for vertex ", vId);
                            const auto newStart = data.intervalForVertexLocation(vId, Cis[vId].min);
                            const auto maxEl = data.intervalForVertexLocation(vId, Cis[vId].max);
                            bool wasInserted = false;
                            for(std::size_t target = newStart; target <= maxEl; ++target)
                            {
                                if (data.WhiteIntervals[vId][target].max > Cis[vId].max) break;
                                // Note that this should be at most 1 index apart, otherwise something weird is happening.
                                if (processedIntervals.find(std::make_pair(vId, target)) == processedIntervals.end())
                                {
                                    wasInserted = true;
                                    PrioQueueNode prioNode(data.WhiteIntervals[vId][target], vId, target);
                                    logger.trace("\t\t Inserting from higher in prioqueue with value: ", data.WhiteIntervals[vId][target].min, ", with vertex: ", vId);
                                    prioQueue.insert(prioNode, vId);
                                    Cis[vId].min = prioNode.inter.min;
                                    break;
                                }
                            }
                            if(!wasInserted)
                            {
                                logger.error("\t\t Extended highend without having any in the queue anymore! Vert=", vId, " new high:",Cis[vId].max);
                            }
                            
                            /*
                            PrioQueueNode prioNode(data.WhiteIntervals[vId][startInterval], vId, startInterval);
                            logger.info("\t\t Inserting in prioqueue with value: ", data.WhiteIntervals[vId][startInterval].min, ", with vertex: ", vId);
                            prioQueue.insert(prioNode, vId);*/
                        }
                    }
                }
            }
        }
    };
}
#endif