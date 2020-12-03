#ifndef HELPERS_CLEAN_ALT_SWEEPLINE_H
#define HELPERS_CLEAN_ALT_SWEEPLINE_H
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Helpers/Logger.h>
#include <LoopsLib/Geometry/Interval.h>
#include "FrechetHelpers.h"
#include <LoopsLib/DS/Heap.h>
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
            size_t seed = 332;
            hash_combine(seed, pair.first);
            hash_combine(seed, pair.second);
            return seed;
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

    //Src: https://www2.cs.arizona.edu/~alon/papers/gFrechet.pdf
    template<typename IterationHooks, typename LoggerType = IterationHooks>
    struct CleanAltSweepline {

        static_assert(detail::has_beforeEdgeProcess<IterationHooks>::value || detail::has_beforeEdgeProcessExtended<IterationHooks>::value,
            "Hooks should have method beforeEdgeProcess(DS::BaseGraph::Edge*, const PrioQueueNode&, const Interval&, const StrongFrechetGraphData&)");
        
        //Logger.
        LoopsLib::Helpers::LogFactory<LoggerType> logger;
        // The graph
        const Graph* m_graph;
        // Data for strong frechet for a specific path
        const StrongFrechetGraphData* m_data;
        // Slow down errors in the logging by waiting a bit
        bool m_slowErrors = false;

        // The heap to use
        using AltHeap = LoopsLib::DS::Heap<PrioQueueNode>;

        // The pathpointers type
        using PathPointers = std::vector<std::vector<PathPointer>>;

        /**
         * \brief Create the sweepline algorithm
         * \param graph Target base graph to apply the algorithm to
         * \param data Constructed strong Frechet data for the graph.
         */
        CleanAltSweepline(const StrongFrechetGraphData& data) :m_graph(data.m_graph),m_data(&data) {}


        void slowError()
        {
            if(m_slowErrors)
            {
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(2s);
            }
        }

        void updatePathPointers(Graph::Id_t targetVert, const Interval& srcInterval, const Graph::Id_t usedEdge, const Interval& origCi, const Interval& newCi, PathPointers& pathPointers)
        {
            std::pair<int, int> origWis;
            if(origCi.isEmpty()) origWis = std::make_pair(-1, -1);
            else origWis = std::make_pair(m_data->intervalForVertexLocation(targetVert, origCi.min), m_data->intervalForVertexLocation(targetVert, origCi.max));
            auto isIn = [](int ind, const auto& inter)
            {
                return inter.first <= ind && ind <= inter.second;
            };

            auto setNewPointer = [this,&pathPointers,targetVert,usedEdge,&srcInterval,&newCi](int interval)
            {
                auto lowestReachable = std::max(newCi.min, m_data->WhiteIntervals[targetVert][interval].min);
                // Don't overwrite if other was better. Should this occur?
                if (lowestReachable >= pathPointers[targetVert][interval].lowestPointAtSelf) return;

                pathPointers[targetVert][interval] = PathPointer{
                    usedEdge,
                    srcInterval,
                    false,
                    lowestReachable
                };
            };

            auto& pPointers = pathPointers[targetVert];

            auto newWis = std::make_pair(m_data->intervalForVertexLocation(targetVert, newCi.min), m_data->intervalForVertexLocation(targetVert, newCi.max));

            // Catch the case where the first interval's reachable range is expanded.
            if(newWis.first == origWis.first && newCi.min < origCi.min)
            {
                setNewPointer(newWis.first);
            }
            else if(isIn(origWis.first, newWis) && m_data->WhiteIntervals[targetVert][origWis.first].min < origCi.min)
            {
                setNewPointer(origWis.first);
            }

            // Also catches empty original Ci
            if(newWis.first > origWis.second)
            {
                for(auto i = newWis.first; i <= newWis.second; ++i)
                {
                    setNewPointer(i);
                }
            }
            else
            {
                // Early out when the new range is fully covered
                if (newWis.first >= origWis.first && newWis.second <= origWis.second) return;

                // Iterate over all intervals in the union of the original and new Wis.
                auto minInter = std::min((int)newWis.first, origWis.first);
                auto maxInter = std::max((int)newWis.second, origWis.second);
                for(auto i = minInter; i <= maxInter; ++i)
                {
                    // Newly discovered reachable interval, so update.
                    if(isIn(i, newWis) && !isIn(i, origWis))
                    {
                        setNewPointer(i);
                    }
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

                // Initialize Cis with degenerate single point values
                Cis.at(i).min = m_data->WhiteIntervals.at(i).at(0).min;
                Cis.at(i).max = m_data->WhiteIntervals.at(i).at(0).min;

                // Leftmost coordinate should be white.
                Interval initialInterval = Cis.at(i);
                heap.insert(PrioQueueNode(initialInterval, i, 0),i);
            }
            allocatePathPointers(pathPointers);
        }
        /**
         * \brief Initializes the heap, Cis and path pointers for a regular sweepline application for full path finding in the graph.
         * The heap will contain intervals that have a white left endpoint. The Cis will be updated accordingly. The pathpointers are
         * resized to be able to contain all pointers for the intervals.
         * \param heap The heap to fill
         * \param Cis
         * \param pathPointers
         */
        void initializeDefaultStartFullIntervals(AltHeap& heap, std::vector<Interval>& Cis, std::vector<std::vector<PathPointer>>& pathPointers) const
        {
            // Number of vertices, represented in the data object
            const auto vNum = m_data->FDiWhiteIntervals.size();

            Cis.resize(vNum, Interval{}); //All are empty
            // Initialize with white starting points
            NT x = 0;
            for (auto i = 0; i < vNum; ++i)
            {
                if (!m_data->IntervalInclusions[i][0].leftIncluded) continue;
                // Initialize Cis.
                Cis.at(i).min = m_data->WhiteIntervals.at(i).at(0).min;
                Cis.at(i).max = m_data->WhiteIntervals.at(i).at(0).max;
                // Leftmost coordinate should be white.
                Interval initialInterval = Cis.at(i);
                heap.insert(PrioQueueNode(initialInterval, i, 0),i);
                
            }
            pathPointers.resize(vNum, {});
            for (std::size_t i = 0; i < vNum; ++i)
            {
                pathPointers.at(i).resize(m_data->WhiteIntervals[i].size(), {});
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
        void initializeDefaultRandomized(AltHeap& heap, std::vector<Interval>& Cis, std::vector<std::vector<PathPointer>>& pathPointers) const
        {
            // Number of vertices, represented in the data object
            const auto vNum = m_data->FDiWhiteIntervals.size();

            Cis.resize(vNum, Interval{}); //All are empty
            // Initialize with white starting points
            NT x = 0;
            std::vector<PrioQueueNode> toInsert;
            for (auto i = 0; i < vNum; ++i)
            {
                //if (!m_data->IntervalInclusions[i][0].leftIncluded) continue;
                //// Leftmost coordinate should be white.
                //Interval initialInterval(0, 0);

                //toInsert.push_back(PrioQueueNode(initialInterval, i, 0));
                //// Initialize Cis.
                //Cis.at(i).min = Cis[i].max = m_data->FDiWhiteIntervals.at(i).at(0).min;
                if (!m_data->IntervalInclusions[i][0].leftIncluded) continue;
                // Initialize Cis.
                Cis.at(i).min = m_data->WhiteIntervals.at(i).at(0).min;
                Cis.at(i).max = m_data->WhiteIntervals.at(i).at(0).max;
                // Leftmost coordinate should be white.
                Interval initialInterval = Cis.at(i);
                toInsert.push_back(PrioQueueNode(initialInterval, i, 0));
            }
            // Shuffle randomly
            LoopsLib::Helpers::RandomHelpers::shuffle(toInsert.begin(), toInsert.end());

            // Add to queue
            for(const auto& el: toInsert)
            {
                heap.insert(el,el.vertex);
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
            // Frechet graph data
            auto& data = *m_data;

            // Interval to insert
            auto interToQueue = data.WhiteIntervals.at(vertex).at(newIntervalIndex);

            // Don't queue if the interval is outside the reachable region
            if (Ci.max < interToQueue.min) return;

            // Only add if not empty (shouldn't happen really..)
            if (!interToQueue.isEmpty())
            {
                // Push to the queue.
                logger.trace("\t Interval ", newIntervalIndex, " [", interToQueue.min, ",", interToQueue.max, "] inserted ");
                prioQueue.insert(PrioQueueNode(interToQueue, vertex, newIntervalIndex), vertex);
            }
        }

        /**
         * \brief Acquires the left-right reachability pointers from the srcVertex to the targetVert at the current cell currCell.
         * takes into account the current sweepline value x, that is the actual lower location of the white interval at the source vertex.
         * Makes sure that the reachbility pointers point to an white interval, or returns an empty pointer interval otherwis.
         * 
         */ 
        bool getReachabilityPointers(std::size_t srcVertex, std::size_t targetVert, LoopsLib::NT x, LoopsLib::Geometry::Interval& lrPointer, LeftRightInclusion& lrInclusion,
            std::pair<int,int>& reachableWis){

            const auto& lrPointers = m_data->lrPointers.at(srcVertex);
            // Cell for the location, clamped to [0,polylinesize)
            const std::size_t currCell = m_data->positionToCell(x);

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
            
            // Integer indices of reachable white intervals
            reachableWis = std::make_pair((int)m_data->intervalForVertexLocation(targetVert, lrPointer.min), (int)m_data->intervalForVertexLocation(targetVert, lrPointer.max));
            logger.trace("\t\tChecking with lr pointer ", lrPointer.min, "-",lrPointer.max);

            // The lowest point in the lr pointer, either the pointer's own minimuim or the sweepline value
            const NT maxLower = std::max(lrPointer.min, x);
            logger.trace("\t\tLowest point considered before reparations ", maxLower);

            // Update accordingly, we may have ended up in a non-white interval
            if(!m_data->isWhitePoint(targetVert, maxLower))
            {
                int wiIndex = m_data->firstIntervalAbove(targetVert, maxLower);
                if (wiIndex == -1) {
                    logger.trace("\t\tBottom of src interval caused the vertex to be unreachable");
                    return false;
                }
                lrPointer.min = m_data->WhiteIntervals[targetVert][wiIndex].min;
                reachableWis.first = wiIndex;
                logger.trace("\t\tBottom of src interval caused the vertex to be unreachable");
            }
            else
            {
                // Update the minimum according to the current sweepline value.
                
                lrPointer.min = std::max(lrPointer.min, x);
                logger.trace("\t\tLrpointer bottom set to ",lrPointer.min);
            }

            // Finding a substitute may have caused the interval to actually become empty
            if (lrPointer.isEmpty()) {
                logger.trace("\t\t LR pointer empty");
                return false;
            }
            return true;
        }

        /**
         * \brief Test if the given edge path is within the epsilon distance as given in the local data object
         * Requires everything to be computed on the data object
         * Contains very verbose print statements
         * \param edgePath The edge path
         * \return Is the path reachable
         */
        bool testSinglePath(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& edgePath)
        {


            std::vector<int> vertIndices;
            vertIndices.push_back(m_data->m_view.indexForVertex(m_graph->edge(*edgePath.begin())->m_source->id()));
            for(const auto& eId: edgePath)
            {
                vertIndices.push_back(m_data->m_view.indexForVertex(m_graph->edge(eId)->m_sink->id()));
            }

            struct Node
            {
                NT lowPoint;
                int pathIndex;
                int intervalIndex;
                bool operator<(const Node& other) const
                {
                    if(lowPoint == other.lowPoint)
                    {
                        return pathIndex < other.pathIndex;
                    }
                    return lowPoint < other.lowPoint;
                }
            };

            auto getId = [this](const auto& pathInd, const auto& interval) -> int
            {
                return pathInd * m_data->polylineEdgeCount() + interval;
            };

            LoopsLib::DS::Heap<Node> queue;
            queue.insert(Node{ m_data->WhiteIntervals[vertIndices[0]].begin()->min, 0, 0 },getId(0,0));
            logger.info("Testing path");
            while(!queue.empty())
            {
                auto el = queue.extract();
                const auto node = el.value;
                logger.trace("Handling vertex ", node.pathIndex, " of path");
                Interval lrPointer;
                LeftRightInclusion lrInclusion;
                std::pair<int, int> reachableWis;

                const int srcVInd = vertIndices[node.pathIndex];
                const int nextInd = node.pathIndex + 1;
                const int nextVInd = vertIndices[node.pathIndex + 1];

                // Get reachability to next
                if(!getReachabilityPointers(srcVInd, nextVInd, node.lowPoint, lrPointer, lrInclusion, reachableWis))
                {
                    logger.trace("\t No reachability between vertices ", node.pathIndex, " and ", node.pathIndex + 1, " in provided path");
                    const auto& pntr = m_data->lrPointers[srcVInd][(std::size_t)node.lowPoint];
                    std::size_t i = 0;
                    for(const auto& el : m_data->WhiteIntervals[srcVInd])
                    {
                        logger.trace("\t\t WI:", i," range=",el.min,"-",el.max);
                        for(const auto& pntr: m_data->lrPointers[srcVInd][m_data->IntervalIdToCell[srcVInd][i]])
                        {
                            logger.trace("\t\t\tPntr: to ", pntr.first, " inter=", pntr.second.pointers.min, ":", pntr.second.pointers.max, ", wi reachable: ",
                                pntr.second.minWi, "-", pntr.second.maxWi);
                        }
                        
                        ++i;
                    }
                    continue;
                }
                logger.trace("lr pointer: ", lrPointer.min, "-", lrPointer.max);
                for(int i = reachableWis.first; i <= reachableWis.second; ++i)
                {
                    if(nextInd == vertIndices.size()-1 && i == m_data->WhiteIntervals[nextVInd].size()-1)
                    {
                        logger.trace("\t Reached end");
                        return true;
                    }

                    if(queue.containsId(getId(nextInd, i)))
                    {
                        logger.trace("\t Updating vert white interval combo for interval ", i);
                        auto prevNode = queue.dataForId(getId(nextInd, i));
                        // We need to replace
                        if(prevNode.lowPoint > std::max(lrPointer.min, m_data->WhiteIntervals[nextVInd][i].min))
                        {
                            const auto newLow = std::max(lrPointer.min, m_data->WhiteIntervals[nextVInd][i].min);
                            queue.updateKey(getId(nextInd,i), Node{ newLow,nextInd, i });
                            logger.trace("\t Expanded lower reach from " , prevNode.lowPoint, " to ", newLow);
                        }
                    }
                    else
                    {
                        const auto low = std::max(lrPointer.min, m_data->WhiteIntervals[nextVInd][i].min);
                        logger.trace("\t Inserting new node with lower ",low," for next index at white interval", i);
                        queue.insert(Node{low,nextInd, i}, getId(nextInd, i));
                    }
                }
            }
            return false;
        }

        /**
         * \brief Test if the given edge path is within the epsilon distance as given in the local data object
         * Requires everything to be computed on the data object
         * Is stripped of print statements.
         * \param edgePath The edge path
         * \return Is the path reachable
         */
        bool testSinglePathNoPrinting(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& edgePath)
        {


            std::vector<int> vertIndices;
            vertIndices.push_back(m_data->m_view.indexForVertex(m_graph->edge(*edgePath.begin())->m_source->id()));
            for (const auto& eId : edgePath)
            {
                vertIndices.push_back(m_data->m_view.indexForVertex(m_graph->edge(eId)->m_sink->id()));
            }

            struct Node
            {
                NT lowPoint;
                int pathIndex;
                int intervalIndex;
                bool operator<(const Node& other) const
                {
                    if (lowPoint == other.lowPoint)
                    {
                        return pathIndex < other.pathIndex;
                    }
                    return lowPoint < other.lowPoint;
                }
            };

            auto getId = [this](const auto& pathInd, const auto& interval) -> int
            {
                return pathInd * m_data->polylineEdgeCount() + interval;
            };

            LoopsLib::DS::Heap<Node> queue;
            queue.insert(Node{ m_data->WhiteIntervals[vertIndices[0]].begin()->min, 0, 0 }, getId(0, 0));
            while (!queue.empty())
            {
                auto el = queue.extract();
                const auto node = el.value;
                Interval lrPointer;
                LeftRightInclusion lrInclusion;
                std::pair<int, int> reachableWis;

                const int srcVInd = vertIndices[node.pathIndex];
                const int nextInd = node.pathIndex + 1;
                const int nextVInd = vertIndices[node.pathIndex + 1];

                // Get reachability to next
                if (!getReachabilityPointers(srcVInd, nextVInd, node.lowPoint, lrPointer, lrInclusion, reachableWis))
                {
                    continue;
                }
                for (int i = reachableWis.first; i <= reachableWis.second; ++i)
                {
                    if (nextInd == vertIndices.size() - 1 && i == m_data->WhiteIntervals[nextVInd].size() - 1)
                    {
                        return true;
                    }

                    if (queue.containsId(getId(nextInd, i)))
                    {
                        auto prevNode = queue.dataForId(getId(nextInd, i));
                        // We need to replace
                        if (prevNode.lowPoint > std::max(lrPointer.min, m_data->WhiteIntervals[nextVInd][i].min))
                        {
                            const auto newLow = std::max(lrPointer.min, m_data->WhiteIntervals[nextVInd][i].min);
                            queue.updateKey(getId(nextInd, i), Node{ newLow,nextInd, i });
                        }
                    }
                    else
                    {
                        const auto low = std::max(lrPointer.min, m_data->WhiteIntervals[nextVInd][i].min);
                        queue.insert(Node{ low,nextInd, i }, getId(nextInd, i));
                    }
                }
            }
            return false;
        }

        /**
         * \brief Applies the Alt et al. sweepline/dynamic program computation for Frechet mapmatching, adapted for directed graphs.
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
            std::vector<std::pair<int, int>> CiWis;
            CiWis.resize(Cis.size(), std::make_pair(-1, -1));
            for(int i = 0; i < Cis.size(); ++i)
            {
                if (!Cis[i].isEmpty()) {
                    CiWis[i] = std::make_pair(data.intervalForVertexLocation(i, Cis[i].min), data.intervalForVertexLocation(i, Cis[i].max));
                }
            }

            // That vertex-whiteinterval combos already processed.
            std::unordered_set<std::pair<LoopsLib::DS::BaseGraph::Id_t, int>,PairHasher<LoopsLib::DS::BaseGraph::Id_t, int>> processedIntervals;

            bool done = false;
            std::size_t totalEdgesSeen = 0;
            while (!prioQueue.empty())
            {
                /// Step 1)
                // Extract leftmost element
                const auto prioEntry = prioQueue.extract();
                const auto node = prioEntry.value;

                // Advance x, the current sweepline value
                if (node.inter.min < x)
                {
                    logger.error("Non-monotonous element in queue!");
                    continue;
                }
                x = node.inter.min;
                
                logger.trace("Vertex from prio queue: ", node.vertex, " (orig ",m_data->m_view.vertexByIndex(node.vertex)->id(),") with lower endpoint ", x, ", min-max:", node.inter.min," ",node.inter.max,". New prio size: ", prioQueue.size(),
                    ", associated Ci:", Cis[node.vertex].min, ",",Cis[node.vertex].max);

                // Update reachable interval of current vertex
                Cis[node.vertex].min = x;

                // Assign processed to path pointer of the current vertex-interval combo.
                {
                    const auto inter = data.intervalForVertexLocation(node.vertex, node.inter.min);
                    pathPointers[node.vertex][inter].wasProcessed = true;
                    //
                    auto pair = std::make_pair(node.vertex, node.interIndex);
                    processedIntervals.insert(pair);
                }

                // Check if the interval index does not concur. This is an error that shouldn't occur
                if(CiWis[node.vertex].first > node.interIndex || CiWis[node.vertex].second < node.interIndex)
                {
                    logger.error("Interval in node out of range:", node.interIndex, " should be in ", CiWis[node.vertex].first,"-", CiWis[node.vertex].second);
                    logger.error("Ci:", Cis[node.vertex].min, "-", Cis[node.vertex].max);
                    int cnt = 0;
                    for(const auto& el : data.WhiteIntervals[node.vertex])
                    {
                        logger.error("- Wi",cnt,":", el.min, "-", el.max);
                        ++cnt;
                    }
                    logger.flush();
                    slowError();
                }

                /// Step 2)
                // Insert next interval only if another interval exists
                if (node.interIndex + 1 < data.WhiteIntervals[node.vertex].size())
                {
                    insertNextInterval(prioQueue, node.vertex, node.interIndex+1, Cis[node.vertex]);
                }
                auto indentGuard = logger.addGlobalPrefix("\t");
                /// Step 3)
                // Update adjacent vertices to comply with new interval
                for (auto* e : m_data->m_view.outEdgesByIndex(node.vertex))
                {
                    // Stop when got done message by a callback
                    if (done) break;
                    ++totalEdgesSeen;
                    
                    // Index to identify target v with.
                    auto vId = m_data->m_view.indexForVertex(e->m_sink->id());

                    // Reachability data for edge e and current vertex/white interval
                    Interval lrPointer;
                    LeftRightInclusion lrInclusion;
                    std::pair<int, int> reachableWis;
                    logger.trace("Checking edge with endpoint ", vId);

                    // No suitable reachability pointers were found: we do not have to update, so continue with next edge
                    if(!getReachabilityPointers(node.vertex, vId, x, lrPointer, lrInclusion, reachableWis)){
                        continue;
                    }
                    auto secondIndentGuard = logger.addGlobalPrefix("\t");

                    logger.trace("Starting edge process");

                    // Apply the provided hook of 
                    auto edgeProcessIndent = logger.addGlobalPrefix("\t");
                    WenkSweeplineResult result = WenkSweeplineResult::ContinueIt;
                    if constexpr(detail::has_beforeEdgeProcess<IterationHooks>::value)
                    {
                        result = hooks.beforeEdgeProcess(e, node, lrPointer, data);
                    }
                    else if constexpr(detail::has_beforeEdgeProcessExtended<IterationHooks>::value)
                    {
                        result = hooks.beforeEdgeProcess(e, node, lrPointer, data,pathPointers);
                    }
                    switch(result)
                    {
                    case WenkSweeplineResult::Done:
                        return;
                    case WenkSweeplineResult::SkipRest:
                        done = true;
                        continue;
                    case WenkSweeplineResult::DoneAfterUpdates:
                        // Flag that we are done, but continue with path pointer updates before quitting
                        done = true;
                        break;
                    default:
                            break;
                    }
                    edgeProcessIndent.restore();
                    logger.trace("Edge process done");

                    // If the lr-pointer to the new vertex is fully included in the reachable consecutive chain,
                    // we don't need to update and are done.
                    if(!Cis[vId].isEmpty() && Cis[vId].contains(lrPointer.min) && Cis[vId].contains(lrPointer.max))
                    {
                        logger.trace("Lrpointer fully overlaps Ci: ", Cis[vId].min, ",", Cis[vId].max);
                        continue;
                    }
                    logger.trace("Processing");

                    // Update vertex Ci to sweepline
                    Cis[vId].min = std::max(Cis[vId].min, x);
                    // Potentially fix that we are in the middle of a forbidden interval.
                    if(!data.isWhitePoint(vId, Cis[vId].min))
                    {
                        auto ind = data.firstIntervalAbove(vId, Cis[vId].min);
                        // Make empty, since now we cannot reach anything anymore
                        if (ind == -1) Cis[vId] = Interval{};
                        else
                        {
                            Cis[vId].min = data.WhiteIntervals[vId][ind].min;
                        }
                    }

                    // Compute new reachable chain
                    Interval newCi;
                    newCi = lrPointer;

                    if (Cis[vId].isEmpty()) Cis[vId] = Interval{ };

                    if(newCi.max < Cis[vId].min && ! Cis[vId].isEmpty())
                    {
                        // Should be fine when contiguous?
                        logger.error("New lrpointer is below current Ci: lrPointer=",newCi, ", Ci=",Cis[vId], ": current processed vertex: ", node.vertex, "(orig=",
                            m_data->m_view.vertexByIndex(node.vertex)->id(),"), target vertex:", vId, "(orig=",
                            m_data->m_view.vertexByIndex(vId)->id(), ")");
                    }

                    // When Cis[vId] was empty or smaller than LR pointer, we completely replace it.
                    // Otherwise, update the bounds
                    if (!(Cis[vId].isEmpty() || Cis[vId].max < lrPointer.min))
                    {
                        newCi.min = std::min(newCi.min, Cis[vId].min);
                        newCi.max = std::max(newCi.max, Cis[vId].max);
                    }

                    // Determine intervals that intersect with the current consecutive chain for vId
                    CiWis[vId] = std::make_pair(data.intervalForVertexLocation(vId, newCi.min), data.intervalForVertexLocation(vId, newCi.max));
                    logger.trace("New ci: ", newCi.min, ",", newCi.max, " reaching wis: ", CiWis[vId].first, ",", CiWis[vId].second);

                    // Got a better reachability value in the lower part of the reachable chain or an entirely new chain,
                    // so update the priority queue
                    if(newCi.min < Cis[vId].min || newCi.min > Cis[vId].max)
                    {
                        const auto queueInterval = CiWis[vId].first;

                        auto intervalToQueue = data.WhiteIntervals[vId][queueInterval];
                        // Minimum value for interval
                        intervalToQueue.min = std::max(newCi.min, intervalToQueue.min);

                        PrioQueueNode prioNode(intervalToQueue, vId, queueInterval);
                        const bool didInsert = prioQueue.upsert(prioNode, vId);
                        if (didInsert)
                        {
                            logger.trace("Inserting in prioqueue with value: ", newCi.min, ", with vertex: ", vId);
                        }
                        else
                        {
                            logger.trace("Updating key in prioqueue to ", newCi.min, " for vert ", vId);
                            if (prioQueue.dataForId(vId).inter.min < intervalToQueue.min)
                            {
                                logger.error("Updating interval to higher value: original ", prioQueue.dataForId(vId).inter.min, " new: ", intervalToQueue.min);
                            }
                        }
                        assert(!prioQueue.dataForId(vId).inter.isEmpty());
                    }
                    else
                    {
                        logger.trace("No update, reachable C is now ", Cis[vId].min, "-", Cis[vId].max);
                    }

                    // Sanity checks
                    if(newCi.min > Cis[vId].max)
                    {
                        assert(data.WhiteIntervals[vId][data.CellToIntervalId[vId][m_data->positionToCell(lrPointer.max)]].max == lrPointer.max);

                        logger.trace("Updating full Ci for ", vId, ": from ", Cis[vId].min, ",", Cis[vId].max, " to ", newCi.min, ",", newCi.max);

                        // Get the indices of the intervals that are reachable via the lrpointer 
                        const int startInterval = data.intervalForVertexLocation(vId, newCi.min);
                        const int endInterval = data.intervalForVertexLocation(vId, newCi.max);

                        // Sanity check
                        if (startInterval != reachableWis.first || endInterval != reachableWis.second)
                        {
                            if (startInterval != 0 && endInterval != 0)
                            {
                                logger.error("Not same intervals for updating Ci via lrpointer that is above Ci: inters of lr =", startInterval, ",", endInterval, " vs ",
                                    reachableWis.first, ",", reachableWis.second);
                                logger.flush();
                                slowError();

                            }
                        }

                        //Overwrite error, should also not occur!
                        if (prioQueue.containsId(vId))
                        {
                            if (prioQueue.dataForId(vId).inter.max < newCi.min)
                            {
                                logger.error("Unprocessed entry in heap to be overwritten:",
                                    prioQueue.dataForId(vId).inter.min, ",",
                                    prioQueue.dataForId(vId).inter.max, " for inter ", prioQueue.dataForId(vId).interIndex,
                                    " Ci: ", newCi.min, " : ", newCi.max);
                                logger.error("WIs:");
                                for (const auto& wi : m_data->WhiteIntervals[vId])
                                {
                                    logger.error("Wi ", wi.min, ":", wi.max);
                                }
                            }
                        }
                        
                    }
                    else if(newCi.max > Cis[vId].max && !Cis[vId].isEmpty())
                    {
                        // Take diff with current
                        const int startInterval = data.intervalForVertexLocation(vId, Cis[vId].max) + 1;
                        const int endInterval = data.intervalForVertexLocation(vId, newCi.max);
                        logger.trace("Updating higherend Ci for ", vId, " from  ", Cis[vId].max, " to ", newCi.max, " inters: ", startInterval, "-", endInterval);
                        
                        // NOTE: THIS SHOULDN'T BE NECESSARY?
                        // May only happen on start vertices?
                        if (!prioQueue.containsId(vId) && Cis[vId].min != 0)
                        {
                            const auto newStart = data.intervalForVertexLocation(vId, newCi.min);
                            const auto maxEl = data.intervalForVertexLocation(vId, newCi.max);
                            bool wasInserted = false;
                            logger.warn("Adding new element to prioqueue for higher reachable interval");
                            for (std::size_t target = newStart; target <= maxEl; ++target)
                            {
                                // This should also actually never happen
                                if (data.WhiteIntervals[vId][target].max > newCi.max) break;

                                // Note that this should be at most 1 index apart, otherwise something weird is happening.
                                if (processedIntervals.find(std::make_pair(vId, target)) == processedIntervals.end())
                                {
                                    wasInserted = true;
                                    PrioQueueNode prioNode(data.WhiteIntervals[vId][target], vId, target);
                                    logger.trace("Inserting from higher in prioqueue with value: ", data.WhiteIntervals[vId][target].min, ", with vertex: ", vId);
                                    prioQueue.insert(prioNode, vId);
                                    //newCi.min = prioNode.inter.min;
                                    // Fix intervals
                                    //CiWis[vId].first = data.intervalForVertexLocation(vId, newCi.min);
                                    break;
                                }
                            }
                            if (!wasInserted)
                            {
                                logger.error("Extended highend without having any in the queue anymore! Vert=", vId,
                                    " new high:", newCi.max, ", lowest: ", newCi.min);
                            }
                        }
                    }

                    /// Step 4) update path pointers
                    updatePathPointers(vId, node.inter, e->id(), Cis[vId], newCi, pathPointers);

                    Cis[vId] = newCi;
                    logger.trace("New Ci for ", vId, ": ", Cis[vId]);
                }
                if(done)
                {
                    break;
                }
            }
            logger.info("Total edges seen:", totalEdgesSeen);
        }
    };
}
#endif