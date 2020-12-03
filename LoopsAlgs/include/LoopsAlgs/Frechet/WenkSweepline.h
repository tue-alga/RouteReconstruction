#ifndef HELPERS_WENK_SWEEPLINE_H
#define HELPERS_WENK_SWEEPLINE_H
#include "FrechetHelpers.h"
#include <LoopsLib/DS/Heap.h>
#include <LoopsLib/Helpers/Logger.h>

namespace LoopsAlgs::Frechet
{
    template<typename IterationHooks, typename LoggerType = IterationHooks>
    struct WenkSweepline {

        static_assert(detail::has_beforeEdgeProcess<IterationHooks>::value,
            "Hooks should have method beforeEdgeProcess(DS::BaseGraph::Edge*, const PrioQueueNode&, const Interval&, const StrongFrechetGraphData&)");
        
        //Logger.
        LoopsLib::Helpers::LogFactory<LoggerType> logger;
        // The graph
        Graph* m_graph;
        // Data for strong frechet for a specific path
        StrongFrechetGraphData* m_data;

        using WenkHeap = LoopsLib::DS::Heap<PrioQueueNode>;

        using PathPointers = std::vector<std::vector<PathPointer>>;

        /**
         * \brief Create the sweepline algorithm
         * \param graph Target base graph to apply the algorithm to
         * \param data Constructed strong Frechet data for the graph.
         */
        WenkSweepline(Graph& graph, StrongFrechetGraphData& data) :m_graph(&graph),m_data(&data) {}

        void updatePathPointers(Graph::Id_t targetVert, const PrioQueueNode& node, const Graph::Id_t usedEdge, int startInterval, int endInterval, PathPointers& pathPointers)
        {
            auto& data = *m_data;
            // Find out which intervals of the target are present in the current source node. We should avoid assigning these to avoid infinite loops.
            std::set<int> avoidIntervals;
            for (auto el : pathPointers[node.vertex])
            {
                if (m_graph->edge(el.edge)->m_source->id() == targetVert)
                {
                    int interId = data.CellToIntervalId.at(targetVert).at((int)el.srcInterval.min);
                    if(interId != -1)
                        avoidIntervals.insert(interId);
                }
            }

            // Update the path pointer(s) for modified intervals.
            for (int current = startInterval; current <= endInterval; ++current)
            {
                //TODO: HACK. Probably to fix this, we need a special case for the degenerate start intervals...
                //assert(!data.pathPointers[targetVert][current].wasProcessed);
                if (pathPointers.at(targetVert).at(current).wasProcessed) continue;

                if (avoidIntervals.find(current) != avoidIntervals.end()) continue;
                assert(!node.inter.isEmpty());
                pathPointers[targetVert][current] = PathPointer{ usedEdge, node.inter,false };
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
        void initializeDefaultStart(WenkHeap& heap, std::vector<Interval>& Cis, std::vector<std::vector<PathPointer>>& pathPointers) const
        {
            // Number of vertices, represented in the data object
            const auto vNum = m_data->FDiWhiteIntervals.size();

            Cis.resize(vNum, Interval{}); //All are empty
            // Initialize with white starting points
            NT x = 0;
            for (auto i = 0; i < vNum; ++i)
            {
                if (!m_data->IntervalInclusions[i][0].leftIncluded) continue;
                // Leftmost coordinate should be white.
                Interval initialInterval(0, 0);
                heap.insert(PrioQueueNode(initialInterval, i, 0));
                // Initialize Cis.
                Cis.at(i).min = Cis[i].max = m_data->FDiWhiteIntervals.at(i).at(0).min;
            }
            pathPointers.resize(vNum, {});
            for(std::size_t i = 0 ; i < vNum; ++i)
            {
                pathPointers.at(i).resize(m_data->WhiteIntervals[i].size(),{});
            }
        }

        /**
         * \brief Insert the next interval in the priority queue
         * \param prioQueue The priority queue
         * \param vertex The vertex for which to insert the new interval
         * \param newIntervalIndex The index of the new interval (number of the interval in FD_i)
         * \param Ci The current 'reachable' interval. New interval should be clipped to the maximum of this interval.
         */
        inline void insertNextInterval(WenkHeap& prioQueue, Graph::Id_t vertex, int newIntervalIndex, const Interval& Ci)
        {
            auto& data = *m_data;

            auto interToQueue = data.WhiteIntervals.at(vertex).at(newIntervalIndex);
            interToQueue.max = std::min(Ci.max, interToQueue.max);

            if (!interToQueue.isEmpty())
            {
                // Push to the queue.
                logger.deep("\t Interval ", newIntervalIndex, " [", interToQueue.min, ",", interToQueue.max, "] inserted ");
                prioQueue.insert(PrioQueueNode(interToQueue, vertex, newIntervalIndex), vertex);
            }
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
        void apply(WenkHeap& prioQueue, std::vector<Interval>& Cis, IterationHooks& hooks, std::vector<std::vector<PathPointer>>& pathPointers)
        {
            auto& data = *m_data;
            decltype(prioQueue.peak().value.inter.min) x = 0;

            while (!prioQueue.empty())
            {
                const auto prioEntry = prioQueue.extract();
                const auto node = prioEntry.value;
                // Advance x
                x = node.inter.min;
                logger.info("Vertex from prio queue: ", node.vertex, " with val ", x, ", min-max:", node.inter.min," ",node.inter.max);

                // Assign processed
                pathPointers[node.vertex][data.CellToIntervalId[node.vertex][(int)node.inter.min]].wasProcessed = true;

                // Insert next interval only if another interval exists
                if (node.interIndex + 1 < data.WhiteIntervals[node.vertex].size())
                {
                    insertNextInterval(prioQueue, node.vertex, node.interIndex+1,  Cis[node.vertex]);
                }

                // Decrement reachable region a bit 
                Cis[node.vertex].min -= 0.001;

                assert(data.CellToIntervalId[node.vertex][(int)node.inter.min] != -1);
                assert((int)node.inter.max == data.polyline.size() || data.CellToIntervalId[node.vertex][(int)node.inter.max] != -1);

                // Shortcuts to left/right pointers for vertex
                const auto& lrPointers = data.lrPointers.at(node.vertex);

                // The cell associated with the minimum of the current interval.
                auto targetCell = (int)node.inter.min;

                // Update adjacent vertices to comply with new interval
                for (auto* e : m_data->m_view.outEdgesByIndex(node.vertex))
                {
                    // Index to identify e with.
                    auto vId = m_data->m_view.indexForVertex(e->m_sink->id());

                    // Ignore edges for which no path exists.
                    // Assume that both lpointer and rpointer are invalid in that case!
                    if (lrPointers[targetCell].find(vId) == lrPointers[targetCell].end())
                    {
                        logger.info("\tEdge with endpoint ", vId, " not reachable from vert ", node.vertex, " and  cell ", targetCell);
                        continue;
                    }
                    logger.info("\tChecking edge with endpoint ", vId);

                    // Get left/right pointer and inclusion
                    Interval lrPointer = lrPointers[targetCell].at(vId).pointers;
                    LeftRightInclusion lrInclusion = lrPointers[targetCell].at(vId).inclusion;
                    // Update the minimum according to the current sweepline value.
                    lrPointer.min = std::max(lrPointer.min, x);

                    // The pointer range became empty: continue. TODO: this should actually not happen?
                    if (lrPointer.isEmpty()) {
                        logger.info("\t\t LR pointer empty");
                        continue;
                    }

                    // Fixup pointer when x caused it to end up in a black interval
                    {
                        auto lrInter = data.CellToIntervalId[vId][(int)lrPointer.min];
                        if (data.WhiteIntervals[vId][lrInter].max < lrPointer.min)
                        {
                            assert(data.WhiteIntervals[vId].size() > lrInter + 1);
                            lrPointer.min = data.WhiteIntervals[vId][lrInter + 1].min;
                        }
                    }

                    //Check rpointer is at end of free surface, then we are done.
                    auto result = hooks.beforeEdgeProcess(e, node, lrPointer, data);
                    switch(result)
                    {
                    case WenkSweeplineResult::Done:
                        return;
                    case WenkSweeplineResult::SkipRest:
                        continue;
                    default:
                            break;
                    }

                    //Fixup Cis lowerend
                    if (Cis[vId].contains(x))
                    {
                        auto currCiInterval = data.CellToIntervalId[vId][(int)Cis[vId].min];
                        // Findout interval 
                        auto newCiInterval = data.CellToIntervalId[vId][(int)x];
                        // We ended up with a different interval: either a black interval or a higher interval.
                        if (currCiInterval != newCiInterval)
                        {
                            // Interval is a black interval
                            if (newCiInterval == -1)
                            {
                                int i = currCiInterval + 1;
                                for (; i < data.WhiteIntervals[vId].size(); ++i)
                                {
                                    if (data.WhiteIntervals[vId][i].max > x) break;
                                }
                                assert(i != data.WhiteIntervals[vId].size());
                                Cis[vId].min = data.WhiteIntervals[vId][i].min;
                            }
                            // Higher interval: take the new minimum as the end of Cis.
                            else
                            {
                                Cis[vId].min = std::max(data.WhiteIntervals[vId][newCiInterval].min, x);
                            }
                        }
                        // x is exactly in a black interval: move the minimum to the first white interval
                        // If none exists, something is wrong with the left/right pointers
                        else if (x > data.WhiteIntervals[vId][currCiInterval].max)
                        {
                            int i = currCiInterval + 1;
                            for (; i < data.WhiteIntervals[vId].size(); ++i)
                            {
                                if (data.WhiteIntervals[vId][i].max > x) break;
                            }
                            assert(i != data.WhiteIntervals[vId].size());
                            Cis[vId].min = std::max(x, data.WhiteIntervals[vId][i].min);
                        }
                        else
                        {
                            Cis[vId].min = x;
                        }
                    }

                    bool updateQInterval = false;
                    // Replace Ci with interval when r(C_i) < l_{i,j}(I)
                    // This only happens when r(C_i) < x, since otherwise,
                    // the feasible chain of I should overlap with C_i, by convexity
                    // of the cells.
                    if (Cis[vId].max < lrPointer.min)
                    {
                        if (Cis[vId].isEmpty())
                        {
                            logger.info("\t\t Updating Ci that was empty");
                        }
                        assert(data.WhiteIntervals[vId][data.CellToIntervalId[vId][(int)lrPointer.max]].max == lrPointer.max);

                        /*assert(Cis[vId].isEmpty() || (
                            data.CellToIntervalId[vId][(int)Cis[vId].max] != data.CellToIntervalId[vId][(int)lrPointer.min]
                            ));*/

                        Cis[vId] = lrPointer;
                        // Update lower end to be at start of an interval.
                        {

                            int lowerStart = data.CellToIntervalId[vId][(int)lrPointer.min];
                            if (lowerStart == -1)
                            {
                                int i = (int)lrPointer.min;
                                for (; i < data.polyline.size(); ++i)
                                {
                                    if (data.CellToIntervalId[vId][i] != -1) break;
                                }
                                Cis[vId].min = data.WhiteIntervals[vId][data.CellToIntervalId[vId][i]].min;
                                logger.info("\t\t Fixing non white-interval lowerend");
                            }
                            else if (data.WhiteIntervals[vId][lowerStart].max < lrPointer.min)
                            {
                                logger.info("\t\t Fixing non white-interval lowerend");
                                for (; lowerStart < data.WhiteIntervals[vId].size(); ++lowerStart)
                                {
                                    if (data.WhiteIntervals[vId][lowerStart].max > lrPointer.min)
                                    {
                                        Cis[vId].min = std::max(data.WhiteIntervals[vId][lowerStart].min, lrPointer.min);
                                        break;
                                    }
                                }
                            }
                        }

                        logger.info("\t\t Updating full Ci for ", vId, ": ", Cis[vId].min, ",", Cis[vId].max);
                        updateQInterval = true;
                        //Update path pointers
                        int start = (int)Cis[vId].min;
                        // Get associated interval
                        auto startInterval = data.CellToIntervalId[vId][start];
                        // Update affected intervals
                        auto endInterval = data.CellToIntervalId[vId][(int)lrPointer.max];
                        // Update the path pointer(s) for modified intervals.
                        updatePathPointers(vId, node, e->id(), startInterval, endInterval, pathPointers);
                        logger.info("\t\t Pp: ", node.vertex, "->", vId, "[", startInterval, "]");
                    }
                    // Update the min value of the reachable chain
                    else if (Cis[vId].min > lrPointer.min) // Lpointer guaranteed to point to an interval!
                    {
                        // Update path pointers
                        int start = (int)lrPointer.min;
                        int localInterval = data.CellToIntervalId[vId][(int)Cis[vId].min];

                        bool needUpdate = true;
                        // Check if the new region actually contributes something
                        assert(data.CellToIntervalId[vId][start] != -1);
                        if (data.WhiteIntervals[vId][start].max < lrPointer.min)
                        {
                            for (; start <= localInterval; ++start)
                            {
                                if (data.WhiteIntervals[vId][start].max > lrPointer.min)break;
                            }
                            needUpdate = Cis[vId].min > data.WhiteIntervals[vId][start].min;
                        }
                        if (needUpdate)
                        {
                            auto startInterval = data.CellToIntervalId[vId][start];
                            auto endInterval = data.CellToIntervalId[vId][(int)Cis[vId].min];
                            // Update all affected intervals
                            updatePathPointers(vId, node, e->id(), startInterval, endInterval, pathPointers);

                            logger.info("\t\t Pp: ", node.vertex, "->", vId, "[", start, "]");

                            Cis[vId].min = lrPointer.min;

                            //Update interval in prio queue.
                            updateQInterval = true;
                            logger.info("\t\t Updating lowerend Ci for ", vId, ": ", Cis[vId].min);
                        }

                    }
                    // Update the max of the reachable chain
                    if (lrPointer.max > Cis[vId].max)
                    {
                        // Take diff with current
                        int start = (int)Cis[vId].max;
                        assert(
                            (Cis[vId].max == 0 && Cis[vId].min <= 0) ||
                            std::abs(data.WhiteIntervals[vId][data.CellToIntervalId[vId][start]].max - Cis[vId].max) < 0.0000001);
                        auto startInterval = data.CellToIntervalId[vId][start] + 1;
                        if (startInterval < data.IntervalIdToCell[vId].size())
                        {
                            auto endInterval = data.CellToIntervalId[vId][(int)lrPointer.max];
                            // Update the path pointer(s) for modified intervals.
                            updatePathPointers(vId, node, e->id(), startInterval, endInterval, pathPointers);
                            logger.info("\t\t Pp: ", node.vertex, "->", vId, "[", start, "]");
                            logger.info("\t\t Updating higherend Ci for ", vId, " from  ", Cis[vId].max, " to ", lrPointer.max);
                            Cis[vId].max = lrPointer.max;
                            if (!prioQueue.containsId(vId))
                            {
                                PrioQueueNode prioNode(data.WhiteIntervals[vId][startInterval], vId, startInterval);
                                prioQueue.insert(prioNode, vId);
                            }
                        }
                    }


                    // Check if new intervals were added or changed, in which case we set the path pointer to the
                    // last popped interval of the prio queue.

                    if (updateQInterval)
                    {
                        //TODO fix this, this is incorrect.
                        auto queueInterval = data.CellToIntervalId[vId][(int)Cis[vId].min];

                        auto intervalToQueue = data.WhiteIntervals[vId][queueInterval];
                        intervalToQueue.min = std::max(Cis[vId].min, intervalToQueue.min);

                        PrioQueueNode prioNode(intervalToQueue, vId, queueInterval);
                        assert(!Cis[vId].isEmpty());
                        // Find associated interval in prio queue.
                        if (prioQueue.containsId(vId))
                        {
                            logger.info("\t\t Updating key in prioqueue to ", Cis[vId].min, " for vert ", vId);
                            prioQueue.updateKey(vId, prioNode);
                        }
                        else
                        {
                            logger.info("\t\t Inserting in prioqueue with value: ", Cis[vId].min, ", with vertex: ", vId);
                            prioQueue.insert(prioNode, vId);
                        }
                        assert(!prioQueue.dataForId(vId).inter.isEmpty());
                    }
                }
            }
        }
    };
}
#endif