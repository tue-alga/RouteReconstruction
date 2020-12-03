#ifndef GRAPHALGS_SHORTESTPATH_H
#define GRAPHALGS_SHORTESTPATH_H
#include <LoopsLib/DS/Heap.h>
#include <LoopsLib/DS/OwnGraph.h>

namespace LoopsLib::GraphAlgs
{
    /**
     * \brief 
     * Not thread safe! Don't run algorithms with the same underlying instance.
     * \tparam WeightProvider 
     */
    template<typename WeightProvider>
    class WeightedShortestPath
    {

    public:
        using Weight_t = std::decay_t<decltype(std::declval<WeightProvider&>()[0])>;
        /*using DistanceMap_t = std::vector<Weight_t>; 
        using PredecessorMap_t = std::vector<DS::BaseGraph::Edge*>;*/
        using DistanceMap_t = std::unordered_map<DS::BaseGraph::Id_t, Weight_t>;
        using PredecessorMap_t = std::unordered_map<DS::BaseGraph::Id_t, DS::BaseGraph::Edge*>;

    private:
        Weight_t m_upperBound = std::numeric_limits<Weight_t>::max();
        mutable Weight_t m_lastDist = std::numeric_limits<Weight_t>::max();
    public:
        Weight_t lastSpDist() const { return m_lastDist; }
        WeightedShortestPath(){}
        WeightedShortestPath(Weight_t upperBound): m_upperBound(upperBound){}

        void computeSingleSourceData(const DS::BaseGraph* graph, WeightProvider& provider, DS::BaseGraph::Id_t sourceVertex,
            std::unordered_map<DS::BaseGraph::Id_t, Weight_t>& distanceMap, std::unordered_map<DS::BaseGraph::Id_t, DS::BaseGraph::Edge*>& predecessorPerVertex, Weight_t maxDist) const
        {
            auto isInf = [&distanceMap](DS::BaseGraph::Id_t vId)
            {
                return distanceMap.find(vId) == distanceMap.end();
            };
            LoopsLib::DS::Heap<Weight_t> prioQueue;

            std::unordered_set<DS::BaseGraph::Id_t> visited;
            auto wasVisited = [&visited](DS::BaseGraph::Id_t vId)
            {
                return visited.find(vId) != visited.end();
            };
            auto markVisited = [&visited](DS::BaseGraph::Id_t vId)
            {
                visited.insert(vId);
            };

            prioQueue.insert((Weight_t)0, sourceVertex);
            distanceMap[sourceVertex] = 0;
            while (!prioQueue.empty())
            {
                auto heapNode = prioQueue.extract();
                // Mark visited
                markVisited(heapNode.id);

                for (auto* e : graph->vertex(heapNode.id)->m_outEdges)
                {
                    // Skip vertices that were already visited
                    if (wasVisited(*e->m_sink)) continue;

                    auto newDist = distanceMap[heapNode.id] + provider[e->id()];
                    if ((isInf(*e->m_sink) || newDist < distanceMap[*e->m_sink]) && newDist < maxDist)
                    {
                        distanceMap[*e->m_sink] = newDist;
                        if (prioQueue.containsId(*e->m_sink))
                            prioQueue.updateKey(*e->m_sink, newDist);
                        else
                            prioQueue.insert(newDist, *e->m_sink);
                        predecessorPerVertex[*e->m_sink] = e;
                    }
                }
            }
        }

        void computeSingleSourceData(const DS::BaseGraph* graph, WeightProvider& provider, DS::BaseGraph::Id_t sourceVertex,
            std::unordered_map<DS::BaseGraph::Id_t, Weight_t>& distanceMap, std::unordered_map<DS::BaseGraph::Id_t,DS::BaseGraph::Edge*>& predecessorPerVertex) const
        {
            computeSingleSourceData(graph, provider, sourceVertex, distanceMap, predecessorPerVertex, m_upperBound);
        }

        void computeSingleSourceData(const DS::BaseGraph* graph, WeightProvider& provider, DS::BaseGraph::Id_t sourceVertex,
            std::vector<Weight_t>& distanceMap, std::vector<DS::BaseGraph::Edge*>& predecessorPerVertex) const
        {
            distanceMap.assign(graph->number_of_vertices(), std::numeric_limits<Weight_t>::max());
            distanceMap[sourceVertex] = 0;
            DS::Heap<Weight_t> prioQueue;

            std::vector<bool> visited;
            visited.assign(graph->number_of_vertices(), false);
            predecessorPerVertex.assign(graph->number_of_vertices(), nullptr);

            prioQueue.insert((Weight_t)0, sourceVertex);
            while (!prioQueue.empty())
            {
                auto heapNode = prioQueue.extract();
                // Mark visited
                visited[heapNode.id] = true;

                for (auto* e : graph->vertex(heapNode.id)->m_outEdges)
                {
                    // Skip vertices that were already visited
                    if (visited[*e->m_sink]) continue;

                    auto newDist = distanceMap[heapNode.id] + provider[e->id()];
                    if ((isInf(*e->m_sink) || newDist < distanceMap[*e->m_sink]) && newDist < m_upperBound)
                    {
                        distanceMap[*e->m_sink] = newDist;
                        if (prioQueue.containsId(*e->m_sink))
                            prioQueue.updateKey(*e->m_sink, newDist);
                        else
                            prioQueue.insert(newDist, *e->m_sink);
                        predecessorPerVertex[*e->m_sink] = e;
                    }
                }
            }
        }
        
        
        
        /**
         * \brief Simple Dijkstra with binary heap implementation to compute shortest path with positive weights
         * \param graph The graph
         * \param provider The weight provider. Should support operator[](index)
         * \param sourceVertex The source vertex
         * \param sinkVertex The sink vertex
         * \param path Output path. Will be empty when no path is found.
         */
        bool computeShortestPath(const DS::BaseGraph* graph, const WeightProvider& provider, DS::BaseGraph::Id_t sourceVertex, DS::BaseGraph::Id_t sinkVertex, std::vector<DS::BaseGraph::Edge*>& path) const
        {
            if(sourceVertex == sinkVertex)
            {
                m_lastDist = 0;
                return true;
            }
            return computeShortestPath(graph, provider, sourceVertex, sinkVertex, path, m_upperBound);
        }
        /**
         * \brief Simple Dijkstra with binary heap implementation to compute shortest path with positive weights
         * \param graph The graph
         * \param provider The weight provider. Should support operator[](index)
         * \param sourceVertex The source vertex
         * \param sinkVertex The sink vertex
         * \param path Output path. Will be empty when no path is found.
         */
        bool computeShortestPath(const DS::BaseGraph* graph, const WeightProvider& provider, DS::BaseGraph::Id_t sourceVertex, DS::BaseGraph::Id_t sinkVertex, std::vector<DS::BaseGraph::Edge*>& path,
            NT upperBound) const
        {
            if (sourceVertex == sinkVertex)
            {
                m_lastDist = 0;
                return true;
            }
            std::unordered_map<DS::BaseGraph::Id_t, Weight_t> distanceMap;
            std::unordered_map<DS::BaseGraph::Id_t, DS::BaseGraph::Edge*> predecessorPerVertex;

            auto isInf = [&distanceMap](DS::BaseGraph::Id_t vId)
            {
                return distanceMap.find(vId) == distanceMap.end();
            };
            DS::Heap<Weight_t> prioQueue;

            std::unordered_set<DS::BaseGraph::Id_t> visited;
            auto wasVisited = [&visited](DS::BaseGraph::Id_t vId)
            {
                return visited.find(vId) != visited.end();
            };
            auto markVisited = [&visited](DS::BaseGraph::Id_t vId)
            {
                visited.insert(vId);
            };

            prioQueue.insert((Weight_t)0, sourceVertex);
            distanceMap[sourceVertex] = 0;
            bool done = false;
            while (!prioQueue.empty() && !done)
            {
                auto heapNode = prioQueue.extract();
                // Mark visited
                markVisited(heapNode.id);
                if (heapNode.id == sinkVertex) break;

                for (auto* e : graph->vertex(heapNode.id)->m_outEdges)
                {
                    // Skip vertices that were already visited
                    if (wasVisited(*e->m_sink)) continue;

                    auto newDist = distanceMap[heapNode.id] + provider[e->id()];
                    if ( (isInf(*e->m_sink) || newDist < distanceMap[*e->m_sink]) && newDist < upperBound)
                    {
                        distanceMap[*e->m_sink] = newDist;
                        if (prioQueue.containsId(*e->m_sink))
                            prioQueue.updateKey(*e->m_sink, newDist);
                        else
                            prioQueue.insert(newDist, *e->m_sink);
                        predecessorPerVertex[*e->m_sink] = e;
                    }
                }
            }
            if (isInf(sinkVertex)) return false;

            m_lastDist = distanceMap[sinkVertex];

            // Reconstruct path
            auto currVert = sinkVertex;
            while (true)
            {
                if (currVert == sourceVertex) break;
                path.push_back(predecessorPerVertex[currVert]);
                currVert = predecessorPerVertex[currVert]->m_source->id();
            }
            // Reverse the path to be in the correct direction
            std::reverse(path.begin(), path.end());
            return true;
        }
    };

    /**
     * \brief Computes the shortest path solely based on number of edges in the path.
     */
    class LeastEdgesPath
    {
    public:
        /**
         * \brief Simple Dijkstra with binary heap implementation to compute shortest path with positive weights
         * \param graph The graph
         * \param provider The weight provider. Should support operator[](index)
         * \param sourceVertex The source vertex
         * \param sinkVertex The sink vertex
         * \param path Output path. Will be empty when no path is found.
         */
        void computeShortestPath(const DS::BaseGraph* graph, DS::BaseGraph::Id_t sourceVertex, DS::BaseGraph::Id_t sinkVertex, std::vector<DS::BaseGraph::Edge*>& path)
        {
            struct WeightProvider
            {
                DS::BaseGraph::Id_t operator[](std::size_t index) const
                {
                    return 1;
                }
            };
            WeightedShortestPath<WeightProvider> wsp;
            WeightProvider provider;
            wsp.computeShortestPath(graph, provider , sourceVertex, sinkVertex, path);
        }
    };
    namespace templated
    {
        template<typename GraphType, typename WeightProvider>
        class WeightedShortestPath
        {

        public:
            using Id_t = typename GraphType::Id_t;
            using EdgeHandle = typename GraphType::EdgeHandle;
            using Weight_t = std::decay_t<decltype(std::declval<WeightProvider&>()[std::declval<Id_t>()])>;
            /*using DistanceMap_t = std::vector<Weight_t>;
            using PredecessorMap_t = std::vector<DS::BaseGraph::Edge*>;*/
            using DistanceMap_t = std::unordered_map<Id_t, Weight_t>;
            using PredecessorMap_t = std::unordered_map<Id_t, EdgeHandle>;

        private:
            Weight_t m_upperBound = std::numeric_limits<Weight_t>::max();
            mutable Weight_t m_lastDist = std::numeric_limits<Weight_t>::max();
        public:
            Weight_t lastSpDist() const { return m_lastDist; }
            WeightedShortestPath() {}
            WeightedShortestPath(Weight_t upperBound) : m_upperBound(upperBound) {}

            void computeSingleSourceData(const GraphType* graph, WeightProvider& provider, Id_t sourceVertex,
                DistanceMap_t& distanceMap, PredecessorMap_t& predecessorPerVertex, Weight_t maxDist) const
            {
                auto isInf = [&distanceMap](Id_t vId)
                {
                    return distanceMap.find(vId) == distanceMap.end();
                };
                LoopsLib::DS::Heap<Weight_t,Id_t> prioQueue;

                std::unordered_set<Id_t> visited;
                auto wasVisited = [&visited](Id_t vId)
                {
                    return visited.find(vId) != visited.end();
                };
                auto markVisited = [&visited](Id_t vId)
                {
                    visited.insert(vId);
                };

                prioQueue.insert((Weight_t)0, sourceVertex);
                distanceMap[sourceVertex] = 0;
                while (!prioQueue.empty())
                {
                    auto heapNode = prioQueue.extract();
                    // Mark visited
                    markVisited(heapNode.id);

                    for (auto* e : graph->vertex(heapNode.id)->outEdges())
                    {
                        const auto targetVId = e->sink()->id();
                        // Skip vertices that were already visited
                        if (wasVisited(targetVId)) continue;


                        auto newDist = distanceMap[heapNode.id] + provider[e->id()];
                        if ((isInf(targetVId) || newDist < distanceMap[targetVId]) && newDist < maxDist)
                        {
                            distanceMap[targetVId] = newDist;
                            if (prioQueue.containsId(targetVId))
                                prioQueue.updateKey(targetVId, newDist);
                            else
                                prioQueue.insert(newDist, targetVId);
                            predecessorPerVertex[targetVId] = e;
                        }
                    }
                }
            }

            void computeSingleSourceData(const GraphType* graph, WeightProvider& provider, Id_t sourceVertex,
                DistanceMap_t& distanceMap, PredecessorMap_t& predecessorPerVertex) const
            {
                computeSingleSourceData(graph, provider, sourceVertex, distanceMap, predecessorPerVertex, m_upperBound);
            }

            /**
             * \brief Simple Dijkstra with binary heap implementation to compute shortest path with positive weights
             * \param graph The graph
             * \param provider The weight provider. Should support operator[](index)
             * \param sourceVertex The source vertex
             * \param sinkVertex The sink vertex
             * \param path Output path. Will be empty when no path is found.
             */
            bool computeShortestPath(const GraphType* graph, const WeightProvider& provider, Id_t sourceVertex, Id_t sinkVertex, std::vector<EdgeHandle>& path) const
            {
                return computeShortestPath(graph, provider, sourceVertex, sinkVertex, path, m_upperBound);
            }
            /**
             * \brief Simple Dijkstra with binary heap implementation to compute shortest path with positive weights
             * \param graph The graph
             * \param provider The weight provider. Should support operator[](index)
             * \param sourceVertex The source vertex
             * \param sinkVertex The sink vertex
             * \param path Output path. Will be empty when no path is found.
             */
            bool computeShortestPath(const GraphType* graph, const WeightProvider& provider, Id_t sourceVertex, Id_t sinkVertex, std::vector<EdgeHandle>& path,
                NT upperBound) const
            {
                DistanceMap_t distanceMap;
                PredecessorMap_t predecessorPerVertex;

                auto isInf = [&distanceMap](Id_t vId)
                {
                    return distanceMap.find(vId) == distanceMap.end();
                };
                DS::Heap<Weight_t,Id_t> prioQueue;

                std::unordered_set<Id_t> visited;
                auto wasVisited = [&visited](Id_t vId)
                {
                    return visited.find(vId) != visited.end();
                };
                auto markVisited = [&visited](Id_t vId)
                {
                    visited.insert(vId);
                };

                prioQueue.insert((Weight_t)0, sourceVertex);
                distanceMap[sourceVertex] = 0;
                bool done = false;
                while (!prioQueue.empty() && !done)
                {
                    auto heapNode = prioQueue.extract();
                    // Mark visited
                    markVisited(heapNode.id);
                    if (heapNode.id == sinkVertex) break;

                    for (auto* e : graph->vertex(heapNode.id)->outEdges())
                    {
                        const auto targetVId = e->sink()->id();
                        // Skip vertices that were already visited
                        if (wasVisited(targetVId)) continue;
                        auto newDist = distanceMap[heapNode.id] + provider[e->id()];
                        if ((isInf(targetVId) || newDist < distanceMap[targetVId]) && newDist < upperBound)
                        {
                            distanceMap[targetVId] = newDist;
                            if (prioQueue.containsId(targetVId))
                                prioQueue.updateKey(targetVId, newDist);
                            else
                                prioQueue.insert(newDist, targetVId);
                            predecessorPerVertex[targetVId] = e;
                        }
                    }
                }
                if (isInf(sinkVertex)) return false;

                m_lastDist = distanceMap[sinkVertex];

                // Reconstruct path
                auto currVert = sinkVertex;
                while (true)
                {
                    if (currVert == sourceVertex) break;
                    path.push_back(predecessorPerVertex[currVert]);
                    currVert = predecessorPerVertex[currVert]->source()->id();
                }
                // Reverse the path to be in the correct direction
                std::reverse(path.begin(), path.end());
                return true;
            }
        };

    }
}
#endif