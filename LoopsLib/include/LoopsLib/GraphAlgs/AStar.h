#ifndef GRAPHALGS_ASTAR_H
#define GRAPHALGS_ASTAR_H
#include <LoopsLib/DS/Heap.h>
#include <LoopsLib/DS/OwnGraph.h>

namespace LoopsLib::GraphAlgs
{
    /**
     * \brief
     * Not thread safe! Don't run algorithms with the same underlying instance.
     * \tparam WeightProvider
     */
    template<typename WeightProvider,typename HeuristicProvider>
    class AStar
    {
    public:
        using Weight_t = std::decay_t<decltype(std::declval<WeightProvider&>()[0])>;
        static_assert(std::is_same_v < Weight_t, decltype(std::declval<HeuristicProvider&>()(0,0))>, "Heuristic and weight provider should provide same type");
        /*using DistanceMap_t = std::vector<Weight_t>;
        using PredecessorMap_t = std::vector<DS::BaseGraph::Edge*>;*/
        using DistanceMap_t = std::unordered_map<DS::BaseGraph::Id_t, Weight_t>;
        using PredecessorMap_t = std::unordered_map<DS::BaseGraph::Id_t, DS::BaseGraph::Edge*>;

    private:
        Weight_t m_upperBound = std::numeric_limits<Weight_t>::max();
        mutable Weight_t m_lastDist = std::numeric_limits<Weight_t>::max();
        const DS::BaseGraph* m_graph = nullptr;
    public:
        Weight_t lastSpDist() const { return m_lastDist; }
        AStar() {}
        AStar(const DS::BaseGraph* graph):m_graph(graph){}
        AStar(Weight_t upperBound) : m_upperBound(upperBound) {}
        AStar(const DS::BaseGraph* graph, Weight_t upperBound) : m_graph(graph),m_upperBound(upperBound) {}
        void setGraph(const DS::BaseGraph* graph) { m_graph = graph; }

        /**
         * \brief Simple Dijkstra with binary heap implementation to compute shortest path with positive weights
         * \param graph The graph
         * \param provider The weight provider. Should support operator[](index)
         * \param sourceVertex The source vertex
         * \param sinkVertex The sink vertex
         * \param path Output path. Will be empty when no path is found.
         */
        bool computeShortestPath(const WeightProvider& provider, const HeuristicProvider& heurProvider, DS::BaseGraph::Id_t sourceVertex, DS::BaseGraph::Id_t sinkVertex, std::vector<DS::BaseGraph::Edge*>& path) const
        {
            return computeShortestPath(m_graph, provider, heurProvider, sourceVertex, sinkVertex, path, m_upperBound);
        }

        /**
         * \brief Simple Dijkstra with binary heap implementation to compute shortest path with positive weights
         * \param graph The graph
         * \param provider The weight provider. Should support operator[](index)
         * \param sourceVertex The source vertex
         * \param sinkVertex The sink vertex
         * \param path Output path. Will be empty when no path is found.
         */
        bool computeShortestPath(const DS::BaseGraph* graph, const WeightProvider& provider, const HeuristicProvider& heurProvider, DS::BaseGraph::Id_t sourceVertex, DS::BaseGraph::Id_t sinkVertex, std::vector<DS::BaseGraph::Edge*>& path) const
        {
            return computeShortestPath(graph, provider, heurProvider, sourceVertex, sinkVertex, path, m_upperBound);
        }
        /**
         * \brief Simple Dijkstra with binary heap implementation to compute shortest path with positive weights
         * \param graph The graph
         * \param provider The weight provider. Should support operator[](index)
         * \param heurProvider The heuristic provider. Should support operator()(currentVert, sinkVertex). Should always underestimate the actual distance!
         * \param sourceVertex The source vertex
         * \param sinkVertex The sink vertex
         * \param path Output path. Will be empty when no path is found.
         */
        bool computeShortestPath(const DS::BaseGraph* graph, const WeightProvider& provider, const HeuristicProvider& heurProvider, DS::BaseGraph::Id_t sourceVertex, DS::BaseGraph::Id_t sinkVertex, std::vector<DS::BaseGraph::Edge*>& path,
            Weight_t upperBound) const
        {
            // The actual distance map
            std::unordered_map<DS::BaseGraph::Id_t, Weight_t> distanceMap;
            std::unordered_map<DS::BaseGraph::Id_t, DS::BaseGraph::Edge*> predecessorPerVertex;

            struct Node
            {
                Weight_t currDist;
                Weight_t heurDist;
                Weight_t totalWeight() const
                {
                    return currDist + heurDist;
                }
                bool operator<(const Node& other) const
                {
                    return totalWeight() < other.totalWeight();
                }
            };

            auto isInf = [&distanceMap](DS::BaseGraph::Id_t vId)
            {
                return distanceMap.find(vId) == distanceMap.end();
            };
            DS::Heap<Node> prioQueue;

            std::unordered_set<DS::BaseGraph::Id_t> visited;
            auto wasVisited = [&visited](DS::BaseGraph::Id_t vId)
            {
                return visited.find(vId) != visited.end();
            };
            auto markVisited = [&visited](DS::BaseGraph::Id_t vId)
            {
                visited.insert(vId);
            };

            prioQueue.insert({ (Weight_t)0, heurProvider(sourceVertex,sinkVertex)}, sourceVertex);
            distanceMap[sourceVertex] = 0;
            bool done = false;
            while (!prioQueue.empty() && !done)
            {
                auto heapNode = prioQueue.extract();

                // Early out when the underestimated distance is already worse than the known 
                // distance to the sink.
                if(!isInf(sinkVertex) && heapNode.value.totalWeight() >= distanceMap[sinkVertex])
                {
                    break;
                }
                // Mark visited
                markVisited(heapNode.id);
                if (heapNode.id == sinkVertex) break;

                for (auto* e : graph->vertex(heapNode.id)->m_outEdges)
                {
                    // Skip vertices that were already visited
                    if (wasVisited(*e->m_sink)) continue;

                    auto newDist = distanceMap[heapNode.id] + provider[e->id()];
                    auto hDist = heurProvider(*e->m_sink, sinkVertex);
                    // Ignore when we cannot meet the upperbound, assuming the heuristic.
                    if (newDist + hDist > upperBound) continue;

                    if (isInf(*e->m_sink) || newDist < distanceMap[*e->m_sink])
                    {
                        distanceMap[*e->m_sink] = newDist;
                        Node node = { newDist, hDist };
                        if (prioQueue.containsId(*e->m_sink))
                            prioQueue.updateKey(*e->m_sink, node);
                        else
                            prioQueue.insert(node, *e->m_sink);
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
}
#endif