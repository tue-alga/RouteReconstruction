#ifndef GRAPHALGS_WIDESTPATH_H
#define GRAPHALGS_WIDESTPATH_H
#include <LoopsLib/DS/Heap.h>
#include <LoopsLib/DS/OwnGraph.h>
#include <LoopsLib/Helpers/Iterators.h>
#include <boost/graph/directed_graph.hpp>

namespace LoopsLib::GraphAlgs
{
    template<typename Graph_t,typename VectorLikeType>
    struct EdgeIdToWeightMapper
    {
        using Edge_t = typename boost::graph_traits<Graph_t>::edge_descriptor;
        using Value_t = std::decay_t<decltype(std::declval< VectorLikeType>()[std::declval<Edge_t>()->id()])>;
        VectorLikeType data;
        EdgeIdToWeightMapper(VectorLikeType data):data(data){}

        void setData(VectorLikeType data)
        {
            this->data = data;
        }

        Value_t operator()(Edge_t edge) const
        {
            return data[edge->id()];
        }
    };
    template<typename Graph_t, typename VectorLikeType>
    struct EdgeToWeightMapper
    {
        using Edge_t = typename boost::graph_traits<Graph_t>::edge_descriptor;
        using Value_t = std::decay_t<decltype(std::declval< VectorLikeType>()[std::declval<Edge_t>()])>;
        VectorLikeType data;
        EdgeToWeightMapper(VectorLikeType data) :data(data) {}

        void setData(VectorLikeType data)
        {
            this->data = data;
        }

        Value_t operator()(Edge_t edge) const
        {
            return data[edge];
        }
    };

    template<typename Graph_t, typename WeightProvider>
    class WidestPath
    {
    public:
        using Edge_t = typename boost::graph_traits<Graph_t>::edge_descriptor;
        using Vertex_t = typename boost::graph_traits<Graph_t>::vertex_descriptor;
        using Value_t = decltype(std::declval<WeightProvider>()(std::declval<Edge_t>()));
        using OutPath_t = std::vector<Edge_t>;
    private:
        Value_t m_negInf;
    public:
        WidestPath(Value_t negInf = std::numeric_limits<Value_t>::lowest()):m_negInf(negInf)
        {
            
        }

        void compute(const Graph_t& graph, WeightProvider& weightProvider, Vertex_t src,Vertex_t sink, std::vector<Edge_t>& path, Value_t& outValue)
        {
            using namespace LoopsLib::Helpers;

            DS::Heap <Value_t, Vertex_t,std::greater<>> priorityQueue;
            std::unordered_map<Vertex_t, Value_t> dist;
            std::unordered_map<Vertex_t, Edge_t> parent;
            auto isInf = [&dist](Vertex_t vert)
            {
                return dist.find(vert) == dist.end();
            };
            auto negInf = m_negInf;
            auto getDistance = [&isInf, negInf, &dist](Vertex_t vert) -> Value_t
            {
                return isInf(vert) ? negInf : dist[vert];
            };
            auto setDistance = [&dist](Vertex_t vert, Value_t distance)
            {
                dist[vert] = distance;
            };
            dist[src] = -negInf; // Positive infinity, so that weight of edge is always chosen.
            priorityQueue.insert(dist[src], src);
            while(!priorityQueue.empty())
            {
                auto top = priorityQueue.extract();
                //std::cout << "Handling vertex" << std::endl;
                for(auto edge : Iterators::PairIterable(boost::out_edges(top.id, graph)))
                {
                    //std::cout << "--Checking vert" << std::endl;
                    Value_t currDistance = getDistance(top.id);
                    Value_t weight = weightProvider(edge);
                    auto distance = std::max(getDistance(boost::target(edge,graph)), std::min(currDistance, weight));

                    // Relaxation of edge and adding into Priority Queue 
                    if (distance > getDistance(boost::target(edge,graph))) {
                        //std::cout << "--Adding new" << std::endl;

                        // Updating bottle-neck distance 
                        setDistance(boost::target(edge, graph), distance);

                        // Keep track of parent
                        parent[boost::target(edge, graph)] = edge;

                        // Add the vertex to be explored further, or update in heap if needed.
                        priorityQueue.upsert(distance, boost::target(edge, graph));
                    }
                }
            }
            if (parent.find(sink) == parent.end()) return;

            outValue = dist[sink];
            Vertex_t curr = sink;
            while(curr != src)
            {
                path.push_back(parent[curr]);
                curr = boost::source(parent[curr], graph);
            }
            std::reverse(path.begin(), path.end());
        }
    };
}
#endif