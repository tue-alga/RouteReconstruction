#ifndef DS_VERTEX_ORDER_H
#define DS_VERTEX_ORDER_H
#include "OwnGraph.h"
#include <numeric>
#include <chrono>
#include <random>

namespace LoopsLib::DS
{
    class VertexOrder
    {
        // Contains order of vertex IDs
        std::vector<int> m_order;
        // The graph to apply to
        DS::BaseGraph* m_graph;
        // Fixed pairs of ID and index in order.
        std::vector<std::pair<int, int>> m_fixed;

        void constructOrder()
        {
            //Seed
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            m_order = std::vector<int>(m_graph->number_of_vertices(),0);
            std::iota(m_order.begin(), m_order.end(), 0);
            // Random shuffle
            std::shuffle(m_order.begin(), m_order.end(), std::default_random_engine(seed));
            // Rearrange the fixed elements
            for(auto pair : m_fixed)
            {
                // Find in order O(n) time
                auto it = std::find(m_order.begin(), m_order.end(), pair.first);
                // Swap values
                std::swap(m_order[pair.second], *it);
            }
        }
    public:
        VertexOrder(DS::BaseGraph* graph, const std::vector<std::pair<int,int>>& fixed = {}): m_graph(graph),
        m_fixed(fixed)
        {
            constructOrder();
        }
        void newOrder(){
            constructOrder();
        }
        /**
         * \brief Creates a mapping that maps a vertex ID to its location in the order.
         * \return The mapping
         */
        std::vector<int> mapping() const
        {
            std::vector<int> out(m_order.size(), -1);
            for(int i = 0; i < m_order.size(); ++i)
            {
                out[m_order[i]] = i;
            }
            return out;
        }
        const std::vector<int>& order() const
        {
            return m_order;
        }
        void generateDAGFromOrder(DS::BaseGraph& output)
        {
            if(m_order.empty())
            {
                throw std::runtime_error("Invalid call to generateDAGFromOrder(), call generateOrder() first!");
            }
            output = DS::BaseGraph(m_graph->number_of_vertices());
            // Seen verts previously. Any seen vert may connect to the current vertex
            // in question, but not the other way around.
            std::set<int> seenVerts;
            for(int i = 0; i < m_order.size(); ++i)
            {
                auto vert = m_graph->vertex(m_order[i]);
                // Only consider incoming edges of the vertex.
                for(auto e : vert->m_inEdges)
                {
                    // Only allow edges that originate from a seen vertex.
                    if(seenVerts.find(e->m_source->id()) == seenVerts.end())
                    {
                        continue;
                    }
                    output.addEdge(e->m_source->id(), vert->id());
                }
                seenVerts.insert(vert->id());
            }
        }
    };
}
#endif