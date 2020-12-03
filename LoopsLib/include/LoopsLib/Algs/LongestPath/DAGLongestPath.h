#ifndef ALGS_DAG_LONGEST_PATH_H
#define ALGS_DAG_LONGEST_PATH_H
#include <LoopsLib/DS/BaseGraph.h>

namespace LoopsLib::Algs
{
    template<typename EdgeData, typename VertexData>
    class DAGLongestPath
    {
        void topologicalSort(const DS::OwnGraph<EdgeData, VertexData>& graph,int start, std::vector<int>& ids)
        {
            std::stack<int> current;
            current.push(start);
            std::stack<int> outputStack;
            std::unordered_set<int> visited;
            while(!current.empty())
            {
                auto el = current.top();
                current.pop();
                if(visited.find(el) != visited.end())
                {
                    outputStack.push(el);
                }
                else
                {
                    visited.insert(el);
                    current.push(el);
                    for(auto e: graph.vertex(el)->m_outEdges)
                    {
                        current.push(e->m_sink->id());
                    }
                }
            }
            ids.resize(outputStack.size(),0);
            // Index in output vector
            int ind = 0;
            while(!outputStack.empty())
            {
                ids[ind] = outputStack.top();
                ++ind;
                outputStack.pop();
            }
        }
    public:
        void longestPath(const DS::OwnGraph<EdgeData,VertexData>& graph, const std::vector<double>& weights, int source, int sink)
        {
            // Build topological order
            std::vector<int> topoOrder;
            topologicalSort(graph, source, topoOrder);
            // Apply dynamic program on topo order and weights.
            std::vector<double> dists;
            std::vector<int> pred;
            dists.resize(graph.number_of_vertics(), -std::numeric_limits<double>::max());
            pred.resize(graph.number_of_vertics(), -1);
            dists[source] = 0;
            for(auto el : topoOrder)
            {
                for(auto e : graph.vertex(el)->m_outEdges)
                {
                    auto target = e->m_sink->id();
                    if(dists[target] < dists[el] + weights[e->id()])
                    {
                        dists[target] = dists[el] + weights[e->id()];
                        pred[target] = el;
                    }
                }
            }
            // Retrieve path from sink
            std::vector<int> reversePath;
            reversePath.push_back(sink);
            auto curr = pred[sink];
            while (curr != source)
            {
                reversePath.push_back(curr);
                curr = pred[curr];
            }
            reversePath.push_back(source);
            std::reverse(reversePath.begin(), reversePath.end());
            // Return the path
;        }
    };
}
#endif