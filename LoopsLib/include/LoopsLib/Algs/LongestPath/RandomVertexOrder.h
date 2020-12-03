#ifndef LONGESTPATH_RANDOMVERTEXORDER_H
#define LONGESTPATH_RANDOMVERTEXORDER_H
#include <Eigen/Eigen>
#include <LoopsLib/DS/GraphHelpers/DagView.h>
#include <LoopsLib/DS/VertexOrder.h>
#include "ILongestPathAlg.h"

namespace LoopsLib::Algs::LongestPath{
    class RandomVertexOrder : public ILongestPath {
        int m_dagRetries = 30;
    public:

        RandomVertexOrder(DS::BaseGraph* graph)
            : ILongestPath("RandomVertexOrder", graph)
        {
        }

        void setNumberOfDagRetries(int value)
        {
            m_dagRetries = value;
        }
        int numberOfDagRetries() const
        {
            return m_dagRetries;
        }
    protected:
        BasisElement computeLongestPath(const FieldType& field, NT maxWeight) override
        {
            // Setup vertex order fixed locations
            std::vector<std::pair<int, int>> fixedLocations;
            fixedLocations.emplace_back(m_source, 0);
            fixedLocations.emplace_back(m_sink, m_graph->number_of_vertices() - 1);
            // Setup a vertex order
            DS::VertexOrder ord(m_graph, fixedLocations);

            // Find a dag ordering that remains connected
            int retries = m_dagRetries;
            while (!DS::isDagConnected(m_graph, ord.mapping(), m_source, m_sink) && retries > 0)
            {
                ord.newOrder();
                --retries;
            }
            if (retries == 0)
            {
                return BasisElement{};
            }
            auto mapping = ord.mapping();
            // Apply longest path on the dag.
            DS::GraphHelpers::DagView dagView(m_graph, mapping);

            // Longest path on dag.
            std::vector<double> dists(m_graph->number_of_vertices(), -std::numeric_limits<double>::max());
            std::vector<DS::BaseGraph::Edge*> pred(m_graph->number_of_vertices(), nullptr);
            dists[m_source] = 0;
            // Loop over vertices in the order
            for (auto id : ord.order())
            {
                for (auto* e : dagView.dagEdgesOut(m_graph->vertex(id)))
                {
                    if (dists[*e->m_sink] < dists[id] + field[e->id()])
                    {
                        dists[*e->m_sink] = dists[id] + field[e->id()];
                        pred[*e->m_sink] = e;
                    }
                }
            }
            assert(dists[m_sink] > -std::numeric_limits<double>::max());
            // Reconstruct path
            BasisElement path;
            DS::BaseGraph::Vertex* curr = m_graph->vertex(m_sink);
            while (curr->id() != m_source)
            {
                // Get predecessor edge
                auto* predE = pred[curr->id()];
                path.push_back(curr->id());
                // Recurse on source
                curr = predE->m_source;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
    public:
        Algs::BasisElement retry(const FieldType& field) override
        {
            return computeLongestPath(field, std::numeric_limits<double>::max());
        }
    };
}
#endif