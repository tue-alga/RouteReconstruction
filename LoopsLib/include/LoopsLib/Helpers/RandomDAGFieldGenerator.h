#ifndef HELPERS_RANDOM_DAG_FIELDGENERATOR_H
#define HELPERS_RANDOM_DAG_FIELDGENERATOR_H
#include <Eigen/Eigen>
#include <LoopsLib/DS/OwnGraph.h>
#include <LoopsLib/DS/VertexOrder.h>
#include <cassert>

namespace LoopsLib::Helpers {
    class RandomDAGFieldGenerator {
        // The base network
        DS::BaseGraph* m_graph;
        // Number of retries to generate DAG from network
        int m_genRetries = 30;
        // Source and sink
        long long m_source, m_sink;
    public:
        RandomDAGFieldGenerator(DS::BaseGraph* baseGraph, long long source, long long sink):
        m_graph(baseGraph),
        m_source(source),
        m_sink(sink)
        {
            
        }
        void generateField(int maxNumberOfPaths, double averageValue, double spread, Eigen::VectorXd& output)
        {
            output.setConstant(m_graph->number_of_edges(), 0.0);

            // Random generator for flow values
            std::default_random_engine eng;
            std::uniform_real_distribution<double> urd;

            DS::VertexOrder ord(m_graph);
            for(int i = 0; i < maxNumberOfPaths; ++i)
            {
                int j = 0;
                for(; j < m_genRetries; ++j)
                {
                    ord.newOrder();
                    std::vector<int> mapping = ord.mapping();
                    if(DS::isDagConnected(m_graph, mapping, m_source, m_sink))
                    {
                        break;
                    }
                }
                // Could not find proper DAG for the graph this time...
                if (j == m_genRetries) continue;

                // Find a path in the DAG.
                std::vector<int> mapping = ord.mapping();
                DS::DagView view(m_graph, mapping);
                // DFS to find path
                struct State
                {
                    DS::DagView::DagEdgeIterator currentInParent;
                    DS::DagView::DagEdgeIterator end;
                    bool undiscovered;
                    State(DS::DagView::DagEdgeIterator begin, DS::DagView::DagEdgeIterator end):
                    currentInParent(begin),
                    undiscovered(true),
                        end(end) {
                    }
                    bool atEnd() const
                    {
                        return currentInParent == end;
                    }
                };
                // The edges
                std::stack<State> edges;
                // Seen vertices
                std::set<long long> seen;
                auto* srcVert = m_graph->vertex(m_source);
                edges.push(State(view.outEdgesBegin(srcVert), view.outEdgesEnd(srcVert)));
                while(!edges.empty())
                {
                    auto& el = edges.top();
                    if(el.currentInParent->m_sink->id() == m_sink)
                    {
                        edges.push(el);
                        break;
                    }
                    if(el.undiscovered)
                    {
                        el.undiscovered = false;
                        auto* targetVert = el.currentInParent->m_sink;
                        if (seen.find(targetVert->id()) == seen.end())
                            edges.push(State(view.outEdgesBegin(targetVert), view.outEdgesEnd(targetVert)));
                        else
                            edges.pop();
                    }
                    else
                    {
                        for(; el.currentInParent != el.end; ++el.currentInParent)
                        {
                            auto* targetVert = el.currentInParent->m_sink;
                            if (seen.find(targetVert->id()) == seen.end())
                                edges.push(State(view.outEdgesBegin(targetVert), view.outEdgesEnd(targetVert)));
                            else
                                edges.pop();
                        }
                        if (el.atEnd())
                        {
                            edges.pop();
                        }
                    }
                }
                // Should always find a path
                assert(edges.size() > 0);
                // Apply flow value to found path.
                const double flowValue = averageValue + 2 * (urd(eng) - 1) * spread;
                while(!edges.empty())
                {
                    auto& el = edges.top();
                    output(el.currentInParent->id()) += flowValue;
                    edges.pop();
                }
            }
        }
    };
}
#endif