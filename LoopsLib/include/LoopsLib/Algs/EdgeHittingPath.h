#ifndef ALGS_EDGEHITTINGPATH_H
#define ALGS_EDGEHITTINGPATH_H
#include <LoopsLib/DS/OwnGraph.h>
#include <LoopsLib/DS/GraphHelpers/GraphModificationGuard.h>
#include <LoopsLib/Algs/MaxFlow/MaxFlowCplex.h>

namespace LoopsLib::Algs
{
    /**
     * \brief Algorithm for determining an edge hitting path by performing max flow
     */
    class EdgeHittingPath
    {
        DS::BaseGraph* m_graph;
        DS::BaseGraph::Id_t m_source;
        DS::BaseGraph::Id_t m_sink;

        bool m_verbose = true;

        void reconstructPath(const Eigen::VectorXi& presentEdges, DS::BaseGraph::Id_t edgeToHit, std::vector<DS::BaseGraph::Edge*>& path)
        {
            auto* curr = m_graph->vertex(m_source);
            std::unordered_set<int> seenVerts;
            seenVerts.insert(m_source);
            std::stack<std::pair<DS::BaseGraph::Vertex*, decltype(std::declval<DS::BaseGraph::Vertex>().m_outEdges.begin())>> processStack;
            processStack.push(std::make_pair(m_graph->vertex(m_source), m_graph->vertex(m_source)->m_outEdges.begin()));

            while(!processStack.empty())
            {
                auto pair = processStack.top();
                bool nextEdgeFound = false;
                auto it = pair.second;
                auto* vert = pair.first;
                for(; it != vert->m_outEdges.end(); ++it)
                {
                    auto* edge = *it;
                    if (seenVerts.find(edge->m_sink->id()) != seenVerts.end()) continue;
                    if (edge->id() != edgeToHit && presentEdges(edge->id()) != 1) continue;
                    seenVerts.insert(edge->m_sink->id());

                    auto nextIt = it;
                    ++nextIt;
                    processStack.top().second = nextIt;
                    processStack.push(std::make_pair(edge->m_sink, edge->m_sink->m_outEdges.begin()));
                    break;
                }
                if (processStack.top().first->id() == m_sink) break;
                if (it == vert->m_outEdges.end()) processStack.pop();
            }
            if (processStack.empty())
            {
                assert(false);
                throw std::runtime_error("Invalid path in EdgeHittingPath");
            }
            processStack.pop();
            while(!processStack.empty())
            {
                path.push_back(*std::prev(processStack.top().second));
                processStack.pop();
            }
            std::reverse(path.begin(), path.end());
        }
    public:
        void setVerbose(bool value)
        {
            m_verbose = value;
        }
        EdgeHittingPath(DS::BaseGraph* graph, DS::BaseGraph::Id_t source, DS::BaseGraph::Id_t sink):
        m_graph(graph),
        m_source(source),
        m_sink(sink)
        {
            
        }
        void setGraphData(DS::BaseGraph* graph, DS::BaseGraph::Id_t source, DS::BaseGraph::Id_t sink)
        {
            m_graph = graph;
            m_source = source;
            m_sink = sink;
        }

        void computePath(DS::BaseGraph::Id_t edgeToHit, std::vector<DS::BaseGraph::Edge*>& path)
        {
            // Needed later
            int initialVertexCount = m_graph->number_of_vertices();

            // Temporarily modify the graph. Changes revert when the guard goes out of scope
            DS::GraphHelpers::GraphModificationGuard graphMod(m_graph);

            auto* e = m_graph->edge(edgeToHit);
            // Create a new source and sink
            auto* newSource = graphMod.addVertex();
            auto* newSink = graphMod.addVertex();

            // Connect the end of the target with the source
            graphMod.addEdge(newSource, e->m_sink);
            graphMod.addEdge(newSource, m_graph->vertex(m_source));
            // Connect new sink
            auto* firstSinkEdge = graphMod.addEdge(e->m_source, newSink);
            auto* secondSinkEdge = graphMod.addEdge(m_graph->vertex(m_sink), newSink);

            // Remove the edge from the graph.
            graphMod.removeEdge(m_graph->edge(edgeToHit));

            // Create the maxflow algorithm
            Algs::MaxFlow::MaxFlowCplex maxFlowAlg(m_graph, newSource->id(), newSink->id());
            maxFlowAlg.setVerbose(m_verbose);

            Eigen::VectorXi capacities = Eigen::VectorXi::Constant(m_graph->number_of_edges(), 1);
            Eigen::VectorXi output;

            maxFlowAlg.computeIntegerMaxFlow(capacities, output);
            int totalFlow = output(firstSinkEdge->id()) + output(secondSinkEdge->id());

            // Need flow of exactly 2
            if (totalFlow < 2) return;

            reconstructPath(output, edgeToHit, path);
            graphMod.revertModifications();
            if(initialVertexCount != m_graph->number_of_vertices())
            {
                assert(false);
            }
        }
    };
}
#endif