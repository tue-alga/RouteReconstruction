#ifndef GRAPHALGS_CYCLEFINDER_H
#define GRAPHALGS_CYCLEFINDER_H
#include <LoopsLib/DS/OwnGraph.h>
#include "DFS.h"
#include <LoopsLib/Helpers/Iterators.h>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/depth_first_search.hpp>

namespace LoopsLib::GraphAlgs
{
    template<typename Graph_t>
    class FindCycles
    {
    public:
        using edge_descriptor = typename boost::graph_traits<Graph_t>::edge_descriptor;
        using vertex_descriptor = typename boost::graph_traits<Graph_t>::vertex_descriptor;
    private:
        bool m_verifyCirculation = false;
        bool m_verifyCycle = false;
        template<typename DfsGraph>
        struct DfsCycleVisitor
        {
            using vertex = typename boost::graph_traits<DfsGraph>::vertex_descriptor;

            struct DfsCycleVisitorData
            {
                std::map<vertex, std::pair<vertex,edge_descriptor>> pred;
                vertex_descriptor start;
                edge_descriptor backEdge;
                bool done = false;
                void reconstructCycle(const DfsGraph& graph, std::vector<edge_descriptor>& cycle)
                {
                    if (!done) return;
                    auto end = boost::target(backEdge, graph);
                    cycle.push_back(backEdge);
                    auto curr = boost::source(backEdge, graph);
                    while (curr != end)
                    {
                        assert(pred.find(curr) != pred.end());
                        auto prev = pred[curr];
                        cycle.push_back(prev.second);
                        curr = prev.first;
                    }
                    std::reverse(cycle.begin(), cycle.end());
                }
            };

            DfsCycleVisitorData* data = nullptr;

            DfsCycleVisitor(){}
            DfsCycleVisitor(DfsCycleVisitorData* data):data(data){}
            DfsCycleVisitor(const DfsCycleVisitor& other): data(other.data){}

            DfsCycleVisitor& operator=(const DfsCycleVisitor& other) {
                data = other.data;
                return *this;
            }

            void setData(DfsCycleVisitorData& data)
            {
                this->data = &data;
            }

            DfsCycleVisitorData createData() const
            {
                return DfsCycleVisitorData();
            }

            std::function<bool(vertex, const DfsGraph&)> end()
            {
                return [this](vertex v, const DfsGraph& graph)
                {
                    return data->done;
                };
            }
            

            //This is invoked on every vertex of the graph before the start of the graph search.
            void initialize_vertex(vertex_descriptor v, const DfsGraph& g)
            {
            }
            //This is invoked on the source vertex once before the start of the search.
            void start_vertex(vertex_descriptor v, const DfsGraph& g)
            {
                data->start = v;
            }
            //This is invoked when a vertex is encountered for the first time.
            void discover_vertex(vertex_descriptor v, const DfsGraph& g)
            {
            }
            //This is invoked on every out - edge of each vertex after it is discovered.
            void examine_edge(edge_descriptor e, const DfsGraph& g)
            {
                if (data->done) return;
                data->pred[boost::target(e,g)] = std::make_pair(boost::source(e, g), e);
            }
            // This is invoked on each edge as it becomes a member of the edges that form the search tree.
            void tree_edge(edge_descriptor e, const DfsGraph& g)
            {
            }
            // his is invoked on the back edges in the graph.For an undirected graph there is some ambiguity 
            // between tree edges and back edges since the edge(u, v) and (v, u) are the same edge, but both 
            // the tree_edge() and back_edge() functions will be invoked.One way to resolve this ambiguity is
            // to record the tree edges, and then disregard the back - edges that are already marked as tree edges.
            // An easy way to record tree edges is to record predecessors at the tree_edge event point.
            void back_edge(edge_descriptor e, const DfsGraph& g)
            {
                if (data->done) return;

                data->done = true;
                // Reconstruct the cycle
                data->pred[boost::target(e,g)] = std::make_pair(boost::source(e,g),e);
                data->backEdge = e;
            }
            // This is invoked on forward or cross edges in the graph.In an undirected graph this method is never called.
            void forward_or_cross_edge(edge_descriptor e, const DfsGraph& g)
            {
            }
            // This is invoked on each non - tree edge as well as on each tree edge after finish_vertex has been called on its target vertex.
            void finish_edge(edge_descriptor e, const DfsGraph& g)
            {
            }
            //This is invoked on vertex u after finish_vertex has been called for all the vertices in the DFS - tree rooted at vertex u.
            // If vertex u is a leaf in the DFS - tree, then the finish_vertex function is called on u after all the out - edges of u have been examined.
            void finish_vertex(vertex_descriptor v, const DfsGraph& g)
            {
            }
        };

    public:
        void setVerifyCirculation(bool value)
        {
            m_verifyCirculation = value;
        }
        void setVerifyCycle(bool value)
        {
            m_verifyCycle = value;
        }
        template<typename NT>
        bool isCirculation(const Graph_t& graph, const std::vector<NT>& edgeFlow, NT threshold)
        {
            using namespace LoopsLib::Helpers;
            bool ret = true;
            for(const auto& v : Iterators::PairIterable(boost::vertices(graph)))
            {
                NT total = 0.0;
                for(const auto& e : Iterators::PairIterable(boost::out_edges(v,graph)))
                {
                    total += edgeFlow[e];
                }
                for (const auto& e : Iterators::PairIterable(boost::in_edges(v, graph)))
                {
                    total -= edgeFlow[e];
                }
                if (std::abs(total) > threshold) {
                    std::cout << "[CycleFinder] Violation of " << std::abs(total) << " in circulation" << std::endl;
                    ret = false;
                }
            }
            return ret;
        }
                
        template<typename NT>
        void cyclesFromCirculation(const Graph_t& graph, const std::vector<NT>& edgeFlow, NT threshold, std::vector<std::vector<edge_descriptor>>& cycles,
            std::vector<NT>& cycleFlows)
        {
            /*if(m_verifyCirculation)
            {
                if(!isCirculation(graph,edgeFlow, threshold))
                {
                    std::cerr << "[CycleFinder] Detected a non-circulation flow in cyclesFromCirculation" << std::endl;
                }
            }*/

            using namespace LoopsLib::Helpers;
            struct WeightedEdge
            {
                edge_descriptor edge;
                NT weight;
                WeightedEdge()
                {
                    
                }
                WeightedEdge(edge_descriptor edge, NT weight):edge(edge),weight(weight){}
                bool operator<(const WeightedEdge& other) const
                {
                    return weight < other.weight || (weight == other.weight && edge < other.edge);
                }
            };
            using ColorMap_t = boost::associative_property_map< std::map<vertex_descriptor, boost::default_color_type> >;
            std::vector<NT> residual = edgeFlow;
            // Sort on weights, pick low weights first
            std::set<WeightedEdge> toProcess;
            // Setup initial to process set
            for(const auto& edge: Iterators::PairIterable(boost::edges(graph)))
            {
                if (edgeFlow[edge] < threshold) continue;

                toProcess.emplace(edge, edgeFlow[edge]);
            }
            struct EdgePred
            {
                NT threshold = 1e-8;
                std::vector<NT>* resid = nullptr;
                EdgePred(){}
                EdgePred(NT threshold, std::vector<NT>* resid):threshold(threshold), resid(resid){}
                EdgePred(const EdgePred& other):threshold(other.threshold),resid(other.resid){}
                EdgePred& operator=(const EdgePred& other)
                {
                    threshold = other.threshold;
                    resid = other.resid;
                    return *this;
                }

                bool operator()(const edge_descriptor& edge) const
                {
                    bool ret = (*resid)[edge] >= threshold;
                    return ret;
                }
            };
            boost::filtered_graph<Graph_t, EdgePred> view(graph, EdgePred(threshold, &residual));
            // Process the elements in the set.
            std::cout << "[CycleFinder] Starting search, process stack size: " << toProcess.size() << std::endl;
            while(!toProcess.empty())
            {
                auto el = *toProcess.begin();
                toProcess.erase(toProcess.begin());
                if (residual[el.edge] < threshold) continue;

                auto startVert = boost::source(el.edge, graph);

                // Find a cycle
                std::map<vertex_descriptor, boost::default_color_type> colors;
                ColorMap_t color_map(colors);
                DfsCycleVisitor<decltype(view)> visitor;
                auto cycleData = visitor.createData();
                visitor.setData(cycleData);
                //std::cout << "[CycleFinder] Running DFS" << std::endl;
                boost::depth_first_visit(view, startVert, visitor, color_map, visitor.end());
                //std::cout << "[CycleFinder] DFS done" << std::endl;
                // Not found?
                if(!cycleData.done)
                {
                    // Happens due to precision issues.
                    //std::cout << "Did not find cycle?" << std::endl;
                }
                else
                {
                    //std::cout << "[CycleFinder] Constructing cycle" << std::endl;
                    cycles.push_back({});
                    auto& cycle = cycles.back();
                    cycleData.reconstructCycle(view,cycle);
                    // Verify the cycle
                    if(m_verifyCycle)
                    {
                        // Verify first
                        if(boost::source(cycle.front(),graph) != boost::target(cycle.back(),graph))
                        {
                            std::cout << "[CycleFinder] ERROR: Edges do not connect!" << std::endl;
                        }
                        for(auto it = cycle.begin(),it2=cycle.begin()+1; it2 != cycle.end(); ++it,++it2)
                        {
                            if(boost::target(*it,graph) != boost::source(*it2,graph))
                            {
                                std::cout << "[CycleFinder] ERROR: Edges do not connect!" << std::endl;
                            }
                        }
                    }

                    auto eIt = std::min_element(cycle.begin(), cycle.end(), [&residual](edge_descriptor e0, edge_descriptor e1)
                    {
                        return residual[e0] < residual[e1];
                    });
                    NT minVal = residual[*eIt];
                    if(minVal <= threshold)
                    {
                        cycles.pop_back(); //TODO fix this later
                        continue;
                    }
                    // Add to output
                    cycleFlows.push_back(minVal);

                    

                    // Subtract
                    for (const auto& e : cycle) residual[e] -= minVal;
                }
            }
        }
    };

    class CycleFinder
    {
        void dfsCycleFromEdge(DS::BaseGraph* graph, DS::BaseGraph::Edge* e, std::vector<DS::BaseGraph::Edge*>& cycle)
        {
            for(auto res : DFSIterable(graph, e->m_source->id()))
            {
                cycle.push_back(res.edge());
                if(res.sinkSeen())
                {
                    return;
                }
            }
            cycle.clear();
        }
    public:
        void findCycle(DS::BaseGraph* graph, std::vector<DS::BaseGraph::Edge*>& cycle)
        {
            for(auto* e : graph->edges())
            {
                dfsCycleFromEdge(graph, e, cycle);
                if (cycle.size() > 0) return;
            }
        }
    };
}
#endif