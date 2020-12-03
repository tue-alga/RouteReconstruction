#ifndef LOOPSALGS_FLOW_MINCOSTFLOW_H
#define LOOPSALGS_FLOW_MINCOSTFLOW_H
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/directed_graph.hpp>
#include <ilcplex/ilocplexi.h>
#include <vector>
#include <LoopsLib/Helpers/Iterators.h>

namespace LoopsAlgs::Flow
{
    class MinCostFlow
    {
        bool m_verbose = false;
    public:
        template<typename Graph_t, typename NT>
        void compute(const Graph_t& graph,
            const std::set<typename boost::graph_traits<Graph_t>::vertex_descriptor>& sources,
            const std::set<typename boost::graph_traits<Graph_t>::vertex_descriptor>& sinks,
            const std::vector<NT>& field,
            std::vector<NT>& outFlow,
            std::unordered_map<typename boost::graph_traits<Graph_t>::vertex_descriptor, NT>& sourceSupply,
            std::unordered_map<typename boost::graph_traits<Graph_t>::vertex_descriptor, NT>& sinkDemand)
        {
            if(sources.empty())
            {
                std::cout << "Warning: no sources provided to MCF" << std::endl;
                return;
            }
            if(sinks.empty())
            {
                std::cout << "Warning: no sinks provided to MCF" << std::endl;
                return;
            }

            using namespace LoopsLib::Helpers;
            IloEnv env;
            IloModel model(env);

            // This may be prohibitively large
            // Organized per edge of the field. So first elements are the <commodityCount> 
            // coefficients for the first edge, etc.
            IloNumVarArray flowVars(env, field.size());
            for (auto i = 0; i < field.size(); ++i) {
                flowVars[i]= IloNumVar(env);
            }

            IloExpr squareDiff(env);
            for (auto e = 0; e < field.size(); ++e)
            {
                squareDiff += IloSquare(field[e] - flowVars[e]);
                model.add(flowVars[e] >= 0); //Non-negativity
            }


            // Add flow conservation
            for (auto vId : Iterators::PairIterable(boost::vertices(graph)))
            {
                const bool isSource = sources.find(vId) != sources.end();
                const bool isSink = sinks.find(vId) != sinks.end();
                
                // No reasonable flow conservation property
                if (isSource && isSink) continue;

                // Conserve flow for non-source/sink vertices
                IloExpr inSum(env), outSum(env);
                for (const auto& e : Iterators::PairIterable(boost::in_edges(vId, graph)))
                {
                    inSum += flowVars[e]; //TODO: fix for arbitrary graphs
                }
                for (const auto& e : Iterators::PairIterable((boost::out_edges(vId, graph))))
                {
                    outSum += flowVars[e];
                }

                // Ignore source/sink vertices for the commodity
                if (isSource)
                {
                    model.add(outSum >= inSum);
                }
                else if (isSink)
                {
                    model.add(inSum >= outSum);
                }
                else
                {
                    model.add(inSum == outSum);
                }
            }

            model.add(IloMinimize(env, squareDiff));

            // The solver
            IloCplex solver(model);

            if (!m_verbose) solver.setOut(env.getNullStream()); 

            try
            {
                bool success = solver.solve();
                if (!success)
                {
                    std::cout << "Cplex failed to solve" << std::endl;
                    // Cleanup memory
                    env.end();
                    return;
                }
                outFlow.resize(field.size(), 0);
                for (auto e = 0; e < field.size(); ++e)
                {
                    outFlow[e] = solver.getValue(flowVars[e]);
                }
                for(const auto& v: sources)
                {
                    if(sinks.find(v) != sinks.end())
                    {
                        NT total = 0;
                        for (const auto& e : Iterators::PairIterable(boost::out_edges(v, graph)))
                        {
                            total += solver.getValue(flowVars[e]);
                        }
                        for (const auto& e : Iterators::PairIterable(boost::in_edges(v, graph)))
                        {
                            total -= solver.getValue(flowVars[e]);
                        }
                        if(total > 0)
                        {
                            sourceSupply[v] = total;
                            sinkDemand[v] = 0;
                        }
                        else
                        {
                            sourceSupply[v] = 0;
                            sinkDemand[v] = -total;
                        }
                    }
                    else
                    {
                        NT total = 0;
                        for(const auto& e : Iterators::PairIterable(boost::out_edges(v,graph)))
                        {
                            total += solver.getValue(flowVars[e]);
                        }
                        for (const auto& e : Iterators::PairIterable(boost::in_edges(v, graph)))
                        {
                            total -= solver.getValue(flowVars[e]);
                        }
                        sourceSupply[v] = total;
                    }
                }
                for (const auto& v : sinks)
                {
                    if (sources.find(v) != sources.end()) continue;
                    NT total = 0;
                    for (const auto& e : Iterators::PairIterable(boost::in_edges(v, graph)))
                    {
                        total += solver.getValue(flowVars[e]);
                    }
                    for (const auto& e : Iterators::PairIterable(boost::out_edges(v, graph)))
                    {
                        total -= solver.getValue(flowVars[e]);
                    }
                    // Should be positive
                    sinkDemand[v] = total;
                }
            }
            catch (IloException& e)
            {
                std::cout << "Ilo Exception:" << e.getMessage() << std::endl;

                env.end();
                return;
            }
            env.end();
        }

        template<typename Graph_t, typename NT>
        void compute(const Graph_t& graph,
            const std::set<typename boost::graph_traits<Graph_t>::vertex_descriptor>& sources,
            const std::set<typename boost::graph_traits<Graph_t>::vertex_descriptor>& sinks,
            const std::vector<NT>& field,
            std::vector<NT>& outFlow)
        {
            std::unordered_map<typename boost::graph_traits<Graph_t>::vertex_descriptor, NT> dummy;
            compute(graph, sources, sinks, field, outFlow, dummy, dummy);
        }
    };
}

#endif