#ifndef LOOPSALGS_FLOW_MULTICOMMODITYMINCOSTFLOW_H
#define LOOPSALGS_FLOW_MULTICOMMODITYMINCOSTFLOW_H
#include "LoopsLib/DS/EmbeddedGraph.h"
#include "LoopsLib/DS/GraphView.h"
#include <ilcplex/ilocplexi.h>

namespace LoopsAlgs::Flow
{
    class MultiCommodityMinCostFlow
    {
    public:

        using NT = LoopsLib::MovetkGeometryKernel::NT;
    private:
        template<typename First, typename Second>
        struct PairLess
        {
            bool operator()(const std::pair<First,Second>& p0, const std::pair<First, Second>& p1) const
            {
                if (p0.first == p1.first) return p0.second < p1.second;
                return p0.first < p1.first;
            }
        };
        using CommodityEdgeToVarMap = std::map<std::pair<int, std::size_t>, std::size_t, PairLess<int, std::size_t>>;
        bool m_verbose = true;

        void setupSquaredDifferenceModel(const LoopsLib::DS::EmbeddedGraph& graph,
                        const std::vector<std::set<LoopsLib::DS::EmbeddedGraph::NodeIndex>>& sources,
                        const std::vector<std::set<LoopsLib::DS::EmbeddedGraph::NodeIndex>>& sinks,
                        const std::vector<NT>& field,
                        int commodityCount, IloEnv& env, IloModel& model, const IloNumVarArray& vars, const CommodityEdgeToVarMap& map);

        NT solve(IloEnv& env, IloModel& model, IloNumVarArray& vars, int commodityCount, std::size_t fieldSize, const CommodityEdgeToVarMap& map,
            std::vector<std::map<std::size_t, NT>>& commodityFlow);

    public:
        using Trajectory = std::vector<LoopsLib::MovetkGeometryKernel::MovetkPoint>;

        void setVerbose(bool value)
        {
            m_verbose = value;
        }

        template<typename CommodityHasEdgeFunc>
        NT computeSquaredDifferenceObjective(const LoopsLib::DS::EmbeddedGraph& graph, 
            const std::vector<std::set<LoopsLib::DS::EmbeddedGraph::NodeIndex>>& sources,
            const std::vector<std::set<LoopsLib::DS::EmbeddedGraph::NodeIndex>>& sinks,
            const std::vector<NT>& field,
            int commodityCount, CommodityHasEdgeFunc&& hasEdgeFunc, std::vector<std::map<std::size_t, NT>>& commodityFlow)
        {
            
            IloEnv env;
            IloModel model(env);
            IloNumVarArray commodityFlowVars(env);
            std::map<std::pair<int, std::size_t>, std::size_t,PairLess<int,std::size_t>> commodityEdgeToVar;
            std::size_t currVar = 0;
            for (auto* e : graph.edges())
            {
                const auto eId = e->id();
                for (int k = 0; k < commodityCount; ++k)
                {
                    // Check if edge is available for commodity
                    if(hasEdgeFunc(k,eId))
                    {
                        IloNumVar var(env);
                        commodityFlowVars.add(var);
                        commodityEdgeToVar[std::make_pair(k, eId)] = currVar;
                        ++currVar;
                    }
                }
            }

            setupSquaredDifferenceModel(graph, sources, sinks, field, commodityCount, env, model, commodityFlowVars, commodityEdgeToVar);

            return solve(env, model, commodityFlowVars, commodityCount, field.size(), commodityEdgeToVar, commodityFlow);
        }
    };
}
#endif