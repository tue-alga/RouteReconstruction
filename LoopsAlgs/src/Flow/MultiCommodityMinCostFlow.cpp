#include <LoopsAlgs/Flow/MultiCommodityMinCostFlow.h>

void LoopsAlgs::Flow::MultiCommodityMinCostFlow::setupSquaredDifferenceModel(const LoopsLib::DS::EmbeddedGraph& graph,
                                                            const std::vector<std::set<LoopsLib::DS::EmbeddedGraph::
                                                                NodeIndex>>& sources,
                                                            const std::vector<std::set<LoopsLib::DS::EmbeddedGraph::
                                                                NodeIndex>>& sinks, const std::vector<NT>& field,
                                                            int commodityCount, IloEnv& env, IloModel& model,
                                                            const IloNumVarArray& commodityFlowVars
    , const CommodityEdgeToVarMap& map)
{
    // Setup square difference goal
    IloExpr squareDiff(env);
    // Field size is same as number of edges
    for (auto e = 0; e < field.size(); ++e)
    {
        IloExpr edgeExpr(env);
        for (int k = 0; k < commodityCount; ++k)
        {
            auto pos = map.find(std::make_pair(k, e));
            if (pos == map.end()) continue;
            model.add(commodityFlowVars[pos->second] >= 0); // Non-negative flow
            edgeExpr += commodityFlowVars[pos->second];
        }
        squareDiff += IloSquare(field[e] - edgeExpr);
    }
    model.add(IloMinimize(env, squareDiff));

    std::cout << "Commodities: " << commodityCount << std::endl;

    // Add flow conservation
    for (int k = 0; k < commodityCount; ++k)
    {
        const auto& commSources = sources[k];
        const auto& commSinks = sinks[k];
        // Run over all vertices in the graph
        for (auto vId = 0; vId < graph.number_of_vertices(); ++vId)
        {
            IloExpr inSum(env), outSum(env);
            bool hasIn = false;
            // Check if in and out flow is available.
            for (auto* e : graph.vertex(vId)->m_inEdges)
            {
                auto pos = map.find(std::make_pair(k, e->id()));
                if (pos == map.end()) continue;
                inSum += commodityFlowVars[pos->second];
                hasIn = true;
            }
            bool hasOut = false;
            for (auto* e : graph.vertex(vId)->m_outEdges)
            {
                auto pos = map.find(std::make_pair(k, e->id()));
                if (pos == map.end()) continue;
                outSum += commodityFlowVars[pos->second];
                hasOut = true;
            }
            if (!hasIn && !hasOut) {
                // Cleanup Cplex memory?
                inSum.end();
                outSum.end();
                continue;
            }
            
            if (commSources.find(vId) != commSources.end())
            {
                // Cannot say anything here.
                if (commSinks.find(vId) != commSinks.end()) continue;
                model.add(outSum >= inSum);
            }
            else if (commSinks.find(vId) != commSinks.end())
            {
                model.add(inSum >= outSum);
            }
            else
            {
                model.add(inSum == outSum);
            }
        }
    }
}

LoopsAlgs::Flow::MultiCommodityMinCostFlow::NT LoopsAlgs::Flow::MultiCommodityMinCostFlow::solve(IloEnv& env,
                                                                                                 IloModel& model,
                                                                                                 IloNumVarArray& vars,
                                                                                                 int commodityCount,
                                                                                                 std::size_t fieldSize, const CommodityEdgeToVarMap& map,
                                                                                                 std::vector<std::map
                                                                                                     <std::size_t,NT>>&
                                                                                                 commodityFlow)
{
    // The solver
    IloCplex solver(model);

    if (!m_verbose)
    {
        solver.setOut(env.getNullStream());
    }

    try
    {
        const bool success = solver.solve();
        if (!success)
        {
            std::cout << "Cplex failed to solve" << std::endl;
            // Cleanup memory
            env.end();
            return -1.0;
        }
        commodityFlow.resize(commodityCount, {});
        for(const auto& pair: map)
        {
            auto k = pair.first;
            commodityFlow[k.first][k.second] = solver.getValue(vars[pair.second]);
        }
    }
    catch (IloException& e)
    {
        std::cout << "Ilo Exception:" << e.getMessage() << std::endl;

        env.end();
        return -1.0;
    }
    catch(std::exception& e)
    {
        std::cout << "Exception: " << e.what() << std::endl;
        env.end();
        return -1.0;
    }
    NT retVal = solver.getObjValue();
    env.end();

    return retVal;
}
