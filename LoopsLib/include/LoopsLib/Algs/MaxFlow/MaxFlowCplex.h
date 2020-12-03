#ifndef ALGS_MAXFLOW_MAXFLOWCPLEX_H
#define ALGS_MAXFLOW_MAXFLOWCPLEX_H
#include <Eigen/Eigen>
#include <LoopsLib/DS/OwnGraph.h>
#include <ilcplex/ilocplexi.h>
namespace LoopsLib::Algs::MaxFlow
{
    class MaxFlowCplex
    {
        DS::BaseGraph* m_graph;
        DS::BaseGraph::Id_t m_source, m_sink;
        bool m_verbose = true;

        template<typename VarType, typename VarArrayType, typename OutputType>
        void innerComputeMaxFlow(const OutputType& capacities, OutputType& maxFlow)
        {
            IloEnv env;
            IloModel model(env);

            // Add variables for edges
            VarArrayType vars(env);
            for (auto i = 0; i < m_graph->number_of_edges(); ++i)
            {
                VarType var(env);
                vars.add(var);
            }
            addCapacityConstraints(vars, model, capacities);

            // Create flow conservation
            addFlowConservation(vars, model);

            // Create the outflow expression
            IloExpr outFlow(env);
            for (auto* e : m_graph->vertex(m_sink)->m_inEdges)
            {
                outFlow += vars[e->id()];
            }

            model.add(IloMaximize(env, outFlow));

            // The solver
            IloCplex solver(model);

            if(!m_verbose)
            {
                solver.setOut(env.getNullStream());
            }

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
                maxFlow.setConstant(m_graph->number_of_edges(), 0);
                for (auto i = 0; i < m_graph->number_of_edges(); ++i)
                {
                    maxFlow(i) = solver.getValue(vars[i]);
                }
            }
            catch (IloException& e)
            {
                std::cout << "Ilo Exception:" << e.getMessage() << std::endl;
            }
            env.end();
        }
    public:
        void setVerbose(bool value)
        {
            m_verbose = value;
        }
        MaxFlowCplex(DS::BaseGraph* graph, DS::BaseGraph::Id_t source, DS::BaseGraph::Id_t sink):
        m_graph(graph),
        m_source(source),
        m_sink(sink)
        {
            
        }
        template<typename VarArrayType, typename EigenWeightsVector>
        void addCapacityConstraints(VarArrayType& vars, IloModel& model, EigenWeightsVector& capacities)
        {
            for (auto i = 0; i < m_graph->number_of_edges(); ++i)
            {
                auto var = vars[i];
                auto e = m_graph->edge(i);
                model.add(vars[i] >= 0);
                model.add(var <= capacities(i));
            }
        }
        template<typename VarArrayType>
        void addFlowConservation(VarArrayType& vars, IloModel& model)
        {
            for (auto v : m_graph->vertices())
            {
                // Ignore source and sink.
                if (v->id() == m_source || v->id() == m_sink) continue;

                IloExpr flow(vars.getEnv());
                for (auto e : v->m_inEdges)
                {
                    flow += vars[e->id()];
                }
                for (auto e : v->m_outEdges)
                {
                    flow -= vars[e->id()];
                }
                model.add(flow == 0);
            }
        }
        void computeIntegerMaxFlow(const Eigen::VectorXi& capacities, Eigen::VectorXi& maxFlow)
        {
            innerComputeMaxFlow<IloIntVar, IloIntVarArray, Eigen::VectorXi>(capacities, maxFlow);
        }
        void computeMaxFlow(const Eigen::VectorXd& capacities, Eigen::VectorXd& maxFlow)
        {
            innerComputeMaxFlow<IloNumVar, IloNumVarArray, Eigen::VectorXd>(capacities, maxFlow);
        }
    };
}
#endif