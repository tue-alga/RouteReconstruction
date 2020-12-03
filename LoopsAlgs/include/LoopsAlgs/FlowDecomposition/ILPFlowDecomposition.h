#ifndef ALGS_ILPFLOWDECOMPOSITION_H
#define ALGS_ILPFLOWDECOMPOSITION_H
#include <LoopsAlgs/IFlowDecomposition.h>
#include <LoopsLib/DS/OwnGraph.h>
#include <ilcplex/ilocplexi.h>

namespace LoopsAlgs::FlowDecomposition
{
    class ILPFlowDecomposition : public IFlowDecomposition
    {
        long long m_timeLimit = -1;
    public:
        ILPFlowDecomposition(LoopsLib::Models::DecompositionResult* decompObj)
            : IFlowDecomposition("ILP Decomp", decompObj)
        {
        }
        void setTimeLimit(long long value)
        {
            m_timeLimit = value;
        }
        long long timeLimit() const
        {
            return m_timeLimit;
        }

    protected:
        template<typename IloArrayT>
        struct MatrixView
        {
            IloArrayT& m_array;
            int m_numRows;
        public:
            MatrixView(IloArrayT& array, int numRows):m_array(array),m_numRows(numRows){}

            auto operator()(int r, int c)
            {
                return m_array[c*m_numRows + r];
            }
            template<typename OtherArrayT>
            auto operator*(const OtherArrayT& arr)
            {
                IloExpr expr(m_array.getEnv());
                for(int i = 0; i < m_numRows; ++i)
                {
                    for(int j =0; j < arr.getSize(); ++j)
                    {
                        expr += this->operator()(i, j) * arr[j];
                    }
                }
                return expr;
            }
            template<typename OtherArrayT>
            auto sumOfSquares(const OtherArrayT& valueVars, const Eigen::VectorXd& subtractCoeffs)
            {
                IloExpr expr(m_array.getEnv());
                for (int i = 0; i < m_numRows; ++i)
                {
                    IloExpr localSum(m_array.getEnv());
                    for (int j = 0; j < valueVars.getSize(); ++j)
                    {
                        localSum += this->operator()(i, j) * valueVars[j];
                    }
                    expr += (localSum - subtractCoeffs(i))*(localSum - subtractCoeffs(i));
                }
                return expr;
            }
            template<typename OtherArrayT>
            auto sumOfSquares(const OtherArrayT& valueVars, const std::vector<LoopsLib::NT>& subtractCoeffs)
            {
                IloExpr expr(m_array.getEnv());
                for (int i = 0; i < m_numRows; ++i)
                {
                    IloExpr localSum(m_array.getEnv());
                    for (int j = 0; j < valueVars.getSize(); ++j)
                    {
                        localSum += this->operator()(i, j) * valueVars[j];
                    }
                    expr += (localSum - subtractCoeffs[i])*(localSum - subtractCoeffs[i]);
                }
                return expr;
            }
            int columnCount() const
            {
                return m_array.getSize() / m_numRows;
            }
        };

        /**
         * \brief Converts the result of the solver to Eigen vector
         * \param output The output eigen vector
         * \param solver The cplex solver
         * \param vars The number variable array to convert to Eigen
         */
        void toEigen(std::vector<LoopsLib::NT>& output, IloCplex& solver, IloNumVarArray& vars);

        /**
         * \brief Converts the result of a matrix bool variable of the solver to Eigen matrix
         * \param output The output eigen matrix
         * \param solver The cplex solver
         * \param vars A matrix view of a bool variable array
         */
        void toEigen(Basis_t& output, IloCplex& solver, MatrixView<IloBoolVarArray>& varsMatrix);

        void decomposeForSourceSink(LoopsLib::DS::BaseGraph::Id_t src, LoopsLib::DS::BaseGraph::Id_t sink)
        {
            IloEnv env;
            IloModel model(env);
            const int eCount = graph()->number_of_edges();
            // Decomposition path values
            IloNumVarArray valueVars(env);
            for (int i = 0; i < eCount; ++i)
            {
                auto var = IloNumVar(env);
                valueVars.add(var);
                // Needs to be non-negative.
                model.add(var >= 0);
            }
            // Column major path inclusion variables matrix
            IloBoolVarArray pathInclusionVars(env);
            for (int i = 0; i < eCount*eCount; ++i)
            {
                pathInclusionVars.add(IloBoolVar(env));
            }

            // Maximum vertex label
            const IloInt maxLabel = graph()->number_of_vertices() + 20;

            IloIntVarArray vertexLabeling(env);
            for (int i = 0; i < eCount; ++i)
            {
                for (int j = 0; j < graph()->number_of_vertices(); ++j)
                {
                    auto var = IloIntVar(env);
                    model.add(var >= 0);
                    model.add(var <= maxLabel); // Add some wiggle room
                    vertexLabeling.add(var);
                }
            }
            // For every path
            MatrixView<IloBoolVarArray> matView(pathInclusionVars, eCount);
            MatrixView<IloIntVarArray> matViewLabels(vertexLabeling, graph()->number_of_vertices());
            for (int c = 0; c < eCount; ++c)
            {
                // Setup simplicity constraints
                for (int v = 0; v < graph()->number_of_vertices(); ++v)
                {
                    IloExpr incoming(env);
                    for (auto e : graph()->vertex(v)->m_inEdges)
                    {
                        incoming += matView(e->id(), c);
                    }
                    IloExpr outgoing(env);
                    for (auto e : graph()->vertex(v)->m_outEdges)
                    {
                        outgoing += matView(e->id(), c);
                    }
                    // Sink: disable outgoing, limit incoming
                    if (v == sink)
                    {
                        model.add(incoming <= 1);
                        model.add(outgoing == 0);
                        model.add(matViewLabels(v, c) == maxLabel);
                    }
                    // Source: disable incoming, limit outgoing
                    else if (v == src)
                    {
                        model.add(outgoing <= 1);
                        model.add(incoming == 0);
                        model.add(matViewLabels(v, c) == 2);
                    }
                    // Otherwise: sink in and out, atmost 1 edge for each
                    else
                    {
                        model.add(incoming <= 1);
                        model.add(outgoing == incoming);
                    }
                }
                // Exclude small loops
                for (int eI = 0; eI < eCount; ++eI)
                {
                    auto* e = graph()->edge(eI);
                    auto var = matView(e->id(), c);
                    if (e->m_sink->connectedTo(e->m_source->id()))
                    {
                        auto* otherE = e->m_sink->findOutEdge(e->m_source);
                        model.add(matView(e->id(), c) + matView(otherE->id(), c) <= 1);
                    }
                    // Enforce simplicity by labeling
                    model.add(matViewLabels(*e->m_sink, c) - matViewLabels(*e->m_sink, c) > 0 || matView(*e, c) == 0);
                }
            }

            // Add objective
            IloExpr goal = matView.sumOfSquares(valueVars, field());
            model.add(IloMinimize(env, goal));

            IloCplex solver(model);
            try
            {
                bool success = solver.solve();
                if (!success)
                {
                    log() << "Failed to solve";
                    basis().clear();
                    coefficients().clear();
                }
                else
                {
                    // Convert the basis and decomposition values to eigen matrices.
                    toEigen(basis(), solver, matView);
                    toEigen(coefficients(), solver, valueVars);
                }
            }
            catch (IloException& e)
            {
                log() << "IloException: " << e.getMessage() << std::endl;
            }
            env.end();
        }

        virtual void findNewBasisElements() override;
    public:
        std::map<std::string, std::string> metaData() const override
        {
            return{};
        }
        void setFromMetaData(const std::map<std::string, std::string>& metData) override{}
        std::string paramDescription() const override
        {
            std::stringstream ss;
            ss << "TimeLimit " << m_timeLimit;
            return ss.str();
        }
    };
}
#endif