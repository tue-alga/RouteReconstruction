#ifndef ALGS_NNLS_H
#define ALGS_NNLS_H
#include <Eigen/Eigen>
#include <unordered_set>
#include <numeric>
#include <ilcplex/ilocplex.h>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Helpers/Iterators.h>

namespace LoopsLib::Algs 
{

    /**
     * \brief Nonnegative least squares
     */
    class NNLS
    {
        // Show output or not
        bool m_disableOutput = false;

        // Number of threads to use
        int m_numberOfThreads = 8;

        // Try to use coefficients as start
        bool m_useCoeffsAsStart = false;

        bool solveModel(IloNumVarArray& coeffVars, IloModel& model, std::vector<NT>& output, NT& objectiveResult);

        bool solveModel(IloNumVarArray& coeffVars, IloExprArray& fieldExpressions, const std::vector<NT>& field, IloModel& model, std::vector<
                        NT>& output, NT& objectiveResult);

        void setup(long long numberOfEdges, int basisSize, IloEnv& env, IloModel& model, IloNumVarArray& coeffVar,
                   IloExprArray& fieldExpressions);

        std::ostream* m_stream;
        std::ostream& out()
        {
            return *m_stream;
        }
        template<typename...Args>
        void printOut(Args&&...args)
        {
            using exp = int[];
            out() << "[NNLS] ";
            (void)exp {
                (args,0)...
            };
            out() << std::endl;
        }

        // Maximum number of greedy steps to apply in greedyRefine()
        int m_maxGreedySteps = 20;
        // Max number of time spent in greedy refinement
        double m_maxGreedyTime = 60. * 5.;
    public:
        using NT = MovetkGeometryKernel::NT;
        ~NNLS();
        NNLS();

        void setOutStream(std::ostream& stream);

        void setDisableOutput(bool value);

        bool useCoeffsAsStart() const;

        void setUseCoeffsAsStart(bool val);

        int maxGreedySteps() const
        {
            return m_maxGreedySteps;
        }

        void setMaxGreedySteps(int steps);

        void setMaxGreedyTime(const double& maxGreedyTime)
        {
            m_maxGreedyTime = maxGreedyTime;
        }
        /**
         * \brief Maximum allowed time spent in greedy refinement
         * \return 
         */
        double maxGreedyTime() const
        {
            return m_maxGreedyTime;
        }


        bool solve(const std::vector<std::vector<DS::BaseGraph::Edge*>>& basis, const std::vector<NT>& field,
                   std::vector<NT>& output, NT& objectiveResult);

        bool solve(const std::vector<std::vector<DS::BaseGraph::Id_t>>& basis, const std::vector<NT>& field,
            std::vector<NT>& output, NT& objectiveResult);

        bool solveBatched(const std::vector<std::vector<DS::BaseGraph::Id_t>>& basis, const std::vector<NT>& field, std::size_t batchSize, 
                          std::vector<NT>& output, NT& objectiveResult);

        /**
         * \brief Solves the NNLS problem for the given basis incrementally, retaining atmost the given retainsize for the basis size
         * \param basis The basis
         * \param field The field
         * \param batchSize The amount of new elements from the batch to take into account
         * \param retainSize The number of basis elements to retain 
         * \param retainFraction When true, only retain retainSize/batches per step, where batches is the basis size divided by the batch size
         * \param retainedIndices The indices in the basis of the retained elements
         * \param output Coefficients for the basis
         * \param objectiveResult The output objective value
         * \return 
         */
        bool solveIncrementally(const std::vector<std::vector<DS::BaseGraph::Id_t>>& basis, const std::vector<NT>& field, std::size_t batchSize,
            std::size_t retainSize, bool retainFraction, std::vector<std::size_t>& retainedIndices, std::vector<NT>& output, NT& objectiveResult);

        /**
         * \brief Given a basis, field and coefficients, tries to refine the coefficients by trying to greedily minimize the least squares for 
         * each consequent basis element, only increasing the coefficients
         * \param basis The basis
         * \param field The field
         * \param coefficients The current coefficients, that will be refined in place.
         */
        void greedyRefine(const std::vector<std::vector<DS::BaseGraph::Id_t>>& basis, const std::vector<NT>& field, std::vector<NT>& coefficients, NT& objectiveValue);

        /*bool solveBatched(const std::vector<std::vector<DS::BaseGraph::Id_t>>& basis, const std::vector<NT>& field, std::size_t batchSize, NT
            pruneValue,
            std::vector<NT>& output, NT& objectiveResult, std::vector<std::size_t>& retainSet);*/
        
        template<typename Iterable, typename ToIdConverter>
        bool solve(const Iterable& iterable, long long iterableSize, ToIdConverter&& converter, const std::vector<NT>& field, std::vector<NT>& output, NT& objectiveResult)
        {
            IloEnv env;
            IloModel model;
            IloNumVarArray coeffs;
            IloExprArray fieldExpressions;
            setup(field.size(), iterableSize, env, model, coeffs, fieldExpressions);
            long long idx = 0;
            for(const auto& basisElement: iterable)
            {
                for(const auto& edgeRepresentation : basisElement)
                {
                    auto eId = converter(edgeRepresentation);
                    fieldExpressions[eId] += coeffs[idx];
                }
                ++idx;
            }
            return solveModel(coeffs, fieldExpressions, field, model, output, objectiveResult);
        }
        template<typename It, typename ToEdgeIdIterableConverter>
        bool solve(It begin, It end, ToEdgeIdIterableConverter&& converter, const std::vector<NT>& field, std::vector<NT>& output, NT& objectiveResult)
        {
            IloEnv env;
            IloModel model;
            IloNumVarArray coeffs;
            IloExprArray fieldExpressions;
            setup(field.size(), std::distance(begin,end), env, model, coeffs, fieldExpressions);
            long long idx = 0;
            for (const auto& basisElement : Helpers::Iterators::PairIterable(begin,end))
            {
                for (const auto& edgeRepresentation : basisElement)
                {
                    auto eId = converter(edgeRepresentation);
                    fieldExpressions[eId] += coeffs[idx];
                }
                ++idx;
            }
            return solveModel(coeffs, fieldExpressions, field, model, output, objectiveResult);
        }

        /**
         * \brief Solve the NNLS problem with only partial observation. 
         * \param basis The basis to use, specified as list of list of edges.
         * \param field Field, defined as a map from edge index to the field value
         * \param output The output coefficients
         * \param objectiveResult Object function value (sum of square diff)
         * \return 
         */
        bool solve(const std::vector<std::vector<DS::BaseGraph::Id_t>>& basis, const std::map<DS::BaseGraph::Id_t,NT>& field,
            std::vector<NT>& output, NT& objectiveResult);

        /**
         * \brief Applies non-negative least squares to the given basis and field via Cplex.
         * \param basis
         * \param field
         * \param output
         * \param objectiveResult
         * \return
         */
        bool solve(const Eigen::MatrixXd& basis, const Eigen::VectorXd& field, Eigen::VectorXd& output,
                   double& objectiveResult);
    };
}
#endif
