#ifndef ALGS_LONGESTPATH_LONGESTPATHILP_H
#define ALGS_LONGESTPATH_LONGESTPATHILP_H
#include <LoopsLib/DS/OwnGraph.h>
#include <Eigen/Eigen>
#include <ilcplex/ilocplexi.h>
#include "ILongestPathAlg.h"
namespace LoopsLib::Algs::LongestPath
{
    /**
		 * \brief ILP based longest path searcher. Actually, MIP, but whatever.
		 */
    class LongestPathILP : public ILongestPath
    {
        NT m_bestWeight = 0;
    public:
        LongestPathILP(DS::BaseGraph* graph)
            : ILongestPath("ILP",graph)
        {
        }

    private:
        /**
			 * \brief Setup the constraints for the simple path requirement
			 * \param vars The edge variables
			 * \param target The target model
			 */
        void setupSimpleConstraints(IloBoolVarArray& vars, IloModel& target) const;

        void setupNewConstraints(IloIntVarArray& intVars, IloBoolVarArray& vars, IloModel& target) const;
    protected:
        BasisElement computeLongestPath(const FieldType& field, NT maxWeight) override;
    public:
        /**
             * \brief Retries to compute a longest path, given that the previous one was not usable
             * \param field The field to use
             * \return A new longest path
             */
        BasisElement retry(const FieldType& field) override;

    protected:
        void clear() override;
    };
}
#endif
