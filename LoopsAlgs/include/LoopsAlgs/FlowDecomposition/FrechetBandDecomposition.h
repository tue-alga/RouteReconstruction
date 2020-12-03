#ifndef ALGS_FLOWDECOMPOSITION_FRECHETBANDDECOMPOSITION_H
#define ALGS_FLOWDECOMPOSITION_FRECHETBANDDECOMPOSITION_H
#include <vector>
#include <LoopsAlgs/IFlowDecomposition.h>
#include <LoopsAlgs/Frechet/PathHittingStrongFrechet.h>

namespace LoopsAlgs::FlowDecomposition
{
    /**
     * \brief Algorithm for a simple approach to path decomposition.
     * The algorithm decomposes the flow in cycles and simple paths.
     * If the given field is not an actual flow,
     * \tparam Graph
     * \tparam FlowField
     */
    class FrechetBandDecomposition : public IFlowDecomposition
    {
    public:
        FrechetBandDecomposition(LoopsLib::Models::DecompositionResult* decompObj);

    protected:
        void findNewBasisElements() override;
    public:
        std::map<std::string, std::string> metaData() const override
        {
            return std::map<std::string, std::string>{ };
        }
        void setFromMetaData(const std::map<std::string, std::string>& metData) override
        {
        }

        std::string paramDescription() const override
        {
            std::stringstream ss;
            return ss.str();
        }
    };
}
#endif