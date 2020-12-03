#ifndef ALGS_FLOWDECOMPOSITION_HITTINGSPATHSDECOMPOSITION_H
#define ALGS_FLOWDECOMPOSITION_HITTINGSPATHSDECOMPOSITION_H
#include <vector>
#include <LoopsAlgs/IFlowDecomposition.h>

namespace LoopsAlgs::FlowDecomposition
{
    /**
     * \brief Algorithm for a simple approach to path decomposition.
     * The algorithm decomposes the flow in cycles and simple paths.
     * If the given field is not an actual flow,
     * \tparam Graph
     * \tparam FlowField
     */
    class HittingPathsDecomposition : public IFlowDecomposition
    {
        int m_numberOfIterations = 5;

    public:
        HittingPathsDecomposition(LoopsLib::Models::DecompositionResult* decompObj);


    private:
        void findPaths(const DecompositionCoeffs_t& weights, int numberOfPaths, LoopsLib::DS::BaseGraph::Id_t src,
                       LoopsLib::DS::BaseGraph::Id_t sink, Basis_t& output);

    protected:
        void extendBasisForEndpoints(LoopsLib::DS::BaseGraph::Id_t src,
            LoopsLib::DS::BaseGraph::Id_t sink);

        void findNewBasisElements() override;
    public:
        std::map<std::string, std::string> metaData() const override
        {
            return { {"numberOfIterations",std::to_string(m_numberOfIterations)} };
        }
        void setFromMetaData(const std::map<std::string, std::string>& metData) override
        {
            m_numberOfIterations = std::stoi(metData.at("numberOfIterations").c_str());
        }

        std::string paramDescription() const override { return ""; }
    };
}
#endif