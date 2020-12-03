#ifndef ALGS_SHORTESTPATHSDECOMPOSITION_H
#define ALGS_SHORTESTPATHSDECOMPOSITION_H
#include <LoopsAlgs/IFlowDecomposition.h>

namespace LoopsAlgs::FlowDecomposition
{
    class ShortestPathsDecomposition : public IFlowDecomposition
    {
        int m_maxPathsPerAvailable = -1;
    public:
        ShortestPathsDecomposition(LoopsLib::Models::DecompositionResult* decompObj);

        void setMaxPathsPerAvailable(int value);

    protected:

        virtual void findNewBasisElements() override;
    public:
        std::map<std::string, std::string> metaData() const override;
        void setFromMetaData(const std::map<std::string, std::string>& metData) override;

        std::string paramDescription() const override;
    };
}
#endif