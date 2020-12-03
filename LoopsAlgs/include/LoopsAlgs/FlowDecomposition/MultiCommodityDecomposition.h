#ifndef LOOPSALGS_FLOWDECOMPOSIION_MULTICOMMODITYDECOMPOSITION_H
#define LOOPSALGS_FLOWDECOMPOSIION_MULTICOMMODITYDECOMPOSITION_H
#include "LoopsAlgs/IFlowDecomposition.h"

namespace LoopsAlgs::FlowDecomposition
{
    class MultiCommodityDecomposition: public IFlowDecomposition
    {
    public:
        MultiCommodityDecomposition(DecompositionResult_t* decompObj)
            : IFlowDecomposition("MultiCommodityDecomposition", decompObj)
        {
        }

        explicit MultiCommodityDecomposition()
            : IFlowDecomposition("MultiCommodityDecomposition")
        {
        }

    protected:
        void findNewBasisElements() override;
    public:
        std::map<std::string, std::string> metaData() const override { return{}; }
        void setFromMetaData(const std::map<std::string, std::string>& metData) override{}
        std::string paramDescription() const override { return ""; }
        bool isDone() override;
    };
}
#endif