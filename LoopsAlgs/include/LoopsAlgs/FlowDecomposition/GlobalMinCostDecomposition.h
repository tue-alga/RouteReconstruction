#ifndef LOOPSALGS_FLOWDECOMPOSITION_GLOBALMINCOSTDECOMPOSITION_H
#define LOOPSALGS_FLOWDECOMPOSITION_GLOBALMINCOSTDECOMPOSITION_H
#include <LoopsAlgs/IFlowDecomposition.h>
#include "LoopsLib/DS/BoostInterface.h"

namespace LoopsAlgs::FlowDecomposition
{
    class GlobalMinCostDecomposition;
}
namespace LoopsLib::Helpers
{
    template<> constexpr int logLevel<LoopsAlgs::FlowDecomposition::GlobalMinCostDecomposition>()
    {
        return LogLevel::Info;
    }
}

namespace LoopsAlgs::FlowDecomposition
{
    class GlobalMinCostDecomposition : public IFlowDecomposition
    {
    public:
        GlobalMinCostDecomposition(DecompositionResult_t* decompObj);

        explicit GlobalMinCostDecomposition();

    protected:
        void verifyFlow(const std::set<boost::graph_traits<Graph_t>::vertex_descriptor>& sources, const std::set<boost::graph_traits<Graph_t>::
                        vertex_descriptor>& sinks, const std::vector<NT>& flow);
        virtual void findNewBasisElements() override;
    public:
        std::map<std::string, std::string> metaData() const override;
        void setFromMetaData(const std::map<std::string, std::string>& metData) override;

        std::string paramDescription() const override;

        bool isDone() override;
    };
}
#endif