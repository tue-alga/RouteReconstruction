#ifndef ALGS_TRIVIALDECOMPOSITION_H
#define ALGS_TRIVIALDECOMPOSITION_H
#include <LoopsAlgs/IFlowDecomposition.h>
#include <LoopsLib/DS/OwnGraph.h>
#include <ilcplex/ilocplexi.h>

namespace LoopsAlgs::FlowDecomposition
{
    class TrivialDecomposition : public IFlowDecomposition
    {
        int m_pathSize = 1;
    public:
        TrivialDecomposition(LoopsLib::Models::DecompositionResult* decompObj)
            : IFlowDecomposition("Trivial decomposition", decompObj)
        {
        }
        int pathSize() const
        {
            return m_pathSize;
        }
        void setPathSize(int size)
        {
            m_pathSize = size;
        }

    protected:
        void generatePaths(LoopsLib::DS::BaseGraph::Vertex* vertex);

        virtual void findNewBasisElements() override;
    public:
        std::map<std::string, std::string> metaData() const override
        {
            return std::map<std::string, std::string>{ {"name","TrivialDecomposition"},{"pathSize",std::to_string(m_pathSize)} };
        }
        void setFromMetaData(const std::map<std::string, std::string>& metData) override
        {
            m_pathSize = std::stoi(metData.at("pathSize"));
        }

        std::string paramDescription() const override
        {
            return "pathSize " + std::to_string(m_pathSize);
        }
    };
}
#endif