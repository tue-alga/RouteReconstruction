#include <LoopsAlgs/FlowDecomposition/ILPFlowDecomposition.h>
using namespace LoopsLib;
void LoopsAlgs::FlowDecomposition::ILPFlowDecomposition::toEigen(std::vector<NT>& output, IloCplex& solver,
                                                            IloNumVarArray& vars)
{
    output.assign(vars.getSize(), 0);
    for (int i = 0; i < vars.getSize(); ++i)
    {
        output[i] = solver.getValue(vars[i]) > 0.5;
    }
}

void LoopsAlgs::FlowDecomposition::ILPFlowDecomposition::toEigen(Basis_t& output, IloCplex& solver,
                                                            MatrixView<IloBoolVarArray>& varsMatrix)
{
    output.reserve(varsMatrix.m_numRows);
    //output.setConstant(varsMatrix.m_numRows, varsMatrix.columnCount(), 0);
    for (int r = 0; r < varsMatrix.m_numRows; ++r)
    {
        std::map<DS::BaseGraph::Id_t, std::set<DS::BaseGraph::Edge*>> srcMap;
        std::map<DS::BaseGraph::Id_t, std::set<DS::BaseGraph::Edge*>> sinkMap;
        std::set<DS::BaseGraph::Id_t> srcs;
        for (int c = 0; c < varsMatrix.columnCount(); ++c)
        {
            auto* e = graph()->edge(c);
            auto var = varsMatrix(r, c);
            if (solver.getValue(var) > 0.5)
            {
                if(srcMap.find(e->m_source->id()) == srcMap.end())
                {
                    srcMap[e->m_source->id()] = {};
                }
                srcMap[e->m_source->id()].insert(e);
                if (sinkMap.find(e->m_sink->id()) == sinkMap.end())
                {
                    sinkMap[e->m_sink->id()] = {};
                }
                sinkMap[e->m_sink->id()].insert(e);
                srcs.insert(e->m_source->id());
            }
            DS::BaseGraph::Id_t startV = std::numeric_limits<decltype(startV)>::max();
            for(auto el : srcs)
            {
                if(sinkMap.find(el) != sinkMap.end())
                {
                    continue;
                }
                assert(startV == std::numeric_limits<decltype(startV)>::max());
                startV = el;
            }
            std::vector<DS::BaseGraph::Edge*> path;
            throw std::runtime_error("TODO: Not implemented!");
        }
    }
}

void LoopsAlgs::FlowDecomposition::ILPFlowDecomposition::findNewBasisElements()
{
    for (auto reprInd = 0; reprInd < numberOfRepresentatives(); ++reprInd)
    {
        const auto& repr = representative(reprInd);
        const auto src = graph()->getIndex().closest(repr[0]);
        for (auto reprInd2 = 0; reprInd2 < numberOfRepresentatives(); ++reprInd2)
        {
            const auto sink = graph()->getIndex().closest(representative(reprInd2).back());
            decomposeForSourceSink(src, sink);
        }
    }
}
