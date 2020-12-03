#include <LoopsAlgs/FlowDecomposition/FrechetBandDecomposition.h>
#include "LoopsAlgs/MapMatching/StrongFrechetMapMatching.h"

LoopsAlgs::FlowDecomposition::FrechetBandDecomposition::FrechetBandDecomposition(
    LoopsLib::Models::DecompositionResult* decompObj) : IFlowDecomposition("FrechetBandDecomposition",decompObj)
{
}

void LoopsAlgs::FlowDecomposition::FrechetBandDecomposition::findNewBasisElements()
{
    auto logger = LoopsLib::Helpers::logFactory(this);
    if (m_decompObj->paths().empty())
    {
        logger.error("No paths");
        return;
    }
    if(graph()->getIndex().isEmpty())
    {
        graph()->getIndex().construct(graph()->locations());
    }

    const auto numRepresentatives = numberOfRepresentatives();

    MapMatching::StrongFrechetMapMatching frechetAlg;
    const auto epsilon = decompositionObject()->m_relatedInstance->m_epsilon;
    for (std::size_t i = 0; i < numRepresentatives; ++i)
    {
        try
        {
            // Just get a single path to test. Path is given as ...
            std::vector<LoopsLib::DS::BaseGraph::Id_t> outputPath; // Path of edges
            frechetAlg.setTargetGraph(graph());
            frechetAlg.applyFixed(representative(i), epsilon, outputPath);

            if (outputPath.size() == 0) {
                logger.info("No Frechet path for path ID ", decompositionObject()->m_relatedInstance->representativeDescription(i));
                continue;
            }

            // Add to the basis
            basis().push_back(outputPath);
        }
        catch (std::exception& e)
        {
            std::string msg = e.what();
            msg += " \nat representative" + std::to_string(i) + ", id = " + this->m_decompObj->m_relatedInstance->representativeDescription(i);
            throw std::runtime_error(msg);
        }
    }
}
