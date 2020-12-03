#include <LoopsAlgs/FlowDecomposition/FrechetDecomposition.h>

using namespace LoopsLib;

LoopsAlgs::FlowDecomposition::FrechetDecomposition::
FrechetDecomposition(Models::DecompositionResult* decompObj): IFlowDecomposition("Frechet", decompObj)
{
}


void LoopsAlgs::FlowDecomposition::FrechetDecomposition::findNewBasisElements()
{
    auto logger = Helpers::logFactory(this);
    if (m_decompObj->paths().empty())
    {
        logger.error("No paths");
        return;
    }
    const auto numRepresentatives = numberOfRepresentatives();
    for(std::size_t i = 0; i < numRepresentatives; ++i)
    {
        try
        {
            // Just get a single path to test. Path is given as ...
            std::vector<DS::BaseGraph::Id_t> outputPath; // Path of vertices
            m_frechetAlg.setTargetGraph(graph());
            NT epsilon = -1;
            m_frechetAlg.apply(representative(i), m_lowerBound, m_precision, outputPath, epsilon);

            if (outputPath.size() == 0) {
                logger.info("No Frechet path for path ID ", decompositionObject()->m_relatedInstance->representativeDescription(i));
                continue;
            }

            // Add to the basis
            basis().push_back(outputPath);
        }
        catch(std::exception& e)
        {
            std::string msg = e.what();
            msg += " \nat representative" + std::to_string(i) + ", id = " + this->m_decompObj->m_relatedInstance->representativeDescription(i);
            throw std::runtime_error(msg);
        }
    }
}

void LoopsAlgs::FlowDecomposition::FrechetDecomposition::setLowerBound(NT lowerBound)
{
    m_lowerBound = lowerBound;
}

NT LoopsAlgs::FlowDecomposition::FrechetDecomposition::lowerBound() const
{
    return m_lowerBound;
}

void LoopsAlgs::FlowDecomposition::FrechetDecomposition::setPrecision(NT precision)
{
    m_precision = precision;
}

NT LoopsAlgs::FlowDecomposition::FrechetDecomposition::precision() const
{
    return m_precision;
}

std::map<std::string, std::string> LoopsAlgs::FlowDecomposition::FrechetDecomposition::metaData() const
{
    return {};
}

void LoopsAlgs::FlowDecomposition::FrechetDecomposition::setFromMetaData(
    const std::map<std::string, std::string>& metData)
{
}

std::string LoopsAlgs::FlowDecomposition::FrechetDecomposition::paramDescription() const
{
    return "";
}
