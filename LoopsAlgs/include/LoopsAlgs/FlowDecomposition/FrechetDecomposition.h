#ifndef ALGS_FLOWDECOMPOSITION_FRECHETDECOMPOSITION_H
#define ALGS_FLOWDECOMPOSITION_FRECHETDECOMPOSITION_H
#include <LoopsAlgs/IFlowDecomposition.h>
#include <LoopsAlgs/Frechet/FrechetOnGraph.h>
#include "LoopsAlgs/MapMatching/StrongFrechetMapMatching.h"

namespace LoopsAlgs::FlowDecomposition
{
    class FrechetDecomposition : public IFlowDecomposition
    {
        MapMatching::StrongFrechetMapMatching m_frechetAlg;
        std::vector<LoopsLib::MovetkGeometryKernel::MovetkPoint> m_locations;

        NT m_lowerBound = 5.0, m_precision = 0.1;
    public:
        FrechetDecomposition(LoopsLib::Models::DecompositionResult* decompObj);


    protected:
        void findNewBasisElements() override;
    public:
        void setLowerBound(NT lowerBound);

        NT lowerBound() const;

        void setPrecision(NT precision);

        NT precision() const;

        std::map<std::string, std::string> metaData() const override;

        void setFromMetaData(const std::map<std::string, std::string>& metData) override;

        std::string paramDescription() const override;
    };
}
#endif
