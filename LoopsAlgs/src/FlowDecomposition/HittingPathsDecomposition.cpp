#include <LoopsAlgs/FlowDecomposition/HittingPathsDecomposition.h>
#include <LoopsLib/Algs/NNLS.h>
#include <LoopsLib/Algs/EdgeHittingPath.h>
using namespace LoopsAlgs::FlowDecomposition;
using namespace LoopsLib;
HittingPathsDecomposition::HittingPathsDecomposition(Models::DecompositionResult* decompObj):
IFlowDecomposition("HittingPaths", decompObj)
{
}


void HittingPathsDecomposition::findPaths(const DecompositionCoeffs_t& weights,
                                                                   int numberOfPaths, DS::BaseGraph::Id_t src,
                                                                   DS::BaseGraph::Id_t sink, Basis_t& output)
{
    Algs::EdgeHittingPath edgeHitAlg(graph(), src, sink);
    edgeHitAlg.setVerbose(false);
    for (std::size_t i = 0; i < numberOfPaths; i++)
    {
        std::vector<DS::BaseGraph::Edge*> path;
        edgeHitAlg.computePath(i, path);
        if (path.empty()) continue;
        output.emplace_back();
        std::transform(path.begin(), path.end(), std::back_inserter(output.back()), [](auto* e) {return e->id(); });
    }
}

void LoopsAlgs::FlowDecomposition::HittingPathsDecomposition::extendBasisForEndpoints(DS::BaseGraph::Id_t src,
                                                                                 DS::BaseGraph::Id_t sink)
{
    Basis_t extra;
    this->findPaths(field(), graph()->number_of_edges(), src, sink, extra);

    this->extendBasis(extra);
}

void LoopsAlgs::FlowDecomposition::HittingPathsDecomposition::findNewBasisElements()
{
    struct SrcSinkPair
    {
        DS::BaseGraph::Id_t src, sink;
        bool operator<(const SrcSinkPair& other) const
        {
            return src < other.src || (src == other.src && sink < other.sink);
        }
    };
    std::set<SrcSinkPair> uniquePairs;
    for (auto reprInd = 0; reprInd < numberOfRepresentatives(); ++reprInd)
    {
        const auto& repr = representative(reprInd);
        auto src = graph()->getIndex().closest(repr[0]);
        auto sink = graph()->getIndex().closest(repr.back());

        uniquePairs.insert(SrcSinkPair{ src,sink });
    }
    for (const auto& p : uniquePairs)
    {
        // Extend the basis with paths for these endpoints
        extendBasisForEndpoints(p.src, p.sink);
    }
}
