#ifndef LOOPSLIB_MODELS_RECONSTRUCTEDTRAJECTORY_H
#define LOOPSLIB_MODELS_RECONSTRUCTEDTRAJECTORY_H
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/DS/OwnGraph.h>
#include <LoopsLib/DS/EmbeddedGraph.h>
namespace LoopsLib::Models {

    class ReconstructedTrajectory {
        // The reconstructed path
        std::vector<DS::BaseGraph::EdgeIndex> m_edges;
        // The representative that this path came from
        std::size_t m_sourceRepresentative = 0;
        // Unique ID (within decomposition) of trajectory. Not enforced!
        long long m_id = 0;
    public:
        ReconstructedTrajectory(long long id);

        ReconstructedTrajectory(const std::vector<DS::BaseGraph::EdgeIndex>& path, std::size_t sourceRepresentative,
                                long long id);

        std::vector<DS::BaseGraph::EdgeIndex>::const_iterator begin() const;

        std::vector<DS::BaseGraph::EdgeIndex>::const_iterator end() const;

        void setId(long long id);

        void setSourceRepresentative(std::size_t sourceRepresentative);

        std::size_t sourceRepresentative() const;

        const std::vector<DS::BaseGraph::EdgeIndex>& path() const;

        std::vector<DS::BaseGraph::EdgeIndex>& path();

        long long id() const;
    };
}
#endif