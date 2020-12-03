#ifndef ALGS_FLOWDECOMPOSITION_FRECHETHITTINGDECOMPOSITION_H
#define ALGS_FLOWDECOMPOSITION_FRECHETHITTINGDECOMPOSITION_H
#include <vector>
#include <LoopsAlgs/IFlowDecomposition.h>
#include <LoopsAlgs/Frechet/PathHittingStrongFrechet.h>

namespace LoopsAlgs::FlowDecomposition
{
    /**
     * \brief Algorithm for a simple approach to path decomposition.
     * The algorithm decomposes the flow in cycles and simple paths.
     * If the given field is not an actual flow,
     * \tparam Graph
     * \tparam FlowField
     */
    class FrechetHittingDecomposition : public IFlowDecomposition
    {
        std::vector<LoopsAlgs::Frechet::StrongFrechetGraphData> m_graphDatas;

        int m_num = 5;

        int m_threads = 1;

        bool m_greedyAttachEnds = false;

    public:
        FrechetHittingDecomposition(LoopsLib::Models::DecompositionResult* decompObj);

    protected:
        void extendEnds(const Frechet::Polyline& representative, const std::vector<LoopsLib::NT>& residual, std::vector<long long>& path);

        void findPaths(Frechet::StrongFrechetGraphData& data, const std::vector<LoopsLib::NT>& residual, std::size_t pathIndex,
                       std::size_t currentBasisSize, Basis_t& output);
        void findNewBasisElementsParallel();
        void findNewBasisElements() override;
    public:
        void setGreedyAttachEnds(const bool& greedyAttachEnds);

        bool greedyAttachEnds() const;

        void setPathsPerTrajectory(int num)
        {
            m_num = num;
        }
        int numberOfThreads() const
        {
            return m_threads;
        }
        void setNumberOfThreads(int num)
        {
            m_threads = num;
        }
        int pathsPerTrajectory() const
        {
            return m_num;
        }
        std::map<std::string, std::string> metaData() const override
        {
            return std::map<std::string, std::string>{ };
        }
        void setFromMetaData(const std::map<std::string, std::string>& metData) override
        {
        }

        std::string paramDescription() const override
        {
            std::stringstream ss;
            ss << "pathsPerTrajectort " << m_num << " threadNum " << m_threads;
            return ss.str();
        }
    };
}
#endif