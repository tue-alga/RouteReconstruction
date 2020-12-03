#ifndef ALGS_PROCESSING_TRAJECTORYSETFRECHT_H
#define ALGS_PROCESSING_TRAJECTORYSETFRECHT_H
#include <LoopsLib/Algs/Types.h>
#include <movetk/algo/Similarity.h>

namespace LoopsLib {
    namespace DS {
        class EmbeddedGraph;
    }
}

namespace LoopsLib::Algs::Processing
{
    class TrajectorySetFrechet
    {
        using Point = LoopsLib::MovetkGeometryKernel::MovetkPoint;

        /**
         * \brief Convert graph path to R^2 trajectory specified by Movetk points.
         * \param trajectory The graph path
         * \param graph The underlying graph
         * \param movetkTrajectory Output trajectory
         */
        void toMovetkTrajectory(const BasisElement& trajectory, DS::EmbeddedGraph* graph,
                                std::vector<Point>& movetkTrajectory);

        NT m_tolerance = 0.01;
    public:
        struct Norm
        {
            NT operator()(const Point& p0, const Point& p1);
        };

        NT tolerance() const;

        void setTolerance(NT tolerance);

        /**
         * \brief Returns the smallest Frechet distance to one in the compare set per given trajectory in the trajectorySet.
         * \param trajectorySet The trajectory set
         * \param compareAgainst Set of trajectories to compare against
         * \param outputValues Per trajectory in trajectorySet, the minimum Frechet distance to one of the compare trajectories.
         */
        void minFrechetValues(DS::EmbeddedGraph* graph, const std::vector<BasisElement>& trajectorySet,
                              const std::vector<BasisElement>& compareAgainst, NT upperBound, std::vector<NT>& outputValues);

        void minFrechetValues(DS::EmbeddedGraph* graph, const std::vector<BasisElement>& trajectorySet,
            const std::vector<std::vector<Point>>& compareAgainst, NT upperBound, std::vector<NT>& outputValues);

        void minFrechetValues(DS::EmbeddedGraph* graph, const std::vector<std::vector<Point>>& trajectorySet,
            const std::vector<BasisElement>& compareAgainst, NT upperBound, std::vector<NT>& outputValues);

        void minFrechetValues(DS::EmbeddedGraph* graph, const std::vector<std::vector<Point>>& trajectorySet,
                              const std::vector<std::vector<Point>>& compareAgainst, NT upperBound,
                              std::vector<NT>& outputValues, std::vector<std::size_t>& setIndices);

        /**
         * \brief Returns the smallest Frechet distance to one in the compare set per given trajectory in the trajectorySet.
         * \param trajectorySet The trajectory set for which to compute Frechet distances
         * \param compareAgainst Set of trajectories to find a trajectory in with smallest Frechet distance to every trajectory in trajectory set.
         * \param outputValues Per trajectory in trajectorySet, the minimum Frechet distance to one of the compare trajectories.
         * \param setIndices Index, per trajectory in trajectorySet, in compareAgainst of the trajectory with lowest Frechet
         */
        void minFrechetValues(DS::EmbeddedGraph* graph, const std::vector<BasisElement>& trajectorySet,
            const std::vector<BasisElement>& compareAgainst, NT upperBound, std::vector<NT>& outputValues, std::vector<std::size_t>& setIndices);



        void frechetTable(DS::EmbeddedGraph* graph, const std::vector<BasisElement>& trajectorySet,
                          const std::vector<BasisElement>& compareAgainst, std::vector<std::vector<NT>>& outputValues);
    };
}
#endif