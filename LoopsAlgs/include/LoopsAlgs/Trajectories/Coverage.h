#ifndef LOOPSALGS_TRAJECTORIES_COVERAGE
#define LOOPSALGS_TRAJECTORIES_COVERAGE
#include <LoopsLib/Algs/Types.h>
namespace LoopsAlgs::Trajectories
{
    class Coverage
    {
    public:
        enum class Mode
        {
            ExactSolve,
            Randomized
        };
    private:
        Mode m_mode = Mode::ExactSolve;
        // Tolerance on the frechet distance
        LoopsLib::NT m_frechetTolerance = 0.001;
        // Precision of computed fraction, if not done via exact Mode.
        LoopsLib::NT m_fractionPrecision = 0.01;

        /**
         * \brief Compute the frechet distance table, or load from a cache file if available
         * \param centers The centers to use for the distance
         * \param coverSet The trajectories to cover
         * \param cachePath The patht to the cache file, or empty
         * \param table Output table
         */
        void computeFrechetTable(const LoopsLib::MovetkGeometryKernel::TrajectorySet& centers,
            const LoopsLib::MovetkGeometryKernel::TrajectorySet& coverSet,
            const std::string& cachePath, std::vector<std::vector<LoopsLib::NT>>& table) const;
    public:
        Mode mode() const;

        void setMode(Mode mode);
        using NT = LoopsLib::NT;

        void setFrechetTolerance(const LoopsLib::NT& frechetTolerance);

        LoopsLib::NT frechetTolerance() const;

        void setFractionPrecision(const LoopsLib::NT& fractionPrecision);

        LoopsLib::NT fractionPrecision() const;


        /**
         * \brief Computes the smallest percentage for which that amount of trajectories cover the rest within
         * epsilon distance
         * \param trajectorySet The set of trajectories
         * \param epsilon The maximum Frechet distance allowed
         * \param fractionResolution Precision in the percentage
         * \param fraction The output percentage
         * \return Whether a solution could be found. This can be false when some trajectory has Frechet distance
         * larger than epsilon to all other trajectories.
         */
        bool compute(const LoopsLib::MovetkGeometryKernel::TrajectorySet& centers, 
            const LoopsLib::MovetkGeometryKernel::TrajectorySet& coverSet,
            const std::string& cachePath,
                    NT epsilon, NT& fraction) const;

        /**
         * \brief Computes the smallest percentage for which that amount of trajectories cover the rest within
         * epsilon distance
         * \param trajectorySet The set of trajectories
         * \param epsilon The maximum Frechet distance allowed
         * \param fractionResolution Precision in the percentage
         * \param fraction The output percentage
         * \return Whether a solution could be found. This can be false when some trajectory has Frechet distance
         * larger than epsilon to all other trajectories.
         */
        bool computeUpperBounded(const LoopsLib::MovetkGeometryKernel::TrajectorySet& centers,
            const LoopsLib::MovetkGeometryKernel::TrajectorySet& coverSet,
            const std::string& cachePath,
            NT epsilon, NT& fraction) const;
    };
}
#endif