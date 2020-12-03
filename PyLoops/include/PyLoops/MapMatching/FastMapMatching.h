#ifndef PYLOOPS_MAPMATCHING_FASTMAPMATCHING
#define PYLOOPS_MAPMATCHING_FASTMAPMATCHING
#include <PyLoops/PyLoops.inc.h>

namespace LoopsAlgs::MapMatching
{
    class FastMapMatching;
}

namespace PyLoops::MapMatching{
    class FastMapMatching{
        void multithreadedMapmatchSet(const LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& input,
            const LoopsAlgs::MapMatching::FastMapMatching& mapMatcher,
            int threadNum,
            const std::string& outputFile
            );
    public:
        static void registerPy(pybind11::module& mod);
    };
}
#endif