#ifndef PYLOOPS_PROCESSING_PYCOMPUTECOVERAGE_H
#define PYLOOPS_PROCESSING_PYCOMPUTECOVERAGE_H
#include <PyLoops/PyLoops.inc.h>
#include <LoopsLib/Algs/Types.h>
namespace PyLoops::Processing
{
    class PyComputeCoverage
    {
    public:
        using Kernel = LoopsLib::MovetkGeometryKernel;
        void computeCoverage(const Kernel::TrajectorySet& s0, const Kernel::TrajectorySet& s1, const std::string& cacehPath)
        {
            
        }
        static void registerPy(pybind11::module& mod);
    };
}
#endif