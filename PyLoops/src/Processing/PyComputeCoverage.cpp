#include <PyLoops/Processing/PyComputeCoverage.h>
#include <LoopsAlgs/Trajectories/Coverage.h>
void PyLoops::Processing::PyComputeCoverage::registerPy(pybind11::module& mod)
{
    namespace py = pybind11;

    using Cov = LoopsAlgs::Trajectories::Coverage;
    using Kernel = LoopsLib::MovetkGeometryKernel;

    py::class_<LoopsAlgs::Trajectories::Coverage>(mod, "Coverage")
    .def(py::init<>())
    .def_property("frechetTolerance",&Cov::frechetTolerance, &Cov::setFrechetTolerance)
    .def("computeCoverage",[](const Cov& cov, const Kernel::TrajectorySet& centers, const Kernel::TrajectorySet& coverSet, LoopsLib::NT epsilon,
        const std::string& cachePath)
    {
        LoopsLib::NT fraction = 0.0;
        bool success = cov.computeUpperBounded(centers, coverSet, cachePath, epsilon, fraction);
        return std::make_tuple(success, fraction);
    });
}