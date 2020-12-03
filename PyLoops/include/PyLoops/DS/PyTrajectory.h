#ifndef PYLOOPS_BINDINGS_DS_TRAJECTORY_H
#define PYLOOPS_BINDINGS_DS_TRAJECTORY_H
#include <PyLoops/PyLoops.inc.h>
#include <LoopsLib/DS/EmbeddedGraph.h>
#include "IO/FmmMMTrajectoryVisitor.h"

namespace PyLoops::ds
{
    class PyTrajectory
    {
    public:
        static void registerPy(pybind11::module& mod);
    };
    class TrajectoryList
    {
        std::vector<FMM::CORE::Trajectory> m_trajectories;
    public:
        const std::vector<FMM::CORE::Trajectory>& trajectories() const
        {
            return m_trajectories;
        }
        static void registerPy(pybind11::module& mod);
    };
    class TrajectorySet
    {
    public:
        static void registerPy(pybind11::module & mod);
    };
    class TimestampedTrajectorySet
    {
    public:
        static void registerPy(pybind11::module & mod);
    };
    class GraphTrajectorySet
    {
    public:
        static void registerPy(pybind11::module & mod);
    };
}
#endif