#ifndef PYLOOPS_H
#define PYLOOPS_H
#include <PyLoops/PyLoops.inc.h>

#include "Movetk/GeometryBindings.h"
#include "DS/PyGraph.h"
#include "DS/PyField.h"
#include "Loops/PyDecompositionObject.h"
#include "Loops/PyDecompositionAlgs.h"
#include "IO/WkbCsvTrajectoryReader.h"
#include "Models/PyProblemInstance.h"
#include "Models/PyDecompositionResult.h"
#include "Models/PyProblemInstance.h"
#include "Processing/fmm.h"
#include "Processing/PySpeedBoundedFmm.h"
#include "Processing/PyTrajectoryChecker.h"
#include "Models/PyLongDouble.h"
#include "DS/PyTrajectory.h"
#include "Frechet/PyStrongFrechetGraphData.h"
#include "Loops/FieldGen.h"
#include <PyLoops/MapMatching/FastMapMatching.h>
// IPC
#include "PyLoops/Gui/GuiIpc.h"

#include <PyLoops/Processing/PyComputeCoverage.h>
#include "Processing/TrajectoryProcessing.h"

namespace PyLoops
{
    namespace detail
    {
        template<typename...Types>
        void registerPyTypes(pybind11::module& mod)
        {
            (Types::registerPy(mod), ...);
        }
    }
    inline void registerPyLoops(pybind11::module& mod)
    {
        // Register movetk objects.
        detail::registerPyTypes<
            PyLoops::models::PyLongDouble,
            PyLoops::movetk::PyMovetkPoint,
            PyLoops::ds::PyGraph,
            PyLoops::ds::PyField,
            // Trajectory sets
            PyLoops::ds::TrajectorySet,
            PyLoops::ds::TimestampedTrajectorySet,
            PyLoops::ds::GraphTrajectorySet,
            // Loops models
            PyLoops::models::PyDecompositionResult,
            PyLoops::models::PyProblemInstance,
            PyLoops::io::PyWkbCsvTrajectoryReader,
            // The decomposition algorithms
            PyLoops::Bindings::PyDecompositionAlgs,
            PyLoops::processing::PyUbodt, // Deprecated
            PyLoops::processing::PyFmm,// Deprecated
            PyLoops::processing::PySpeedBoundedFmm,// Deprecated
            PyLoops::ds::PyTrajectory,// Deprecated
            PyLoops::ds::TrajectoryList,// Deprecated
            PyLoops::processing::PyTrajectoryChecker,
            PyLoops::frechet::PyStrongFrechetGraphData,
            PyLoops::MapMatching::FastMapMatching,
            //
            PyLoops::Gui::GuiIpc

        >(mod);

        auto pyGenerateMod = mod.def_submodule("generate");

        detail::registerPyTypes<
            PyLoops::loops::PyFieldGenerator
        >(pyGenerateMod);

        // Submodule for processing
        auto processingMod = mod.def_submodule("processing");
        detail::registerPyTypes<
            processing::TrajectoryProcessing,
            Processing::PyComputeCoverage
        >(processingMod);


        mod.doc() = R"pbdoc(
        Loops python module
        -----------------------
        .. currentmodule:: PyLoops
        .. autosummary::
           :toctree: _generate
    )pbdoc";

#ifdef VERSION_INFO
        mod.attr("__version__") = VERSION_INFO;
#else
        mod.attr("__version__") = "dev";
#endif
    }
}

#endif