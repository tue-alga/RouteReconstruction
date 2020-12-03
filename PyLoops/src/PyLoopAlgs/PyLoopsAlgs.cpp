#include <pybind11/pybind11.h>
#include "DS/OwnGraph.h"
#include "Algs/Types.h"
#include "Bindings/movetk/GeometryBindings.h"
#include "Bindings/ds/PyGraph.h"
#include "Bindings/ds/PyField.h"
#include "Bindings/io/WkbCsvTrajectoryReader.h"
#include "models/PyProblemInstance.h"
#include "models/PyDecompositionResult.h"
#include "Bindings/processing/fmm.h"
#include "Bindings/processing/PySpeedBoundedFmm.h"
#include "Bindings/models/PyLongDouble.h"

namespace py = pybind11;

template<typename...Types>
void registerPyTypes(pybind11::module& mod)
{
    (Types::registerPy(mod), ...);
}

PYBIND11_MODULE(PyLoopsAlgs, m) {
    // Define graph classes

    // Register datastructures
    //py::class_<DS::BaseGraph>(m, "Graph");

    // Register movetk objects.
    registerPyTypes<
        PyLoops::movetk::PyMovetkPoint,
        PyLoops::ds::PyGraph,
        PyLoops::models::PyLongDouble,
        PyLoops::ds::PyField,
        PyLoops::models::PyDecompositionResult,
        PyLoops::models::PyProblemInstance,
        PyLoops::io::PyWkbCsvTrajectoryReader,
        PyLoops::Bindings::PyDecompositionAlgs,
        PyLoops::processing::PyUbodt,
        PyLoops::processing::PyFmm,
        PyLoops::processing::PySpeedBoundedFmm
    >(m);

    m.doc() = R"pbdoc(
        Loops python module
        -----------------------
        .. currentmodule:: PyLoops
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}