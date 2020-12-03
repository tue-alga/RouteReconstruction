#ifndef PYLOOPS_BINDINGS_PYPROBLEMINSTANCE_H
#define PYLOOPS_BINDINGS_PYPROBLEMINSTANCE_H
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Models/ProblemInstance.h"
namespace PyLoops::models
{
    class PyProblemInstance
    {
    public:
        static void registerPy(pybind11::module& mod);
    };
}
#endif