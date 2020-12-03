#ifndef PYLOOPS_BINDINGS_PYPROBLEMINSTANCE_H
#define PYLOOPS_BINDINGS_PYPROBLEMINSTANCE_H
#include <PyLoops/PyLoops.inc.h>
#include <pybind11/stl.h>
#include <LoopsLib/Models/ProblemInstance.h>
namespace PyLoops::models
{
    class PyProblemInstance
    {
    public:
        static void registerPy(pybind11::module& mod);
    };
}
#endif