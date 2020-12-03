#ifndef PYLOOPS_BINDINGS_DECOMPOSITIONOBJECT_H
#define PYLOOPS_BINDINGS_DECOMPOSITIONOBJECT_H
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <LoopsLib/Helpers/DecompositionObject.h>
namespace PyLoops::Bindings
{
    class PyDecompositionObject
    {
        LoopsLib::Helpers::DecompositionObject m_obj;
    public:
        static void registerPy(pybind11::module& mod);
    };
}
#endif