#ifndef PYLOOPS_DS_PYFIELD_H
#define PYLOOPS_DS_PYFIELD_H
#include <pybind11/pybind11.h>
namespace PyLoops::ds
{
    class PyField
    {
    public:
        static void registerPy(pybind11::module& mod);
    };
}
#endif