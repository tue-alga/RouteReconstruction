#ifndef PYLOOPS_DS_PYFIELD_H
#define PYLOOPS_DS_PYFIELD_H
#include <PyLoops/PyLoops.inc.h>
namespace PyLoops::ds
{
    class PyField
    {
    public:
        static void registerPy(pybind11::module& mod);
    };
}
#endif