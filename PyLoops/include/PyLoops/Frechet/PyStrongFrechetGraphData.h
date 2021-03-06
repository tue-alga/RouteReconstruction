#ifndef PYLOOPS_FRECHET_PYSTRONGFRECHETGRAPHDATA_H
#define PYLOOPS_FRECHET_PYSTRONGFRECHETGRAPHDATA_H
#include <PyLoops/PyLoops.inc.h>
#include <LoopsAlgs/Frechet/FrechetHelpers.h>

namespace PyLoops::frechet
{
    class PyStrongFrechetGraphData
    {
        char m_separator = ';';
    public:
        static void registerPy(pybind11::module& mod);
    };
}
#endif 