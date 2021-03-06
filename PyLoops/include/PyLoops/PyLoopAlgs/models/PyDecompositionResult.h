#ifndef PYLOOPS_BINDINGS_PYDECOMPOSITIONRESULT_H
#define PYLOOPS_BINDINGS_PYDECOMPOSITIONRESULT_H
#include <pybind11/pybind11.h>
#include "Models/DecompositionResult.h"
#include <Algs/Types.h>

namespace PyLoops::models
{
    class PyDecompositionResult
    {
    public:
        static void registerPy(pybind11::module& mod);
        static std::vector<std::vector<LoopsLib::NT>> computeFrechetTable(const LoopsLib::Models::DecompositionResult& result);

        static std::vector<LoopsLib::NT> computeMinFrechet(const LoopsLib::Models::DecompositionResult& result);
    };
}
#endif