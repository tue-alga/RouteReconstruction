#ifndef PYLOOPS_BINDINGS_DECOMPOSITIONALGS_
#define PYLOOPS_BINDINGS_DECOMPOSITIONALGS_
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Helpers/DecompositionObject.h"
#include <LoopsAlgs/FlowDecomposition.h>
#include "Algs/Processing/TrajectorySetFrechet.h"

namespace PyLoops::Bindings
{
    /**
     * \brief Custom exception for describing decomposition errors
     */
    class DecompositionException : public std::exception
    {
        std::string m_msg;
    public:
        DecompositionException(const std::string& msg);

        char const* what() const noexcept override;
    };

    class PyDecompositionAlgs
    {
        LoopsLib::Models::ProblemInstance* m_obj = nullptr;
        // Algs
        LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition m_frechetHitting;
        LoopsAlgs::FlowDecomposition::HittingPathsDecomposition m_hitting;
        LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition m_weightedFrechet;

        LoopsLib::Models::DecompositionResult runAlg(LoopsAlgs::FlowDecomposition::IFlowDecomposition* alg);
    public:
        PyDecompositionAlgs(LoopsLib::Models::ProblemInstance* obj);

        void verifyDecomposable();
        
        static void registerPy(pybind11::module& mod);
    };
}
#endif