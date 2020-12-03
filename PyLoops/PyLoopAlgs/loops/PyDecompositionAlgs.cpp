#include "PyDecompositionAlgs.h"
//#include <pybind11/iostream.h>
PyLoops::Bindings::DecompositionException::DecompositionException(const std::string& msg): m_msg(msg)
{
}

char const* PyLoops::Bindings::DecompositionException::what() const noexcept
{
    return m_msg.c_str();
}

LoopsLib::Models::DecompositionResult PyLoops::Bindings::PyDecompositionAlgs::runAlg(
    LoopsAlgs::FlowDecomposition::IFlowDecomposition* alg)
{

    verifyDecomposable();

    LoopsLib::Models::DecompositionResult res;
    res.m_relatedInstance = m_obj;
    alg->setDecompositionObject(&res);
    alg->decompose();
    return res;
}

PyLoops::Bindings::PyDecompositionAlgs::PyDecompositionAlgs(LoopsLib::Models::ProblemInstance* obj):
    m_obj(obj),
    m_frechetHitting(nullptr),
    m_hitting(nullptr),
    m_weightedFrechet(nullptr)
{
}

void PyLoops::Bindings::PyDecompositionAlgs::verifyDecomposable()
{
    if (m_obj->m_field->pathCount() == 0) throw DecompositionException("No field set");
    if (m_obj->m_availablePaths.empty()) throw DecompositionException("No available paths set");
}


void PyLoops::Bindings::PyDecompositionAlgs::registerPy(pybind11::module& mod)
{
    namespace py = pybind11;
    // Register custom exception class.
    py::register_exception<DecompositionException>(mod, "DecompositionException");


    pybind11::class_<PyDecompositionAlgs>(mod, "DecompositionAlgs")
        .def(py::init<LoopsLib::Models::ProblemInstance*>())
        .def("runHittingPaths", [](PyDecompositionAlgs& algs)
        {
            return algs.runAlg(&algs.m_hitting);
        })
        .def("runFrechetHittingPaths", [](PyDecompositionAlgs& algs, LoopsLib::NT epsilon)
        {
            algs.m_frechetHitting.setEpsilon(epsilon);
            return algs.runAlg(&algs.m_frechetHitting);
            
        }, py::return_value_policy::move)
        .def("runWeightedFrechetPaths", [](PyDecompositionAlgs& algs, LoopsLib::NT epsilon, int iterations, bool noDataCaching)
        {
            // py::scoped_ostream_redirect stream(
            //     std::cout,                               // std::ostream&
            //     py::module::import("sys").attr("stdout") // Python output
            // );
            algs.m_weightedFrechet.setEpsilon(epsilon);
            algs.m_weightedFrechet.setNumberOfIterations(iterations);
            algs.m_weightedFrechet.setNoDataCaching(noDataCaching);
            return algs.runAlg(&algs.m_weightedFrechet);
        });
}
