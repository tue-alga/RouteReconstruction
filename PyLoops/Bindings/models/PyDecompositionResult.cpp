#include "PyDecompositionResult.h"
#include <LoopsIO/GraphIO.h>
#include <LoopsIO/DecompositionResultSerializer.h>
#include <LoopsIO/DecompositionResultProviders/BoostProviders.h>
#include <LoopsIO/FieldIO.h>
#include <pybind11/stl.h>
#include <Models/DecompositionResult.h>
#include "Algs/Processing/TrajectorySetFrechet.h"
#include <pybind11/iostream.h>

void PyLoops::models::PyDecompositionResult::registerPy(pybind11::module& mod)
{
    LoopsIO::DecompositionResultSerializer::registerProvider(new LoopsIO::DecompositionResultProviders::BoostBinProvider());
    LoopsIO::DecompositionResultSerializer::registerProvider(new LoopsIO::DecompositionResultProviders::BoostTxtProvider());

    using DC = LoopsLib::Models::DecompositionResult;
    namespace py = pybind11;

    pybind11::class_<DC>(mod, "DecompositionResult")
    .def(py::init<>())
    .def(py::init<LoopsLib::Models::ProblemInstance*>())
    .def("basisSize", [](DC& dc) {return dc.m_basis.size(); })
    .def("basisElement", [](DC& dc, std::size_t index)
    {
        return dc.m_basis.at(index);
    })
    .def("coefficient", [](DC& dc, std::size_t index)
    {
        return dc.m_decompositionCoeffs.at(index);
    })
    // Return available extensions for serialization
    .def_static("availableExtensions", &LoopsIO::DecompositionResultSerializer::availableExtensions)
    // Return objective value of the least squares minimization
    .def("objectiveValue", [](DC& dc) {return dc.m_objectValueNNLS; })
    // Write the result. Throws if extension is not supported.
    .def("write",[](DC& decompObj, const std::string& outputPath)
    {
        LoopsIO::DecompositionResultSerializer::write(outputPath, decompObj);
    })
    .def("applyNNLSAndPrune", [](DC& dc, double pruneValue) {
        pybind11::scoped_ostream_redirect streamRedir(
            std::cout,                               // std::ostream&
            pybind11::module::import("sys").attr("stdout") // Python output
        );
        dc.applyNNLSAndPrune(pruneValue);
    })
    .def("read", [](DC& decompObj, const std::string& outputPath)
    {
        LoopsIO::DecompositionResultSerializer::read(outputPath, decompObj);
    })
    .def("problemInstance", [](DC& dc) {return dc.m_relatedInstance; }, py::return_value_policy::reference)
    .def("setProblemInstance", [](DC& dc, LoopsLib::Models::ProblemInstance* probInst) {dc.m_relatedInstance = probInst; })
    .def("computeFrechetTable", [](DC& dc)
    {
        return PyDecompositionResult::computeFrechetTable(dc);
    })
    .def("computeMinFrechet", [](DC& dc, LoopsLib::NT maximumValue) -> std::vector<LoopsLib::NT>
    {
        return PyDecompositionResult::computeMinFrechet(dc, maximumValue);
    });
}

std::vector<std::vector<LoopsLib::NT>> PyLoops::models::PyDecompositionResult::computeFrechetTable(const LoopsLib::Models::DecompositionResult& result)
{
    LoopsLib::Algs::Processing::TrajectorySetFrechet setFrechet;
    std::vector<std::vector<LoopsLib::NT>> out;
    setFrechet.frechetTable(result.m_relatedInstance->m_graph, result.m_relatedInstance->m_field->m_paths, result.m_basis, out);
    return out;
}

std::vector<LoopsLib::NT> PyLoops::models::PyDecompositionResult::computeMinFrechet(const LoopsLib::Models::DecompositionResult& result, LoopsLib::NT initialHighBound)
{
    LoopsLib::Algs::Processing::TrajectorySetFrechet setFrechet;
    std::vector<LoopsLib::NT> out;
    setFrechet.minFrechetValues(result.m_relatedInstance->m_graph, result.m_relatedInstance->m_field->m_paths, result.m_basis, initialHighBound, out);
    return out;
}
