#include "PyDecompositionResult.h"
#include <LoopsIO/GraphIO.h>
#include <LoopsIO/DecompositionResultSerializer.h>
#include <LoopsIO/DecompositionResultProviders/BoostProviders.h>
#include <LoopsIO/FieldIO.h>
#include <pybind11/stl.h>
#include <Models/DecompositionResult.h>
#include "Algs/Processing/TrajectorySetFrechet.h"

void PyLoops::models::PyDecompositionResult::registerPy(pybind11::module& mod)
{
    LoopsIO::DecompositionResultSerializer::registerProvider(new LoopsIO::DecompositionResultProviders::BoostBinProvider());
    LoopsIO::DecompositionResultSerializer::registerProvider(new LoopsIO::DecompositionResultProviders::BoostTxtProvider());

    using DC = LoopsLib::Models::DecompositionResult;
    namespace py = pybind11;
    pybind11::class_<DC>(mod, "DecompositionResult")
    //.def("coefficients",&DC::m_decompositionCoeffs)
    //.def("basis",&DC::m_basis)
    .def("basisSize", [](DC& dc) {return dc.m_basis.size(); })
    .def("basisElement", [](DC& dc, std::size_t index)
    {
        return dc.m_basis.at(index);
    })
    .def("write",[](DC& decompObj, const std::string& outputPath)
    {
        LoopsIO::DecompositionResultSerializer::write(outputPath, decompObj);
    })
    .def("read", [](DC& decompObj, const std::string& outputPath)
    {
        LoopsIO::DecompositionResultSerializer::read(outputPath, decompObj);
    })
        //.def("nnlsValue", &DC::m_objectValueNNLS)
        .def("problemInstance", [](DC& dc) {return dc.m_relatedInstance; }, py::return_value_policy::reference)
    .def("computeFrechetTable", [](DC& dc)
    {
        return PyDecompositionResult::computeFrechetTable(dc);
    })
    .def("computeMinFrechet", [](DC& dc) -> std::vector<LoopsLib::NT>
    {
        return PyDecompositionResult::computeMinFrechet(dc);
    });
}

std::vector<std::vector<LoopsLib::NT>> PyLoops::models::PyDecompositionResult::computeFrechetTable(const LoopsLib::Models::DecompositionResult& result)
{
    LoopsLib::Algs::Processing::TrajectorySetFrechet setFrechet;
    std::vector<std::vector<LoopsLib::NT>> out;
    setFrechet.frechetTable(result.m_relatedInstance->m_graph, result.m_relatedInstance->m_field->m_paths, result.m_basis, out);
    return out;
}

std::vector<LoopsLib::NT> PyLoops::models::PyDecompositionResult::computeMinFrechet(const LoopsLib::Models::DecompositionResult& result)
{
    LoopsLib::Algs::Processing::TrajectorySetFrechet setFrechet;
    std::vector<LoopsLib::NT> out;
    setFrechet.minFrechetValues(result.m_relatedInstance->m_graph, result.m_relatedInstance->m_field->m_paths, result.m_basis, out);
    return out;
}
