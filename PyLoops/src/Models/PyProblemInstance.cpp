#include <PyLoops/Models/PyProblemInstance.h>
#include <LoopsIO/GraphIO.h>
#include <LoopsIO/DecompositionProviders.h>
#include <LoopsIO/DecompositionIO.h>
#include <LoopsIO/MapProviders.h>
#include <LoopsIO/ProblemInstanceSerializer.h>
#include <LoopsIO/TrajectorySetSerializer.h>
#include <LoopsLib/Models/FlowField.h>
#include <LoopsIO/ProblemInstanceProviders/BoostProviders.h>
#include <LoopsIO/ProblemInstanceProviders/IpePIProvider.h>
#include <pybind11/iostream.h>

void PyLoops::models::PyProblemInstance::registerPy(pybind11::module& mod)
{
    // Register all providers

    LoopsIO::ProblemInstanceSerializer::registerProvider(new LoopsIO::ProblemInstanceProviders::BoostBinProvider());
    LoopsIO::ProblemInstanceSerializer::registerProvider(new LoopsIO::ProblemInstanceProviders::BoostTxtProvider());
    LoopsIO::ProblemInstanceSerializer::registerProvider(new LoopsIO::ProblemInstanceProviders::IpePIProvider());

    namespace py = pybind11;

    using PrI = LoopsLib::Models::ProblemInstance;
    pybind11::class_<PrI>(mod, "ProblemInstance")
    .def(pybind11::init<>())
    // IO
    .def("read", [](PrI& probInst, const std::string& filePath)
    {
        LoopsIO::ProblemInstanceSerializer::read(filePath, probInst);
    })
    .def("write", [](PrI& probInst, const std::string& filePath)
    {
        LoopsIO::ProblemInstanceSerializer::write(filePath, probInst);
    })
    // Access
    .def("fieldFile", [](PrI& pr) {return pr.m_savePaths.fieldPath; })
    .def("graphFile", [](PrI& pr) {return pr.m_savePaths.graphPath; })
    .def("field", [](PrI& pr){ return pr.m_field; }, pybind11::return_value_policy::reference)
    .def("graph", [](PrI& pr) { return pr.m_graph; }, pybind11::return_value_policy::reference)
    .def("setGraph", [](PrI& probInst, LoopsLib::DS::EmbeddedGraph* graph, const std::string& graphFilePath)
    {
        probInst.m_graph = graph;
        probInst.m_savePaths.graphPath = graphFilePath;
    })
    .def("representative", [](const PrI& probInst, int index)
    {
        return probInst.m_representatives[index];
    })
    .def("representativeCount", [](const PrI& probInst)
    {
        return probInst.m_representatives.size();
    })
    .def("pathCount", [](const PrI& probInst)
    {
        return probInst.m_field->m_paths.size();
    })
    .def("path", [](const PrI& probInst,int index)
    {
        return probInst.m_field->m_paths[index];
    })
    .def("representativeSource", [](const PrI& probInst, int index)
    {
        auto src = probInst.m_representativeSources[index];
        return src.desc();
    })
    .def("setField", [](PrI& problInst, LoopsLib::Models::FlowField* field, const std::string& filePath)
    {
        problInst.m_field = field;
        problInst.m_savePaths.fieldPath = filePath;
    })
    .def_readwrite("epsilon", &PrI::m_epsilon)
    .def("setAvailablePaths",[](PrI& probInst, const std::vector<LoopsLib::DS::BaseGraph::Id_t>& ids)
    {
        py::scoped_ostream_redirect stream(
            std::cout,                               // std::ostream&
            py::module::import("sys").attr("stdout") // Python output
        );
        pybind11::print("Setting paths");
        probInst.setRepresentativesFromPaths(ids);
        pybind11::print("Setting paths done");
    })
        .def("setAvailablePathsFromFile", [](PrI& probInst, const std::string& representativesFile,const std::vector<LoopsLib::DS::BaseGraph::Id_t>& ids, bool convertCrs)
    {
        py::scoped_ostream_redirect stream(
            std::cout,                               // std::ostream&
            py::module::import("sys").attr("stdout") // Python output
        );
        pybind11::print("Setting paths");
        LoopsLib::MovetkGeometryKernel::TrajectorySet ts;
        LoopsIO::TrajectorySetSerializer::read(representativesFile,ts);

        probInst.setRepresentativesFromTrajectories(ts, ids,convertCrs);
        pybind11::print("Setting paths done");
    },py::arg("representativesFile"),py::arg("ids"), py::arg("convertCrs")=false)
    .def("pickRandomAvailablePaths", &PrI::pickRandomAvailablePaths);
}
