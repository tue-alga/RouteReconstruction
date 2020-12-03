#include <PyLoops/Bindings/models/PyProblemInstance.h>
#include <LoopsIO/GraphIO.h>
#include <LoopsIO/DecompositionProviders.h>
#include <LoopsIO/DecompositionIO.h>
#include <LoopsIO/MapProviders.h>
#include <LoopsIO/ProblemInstanceSerializer.h>
#include <LoopsLib/Models/FlowField.h>
#include <LoopsIO/ProblemInstanceProviders/BoostProviders.h>

void PyLoops::models::PyProblemInstance::registerPy(pybind11::module& mod)
{
    // Register all providers

    LoopsIO::ProblemInstanceSerializer::registerProvider(new LoopsIO::ProblemInstanceProviders::BoostBinProvider());
    LoopsIO::ProblemInstanceSerializer::registerProvider(new LoopsIO::ProblemInstanceProviders::BoostTxtProvider());


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
    .def("setField", [](PrI& problInst, LoopsLib::Models::FlowField* field, const std::string& filePath)
    {
        problInst.m_field = field;
        problInst.m_savePaths.fieldPath = filePath;
    })
    .def_readwrite("epsilon", &PrI::m_epsilon)
    .def("setAvailablePaths",[](PrI& probInst, const std::vector<LoopsLib::DS::BaseGraph::Id_t>& ids)
    {
        // TODO: validate?
        probInst.setRepresentativesFromPaths(ids);
    })
    .def("pickRandomAvailablePaths", &PrI::pickRandomAvailablePaths);
}
