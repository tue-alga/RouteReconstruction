#include <PyLoops/Loops/PyDecompositionObject.h>
#include <LoopsIO/GraphIO.h>
#include <LoopsIO/DecompositionProviders.h>
#include <LoopsIO/DecompositionIO.h>
#include <LoopsIO/MapProviders.h>
#include <LoopsIO/FieldProviders.h>
#include <LoopsIO/FieldIO.h>
#include <pybind11/iostream.h>

namespace IO = LoopsIO;

void PyLoops::Bindings::PyDecompositionObject::registerPy(pybind11::module& mod)
{
    // Register all providers
    ::IO::GraphIO::registerProvider(new IO::MapProviders::MapTxtMapProvider());
    ::IO::GraphIO::registerProvider(new IO::MapProviders::JsonMapProvider());
    ::IO::GraphIO::registerProvider(new IO::MapProviders::OsmXMLMapProvider());
    ::IO::GraphIO::registerProvider(new IO::MapProviders::BoostBinaryProvider());
    ::IO::FieldIO::registerProvider(new IO::FieldProviders::TxtFieldProvider());
    ::IO::FieldIO::registerProvider(new IO::FieldProviders::JsonFieldProvider());
    ::IO::DecompositionIO::registerProvider(new IO::DecompositionProviders::TxtDecompositionProvider());
    ::IO::DecompositionIO::registerProvider(new IO::DecompositionProviders::JsonDecompositionProvider());

    using DC = Helpers::DecompositionObject;
    namespace py = pybind11;
    pybind11::class_<Helpers::DecompositionObject>(mod, "DecompositionObject")
    
    .def_readwrite("graph", &DC::m_graph)
    .def(py::init([]()
    {
        auto* decompObj = new Helpers::DecompositionObject;
        decompObj->m_graph = new DS::EmbeddedGraph();
        return decompObj;
    }))
    .def("readMap",[](DC& decompObj, const std::string& file)
    {
        ::IO::GraphIO::read(file, *decompObj.m_graph);
    })
    .def_readwrite("field",&DC::m_field)
    .def("readField", [](DC& decompObj, const std::string& file)
    {
        py::scoped_ostream_redirect stream(
            std::cout,                               // std::ostream&
            py::module::import("sys").attr("stdout") // Python output
        );
        auto cb = [&decompObj](const std::string& mapPath)
        {
            ::IO::GraphIO::read(mapPath, *decompObj.m_graph);
            return decompObj.m_graph->number_of_vertices() > 0;
        };
        pybind11::print("Starting read");
        ::IO::FieldIO::read(file, decompObj, cb);
        pybind11::print("Number of paths", decompObj.m_paths.size());
        pybind11::print("Path values", decompObj.m_pathValues.size());
    })
    .def("readDecomposition", [](DC& decompObj, const std::string& file)
    {
        auto cb = [&decompObj](const std::string& mapPath)
        {
            ::IO::GraphIO::read(mapPath, *decompObj.m_graph);
            return !decompObj.m_graphPath.empty();
        };
        auto fieldCb = [&decompObj, cb](const std::string& fieldPath)
        {
            ::IO::FieldIO::read(fieldPath, decompObj, cb);
            return !decompObj.m_fieldPath.empty();
        };
        ::IO::DecompositionIO::read(file, decompObj, cb,fieldCb);
    })
    .def("exportDecomposition",[](DC& decompObj, const std::string& outputPath)
    {
        ::IO::DecompositionIO::write(outputPath, decompObj);
    })
    .def("selectRandomAvailablePaths", &DC::pickRandomAvailablePaths)
    .def("fieldValue", &DC::fieldValue)
    .def("objectiveFunction", [](DC& decompObj)
    {
        decompObj.applyNNLSForCoefficients();
        return decompObj.m_objectValueNNLS;
    })
    .def_property_readonly("mapFilePath",[](DC& decompObj)
    {
        return decompObj.m_graphPath;
    })
    .def_property_readonly("fieldFilePath", [](DC& decompObj)
    {
        return decompObj.m_fieldPath;
    })
    .def("pathCount", [](DC& dc)
    {
        return dc.m_paths.size();
    })
    .def("setAvailablePaths", [](DC& dc, const std::vector<DS::BaseGraph::Id_t>& ids)
    {
        dc.m_availablePaths.clear();
        dc.m_availablePaths.insert(ids.begin(), ids.end());
    });
}
