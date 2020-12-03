#include <PyLoops/Bindings/ds/PyField.h>
#include <LoopsLib/Models/FlowField.h>
#include <LoopsIO/FieldIO.h>
#include <LoopsIO/FieldProviders.h>
#include <pybind11/stl.h>
void PyLoops::ds::PyField::registerPy(pybind11::module& mod)
{
    LoopsIO::FieldIO::registerProvider(new LoopsIO::FieldProviders::TxtFieldProvider());
    LoopsIO::FieldIO::registerProvider(new LoopsIO::FieldProviders::JsonFieldProvider());

    namespace py = pybind11;
    using FF = LoopsLib::Models::FlowField;
    py::class_<LoopsLib::Models::FlowField>(mod, "FlowField")
    .def(py::init<>())
    .def("value",[](FF& field, LoopsLib::DS::BaseGraph::Id_t edgeId)
    {
        return field.m_data[edgeId];
    })
    .def("graphFile", [](FF& ff) {return ff.m_graphFile; })
    .def("read", [](FF& field, const std::string& filePath)
    {
        LoopsIO::FieldIO::read(filePath, field);
    })
        .def("write", [](FF& field, const std::string& filePath)
    {
        LoopsIO::FieldIO::write(filePath, field);
    })
    .def("path",[](FF& field, std::size_t pathInd)
    {
        return field.m_paths[pathInd];
    })
    .def("byteEstimate",[](FF& field)
    {
        std::size_t totalBytes = 0;

        std::size_t pElSize = sizeof(decltype(field.m_paths[0][0]));
        
        for(const auto& p : field.m_paths)
        {
            totalBytes += p.size() * pElSize;
        }
        totalBytes += field.m_paths.size() * sizeof(decltype(field.m_pathValues[0]));
        return totalBytes;
    })
    .def("setFromPaths", &FF::setFieldFromPaths)
    .def("squareLength", [](FF& ff)
    {
        LoopsLib::NT total = 0;
        std::for_each(ff.m_data.begin(), ff.m_data.end(),[&total](LoopsLib::NT val)
        {
            total += val * val;
        });
        return total;
    })
    .def("pathCount", &FF::pathCount)
    .def("fieldLength", [](FF& ff) {return ff.m_data.size(); })
    .def("minValue", [](FF& ff)
    {
        if (ff.m_data.size() == 0) return (LoopsLib::NT)0;
        return *std::min_element(ff.m_data.begin(), ff.m_data.end());
    })
    .def("maxValue", [](FF& ff)
    {
        if (ff.m_data.size() == 0) return (LoopsLib::NT)0;
        return *std::max_element(ff.m_data.begin(), ff.m_data.end());
    })
    .def("avgValue", [](FF& ff)
    {
        if (ff.m_data.size() == 0) return (LoopsLib::NT)0;
        LoopsLib::NT init = 0;
        return std::accumulate(ff.m_data.begin(), ff.m_data.end(),init) / (LoopsLib::NT)(ff.m_data.size());
    })
    .def("pathValue", &FF::pathValue);
}
