#include <PyLoops/Bindings/frechet/PyStrongFrechetGraphData.h>
#include <pybind11/stl.h>

void PyLoops::frechet::PyStrongFrechetGraphData::registerPy(pybind11::module& mod)
{
    namespace py = pybind11;
    using Interval = LoopsLib::Geometry::Interval;
    pybind11::class_<Interval>(mod, "Interval")
        .def(py::init<LoopsLib::NT, LoopsLib::NT>())
        .def(py::init<>())
        .def_readwrite("min", &Interval::min)
        .def_readwrite("max", &Interval::max)
        .def("isEmpty", &Interval::isEmpty);

    using GD = LoopsAlgs::Frechet::StrongFrechetGraphData;
    pybind11::class_<GD>(mod, "StrongFrechetGraphData")
        .def(pybind11::init<>())
        .def("setup", [](GD& gd, LoopsLib::DS::EmbeddedGraph* g, const std::vector<LoopsLib::DS::BaseGraph::Id_t>& path)
        {
            gd.setup(g, path);
        })
        .def("compute", [](GD& gd, LoopsLib::NT epsilon)
        {
            LoopsAlgs::Frechet::FrechetGraphComputations computer(epsilon);
            computer.computeFDi(gd);
            computer.computeLeftRightPointersBF(gd);
        })
            .def("epsilon", [](const GD& gd) {return gd.m_epsilon; })
    .def("lrPointer", [](const GD& gd, std::size_t vert, std::size_t wiIndex, std::size_t targetVert)
    {
            auto v0 = gd.m_view.indexForVertex(vert);
            auto v1 = gd.m_view.indexForVertex(targetVert);
            auto pntr = gd.lrPointers[v0].at((std::size_t)gd.WhiteIntervals[v0].at(wiIndex).min).at(targetVert);
            return pntr.pointers;
    })
    .def("cellIntervalsForEdgeGlobalVs", [](const GD& gd, std::size_t startVert, std::size_t endVert)
    {
        if(!gd.m_view.isAvailable(startVert) || !gd.m_view.isAvailable(endVert))
        {
            throw std::invalid_argument("One of the vertices is not in the view");
        }
        std::vector<std::vector<Interval>> returnValues; // Intervals at start vertex, intervals betweenn, intervals at end vertex
        returnValues.push_back(gd.FDiWhiteIntervals.at(gd.m_view.indexForVertex(startVert)));
        // Compute between values
        std::vector<Interval> between;
        auto v0 = gd.m_graph->locations()[startVert];
        auto v1 = gd.m_graph->locations()[endVert];
        for(std::size_t i = 0; i < gd.polylineEdgeCount(); ++i)
        {
            auto res = LoopsAlgs::Frechet::FrechetGraphComputations::computeIntersectionInterval(v0, v1, gd.polyline[i].get().vertex(0), gd.m_epsilon);
            between.push_back(res.first);
        }
        returnValues.push_back(between);
        returnValues.push_back(gd.FDiWhiteIntervals.at(gd.m_view.indexForVertex(endVert)));
        return returnValues;
    })
    .def("whiteIntervalCount",[](const GD& gd, std::size_t localVertex)
    {
            return gd.WhiteIntervals.at(localVertex).size();
    })
        .def("whiteInterval", [](const GD& gd, std::size_t localVertex, std::size_t index)
    {
            return gd.WhiteIntervals.at(localVertex).at(index);
    }).def("lrPointer", [](const GD& gd, std::size_t localVertex, std::size_t cell, std::size_t targetLocalVert)
    {
        return gd.lrPointers.at(localVertex).at(cell).at(targetLocalVert).pointers;
    })
    .def("indexForVert", [](const GD& gd, LoopsLib::DS::BaseGraph::Id_t vId)
    {
        return gd.m_view.indexForVertex(vId);
    })
    .def("vertexAt", [](const GD& gd, std::size_t index)
    {
        return gd.m_view.vertexByIndex(index)->id();
    })
    .def("viewVertices", [](const GD& gd)
    {
        return gd.m_view.availableVertices();
    },py::return_value_policy::copy);
}
