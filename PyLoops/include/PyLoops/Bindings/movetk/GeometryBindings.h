#ifndef PYLOOPS_BINDINGS_MOVETK_GEOMETRYBINDINGS_H
#define PYLOOPS_BINDINGS_MOVETK_GEOMETRYBINDINGS_H
#include <LoopsLib/Algs/Types.h>
#include <pybind11/pybind11.h>
#include <movetk/geom/GeometryInterface.h>
namespace PyLoops::movetk
{
    class PyMovetkPoint
    {
        LoopsLib::MovetkGeometryKernel::MovetkPoint m_point;
    public:
        PyMovetkPoint()
        {
            m_point = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>()({ 0,0 });
        }
        LoopsLib::NT getX()const
        {
            return m_point.get().x();
        }
        LoopsLib::NT getY()const
        {
            return m_point.get().y();
        }
        static void registerPy(pybind11::module& mod)
        {
            pybind11::class_<PyMovetkPoint>(mod,"Point")
            .def("x", &PyMovetkPoint::getX)
            .def("y", &PyMovetkPoint::getY)
            .def(pybind11::init<>())
            .def(pybind11::init([](LoopsLib::NT x, LoopsLib::NT y)
            {
                auto* pnt = new PyMovetkPoint();
                pnt->m_point = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>()({ x,y });
                return pnt;
            }));
        }
    };
}
#endif