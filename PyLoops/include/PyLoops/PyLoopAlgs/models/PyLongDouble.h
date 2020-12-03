#ifndef PYLOOPS_BINDINGS_PYLONGDOUBLE_H
#define PYLOOPS_BINDINGS_PYLONGDOUBLE_H
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
namespace PyLoops::models
{
    class PyLongDouble
    {
    public:
        static void registerPy(pybind11::module& mod)
        {
            namespace  py = pybind11;
            py::class_<long double>(mod, "LDouble")
                .def(py::init<>())
                .def(py::init<double>())
                .def(py::self + py::self)
                .def(py::self + double())
                .def(py::self - py::self)
                .def(py::self * py::self)
                .def(py::self / py::self)
                .def(py::self *= py::self)
                .def(py::self /= py::self)
                .def(py::self += py::self)
                .def(py::self -= py::self);
        }
    };
}
#endif