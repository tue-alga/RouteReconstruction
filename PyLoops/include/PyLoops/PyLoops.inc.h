#ifndef PYLOOPS_PYLOOPSOPAQUETYPES_H
#define PYLOOPS_PYLOOPSOPAQUETYPES_H
#include <pybind11/pybind11.h>
#include <LoopsLib/Algs/Types.h>

PYBIND11_MAKE_OPAQUE(std::pair<LoopsLib::MovetkGeometryKernel::MovetkPoint, LoopsLib::NT>)

#endif