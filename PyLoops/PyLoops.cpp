#include <PyLoops/PyLoops.h>

PYBIND11_MODULE(PyLoops, m) {
    // Define graph classes
    PyLoops::registerPyLoops(m);
}