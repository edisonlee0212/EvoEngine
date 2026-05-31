#include "PyEvoEngine.hpp"

namespace py = pybind11;

PYBIND11_MODULE(PyEvoEngine, m) {
  m.doc() = "PyEvoEngine";
  py_evo_engine::PyEvoEngine::Initialize(m);
}
