# Python Bindings

[Back to README](../README.md)

`PythonBinding` builds pybind11 modules for automation workflows.

The current Python binding layer focuses on core scripts and bindings that do not depend on runtime package C++ APIs. Package-specific Python C++ APIs for EcoSysLab and DigitalAgriculture are currently disabled while those domains move to runtime packages.

Example scripts live in `PythonBinding`. Package-level Python APIs should be added through a dedicated dynamic package interface later, so package behavior remains compatible with runtime package loading and unloading.

Build and install output is documented in [building.md](building.md). Local render tests that exercise Python capture workflows are documented in [testing.md](testing.md).
