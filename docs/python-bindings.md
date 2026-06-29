# Python Bindings

[Back to README](../README.md)

`PythonBinding` builds pybind11 modules for automation workflows.

The current Python binding layer focuses on core scripts and bindings that do not depend on runtime package C++ APIs. `PyDigitalAgriculture` is enabled for sorghum automation workflows that still use the existing static package linkage, including field creation and field statistics validation.

Example scripts live in `PythonBinding`. New package-level Python APIs should still prefer a dedicated dynamic package interface later, so package behavior remains compatible with runtime package loading and unloading.

Build and install output is documented in [building.md](building.md). Local render tests that exercise Python capture workflows are documented in [testing.md](testing.md).
