# Getting Started

[Back to README](../README.md)

This guide is for a first local checkout. For complete build details and platform requirements, see [building.md](building.md).

## 1. Clone

Clone with submodules:

```bash
git clone --recursive https://github.com/edisonlee0212/EvoEngine.git
cd EvoEngine
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

## 2. Build And Install Apps On Windows

Windows is the primary development platform.

```bat
python Scripts\build_project.py
python Scripts\install_apps.py
```

The installed runtime is written to:

```text
out/install/vs2026-x64/bin/
```

Launch:

```text
out/install/vs2026-x64/bin/EvoEngineLauncher.exe
```

## 3. Open A Project

Use the launcher to create or open a project. To start the editor directly:

```bat
out\install\vs2026-x64\bin\EvoEngineEditor.exe --project path\to\project.eveproj
```

Projects use `.eveproj` files. Project metadata can describe the application name, preferred editor, and runtime packages that should load when the project opens.

## 4. Run Tests

Local render/GPU tests:

```bat
python Scripts\test.py
```

All CTest tests:

```bat
python Scripts\test.py --all
```

See [testing.md](testing.md) for test labels, render artifacts, and CI expectations.

## 5. Learn The Codebase

Start with these docs:

- [architecture.md](architecture.md) for the SDK, app lifecycle, ECS, renderer, jobs, and editor model
- [projects-assets-serialization.md](projects-assets-serialization.md) for project files, assets, metadata, staged loading, and serialization
- [runtime-packages.md](runtime-packages.md) for package build/load/reload behavior
- [extending-evoengine.md](extending-evoengine.md) for choosing components, systems, assets, layers, Services, packages, or Python bindings
