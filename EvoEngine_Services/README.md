# Service Documentation

[Back to README](../README.md)

This directory documents EvoEngine Services separately from the SDK overview. Each Service page describes the Service's purpose, current build status, important source files, SDK extension points, and notes for future work.

| Service | Documentation |
| --- | --- |
| PhysXPhysics | [PhysXPhysics/README.md](PhysXPhysics/README.md) |

Services are registered from `EvoEngine_Services/CMakeLists.txt`. Default registration is not the same as directory presence: some Service directories are present but disabled or gated by platform/optional SDKs.

Runtime packages such as EcoSysLab, DigitalAgriculture, DatasetGeneration, Universe, BillboardClouds, TextureBaking, and MeshRepair are documented under [EvoEngine_Packages](../EvoEngine_Packages/README.md).
