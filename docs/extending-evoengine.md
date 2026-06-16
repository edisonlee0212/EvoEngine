# Extending EvoEngine

[Back to README](../README.md)

Use the smallest extension point that matches the feature.

| Need | Add |
| --- | --- |
| Entity behavior with lifecycle hooks, editor UI, serialization, or asset references | Private component |
| High-volume simulation data with chunked ECS iteration | Data component |
| Scene-level behavior independent of one component instance | System |
| Reusable project data that should be referenced and serialized | Asset |
| Global application behavior, UI, render hooks, or input hooks | Layer |
| Build-time module outside the SDK | Service |
| Loadable/reloadable domain feature | Runtime package |
| Scripted workflow or automation entry point | Python binding |

## Components

Use data components when memory layout and parallel iteration matter most. Use private components when a feature needs editor UI, lifecycle methods, polymorphism, serialized state, or asset references.

## Systems

Add a system when behavior should run over a scene independently of one component instance. Systems can operate on scene state and component queries and are useful for simulation or global update logic.

## Assets

Add an asset when data should be reusable, referenceable, and stored in projects. Persistent assets need type registration, serialization, editor inspection when useful, and asset-reference collection/relink support when they store asset or entity handles.

## Layers

Add a layer when behavior is global to the application or needs top-level UI, render, lifecycle, or input hooks.

## Services

Add a Service when the feature is a build-time dependency and should remain outside the SDK. Services live under `EvoEngine_Services` and are documented in [EvoEngine_Services/README.md](../EvoEngine_Services/README.md).

## Runtime Packages

Add a runtime package when the feature should be loaded, unloaded, rebuilt, or reloaded independently from a running app. Packages live under `EvoEngine_Packages` and are documented in [runtime-packages.md](runtime-packages.md) and [EvoEngine_Packages/README.md](../EvoEngine_Packages/README.md).

## Python Bindings

Add a Python binding when a workflow should run from scripts or external automation. See [python-bindings.md](python-bindings.md).
