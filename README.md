# EvoEngine
EvoEngine is an early-stage, cross-platform interactive application and rendering engine for Windows and Linux. 
## Main features
Here are the features that already exist in the EvoEngine.
 - Modularized Design
    - Removable WindowLayer, RenderLayer, EditorLayer, etc.
    - E.g. Offscreen rendering by unloading WindowLayer and EditorLayer.
 - Complete Entity Component System (ECS) 
    - Cache-friendly data component, similar to the ComponentData in Unity Engine.
    - Customizable private component, similar to Component in Unity Engine. 
    - The ECS is heavily used in my other private research projects.
 - Multi-threading
    - The Job System is designed and closely combined with ECS to provide the ability to operate concurrently on multiple data components.
    - Visit Scene::ForEach(...) for more details about parallel processing on Entities.
    - Visit Jobs::ParallelFor(...) for more details about general parallel processing tasks.
 - Editor
    - Gizmo (Provided by ImGuizmo)
    - Scene hierarchy viewing and editing.
    - Resource/Asset drag-and-drop
    - GUI registry for Entity Inspector for view/editing for both data component and private component defined by the user
    - Profiler
    - ProjectManager
       - Asset
          - Texture
          - 3D Model (Animation supported)
          - Registry
       - Scene I/O
          - Default/Local/Global asset serialization
          - Entity/DataComponent/PrivateComponent/System serialization
       - Prefabs
 - Rendering: PBR + IBL + PCSS
    - Vulkan
    - Support for both deferred (RenderSystem) and forward rendering (RenderSystem/native rendering API).
    - Lighting and Shadows
       - PCF + Cascaded Shadow Map
       - Spot/Point/Directional light support
    - Post-processing
       - Bloom
       - SSAO
       - SSR
    - PBR with IBL support
       - Environmental map
       - Basic light probe/reflection probe support
    - Instanced rendering
    - High-level rendering API - You can issue a complete render command without any vk... command involved.
       - Similiar to https://docs.unity3d.com/ScriptReference/Graphics.DrawMeshInternal.html
 - Animation
 - Cross-platform support for Linux, Windows
 - Native high-level rendering API support (Please visit Graphics for further details)
 - Exportable as a static library (For my research purposes, I'm using the EvoEngine as the underlying rendering framework for my other research projects)
 - Input/Event System
 - Documentation
       - https://codedocs.xyz/edisonlee0212/EvoEngine/
## Upcoming features
Here are the features that will be introduced to EvoEngine in the future, though I don't have a concrete plan of when these will come.
- Procedural terrain and world generation
- Artificial Intelligence
- Audio system
## Getting Started
The project is a CMake project. For project editing and code inspections, Visual Studio 2019 or 2022 is recommended. Simply clone/download the project files and open the folder as a project in Visual Studio and you are ready.
To directly build the project, scripts under the root folder build.cmd (for Windows) and build.sh (for Linux) is provided for building with a single command line.
E.g. For Linux, the command may be :
 - bash build.sh (build in default settings)
 - bash build.sh --clean release (clean and build in release mode)
 - Video demo: 
 - [![EvoEngineOnLinux](https://img.youtube.com/vi/fw8UUDWaMaU/0.jpg)](https://www.youtube.com/watch?v=fw8UUDWaMaU)
Please visit the script for further details.
## Examples
- Rendering
  - This project is mainly for testing and debugging rendering systems. It consists of:
     - Spheres with different material properties.
     - Multiple animated models.
     - Classic Sponza Test Scene
     - Directional light, point light, and spot light.
     - Post-processing
     - Sphere colliders with PhysX
  - Screenshot: ![RenderingProjectScreenshot](/Resources/GitHub/RenderingProjectScreenshot.png?raw=true "RenderingProjectScreenshot")
- Planet
  - The Planet example shows the ability to use ECS for complex behavior. The application contains a simple sphere generation program with dynamic LOD calculation based on the position of the scene camera.
  - Screenshot: ![PlanetProjectScreenshot](/Resources/GitHub/PlanetProjectScreenshot.png?raw=true "PlanetProjectScreenshot")
- Star Cluster
  - The Star Cluster example shows the potential of Job System with ECS by rendering hundreds of thousands of stars at the same time with instanced rendering. The position of each star is calculated in real-time in parallel with a single lambda-expression-based API similar to the m_entities.ForEach() in Unity. 
  - Screenshot: ![StarClusterProjectScreenshot](/Resources/GitHub/StarClusterProjectScreenshot.png?raw=true "StarClusterProjectScreenshot")

## Plans
- Python binding with pybind11
