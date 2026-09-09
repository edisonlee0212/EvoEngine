# EvoEngine Rendering

[Back to README](../README.md)

EvoEngine uses a Vulkan renderer built around `RenderLayer`. This page explains how scene data becomes a rendered camera
image and where each rendering responsibility lives.

Focused guides:

- [Materials and geometry](rendering-materials.md)
- [Texture access](rendering-texture-access.md)
- [Global illumination: DDGI, SDFGI, and HDDAGI](rendering-gi.md)
- [Reflection probes](reflection-probes.md)
- [Rendering demos](rendering-demos.md)
- [Rendering validation](rendering-validation.md)

## Architecture

### Image layouts and synchronization

First-party GPU images use `VK_IMAGE_LAYOUT_GENERAL` for sampled, storage, attachment, and transfer access.
New images transition from `UNDEFINED`; swapchain images transition to `PRESENT_SRC_KHR` for presentation.
Render-graph access declarations still drive memory barriers and queue ownership. Equal layouts do not remove
synchronization requirements. DDGI atlases are initialized before their descriptors are published.

### Rendering flow

```mermaid
flowchart LR
  A[Scene, assets, and package callbacks] --> B[RenderInstanceStorage]
  B --> C[RenderLayer frame preparation]
  C --> D[RenderGraph resources and barriers]
  D --> E[Raster camera]
  D --> F[Ray-tracing camera]
  D --> G[Ray-query camera]
  E --> H[Post-processing and output]
  F --> H
  G --> H
```

| Owner | Responsibility |
| --- | --- |
| `Camera` | View state, output texture, visible background, requested render technique, ray settings, and post-processing stack. |
| `Scene` | Renderable entities, lights, the main camera, environmental-lighting reference, and global reflection-probe fallback. |
| `EnvironmentalLighting` | Indirect GI provider, shared probe layout, DDGI/SDFGI/HDDAGI settings, environment source, reflection probes, and fallback intensities. |
| `RenderInstanceStorage` | Immutable per-frame GPU view of geometry, materials, transforms, lights, cameras, environment data, descriptors, and acceleration-structure inputs. |
| `RenderGraph` | Logical resources, pass ordering, access declarations, transient allocation, and Vulkan barrier planning. |
| `RenderLayer` | Pipeline creation, frame preparation, shadows, camera rendering, probe work, extension callbacks, diagnostics, and output handoff. |

Camera passes consume an immutable render-instance snapshot rather than mutable scene components. Static rigid
renderers may reuse cached records; camera visibility, LOD selection, and draw lists still update each frame.

## Shader Source Policy

All first-party shaders use native Slang source in `.slang` or `.slangh` files. Use Slang modules and `import` for shared
code; legacy shader extensions, GLSL compatibility syntax, and textual `#include` directives are rejected before the
Slang frontend runs. `Extern/` is third-party scope and is exempt from this source policy.

## Frame Flow

1. The application updates the project, scene, transforms, and layers.
2. `RenderLayer::PrepareForRendering` gathers render instances and resolves lighting state.
3. Geometry and textures complete pending uploads; acceleration structures and per-frame descriptors are prepared.
4. Shared work such as shadows, selected GI field updates, and reflection-probe captures is recorded.
5. Each camera executes its raster, ray-tracing, or ray-query graph.
6. Post-processing writes the final camera image for the editor, application window, render texture, or capture path.

Frame-slot fences protect resources still used by submitted GPU work. Transient graph resources and replaced renderer
objects remain alive until their owning slot is recycled.

## Static Scene Contract

`Scene::SetEntityStatic(entity, true)` enables record reuse for unchanged rigid meshes in a root hierarchy.
Skinned meshes, particles, strands, splats, external renderers, and API draws remain dynamic.

Scene/editor structural edits invalidate caches automatically. Code that directly changes a static transform or renderer
must call `RenderLayer::NotifyStaticEntityChanged(scene, entity)` afterward. Otherwise raster and ray inputs can remain
stale. Unchanged TLAS inputs reuse existing acceleration structures; changed inputs trigger the required build or update.
Frame-slot lifetime tracking protects submitted resources.

## Camera Techniques

| Technique | Rendering path | Availability behavior |
| --- | --- | --- |
| Rasterization | Raw opaque/masked GBuffer, fused compute material evaluation and lighting, forward/transparent rendering, then post-processing. | Always available. |
| Ray tracing | Vulkan ray-tracing pipeline running the shared path-tracing integrator. | Falls back to ray query when available, otherwise rasterization. |
| Ray query | Compute pipeline using inline ray queries with the shared path-tracing integrator. | Falls back to ray tracing when available, otherwise rasterization. |

The resolved technique is reported when it differs from the camera request. Ray tracing and ray query share material
evaluation, BSDF logic, environment and emissive sampling, path controls, debug views, and optional outputs. Only their
traversal adapters differ.

Ray-camera accumulation belongs to each camera. Resizing, technique changes, relevant camera settings, and scene changes
invalidate the affected history. An unchanged camera continues accumulating samples across frames.

### Ray-tracing camera pass flow

```mermaid
flowchart TD
  A[Prepared scene, BLAS and TLAS, materials and lights] --> B[Prepare camera accumulation and optional outputs]
  B --> C[RayTracingCamera: dispatch rays]
  subgraph T[Inside the ray-tracing pass]
    C --> D[Ray generation: camera samples]
    D --> E[Shared integrator: surface and shadow rays]
    E --> F[Hit and miss shaders: traversal results]
    F --> G[Material, direct light, emission, environment, next bounce]
    G -->|Continue path| E
    G -->|Finish samples| H[Write radiance history, convergence, hit distance and outputs]
  end
  H --> I[Optional volumetric clouds]
  I --> J[Optional Gaussian splats: cull, sort, overlay]
  J --> K[PostProcessing: bloom and tone mapping]
  K --> L[Camera output]
```

### Ray-query camera pass flow

```mermaid
flowchart TD
  A[Prepared scene, BLAS and TLAS, materials and lights] --> B[Prepare camera accumulation and optional outputs]
  B --> C[RayQueryCamera: dispatch compute in 8 by 8 groups]
  subgraph T[Inside the compute pass]
    C --> D[Compute threads: camera samples]
    D --> E[Shared integrator: surface and shadow rays]
    E --> F[Inline ray queries: candidate and committed hits]
    F --> G[Material, direct light, emission, environment, next bounce]
    G -->|Continue path| E
    G -->|Finish samples| H[Write radiance history, convergence, hit distance and outputs]
  end
  H --> I[Optional volumetric clouds]
  I --> J[Optional Gaussian splats: cull, sort, overlay]
  J --> K[PostProcessing: bloom and tone mapping]
  K --> L[Camera output]
```

Each ray diagram expands one GPU camera pass; bounces and history writes are shader work within that pass.
Both modes use `CameraRayIntegrator`. They trace indirect lighting directly rather than gathering raster GI fields.
Optional outputs include albedo, normal, ray count, path length, timing, and debug data.

## Raster Path

### Raster camera pass flow

```mermaid
flowchart TD
  A[Prepared scene, selected GI field, reflection probes] --> B[Directional shadow maps]
  B --> C[DeferredGeometry: raw GBuffer and depth]
  C --> D[MotionVectors and MotionCoverage]
  D --> E[DepthPyramid]
  E --> F[AmbientOcclusion: optional GTAO]
  F --> G{GI provider}
  G -->|HDDAGI| H[HddagiCameraSurface: normal and roughness]
  H --> I[HddagiCameraGather: full-resolution GI]
  I --> J[Optional horizontal and vertical reflection filters]
  subgraph K[DeferredCamera: material evaluation and lighting]
    KD[Gather DDGI diffuse probes]
    KS[Gather SDFGI diffuse and specular]
    KH[Compose HDDAGI camera images]
    KE[Environment and reflection probes]
  end
  G -->|DDGI| KD
  G -->|SDFGI| KS
  G -->|Environment or unavailable field| KE
  J --> KH
  KD --> L[Optional forward callbacks and volumetric clouds]
  KS --> L
  KH --> L
  KE --> L
  L --> M[Optional transparent geometry]
  M --> N[Optional Gaussian splats: cull, sort, render]
  N --> O[Optional DDGI debug overlays]
  O --> P[PostProcessing: SSR, bloom, tone mapping, SMAA]
  P --> Q[Editor selection and optional SDFGI debug view]
  Q --> R[Camera output]
```

Disabled stages are skipped. Point/spot shadows and GI updates are shared work. The `DeferredCamera` branches show
provider-specific work within one pass; HDDAGI alone prepares separate camera images first. Extensions declare their
own dependencies. Reflection captures use a reduced path and diffuse GI only.

Opaque geometry writes raw attributes and IDs; masked geometry additionally samples base-color alpha for coverage.
GTAO uses geometric normals before deferred compute evaluates materials, resolves the GBuffer, and combines direct
lighting, shadows, GI, reflection probes, and AO. Forward/transparent geometry follows, then optional overlays and
post-processing.

Persistent sampled assets and transient pass resources follow different descriptor policies. Standard material,
environment, and reflection-probe assets share bindless 2D and cubemap index spaces across raster and ray paths;
attachments, histories, and other pass-owned images retain explicit descriptors. See
[Texture access](rendering-texture-access.md) for the ownership rules and [Materials and geometry](rendering-materials.md)
for material behavior and geometry participation.

## Lighting

Direct lighting comes from directional, point, and spot lights. Environment and probe inputs are resolved from the scene
and its `EnvironmentalLighting` asset before rendering.

Choose Environment, Automatic DDGI, Automatic SDFGI (default), or Automatic HDDAGI. The selected automatic provider
updates a shared camera-anchored field; unsupported or unready coverage uses Environment fallback.
See [Global illumination](rendering-gi.md) for provider differences, controls, update passes, and known limitations.

| Use | Source and control |
| --- | --- |
| Visible primary background | The camera background source multiplied by `background_intensity`. |
| Raster diffuse indirect | Valid irradiance from the selected provider; otherwise the indirect environment source multiplied by `diffuse_fallback_intensity`. |
| Raster specular indirect | Selected GI, reflection probes, and environment fallback; local probes and SSR retain composition priority. |
| Ray-camera environment events | The indirect environment source multiplied by `environment_lighting_intensity`. |
| Reflection-probe capture background | The authored bake background, with inherited environment radiance multiplied by `environment_lighting_intensity`. |

All three environmental-lighting intensities default to `1.0`. Visible camera backgrounds are independent from indirect
lighting. Ray cameras do not use raster diffuse or specular fallback intensities, and they do not use the scene-global
prefiltered reflection probe as their environment radiance source.

## Shadows And Post-Processing

Directional lights use four cascades with Stable Sphere fitting by default; Tight Light-Space AABB fitting is available
for comparison. Directional shadows use Vogel-disc percentage-closer filtering. Point and spot lights use their own
shadow atlases and filtering paths. The quality override changes directional, point, and spot shadow-map resolution
together. Every shadow type separates opaque and alpha-masked rigid, meshlet, instanced, skinned, and strand casters.
Opaque depth shaders perform no material or texture access; masked depth shaders evaluate only base-color alpha
coverage. External shadow renderers register explicitly for one of those two contracts.

Raster cameras can use GTAO ambient occlusion, screen-space reflections, SMAA, bloom, tone mapping, and related post
effects from their `PostProcessingStack`. Ray cameras apply bloom and tone mapping after path tracing. Post-processing
resources and temporal histories are camera-owned. Assets must use the current flat GTAO and SMAA schemas.

Bloom's **Compression start** (2) and **Source ceiling** (8) bound sampled HDR contributions before downsampling,
preserving color ratios. They limit the bloom source, not the original scene color or final composited halo.

## Geometry And Optional Features

Regular, skinned, instanced, transparent, strand, Gaussian-splat, and externally registered geometry enter the renderer
through separate render-instance collections. Participation depends on the selected pass and the data supplied by the
renderer owner.

Strands use the mesh-shader raster path when supported. Ray cameras also include strands when the Vulkan device exposes
`VK_NV_ray_tracing_linear_swept_spheres` with `linearSweptSpheres`. Without that capability, strands are omitted from ray
tracing and ray query while raster strand rendering remains available. DDGI does not traverse strand geometry.

## Extending Rendering

Packages and services extend rendering through explicit APIs rather than by modifying built-in pass internals:

- register logical resources and frame- or camera-level render-graph passes;
- register external render instances and, when applicable, compatible acceleration-structure metadata;
- use raw opaque or alpha-only masked deferred callbacks, forward/transparent callbacks, explicit opaque or alpha-only
  masked shadow callbacks, and gizmo callbacks exposed by `RenderLayer`;
- provide package-owned shaders, descriptors, and resources for package-specific rendering.

External geometry participates only in the paths for which it supplies the required draw or traversal contract. For
example, DDGI requires compatible triangle acceleration-structure and offset data.

## Source Entry Points

Start with `RenderLayer`, `Camera`, `RenderGraph`, and `RenderInstanceStorage` in `EvoEngine_SDK`. Material and lighting
shader modules live under `EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine`.

The camera flows follow `RenderLayer::RenderToCamera`, `RenderLayer::RenderToCameraRayTracing`,
`HddagiCameraFrame::AddPasses`, and `PostProcessingStack::Process` / `ProcessRayCamera`.
