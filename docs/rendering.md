# EvoEngine Rendering

[Back to README](../README.md)

This page is the high-level map for EvoEngine's rendering stack. Detailed DDGI behavior, demo setup notes, and capture
commands live in separate focused pages:

- [DDGI](ddgi.md)
- [Rendering demos](rendering-demos.md)
- [Rendering validation](rendering-validation.md)
- [CSM validation](csm_validation.md)

## Ownership

`RenderLayer` owns the SDK render frame. It builds the built-in graphics, compute, and ray-tracing pipelines, prepares
render instance storage, records shadow and camera commands, invokes external render callbacks, renders gizmos, and hands
camera textures to post-processing.

Scene environment data owns scene-level lighting settings such as DDGI runtime state, authored probe volumes, environment
lighting, and debug visualization. `RenderLayer` consumes those settings and exposes their runtime state in the editor
inspector.

`Camera` owns the requested render technique. The supported render modes are:

| Mode | Path |
| --- | --- |
| `Rasterization` | GBuffer/deferred lighting plus forward transparent rendering. |
| `RayTracing` | Vulkan ray-tracing pipeline when the device supports ray tracing. |
| `RayQuery` | Compute ray-query path when the device supports RayQuery. |

If a requested ray mode is unavailable, the camera falls back to the best supported mode and reports the fallback once.

## Frame Flow

The normal frame order is:

1. `Application::Run` or an app-specific loop advances the application.
2. `Application::Loop` polls layers and drives per-frame updates.
3. `Application::UpdateInternal` updates the active scene and layers, then calls `RenderLayer::PrepareForRendering`.
4. `RenderLayer::PrepareForRendering` updates render instance storage, per-frame descriptors, ray-tracing descriptors,
   shadow maps, DDGI inputs, and graph resources.
5. Camera rendering runs through raster, ray-tracing, or ray-query paths.
6. Post-processing presents the camera color target to the editor, app window, or capture path.

## Render Data Preparation

`RenderInstanceStorage` gathers renderer state into GPU-facing buffers for:

- deferred and forward mesh instances;
- transparent mesh instances;
- instanced renderers;
- skinned mesh renderers;
- strands where supported by the pass;
- point-cloud and external render instances where registered.

The storage layer also owns the camera-facing structured data used by raster and ray paths: per-frame camera blocks,
light blocks, glTF material buffers, texture-info buffers, render instance buffers, GBuffer descriptors, ray-tracing
descriptor sets, and acceleration structure inputs.

## Raster Camera Path

Raster cameras render shadow maps, then write GBuffer targets for opaque geometry. Deferred lighting combines direct
lights, shadow maps, image-based lighting, and DDGI when available. Transparent and forward-only geometry is rendered
after deferred lighting.

### Deferred GBuffer Contract

The current GBuffer stores evaluated material attributes for ordinary opaque raster shading. Bindings 18 and 19 are
intentionally absent; the old normal and UV/material-index compatibility attachments have been retired.

| Binding | Current image | Current payload |
| --- | --- | --- |
| 17 | Camera depth | NDC depth. |
| 20 | Base color / AO | `rgb = evaluated linear base color`, `a = evaluated occlusion`. |
| 21 | Normal / roughness | `xyz = world normal`, `a = evaluated roughness`. |
| 22 | PBR / flags | `x = evaluated metallic`, `y = default-lit shading model id`, `z/w = reserved`. |
| 23 | Emissive | `rgb = evaluated emissive radiance`, `a = reserved`. |
| 24 | Utility | `x = instance index`, `y = instance info index`, `z = material index`, `w = reserved`. |

`StandardDeferred.frag` evaluates GLTF material state once during geometry and writes only the expanded payload.
`StandardDeferredLighting.frag`, `StandardDeferredLightingSceneCamera.frag`, SSR, SSAO, scene-camera debug
visualization, editor GBuffer preview images, and editor mouse picking decode material, normal, or selection state from
bindings 20-24. Editor picking reads the instance index from Utility.x.

The target Unreal-style deferred path stores ordinary opaque shading state in the geometry pass. The first migration
keeps depth as-is and introduces this logical schema:

| Logical attachment | Initial format target | Payload |
| --- | --- | --- |
| Base color / AO | `VK_FORMAT_R16G16B16A16_SFLOAT` | `rgb = linear base color`, `a = occlusion`. |
| Normal / roughness | `VK_FORMAT_R16G16B16A16_SFLOAT` | `xyz = world normal`, `a = roughness`. A later compact packing may replace full-vector normal storage after validation. |
| PBR / flags | `VK_FORMAT_R16G16B16A16_SFLOAT` | `x = metallic`, `y = shading model id`, `z = material flags`, `w = reserved custom data`. |
| Emissive | `VK_FORMAT_R16G16B16A16_SFLOAT` | `rgb = emissive radiance`, `a = reserved custom data`. |
| Utility | `VK_FORMAT_R32G32B32A32_SFLOAT` | `x = instance index`, `y = instance info index`, `z = optional material index for debug or fallback`, `w = reserved`. |

The first supported shading model is opaque/default-lit GLTF. Metallic-roughness materials and specular-glossiness
materials are both reduced to base color, metallic, roughness, normal, occlusion, and emissive by the geometry pass.
Masked alpha remains a geometry-pass discard. Transparent blend, transmission, diffuse transmission, volume/scatter,
clearcoat, sheen, anisotropy, iridescence, and other special lobes stay on their existing transparent, forward, ray, or
documented fallback paths until a later milestone defines their GBuffer representation.

Ordinary opaque lighting must not call `EE_EVALUATE_GLTF_RASTER_SURFACE`; material texture sampling during lighting is
allowed only for an explicitly documented fallback or debug path. The retired normal/material attachments should not be
reintroduced for ordinary opaque shading.

### Raster Texture Descriptor Contract

Raster material descriptors use descriptor set 3. Set 0 remains the per-frame set, set 1 remains available for
meshlet/bone/instanced/strand data, and set 2 remains available for lighting or pass descriptors. Pipelines that need the
material set but do not use intermediate sets should bind empty layouts for the unused set slots.

Raster material texture sampling targets fixed individual texture bindings rather than bindless descriptor arrays.
Raster shaders must not use descriptor arrays such as `sampler2D[]` or `samplerCube[]`, `nonuniformEXT`, or dynamic
descriptor indexing for texture sampling. Fixed-binding image arrays and atlases are allowed for raster when the descriptor
itself is a normal fixed binding, such as a shadow-map array or atlas texture.

| Material binding | Fixed raster texture | Fallback |
| --- | --- | --- |
| 0 | Base color or diffuse | White. |
| 1 | Metallic-roughness or specular-glossiness | White. |
| 2 | Normal | Flat normal. |
| 3 | Emissive | Black. |
| 4 | Occlusion | White. |

Raster material descriptor sets are renderer-owned runtime state keyed by material index. The first migration intentionally
does not deduplicate descriptor sets across material indices because material indices can change while the renderer is
running. Each descriptor slot uses the texture's existing combined image sampler. Missing, ignored, or pending textures
bind the documented fallback textures.

Opaque deferred pipelines currently enable the fixed raster material backend. Direct draws bind per-material descriptor
sets per draw. When indirect rendering is enabled, `DeferredGeometryPass` uses material-batched indirect ranges: each
contiguous range has one material descriptor, one compatible pipeline-state key, one push-constant base instance, and an
offset/count into the shared indirect command buffers. This restores deferred mesh indirect rendering without returning
opaque raster material sampling to bindless texture arrays. These opaque material-producing pipelines use a raster
material per-frame descriptor set that keeps the shared per-frame buffers but omits bindless texture and cubemap array
bindings.

Built-in shadow alpha and transparent mesh pipelines also use the fixed raster material backend and bind the material
descriptor set before direct material-sampling draws. Opaque shadow indirect draws may remain indirect because they use a
texture-free empty fragment shader; alpha-tested shadow indirect draws are temporarily routed through direct submission
until material-batched indirect buffers can bind one material descriptor per batch. Package or external forward callbacks
that evaluate glTF raster materials are explicit migration fallbacks until their owners provide fixed material
descriptors or material-batched submission. Alpha-tested built-in shadow pipelines also use the raster material per-frame
descriptor set without bindless texture arrays.

Raster lighting uses a fixed raster-global texture descriptor set for image-based lighting inputs instead of sampling the
bindless texture arrays. Deferred lighting binds this set after the shared lighting descriptor set, and transparent mesh
lighting binds it after the material descriptor set. The fixed slots are BRDF LUT, skybox cubemap, irradiance cubemap, and
prefiltered environment cubemap. Set 2 still owns shared shadow-map and DDGI atlas bindings.

Material and mesh thumbnail rendering uses `AssetThumbnailProvider` and `OffscreenPreviewRenderer`, which build a
temporary scene, upload referenced preview textures, force a raster camera, disable preview-only volumetric clouds and
DDGI state, and call `RenderLayer::RenderSceneToCameraImmediately`. These preview paths do not own separate glTF raster
material pipelines, so material-sampling preview output inherits the same fixed material descriptor layouts and per-draw
descriptor binding used by the normal RenderLayer camera passes.

Bindless texture arrays are reserved for ray tracing and ray query paths. Raster-only or lower-end device mode must avoid
creating bindless descriptor layouts when ray tracing and ray query are unavailable or disabled. BRDF LUTs, environment
cubemaps, DDGI atlases, volumetric cloud textures, and pass-local textures may remain in fixed global or pass descriptor
sets during migration, but rasterization should not sample them through bindless descriptor arrays at the end.
Non-ray-tracing compute passes use fixed global or pass descriptor sets; ray tracing, ray query, and ray diagnostics keep
their bindless texture access.

Current shadow policy:

- directional CSM uses Legacy Stable fitting;
- split placement uses Practical Log/Uniform;
- directional, point, and spot lights use PCF sampling;
- PCF radius is derived from light size as `100 x light_size`;
- shadow-map quality sets directional, point, and spot shadow-map resolution together.

## Ray Camera Paths

Ray-tracing cameras update the TLAS, bind the per-frame descriptor set and ray-tracing descriptor set, then dispatch the
camera ray-generation shader. The camera raygen owns path depth, direct light evaluation, environment misses,
BSDF-sampled next-bounce rays, throughput, Russian roulette, firefly clamping, and Auto SPP convergence. Closest-hit
shaders record surface identity and geometry data into `CameraRayTracingPayload`.

RayQuery cameras use the compute path and share the same high-level camera material and light data where supported.

## Render Graph And Extension Model

`RenderGraph` declares logical resources, pass queues, access plans, and compiled barrier plans. `RenderLayer` applies
the compiled barriers on the current command path. Queue-family ownership transfer and broader async queue submission are
still future work.

Packages and services extend rendering through registered callbacks and explicit external geometry registration rather
than by mutating built-in pass internals. External geometry can participate in DDGI only when it supplies compatible BLAS
and triangle offset data.

## Current Migration Notes

The renderer has moved many built-in resources into explicit graph resources, but some legacy areas remain:

- SSAO is still a graphics-bound fullscreen render-pass path.
- The depth pyramid pass currently exists as a graph resource but is still a clear-only producer rather than a
  hierarchical reduction.
- Post-processing still contains graphics-bound passes that should move carefully after camera graph ownership is stable.
- Async compute/graphics overlap should wait until the remaining resource ownership boundaries are explicit.

## File Map

| Area | Main files |
| --- | --- |
| Render frame orchestration | `EvoEngine_SDK/src/RenderLayer.cpp` |
| Camera modes and fallback | `EvoEngine_SDK/src/Camera.cpp`, `EvoEngine_SDK/include/Rendering/Camera.hpp` |
| Render graph | `EvoEngine_SDK/src/RenderGraph.cpp`, `EvoEngine_SDK/include/Rendering/RenderGraph.hpp` |
| Render instance storage | `EvoEngine_SDK/src/RenderInstanceStorage.cpp`, `EvoEngine_SDK/include/Rendering/RenderInstances/RenderInstanceStorage.hpp` |
| Lighting shaders | `EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/Lighting.glsl` |
| glTF raster material shaders | `EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/GltfRasterMaterial.glsl` |
| glTF ray material shaders | `EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/GltfRayTracingBsdf.glsl` |
