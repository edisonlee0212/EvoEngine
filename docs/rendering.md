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
`StandardDeferredLighting.frag`, `StandardDeferredLightingSceneCamera.frag`, SSR, AO, TAA, scene-camera debug
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

Built-in shadow-map passes treat all mesh materials as opaque. They use texture-free depth shaders, do not bind raster
material descriptor sets, and do not sample material textures for alpha discard. This keeps regular mesh shadow draws on
the opaque shadow indirect command path when indirect rendering is enabled. Transparent mesh pipelines still use the
fixed raster material backend and bind the material descriptor set before direct material-sampling draws. Package or
external forward callbacks that evaluate glTF raster materials are explicit migration fallbacks until their owners
provide fixed material descriptors or material-batched submission.

Raster lighting uses a fixed raster-global texture descriptor set for image-based lighting inputs instead of sampling the
bindless texture arrays. Deferred lighting binds this set after the shared lighting descriptor set, and transparent mesh
lighting binds it after the material descriptor set. The fixed slots are BRDF LUT, skybox cubemap, irradiance cubemap, and
prefiltered environment cubemap. Set 2 still owns shared shadow-map and DDGI atlas bindings.

Material and mesh thumbnail rendering uses `AssetThumbnailProvider` and `OffscreenPreviewRenderer`, which build a
temporary scene, upload referenced preview textures, force a raster camera, disable preview-only volumetric clouds and
DDGI state, and call `RenderLayer::RenderSceneToCameraImmediately`. These preview paths do not own separate glTF raster
material pipelines, so material-sampling preview output inherits the same fixed material descriptor layouts and per-draw
descriptor binding used by the normal RenderLayer camera passes.

Bindless texture arrays are reserved for ray tracing and ray query paths. When ray tracing and ray query are unavailable
or disabled, `RenderLayer` creates the ordinary per-frame descriptor layout without texture or cubemap descriptor arrays
and skips binding the global texture storage arrays. Raster material textures, raster lighting inputs, DDGI atlases,
volumetric cloud textures, pass-local textures, and non-ray-tracing compute texture inputs use fixed material, global, or
pass descriptor sets. Ray tracing, ray query, and ray diagnostics keep their bindless texture access.

Current shadow policy:

- directional CSM uses Legacy Stable fitting;
- split placement uses Practical Log/Uniform;
- directional, point, and spot lights use PCF sampling;
- PCF radius is derived from light size as `100 x light_size`;
- directional shadows default to 8192, while point and spot shadows default to 4096;
- an explicit shadow-map quality override sets directional, point, and spot resolution together.

## Ray Camera Paths

Each frame slot owns a persistent TLAS. Unchanged instance input reuses it without recording GPU work, compatible
transform or instance-data changes use an in-place TLAS update, and topology or active-state changes rebuild the same
allocation when its capacity permits. Upload, build/update, and traversal barriers are recorded on the main frame queue;
there is no separate immediate-submit fence. Mesh-empty ray scenes bind a valid TLAS containing one inactive dummy
instance. Instanced meshes assign each particle a ray-only instance block containing its composed world transform so hit
reconstruction does not fall back to the particle renderer's parent transform. Static mesh BLAS objects remain
asset-owned; animated skinned-mesh BLAS maintenance is handled separately.

Ray-tracing cameras bind the per-frame descriptor set and ray-tracing descriptor set, then dispatch the camera
ray-generation shader. The camera raygen owns path depth, direct light evaluation, environment misses, BSDF-sampled
next-bounce rays, throughput, Russian roulette, firefly clamping, and Auto SPP convergence. Closest-hit shaders record
surface identity and geometry data into `CameraRayTracingPayload`.

RayQuery cameras use the compute path and share the same high-level camera material and light data where supported.

## Render Graph And Extension Model

`RenderGraph` declares logical resources, pass queues, access plans, and compiled barrier plans. `RenderLayer` applies
the compiled barriers on the current command path. Queue-family ownership transfer and broader async queue submission are
still future work.

Packages and services extend rendering through registered callbacks and explicit external geometry registration rather
than by mutating built-in pass internals. External geometry can participate in DDGI only when it supplies compatible BLAS
and triangle offset data.

## Current Migration Notes

The raster post-processing stack uses technique-specific ordering: ambient occlusion, SSR/reflections, TAA when selected,
bloom, tone mapping, then SMAA when selected. `AmbientOcclusion` owns the SSAO/GTAO selection and `AntiAliasing` owns the
TAA/SMAA selection. Initialized stacks enable GTAO, SMAA Ultra, and tone mapping; bloom and SSR are disabled by default.
Ray-tracing and ray-query cameras use only bloom and tone mapping for this branch.

TAA follows the Best Quality configuration from [GameTechDev/TAA](https://github.com/GameTechDev/TAA) by default. The
resolve operates on Reinhard tone-mapped history, uses YCoCg variance AABB intersection with a 9-pixel neighborhood,
selects the longest velocity from a 9-pixel neighborhood, samples history with the reference 5-tap bicubic filter, and
writes a separate inverse-Reinhard linear HDR output for bloom and tone mapping. High Quality and Performance presets
retain the reference's lower-cost combinations, while Custom exposes the individual settings.

SMAA 1x follows [iryoku/smaa commit 71c806a](https://github.com/iryoku/smaa/tree/71c806a838bdd7d517df19192a20f0c61b3ca29d)
with the reference luma-edge, blend-weight, and neighborhood-blending passes and exact `160x560` RG8 area and `64x16` R8
search textures. Low, Medium, High, and Ultra select the matching reference presets. Edges are detected in perceptual color
while neighborhood blending operates in linear light. When tone mapping is disabled, edge detection uses a bounded
Reinhard perceptual proxy and neighborhood blending preserves the original HDR color.

AA persistence uses `enable_anti_aliasing` and an `anti_aliasing` map containing `algorithm`, `taa`, and `smaa`. Missing,
invalid-enum, and legacy-only AA data falls back to enabled SMAA Ultra. The removed TAA-only keys are intentionally not
migrated.

Motion vectors store `previous_pixel - current_pixel` in pixel units and
`previous_normalized_linear_depth - current_normalized_linear_depth` in `z`. Camera and rigid motion use the deferred
compute pass. Deferred skinned meshes use a depth-tested geometry pass with an explicit previous rendered bone pose and
previous object transform. Newly spawned or incompatible poses write a velocity that deterministically rejects history.
Native rigid transparent meshes use a depth-tested geometry pass with previous object transforms and the same far-to-near
order as transparent color rendering. That pass replaces motion `xy` while preserving the opaque/background normalized
depth payload in `z`; newly spawned or incompatible transforms still reject history. Layered transparency therefore uses
the visible transparent surface's motion with the opaque background's depth confidence rather than maintaining a separate
transparent history. Unsupported forward, external, instanced, strands, transparent-skinned, and Gaussian-splat motion
conservatively rejects history for the camera until those paths gain surface motion coverage. The normalized depth payload
is derived from view-linear clip depth using EvoEngine's zero-to-one Vulkan projection; camera-transform changes retain
history and rely on this motion/depth reprojection instead of the ray-accumulation frame counter. Explicit camera resets
increment a separate history version so resize and camera settings changes still invalidate temporal history.

The current-color neighborhood has both direct-fetch and thread-group shared-memory implementations. Best Quality and
High Quality use FP32, matching the reference's default precision; Performance enables FP16 color intermediates only when
Vulkan 1.2 `shaderFloat16` is supported. Shared memory, variance moments, AABB intersection, bicubic coordinates, motion,
and depth calculations remain FP32. Reinhard samples stay FP32 when FP16 rounding would make inverse reconstruction
numerically sensitive, and invalid bicubic or clipping results fall back to the current sample. Invalid motion sentinels
reject their own pixel without participating in neighboring longest-velocity selection. History-resolved HDR luminance
is constrained to an expanded current 3x3 neighborhood envelope and re-encoded into history when constrained; current
frame highlights are not clamped. TAA history remains owned per camera by `AntiAliasing` and is invalidated on
resize, skipped frames, toggles, preset or persistent-setting changes, unsupported camera-wide motion, and explicit reset.

The renderer has moved many built-in resources into explicit graph resources, but some legacy areas remain:

- TAA currently owns its own per-camera history textures until graph history resources expose explicit ping-pong bindings.
- The depth pyramid pass is a graph resource with hierarchical reduction and can be used by future post-processing
  optimizations when resource ownership is explicit.
- Post-processing still contains owned resources that should move carefully after camera graph ownership is stable.
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
