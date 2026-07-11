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
| 22 | PBR / flags | `x = evaluated metallic`, `yzw = evaluated dielectric/specular F0`. |
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
| PBR / flags | `VK_FORMAT_R16G16B16A16_SFLOAT` | `x = metallic`, `yzw = dielectric/specular F0`. |
| Emissive | `VK_FORMAT_R16G16B16A16_SFLOAT` | `rgb = emissive radiance`, `a = reserved custom data`. |
| Utility | `VK_FORMAT_R32G32B32A32_SFLOAT` | `x = instance index`, `y = instance info index`, `z = optional material index for debug or fallback`, `w = reserved`. |

The first supported shading model is opaque/default-lit GLTF. Metallic-roughness materials produce base color, metallic,
roughness, and derived F0. Specular-glossiness materials retain the Khronos diffuse term, independent colored F0, and
`roughness = 1 - glossiness`; they are no longer approximated as metallic-roughness.
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

glTF base-material evaluation shares one texture-info ABI across raster, RT-pipeline, and RayQuery. It carries UV0 and
UV1 independently, applies `KHR_texture_transform` after selecting the extension-overridden coordinate set, multiplies
linear `COLOR_0` RGBA into metallic-roughness base color or specular-glossiness diffuse, and treats `OPAQUE` alpha as
coverage-independent. Ray footprints track separate UV0/UV1 texel densities before selecting a texture gradient; zero
footprints use explicit mip 0 outside fragment stages. Color-semantic RGB channels (base/diffuse, emissive,
specular-glossiness, specular color, sheen color, and diffuse-transmission color) use the exact sRGB transfer function
when the texture view does not already decode sRGB. Alpha and data-texture channels remain linear.

`GltfShadeMaterial` is the canonical owner of material-facing raster state: `double_sided` selects culling, while alpha
mode and transmission select the opaque or transparent pass. The Material inspector exposes those canonical controls
instead of separate cull/blending overrides so raster, RT-pipeline, and RayQuery cannot silently disagree.

Requested DDS images are preferred and retain authored BC7 mip chains and hardware sRGB decoding; same-stem
PNG/TGA/JPG/JPEG files remain fallback sources when the DDS is absent. Float fallback textures generate a complete mip
chain, but the shared `Texture2D` storage still uses repeat/linear sampling and performs semantic sRGB decoding after
hardware filtering. Per-glTF sampler wrap/filter state and fully linear-space filtering/mipmap generation for sRGB
fallback images are deliberate follow-up work.

The canonical extension parser currently reads external-image `.gltf` files. `.glb`, embedded-image, data-URI material
extension parsing, and `TEXCOORD_2+` vertex storage remain follow-up work. Authored glTF tangents are preserved; missing
tangents are generated from the normal texture's selected UV0/UV1 set before geometry upload, but the generator is not
yet MikkTSpace and does not split vertices at tangent discontinuities.

### Advanced glTF Ray Materials

The shared RT-pipeline/RayQuery material path imports and evaluates the ratified `KHR_materials_iridescence`,
`KHR_materials_anisotropy`, and `KHR_materials_dispersion` extensions. Iridescence intensity uses texture R, thin-film
thickness uses texture G, and anisotropy uses normalized texture RG with strength in B. These are linear data textures and
reuse the same UV0/UV1, `KHR_texture_transform`, storage-flip, and ray-footprint behavior as the base material inputs.
Dispersion has no texture and is evaluated only by the specular-transmission lobe.

The implementation follows Khronos when the pinned reference differs:

- iridescence uses the Khronos analytical spectral integration with the colored substrate F0. Dielectrics use the
  IOR/specular-weighted F0, metals use base-color F0, and the maximum Fresnel component controls base-layer attenuation;
- positive anisotropy rotation is counter-clockwise from tangent toward bitangent. Host state stores `(cos(theta),
  sin(theta))`; the pinned reference's inverse rotation is not reproduced;
- dispersion perturbs the material IOR on both entry and exit. The reference's `ior.x`-only implementation does not
  disperse an air-to-material entry interface;
- a volume boundary is selected by `thicknessFactor > 0`. Ray traversal supplies the real segment length, so the
  thickness texture remains a raster thickness estimate and does not turn a traced volume boundary on or off;
- `KHR_materials_specular` keeps authored color components above 1 and applies the scalar factor after the colored,
  unweighted dielectric F0. Its grazing F90 is the scalar specular factor, so factor 0 disables dielectric reflection and
  fractional factors do not incorrectly approach white. New material assets use the Khronos default of 1; schema-1
  `.evematerial` values of 0 migrate to 1 because the old shader treated them as implicit full specular;
- `KHR_materials_ior.ior: 0` retains the specification's positive-infinity compatibility mode: surface F0 is 1,
  transmission uses a finite infinity surrogate for stable arithmetic, and dispersion is disabled. Invalid authored IORs
  between 0 and 1 normalize to 1 rather than being interpreted as this compatibility mode;
- unlit ray materials return base color only, without adding emissive first. Diffuse or glossy transmission events both
  update the current volume medium.

`KHR_materials_retroreflection` is not in the Khronos extension registry. EvoEngine accepts that spelling, plus the older
`EXT_materials_retroreflection` spelling, only as experimental compatibility with `vk_gltf_renderer` at
`f72d2f3711116261a76e7b8b0f4724e167703a55`. The inspected BSDF dependency was `nvpro_core2` revision
`907fba3c5b7a9597e7e63a5388079b964bd6ddb4`. The shared ray BSDF applies the Minimal Retroreflective Microfacet Model view
substitution to reflection lobes and leaves transmission conventional. Evaluation and sampling both report the same
marginal forward/retro mixture BSDF and PDF. Sample throughput is the full mixture BSDF divided by that marginal PDF; it
does not divide again by the selected branch probability, which would over-brighten fractional blends.

Opaque raster stores IOR/specular-aware colored F0. Ray shadow transmission samples the specular-transmission,
base-color, diffuse-transmission factor/color, specular factor/color, and vertex-color inputs. It applies Fresnel remaining
energy first, then layers diffuse transmission only over the `(1 - specularTransmission)` share. The current fixed raster
material descriptors and deferred GBuffer do not encode iridescence, anisotropy, dispersion, or retroreflection lobes, so
those effects are ray-path features rather than claimed raster parity. Deferred raster also stores F0 but has no separate
fractional specular F90 channel. The legacy CPU/compute ray display is likewise not an advanced-material integrator.
Clearcoat-normal scale, coated-emission attenuation, and validation/rejection of spec-forbidden unlit or
specular-glossiness extension combinations remain documented follow-up work rather than silent conformance claims.

### Emissive-Triangle Next-Event Sampling

RT-pipeline and RayQuery cameras share one static emissive-triangle distribution. Eligible emitters are fill-mode,
opaque, non-transmissive, non-unlit `MeshRenderer` instances with a valid BLAS. Skinned meshes, particle/instanced
meshes, strands, Gaussian splats, external geometry, alpha-mask/blend materials, and transmissive materials are not in
the distribution. They retain hit-time emission where their existing material path permits it.

Each entry identifies the packed `GeometryStorage` instance/primitive pair used by the BLAS. Selection weight is
world-space triangle area times emissive-factor luminance, with a two-sided importance factor where applicable. The GPU
samples the stored float CDF, and each entry's area PDF is derived from that exact quantized CDF interval so sampling and
hit-side MIS have identical discrete support. Textures are deliberately excluded from the proposal distribution; the
sampled UV0/UV1 emission is evaluated exactly at mip 0 with the shared texture transform and sRGB rules. Hits on table
emitters use that same explicit-LOD radiance for the competing BSDF estimator; unsupported hit-only emitters retain their
ray-footprint LOD. This keeps the MIS estimators on one integrand without requiring CPU texture readback.

Each frame slot retains its distribution across `RenderInstanceStorage::Clear()`. An exact ordered signature of the
eligible static mesh handle/version, packed triangle range, ray instance index, model transform, and derived importance
gates the triangle walk; unchanged slots restore only the table count and do not transform, sort, compare, or upload the
triangle records again. The key intentionally does not use the global geometry-storage revision, so unrelated skinned
mesh updates cannot invalidate the static-emitter table. The storage buffer is uploaded only when that exact signature
changes.

Emissive NEE is an independent one-sample estimator in addition to the existing punctual/environment estimator. It uses
the area-to-solid-angle PDF and balance-heuristic MIS against the BSDF or volume phase PDF. BSDF/phase rays that hit a
table emitter perform a key lookup and apply the reciprocal MIS weight. Primary and Dirac hits, unsupported emitters, and
zero-width CDF entries keep hit weight 1. The camera setting `emissive_triangle_nee_enabled` and
`--preview-emissive-nee enabled|disabled` capture override disable only this estimator and its hit competitor; hit-time
emission remains available for matched energy tests.

Opaque deferred pipelines currently enable the fixed raster material backend. Direct draws bind per-material descriptor
sets per draw. When indirect rendering is enabled, `DeferredGeometryPass` uses material-batched indirect ranges: each
contiguous range has one material descriptor, one compatible pipeline-state key, one push-constant base instance, and an
offset/count into the shared indirect command buffers. This restores deferred mesh indirect rendering without returning
opaque raster material sampling to bindless texture arrays. These opaque material-producing pipelines use a raster
material per-frame descriptor set that keeps the shared per-frame buffers but omits bindless texture and cubemap array
bindings.

Built-in shadow-map passes treat all mesh materials as opaque. They use texture-free depth shaders, do not bind raster
material descriptor sets, do not sample material textures for alpha discard, and retain fixed pass-level culling rather
than material- and transform-aware logical facing. Alpha-cutout silhouettes and mirrored single-sided shadow casters are
therefore deliberate follow-up work. This keeps regular mesh shadow draws on the opaque shadow indirect command path when
indirect rendering is enabled. Transparent mesh pipelines still use the fixed raster material backend and bind the
material descriptor set before direct material-sampling draws. Package or external forward callbacks that evaluate glTF
raster materials are explicit migration fallbacks until their owners provide fixed material descriptors or
material-batched submission.

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
asset-owned. Each animated skinned renderer owns a persistent updateable BLAS and a fixed packed ray-payload topology.
Bone-only changes remap the deformed vertices into that topology, update the existing GeometryStorage vertex range, and
record an in-place BLAS update before TLAS maintenance. A BLAS content generation forces a TLAS update even when its
device address and instance bytes are unchanged, so deformed bounds remain current. Pose and generation state commit only
when the frame is submitted; discarded frames retry both the payload and BLAS update.

Ray cameras share one GLSL estimator in `CameraRayIntegrator.glsl`. It owns path depth, direct-light and environment MIS,
BSDF sampling, volume transport, throughput, Russian roulette, invalid-radiance rejection, configurable firefly clamping,
accumulation, and Auto SPP convergence. `CameraRayTracingTraversal.glsl` adapts that estimator to the Vulkan ray-tracing
pipeline and payload shaders; `CameraRayQueryTraversal.glsl` adapts it to inline RayQuery traversal from a compute shader.
The active `.rgen` and `.comp` files are stage-specific entry points only. `CameraLegacy` remains a separate fallback.

Acceleration structures are a shared capability rather than an RT-pipeline capability. Static, skinned, and particle
BLAS data, TLAS updates, geometry descriptors, and synchronization are available when either RT pipelines or RayQuery are
enabled. RT pipelines, shader binding tables, SER, point-cloud ray tracing, and DDGI ray diagnostics remain RT-only.
RayQuery pipeline creation and dispatch require only acceleration-structure and RayQuery support; an unavailable requested
ray mode falls back to the other ray technique before rasterization.

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
| Shared ray estimator and traversal adapters | `EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/CameraRayIntegrator.glsl`, `CameraRayTracingTraversal.glsl`, `CameraRayQueryTraversal.glsl` |
