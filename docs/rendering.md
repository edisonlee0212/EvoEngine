# EvoEngine Rendering

[Back to README](../README.md)

This page is the high-level map for EvoEngine's rendering stack. Detailed DDGI behavior, demo setup notes, and capture
commands live in separate focused pages:

- [DDGI](ddgi.md)
- [Reflection probes](reflection-probes.md)
- [Rendering demos](rendering-demos.md)
- [Rendering validation](rendering-validation.md)
- [CSM validation](csm_validation.md)

## Ownership

`RenderLayer` owns the SDK render frame. It builds the built-in graphics, compute, and ray-tracing pipelines, prepares
render instance storage, records shadow and camera commands, invokes external render callbacks, renders gizmos, and hands
camera textures to post-processing.

The environmental-lighting ownership refactor separates visible background, scene-global specular fallback, local
reflection probes, and DDGI authoring. The target ownership is:

- `Camera` owns visible background source selection and `background_intensity`;
- `Scene` owns the explicit global `GlobalReflectionProbe` fallback and an optional `EnvironmentalLighting` asset
  reference;
- `EnvironmentalLighting` owns local reflection-probe definitions, DDGI authoring/settings, the shared indirect
  environment source, `environment_lighting_intensity`, `diffuse_fallback_intensity`, and
  `specular_fallback_intensity`, plus the shared reflection-probe bake background.

`RenderLayer` remains the renderer orchestrator. It consumes resolved scene lighting inputs and exposes runtime state in
the editor inspector, but it should not be the authoring owner for DDGI volumes, local probes, camera background, or global
fallback selection.

`Camera` owns the requested render technique. The supported render modes are:

| Mode | Path |
| --- | --- |
| `Rasterization` | GBuffer/deferred lighting plus forward transparent rendering. |
| `RayTracing` | Vulkan ray-tracing pipeline when the device supports ray tracing. |
| `RayQuery` | Compute ray-query path when the device supports RayQuery. |

If a requested ray mode is unavailable, the camera falls back to the best supported mode and reports the fallback once.

## Environmental Lighting Ownership Target

The renderer resolves `Scene + EnvironmentalLighting` into a `ResolvedEnvironmentalLighting` runtime view. New scenes and
loaded scenes with an empty `Scene::environmental_lighting` reference create a temporary `EnvironmentalLighting` asset and
link it to the scene. That default asset resolves to the engine-default indirect environment source with
`environment_lighting_intensity = 1.0f`, `diffuse_fallback_intensity = 1.0f`, and
`specular_fallback_intensity = 1.0f`. The resolver still keeps a defensive no-asset default for malformed in-memory state;
normal scene creation and loading should not hit it.

Current implementation status: the `EnvironmentalLighting` asset schema, `Scene::environmental_lighting` reference,
resolver, and renderer consumption path exist. Temporary scene-level environmental lighting and global reflection probe
assets are serialized into the scene's `LocalAssets` storage. The renderer consumes the resolved `EnvironmentalLighting`
asset for local probes and DDGI volumes. There is no scene-local `ReflectionProbe` or `DdgiVolume` runtime, inspector,
serialization, or extraction path.

The Environmental Lighting inspector groups asset authoring into **General**, **DDGI**, and **Reflection Probes** tabs.
The tab selection is editor-session state only and does not affect serialization or runtime behavior.

Visible background and lighting are independent:

```text
Camera primary miss:
  Camera::BackgroundSource * Camera::background_intensity
```

Camera background changes must not alter DDGI, diffuse IBL, global specular fallback, local probes, ray-camera environment
lighting, or reflection-probe bake input.

`Camera::BackgroundSource` currently resolves visible primary misses through these modes:

| Source | Visible input |
| --- | --- |
| `Clear Color` | `CameraSettings::clear_color.rgb * CameraSettings::background_intensity`. |
| `Cubemap` | `Camera::skybox`, falling back to the engine default skybox when unset. |
| `Environmental Map` | `Camera::background_environment.environment_cubemap`, falling back to the engine default skybox when unset. |
| `Inherit Environmental Lighting` | The scene/environmental-lighting indirect sky source for convenience, falling back to the engine default skybox when unset. |
| `Engine Default Skybox` | The engine default skybox. |

Raster diffuse indirect resolves as:

```text
valid finite DDGI gather at shaded point
  -> accumulated DDGI irradiance

otherwise:
  -> ResolvedEnvironmentalLighting::indirect_environment_source
     * diffuse_fallback_intensity
```

The engine-default indirect environment source supplies the source payload when no `EnvironmentalLighting` asset is
assigned, but the fallback contribution still uses the resolved fallback intensity.

`environment_lighting_intensity` is a source sampling/input scale, not a final surface-lighting multiplier. It scales
environment radiance when the indirect source is sampled by DDGI miss rays and ray-camera environment events. It does not
multiply raster diffuse/specular fallbacks, final valid DDGI irradiance, a reflection-probe bake background, or final valid
local reflection-probe samples.

Raster specular IBL resolves as:

```text
valid local reflection probe at shaded point
  -> baked prefiltered payload * local probe intensity

otherwise, or for remaining local-probe blend weight:
  -> scene global reflection probe fallback
     * specular_fallback_intensity
  -> engine default global reflection probe, when the scene fallback is missing or not ready
     * specular_fallback_intensity
```

The engine default global reflection probe is a `Resources`-owned `GlobalReflectionProbe`, separate from the default
`EnvironmentalMap`. `EnvironmentalMap` supplies diffuse irradiance, unfiltered cubemap radiance, and sampling PDF data; it
does not own the prefiltered specular fallback.

The shaded world position selects local probes. Ray cameras do not sample local reflection probes, because the ray path
traces scene geometry directly. Ray-camera environment lighting resolves as:

```text
Primary ray miss:
  Camera::BackgroundSource * Camera::background_intensity

Surface/volume environment sampling and secondary misses:
  ResolvedEnvironmentalLighting::indirect_environment_source * environment_lighting_intensity
```

Ray cameras do not use `Scene::global_reflection_probe_fallback` as their environment radiance source; that reference
provides the raster/global prefiltered specular fallback payload. Ray cameras ignore `diffuse_fallback_intensity` and
`specular_fallback_intensity` because they do not have a DDGI-missing or reflection-probe-missing fallback path.

DDGI probe-ray misses and diffuse IBL fallback use the same indirect environment source but independent intensity controls.
DDGI miss radiance is `indirect_environment_source * environment_lighting_intensity`; raster diffuse fallback is
`indirect_environment_source * diffuse_fallback_intensity`. Valid DDGI surface irradiance is used as accumulated.
Reflection-probe baking uses its own camera-style Background source, color/assets, and intensity for visible miss pixels.
Bake background intensity is independent of Environmental Lighting intensity and both fallback factors.

The runtime model is asset-owned and has no legacy scene-component compatibility path.

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
intentionally absent.

| Binding | Current image | Current payload |
| --- | --- | --- |
| 17 | Camera depth | NDC depth. |
| 20 | Base color / AO | `rgb = evaluated linear base color`, `a = evaluated occlusion`. |
| 21 | Normal / roughness | `xyz = world normal`, `a = evaluated roughness`. |
| 22 | PBR / flags | `x = evaluated metallic`, `yzw = evaluated dielectric/specular F0`. |
| 23 | Emissive | `rgb = evaluated coated emissive radiance`, `a = scalar specular F90`; a negative alpha marks unlit. |
| 24 | Utility | `x = instance index`, `y = instance info index`, `z = material index`, `w = reserved`. |

`StandardDeferred.slang` evaluates GLTF material state once during geometry and writes only the expanded payload.
`StandardDeferredLighting.slang`, `StandardDeferredLightingSceneCamera.slang`, SSR, AO, TAA, scene-camera debug
visualization, editor GBuffer preview images, and editor mouse picking decode material, normal, or selection state from
bindings 20-24. Editor picking reads the instance index from Utility.x.

The deferred path stores ordinary opaque shading state in the geometry pass using this logical schema:

| Logical attachment | Initial format target | Payload |
| --- | --- | --- |
| Base color / AO | `VK_FORMAT_R16G16B16A16_SFLOAT` | `rgb = linear base color`, `a = occlusion`. |
| Normal / roughness | `VK_FORMAT_R16G16B16A16_SFLOAT` | `xyz = world normal`, `a = roughness`. A later compact packing may replace full-vector normal storage after validation. |
| PBR / flags | `VK_FORMAT_R16G16B16A16_SFLOAT` | `x = metallic`, `yzw = dielectric/specular F0`. |
| Emissive | `VK_FORMAT_R16G16B16A16_SFLOAT` | `rgb = coated emissive radiance`, `a = scalar specular F90`; a negative alpha marks unlit. |
| Utility | `VK_FORMAT_R32G32B32A32_SFLOAT` | `x = instance index`, `y = instance info index`, `z = optional material index for debug or fallback`, `w = reserved`. |

The first supported shading model is opaque/default-lit GLTF. Metallic-roughness materials produce base color, metallic,
roughness, and derived F0. Specular-glossiness materials retain the Khronos diffuse term, independent colored F0, and
`roughness = 1 - glossiness`; they are no longer approximated as metallic-roughness.
Masked alpha remains a geometry-pass discard. Transparent blend, transmission, diffuse transmission, volume/scatter,
clearcoat, sheen, anisotropy, iridescence, and other special lobes stay on their existing transparent, forward, ray, or
documented fallback paths until their GBuffer representation is implemented.

Ordinary opaque lighting must not call `EE_EVALUATE_GLTF_RASTER_SURFACE`; material texture sampling during lighting is
allowed only for an explicitly documented fallback or debug path.

### Raster Texture Descriptor Contract

Raster material descriptors use descriptor set 3. Set 0 remains the per-frame set, set 1 remains available for
meshlet/bone/instanced/strand data, and set 2 remains available for lighting or pass descriptors. Pipelines that need the
material set but do not use intermediate sets should bind empty layouts for the unused set slots.

Raster material texture sampling targets fixed individual texture bindings rather than bindless descriptor arrays.
Raster material shaders must not use runtime bindless descriptor arrays, `nonuniformEXT`, or dynamic material-texture
indexing. Fixed-size image arrays and atlases are allowed when the descriptor itself is a normal fixed binding, such as a
shadow-map array, a DDGI atlas, or the 32-entry local-reflection-probe array. The local-probe sampler uses a bounded switch
so it does not require descriptor-indexing features.

| Material binding | Fixed raster texture | Fallback |
| --- | --- | --- |
| 0 | Base color or diffuse | White. |
| 1 | Metallic-roughness or specular-glossiness | White. |
| 2 | Normal | Flat normal. |
| 3 | Emissive | Black. |
| 4 | Occlusion | White. |
| 5 | Clearcoat | White. |
| 6 | Clearcoat roughness | White. |
| 7 | Clearcoat normal | Flat normal. |

Raster material descriptor sets are renderer-owned runtime state keyed by material index. Descriptor sets are not
deduplicated across material indices because material indices can change while the renderer is running. Each descriptor
slot uses the texture's existing combined image sampler. Missing, ignored, or pending textures bind the documented
fallback textures.

glTF base-material evaluation shares one texture-info ABI across raster, RT-pipeline, and RayQuery. It carries UV0 through
UV3 independently, applies `KHR_texture_transform` after selecting the extension-overridden coordinate set, multiplies
linear `COLOR_0` RGBA into metallic-roughness base color or specular-glossiness diffuse, and treats `OPAQUE` alpha as
coverage-independent. Ray footprints track separate texel densities for all four UV sets before selecting a texture gradient; zero
footprints use explicit mip 0 outside fragment stages. Color-semantic RGB channels (base/diffuse, emissive,
specular-glossiness, specular color, sheen color, and diffuse-transmission color) use the exact sRGB transfer function
when the texture view does not already decode sRGB. Alpha and data-texture channels remain linear.

Selections outside `TEXCOORD_0` through `TEXCOORD_3` disable only that texture binding and emit an error instead of
silently sampling UV0. Missing tangents are generated with MikkTSpace from the normal texture's selected UV set, including
vertex splits at mirrored or discontinuous tangent charts. Authored tangents remain unchanged.

`GltfShadeMaterial` is the canonical owner of material-facing raster state: `double_sided` selects culling, while alpha
mode and transmission select the opaque or transparent pass. The Material inspector exposes those canonical controls
instead of separate cull/blending overrides so raster, RT-pipeline, and RayQuery cannot silently disagree.

Requested DDS images are preferred and retain authored BC7 mip chains and hardware sRGB decoding; same-stem
PNG/TGA/JPG/JPEG files remain fallback sources when the DDS is absent. Float fallback textures generate a complete mip
chain. Each imported glTF texture carries its authored wrap, magnification, minification, and mip-filter state into the
combined sampler used by raster, RT-pipeline, and RayQuery. sRGB images use hardware sRGB views; if the selected format
cannot generate filtered mips, the CPU fallback decodes RGB to linear, filters each level, and re-encodes it. Alpha remains
linear. The canonical extension parser accepts external `.gltf`, data-URI and buffer-view images, and binary `.glb`
containers; broad compression-extension and non-glTF format parity remain outside this roadmap.

### Advanced glTF Ray Materials

The shared RT-pipeline/RayQuery material path imports and evaluates the ratified `KHR_materials_iridescence`,
`KHR_materials_anisotropy`, and `KHR_materials_dispersion` extensions. Iridescence intensity uses texture R, thin-film
thickness uses texture G, and anisotropy uses normalized texture RG with strength in B. These are linear data textures and
reuse the same four-UV, `KHR_texture_transform`, storage-flip, and ray-footprint behavior as the base material inputs.
Dispersion has no texture and is evaluated only by the specular-transmission lobe.

Ray materials keep one fixed full descriptor and structure ABI for every scene. A thin RTX, RayQuery, or any-hit
entrypoint receives the promoted scene-feature mask and passes it as a typed Slang value generic into the imported material,
BSDF, traversal, emissive-sampling, and integrator functions. This preserves stable host/device layouts while allowing Slang
to eliminate extension paths that the scene cannot use. Shared imported modules do not consume preprocessor configuration;
the all-feature entrypoint default exists only for explicit startup fallback pipelines.

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
  fractional factors do not incorrectly approach white. New material assets use the Khronos default of 1;
- `KHR_materials_ior.ior: 0` uses the specification's positive-infinity convention: surface F0 is 1, transmission uses a
  finite infinity surrogate for stable arithmetic, and dispersion is disabled. Invalid authored IORs between 0 and 1
  normalize to 1 rather than being interpreted as this convention;
- unlit raster and ray materials return base color only, without adding emissive or lighting first. Diffuse or glossy
  transmission events both update the current volume medium.

EvoEngine's retroreflection material fields are experimental and not tied to a Khronos glTF extension registry entry. The
shared ray BSDF applies the Minimal Retroreflective Microfacet Model view substitution to reflection lobes and leaves
transmission conventional. Evaluation and sampling both report the same marginal forward/retro mixture BSDF and PDF.
Sample throughput is the full mixture BSDF divided by that marginal PDF; it does not divide again by the selected branch
probability, which would over-brighten fractional blends.

Opaque raster stores IOR/specular-aware colored F0 plus scalar F90; the emissive attachment alpha carries F90, with a
negative value reserved for the unlit lighting bypass. Ray shadow transmission samples the specular-transmission,
base-color, diffuse-transmission factor/color, specular factor/color, and vertex-color inputs. It applies Fresnel remaining
energy first, then layers diffuse transmission only over the `(1 - specularTransmission)` share. The current fixed raster
material descriptors include clearcoat factor, roughness, and normal textures so authored clearcoat-normal scale and
coated-emission attenuation work in opaque and transparent raster as well as RTX and RayQuery. The deferred raster
lighting model still does not add a clearcoat reflection lobe or encode iridescence, anisotropy, dispersion, or
retroreflection, so those effects remain ray-path features rather than claimed raster parity. The CPU/debug ray display is
likewise not an advanced-material integrator.

Native `KHR_materials_pbrSpecularGlossiness` remains a distinct diffuse/F0/glossiness model; EvoEngine does not reproduce
the pinned reference's lossy metallic-roughness conversion. Khronos material extensions that explicitly exclude unlit or
specular-glossiness are rejected per material with an error diagnostic. The importer preserves the material-array entry
and its primary unlit or specular-glossiness model while ignoring only conflicting extension factors and textures.

Clearcoat emission uses `emission * (1 - clearcoat * clearcoatFresnel)`. Ray hit emission and emissive-triangle NEE call
the same helper; the NEE path reconstructs the sampled emitter's UVs, tangent basis, emissive/clearcoat textures, and
clearcoat normal so MIS never combines differently coated radiance values.

### Emissive-Triangle Next-Event Sampling

RT-pipeline and RayQuery cameras plus RT-pipeline DDGI share one emissive-triangle distribution. Eligible emitters are
fill-mode rigid, skinned, particle-instanced, and override-BLAS/triangle-range instances collected by the deferred,
forward, or transparent paths. External DDGI geometry participates when it supplies its packed triangle count as well
as the existing offset. Every entry must have a triangle material payload matching the TLAS; strands, Gaussian splats,
unlit materials, and external geometry without a count remain hit-only emitters.

Each entry identifies the packed `GeometryStorage` instance/primitive pair used by the BLAS. A numerically robust alias
table gives O(1) selection and preserves positive support across extreme area/power ratios. The proposal uses world-space
triangle area times a seven-point estimate of emissive-texture luminance, including UV0-UV3 selection, texture transform,
sampler wrapping, sRGB decoding, and two-sided importance. If CPU texels are unavailable it falls back to factor
luminance; a small factor-derived support floor prevents a sparse estimate from removing a genuinely emissive triangle.
Masked and blended proposals also estimate per-triangle opacity from base-color alpha and vertex alpha, reducing rejected
samples without conditioning or changing the estimator.
The stored area PDF remains the exact triangle-selection probability divided by world area. Sampled-point and hit-side
emission use the same explicit-LOD material evaluator, so MIS still combines estimators on one integrand.

Each frame slot retains its distribution across `RenderInstanceStorage::Clear()`. An exact ordered signature of the
eligible mesh/material handles, emissive-texture content, packed triangle range, ray instance index, model transform,
and derived importance
gates the triangle walk; unchanged slots restore only the table count and do not transform, sort, compare, or upload the
triangle records again. The key intentionally does not use the global geometry-storage revision, so unrelated skinned
mesh updates cannot invalidate the static-emitter table. The storage buffer is uploaded only when that exact signature
changes.

Emissive NEE is an always-on, independent one-sample estimator in addition to the existing punctual/environment
estimator. It uses
the area-to-solid-angle PDF and balance-heuristic MIS against the BSDF or volume phase PDF. BSDF/phase rays that hit a
table emitter perform a key lookup and apply the reciprocal MIS weight. Primary and Dirac hits and unsupported emitters
keep hit weight 1. Hit-time emission remains available for matched energy tests. Blend-material samples multiply emission
by deterministic sampled opacity, matching their stochastic camera visibility in expectation.

DDGI reuses the same record lookup, alias-table selection, reconstruction, material evaluation, and area-to-solid-angle
PDF without camera BSDF MIS. It applies one Lambert receiver sample with binary visibility and light-technique weight one at each eligible
non-fixed front-face probe hit; direct emitter hits remain unweighted. The default-on global DDGI setting and each
volume's `Inherit`/`On`/`Off` override disable only this explicit estimator, never direct-hit emission.

### Camera-Ray Shader Variants

Normal scene preparation detects the glTF behaviors used by collected materials and requests an exact camera-ray shader
variant. The detector is material-order invariant and promotes volume scatter to volume and transmission. The host and
shader storage ABI always retain every material field; only `EE_GLTF_USE_*` behavior gates vary. RTX compiles the shared
integrator through specialized raygen and any-hit modules while reusing miss/closest-hit modules. RayQuery compiles only
its compute shader and does not depend on an RTX pipeline, SBT, or SER capability.

The all-feature startup pipelines are permanent fallbacks. Missing variants compile asynchronously on the render
executor, remain cached by technique plus feature mask, and publish only at frame preparation. A publication resets the
matching camera histories once. The bounded caches retain an evicted pipeline until its submitted frame has completed.
The editor's Render Layer inspection shows requested and active keys; automated ray captures wait for the requested
variant before counting samples.

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
raster materials are temporary fallback paths until their owners provide fixed material descriptors or material-batched
submission.

Raster lighting uses a fixed raster-global texture descriptor set for image-based lighting inputs instead of sampling the
bindless texture arrays. Deferred lighting binds this set after the shared lighting descriptor set, and transparent mesh
lighting binds it after the material descriptor set. The fixed slots are BRDF LUT, skybox cubemap, irradiance cubemap,
prefiltered global environment cubemap, ambient occlusion, and a fixed array of 32 placed reflection-probe cubemaps. Unused
or unavailable local slots bind the scene/camera global prefiltered cubemap.
Set 2 still owns shared shadow-map and DDGI atlas bindings.

### Environment Lighting Controls

The refactor replaces scattered scene-environment controls with three resolved environmental-lighting scalars:
`environment_lighting_intensity`, `diffuse_fallback_intensity`, and `specular_fallback_intensity`.
`environment_lighting_intensity` scales sampled indirect environment radiance for DDGI probe-ray misses and ray-camera
environment events. It does not scale raster diffuse/specular fallbacks, visible camera or reflection-bake backgrounds,
direct lights, primary emission, accumulated DDGI irradiance, or valid runtime local reflection-probe payloads.

`diffuse_fallback_intensity` is diffuse-fallback-only. It directly scales raster diffuse IBL when no valid DDGI gather is
available. It does not scale DDGI miss radiance, accumulated DDGI surface irradiance, ray-camera environment lighting,
reflection-probe baking, specular fallback, direct light, emission, or visible camera background.

`specular_fallback_intensity` is specular-fallback-only. It scales the scene-global or engine-global prefiltered fallback
when no local reflection probe covers the shaded point, or when local probe blending leaves remaining weight to the
fallback. It does not scale valid local reflection-probe samples, ray-camera environment lighting, reflection-probe
baking, diffuse fallback, direct light, emission, DDGI, or visible camera background.

Local probes keep their own authored `reflection_intensity`. The asset-level **Enable local probe reflections** switch
removes every local probe from runtime selection without disabling authoring, bounds, or baking. Selection uses the shaded world position, not the camera
position, and blends at most one strictly lower-priority boundary probe before returning uncovered weight to the scene
global fallback. Missing, unloaded, disabled, or invalid local payloads also return their weight to the same global
fallback. After selection, rough probe specular is multiplied by the scalar visibility confidence derived from material
AO, eligible GTAO, and DDGI gather visibility. SSAO remains diffuse-only, unavailable GTAO and missing DDGI coverage fall
back to white, and the term never affects direct, emissive, background, diffuse, or DDGI diffuse energy.

The scene-global `GlobalReflectionProbe` fallback supplies the prefiltered specular payload. If it is missing or not
runtime-ready, raster lighting binds the engine default global reflection probe so descriptors remain valid. A zero
`specular_fallback_intensity` makes that bound fallback contribute black. The indirect environment source supplies
  diffuse irradiance, unfiltered radiance, and sampling data for DDGI misses, diffuse IBL fallback, and ray-camera
  environment lighting. DDGI remains diffuse-only and does not become a specular source.
See [Reflection probes](reflection-probes.md) for persistence, selection, baking, and format contracts.

Material and mesh thumbnail rendering uses `AssetThumbnailProvider` and `OffscreenPreviewRenderer`, which build a
temporary scene, upload referenced preview textures, force a raster camera, disable DDGI state, and call
`RenderLayer::RenderSceneToCameraImmediately`. These preview paths do not own separate glTF raster material pipelines, so
material-sampling preview output inherits the same fixed material descriptor layouts and per-draw descriptor binding used
by the normal RenderLayer camera passes.

Volumetric cloud settings, shaders, render passes, and renderer resources remain compiled, but no scene-owned field
currently configures or enables them. Scene serialization ignores old `environment:` cloud settings until a future
ownership model relinks the feature.

Bindless texture arrays are reserved for ray tracing and ray query paths. When ray tracing and ray query are unavailable
or disabled, `RenderLayer` creates the ordinary per-frame descriptor layout without texture or cubemap descriptor arrays
and skips binding the global texture storage arrays. Raster material textures, raster lighting inputs, DDGI atlases,
volumetric cloud textures, pass-local textures, and non-ray-tracing compute texture inputs use fixed material, global, or
pass descriptor sets. Ray tracing, ray query, and ray diagnostics keep their bindless texture access.

Current shadow policy:

- directional CSM defaults to Stable Sphere fitting; Render Layer inspection can switch globally and transiently between
  Stable Sphere and unsnapped Tight Light-Space AABB, and the choice is not serialized;
- split placement uses Practical Log/Uniform distances stored per camera, matching each camera-indexed cascade matrix;
- directional shadows use 16-sample Vogel-disc PCF through a linear comparison sampler; point and spot shadows retain
  their existing 32-sample PCF paths;
- when mesh shaders are supported and enabled, built-in strands render normally and cast directional, point, and spot
  shadows through the mesh-shader backend;
- directional light size is the PCF radius in world units and each fit includes that footprint plus its packed-viewport
  comparison/snap guard and a conservative TAA-jitter envelope;
- both fits include the bounded depth overlap used by cascade-transition blending;
- directional constant, slope, and normal-offset bias are authored in texels and use the packed viewport's corrected
  world-units-per-texel scale;
- directional, point, and spot shadows default to 4096;
- an explicit shadow-map quality override sets directional, point, and spot resolution together.

## Ray Camera Paths

`CameraRenderMode::RayTracing` runs the path tracer through the Vulkan ray-tracing pipeline. It traces camera rays with
the shared glTF material and BSDF modules, accumulates radiance across frames, and supports shader execution reordering
when the device and selected mode allow it.

`CameraRenderMode::RayQuery` runs the same path-tracing integrator through a native-Slang compute shader using inline
RayQuery traversal. The RTX and RayQuery paths share material evaluation, environment and emissive-triangle next-event
sampling, MIS, firefly clamping, Auto SPP convergence, and ray debug views. Backend-specific traversal stays behind the
integrator boundary so the two modes do not fork shading behavior.

Ray cameras default to one sample per pixel per frame. Emissive-triangle NEE and firefly clamping are always active; the
firefly luminance threshold remains configurable.

Ray-camera history is owned by each camera. A single history slot retains radiance, convergence, and enabled optional
output images. Scene changes invalidate every camera, while camera-state changes invalidate only the changed camera;
the Scene camera affects the main camera only when `Copy Transform` is enabled. Render technique and extent changes also
invalidate the owning camera. Per-camera directional-shadow cascade fits are not treated as global scene changes. Optional
outputs use the shared descriptor layout and are allocated only when requested.

Ray-camera shaders under `DefaultResources/Shaders` use the native Slang frontend. Shared modules are imported by module
name, while RTX ray-generation and RayQuery compute entrypoints supply the traversal adapter appropriate to their backend.

## Render Graph And Extension Model

`RenderGraph` declares logical resources, pass queues, access plans, and compiled barrier plans. `RenderLayer` applies
the compiled barriers on the current command path. Queue-family ownership transfer and broader async queue submission are
still future work.

Packages and services extend rendering through registered callbacks and explicit external geometry registration rather
than by mutating built-in pass internals. External geometry can participate in DDGI only when it supplies compatible BLAS
and triangle offset data.

## Current Rendering Notes

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

AA persistence uses `enable_anti_aliasing` and an `anti_aliasing` map containing `algorithm`, `taa`, and `smaa`. Missing or
invalid current AA fields resolve to enabled SMAA Ultra with Best Quality TAA defaults.

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

## SDK Shader Language Boundary

All shaders under `EvoEngine_SDK/Internals/DefaultResources/Shaders` use explicit native Slang dialect markers and
module imports. The SDK tree contains zero shader `#include` directives and zero `.glsl` files. The remaining 16 GLSL
headers are owned by `EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Includes`; EcoSysLab intentionally
keeps its compatibility compile path until its separate Slang migration.

The renderer has moved many built-in resources into explicit graph resources, but some ownership work remains:

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
| Lighting shaders | `EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Lighting.slang`, `LightingFixedSet3.slang`, `LightingFixedSet4.slang` |
| glTF raster material shaders | `EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/GltfRasterMaterial.slang` |
| glTF ray material shaders | `EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/GltfRayTracingBsdf.slang` |
| Shared ray estimator and traversal adapters | `EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/CameraRayIntegrator.slang`, `CameraRayTraversal.slang`, `CameraRayTraversalPolicies.slang` |
