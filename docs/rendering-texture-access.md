# Rendering Texture Access

[Back to rendering](rendering.md)

EvoEngine classifies sampled textures by ownership and lifetime. Bindless and fixed descriptors are access mechanisms,
not mutually exclusive texture types: both may reference the same image allocation, image view, and sampler without
duplicating uploaded texels.

| Category | Ownership | Descriptor policy |
| --- | --- | --- |
| `PersistentSampledAsset` | Asset-backed data whose identity outlives one pass or frame. | Use the global 2D or cubemap descriptor array when a GPU-selected index chooses the asset. A fixed descriptor is permitted for a bounded conversion, preview, export, or compatibility consumer. |
| `TransientPassResource` | Attachment, render target, history image, or other pass-owned sampled input. | Keep it in the render graph or pass descriptor set. Never allocate a persistent global-array slot for it. |
| `BoundedPassArray` | A deliberately bounded pass-local collection. | Keep an explicit descriptor or descriptor array when the pass owns the bound and selection contract. |

The global arrays are set 0 binding 9 for 2D sampled views and binding 10 for cubemap sampled views. Rasterization, ray
tracing, and ray query share these logical index spaces. Per-frame descriptor sets may mirror the arrays for safe
frames-in-flight updates, but an index must identify the same sampled view in every mirror and render path.

## Standard Renderer Decisions

| Resource | Category | Access |
| --- | --- | --- |
| Standard glTF material maps | `PersistentSampledAsset` | Shared 2D array for raster and ray material evaluation. |
| BRDF LUT, skybox, irradiance, and prefiltered environment | `PersistentSampledAsset` | Shared 2D/cubemap arrays. |
| Global and local reflection probes | `PersistentSampledAsset` | Shared cubemap array; a bake or preview pass may also bind the same sampled view explicitly. |
| G-buffer, ambient occlusion, SSR inputs and histories, render targets | `TransientPassResource` | Camera render-graph or pass descriptors. |
| Shadow maps and DDGI irradiance/visibility atlases | `BoundedPassArray` | Explicit lighting/DDGI pass descriptors. |
| Editor thumbnails and previews | Depends on the source resource | Remain explicit unless they consume a shared persistent sampled view. |
| Package-owned render passes | Declared by the package owner | Persistent GPU-selected assets use the global arrays; bounded or transient inputs remain explicit. |

The standard raster material, IBL, and local-reflection-probe consumers use the shared arrays. The remaining raster
lighting descriptor set contains only pass-local camera inputs such as ambient occlusion.

## Runtime Diagnostics

`TextureStorage` exposes read-only diagnostics for each logical array: configured capacity, current occupancy,
high-water mark, pending/retiring/reusable counts, descriptor and registration revisions, full mirror rebuild count,
descriptors written, descriptor-update CPU time, per-mirror descriptor metadata bytes, and overflow attempts.
`RenderLayer` additionally exposes the 2D and cubemap revisions applied to every frame-safe descriptor-set mirror.

The headless dual-camera rendering gate emits these values together with rebuilds per frame, descriptors per rebuild,
fixed material descriptor binds, draw calls, command-recording CPU time, GPU frame time, and total descriptor metadata.
The reported timing values are observations, not pass/fail thresholds.

The 2026-08-30 Release snapshot for the 2560x1440 dual-camera Rendering Demo recorded 126/2048 occupied 2D slots and
25/256 occupied cubemap slots, zero pending/retiring slots, zero overflow attempts, and 110,592 bytes of descriptor
metadata across two frame mirrors. The stable 2D array rebuilt once. Dynamic reflection-probe writes advanced the
cubemap descriptor revision and caused 1,543 full 256-entry mirror rebuilds over 1,803 frames (0.856 per frame), for
9.65 ms total descriptor-update CPU time. The sampled frame recorded 3.59 ms command-recording CPU time, 31.67 ms GPU
span, 472 draw calls, and zero fixed material descriptor binds. This rebuild frequency is a candidate for a separately
authorized dirty-slot or batched-write optimization; the correctness migration deliberately retains full-array
revision rebuilds.

No pre-migration timing checkpoint was captured with the same executable, scene, driver, and profiler settings, so a
numerical before/after performance delta would not be valid. The structural delta is verified: standard material,
global IBL, and local-probe fixed asset bindings are absent, while the measurements above establish the reproducible
post-migration baseline and identify the cubemap rebuild cost that should be compared in future optimization work.

## Review Gate

`Scripts/check_texture_access_policy.py` inventories tracked first-party global sampled-image declarations and host
sampled-image descriptor bindings. Its reviewed policy is
`EvoEngine_Tests/ShaderPolicy/texture-access-policy.json`. A source change that adds, removes, or moves an inventoried
record fails `TextureAccessPolicy.FirstParty`. Refreshing the file preserves existing classifications and marks new
records `UNCLASSIFIED`; every new record must be assigned one of the three categories before the gate passes.

New persistent systems selected by GPU data use the global arrays. New transient or bounded pass-local systems use
explicit descriptors unless their ownership contract is deliberately changed and the inventory classification is
reviewed.
