# Rendering Materials And Geometry

[Back to rendering overview](rendering.md)

EvoEngine imports glTF material state into one renderer-owned representation shared by rasterization, ray tracing, and
ray query. Each technique chooses the passes and lobes it can evaluate without changing the authored material.

## Material Workflows

The renderer-owned physically based workflow is metallic-roughness. Materials also carry alpha mode, sidedness,
emissive state, normal mapping, occlusion, and the supported glTF extension parameters.

`KHR_materials_pbrSpecularGlossiness` is accepted only as a glTF import format. Its factors and textures are converted
immediately to core metallic-roughness using Khronos' reference workflow conversion. The conversion is lossy because
core metallic-roughness cannot represent arbitrary dielectric specular color or strength. Textures with incompatible UV
mappings are rebased into the diffuse texture's mapping with an import warning; unreadable source pixels fall back to
the corresponding factors. PNG, JPEG, TGA, embedded images, data URIs, and BC7 DDS alternatives are decoded directly
for conversion without uploading specular-glossiness-only sources. Converted base-color and metallic-roughness mip
chains are stored as disposable BC7 DDS pairs under `Cache/GltfMaterialConversion`; deleting this directory only makes
the next import regenerate them. No specular-glossiness state is stored, edited, serialized, or evaluated at runtime.
`KHR_materials_specular` remains independently supported and is not used by this workflow conversion.

Opaque/default-lit raster materials are evaluated during the geometry pass and stored in the GBuffer. Alpha masking is a
deterministic geometry-pass discard. Blended, transmissive, and other forward-only materials use the transparent or
forward path. Unlit materials return base color without adding lighting or emission first.

The shared ray material path evaluates the same base inputs and supports advanced reflection and transmission behavior,
including:

- clearcoat, sheen, specular, index of refraction, transmission, volume, and diffuse transmission;
- iridescence, anisotropy, and dispersion;
- emissive strength and alpha-aware visibility;
- EvoEngine's experimental retroreflection fields.

Not every advanced lobe has an opaque GBuffer representation. Such materials remain on their forward, transparent, or
ray paths rather than being approximated as ordinary deferred materials.

## Textures And Vertex Inputs

Material textures can select `TEXCOORD_0` or `TEXCOORD_1`. Additional imported mesh coordinate channels are ignored.
`KHR_texture_transform` is applied after choosing the authored coordinate set. A texture binding that selects another
coordinate set is disabled with a diagnostic rather than remapped. Vertex `COLOR_0` multiplies the
metallic-roughness base color.

Deferred rendering stores vertex color as clamped, rounded UNORM8 RGBA metadata. Negative and HDR vertex colors are
therefore not preserved by the deferred path; forward-only paths retain full-float interpolation.

The stable G-buffer metadata attachment is `R32G32B32A32_UINT`: instance and info IDs retain all 32 bits, while the
material word reserves its high two bits for procedural base-color replacement and tangent handedness. Consumers use
integer texel loads; the attachment is never filtered or converted through floating point.

Color textures use sRGB decoding while alpha and data channels remain linear. Imported wrap, magnification,
minification, and mip-filter settings are preserved. Authored DDS/BC7 mip chains are preferred when available; common
glTF image sources remain supported fallbacks.

Authored tangents are preserved. When a normal-mapped mesh has no tangents, the importer generates MikkTSpace tangents
from the normal texture's selected UV set and splits mirrored or discontinuous tangent charts as needed.

Missing textures bind semantic fallbacks: white for multiplicative color/data inputs, black for emission, and a flat
normal for normal maps. An unsupported UV selection disables only the affected texture and reports the problem rather
than silently sampling another coordinate set.

## Raster And Ray Resource Models

Raster materials use fixed texture descriptors. This keeps rasterization available on devices that do not expose the
descriptor-indexing features used by ray traversal. Renderer-owned descriptor state is updated as material indices or
textures change.

Ray tracing and ray query use bindless texture storage because a ray can encounter any material after traversal begins.
They share the same host material and texture-info buffers, scene feature detection, and specialized shader variants.
Missing variants compile asynchronously while an all-feature fallback remains available.

Material and mesh thumbnails render through the normal fixed-descriptor raster path in a temporary scene. They disable
DDGI and do not maintain a separate material implementation.

## Geometry Participation

| Geometry | Raster cameras | Ray cameras | DDGI | Shadows |
| --- | --- | --- | --- | --- |
| Rigid meshes | Deferred, forward, and transparent | Triangle BLAS | Supported | Supported |
| Skinned meshes | Deferred, forward, and transparent | Updated triangle BLAS | Supported | Supported |
| Instanced meshes | Deferred, forward, and transparent | Triangle instances | Supported when triangle metadata is available | Supported |
| Strands | Mesh-shader path when supported | Linear swept spheres when the NVIDIA extension is available | Not traversed | Mesh-shader shadows when supported; LSS ray shadows in ray cameras |
| Gaussian splats | Dedicated raster passes | Not traversed | Not traversed | Not supported |
| External geometry | Owner-provided callback | Owner-provided acceleration structure | Supported only with compatible DDGI triangle metadata | Owner-provided callback |

Emissive-triangle next-event sampling uses eligible triangle geometry. Emissive strands remain visible when directly hit
by a ray camera but are not part of the triangle sampling distribution.

## Technique Differences And Limitations

- Raster and ray cameras share authored material meaning, but their integrators and temporal histories are not expected
  to produce pixel-identical images.
- Built-in raster shadow-map passes treat mesh materials as opaque and do not sample alpha textures for cutout
  silhouettes.
- DDGI transports diffuse irradiance with a bounded material model; it is not a full camera-path BSDF integrator.
- Ray-camera strand traversal is optional and capability-gated. Unsupported devices keep the raster strand path.
- Gaussian splats do not enter triangle acceleration structures, DDGI, or ray-camera traversal.

Implementation and shader ABI details are authoritative in the material buffers, render-pass setup, and Slang modules;
they are intentionally not duplicated in this guide.
