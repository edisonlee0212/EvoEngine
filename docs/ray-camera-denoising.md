# Ray-camera denoising input contract

The ray camera uses NRD's diffuse/specular radiance denoisers. The standard Windows preset includes NRD. When denoising is enabled, Path Tracing and ReSTIR prepare view-Z, motion, and fresh diffuse/specular/excluded RGB images on each frame, dispatch the selected denoiser, and composite its output before post-processing. Path Tracing with sample accumulation averages the diffuse/specular inputs before denoising; ReSTIR and Path Tracing without accumulation feed fresh inputs into NRD's temporal accumulation.

**Accumulated PT input** automatically replaces NRD's temporal accumulation with separate running diffuse and specular PT averages whenever Path Tracing, sample accumulation, and NRD are enabled. The two RGBA32F histories add 32 bytes per pixel (about 63.3 MiB at 1920x1080). Their alpha channels track per-pixel sample counts; reset frames initialize every pixel, including misses. NRD's own history restarts every frame while its spatial filtering remains active; hit distances and guides remain current-frame inputs. Scene or camera changes reset the PT averages. The camera inspector and demo captures use the existing accumulation control (`--preview-accumulate-samples enabled|disabled`) to select this behavior. Old `nrd_accumulated_input` values are ignored when loading saved cameras and editor layouts. Toggle regression captures use `--preview-nrd-toggle-after-frame N` and fail if the toggle reduces accumulated camera frames. For a resize regression capture, `--preview-resize-after-frame N` starts at half the requested resolution and resizes to `--preview-width` × `--preview-height` after frame N.

The dedicated primary-direct **SSS ReSTIR** path runs inside the PT-style camera path rather than the ordinary ReSTIR
pipeline. With accumulation enabled it also feeds running lobe averages and restarts NRD each frame. Its own SSS
reservoir history is separate. Statements below about fresh ReSTIR input refer to ordinary, non-SSS ReSTIR.

| NRD input | Current source | Remaining work |
| --- | --- | --- |
| World normal and linear roughness | The guide traces signed world normals and primary linear roughness independently of the optional `Camera.RayNormal` output. GGX alpha axes are reduced by geometric mean and square-rooted back to NRD linear roughness. All NRD-enabled integrators use pixel-center primary guides while beauty samples remain jittered. The guide pass packs the normal and roughness into `Camera.NrdNormalRoughness` using NRD's R10G10B10A2 octahedral formula. | Guide alpha encodes opaque (0), metal (1), and transmission (2) categories. Both denoisers reject category boundaries. Subpixel geometry coverage can still differ between the guide and the sampled signal. |
| View Z | The NRD guide pass traces the center primary world position; ReSTIR also stores sampled primary surfaces in the 48-byte `RestirPtPrimarySurface`; `Camera.RayHitDistance` stores primary ray distance, which is not view Z. `RestirPtNrdGuides.slang` writes signed view Z and a finite out-of-range miss value to `Camera.NrdViewZ` in NRD-enabled builds. | Validate deforming geometry and unusual primary hits. |
| Motion | `RestirPtNrdGuides.slang` reconstructs previous world position from `previous_instances` and writes normalized non-jittered 2.5D motion, including the positive-depth delta `abs(previousViewZ) - abs(currentViewZ)` required by NRD internal reprojection, to `Camera.NrdMotion` in NRD-enabled builds. | Validate invalid history and deforming geometry. Skinned deformation requires previous-pose reconstruction. The rounded `screen_motion` used for ReSTIR pairing is not an NRD guide. |
| Diffuse and specular radiance | Candidate resolve routes its selected path by primary BSDF lobe. Spatial and Enhanced combine route each central/neighbor term before summing. Primary emission and background go to `Camera.NrdExcludedSignal`. The center guide records primary diffuse/specular material factors. Resolve divides the noisy signals by these factors, and the composite multiplies the denoised signals by them. | Validate complex materials and reused-path behavior beyond Sponza. |
| Diffuse and specular hit distance | The center guide traces separate diffuse/specular distance probes from the receiving surface. Signal packing places these complete representative distances in both lobe alpha channels; hit-distance reconstruction is disabled. `Camera.RayHitDistance` remains the camera-to-primary distance. | Validate representative distances on layered materials, mirrors and subpixel geometry. |

ReSTIR denoising uses a fresh per-frame signal even when camera accumulation is enabled. With multiple ReSTIR samples per
frame, each complete estimate contributes to the averaged diffuse, specular, and excluded signals before NRD runs.
The final sample supplies the primary-hit guide and material factors used for denoising. ReSTIR moving-camera and
moving-object captures retain NRD history while raw camera accumulation resets. When NRD is active, its surviving
history frame count is independent of the advancing path-sampling seed. Compare denoised and raw output at
equal GPU time.
In NRD-enabled builds, multiple ReSTIR samples use a 48-byte-per-pixel accumulation buffer for each frame in flight.

SSS occupies material category 3 in addition to opaque, metal and transmission. That category distinguishes SSS from
other lobes, but does not distinguish individual scattering profiles.

## Pass boundary

ReSTIR combine writes per-lobe RGB while each source reservoir is known. Resolve validates the radiance and adjusts those signals before accumulation. The guide pass derives view Z and motion from primary surfaces and previous-instance transforms; the center guide supplies matching primary material factors and independent receiving-surface lobe distances. RELAX consumes demodulated fresh signals, and the composite remodulates its output before tone mapping.

PT writes fresh RGB lobe signals and uses the shared NRD guide pass for center normals, material factors, motion and lobe distances. The signal-pack pass demodulates PT after that guide pass, avoiding a second center-guide trace inside the PT shader. ReSTIR resolves continue to demodulate their signals before packing.

Path Tracing and ReSTIR share the guide, RELAX, and composite passes. Path Tracing routes each fresh sample by its primary BSDF lobe before camera accumulation. ReSTIR Candidate tags its selected path; Spatial Only and Enhanced classify each contribution during combine, while its source reservoir is still known. Temporal Only classifies its merged reservoir during history resolve. Primary emission, unlit surfaces, and background remain outside the denoised lobes. The composite limits unusually bright excluded primary emission after the first accumulated frame, uses raw accumulated radiance for guide misses, and keeps the denoised lobe output at every accumulated sample count. Raw accumulated radiance is not blended back into valid denoised pixels.

RELAX uses complete per-lobe hit distances without reconstruction, zero diffuse and specular prepass radii, and two A-trous iterations by default. REBLUR uses a 10-pixel maximum blur radius by default, the configurable settings below, and complete per-lobe hit distances without reconstruction. Its pack pass converts demodulated RGB to YCoCg and normalizes hit distance with NRD's default REBLUR parameters; the composite converts its output back to RGB. All NRD-enabled integrators trace a pixel-center primary guide for stable view Z, normal/roughness, and motion; the beauty path remains stochastic. NRD history remains valid through camera and rigid-object motion when the input history is valid. Current and previous projection matrices passed to NRD flip clip-space Y to match its top-left UV convention to the ray images' bottom-left UV convention. Motion Z uses positive depth differences because NRD converts right-handed matrices and signed depth internally.

Historical quality/timing measurements are retained in [denoising validation notes](ray-camera-denoising-validation.md).
They are not a current benchmark of accumulated-input rendering.

NRD's [input contract](https://github.com/NVIDIA-RTX/NRD/blob/master/README.md#inputs) defines the guide and signal requirements. Its [SDK license](https://github.com/NVIDIA-RTX/NRD/blob/master/LICENSE.txt) governs integration and distribution of the library.

## RELAX inspector controls

Select **NRD Denoiser > RELAX** and expand **RELAX Settings**. Controls are saved in scene cameras and editor layouts; changes apply on the next frame and restart NRD history while retaining raw PT accumulation. **Reset RELAX settings** restores the defaults below.

| Control | Default | Purpose |
| --- | --- | --- |
| Diffuse / specular prepass radius | 0 / 0 px | Filtering before history accumulation; zero disables the corresponding prepass |
| Diffuse / specular history frames | 30 / 30 | Independent history limits from 0 to 255 |
| A-trous iterations | 2 | Spatial filter iterations, adjustable from 2 to 8 |
| Normal rejection fraction | 0.5 | Larger values accept more normal variation |
| Roughness rejection fraction | 0.15 | Larger values accept more roughness variation |
| Diffuse / specular luminance sensitivity | 2 / 1 | Luminance edge stopping during spatial filtering |
| Depth rejection threshold | 0.003 | Normalized depth threshold for spatial filtering |
| Anti-firefly | Off | NRD firefly suppression |
| Roughness edge stopping | On | Reject filtering across roughness differences |

Fast history and history reconstruction are bounded by the selected lobe history limits. Prepass filtering is disabled by default in both denoisers. Checkerboard, reconstruction, confidence and material-input conventions remain tied to the engine's signal contract.

A-trous iterations apply edge-aware spatial filtering with increasing sample spacing, reaching a wider neighborhood
without a dense large kernel. More iterations cost additional passes and can soften detail; they do not add PT samples.
RELAX's optional anti-firefly filter is off by default. REBLUR retains the pinned NRD default anti-firefly filter (on),
without a separate editor toggle. These denoiser filters differ from PT's raw-output display filter; neither is a
firefly clamp on the PT accumulation. High-energy input outliers can still harm denoising.

The history controls bound NRD temporal blending, not a ring of full-resolution saved frames. Raising RELAX's limit
to 255 does not allocate 255 images. In accumulated-input mode these temporal limits are overridden to zero and NRD
uses `RESTART`; they affect fresh-input operation instead. NRD Temporal AA remains a separate history after denoising.

## Windows build

The Vulkan device enables `VK_KHR_push_descriptor` when available for NRD's root descriptors. The NRI wrapper receives that enabled extension and a Vulkan version capped at the engine's enabled 1.3 feature contract. Hardware support for a newer Vulkan version does not enable its optional device features. NRD initialization declines devices without push-descriptor support.

The standard `vs2026-x64` preset enables NRD and uses the source checkout at `Extern/3rdParty/NRD/NRD`. EvoEngine builds NRD with NRI and SPIR-V shaders and installs their runtime libraries and license files with its applications and runtime templates. Other configurations can still disable the build dependency with `EVOENGINE_ENABLE_NRD=OFF`. In an NRD-enabled editor, the camera inspector's **NRD Denoising** checkbox switches Path Tracing and ReSTIR Candidate, Spatial Only, Temporal Only, and Enhanced between raw and denoised output; **NRD Denoiser** selects RELAX or REBLUR. New cameras default to REBLUR; saved cameras retain their explicit selection. Switching NRD on/off or changing its settings resets only denoiser history; accumulated raw PT radiance is retained. Restarted NRD input averages initialize independently of the raw PT sample count. Off skips the NRD guide, denoiser, and composite passes. Fresh raw-mode cameras do not allocate guide or denoised-output images. Temporal Only also uses raw shader variants for candidate, combine, and resolve that skip NRD signal work; other raw-mode shaders still prepare split signals. Demo captures can set `--preview-nrd-denoising enabled|disabled` and `--preview-nrd-denoiser relax|reblur` to compare outputs from one executable. The pinned checkout is NRD 4.17.3 at `792eff19` with NRI v179.

## REBLUR controls

With a ray camera selected, enable **NRD Denoising**, select **REBLUR**, and expand **REBLUR Settings**:

| Control | Default | Purpose |
| --- | --- | --- |
| Minimum blur radius | 1 px | Spatial smoothing after convergence |
| Maximum blur radius | 10 px | Maximum base spatial smoothing radius |
| Diffuse prepass radius | 0 px | Diffuse filtering before accumulation; zero disables it |
| Specular prepass radius | 0 px | Specular filtering and motion tracking before accumulation |
| Maximum history frames | 30 | NRD history limit, adjustable from 0 to 63 |
| Normal rejection fraction | 0.15 | Lobe angle fraction; larger values allow more normal variation |
| Roughness rejection fraction | 0.15 | Larger values allow more roughness variation |

Edits apply on the next frame and restart NRD history while retaining raw PT accumulation, without
recreating the denoiser or recompiling shaders. **Reset REBLUR settings** restores the
values above. Settings persist in scene cameras and editor camera layouts; older files
use the defaults. NRD history is separate from ReSTIR's temporal reservoir cap; **Accumulate Samples** selects
the input/history policy described above. Fast history and history reconstruction are
bounded by the selected history limit. The minimum blur radius cannot exceed the maximum.
New cameras default to RayQuery with Path Tracing, RGB REBLUR, NRD denoising and NRD Temporal AA enabled. Saved render modes, integrator choices and settings are retained.

SH denoising has been removed after comparisons showed no clear visual improvement.
RGB REBLUR and RELAX remain available. ReSTIR reservoirs use 128 bytes, spatial
shift records use 32 bytes, and multiple-sample NRD accumulation uses 48 bytes per pixel.

Motion comparisons use 1920x1080 Sponza. `--preview-camera-rotate-each-frame x,y,z`
applies world-space Euler angle increments in degrees; translation uses
`--preview-camera-translate-each-frame x,y,z` in world units.

## Receiving-surface hit distances and temporal AA

The guide pass traces one diffuse cosine-weighted ray and one specular GGX VNDF ray
from the pixel-center receiving surface, using the existing BSDF sampler and alpha-aware
ray traversal. These are short distance probes, not additional full lighting estimates.
They provide both lobe distances independently of candidate selection and temporal/spatial
reuse. A miss uses 65504; rejected GGX samples use a mirror-direction probe because they
have no lighting contribution. The probes describe local receiving geometry, rather than
copying a source reservoir's distance. They remain an approximation to the effective
multi-path signal; perfect mirrors, multilayer materials and subpixel coverage warrant
further scene validation.

The BSDF stores squared roughness (GGX alpha). Guides and NRD material factors use
its square root, matching the Vulkan sample: isotropic roughness 0.5 has alpha 0.25
and must reach NRD as 0.5. Anisotropic axes use the fourth root of their product.

Guide normals, roughness, depth, categories and material factors come from the same
center hit. Each independent ReSTIR estimate prepares these factors before combine;
Path Tracing demodulates its RGB signals after the center factors are available.
The signal pack pass supplies representative hit distances to both RELAX and REBLUR;
only REBLUR performs YCoCg conversion and hit-distance normalization.

**NRD Temporal AA** adds optional color stabilization after denoising and before bloom/
tone mapping. It is enabled by default and saved with camera settings. Each bilinear
history tap is reprojected using the existing 2.5D motion and rejected for depth,
normal, roughness, material or finite-value mismatch. Accepted history is clamped to
the current 3x3 color neighborhood and blended with at least 10% current color.
Disocclusions, guide misses and invalid history use current color. Camera motion keeps
history for reprojection; integrator/settings/scene/resolution changes reset it.
Two RGBA16F color and two RG32UI geometry histories cost about 63 MiB at 1080p.
Geometry preserves the guide's 10-bit normal/roughness channels and full 32-bit depth.
`--preview-nrd-temporal-aa enabled|disabled` allows matched captures. This smoothing
cannot recover missing lighting information or replace correct NRD signals.

Reference audit: [nvpro-samples/vk_denoise_nrd](https://github.com/nvpro-samples/vk_denoise_nrd)
at `a9119d11`. Its two full lobe paths and same-pixel temporal blend informed this
integration; EvoEngine uses short probes and motion-aware temporal filtering.

## Validation and limitations

Use matched raw PT references, static and moving cameras, resize and NRD toggle tests. Compare equal samples and equal
camera GPU time separately; guide traces, reuse and filtering add work beyond beauty sampling. Check guide coverage,
disocclusions, moving skinned geometry, reflected detail and material boundaries rather than only endpoint RMS.

NRD cannot recover missing signals or guarantee unbiased convergence. Accumulated-input spatial filtering can preserve
residual blur; temporal AA adds its own history and response delay. See the [historical validation notes](ray-camera-denoising-validation.md)
for previously measured limitations and the [capture runbook](rendering-validation.md) for reproducible checks.
