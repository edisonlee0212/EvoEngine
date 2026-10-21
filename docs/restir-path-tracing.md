# ReSTIR path tracing

[Back to rendering overview](rendering.md)

## Integrator modes

**Ray Integrator** selects Path Tracing or ReSTIR PT Candidate Only, Spatial Only, Temporal Only, or Enhanced.
ReSTIR runs through inline RayQuery. Candidate Only generates a path reservoir; Spatial Only adds same-frame reuse;
Temporal Only reprojects previous-frame reservoirs; Enhanced combines temporal and spatial reuse. These reuse modes
remain experimental and have not established a broad equal-GPU-time quality advantage over ordinary PT.

- **Spatial neighbors:** 1–16 reciprocal pairs, default 3, using approximately 16-pixel Gaussian offsets. More pairs
  increase replay work and memory. Material, normal and distance checks reject incompatible neighbors.
- **Temporal history cap:** 1–32, default 20. **Adaptive duplication cap**, enabled for new cameras, lowers confidence
  toward 1 where nearby pixels share a path seed. It may reduce persistent fireflies but can increase variance or
  change brightness; compare against PT. Disabling it skips the duplication pass and full-resolution duplication map.
- **Hybrid shifts:** off by default. Spatial Only reconnects eligible three-vertex diffuse/emissive paths. Enhanced
  also supports eligible longer opaque paths using footprint-selected reconnection vertices. Unsupported endpoints,
  moving reconnection geometry, delta, transmission and volume events require full replay or remain unsupported.
- **Accumulate Samples:** averages resolved radiance across frames. Disabling it preserves same-frame reuse.

Reservoir history confidence is not an independent-sample count. Repeated temporal reuse correlates successive resolved
estimates, so raw frame averaging can improve more slowly than ordinary PT even while fresh candidates are generated.
The history and adaptive caps bound reuse confidence; they do not guarantee a convergence rate. Compare a low history cap
and temporal reuse disabled against PT at both equal samples and equal measured GPU time.

Every sample per frame runs an independent candidate/reuse/resolve estimate. Temporal estimates read the previous
frame's reservoir and publish the last estimate for the next frame. All pixels continue sampling; adaptive stopping
is unsupported. Sampling and spatial pairing use the advancing renderer frame, so clearing accumulation or reservoir
history does not repeat the same random pattern. Ray debug views must be off for reuse.

Ray-camera history retirement waits for submitted GPU work before releasing resources, including when a
camera component or its owner is disabled or removed from the scene.

Temporal modes retain reservoir history across camera motion and rigid mesh transforms while restarting radiance
accumulation; other scene changes clear history. They use two reservoir/surface history slots and camera-owned scratch
buffers synchronized by the render graph. ReSTIR texture footprints grow with path distance and capped roughness-aware
spread; alpha, shadow and emissive visibility retain their existing LOD rules.

Triangle meshes, supported strands, unlit/emissive/environment/punctual lighting, reflection, transmission, absorption
and homogeneous scattering are supported. Dispersion and Gaussian splats are unavailable. Unsupported ordinary ReSTIR
scenes report an error. SSS uses [dedicated direct reuse](subsurface-scattering.md) with its own eligibility and fallback
rules. Inspect raw rendering separately from [NRD denoising](ray-camera-denoising.md).

The six optional ray outputs describe fresh candidates before reuse: primary albedo/normal/roughness, candidate ray
count/path length and candidate debug radiance. Timing currently reports zero. Temporal Only omits NRD signal work
when denoising is disabled. Scene-specific shader activation resets history. The editor prioritizes selected camera
modes and prepares other variants in the background; see [shader preparation](shader-preparation.md).

Captures accept `--preview-ray-integrator`, `--preview-restir-spatial-neighbors`, `--preview-restir-spatial-hybrid`,
`--preview-restir-temporal-history-cap`, `--preview-restir-temporal-adaptive-cap`, and `--preview-accumulate-samples`.
`--preview-ray-profile-report` records shift outcomes, timing, memory and bright-pixel diagnostics;
`--preview-restir-generated-profile` adds candidate statistics. Capture startup disables the unused scene main camera
before waiting for readiness. See [validation](rendering-validation.md) for comparison procedures.
