# DDGI sanitization record

M10 reviewed the implementation left by the DDGI overhaul milestones M1-M9. This record groups temporary paths and
defensive code by the invariant that owns them. `Remove` means the supported path no longer contains the mechanism;
`replace` or `narrow` means the invariant is preserved with a smaller mechanism; and `retain` means the path remains an
intentional part of the renderer rather than unowned milestone scaffolding.

## Ownership, ABI, and lifecycle

| Path or class | Disposition | Owning invariant and evidence |
|---|---|---|
| Nine-field single-volume `RenderInfoBlock` mirror | Remove | The fixed array of eight `DdgiVolumeInfoBlock` records is the only GPU source. Probe visualization reads slot zero. ABI tests freeze the 112-byte DDGI header and 128-byte volume-array offset; DDGI ends at byte 1408, before the appended local-reflection-probe metadata, and the full block is 5520 bytes. |
| `runtime_state` and `ddgi_runtime` migration aliases | Remove | Per-volume state is accessed through its owning object. A removed-symbol grep and the focused DDGI suite cover the qualified paths. |
| Scene/settings diagnostic-source overload and cached candidate metadata | Remove | Production diagnostics use the already-resolved per-volume source. Deterministic seeding retains only `selected_volume_index`, which affects the ray rotation and emissive streams. |
| Debug-volume selector that always selected the primary volume | Remove | All editor diagnostics explicitly use the deterministic primary volume. There is no second hidden selection policy. |
| Assignment-only runtime ownership, weak-volume, and update-reason fields | Remove | The runtime map owns state by stable entity ID and the live update-reason value remains the single diagnostic source. |
| Fixed eight-volume GPU ABI and valid unused descriptor slots | Retain | Vulkan descriptors must remain valid even when fewer than eight volumes contribute. Limit, layout, inactive-slot, and volume-removal tests cover the contract. |
| Invalid authoring-edit rollback | Retain | Volume nine and aggregate probe 8193 are rejected without silently replacing a valid volume. This is authoring safety, not a transient check. |
| Legacy environment-backed implicit volume | Remove | DDGI authoring resolves from the assigned `EnvironmentalLighting` asset. Scenes without that asset use the default no-asset lighting view rather than an implicit scene-local volume. |

## Scheduling, convergence, and update paths

| Path or class | Disposition | Owning invariant and evidence |
|---|---|---|
| Round-robin, fixed-budget, and adaptive scheduler candidates plus `EVOENGINE_DDGI_PROBE_SCHEDULER` | Remove | The frozen M6 comparisons did not satisfy the M2 quality/convergence gate. The production schedule is one deterministic full `[0, probe_count)` sweep. |
| Cursor, pending-count, and partial-window telemetry | Remove | A full-only schedule has no cursor or remainder. Sweep completion remains because variability observations are valid only after a complete update. |
| Full-probe update policy | Retain | It is the measured no-adopt result of M6, not a fallback. Periodic refresh and contributor-triggered invalidation prevent convergence from freezing on a transient sample. |
| Serial atlas update | Retain | It is the device-capability and pipeline-initialization fallback for the two cooperative workgroup variants. Dispatch-limit and shader-path tests keep the fallback deterministic. |
| Parallel shared update as the default | Retain | The frozen M4 comparison selected it. Startup reports requested, selected, and executed variants so fallback remains observable. |

## Lighting and shader policy

| Path or class | Disposition | Owning invariant and evidence |
|---|---|---|
| Late full-color GTAO combine with metallic interpolation | Replace | GTAO now produces an R16F visibility texture before deferred lighting. Deferred opaque lighting multiplies material occlusion and GTAO into the composed indirect diffuse/DDGI lobe only; direct light, emission, and specular IBL are untouched. Transparent lighting does not sample screen-space GTAO because it comes from the opaque depth and GBuffer. The deleted combine shader and lobe-composition tests prevent the workaround from returning. |
| Full-resolution RGBA32F AO source and scratch copies | Replace | Two transient R16F images carry only AO. Targeted compute-write to sampled-read barriers replace the AO `EverythingBarrier` calls. |
| DDGI replacing only diffuse IBL | Retain | `mix(diffuse_ibl, ddgi_diffuse, coverage * confidence)` preserves valid black probes, uncovered diffuse fallback, and independent split-sum specular IBL. Numerical and shader tests cover exact-metal behavior. |
| Lambert-only recursive probe-hit approximation | Retain | DDGI transports diffuse irradiance and does not claim camera-path BSDF parity. This bounded approximation is documented and keeps recursive DDGI from inventing specular reflection. |
| Shared camera/DDGI emissive sampling and PDF helpers | Retain | One distribution and helper implementation prevents estimator drift. DDGI deliberately uses one Lambertian NEE sample without camera BSDF MIS. |
| Unsupported emitter/material categories | Retain | Skinned/deformed, particle-instanced, external, blended/transmissive, strand, and Gaussian emitters remain excluded until sampled geometry and PDF exactly match the TLAS. Rigid opaque and alpha-masked categories have focused inventory and numerical coverage. |
| Finite, bounds, descriptor, and capacity checks at ABI/resource/numerical boundaries | Retain | These checks protect untrusted scene data, Vulkan descriptor validity, and NaN propagation. Internal duplicated or impossible-state checks were removed only where an owner already enforces the invariant. |

## Resource lifetime and synchronization

| Path or class | Disposition | Owning invariant and evidence |
|---|---|---|
| All-frame drains for scene changes, volume removal, persistent atlas/metadata rebuilds, probe-state rebuilds, and diagnostic-buffer rebuilds | Remove | The recycled frame slot is fenced before preparation. Every graph-bound buffer, image, view, and descriptor is retained by that slot's transient-resource store until its next recycle. Readback copy destinations are retained explicitly. |
| All-frame debug readback drain | Narrow | Metadata and selected-ray copies carry the exact frame-slot submission ticket. Refresh waits only that ticket, skips work that has not been submitted, rejects discarded work, and downloads each generation once. |
| Variability generation recorded before submission | Replace | A variability readback becomes consumable only when its exact copy ticket reaches `Submitted`; `Pending` is deferred and `Discarded` schedules a later observation instead of consuming stale bytes. |
| Grow-and-drain fallback probe-state buffer | Replace | One 131072-byte inactive buffer is allocated from the hard 8192-resident-probe cap. It never grows, so inactive fixed-slot descriptors cannot outlive a replaced buffer. |
| Per-frame transient retention stores | Retain | They are the precise ownership mechanism for GPU work in flight and are cleared only after the corresponding frame-slot fence. |

## Validation and historical evidence

| Path or class | Disposition | Owning invariant and evidence |
|---|---|---|
| Frozen DDGI baseline release, manifest history, and replay evidence | Remove | The PR keeps only stable DDGI ray-camera reference images for manual/ad-hoc comparison. There is no checked-in DDGI baseline policy bundle. |
| DDGI baseline validator and publisher runner | Remove | The frozen baseline gate is no longer maintained. Focused runtime validation scripts own current DDGI, environment-lighting, and reflection-probe behavior checks. |
| Reference images | Retain | Stable images moved to `EvoEngine_Tests/Rendering/DDGI/References/` so they are clearly reference material rather than a committed pass/fail release. |
| Capture-only acceptance bypass | Remove | Diagnostic suites remain explicit and cannot produce milestone acceptance evidence by bypassing their own checks. |
| Brittle implementation-spelling assertions | Narrow | Behavioral math, ABI/layout, dispatch, resource-lifetime, shader compilation, and numerical tests own the contracts. Source checks remain only for shader or Vulkan wiring that is not exposed through a test API. |

## Portability and explicit deferrals

| Path or class | Disposition | Owning invariant and evidence |
|---|---|---|
| Vulkan RT-pipeline-only DDGI | Retain | RayQuery DDGI is not implemented. Capability checks and documentation prevent a silent lower-fidelity path. |
| No strand or participating-medium emissive NEE | Retain | Their geometry/transport estimators are not equivalent to the supported triangle/Lambert model. The exclusion is explicit rather than hidden behind a fallback. |
| Serial/device capability fallback | Retain | Portability and valid output take precedence over selecting an unsupported cooperative path. The actual executed path remains reported. |

## Retained references

Stable DDGI ray-camera references live under `EvoEngine_Tests/Rendering/DDGI/References/`. They are intentionally not an
authoritative replay bundle. Use `Scripts\compare_reference_render.py` for ad-hoc image comparisons when a capture needs a
numeric delta against one of those images.

## M10 closeout evidence

The cleanup reduced the modified production paths by 218 net lines and the current validation scripts by 721 net
lines. The historical replay archive is additive and is excluded from those counts. The final focused suites passed
156 DDGI/render-graph/raster-material tests and three Vulkan shader-compilation tests; the full suite passed 533 tests
with the existing Sponza import test skipped.

The deterministic 1920x1080 GTAO pair under `out/ddgi-validation-m10-gtao` isolates the intended lighting change.
The exact-metal probe's 6,921-pixel circular ROI is byte-identical with GTAO disabled and enabled. The dielectric and
rough-dielectric ROIs change by 226 and 263 pixels respectively, both with negative mean luminance deltas. Across the
frame, GTAO changes 90,152 pixels with normalized RMS error 0.002117005. This proves screen-space AO changes diffuse
ambient lighting without attenuating exact-metal specular lighting.

The canonical four-fixture holdout passed every hard limit and rejected the deliberate aggregate-timing perturbation:

| Fixture | Quality error | Repeatability error | Convergence frames | Resident bytes | Aggregate median / p95 / relative MAD (ms) |
|---|---:|---:|---:|---:|---:|
| Cornell | 0.021075721 | 0 | 40 | 4,714,008 | 0.466256 / 0.473642 / 0.008802 |
| Sponza | 0.070174638 | 0 | 35 | 4,194,320 | 0.953584 / 0.978397 / 0.007164 |
| Scrolling | 0.062487363 | 0 | 21 | 2,490,384 | 0.354128 / 0.361101 / 0.010651 |
| Emissive alpha cutout | 0.076611690 | 0 | 35 | 2,490,384 | 0.355456 / 0.362514 / 0.013280 |

The installed RelWithDebInfo Editor then completed the automated 1920x1080 Bistro interval for 30 seconds with 3,388
active and updated probes, 216,832 recorded ray samples, finite nonblack output, Vulkan and synchronization validation,
and a clean shutdown.

## M13 post-M12 disposition

M13 repeated the audit after the environment-control split and spatial-reflection-probe integration. The production
and validation paths changed by 301 added and 690 removed lines before this record, for a net reduction of 389 lines.
The accepted DDGI algorithm, eight-volume/8192-probe limits, asset-owned volume resolution, fixed descriptor arrays, and
serial/parallel device fallback remain unchanged.

| Path or class | Disposition | Owning invariant and evidence |
|---|---|---|
| Identity probe-update index buffers, full-range scheduler, update windows, sweep counters, and associated telemetry | Remove | M6 permanently selected a full deterministic update. Every shader now derives the logical probe directly from dispatch coordinates. This removes one two-frame index-buffer ring, its CPU vector/upload/bind path, and four storage-buffer descriptor bindings without changing dispatch order. The canonical `probe_update_index_bytes` report key is emitted as a literal zero so current evidence stays machine-readable without retaining a runtime statistic. |
| Duplicate volume validation, sorting, owner indices, and stable-ID candidate rejoin | Remove | Candidate collection already validates and produces the final priority/density/stable-ID order. Runtime info and candidate vectors now remain index-aligned, eliminating a second validation/sort and the per-volume lookup without weakening atomic volume-set rejection. |
| Public test-only scheduler, debug-coordinate, newly-exposed-count, and update-stat helpers | Remove | Production owns grid indexing, atlas layout, full-reset policy, periodic refresh, and performance reporting. Focused behavioral/layout tests call those owners directly instead of preserving wrappers solely for tests or editor spelling checks. |
| Atlas-update push constant and shader metadata scan | Narrow | The push constant is packed from 192 to 160 bytes. Removed update-window fields and duplicated atlas rows/state parameters have no consumer. Serial and parallel variants share one metadata calculation and retain identical irradiance, visibility, relocation, classification, and scroll inputs. |
| DDGI environment signature | Narrow | Diffuse intensity is the first ownership gate. When it is zero, background mode, cubemap content, and gamma cannot affect DDGI and no cubemap lookup occurs. Solid-color mode hashes only its diffuse inputs. Specular intensity remains excluded, preserving M11 ownership. |
| Reflection-bake DDGI readiness gate | Replace | The old aggregate-performance check could see zero active probes before history was ready and start a bake early. Bake readiness now follows the prepared ordered runtime set and requires history plus lighting descriptors, and convergence only when variability gating is enabled. |
| Pass-local guards after graph ownership transfer | Narrow | Pipeline, resource, image-view, descriptor, device-limit, and dispatch preflight completes before graph barriers are acquired. Helpers no longer repeat checks already enforced by the recorder. Untrusted resource and Vulkan boundary checks remain. |
| DDGI atlas sampler fallbacks in active ray/visualization passes | Remove | The renderer owns and supplies the initialized DDGI atlas sampler before these passes can record. The lighting descriptor fallback remains because unused fixed-array slots must always be valid. |
| Specialty validation capture readback/save lambdas | Replace | Reflection, environment, emissive, and multi-volume validation now share one checked readback-and-encode path. Their 17 PNGs require 17 waits and 17 readbacks instead of 51 waits and 34 readbacks; timing-specific emissive waits remain. |
| Unfinished atlas, sampling-weight, and update-age debug controls | Remove | The atlas and sampling toggles had no visualization consumer, while update age was reset to zero by every full sweep and never aged. The inspector retains the working atlas coordinate readout, probe metadata, state visualization, and selected-ray diagnostics. |
| Implementation-spelling validation meta-tests | Remove | The deleted blocks tested runner source text rather than renderer behavior. Shader compilation, ABI/layout tests, installed-editor specialty fixtures, and focused source-contract tests own those contracts. |
| Finite, capacity, layout, descriptor, serialized-data, and legacy-loading checks | Retain | These guard scene input, numerical stability, Vulkan requirements, authored rollback, and migration boundaries. M13 removed only checks duplicated behind an existing owner. |

The index-ring removal saves `probe_count * 4 * 2` persistent bytes, or 65,536 bytes at the aggregate 8,192-probe
limit. It also removes the identity upload and repeated index fetches from ray generation, atlas update, relocation, and
classification. Final deterministic-image, timing, resident-memory, build, test, install, and Bistro evidence is
recorded in `tasks/done.md` with the M13 closeout.

## Post-M13 follow-up audit

The committed M13 result was audited again from its consumers rather than from the prior disposition list. The
follow-up removes only state and work that has no rendering, serialization, synchronization, or validation owner.

| Path or class | Disposition | Owning invariant and evidence |
|---|---|---|
| Global volume-bounds toggle and per-volume visualization fields | Remove | None of the five fields reached a renderer, pass, shader, or component inspector. Their YAML, demo assignments, clamps, and smoke predicates only preserved an authoring surface with no behavior. Old keys are ignored on load. The working global probe-position, illumination, state, selected-probe, and ray controls remain. |
| Per-volume `RenderInfo` preservation and current-trigger parameter chain | Remove | The outer frame preparation always rebuilds the final ordered DDGI header and volume array after each per-volume return. Scene changes are already accumulated in the global latch and copied to every runtime before preparation, so the second parameter and OR performed no work. Atomic invalid-set preservation remains at the outer ownership boundary. |
| Runtime policy, variability, debug-readback, layout, and validation-report mirrors | Narrow | The unused lighting-policy bit and write-only variability-valid bit are gone. Debug counts/readiness derive from the downloaded vectors, the fixed two-float variability readback uses `sizeof(glm::vec2)` directly, and the obsolete index-memory report value is a schema-preserving literal zero. |
| Probe ray-output clear and scroll-to-clear dependency | Remove | Full-probe tracing launches the exact layout product and ray generation writes every output slot, including inactive rays. The graph now declares ray output and selected diagnostics as writes instead of clearing and then read-writing them. Atlas preparation exists only for persistent atlas/state clearing; incremental scrolling no longer requires that optional pass, while shared write hazards order scrolling after it whenever a full clear is present. |
| Handwritten DDGI render-graph descriptors and demo scheduler check | Replace | Graph tests instantiate the production atlas, ray, update, relocation, classification, and variability descriptors. The trace-only graph begins with a writer-only ray output and no atlas-clear pass, so tests cannot silently preserve the deleted clear/read-write contract. The demo idle-readback gate uses steady-state reasons and finite active metadata instead of calling the removed partial-scheduler pending-count API. |
| Probe and ray visualization interfaces | Narrow | Visibility coloring already came from probe metadata, so the unused visibility-atlas descriptor, view, graph resource, and fragment declaration are removed. Unused vertex inputs/varyings and push lanes are removed; the ray-visualization push ABI is 16 bytes instead of 32, and non-atlas modes no longer perform an irradiance texture sample whose result they overwrite. |
| Variability reduction and shader access declarations | Narrow | Later reduction stages do not bind the unused probe-state descriptor or expose the unused atlas-parameter push suffix. Read-only and write-only shader resources now state their actual access. The shared descriptor layout, ping-pong images, dispatch bounds, and explicit reduction/readback dependencies remain. |
| Active-volume descriptor fallback and gather-timing failure cleanup | Narrow | Active DDGI volumes normally bind their owned sampler, atlases, and state. The fallback state and sampler remain for a valid no-trace/no-TLAS frame and resource-loss boundary, while inactive fixed-array slots always receive valid fallbacks. Gather timing validates its output view before acquiring graph barriers and has one normal release path. |

Finite and bounds checks, invalid-edit rollback, eight-volume/8192-probe capacity, fixed descriptor arrays, serialized-data
migration, exact readback tickets, Vulkan synchronization, deterministic selection, and serial device fallback are retained.
