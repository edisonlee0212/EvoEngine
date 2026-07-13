# M13 Dynamic Geometry Scope

## Supported

- TLAS updates classify identical input as no-op, reuse the existing TLAS for compatible transform, visibility, and BLAS
  content changes, and upload only coalesced dirty instance ranges. BLAS-content-only updates record no instance copy.
- Capture telemetry reports submitted TLAS source bytes, uploaded bytes, copy ranges, builds, updates, no-ops, full uploads,
  and zero-instance-upload updates for the measured frame window. Discarded submissions do not contribute upload work.
- glTF mesh-default morph weights and POSITION, NORMAL, and TANGENT target deltas are imported. Target deltas follow
  MikkTSpace vertex splits and are serialized with Mesh and SkinnedMesh assets.
- Setting morph targets requires a matching neutral vertex basis and bakes nonzero default weights into the stored default
  geometry; serialized morph state with a missing or mismatched basis is rejected. Explicit CPU geometry mutation through
  merge or normal/tangent regeneration invalidates attached morph targets before changing vertex topology.
- MeshRenderer and SkinnedMeshRenderer expose programmatic morph-weight overrides. Non-default static-mesh weights use a
  renderer-owned updateable BLAS and matching hit-shader payload range. Steady changes refit the same BLAS and update the
  same payload allocation. Skinned meshes apply morph targets before skeletal skinning and use the existing persistent
  deformation path.
- Dynamic ray bounds are united with the default raster bounds so ray-only deformation is not culled and the unchanged
  raster surface remains covered.

## Deferred

- glTF animated morph-weight channels and node-level weight overrides. These require animation interpolation, previous
  weight history, and motion-vector ownership that the current Animation asset does not provide.
- Raster, shadow-map, and emissive-light-sampling deformation for per-renderer weight overrides. The default glTF shape is
  baked consistently for raster and ray rendering; non-default programmatic overrides are ray-only in M13.
- GPU morph evaluation and GPU-generated TLAS instances. Current scene transforms and visibility are authored on the CPU,
  so a GPU instance path would duplicate the source data and add a compute-to-AS-build dependency. M13's dirty-range path
  measures the real CPU-authored workload first; a GPU path remains gated on a future device-authored large-instance
  producer and a demonstrated end-to-end win.
- Morph targets for particle instancing, procedural strands, Gaussian splats, external DDGI geometry, and topology-changing
  deformation. These paths retain their existing explicit geometry ownership and update behavior.

## Validation Contract

- The fresh static M6 records remain the zero-build/zero-update gate.
- The final current-source `motion-as` acceptance set is RTX, RayQuery, and forced query-only. Each must contain zero AS
  builds, 30 BLAS updates, 15 TLAS updates, exactly one BLAS-content-only update with no instance upload, and the approved
  GPU/wall ceilings. The user ratified conservative host-inclusive smoke ceilings of 0.48 seconds for RTX and 0.43 seconds
  for both RayQuery modes after all GPU, AS, image, and fatal-log gates passed. RayQuery and query-only HDR output must be
  bit-identical.
- M13 uses ten renderer launches in total: six superseded captures from before the final morph-correctness and mesh-identity
  repairs, three authorized final-source captures, and one post-commit 2560x1440 2048-SPP Bistro delivery. No reference run
  or retry is authorized.
- Behavioral tests cover dirty-range coalescing, growth/shrink and shifted suffixes, telemetry subtraction, glTF
  POSITION/NORMAL/TANGENT import, default-shape baking and serialization, MikkTSpace remapping/invalidation, and
  morph-before-skin ordering. Source guards cover submission retry, persistent payload/BLAS ownership, stable handles, and
  conservative-bound wiring.
