# August 11 Sorghum A/B/C Vulkan handoff

This branch runs the measured-endpoint 6x10 A/A/B/B/C/C field on current EvoEngine `dev` without CUDA. The field has
60 plants, 20 per genotype, and opens from `Resources/DigitalAgricultureProject/test_lsystem_sorghum.eveproj`.

## Build and run

Clone the integration branch with its project data, then fetch the Git LFS assets:

```powershell
git lfs install
git clone --branch codex/sorghum-vulkan-dev-integration --recurse-submodules `
  https://github.com/edisonlee0212/EvoEngine.git Sorghum-Vulkan
cd Sorghum-Vulkan
git -C Resources/DigitalAgricultureProject lfs pull
```

From the repository root:

```powershell
cmake --preset vs2026-x64 -B out/build/vs2026-x64-nocuda `
  -DCMAKE_INSTALL_PREFIX="$PWD/out/install/vs2026-x64-nocuda" `
  -DBUILD_TESTING=OFF `
  -DEvoEngine_App-DigitalAgricultureApp=ON `
  -DEvoEngine_App-EvoEngineEditor=OFF `
  -DEvoEngine_App-EvoEngineLauncher=OFF `
  -DPythonBinding-PyEvoEngine=OFF `
  -DEVOENGINE_ENABLE_CudaModule_SERVICE=OFF `
  -DEVOENGINE_ENABLE_DatasetGeneration_PACKAGE=OFF `
  -DEVOENGINE_ENABLE_Gpr_PACKAGE=OFF `
  -DEVOENGINE_ENABLE_LogGrading_PACKAGE=OFF `
  -DEVOENGINE_ENABLE_LogScanning_PACKAGE=OFF `
  -DEVOENGINE_ENABLE_MeshRepair_PACKAGE=OFF `
  -DEVOENGINE_ENABLE_TextureBaking_PACKAGE=OFF `
  -DEVOENGINE_ENABLE_Universe_PACKAGE=OFF
cmake --build out/build/vs2026-x64-nocuda --config RelWithDebInfo --target DigitalAgricultureApp
cmake --install out/build/vs2026-x64-nocuda --config RelWithDebInfo
out/install/vs2026-x64-nocuda/bin/DigitalAgricultureApp.exe --editor
```

The app and project both request the `DigitalAgriculture` and `LSystem` runtime packages. No package selection or
manual scene navigation is required. Use `--editor` to change descriptors and `--player` for the faster, read-only
field review path. Both modes load only the start scene and its referenced assets; this includes the three genotype
descriptors without scanning the rest of the research archive.
Use `RelWithDebInfo` for field review: the verified no-CUDA Player reached its first complete field frame in about
one minute on the integration machine, while the unoptimized Debug build took several minutes.
The Vulkan device setup also supports GPUs that expose only one graphics/compute/present queue by safely sharing that
queue between engine roles.

After an editor install exists, colleagues can double-click `Open-Sorghum-Editor.cmd` in the repository root. The
launcher finds the standard Sorghum install layouts. The scene opens with the **Sorghum Genotype Lab** window
visible. Use its A/B/C descriptor buttons,
leave **Live preview** enabled, and choose either representative-only or whole-genotype updates while dragging.
Dragging uses a coarse mesh; releasing the control queues all 20 plants for that genotype at full quality, two per
frame. Each current mesh remains visible until its replacement is GPU-ready, so the editor stays responsive and all
60 plants remain visible throughout the update.

Use **Rasterization (interactive)** while tuning. **Vulkan Ray Tracing** and **Vulkan Ray Query** consume the exact
same generated meshes and can be selected from the lab for quality review. Unsupported modes fall back through the
engine capability policy. CUDA remains an optional scientific/legacy layer and is not required for descriptor edits
or Sorghum geometry.

The pinned data commit already removes stale `.evefoldermeta` files, so a normal first asset scan does not dirty the
data submodule merely by pruning old metadata.

## The three editable genotype assets

- `Assets/GeneratedAssets/Experiments/Sorghum2026_6x10_2026-08-11/Descriptors/FinalSnapshot/GenotypeA.sorghumls`
- `Assets/GeneratedAssets/Experiments/Sorghum2026_6x10_2026-08-11/Descriptors/FinalSnapshot/GenotypeB.sorghumls`
- `Assets/GeneratedAssets/Experiments/Sorghum2026_6x10_2026-08-11/Descriptors/FinalSnapshot/GenotypeC.sorghumls`

They live inside the `Resources/DigitalAgricultureProject` submodule. The field scene references them directly.
Use the descriptor inspector's Live Preview while editing and save the descriptor asset when the result is accepted.
Dragging previews one representative plant by default for smooth feedback; releasing the control rebuilds all 20
plants in that genotype progressively at full quality. Enable `Update whole genotype while dragging` only when
seeing every plant change during the drag is worth the additional latency of the roughly 69-million-triangle final
field.

## Parameters that control measured endpoint agreement

Change these in this order:

1. `total_phytomer_count`, `internode_length`, and `internode_thickness` control main-culm leaf count, height, and
   diameter. The rank curves matter more than a single overall scale.
2. `leaf_blade_length`, `leaf_blade_max_width`, `leaf_sheath_length`, and `leaf_insertion_angle` match the measured
   leaf-by-rank distributions.
3. `tiller_count`, origin ranks, `tiller_insertion_angle`, `tiller_height_ratio`, and `tiller_leaf_count_ratio` match
   primary-tiller count and placement without changing the main culm.
4. `main_culm_lean_angle`, `leaf_gravity_droop_compliance`, `leaf_bending`, `leaf_curling`, waviness, and axial twist
   control posture. Do not use posture parameters to compensate for incorrect measured organ lengths.
5. For reproductive Genotype C, use `enable_panicle`, peduncle and rachis length, branch length/angle/count, and
   spikelet size/count. A and B remain non-emerged at this endpoint.
6. Material and atlas controls change appearance only. They must not be used to fix morphology measurements.

The endpoint descriptors use `finalize_snapshot_morphology: true`. This intentionally guarantees that the last
snapshot expresses the sampled final organ dimensions. The intermediate GDD path is not calibrated and must not be
interpreted as observed phenology.

## Renderer behavior

Sorghum culms, leaves, and panicles are CPU-generated once and published through the same `PlantRenderTarget` mesh
channels used by current Vulkan dev. Rasterization, Vulkan ray query, and Vulkan ray tracing therefore consume the
same vertices, indices, transforms, and glTF material state. CUDA remains optional for the separate scientific
illumination services; it is not required to edit descriptors, regenerate geometry, load the field, or render it.

Older project materials are read through a narrow compatibility layer. Base color, normal, AO, and scalar PBR values
are retained. Standalone legacy metallic and roughness images are deliberately not misbound to glTF's packed
metallic-roughness slot; exact image-map fidelity needs an offline G/B channel packer, while Sorghum geometry and
endpoint measurements are unaffected.

## Fixed experiment contract

- Layout: six rows by ten plants in A, A, B, B, C, C order.
- Source mapping: A = range 75, B = range 74, C = range 73.
- Measured endpoint means: heights 1.402 m, 2.190 m, and 2.220 m; main-culm leaf counts 13.2, 13.2, and 14.2;
  primary-tiller counts 2.8, 2.2, and 1.8 for A, B, and C.
- Genotype C has emerged panicles; A and B do not.
- Field placement, per-plant seeds, normalized measurements, provenance, and validation reports are stored under
  `Data/Experiments/Sorghum2026_6x10_2026-08-11` in the data submodule.
