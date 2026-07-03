# Sorghum Leaf Material Tools

These scripts keep the Gemini sorghum leaf material workflow outside engine code.

## Run Chord

`run_chord_leaf_pbr.py` runs Ubisoft Chord on the original Gemini leaf albedo and writes model-derived maps only:

- `albedo.png`
- `normal.png`
- `roughness.png`
- `metallic.png`

It does not use heuristic fallback maps. If Chord cannot load or infer, the script exits with an error.

## Bake Atlas

`bake_sorghum_leaf_atlas.py` converts a flat leaf albedo plus mask into square PBR starter atlases for the current EvoEngine sorghum UV layout:

- leaf surfaces sample UV V 0.5..1.0
- stem/internode surfaces sample UV V 0.0..0.5

When `--pbr-dir` points at a Chord output folder, the leaf atlas region uses Chord `albedo`, `normal`, `roughness`, and `metallic` maps. Chord does not output AO, so the leaf AO region is neutral white.

## Bake LSystem Variant Atlas

`bake_lsystem_leaf_variant_atlas.py` packs nine RGBA leaf captures into one 3x3 LSystem atlas. By default it reads sources from `out\generated_assets\SorghumLeafMaterials\ImageTestLeafVariants\source`, and each tile uses the top half for the source neck + blade alpha cutout and the bottom half for deterministic generated sheath color.

The script also generates quick procedural PBR starter maps:

- RGB-dilated RGBA albedo to reduce alpha-cut mip halos
- normal map derived from a synthetic midrib/vein/fiber height field
- roughness map with semi-matte blades and rougher sheath fibers
- subtle AO map for midrib, vein, and sheath fold grounding
- height map for future displacement/parallax use

Example:

```powershell
python 'Scripts\sorghum_leaf_materials\bake_lsystem_leaf_variant_atlas.py' `
  --input-dir 'out\generated_assets\SorghumLeafMaterials\ImageTestLeafVariants\source' `
  --out-dir 'out\generated_assets\SorghumLeafMaterials\ImageTestLeafVariants'
```

## Apply Atlas To Scene

`apply_sorghum_leaf_atlas_to_scene.py` updates scene-local `Material` assets referenced by `Leaf Mesh` and `Stem Mesh` renderers. It assigns the atlas Texture2D handles from `.evefilemeta` files and neutralizes material tint by setting albedo color to white.

Bake scratch atlases under `out\generated_assets` first. Copy only the selected atlas into a project `Assets` folder, then run the scene patch with that promoted atlas path. Direct baking into `Resources\<Project>\Assets` requires `--allow-project-assets`.

Example:

```powershell
& 'C:\Program Files\Blender Foundation\Blender 5.0\5.0\python\bin\python.exe' `
  'Scripts\sorghum_leaf_materials\apply_sorghum_leaf_atlas_to_scene.py' `
  --scene 'Resources\DigitalAgricultureProject\Assets\2026-07-01_Sorghum\2026-06-04_Sorghum_LSystem.evescene' `
  --atlas-dir 'Resources\DigitalAgricultureProject\Assets\SorghumLeafMaterials\GeminiLeaf01\atlas'
```
