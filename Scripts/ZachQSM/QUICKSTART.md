# Zach QSM tree dataset: start here

The Windows delivery ZIP contains the headless EcoSysLab application, a private portable Python runtime, its libraries, and the 12-tree trial. You do not need AI, an EvoEngine checkout, Visual Studio, Python installation, or an internet connection to generate trees from this ZIP. The source branch is a separate option for developers who want to rebuild the application.

## 1. Extract and inspect the supplied trees

Use Windows 10/11, 64 bit. Extract the whole ZIP into a writable folder with a short path, such as `C:\ZachQSM`. Do not run commands from inside the ZIP. Allow about 8 GB of free disk space for the extracted package and a second 12-tree run; larger trees need more space and memory. The trial was checked on Windows x64; other operating systems have not been tested.

Open `trial_12/tree_gallery.png`, `skeleton_gallery.png` and `measurements.png`. The same measurements are available as CSV, and the graphs as PDF. Each `tree_001` through `tree_012` folder contains wood in both OBJ and PLY, native skeleton and branch graphs, measurements, optional leaf planes, and a growth history. Import mesh coordinates as metres, with Y up. `forest.glb` is a combined wood scene; leaves are separate OBJ files.

The supplied trees use seeds 1001–1012, five model years, and 30-day growth steps. Their skeleton heights are about 1.98–2.50 m. Your original `Oak.shoot` is unchanged. The foliage and pruning files come from the matching bundled EcoSysLab oak. These are model settings, not an established calibration to a measured oak species or population.

## 2. Reproduce the trial without AI

Double-click `RUN_TRIAL.cmd`. It generates a new set in `outputs\trial_12`. Keep the command window open until it reports completion. Growth finishes first, then each tree is meshed, checked and drawn. Meshing can take several minutes for the full cohort; larger ages and finer surfaces increase the work substantially.

For explicit controls, open PowerShell in the extracted folder and run:

```powershell
.\GENERATE.cmd --output outputs\trial_12 --count 12 --seed 1001 --years 5 --step-days 30 --leaves
```

Use a new output folder each time. Existing results are protected from overwriting. To repeat again, use `outputs\trial_12_repeat`. Seeds are consecutive: a count of 12 starting at 1001 uses 1001 through 1012. A model year is 365 days; it is not a promise of measured biological age.

Check the supplied files and compare the reproduced native exports:

```powershell
.\VERIFY.cmd trial_12
.\VERIFY.cmd trial_12 --compare outputs\trial_12
```

The comparison checks skeletons, node and flow graphs, growth histories and leaf files. It does not require mesh file ordering or PNG compression to match byte for byte. Keep the same app, libraries, descriptors, seed, age and time step for comparisons. Cross-platform byte identity is not promised.

## 3. Make your own runs

```powershell
# One small test before a large batch
.\GENERATE.cmd --output outputs\small_test --count 1 --seed 2001 --years 2 --leaves

# Twelve new seeds at the trial age
.\GENERATE.cmd --output outputs\new_seeds --count 12 --seed 2001 --years 5 --leaves

# Finer cylinder surfaces; wood only when --leaves is omitted
.\GENERATE.cmd --output outputs\fine_wood --count 1 --seed 1001 --years 5 --radial-segments 32
```

`--radial-segments` changes surface smoothness, not bark anatomy or the growth model. `--help` lists all controls. If native growth finished but meshing was interrupted, resume with `--output outputs\your_run --postprocess-only`; keep the same `--radial-segments` value used originally.

## 4. Change parameters without AI

The original descriptors are in `app\ZachQSM`. Keep them unchanged. Create a separate copy with one scalar changed:

```powershell
.\MAKE_VARIANT.cmd --output parameters\slower_oak --set shoot.growth_rate=0.16
.\GENERATE.cmd --output outputs\slower_oak --count 1 --seed 1001 --years 5 --leaves --shoot parameters\slower_oak\Oak.shoot --foliage parameters\slower_oak\Oak.foliage --pruning parameters\slower_oak\Oak.pruning
```

The helper creates all three descriptor files in a new folder and preserves the original set. It accepts existing numeric and true/false fields. It rejects unknown fields and does not edit the procedural angle graphs. Use the exact field names in `PARAMETERS.md` and the descriptor files. You can also edit copies with Notepad; retain the YAML indentation and file extension.

For example, make a separate pruning experiment with `--set pruning.low_branch_pruning=0.2`, or change foliage density with `--set foliage.leaf_count=1`. These are exploratory settings, not recommended species calibrations. Hold the seed, age and time step fixed and change one field at a time. Compare dimensions, branch counts, volumes and the actual images before combining changes.

The angle fields are procedural graphs rather than single numbers. For those graphs, use the EcoSysLab editor or carefully inspect the documented graph structure in source. `straight_trunk` keeps the main axis straight over a distance; it does not remove lower branches. Lower-branch pruning, light-driven pruning, growth allocation and competition affect forest-like crowns. Placing separate trees together after growth does not recreate a crowded growth history.

## 5. Place trees in a scene

Copy `trial_12\scene.json`, change each tree's `position_m` (X,Y,Z), `mesh` path and `species` label, then run:

```powershell
.\COMPOSE_SCENE.cmd trial_12\scene.json --output outputs\forest.glb
```

Mesh paths are relative to the JSON file. A species label describes a tree; it does not change its growth. To use another descriptor, first generate that tree using the corresponding shoot, foliage and pruning files, then place its mesh in the scene. This tool assembles independently grown trees and does not simulate shared light competition.

## 6. Understand the geometry and model sources

`skeleton.csv`, `nodes.yaml` and `flows.yaml` preserve the native graph. A flow is a native branch segment between forks; its count is not a botanical branch census. `branches.csv` contains length, native order and attachment angle. `cylinders.csv` uses constant-radius cylinders from the native node diameters. The wood surface joins those cylinders with separately recorded connectors and endpoint spheres. Internal cavities are filled and recorded. The resulting exterior is closed and connected, but contact between nearby branches can merge surfaces. Use the native graph for branch identity.

Cylinder volume sums and closed mesh volumes are reported separately: overlaps, connectors and rounded junctions make them different. Closed geometry does not by itself establish biological realism or validate QSM accuracy. The trial contains no simulated LiDAR scans or reconstructed QSMs. Simple leaf planes are optional, separately labelled by source node, and are not measured leaf anatomy.

`PARAMETERS.md` explains the supplied fields and their effects. `README.md` gives source-build instructions and more detail. Model background: [Rhizomorph (2023)](https://doi.org/10.1145/3592145) and its [implementation](https://github.com/edisonlee0212/rhizomorph); [Synthetic Silviculture (2019)](https://doi.org/10.1145/3306346.3323039) is related background. This app runs the current EcoSysLab code, not a claimed exact reproduction of the 2019 paper. Existing software and third-party notices are included in the ZIP.

## 7. Optional help from AI

AI is optional; the commands above are sufficient. Give an assistant `START_HERE.html` or this guide, `PARAMETERS.md`, the descriptor copies, and the exact command/error log. For code-level questions, also give it the custom GitHub branch. Ask it to verify field names in the actual files rather than inventing controls.

Example prompt: "Using this guide and the actual parameter files, show me the command to copy the oak settings and change only growth_rate from 0.2 to 0.16. Keep seed 1001, five years and a 30-day step. Save into a new folder and leave the original files unchanged. Explain what the model parameter affects and what it does not establish biologically."

Example analysis prompt: "Compare summary.csv and the tree images from my baseline and modified run. Separate measured differences from biological interpretations. Do not change the generator or silently tune other parameters."

If a run fails, share the command, its terminal error, and the output folder's `native.log`. Keep the error visible; do not edit validation checks merely to make a failed result pass.
