# ZachQSMTreeDatasetGenerator

Generate reproducible EcoSysLab trees without opening the editor. The native C++ app grows the trees and exports the exact skeleton. The Python command calls that app, builds closed wood surfaces, checks the exports, and saves figures.

## Run an installed copy

If you have the portable Windows delivery ZIP, start with its `START_HERE.html` and `RUN_TRIAL.cmd`. That package includes Python and the dependencies; no AI assistant, compiler or Python installation is required. See [the recipient walkthrough](QUICKSTART.md) for reproduction, parameter changes and optional AI help.

For a normal source build or installed copy, use Python 3.11 or newer (3.13 was tested). From the installed `bin` directory, one command creates a local Python environment on first use and runs the complete workflow:

```powershell
ZachQSM\generate.cmd --output trial_12 --count 12 --seed 1001 --years 5 --leaves
```

Python must be available as `python`; set the `ZACH_PYTHON` environment variable to a full interpreter path if needed. The first run downloads Python packages. For manual environment setup:

```powershell
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r ZachQSM/requirements.txt
.venv\Scripts\python.exe ZachQSM/generate.py --output trial_12 --count 12 --seed 1001 --years 5 --leaves
```

`requirements-windows-py313.txt` records the exact package versions checked in a fresh Windows Python 3.13 environment. Use that file in place of `requirements.txt` when recreating that environment. Every run also records its actual dependency versions.

The output directory must be new. Repeat the same command with a new output path to repeat a run. `--seed` is the first tree's seed; subsequent trees use consecutive integers. `--years` means model time, with 365 days per year. `--step-days` defaults to 30. Keep the time step, executable, dependencies and descriptors fixed when comparing seeds. Cross-platform bitwise identity is not promised.

Change `--shoot`, `--foliage`, and `--pruning` to use other EcoSysLab basic descriptors. `--leaves` adds simple leaf planes without changing the woody growth parameters. `--radial-segments 32` gives finer cylinder and junction surfaces. This changes mesh resolution, not the growth model or bark anatomy. Use `--postprocess-only` to retry mesh and figure export after a successful native run.

For skeleton-only use, call `ZachQSMTreeDatasetGenerator.exe --config run.yaml`. The YAML fields are `output`, `count`, `seed`, `years`, `step_days`, `shoot`, `foliage`, `pruning`, and `leaves`; descriptor paths must be absolute or relative to the process working directory. An optional `simulation` mapping accepts native `SimulationSettings` fields. Root growth, soil water simulation and GPU physics are disabled in this aboveground dataset workflow. The soil component supplies the required scene context; it does not constrain growth. Light and self-shading use EcoSysLab's native environment grid.

## Build this branch

On Windows, install Visual Studio 2026 with C++ tools, CMake 4.2 or newer, the Vulkan SDK, Git, and Python. From the repository root:

```powershell
git submodule update --init --recursive
cmake --preset zach-headless
python Scripts/install_apps.py --preset zach-headless --config Release --incremental --no-open --jobs 6
python -m pip install -r Scripts/ZachQSM/requirements.txt
python Scripts/ZachQSM/generate.py --output ../trial_12 --count 12 --seed 1001 --years 5 --leaves
```

The installed executable is `out/install/zach-headless/bin/ZachQSMTreeDatasetGenerator.exe`. The preset disables the editor and unrelated packages. No window, display, camera, or GPU rendering context is created during generation; the engine still has native runtime library dependencies. The whole installed `bin` directory is needed, including `Packages`, descriptors and DLLs. Do not distribute the EXE alone.

## Files and geometry

All coordinates are in metres, Y points up, and each independent tree starts at its own origin.
Reported height and crown widths are extents of the native skeleton endpoints. The wood surface extends beyond those endpoints by the local radius. Volume and surface area measurements refer to the final closed wood mesh unless explicitly labelled as cylinder sums.

| File | Meaning |
| --- | --- |
| `skeleton.csv` | Native node ID, parent ID, flow ID, order, endpoints, diameter and length. Root parent is -1. |
| `nodes.yaml`, `flows.yaml` | Native EcoSysLab graph exports. `T`, `ST`, `ET` are diameters, not radii. A flow is EcoSysLab's branch segment between forks. |
| `cylinders.csv` | One constant-radius cylinder per native node; radius = diameter / 2. These dimensions stay separate from the joined surface. |
| `junction_connectors.csv` | Separately labelled geometric connections across native lateral-branch offsets; raw node positions stay unchanged. |
| `branches.csv` | Length summed over each flow, native order, basal diameter and angle to the parent node direction. |
| `wood.obj`, `wood.ply` | Exterior of the union of native cylinders, labelled parent-child connectors, and spheres at their endpoints. Connectors and junction spheres close angled connections. The base and tips are capped. |
| `leaves.obj`, `leaf_instances.csv` | Optional two-triangle leaf planes and their source node IDs, using the basic foliage descriptor. These are render placements, not measured leaf anatomy. |
| `growth.csv` | Native node count, flow count and height after every growth step. |
| `metrics.json`, `summary.csv` | Tree dimensions, branch counts, cylinder volume sum, joined mesh volume, and mesh checks. |
| `run.yaml`, `parameters/`, `effective_settings.yaml`, `provenance.json` | Exact run inputs, copied descriptors, native defaults, executable hash and dependency versions. |
| `validation.json`, `sha256.json` | Completion/check results and hashes of delivered files. |
| `tree_gallery.png`, `skeleton_gallery.png`, `measurements.png/pdf` | Actual meshes, native skeletons at a common scale, and measurements. |

The joined surface is a documented geometric construction from the model's skeleton. Junction balls add volume and round tips; cylinder sums count overlaps. Neither quantity is interchangeable with biological wood volume. Meshes contain no voxel-derived surface. EcoSysLab deliberately offsets lateral branch starts through its native `branch_push` rule. `junction_connectors.csv` records each added connection from a parent endpoint to its child start, using the child radius; the raw skeleton is preserved. Geometric branch contacts may fuse in the union; use the native parent graph for branch identity rather than deriving topology from the surface. A closed mesh does not by itself establish botanical realism or validate a QSM method. No LiDAR simulation or QSM reconstruction is included in this trial.

The validator rejects duplicate roots/IDs, missing parents, cycles, non-finite geometry, inconsistent lengths, attachment offsets exceeding the native branch-push bound (sqrt(2) times parent radius, plus 0.1 mm tolerance), disconnected wood, open surfaces, and invalid winding. Zero-length native nodes are recorded; their endpoint sphere remains in the surface. Diameter at breast height is the diameter of the first order-zero segment crossing 1.3 m; it is absent for trees below that height. It is not a fitted circle through the final mesh.

The export keeps the exterior wood surface and fills enclosed internal cavities. Such cavities have inward-facing closed boundaries and can occur as tiny numerical remnants of the cylinder union. Each tree's metrics record their count and total filled volume. Any separate exterior wood component is rejected; detached branches are never discarded to pass the connectedness check.

## Parameters and model sources

See the [field-by-field oak parameter reference](PARAMETERS.md) for the supplied values, units and implementation details.

`Oak.shoot` is the supplied oak configuration from the October 27 correspondence. Its SHA256 is `c34baabe06b4e2858fd1d59bd231fe1752b80fcdaa9a6cc8bc5d81a6f297e356`. The foliage and pruning files are the ones referenced by the bundled EcoSysLab `Oak.tree`. Its reproduction reference points to an empty `Default.repro`, so the app uses the corresponding native default reproduction descriptor during growth. Flowers and fruits are not exported. Bark materials are not used for the cylinder surfaces. These are artist/research model settings; no empirical calibration, taxonomic identification, or fit to a measured oak population is established here.

| Parameter group | Practical effect |
| --- | --- |
| `growth_rate`, `internode_length` | Extension speed and the length scale of individual axis segments. |
| `base_internode_count`, `straight_trunk` | Initial trunk construction and the length over which trunk growth stays straight. |
| `branching_angle`, `roll_angle`, `apical_angle` graphs | Branch departure, orientation around the axis, and apical direction. Values can vary with graph inputs. |
| `gravitropism`, `phototropism`, `horizontal_tropism` | Directional growth responses to gravity, light and horizontal directions. |
| Apical control/dominance, bud flushing and extinction | Competition between tips and whether buds produce or retain branches. |
| Thickness and sagging controls | Axis thickening and bending under the model's load response. |
| Pruning descriptor | Shedding and pruning thresholds; affects retained branch structure. |
| Foliage descriptor | Leaf size, number, placement, and the branch regions allowed to carry leaves. |
| Simulation light settings | Skylight, environmental illumination and shadow attenuation. |

Read the exact field names and implementation in `BasicShootDescriptor.hpp/.cpp`, `BasicPruningDescriptor.hpp/.cpp`, `ShootModel.cpp`, and `SimulationSettings.hpp`. `effective_settings.yaml` records values actually used. Keep the seed fixed and change one group at a time to study its effect. Different seeds vary the stochastic decisions; they do not by themselves change species or create a closed-canopy growth environment.

For the longer clear trunks in the forest reference figure, `straight_trunk` keeps the main axis straight but does not remove lower branches. The pruning file's `low_branch_pruning` controls removal below a fraction of the maximum root-to-tip path distance; it is not a height in metres. `light_pruning_factor` sheds shaded tips below its light threshold. Together with shared-scene competition these are starting points for a higher crown base. Smaller departure angles can make branches more upright; `apical_control`, `apical_dominance`, bud flushing, and directional growth responses change crown balance. These are mechanisms to explore, not calibrated presets for the species shown in the figure. Compare height, crown width, crown base and branch order at fixed seeds before selecting a new descriptor.

The active code is EcoSysLab's `Tree::TryGrow` and `ShootModel`, called through `EcoSysLabLayer::Simulate`. The related [Rhizomorph implementation](https://github.com/edisonlee0212/rhizomorph) accompanies [Rhizomorph: The Coordinated Function of Shoots and Roots (2023)](https://doi.org/10.1145/3592145). [Synthetic Silviculture: Multi-scale Modeling of Plant Ecosystems (2019)](https://doi.org/10.1145/3306346.3323039) is related background requested in the correspondence; this branch does not claim to be an exact reproduction of that paper. The current [EcoSysLab package documentation](https://github.com/edisonlee0212/EvoEngine/tree/dev/EvoEngine_Packages/EcoSysLab) describes the maintained package. Surface union uses [Manifold](https://manifoldcad.org/docs/html/).

## Placing several trees

`compose_scene.py` combines existing trees into a GLB scene. Supply a JSON file with `trees`, each containing `id`, `species`, `mesh` (relative path), and `position_m` (X,Y,Z), then run:

```powershell
python ZachQSM/compose_scene.py scene.json --output forest.glb
```

This supports mixed descriptors by placing trees generated in separate runs. It places independently grown trees; it does not reproduce competition in a dense forest. EcoSysLab supports interacting trees in a shared environment, but that is a separate experiment requiring shared-scene growth, spacing and light settings. The 12-tree trial is an open-grown baseline.

## Checks

```powershell
python -m unittest discover -s Scripts/ZachQSM -p test_dataset.py
```

For native repeatability, generate the same seed twice in separate processes and compare `skeleton.csv`, `nodes.yaml`, `flows.yaml`, `growth.csv`, and optional leaf exports. Repeat one seed from the middle of a batch as a separate run to check that prior trees do not affect its result.
