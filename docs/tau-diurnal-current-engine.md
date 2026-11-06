# Position-Y PARbar workflow in the current engine

This ports the field layout and sensor placement from the collaborator's
`tau_seasonal_agreement_handoff` into `PyDigitalAgriculture`. It uses the current
checkout's `CudaModule::EstimateIlluminationRayTracing` with `PARSensorGroup` probes. No historical
ray-generation, hemispherical integration, or material shader implementation is copied.

The current glTF material compatibility path retains legacy scalar roughness/metallic
factors and binds base-color, normal and AO maps. Separate grayscale roughness and
metallic images are not automatically packed into glTF's shared G/B texture slot.
Current L-system material generation also retains its existing material handling.
Thus importing the same assets does not promise identical shading to the old fork.

## Setup

Build with DigitalAgriculture, DatasetGeneration, LSystem, CUDA and
`PythonBinding-PyDigitalAgriculture` enabled. Import the handoff into the resource project:

```powershell
python Scripts/install_tau_scene_assets.py --bundle D:/path/to/tau_seasonal_agreement_handoff
cmake --build out/build/vs2026-x64 --config RelWithDebInfo --target PyDigitalAgriculture --parallel 8
cmake --install out/build/vs2026-x64 --config RelWithDebInfo
```

The importer preserves source scene asset handles and refuses to overwrite differing real
assets. Missing files and Git LFS pointer stubs are replaced. It also derives the supported
PARbar scene described below (requires NumPy). The imported files live
in the `Resources/DigitalAgricultureProject` submodule; retain these resource changes
alongside the engine changes. Leaf/stem textures referenced by the descriptors must
already be available in that resource project: the original handoff omits them.
The runner checks that the albedo, normal and roughness maps resolve before measuring.
It also waits for L-system mesh publication after GPU uploads, then checks that every
plant's rendered triangle count equals its generated geometry snapshot. Merely having
a CPU geometry snapshot does not mean that the plant is present in the ray-traced scene.

### Raised top-bar supports

The runner loads `ManualAssets/Scenes/Sorghum_PositionY_PARBAR45_Supported.evescene`.
`fit_tau_parbar_supports.py` derives it from the two unmodified imported scenes. It
recovers each top bar's upward displacement relative to the original 4x10 model,
after accounting for the Position-Y rig's scale and placement. For BTX the rise is
0.794059 m; for Pawaga it is 0.637988 m.

The uppermost pole segment, its upper coupler and the solar panel are raised
in each rig. The pole segment is extended upward with its bottom and diameter fixed;
the panel and upper coupler are translated by the bar's rise. This preserves their
original vertical offsets relative to the bar without stretching the solar panel.
Each complete rig is then rotated 180 degrees about world Y around the horizontal
midpoint of its top and bottom bar centers. With +X north, this puts low, middle,
top in south-to-north order, as required by the field protocol. Bar heights,
within-rig distances, upward faces and the undirected 45-degree bar axis are preserved.
Plants and ground are not rotated. The source scenes and shared mesh assets remain intact. Rebuilding the
derived scene is deterministic and retains its separate asset handle.

For an existing installation, generate the scene without importing assets again:

```powershell
python Scripts/fit_tau_parbar_supports.py
```

The importer also performs this step and writes a separate support-fit report next
to its import manifest. Being above the canopy does not imply zero instrument
self-shading: the retained pole and panel can still occlude the reference bar.
Low and top heights are retained by agreement. Both middle bar sampling faces are
set to world Y=0.8 m (relative to the scene's nominal ground plane, not a pointwise
terrain clearance). This is a fixed user-selected setting, not a measured field
height, and does not vary with seed. The existing middle poles reach about 0.97 m
and support the raised bars without extension. Middle XZ positions, bar geometry
and sampling spans are retained.
The protocol and PPT give no numerical pole/bar spacing. Existing spacing is retained
as a scene assumption, not a measured field dimension. PPT slide 13 describes the
three sampling levels as well below, just below the top of, and above the canopy.
No illumination results have yet been rerun for this corrected scene.

### No-panicle preview

The `BTX623_20210726_v6_no_panicle.sorghumls` and
`Pawaga_20210726_v8_no_panicle.sorghumls` descriptor variants add only
`enable_panicle: false` to the fixed base descriptors. They have separate asset
handles, preserving the original descriptors. The preview uses seed 7101 and
the supported scene with middle bars at Y=0.8 m. It renders the field without
running the daily illumination sweep. The runner's original descriptor defaults
remain unchanged; select these variants explicitly with `--btx-descriptor` and
`--pawaga-descriptor` when a no-panicle measurement is wanted.

### Ground coverage correction

The inherited ground mesh was about 24 × 24 m but offset toward negative Z; its
positive-Z edge reached only about 4.5 m while the full field extends to Z=12 m.
Consequently, 96 of the 204 planting points were outside its rectangular footprint.
Resource loading checks alone did not detect this.

Confirmed by rendering the collaborator's separate compiled fork at commit
`a58976c5574a26510487bb651234494b0b03b7d7` with its own binding and original full-field
setup: 96/204 actual plant roots are outside the original terrain footprint.
Both scene and terrain asset hashes match the shipped handoff. The same-view image,
plant coordinates and binary provenance are in `out/tau_fork_repro/mary_ground_check`.
No presentation ground extension or soil correction was used in that capture.
The fork saved the image and coordinate/provenance files successfully, then hit an
access violation during `Terminate` (process exit 1, recorded in capture metadata
and `out/tau_fork_repro/mary_ground_capture.log`). No fork shutdown fix was attempted.

After creating the plants, call `EnsureIlluminationSoilContext` **before**
`ConformSorghumLsPlantsToGroundMesh`. It recenters the existing terrain beneath the
field in the terrain's local XZ plane, preserving terrain relief, orientation, scale
and texture density. It requires room for a 3 m margin around the field bounds and
returns false if the ground is too small. Plant XZ coordinates and PARbar transforms
stay fixed; root Y coordinates are then sampled from the corrected terrain location.

`ValidateIlluminationContext` now checks both plant roots and published canopy mesh
bounds against the footprint of this rectangular ground asset. Conforming uncovered
plants fails instead of silently clamping them to an edge sample. Run it again after
growth. `GetIlluminationGroundState` reports terrain corners, outside-root count and
minimum field margin; the runner saves states before and after fitting in `run.json`.

This is a runtime scene correction; the original handoff asset remains intact.
Existing measurement results predate this correction. The user deferred rerunning
illumination, so their numbers must not be presented as measurements of the corrected
ground configuration.

Scene-only verification used `C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe`
with `-X faulthandler out/tau_current/capture_field.py` (exit 0). It verified original
coverage rejection, 96-to-0 outside roots, repeated fitting within 1e-5 m, unchanged
plant XZ/geometry statistics and PARbar probe layout. The grown canopy bounds have a
minimum ground-edge margin of 3.044246 m. Corrected screenshots and metadata are in
`out/tau_current/field_photos_s7101_soil_fixed`. The binding build and three focused
Python tests passed. Installation via `cmake --install out/build/vs2026-x64 --config RelWithDebInfo`
returned 0 (log: `out/tau_current/ground_fix_install.log`). No illumination sweep was rerun.

## One complete valid-window run

Use Python 3.12 matching the built `.pyd`, from the repository root:

```powershell
python Scripts/run_tau_diurnal.py --seed 7101 --output out/tau_current/july26_s7101
```

Defaults are the installed binding at `out/install/vs2026-x64/python`, the
`test_lsystem_sorghum.eveproj` resource project, the Position-Y scene, and July 26
BTx623 v6/Pawaga v8 descriptors. Use `--module-dir`, `--project`, `--sun-csv`,
`--btx-descriptor`, and `--pawaga-descriptor` to override them. The descriptor
filenames resolve under `Assets/ManualAssets/Descriptors/TauSeasonal`.

### Current experiment: fixed base descriptors

For all three dates (July 26, August 22, September 21), use the same default pair:
`BTX623_20210726_v6.sorghumls` and `Pawaga_20210726_v8.sorghumls`.
Change only `--sun-csv` and the output directory when selecting a date; keep the
same seed when comparing dates for the same generated stand. Each run generates
its stand once and keeps it fixed throughout the day. Random differences among
plants and seeds remain; no time-dependent leaf-width or VPD adjustments are applied.
The chosen descriptor paths and `descriptor_schedule: fixed_for_day` are recorded
in `run.json`.

The run uses 17 rows × 12 plants (96 BTx623, 108 Pawaga), row spacing 1 m and
plant spacing 0.3 m. It generates one stand for the whole solar sweep, with leaf
mesh subdivisions 0.02 m / 4, 50 probes on each of six bars, 128 samples per probe,
4 bounces, and 0.001 m normal offset. July's complete **valid window** is 08:15–14:15
MST at 15-minute intervals (25 times), not a 24-hour simulation. The solar table
comes unchanged from the handoff; valid means elevation >30° and azimuth <245°.
The engine skydome defaults are retained; the sun orientation is updated each time.

Outputs:

- `tau.csv`: full-precision top/bottom means, bottom coefficient of variation and
  `tau = mean(bottom probes) / mean(top probes)` for each genotype and time.
- `probes.csv`: all 7,500 scalar readings and positions, including middle bars.
- `plants.json`: actual plant names, positions, genotype, seeds and generation checks.
  Includes pending publication counts and generated/rendered triangle counts.
- `run.json`: binding hash/path, input hashes, settings, checked sensor geometry,
  completion status, counts and elapsed time.

The scalar is RGB luminance of the engine's illumination result. It is not an
absolute calibration to photon PAR units. Tau is dimensionless. Numerical equality
to the collaborator's fork is not a migration criterion, because the current
measurement implementation includes subsequent changes.

All three solar tables are included. The imported August width-level descriptors
remain available as reference assets; they are unused by the current base-descriptor
experiment. August results from this fixed-canopy experiment must be identified
separately from the collaborator's original VPD-adjusted August results.

## Python interface

The new functions are registered only in `PyDigitalAgriculture`, when LSystem and
CUDA are enabled:

- Project: `RunLSystemSorghumProject`, `WaitForProjectIdle`.
- Field: `ConfigureSorghumLsPlantingGrid`, `InstantiateSorghumLsPlantsFromPlantingMarkers`,
  `ConformSorghumLsPlantsToGroundMesh`, `SetSorghumLsGenotypeDescriptors`,
  `ConfigureSorghumLsLeafMeshQuality`, `GrowSorghumLsPlantsToAdulthood`, `GetSorghumLsFieldState`.
- Context: `EnsureIlluminationSoilContext`, `ValidateIlluminationContext`,
  `GetIlluminationGroundState`, `SetSunDirection`, `ConfigureRayTracerSkydome`.
- Sensors: `CreateParbarTopFaceSensorGroup`, `EstimatePARSensors`,
  `GetParbarTopFaceSensorResults`, `ParbarProbeRecord`.

`ConfigureRayTracerSkydome` forwards to the current CUDA lighting configuration.
`EstimatePARSensors` calls the current CUDA service from the binding that owns the
ray-tracing layer, avoiding a separate static CUDA instance inside the runtime package.
`WaitForProjectIdle` drains pending L-system publications as well as asset/GPU uploads.
Probe placement is computed from scene meshes and checked against the collaborator's
verified bar centers and spans after applying the explicit north-order rotation
and 0.8 m middle-height adjustment to the reference. The original `measured_bars.json` is preserved. Probes are not
hardcoded as disconnected points.

Focused CPU checks:

```powershell
python -m unittest discover -s Scripts/tests -p test_tau_diurnal.py -v
```

## Historical verified run (2026-10-21, before ground coverage correction)

The installed current-engine binding completed seed 7101 for all 25 July 26 valid
times, with exit code 0 and clean runtime-package unloading:

```powershell
& 'C:\Users\sdjkl\miniconda3\envs\evoengine\python.exe' -X faulthandler Scripts/run_tau_diurnal.py --seed 7101 --output out/tau_current/july26_s7101_final
```

Loaded module: `D:/Sorghum-Vulkan/out/install/vs2026-x64/python/PyDigitalAgriculture.cp312-win_amd64.pyd`.
The run contains 204 plants, 8,206,780 published plant mesh triangles, 300 probes,
and 7,500 readings. Every plant has zero pending publications and matching snapshot /
rendered triangle counts. Independently recomputing all summary values from probe
rows gives a maximum difference of 2.23e-16. All recorded input/binary hashes match.
The three focused Python tests also pass.

| Time (MST) | BTx623 tau | Pawaga tau |
| --- | ---: | ---: |
| 08:15 | 0.157873 | 0.222377 |
| 12:00 | 0.221949 | 0.413483 |
| 14:15 | 0.271539 | 0.486123 |

Final data, validation record and PNG/PDF curves are under
`out/tau_current/july26_s7101_final`. The log is
`out/tau_current/july26_s7101_final.log`. Earlier diagnostic folders are not the
accepted output; `july26_s7101_current/run.json` is explicitly marked invalid.

The current editor, launcher, agriculture application and core binding were built
successfully. After the final LSystem/binding rebuild, all enabled applications and
runtime payloads were installed successfully (exit 0) using:

```powershell
cmake --install out/build/vs2026-x64 --config RelWithDebInfo
```

Installed applications are `EvoEngineEditor.exe`, `EvoEngineLauncher.exe` and
`DigitalAgricultureApp.exe` under `D:/Sorghum-Vulkan/out/install/vs2026-x64/bin`.
The integration test used the Python executable shown above; GUI interaction was
not part of this test. Build/install logs are in `out/tau_fork_repro/current_field_*.log`.
# July/September no-panicle ensemble

## Extension to 1000 seeds and September panicles (2026-10-23)

The completed campaign is `out/tau_current/tau_1000_campaign`. Its two sequential
conditions are July/September without panicles, followed by September with
panicles. Each condition has a 1000-seed cap (7101–8100); the two-date condition
uses a joint stability check, while the September panicle condition stops
independently. Both retain the previously agreed 30-seed minimum, 10-seed
checkpoints, strict 0.01 threshold for every pointwise quantile and three
consecutive passing checks. All other scene, leaf mesh and illumination settings
remain fixed. The with-panicle condition selects the original July base
descriptors, whose default `enable_panicle` is true.

The first condition copies and validates the prior 100 completed paired seeds
before generating seed 7201 onward. The original 100-seed directory is preserved.
Reuse requires matching descriptors, settings, inputs, daily runner and runtime
hashes. The new ensemble controller itself is allowed to differ. Paired plant
states are compared for each seed. Results and figures are refreshed every 10
seeds; a completed `validation.json` is written only on successful completion.

```powershell
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/run_tau_ensemble.py --max-seeds 1000 --project out/tau_current/ensemble_project_minimal/test_lsystem_sorghum.eveproj --reuse out/tau_current/july_september_no_panicle --output out/tau_current/tau_1000_campaign/july_september_no_panicle
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/run_tau_ensemble.py --max-seeds 1000 --dates 2021-09-21 --panicles --project out/tau_current/ensemble_project_minimal/test_lsystem_sorghum.eveproj --output out/tau_current/tau_1000_campaign/september_with_panicle
```

The saved `run_campaign.ps1` executes these in order, stops on an error and records
its live state in `campaign.json`. Each daily subprocess has a 1800-second timeout;
failures are retained and are not automatically retried. Each condition has its
own raw run directories, command log, ensemble manifest, checkpoints, seed table,
quantiles, field-minute copy and figures. Current progress is in `ensemble.json`;
the legacy field `completed_pairs` means seeds completed across all selected dates,
including the single-date condition. `completed_day_runs` is the actual day count.

Twelve focused tests passed. No engine source changes or new engine build were
needed for this controller extension. `cmake --install out/build/vs2026-x64
--config RelWithDebInfo` succeeded; the installed binding and recorded runtime
DLL hashes still match the original experiment. The Python executable is
`C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe`, loading
`out/install/vs2026-x64/python/PyDigitalAgriculture.cp312-win_amd64.pyd`.

### Material Passport

- Origin Skill / Mode: experiment-agent / run.
- Origin Date: 2026-10-23.
- Verification Status: ANALYZED; campaign complete and September comparison independently checked.
- Version Label: tau_1000_campaign_v1.

### Completed campaign and September comparison

The user-selected final presentation (2026-10-24) is July 26 without panicles and
September 21 with panicles, each using its completed 270-seed ensemble. Separate
two-genotype figures are in `out/tau_current/tau_1000_campaign/final_results`:
`july26_no_panicles` and `september21_with_panicles`, each in PNG, PDF and SVG.
The folder also contains the selected quantiles, field-minute tables, input hashes
in `selection.json`, and `plot_final.py` for figure-only regeneration. The same
axis limits, colors, screening and pointwise intervals are used in both figures.
The September no-panicle outputs remain as a comparison condition. This selection
does not change the fixed July base descriptors or introduce seasonal growth.
Final figures and exported field-minute tables now use the intersection of the
real and simulated valid time windows: July 08:15–14:15 (361 real minutes per
genotype) and September 09:00–15:30 (391 minutes per genotype). The native one-minute
real cadence and 15-minute simulation cadence are retained; there is no value
interpolation or reduction of real measurements to the simulation sampling grid.
Both current valid windows are contiguous, checked by the plotting script. The
source measurement tables remain unchanged; `final_results/selection.json` records
the original and selected counts and common endpoints.
At the user's request, the final figures now shade the pointwise minimum–maximum
range across all 270 seeds, retaining the median curve. The final exported
quantile tables include `minimum` and `maximum` alongside the original quantiles;
both figures use the same expanded y limits to avoid clipping extrema. Titles,
legends and axis labels are retained, while sample-count annotations and footer
prose are omitted. This is a plotting change: the completed experiments stopped
using 2.5%/50%/97.5% quantile stability, and min/max stability was not tested.

Both conditions stopped at 270 seeds after passing the 250, 260 and 270 seed
checks. The paired-date no-panicle condition completed 540 day runs in 13,222.594
seconds (including reuse of the original 200 runs); the September panicle condition
completed 270 day runs in 14,601.734 seconds. The campaign ended on 2026-10-23 at
21:59 EDT. Final maximum checkpoint changes were 0.007031 without panicles and
0.008167 with panicles. These satisfy the specified empirical stability rule.

`september_panicle_comparison/compare.py` reconstructs both September seed tables
from 540 daily tau files, checks low/top ratios, recomputes the 54 quantile groups
per condition, and compares identical seed IDs 7101–7370. All exported plant
names, genotypes, seeds, positions and leaf counts agree between conditions;
this does not establish identical full leaf/stem geometry. The real-minute tables
are identical. Outputs include an overlaid PNG/PDF/SVG, `summary.csv`,
`time_quantiles_comparison.csv`, `paired_seed_differences.csv` and
`comparison_validation.json` with source hashes. The comparison script exited 0
and the overlaid figure was visually checked on 2026-10-24.

For BTx623, the mean of the 27 timepoint medians changes from 0.241961 without
panicles to 0.236212 with panicles (-2.38%); for Pawaga, 0.242746 to 0.235755
(-2.88%). These are equal-weight summaries of sampled times, not integrated daily
transmittances. Median tau decreases at all 27 times for both genotypes. Average
pointwise interval widths decrease by 2.27% and 2.31%, respectively. The largest
median decreases occur at 14:00: 0.011255 for BTx623 and 0.014404 for Pawaga.

## July 12:30 fixed-stand sampling diagnosis (2026-10-28)

Selected July no-panicle stands by BTx623 tau: minimum seed 7212 (0.109813),
nearest-median seed 7127 (0.359377), maximum seed 7155 (0.704053). The experiment
uses the same installed binding, SDK/package DLLs and scene/descriptor hashes as
the archived campaign, and the same 204-plant layout, ground, 300 probes and sun
angles. Exported plant states match the archived runs exactly and remain unchanged
through all measurements and captures. Repeating the original 128-sample ray seed
18 reproduces both genotypes' archived tau exactly for all three stands.

`Scripts/diagnose_tau_sampling.py` runs 128, 512 and 2048 samples with ray seeds
1001–1010, retaining four bounces and the 0.001 m offset. Each stand is grown once
and held in memory throughout the 30 repetitions and baseline measurement. Three
matching CUDA views show the low-bar surroundings, the view along the solar
direction and the sky seen just above the low-bar center. Camera images are
qualitative views, not hemispherical gap-fraction measurements.

| BTx623 stand | 128-sample repeat mean | 512-sample repeat mean | 2048-sample repeat mean | 2048 repeat min–max |
|---|---:|---:|---:|---:|
| Low: 7212 | 0.112090 | 0.112332 | 0.112490 | 0.111646–0.114091 |
| Middle: 7127 | 0.358457 | 0.358275 | 0.358435 | 0.356035–0.360432 |
| High: 7155 | 0.693416 | 0.697492 | 0.698966 | 0.695887–0.702143 |

At 2048 samples the high-minus-low mean difference is 0.586477, versus a maximum
within-stand repeat SD of 0.002253 and repeat range of 0.006256. The largest mean
change from 128 to 2048 samples is 0.005550. This supports plant-configuration
dependent light paths as the dominant source of the selected low/high separation,
rather than ray-sampling variability. It is not a population variance decomposition
and does not establish that the descriptor variability matches the real field.

At 2048 samples, 0/50, 16/50 and 45/50 low-bar points respectively have mean
normalized readings above 0.5 (each point divided by its run's mean top reading).
The low stand's point means span 0.066–0.158; the high stand is bright over most
of the bar. Upward views show the solar region obscured by leaves in the low
stand and an opening in the high stand. This is consistent with local canopy
occlusion; the contribution of individual organs or direct/diffuse light was
not separately measured.

Outputs: `out/tau_current/july1230_sampling_diagnostic`, including per-stand
`measurements.csv`, `probes.csv`, `plants.json`, `run.json`, three PNG captures,
combined summaries, comparison images and `validation.json`. All 93 measurements
and 27,900 probe rows were checked; tau was independently reconstructed from the
probe values. All three worker processes and the analysis script exited 0.

Commands used (with the corresponding label/seed/output for each stand):

```powershell
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/diagnose_tau_sampling.py --seed 7212 --label low --output out/tau_current/july1230_sampling_diagnostic/low_s7212
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' out/tau_current/july1230_sampling_diagnostic/analyze.py
cmake --install out/build/vs2026-x64 --config RelWithDebInfo
```

The existing verified binary was deliberately reused; no engine source changed.
Executable: `C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe`; binding:
`out/install/vs2026-x64/python/PyDigitalAgriculture.cp312-win_amd64.pyd`.
Python syntax validation and 12 existing focused tests passed. Installation logs
are in the diagnosis directory.

### Material Passport

- Origin Skill / Mode: experiment-agent / run and validate.
- Origin Date: 2026-10-28.
- Verification Status: VERIFIED for archived baseline reproduction; ANALYZED for sampling diagnosis.
- Version Label: july1230_sampling_diagnostic_v1.

## Original 100-seed experiment results

Run the corrected field for 2021-07-26 and 2021-09-21 with matched plant seeds,
fixed July base descriptors with `enable_panicle: false`, 128 samples and 4 bounces:

```powershell
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/run_tau_ensemble.py --project out/tau_current/ensemble_project_minimal/test_lsystem_sorghum.eveproj --output out/tau_current/july_september_no_panicle
```

This creates a new output directory and preserves each day's raw probes, per-time
tau, plant state, manifest and process log. Seeds start at 7101 and stop at 7200
at the latest. Each seed is generated separately for both dates with the same
descriptor and layout settings. Solar tables retain their 15-minute sampling.

The isolated project copies the current project file, all `Assets/ManualAssets`
files and the referenced `Assets/GeneratedAssets/Materials` leaf/stem textures,
including their metadata sidecars, with byte-identical SHA-256 verification
in `asset_snapshot.json`. These contain the imported scene dependencies and the
current corrected scene and descriptors. It avoids scanning unrelated generated
soybean assets at each startup. The initial attempt against the full project was
stopped during asset scanning, before producing any measurement; its log is kept
under `out/tau_current/july_september_no_panicle_100` and is excluded from analysis.
An intermediate isolated attempt (`july_september_no_panicle_100_isolated`) was
rejected by the pre-measurement texture completeness check; it is also excluded.
The completed snapshot contains 197 verified files.

At every 10 seeds, calculate linear 2.5%, 50%, and 97.5% sample quantiles at every
date/genotype/time. Starting at 30 seeds, compare against the preceding checkpoint
(30 vs 20, 40 vs 30, etc.). A check passes only when every absolute change is
strictly below 0.01 tau. Stop after three consecutive passing checks (earliest 50
seeds); otherwise run all 100. This is an empirical stability rule, not proof of
distributional convergence or a confidence interval for the mean. The stopping
criterion does not use agreement with the field data.

`ensemble.json` records input hashes, commands, completed seed pairs and all
checks. `quantiles_nNNN.csv` preserves checkpoints; `simulation_seed_values.csv`
and `simulation_time_quantiles.csv` contain the final ensemble and pointwise band.
The comparison figure reuses `out/tau_fork_repro/diurnal_comparison/field_minutes_screening.csv`
without changing screening, normalization, smoothing, or scaling. A copy of the
selected valid real minutes is saved with the outputs. Plotting connects the
simulation samples for display; it does not create measurements between them.

## Completed ensemble: 2026-10-23

### Material Passport

- Origin Skill / Mode: experiment-agent / run.
- Origin Date: 2026-10-23.
- Verification Status: ANALYZED; execution, output consistency and quantile recomputation checked.
- Version Label: july_september_no_panicle_v1.

The command above completed with exit code 0 in 7,932 seconds (2 h 12 min 12 s).
It ran all 100 matched seeds (7101–7200): 200 complete daily runs, 10,400 genotype/time
tau values, and 1,560,000 individual probe records. The final comparison uses the
same 1,558 screened genotype/minute field observations as the previous processing
for these two dates. The independently reconstructed quantiles match the saved
outputs. All paired plant-state exports are identical across the two dates;
runtime and input hashes are consistent across all runs. Every field has 204
plants, loaded leaf/stem textures, covered planting points and middle probes at
0.8 m. See `out/tau_current/july_september_no_panicle/validation.json`.

**The run stopped at the 100-seed cap, not because the band met the stability rule.**
None of the eight checks passed. The last check (100 vs 90 seeds) has maximum
changes of 0.029712 for the lower endpoint, 0.010313 for the median and 0.019413
for the upper endpoint. The worst last-check change is July BTx623 at 13:00,
lower endpoint. These results are an empirical 100-stand distribution; they do
not establish convergence or physical validity of the model.

| Seeds | Maximum absolute change in tau | Pass (<0.01 everywhere) |
|---|---:|---|
| 30 | 0.059502 | No |
| 40 | 0.054230 | No |
| 50 | 0.053542 | No |
| 60 | 0.028237 | No |
| 70 | 0.039483 | No |
| 80 | 0.029471 | No |
| 90 | 0.029865 | No |
| 100 | 0.029712 | No |

Final artifacts in `out/tau_current/july_september_no_panicle/`:

- `diurnal_comparison.png`, `.pdf`, `.svg`: two dates by two genotypes; minute field
  curves, seed medians and pointwise 2.5–97.5% bands. Visually inspected.
- `simulation_seed_values.csv`: individual seed/date/time/genotype tau.
- `simulation_time_quantiles.csv`: 104 time/genotype groups, each with 100 seeds.
- `field_minutes_used.csv`: unchanged selected real minute observations.
- `convergence_history.csv`, `convergence.png`: all recorded checkpoint changes.
- `ensemble.json`, `validation.json`, `verification.log`: provenance and checks.
- `runs/<date>/s<seed>/`: raw `probes.csv`, `tau.csv`, `plants.json`, `run.json`.

The earlier `preview_n010/` figure is explicitly an interim 10-seed check.
Use the root-level comparison figure for the final result.

Validation command (exit 0):

```powershell
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' out/tau_current/verify_tau_ensemble.py
```

Ten focused tests passed with `python -m unittest discover -s Scripts/tests -p 'test_tau*.py'`.
Before this batch, the current runtime was built and all configured applications
installed successfully (both commands exit 0):

```powershell
cmake --build out/build/vs2026-x64 --config RelWithDebInfo --target PyDigitalAgriculture --parallel 1 -- /p:BuildProjectReferences=false /p:TrackFileAccess=false /nr:false
cmake --install out/build/vs2026-x64 --config RelWithDebInfo
```

Executable used: `C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe`.
Binding used: `D:/Sorghum-Vulkan/out/install/vs2026-x64/python/PyDigitalAgriculture.cp312-win_amd64.pyd`.
Build/install logs: `out/tau_current/ensemble_build.log` and `ensemble_install.log`.

## Descriptor dispersion sensitivity (2026-10-28)

This diagnostic tests whether reducing the existing L-system descriptor dispersion
narrows the July 26 12:30 transmittance distribution. It does not change production
descriptors, the field layout, sun, PARbars, materials or the CUDA integrator.
The separate output root is `out/tau_current/july1230_variance_diagnostic/`.

### Generator audit

The measured plants use `SorghumLSDescriptor` and `SorghumGrowthModel`, rather than
the separate DigitalAgriculture `SorghumGenerator` class. Each base no-panicle
descriptor explicitly contains 26 distributions with nonzero standard deviations.
`SampleDistribution` and `SamplePlotted` in `LSystemRuleHelpers.hpp` use normal
distributions; `deviation` means standard deviation, not variance. Plotted values
are the interpolation of `min_value` and `max_value` using the rank-dependent
curve, not simply the stored `max_value`.

Representative nominal input distributions, before organ-specific clamps,
flag-leaf scaling, tiller scaling and basal-internode adjustments:

| Parameter | BTx623 | Pawaga |
|---|---|---|
| Blade length | Mean 27.6–92 cm by rank; SD 10 cm | Mean 27.6–92 cm; SD 9.5 cm |
| Maximum blade width | Mean 2.21–9.7 cm; SD 0.9 cm | Mean 1.94–8.5 cm; SD 0.765 cm |
| Basal internode length (rank fraction 0) | Mean 1.06 cm; SD 1.71 cm | Mean 1.035 cm; SD 1.54 cm |
| Upper internode length (rank fraction 1) | Mean 7.875 cm; SD 6.108 cm | Mean 7.689 cm; SD 5.5 cm |
| Upper leaf-sheath length (rank fraction 1) | Mean 19.05 cm; SD 11.19 cm | Mean 14.89 cm; SD 15.27 cm |
| Leaf roll offset about the culm | SD 11.92–16 degrees | SD 11.92–16 degrees |
| Main-culm lean | Mean 6.8 degrees; SD 4 degrees | Same |
| Main phytomer count | Mean 14; SD 1, rounded | Same |
| Tiller count | Mean 2.25; SD 0.5, rounded and clamped to 1–3 | Mean 2; SD 0.5, same limits |

Some length dispersions are large relative to their means. This is a source-code
and configuration finding, not a claim that they exceed measured biological
variation. The actual trait distributions have not been calibrated here.
`parameter_profiles.csv` and `parameter_summary.csv` evaluate all 26 explicitly
nonzero distributions per genotype. Rank profiles describe nominal inputs, not
exported final organ dimensions.

The code also retains randomness outside descriptor deviations:

- `SorghumGrowthModel::Initialize` chooses a uniform 0–360 degree main-axis
  phyllotaxis phase per plant.
- Leaf rules sample twist amplitude uniformly between 0 and the configured
  maximum (currently 30 degrees, whose descriptor deviation is zero), along with
  random twist phase and frequency.
- Static wind azimuth, leaf waviness phases/amplitude jitter, atlas variant and
  small tint variation remain seed dependent.

Reducing descriptor SD therefore does not eliminate all plant randomness.
Also, positive-length clamping and integer count rounding can change realized
trait means when SD changes, despite unchanged descriptor means. For example,
the nominal expected rounded/clamped BTx623 tiller count is approximately 2.242,
2.157 and 2.023 for SD scales 1, 0.5 and 0.25 respectively (normal-distribution
calculation, not a measured plant statistic).

### Controlled experiment

- Thirty predetermined field seeds: 7101 through 7362 inclusive, step 9. These
  were selected before observing sensitivity results, independently of tau.
- Each seed uses SD scales 1, 0.5 and 0.25 for both genotypes; input variance
  scales are 1, 0.25 and 0.0625. Only explicitly serialized `deviation` scalars or
  their plotted min/max values change. All means and curve shapes stay unchanged.
- A complete private copy of the archived minimal project keeps the production
  assets and previous experiment inputs unchanged. Omitted fields retain inline
  defaults, including unused panicle distributions; panicles remain disabled.
- 204 plants, identical plant-name/seed/position assignments, six bars with 50
  probes each, original soil, July 26 12:30 sun, 2048 samples, four bounces,
  push distance 0.001 m and ray seed 18 for every primary measurement.
- Each original-parameter stand first repeats 128 samples/ray seed 18 and checks
  both genotypes against the archived result, then measures at 2048 samples.
- Every condition uses a fresh process. The initial attempt to replace multiple
  canopies in one process exhausted CUDA memory; its partial results remain in
  `attempt_shared_process/` and are excluded from analysis.
- The range comparison always uses the same 30 field seeds per condition, not
  30 treated stands against all 270 original stands. Report full min–max width,
  SD, median/mean and central 95% width together.

Same field seeds do not guarantee identical leaf orientations after a parameter
change: rounded phytomer counts can change the random-number consumption sequence.
Consecutive field-seed assignments also reuse overlapping individual plant seeds.
This is a descriptive sensitivity test, not an independent-sample inferential
test or an organ-by-organ variance decomposition. One time/date does not establish
the behavior of the full diurnal or seasonal band.

Commands (preparation uses the base Python's installed `ruamel.yaml`; measurement
and analysis use the existing `evoengine` environment):

```powershell
python Scripts/diagnose_tau_variance.py prepare
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/diagnose_tau_variance.py run
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/analyze_tau_variance.py
```

Material Passport: local source and descriptor audit ANALYZED; numerical reproduction
and output integrity VERIFIED. The following sensitivity interpretation is descriptive,
limited to the specified date/time, descriptors and 30 field seeds.



### Completed results

All 90 primary measurements completed successfully. All 30 additional original-parameter
128-sample baselines reproduced both archived genotype tau values exactly (maximum error 0).
All 36,000 probe rows were reconciled with the ratio-of-panel-means summaries.
Production input hashes, private project input hashes and worker script hashes matched.

| Genotype | Descriptor SD scale | Mean tau | Median tau | Tau SD | Full range | Range width |
|---|---:|---:|---:|---:|---|---:|
| BTX | 1 | 0.4128 | 0.3906 | 0.1307 | 0.2207–0.6991 | 0.4783 |
| Pawaga | 1 | 0.3728 | 0.3598 | 0.1031 | 0.1819–0.5956 | 0.4138 |
| BTX | 0.5 | 0.4713 | 0.5054 | 0.1115 | 0.2330–0.6687 | 0.4358 |
| Pawaga | 0.5 | 0.3735 | 0.3432 | 0.1248 | 0.1695–0.6263 | 0.4569 |
| BTX | 0.25 | 0.4758 | 0.4631 | 0.1289 | 0.2367–0.7231 | 0.4864 |
| Pawaga | 0.25 | 0.4151 | 0.4084 | 0.1380 | 0.1384–0.7330 | 0.5947 |

BTx623 full-range width fell 8.9% at half SD, but rose 1.7% at quarter SD;
its tau SD fell 14.7% and 1.4% respectively. Pawaga full-range width increased
10.4% and 43.7%; its tau SD increased 21.0% and 33.8%. Thus the experiment does
not show a consistent or monotonic reduction in tau dispersion from shrinking
all serialized descriptor deviations.

The intervention did reduce one observed morphological dispersion: per-plant
total leaf-count SD fell from 7.77 to 5.05 to 2.10 for BTx623, and from 7.53 to
2.97 to 0.67 for Pawaga. These statistics pool the exported plants and describe
them; they do not treat reused individual plant seeds as independent samples.
The top-reference coefficient of variation stayed between 0.089% and 0.121%
across the six groups. The widened ranges are therefore not explained by a
near-zero or highly variable top reference.

The means also moved: BTx623 mean tau was 0.4128 / 0.4713 / 0.4758 and Pawaga
0.3728 / 0.3735 / 0.4151. Unchanged input means do not imply unchanged realized
canopy means under rounding/clamping. Intrinsic random orientation remains,
and small geometry changes can move a local bar between a leaf shadow and a gap.
These are verified implementation features and plausible mechanisms, not an
isolated attribution of the measured changes to a particular random source.
A narrower simulation band alone would not establish better biological fidelity.

Artifacts: `variance_comparison.png` / `.pdf`, `range_summary.csv`,
`all_measurements.csv`, `plant_leaf_count_summary.csv`, `parameter_summary.csv`,
`parameter_profiles.csv`, `descriptor_audit.json`, `experiment.json`,
`validation.json`, `analysis.log`, and `runs/s<seed>/<condition>/` raw data.
The comparison image was visually checked. It shows all seeds, seed-paired
lines and the median with full min–max range; it is not a confidence interval.

Fourteen focused Python tests passed. No C++ engine source was changed or rebuilt;
measurements used `C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe` and
`D:/Sorghum-Vulkan/out/install/vs2026-x64/python/PyDigitalAgriculture.cp312-win_amd64.pyd`.
The existing applications were installed with
`cmake --install out/build/vs2026-x64 --config RelWithDebInfo` (exit 0; `install.log`).
Archived runtime and input hashes still matched after installation.## Fixed initial leaf azimuth sensitivity (2026-10-28)This separate diagnostic uses the original no-panicle descriptor distributions,not the reduced-SD variants. It changes only the initial main-axis phyllotaxisphase from its original uniform 0–360 degrees to a fixed local phase of 0 degrees.Per-leaf roll, branch/tiller azimuth jitter, twist, wind and size distributionsremain unchanged. This does not make every final leaf point in exactly the sameworld-space direction, and 0 degrees is an experimental condition rather thanan inferred field orientation.The generator now exposes two descriptor fields:```yamlrandomize_initial_leaf_azimuth: falseinitial_leaf_azimuth_degrees: 0```Omitting them retains the original random behavior. The apex's `node_random`draw is retained even in fixed mode, so other stochastic trait draws stay aligned.The selected initial phase is stored on the growth model and can be inspectedthrough `PyDigitalAgriculture.GetSorghumLsInitialLeafAzimuths()`, which returns aplant-name-to-degrees dictionary. The existing field-state API schema is unchanged.Descriptor serialization and the editor inspector support both settings.Thirty existing seeds (7101–7362, step 9) are run at July 26 12:30, with bothoriginal random phase and fixed 0-degree phase in the newly built runtime.Each primary measurement uses 2048 samples, four bounces, ray seed 18 and theunchanged field/PARbar/soil/sun setup. Original-mode workers also repeat the old128-sample result. A fresh process handles each condition to avoid accumulatingmultiple canopy allocations in CUDA memory.Output root: `out/tau_current/july1230_fixed_phase_diagnostic/`.It includes a private project copy, the complete pre-change Python runtime in`runtime_before_change/`, and the preceding variance worker source in`variance_worker_before_change.py`. Fixed-mode descriptor files are exactly theoriginal file bytes with the two explicit phase settings prepended. Existingexperiment assets are not modified. Runtime hashes necessarily change with thenew option; numerical original-mode reproduction is checked separately.Commands:```powershell& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/diagnose_tau_initial_azimuth.py prepare& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/diagnose_tau_initial_azimuth.py run& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/analyze_tau_initial_azimuth.py```The related 30 C++ tests passed, including RNG/trait preservation and descriptorround-trip/legacy-default tests. Both new tests also passed through CTest underthe `lsystem` label. Fourteen Python tau tests passed.Build and test logs are in the output root. Measurements use`C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe` and the installed`D:/Sorghum-Vulkan/out/install/vs2026-x64/python/PyDigitalAgriculture.cp312-win_amd64.pyd`.The C++ test executable is`D:/Sorghum-Vulkan/out/build/vs2026-x64/EvoEngine_Tests/RelWithDebInfo/EvoEngine_Tests.exe`.The changed targets were built successfully with:```powershellcmake --build out/build/vs2026-x64 --config RelWithDebInfo --target LSystemPackage PyDigitalAgriculture PyEvoEngine EvoEngine_Tests DigitalAgricultureApp --parallel 2 -- /p:BuildProjectReferences=false /p:TrackFileAccess=false /nr:falsecmake --install out/build/vs2026-x64 --config RelWithDebInfo```Both commands completed with exit code 0. The install log is `install.log` inthe experiment output root; the successful target build log is`build_targets_full_access.log`.All 60 paired configurations completed (30 field seeds, two initial-phaseconditions). The 30 additional 128-sample baseline measurements bring the totalto 90 measurements and 27,000 probe rows. Both the archived 128-sample baselinesand the preceding experiment's 2048-sample originals reproduced with maximumabsolute tau error 0. All 6,120 fixed-mode plants reported phase 0 degrees;paired names, seeds, positions and leaf counts matched. Runtime and projecthashes remained unchanged throughout the run.| Genotype | Initial phase | Mean tau | Tau SD | Minimum | Maximum | Full range width || --- | --- | ---: | ---: | ---: | ---: | ---: || BTx623 | Random | 0.412771 | 0.130744 | 0.220716 | 0.699062 | 0.478346 || BTx623 | Fixed 0 deg | 0.200007 | 0.066212 | 0.098288 | 0.372293 | 0.274005 || Pawaga | Random | 0.372819 | 0.103133 | 0.181865 | 0.595648 | 0.413784 || Pawaga | Fixed 0 deg | 0.198306 | 0.059439 | 0.094612 | 0.297993 | 0.203381 |The observed full range narrowed by 42.7% for BTx623 and 50.8% for Pawaga;tau SD decreased by 49.4% and 42.4%, respectively. Mean transmission also fellsubstantially, so this intervention changes the illumination/occlusion patternas well as the absolute spread. It supports initial phyllotaxis phase as aninfluential input under this tested sun direction and field setup. It does notestimate a fraction of total variance explained, prove convergence of thesample extrema, or establish the correct orientation for the real field.`initial_phase_comparison.png` and `.pdf` show all paired seeds, medians andfull min-max ranges. `all_measurements.csv`, `range_summary.csv`,`initial_leaf_azimuths.csv`, per-run raw probes, `experiment.json`, and`validation.json` retain the results and verification evidence. The PNG wasvisually inspected for labels, legend placement and clipping.Material Passport: implementation, tests and experiment VERIFIED.This is a paired descriptive sensitivity comparison at one fixed phase and time,not a demonstration that the fixed phase matches the real field.### Initial azimuth width and center comparison (2026-10-28)Material Passport: implementation, tests and experiment VERIFIED.The follow-up retains July 26 12:30 MST, the same 30 field seeds (7101-7362,step 9), no panicles, original trait distributions, 204 plants, existing scene,sun, PARbars, 2048 samples, four bounces and ray seed 18. It adds five conditionsto the preceding full-circle random and fixed-0 results:| Condition | Center | Total width | Descriptor arc start || --- | ---: | ---: | ---: || Original full circle (reused) | Not applicable | 360 | 0 || Fixed 0 (reused) | 0 | 0 | 0 || c0_w180 | 0 | 180 | 270 || c0_w90 | 0 | 90 | 315 || c0_w30 | 0 | 30 | 345 || c90_w0 | 90 | 0 | 90 || c90_w90 | 90 | 90 | 45 |Angles are local initial main-axis phyllotaxis phases in degrees, wrappedto [0,360). They are not measured compass bearings. The new descriptor widthfield `initial_leaf_azimuth_range_degrees` defaults to 360. In random mode,`initial_leaf_azimuth_degrees` specifies the start of the uniform arc. Samplinguses `(start + apex.node_random * width) mod 360` with the original uniform draw;all other random draws and descriptor distributions remain unchanged.The full-circle original retains its original start of 0 degrees; the boundedarcs share a centered quantile mapping with each other. Fixed mode is unchanged.Before running the 150 new configurations, the new runtime repeats original andfixed-0 conditions at seeds 7101, 7155 and 7362. Their plant states, actual phases,and measurements must match the prior files exactly. The other 60 comparisonconfigurations are reused from the verified previous experiment. All reused rawinputs are hashed, as are the current runtime and private project. Each freshworker checks 204 phases against the mapping of saved original phases to within0.0001 degrees (float roundoff), plus plant names, seeds, positions and leaf counts.Output root: `out/tau_current/july1230_azimuth_ranges/`. It includes a privateproject and pre-change runtime/worker backups, per-configuration measurements,raw probes, plant states, actual phases, execution logs and provenance manifest.The runner uses a fresh process per configuration with a 600-second timeout.```powershell& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/diagnose_tau_azimuth_ranges.py prepare& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/diagnose_tau_azimuth_ranges.py run& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/analyze_tau_azimuth_ranges.py```The related 31 C++ tests and 14 Python tau tests passed. The C++ executable was`D:/Sorghum-Vulkan/out/build/vs2026-x64/EvoEngine_Tests/RelWithDebInfo/EvoEngine_Tests.exe`.The measurement executable is the Python executable above, loading`D:/Sorghum-Vulkan/out/install/vs2026-x64/python/PyDigitalAgriculture.cp312-win_amd64.pyd`.The affected targets were built with the same explicit target command in thepreceding section. `cmake --install out/build/vs2026-x64 --config RelWithDebInfo`completed with exit code 0; logs are `build.log`, `install.log`, `tests.log` and`python_tests.log` in the new output root.Analysis is descriptive: sample min-max ranges, means, medians and SDs. Theoverlapping per-plant seed sequences do not support treating all field seeds asindependent observations. This is a single-time sensitivity experiment and doesnot determine real-field orientation or prove convergence of full-range extrema.All 150 new configurations completed. Combined with 60 reused configurations,the comparison contains 210 primary measurements; including the old 128-samplechecks gives 240 measurements and 72,000 probe records. All six fresh runtimecontrols matched exactly. The 42,840 plant-phase records matched the requestedarc mappings (maximum angular difference 0.00001653 degrees); paired names,plant seeds, positions and leaf counts matched. Input hashes remained unchanged.| Initial azimuth | BTx623 mean | BTx623 full range width | Pawaga mean | Pawaga full range width || --- | ---: | ---: | ---: | ---: || Full circle | 0.412771 | 0.478346 | 0.372819 | 0.413784 || 0 +/- 90 deg | 0.324937 | 0.473547 | 0.317320 | 0.340256 || 0 +/- 45 deg | 0.243514 | 0.456376 | 0.227448 | 0.246905 || 0 +/- 15 deg | 0.200448 | 0.291666 | 0.200625 | 0.295812 || Fixed 0 deg | 0.200007 | 0.274005 | 0.198306 | 0.203381 || 90 +/- 45 deg | 0.505170 | 0.389961 | 0.466941 | 0.464979 || Fixed 90 deg | 0.563363 | 0.395505 | 0.568383 | 0.289814 |At center 0, narrowing to +/-90 and +/-45 degrees reduced BTx623 range widthby only 1.0% and 4.6%; narrowing to +/-15 reduced it by 39.0%. Pawaga reductionswere 17.8%, 40.3% and 28.5%, respectively, so its full range did not decreasemonotonically with angular width. With center 90 and width 90, Pawaga's rangewas 12.4% larger than the full-circle reference. Fixed 90 produced mean tauaround 0.56-0.57, compared with about 0.20 at fixed 0. Both angular dispersionand center affect illumination; a smaller angular range alone does not ensurea smaller output range or a better match to the real field. Absolute spreadalso changes alongside the mean. `range_summary.csv` includes tau CV to makerelative spread inspectable, plus SD, median and exact extrema.`azimuth_width_comparison.png/.pdf` and `azimuth_center_comparison.png/.pdf`show all seeds, means, medians and full min-max ranges; both PNGs were visuallyinspected. `all_measurements.csv`, `initial_leaf_azimuths.csv`, `validation.json`and per-run raw data retain the numerical evidence. The statistical claims aredescriptive; no p-values, variance-explained fractions, population-confidenceintervals or claims of an optimal real-field orientation are inferred.A logging-only issue was found during execution: the arc start variableshadowed the monotonic timer, making the new per-run `elapsed_seconds` invalid.Numerical sampling and measurement did not use that timing value. The actualexecuted sources are preserved as `worker_used.py` and `driver_used.py`; theirhashes are checked by analysis. The helper's local variable was renamed aftercompletion, and Python checks passed again. Raw run manifests retain theiroriginal values and `validation.json` flags the invalid timing field; usefilesystem timestamps for this campaign's timing. The current helper's sourcehash consequently differs from the executed snapshot, and the runner's hashguard intentionally rejects resuming this completed campaign with changed code.Interpretation audit: 11/11 checks covered. Genotypes are reported separately(Simpson's paradox); conclusions stay at field-output level (ecologicalfallacy); seeds were predetermined rather than selected by tau (Berkson'sselection bias and regression to the mean); no outcome-dependent covariateswere conditioned on (collider bias); diagnostic base rates are not applicable;all planned configurations completed and are included (survivorship bias);all seven conditions are reported without significance testing (look-elsewhereeffect); widths, centers and seeds were fixed before execution (forking paths);the intervention supports effects within this simulation only, with no claimof validated real-field causation; the direction is explicit configuration tomeasurement rather than reverse causality. The main practical limits remainshared plant seeds, one time point, sample extrema and unmeasured real orientation.### Full July descriptor SD comparison (2026-10-28)

Material Passport: experiment and analysis VERIFIED (2026-10-29).

This expands the earlier 30-seed, single-noon SD pilot to the original July
trend's complete scale: 270 field seeds (7101-7370), all 25 valid solar times
08:15-14:15 MST at 15-minute intervals, 128 samples, four bounces and ray seeds
1-25 in time order. Plants are generated once per day/seed, with no panicles and
unchanged layout, PARbars, soil, textures and base descriptors. The exact pilot
descriptor variants (all explicitly serialized deviations scaled by 0.5 or 0.25,
means/curves preserved) are reused. Initial azimuth retains full-circle random
initialization. This does not lock downstream random outcomes across altered
rounded counts/topology, as documented for the earlier pilot.

Each reduced-SD group runs all 270 full days, without early stopping, for 540 new
days. The 270 original days are reused after checking their complete time grids,
raw probe counts, tau ratios and agreement with the archived seed-value table.
New-runtime full-day controls at seeds 7101, 7235 and 7370 reproduced archived
plant states and every tau/probe CSV value exactly. The latter two controls ran
concurrently; 70.4 seconds for two controls versus 67.0 seconds for the serial
control supported using two worker processes. Each day has a 600-second timeout.
On failure queued jobs are cancelled, completed data retained, and the error
recorded. A resume validates and reuses completed day files without overwriting.

The output root is `out/tau_current/july_full_sd_comparison/`. Its `project/`
and `runtime/` are private snapshots, protecting the long run from later changes
to the installed engine or project. Input and source hashes are recorded in
`experiment.json`; executed worker/driver sources are saved. Progress is updated
after each full day. Individual runs retain `tau.csv`, `probes.csv`, `plants.json`
and `run.json`, with separate process logs.

```powershell
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/run_tau_full_sd_comparison.py prepare
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/run_tau_full_sd_comparison.py controls
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/run_tau_full_sd_comparison.py run
& 'C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe' Scripts/analyze_tau_full_sd_comparison.py
```

The unchanged measurement worker is `Scripts/run_tau_diurnal.py`. The actual
executable is `C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe`, loading
`D:/Sorghum-Vulkan/out/tau_current/july_full_sd_comparison/runtime/PyDigitalAgriculture.cp312-win_amd64.pyd`.
No C++ engine or raytracer changes are required for this expansion. Sixteen
Python tau tests passed, including checks that incomplete paired grids, duplicate
measurements and nonfinite values cannot produce a comparison band.

Installation was checked with `cmake --install out/build/vs2026-x64 --config
RelWithDebInfo` (exit code 0; `install.log`). Measurement keeps using the private
runtime snapshot. Both first reduced-SD days exactly matched the corresponding
pilot plant states; all 50 time-point tau values were independently recomputed
from their 15,000 probes before leaving the campaign running.

The completed analysis validated all 810 full days, recomputed tau from
6,075,000 probe records, verified all 90 matching plant states from the old noon
pilot, and reproduced the original min/median/max curves. It reused the original
722 screened real-data minutes (361 per genotype) and the original intersection
of valid time windows unchanged. Three individual plots and an overlaid
comparison use common axis limits, pointwise full min-max ranges and medians.
No observations are interpolated onto a different cadence; plotted lines and
bands connect the existing sampled times.

All 540 new full days completed without early stopping in 22,276 seconds
(6 h 11 min). Analysis exited 0 and verified 6,075,000 probe records,
40,500 seed/genotype/time records, all 90 matching pilot plant states,
unchanged input hashes and all 722 retained real-data minutes.

| Genotype | SD scale | Original mean band width | Mean band width | Change | Narrower time points / 25 | Mean median shift |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| BTx623 | 0.5 | 0.385999 | 0.432114 | +11.95% | 4 | +0.043430 |
| BTx623 | 0.25 | 0.385999 | 0.423152 | +9.63% | 6 | +0.073301 |
| Pawaga | 0.5 | 0.385969 | 0.431450 | +11.78% | 2 | +0.023748 |
| Pawaga | 0.25 | 0.385969 | 0.448538 | +16.21% | 0 | +0.046166 |

Mean band width is the arithmetic mean of the 25 pointwise max-minus-min
widths, not the range pooled over the entire day. Reduced descriptor SD did
not narrow the average band in any comparison. Some individual times narrowed,
while Pawaga at quarter SD widened at all 25 times. Median transmission shifted
upward on average. Plant leaf-count SD nevertheless decreased: BTx623
7.7634 -> 5.0399 -> 2.1164; Pawaga 7.5458 -> 2.9796 -> 0.6681.
These results support reduced variability in that structural trait, but do not
identify a single cause of the wider transmission bands.

The same 270 seeds ensure matched replication and time grids. SD scaling can
change rounded topology and subsequent random draws; per-plant orientations
were not locked. These pointwise min-max bands are not confidence intervals for
the mean, and their extrema are sample-dependent. This expansion changed only
experiment/analysis scripts, not the engine or measurement implementation.

Outputs are `july26_original`, `july26_half_sd`, `july26_quarter_sd`, and
`july26_sd_trend_comparison` as PNG/PDF/SVG; `simulation_seed_values.csv`,
`simulation_time_summary.csv`, `field_minutes_used.csv`,
`plant_leaf_count_summary.csv`, `range_comparison.csv`, and `validation.json`
retain the numerical results. `analysis.log` records successful validation.
The overlay and independent plots were visually inspected; the plotting margin
was increased to preserve the last time tick, without changing measurements.
Completion is reported in this chat; heartbeat 7 is removed after reporting
preparation to prevent duplicate notifications.
