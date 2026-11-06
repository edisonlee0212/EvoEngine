# Helios soybean batch exporter

## Current consolidated study

`Scripts/run_soybean_experiments.py --stage build|library|preview|single|convergence|field|validation` records commands, logs, timestamps and durations under `out/soybean-experiments/20261016_robust_v1`. Run each named stage separately (the pipe notation lists choices, not a shell command). Completed stages are not repeated automatically. All model files use a new asset subdirectory, leaving earlier experiments intact. The library has 1,024 matched models per leaf type, split into 768 training and 256 disjoint holdout generation seeds.

The current analysis entry point is `Scripts/analyze_soybean_experiments.py --study <study-folder>` and its notebook `Scripts/soybean_experiment_analysis.ipynb`. It reconciles organ/plant/field RGB sums and ROI counts before reporting separate leaf and whole-plant metrics. Run analysis after the field stage before selecting holdout candidates. The old `analyze_prefab_field_illumination.ipynb` is retained as historical analysis only. Python 3.12 runs the engine binding; analysis additionally requires NumPy, pandas and Matplotlib.

`search.samples` can vary numerical sampling without restarting the engine. `search.spacing_pairs_inches` selects explicit `[row_spacing, plant_spacing]` pairs instead of the Cartesian spacing grid, allowing all holdout candidates to share one engine initialization. An unchanged field configuration is reused between sampling settings.

The study chooses training sample count using a recorded convergence criterion: the lowest of 16 or 64 whose maximum relative difference from a 256-sample reference is at most 1% over all check cases and three light metrics, otherwise 256. Holdout and buffer checks always use 256. `sampling_decision.json` records the observed errors and decision. Notebook execution additionally uses `nbformat`, `jupyter_client`, and an installed `ipykernel`; the driver uses the existing scientific environment rather than installing packages. Saved scenes preserve geometry and field sampling settings. OptiX layer lighting must be restored from the corresponding JSON configuration when reproducing measurements.

Each export now includes `.geometry.json` with realized leaf count, plant height and Helios leaf-object area, plus per-model time and OBJ SHA-256 in the manifest. `Scripts/describe_soybean_library.py` independently measures exported geometric triangle area and leaf facet inclinations. These areas need not equal the Helios alpha-aware leaf-object area. Changing leaf aspect ratio can change stem girth through Helios' downstream-leaf-area pipe model, and transverse tessellation also depends on aspect ratio. The study compares complete generated phenotypes, not an equal-area/fixed-stem intervention.

This bridge keeps soybean generation inside Helios and exports each generated plant as a static Wavefront OBJ. The Python driver builds the small C++ bridge, runs one fresh Helios context per seed, and writes a reproducibility manifest.

## Quick start

From the repository root:

```powershell
python Scripts/export_helios_soybeans.py `
  --helios-root D:\Helios `
  --output-dir out\helios-soybeans `
  --count 20 `
  --seed-start 1000 `
  --age-days 45
```

The output directory contains `soybean_0000.obj`, the matching `.mtl` files, copied texture assets, and `manifest.json`. The manifest records the Helios commit, age, parameter overrides, and seed for every plant. Different seeds preserve the Helios parameter distributions while producing different static plants.

Optional overrides:

```powershell
python Scripts/export_helios_soybeans.py `
  --helios-root D:\Helios `
  --count 20 `
  --age-days 45 `
  --max-nodes 20 `
  --internode-length-m 0.035 `
  --leaf-size-range-m 0.10 0.14 `
  --leaf-aspect-ratio-range 0.85 1.05 `
  --petiole-length-range-m 0.12 0.16 `
  --leaf-subdivisions 8
```

Only supplied overrides replace Helios values. In Helios' current soybean model, the default trifoliate values are 25 maximum nodes, 0.035 m internode length, 0.10–0.14 m leaflet prototype scale, 1.0 leaflet aspect ratio, 0.12–0.16 m petiole length, and 8 leaf subdivisions.

`--leaf-aspect-ratio-range` controls leaflet width divided by length. Values near `1.0` produce broad leaflets, while values near `0.45` produce narrow leaflets. The override is applied to both trifoliate and unifoliate soybean leaves.

The exporter requests vertex normals. Helios writes OBJ/MTL data and copies referenced leaf/flower/fruit textures beside the exports. OpenMP is intentionally enabled because the inspected Helios OBJ writer's non-OpenMP path only emits its first CPU-sized face chunk. The OBJ geometry is static; changing age or parameters requires another generation run.

Helios has no Python binding for `plantarchitecture` in the inspected source tree. Python therefore orchestrates the batch, while the linked C++ executable performs the actual model generation. Distributing the linked exporter also requires respecting the Helios GPL license.

## Field parameter search

`Scripts/search_prefab_field_illumination.py` can generate a matched broad/narrow library once at the beginning of a search. The example configuration creates 256 plants of each type with the same Helios seed sequence, then each 4-by-10 field selects 40 distinct models deterministically from that library. Reusing a subset seed across spacing combinations isolates spacing effects; repeating three subset seeds measures plant-to-plant variation.

Validate the 594-run grid without generating models or launching OptiX:

```powershell
py -3.12 Scripts/search_prefab_field_illumination.py `
  --module-dir out\install\vs2026-x64\python `
  --config Scripts\prefab_field_illumination_search.example.json `
  --validate-only
```

Prepare the matched 256-model broad/narrow libraries once:

```powershell
py -3.12 Scripts/search_prefab_field_illumination.py --module-dir out/install/vs2026-x64/python --config Scripts/prefab_field_illumination_search.example.json --prepare-model-library-only
```

Then run or repeat the search without invoking Helios again:

```powershell
py -3.12 Scripts/search_prefab_field_illumination.py --module-dir out/install/vs2026-x64/python --config Scripts/prefab_field_illumination_search.example.json
```

Each search case selects 40 models from the existing library without replacement. The resolved model-library metadata is saved to `search_manifest.json`, and each plant CSV row records the selected source `asset_index`. The current versioned study uses `Scripts/analyze_soybean_experiments.py` and `Scripts/soybean_experiment_analysis.ipynb` for organ-aware incident-light metrics. `Scripts/analyze_prefab_field_illumination.ipynb` is retained only for historical results.

### Central measurement ROI

`PrefabField.illumination_roi` and the optional Python `roi` argument default to `1.0` (the complete field). Existing scenes without this serialized field and old Python calls keep full-field measurement. A fraction applies to **each** grid dimension: `rows=8, columns=20, roi=0.5` generates 160 plants and measures the central 4-by-10 plants. Dimensions round to the nearest cell (ties up), with at least one cell; any odd padding leaves the extra buffer cell on the larger-index side. Valid fractions are finite and in `(0, 1]`.

All generated plants remain in OptiX for occlusion and reflection. Only ROI plants receive measurements, and their original full-field `model_index`, `row`, `column`, and sampling seeds are retained. `GetPrefabFieldModelCount` returns the full count; `GetPrefabFieldMeasurementRegion(field)` returns `row_start`, `column_start`, `rows`, and `columns`. The component inspector shows the region and measures it without removing the buffer plants.

The search config accepts `search.roi`, for example `[0.5]`. CSVs record `roi`, `field_plant_count`, `roi_row_start`, `roi_column_start`, `roi_rows`, `roi_columns`, and `measurement_ground_area`. Summary `plant_count` and all illumination aggregates refer only to the ROI. Ground area uses measured cell count times row spacing times column spacing. The current analysis requires the explicit ROI ground area and incident-total schema 2; older datasets stay separate in the legacy notebook. `run_prefab_field_pipeline.py` also accepts `--roi` and saves it with the scene.

The current analysis includes ROI in ranking and paired-comparison keys and plots different ROI settings separately, rather than pooling regions as repeat trials. Original full-field row and column indices remain in the measured-plant CSVs.

### Organ output fields

Each new search also writes `plant_organ_measurements.csv` and `field_organ_summary.csv`. The rows contain `organ`, `total_area`, RGB `total_flux`, and RGB `average_flux`, together with the experiment parameters. Helios `leaf` maps to `leaf`, `shoot` to `stem`, and both `petiole` and `petiolule` to `petiole`. Other mesh names remain separate, so a general field can retain additional groups such as flowers. All geometry still participates in occlusion and reflection; the groups reuse a single sampling pass.

The Python API exposes `result.organs["leaf"]` for a plant and `summary.organs["leaf"]` for the whole field. Each value has `total_area`, `total_flux`, and `average_flux`. The field's organ average is area weighted. The field inspector displays both field and per-plant organ results; `run_prefab_field_pipeline.py --illumination-json` includes an `organs` object per plant.

`average_flux` preserves the face-mean convention: a double-sided leaf reports `(E_front + E_back) / 2`. `total_flux` sums the incident contributions from all sampled faces: `A * (E_front + E_back)` for leaves and `A * E` for one-sided stem/petiole surfaces. `total_area` always counts geometric area once. For mixed groups, `average_flux` is area weighted independently and is no longer `total_flux / total_area`. Values remain relative RGB, not calibrated PAR or absorbed energy. New manifests identify this definition with `organ_reporting.schema_version = 2`. Earlier CSV totals use the old face-mean area integral and must not be compared directly with new totals. Previously generated whole-plant CSVs cannot be separated by organ without rerunning measurements.

## Recorded robust study (20261016_robust_v1)

`Scripts/run_soybean_experiments.py` defines the versioned study; `Scripts/continue_soybean_study.py` coordinates its remaining stages without regenerating completed libraries or silently retrying failed measurements. These drivers intentionally target this workstation and study ID, not an arbitrary new output directory. Completed raw results are protected from overwrite.

- Each leaf type has 1,024 plants: 768 training and 256 disjoint holdout generation seeds. Broad/narrow aspect ratios are 1.0/0.3. Same input seeds do not imply identical geometry: transverse tessellation and Helios area-dependent stem girth also change.
- A fixed representative pair and 32 paired single-plant draws precede convergence tests and the field search. The main field is 8×20 plants, measuring its central 4×10. Row spacing is 10–35 inch in 2.5-inch steps; plant spacing is 1–5 inch in 0.5-inch steps, converted to metres before engine calls. Five subset seeds give 990 training cases.
- Training uses the lowest tested sample count whose maximum deviation from the 256-sample reference is at most 1% over three light metrics. Candidate spacings are re-evaluated using the disjoint library at 256 samples, followed by buffer and identical-geometry ROI controls.

Records live under `out/soybean-experiments/20261016_robust_v1`: `configs` and `protocol.json` are resolved inputs; `raw` contains untouched CSVs and completion receipts; `analysis` contains reconciled metrics and figures; `logs`, `timings`, and `provenance` retain commands, UTC timestamps, durations and source snapshots. `scene_index.json` locates saved project scenes. Full scene snapshots embed meshes and can be several GB; they are not required to analyze CSVs or regenerate a field from its config. Measurement lighting is configured explicitly on replay, rather than inferred from an editor's saved layer state.

An explicitly selected `run_ids` list executes only those original case IDs, retaining their parameters and seeds. It supports the user-approved recovery in `Scripts/resume_soybean_study.py`: keep the original failed 890-case attempt, check two reference cases against the repaired engine, run only IDs 890–989 into a separate raw folder, then join disjoint rows verbatim under `assembled/03_field_search`. Raw continuation/check chunks set `include_in_study_analysis=false`; the complete assembled dataset is analyzed once. Original failure timings remain visible and are included in work-time accounting.

The larger 12×30 buffer test also partitions by leaf type into separate processes, retaining the same full-case IDs and settings. Its complete dataset is joined under `assembled/05_buffer_sensitivity`; process chunks are not counted as additional statistical repeats.

The current analysis is `Scripts/analyze_soybean_experiments.py` and `Scripts/soybean_experiment_analysis.ipynb`; the older notebook is legacy. Both slides and HTML read the same derived data. Analysis requires NumPy/Pandas; notebook execution and figures additionally require Matplotlib, nbformat, JupyterClient and an installed Python kernel. This workstation uses bundled Python for tabulation, the existing `demeter` environment for plots/notebook execution, and CPython 3.12 for the installed engine binding.

Leaf averages are geometric-area weighted. Incident totals include surface incidences and are not net intercepted fractions, calibrated PAR, absorption or yield. Texture alpha affects ray occlusion, but receiver triangle areas are not fractionally alpha-corrected. Static plants can interpenetrate; collision avoidance and growth adaptation are not simulated. Results therefore remain conditional on this geometry and fixed zenith-skydome setup.

### Regenerate presentation figures without running experiments

```powershell
python Scripts/plot_soybean_presentation.py --study out/soybean-experiments/20261016_robust_v1
```

This figure-only script reads the existing ranked field CSV and isolated-plant facts, using Matplotlib/Pandas (the existing `demeter` environment on this workstation). It does not invoke Helios, the engine, or the experiment runner. Outputs are PNGs and a provenance manifest under `analysis/presentation_figures/`; raw measurements and analysis CSVs remain untouched. Each field metric shows the complete row-by-seeding-spacing grid, with a shared broad/narrow color scale and the best tested cell marked. The four metrics are leaf mean irradiance, leaf incident light per plant, whole-plant incident light per plant, and leaf incident light per ground area. Cell values remain the existing means over five field subsets, not independent plant replicates. The isolated comparison retains paired-difference confidence intervals, not group error bars.

Figures use Libertinus Serif. On other systems, supply `--font-file /path/to/LibertinusSerif-Regular.ttf`. Insert the PNGs into slides rather than recreating their data as PowerPoint charts. The manifest records source hashes, fixed settings, color limits, and best-cell coordinates for verification.

### Legacy versus current illumination results

The historical inch-grid heatmap is not numerically comparable to the current study without accounting for implementation and protocol changes. Its primary hemisphere contribution omitted division by the uniform-hemisphere PDF, `1 / (2*pi)`; the independently sampled direct-sun term was already present. The current estimator includes the PDF weight, so the total does **not** change by one constant `2*pi` multiplier. The old figure's “Mean leaf-area irradiance” also actually used the area-weighted whole-plant surface mean, and its total-light values used the earlier face-mean area integral rather than the corrected face sum.

A bounded audit under `out/soybean-experiments/20261016_robust_v1/audits/legacy_difference_20261017` reconstructs the exact historical raygen Git blob, preserves source/build evidence, and runs 20 controls at the two spacing endpoints with subset seed 101. Four historical field RGB means reproduce exactly using the reconstructed kernel; four current cases reproduce all field/plant/organ area and RGB values exactly. Library/subset, field/ROI, metric definition and sample-count effects are recorded separately in `comparison_chain.csv` and `ground_light_comparison_chain.csv`; `report.html` describes attribution limits. The field/ROI step also changes receiver selection and is not a pure causal buffer test. Original measurements, production source/runtime and the presentation remain unchanged. No full experiment pipeline was rerun.
