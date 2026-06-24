# Scots Pine Parameter Evidence Map

This audit maps the Scots pine descriptor surface to what is directly measured, biologically evident, biologically inducible, image-observable, or purely logical/model-control. It is meant to guide future default changes and prevent calibration-by-hunch.

## Evidence Classes

| Class | Meaning | How to tune |
| --- | --- | --- |
| Direct measured | The experiment or species description gives a numeric or categorical anchor. | Fit against measured data first; use variance from observed cohorts when available. |
| Biological proxy | Literature supports the process, but this model parameter is an abstraction. | Tune against measured outcomes and keep the mechanism interpretable. |
| Image observable | Visible in real renders/images, but not measured in the current trait table. | Tune from side-by-side render sheets and segmentation/label evidence. |
| Logical control | Necessary for the L-system, renderer, or editor, but not a biological trait. | Set conservatively; expose only when it helps calibration. |

## Source Anchors

- Local measured table: `tmp/scots-pine-literature/1_1 Growth_related.txt` from Figshare DOI `10.6084/m9.figshare.29474057`.
- Experiment-average anchors from that table: needle length `104.10 +/- 23.07 mm` (`n=777`), stem/root-collar diameter `3.08 +/- 0.78 mm` (`n=777`), height `50.45 +/- 10.46 mm` (`n=810`).
- Treatment split: control needle length `115.38 +/- 22.11 mm`, stem diameter `3.62 +/- 0.64 mm`, height `51.60 +/- 10.05 mm`; treatment needle length `92.67 +/- 17.82 mm`, stem diameter `2.53 +/- 0.45 mm`, height `49.30 +/- 10.75 mm`.
- Bud break/phenology: Salminen and Jalkanen, Frontiers in Plant Science 2015, DOI `10.3389/fpls.2015.00104`; bud break varies by calendar/daylength and temperature, and Scots pine leader shoot length is strongly previous-season determined.
- Species morphology: Scots pine usually has two needles per fascicle, juvenile first-year seedlings can have single juvenile leaves, mature needles persist several years, and needle color/bark color vary with age/season.
- Segmentation/imagery PDFs: use their categories as image-observable classes: lower/older needles, current-year needles, apical/young tissue, bud scales/sheaths, specular highlights, and background/label separation.

## Global Time and Phenology

| Parameter | Evidence class | Current role | Biological mapping |
| --- | --- | --- | --- |
| `target_gdd.mean/deviation` | Biological proxy | Per-instance thermal growth cap. | Induces final size and cohort stage; fit jointly to height, needle cohort age, and branch state. Not directly measurable without the exact temperature record. |
| `gdd_per_day.mean/deviation` | Biological proxy | Calendar-to-thermal conversion with constant activity `1`. | Should represent indoor/greenhouse forcing after the outdoor first season. Use controlled temperature logs when available; otherwise fit to calendar duration and organ maturity. |
| `plastochron_gdd.mean/deviation` | Biological proxy | Thermal spacing between emitted phytomers. | Tune to phytomer count and height, not independently. Variance gives plant-to-plant developmental heterogeneity. |
| `max_phytomers_per_seasonal_growth.mean/deviation` | Biological proxy | Seasonal apex emission budget. | Represents preformed shoot capacity; Scots pine leader growth is previous-season influenced, so this should couple to previous-season vigor. |
| `needle_bud_storage_vigor_strength` | Biological proxy | How strongly previous-season completion constrains next-season capacity. | Biologically plausible bud-preformation proxy; tune only after height/needle length targets are stable. |
| `needle_bud_storage_completion_floor` | Logical biological proxy | Lower clamp for previous-season vigor. | Prevents collapse from one weak season; choose based on stress-profile behavior, not base morphology. |

## Topology and Branching

| Parameter | Evidence class | Current role | Biological mapping |
| --- | --- | --- | --- |
| `max_branching_order.mean/deviation` | Image observable / biological | Enables lateral branch recursion. | Seedling images with only a leader should stay near `0`; older branched seedlings can increase to `1+`. |
| `branches_per_whorl.mean/deviation` | Biological proxy | Number of lateral buds per whorl. | Scots pine has whorled branching, but seedling branch count must be matched from images; set deviation only if visible branch-count variance exists. |
| `whorl_dormancy_years.mean/deviation` | Literature-supported proxy | Delay before lateral buds expand. | Annual whorl release is plausible; keep around `1` unless modeling immediate lateral emergence. |
| `branch_insertion_angle_deg.mean/deviation` | Image observable | Lateral branch elevation angle. | Fit from side-view geometry if branches are present; currently mostly irrelevant when `max_branching_order=0`. |
| `branch_roll_phyllotaxis_deg.mean/deviation` | Logical / morphology proxy | Rotational spacing of laterals. | Golden-angle-like rotation is a sensible procedural default; tune visually for branch distribution. |
| `lateral_length_ratio.mean/deviation` | Image observable | Lateral internode length scale vs leader. | Tune from branched plants only. |
| `lateral_thickness_ratio.mean/deviation` | Image observable | Lateral diameter scale vs leader. | Tune from branched plants only. |
| `branch_angle_per_node_sigma_deg.mean/deviation` | Logical/image observable | Per-node angular noise. | Use to break perfect regularity after core branch geometry is correct. |
| `roll_phyllotaxis_per_node_sigma_deg.mean/deviation` | Logical/image observable | Per-node roll noise. | Same: small naturalism control, not a measured trait. |

## Stem Geometry, Age, and Orientation

| Parameter | Evidence class | Current role | Biological mapping |
| --- | --- | --- | --- |
| `internode_length_m.mean/deviation` | Direct measured via fit | Length per phytomer. | Jointly fit to measured height and phytomer count; do not tune independently from `plastochron_gdd` and seasonal cap. |
| `leader_internode_thickness_m.mean/deviation` | Direct measured via fit | Main stem diameter scale. | Fit to root-collar/stem diameter proxy; remember rendered stem may not equal measured root collar exactly. |
| `internode_maturation_gdd.mean/deviation` | Biological proxy | Time to full internode length/width. | Fit to observed elongation stage at imaging date; faster maturation for fully extended shoots. |
| `internode_length_maturity_curve` | Biological proxy | Shape of length maturation through normalized maturity. | Smooth sigmoid is reasonable; alter only if measured time series show different elongation dynamics. |
| `internode_width_maturity_curve` | Biological proxy | Shape of width maturation through normalized maturity. | Width may lag length; current shared sigmoid is logical until diameter time series exist. |
| `internode_age_exponent` | Image observable | Visual aging/browning curve for stem color. | Tune from render color, not growth. |
| `internode_length_per_node_cv.mean/deviation` | Logical/image observable | Local internode length variation. | Use for natural unevenness after mean height is fitted. |
| `internode_thickness_per_node_cv.mean/deviation` | Logical/image observable | Local diameter variation. | Use sparingly; can break diameter calibration. |
| `initial_orientation_yaw_deg.mean/deviation` | Logical/render | Whole-plant yaw randomness. | Dataset diversity/camera composition only. |
| `gravitropism_first_order.mean/deviation` | Biological proxy / image observable | Leader curvature response. | Use only for visible leader curvature; not in current trait table. |

## Needle Initiation and Cohorts

| Parameter | Evidence class | Current role | Biological mapping |
| --- | --- | --- | --- |
| `bare_zone_fraction.mean/deviation` | Image observable | Fraction of seasonal axis without needles. | Fit to visible leader/needle distribution. |
| `needle_count_per_cluster.mean/deviation` | Direct species morphology | Needles emitted per fascicle sheath. | Scots pine fascicles usually have two needles; juvenile-leaf detail is currently outside this minimal dataset generator. |
| `fascicle_sheath_length_m.mean/deviation` | Direct species morphology / image observable | Length of the basal sheath cylinder that owns the fascicle branching point. | Scots pine needles emerge from a short persistent sheath; keep this mm-scale and tune from close synthetic labels or real sheath visibility. |
| `fascicle_sheath_width_m.mean/deviation` | Direct species morphology / image observable | Diameter of the basal sheath cylinder. | Should be slightly wider than the combined needle bases but much thinner than a shoot internode; tune from close silhouette and label masks. |
| `needle_year0_length_multiplier` | Biological proxy / direct fit | Initiation-year length scale. | Fit to the visibly youngest/top cohort; later cohorts use neutral length scaling of `1`. |
| `needle_order_length_attenuation` | Biological proxy / logical | Length reduction by branch order. | Set only when branches exist; leader-only seedlings should keep neutral. |
| `needle_order_radius_attenuation` | Biological proxy / logical | Radius reduction by branch order. | Same. |
| `needle_order_min_length_scale` | Logical guard | Lower clamp for order attenuation. | Prevents degenerate needles. |
| `needle_order_min_radius_scale` | Logical guard | Lower clamp for radius attenuation. | Prevents invisible needles. |
| `needle_intra_year_capacity_curve` | Biological proxy / direct fit | Seasonal needle capacity curve by normalized phytomer index. | Replaces the legacy scalar sigmoid/late-decay controls; default ramps from 75% early-season capacity to 100%. Tune directly from cohort gradients. |

## Needle Size, Lifecycle, and Maturation

| Parameter | Evidence class | Current role | Biological mapping |
| --- | --- | --- | --- |
| `needle_length_m.mean/deviation` | Direct measured | Later-cohort needle length, with optional year-0 multiplier. | Anchor to measured mature needle length; deviation should contribute to the measured ~22% CV. |
| `needle_lifespan_years.mean/deviation` | Species/literature morphology | Chronological persistence after maturity. | Scots pine needles persist multiple years; tune for lower old-needle retention only when images show senescence/needle loss. |
| `needle_browning_years.mean/deviation` | Biological proxy / image observable | Time from senescence onset to visual browning/removal. | Separate age browning from drought/stress browning. |
| `needle_flush_delay_gdd.mean/deviation` | Literature-supported proxy | Delay before needle expansion after phytomer initiation. | Active expansion process; fit to apical young-needle stage. |
| `needle_maturation_gdd.mean/deviation` | Biological proxy / direct visual fit | GDD to mature needle length. | Should be hundreds of GDD, not many thousands, for first-season visible maturation. |
| `needle_length_maturity_curve` | Biological proxy | Length expansion curve through normalized maturity. | Smooth sigmoid until time-series elongation data justify a different curve. |
| `needle_branching_angle_deg.mean/deviation` | Image observable | Initial needle angle from axis. | Fit from side views; can reflect juvenile/top needles vs older lower needles only if cohort rules support it. |
| `needle_branching_relax_gdd.mean/deviation` | Biological proxy / image observable | Time for needles to relax toward mature orientation. | Tune top needles going horizontal/downward relative to lower cohorts. |

## Needle Cross Section and Mesh Profiles

| Parameter | Evidence class | Current role | Biological mapping |
| --- | --- | --- | --- |
| `needle_cross_section_width_max_m.mean/deviation` | Image observable / direct species morphology | Max flat width; `0` may use derived behavior. | Species descriptions give mm-scale needle width, but render visibility must be calibrated. |
| `needle_cross_section_thickness_max_m.mean/deviation` | Image observable / direct species morphology | Max thickness/radius proxy. | Use silhouette and mask thickness; measured table lacks cross-section. |
| `needle_cross_section_width_profile` | Biological morphology proxy | Width taper from base to tip. | Taper should be visible and monotone; tune from close views. |
| `needle_cross_section_thickness_profile` | Biological morphology proxy | Thickness taper from base to tip. | Same. |
| `needle_cross_section_temporal_maturity_curve` | Biological proxy | Width/thickness maturation over time. | Width may mature differently from length; current curve is logical until measured. |
| `needle_segment_count` | Logical/render | Longitudinal tessellation count. | Rendering fidelity/performance only. |

## Needle Visual Cohort Features

| Parameter | Evidence class | Current role | Biological mapping |
| --- | --- | --- | --- |
| `needle_lignification_factor_year0` | Image observable / biological proxy | Year-0 material stiffness/visual maturation proxy. | Tune from the youngest cohort; later cohorts use neutral `1`. |
| `needle_stomatal_strip_density_year0` | Image observable/species morphology | Year-0 procedural strip density proxy. | Scots pine has stomatal bands; later cohorts use neutral `1`, so this only shapes the youngest cohort. |
| `needle_basal_taper_ratio_year0` | Image observable | Base radius/taper for year-0 needles. | Tune from silhouettes and fascicle base appearance; later cohorts use neutral `1`. |
| `needle_fascicle_sheath_budget_gdd` | Species morphology / image observable | Sheath visual maturation budget. | Fascicle sheaths are real; tune from close images, not trait table. |
| `needle_specularity_plasticity_year0` | Image observable | Year-0 highlight response. | Rendering/material calibration only; later cohorts use neutral `1`. |
| `young_needle_palette_rgba` | Image observable | Young needle palette. | Fit from real RGB after lighting/background is stable. |
| `older_needle_palette_rgba` | Image observable | Older live-needle palette. | Calibrate to real distal/older live color gradients. |
| `dry_brown_needle_palette_rgba` | Image observable | Senescent/dry needle palette. | Fit from lower/brown needle labels; not a growth parameter. |
| `needle_tip_color_mix_start` | Image observable/logical | Where tip tint begins. | Tune from axial color gradient. |
| `needle_tip_color_exponent` | Image observable/logical | Shape of tip tint gradient. | Tune visually. |
| `needle_old_thinning_fraction` | Image observable | Width/radius loss with old color. | Represents senescent thinning/visibility; fit only after brown cohort timing is right. |
| `needle_min_strand_thickness_m` | Logical/render | Lower bound for rendered strand radius. | Prevents disappearance; tune for visibility, not biology. |
| `needle_micro_variation` | Image observable/logical | Tiny deterministic needle color variation. | Naturalism after mean colors are matched. |
| `stem_micro_variation` | Image observable/logical | Tiny deterministic stem/sheath color variation. | Naturalism after mean colors are matched. |
| `needle_axial_age_span` | Image observable/proxy | Makes tip/base appear older. | Useful for apical/basal color gradients; not directly measured. |
| `needle_axial_age_exponent` | Image observable/logical | Concentrates axial aging. | Shape control only. |

## Needle Curvature, Waviness, and Mechanics

| Parameter | Evidence class | Current role | Biological mapping |
| --- | --- | --- | --- |
| `needle_curvature_adaxial_bias.mean/deviation` | Image observable / morphology proxy | Curvature bias toward adaxial side. | Tune from needle arc/droop shape. |
| `needle_curvature_abaxial_bias.mean/deviation` | Image observable / morphology proxy | Curvature bias toward abaxial side. | Same. |
| `needle_curvature_gradient_per_arclen.mean/deviation` | Image observable/logical | Curvature change along needle length. | Tune from wavy/droopy silhouette. |
| `needle_diameter_for_curvature_m.mean/deviation` | Logical/mechanical proxy | Diameter scale used by curvature response. | Internal scaling; set to match current cross-section scale. |
| `needle_sinusoidal_amplitude_deg.mean/deviation` | Image observable/logical | Waviness amplitude. | Tune from real wavy needles; previous manual tweaks belong here. |
| `needle_sinusoidal_frequency_cycles.mean/deviation` | Image observable/logical | Waviness frequency. | Tune from number of visible waves along needle. |
| `needle_sinusoidal_phase_randomness_deg.mean/deviation` | Logical/image observable | Wave phase randomness. | Prevents synchronized wave artifacts. |
| `needle_young_modulus_baseline_Pa.mean/deviation` | Biological/mechanical proxy | Mechanical stiffness baseline; `0` disables. | Needs biomechanical data or image-inferred droop; leave off until needed. |
| `needle_lignification_maturation_years.mean/deviation` | Biological/mechanical proxy | Time for stiffness maturation; `0` disables. | Use only if physics-style droop becomes active. |
| `needle_density_kg_m3.mean/deviation` | Biological/mechanical proxy | Tissue density for droop simulation. | Needs better tissue data; current value is a render/physics placeholder. |
| `gravity_m_s2.mean/deviation` | Logical/physical | Gravity magnitude for droop. | Physical default is 9.81; current lower value is render tuning. |
| `needle_per_needle_length_cv.mean/deviation` | Direct variance proxy / logical | Needle-to-needle length variation. | Can absorb within-plant variation after plant-level measured variance is matched. |
| `needle_per_needle_curvature_cv.mean/deviation` | Image observable/logical | Per-needle curvature variation. | Use to avoid uniform arcs. |
| `needle_per_needle_radius_cv.mean/deviation` | Image observable/logical | Per-needle radius variation. | Use carefully; affects masks. |
| `needle_per_needle_modulus_cv.mean/deviation` | Mechanical logical | Per-needle stiffness variation. | Only relevant if mechanical droop is active. |
| `needle_per_needle_density_cv.mean/deviation` | Mechanical logical | Per-needle density variation. | Same. |
| `needle_per_needle_wave_amplitude_cv.mean/deviation` | Image observable/logical | Variation in wave amplitude. | Useful for naturalism. |
| `needle_per_needle_wave_frequency_cv.mean/deviation` | Image observable/logical | Variation in wave frequency. | Useful for naturalism. |
| `needle_per_needle_wave_phase_cv.mean/deviation` | Image observable/logical | Variation in wave phase. | Useful for naturalism. |

## Colors and Seasonal/Render Traits

| Parameter | Evidence class | Current role | Biological mapping |
| --- | --- | --- | --- |
| `main_stem_palette_rgba` | Image observable | Young stem/shoot color. | Fit from real images after lighting is fixed. |
| `mature_bark_stem_palette_rgba` | Image observable | Older stem/bark color. | Fit from lower stem/older tissue labels. |
| `node_sheath_brown_palette_rgba` | Image observable | Bud/node sheath brown palette. | Fit from node and basal sheath regions after lighting is fixed. |
| `fascicle_sheath_palette_rgba` | Image observable / species morphology | Basal fascicle sheath color. | Use a brown/tan, bud-scale-like palette until real sheath-specific targets exist; synthetic labels export sheaths separately but postprocess maps them as stem-like tissue. |
| seasonal tint fields in `LSystemLayer` | Image observable / biological proxy | Scene-level seasonal color modulation. | Use for seasonal appearance only; do not bake stress or age effects entirely into global tint. |

## Editor and Dataset Layout Controls

| Parameter | Evidence class | Current role | Biological mapping |
| --- | --- | --- | --- |
| `live_preview` | Logical/editor | Enables descriptor preview. | No biology. |
| `live_preview_rate_hz` | Logical/editor | Preview update rate. | No biology. |
| `live_preview_representative_only` | Logical/editor | Preview simplification. | No biology. |
| `live_preview_cap_target_gdd` | Logical/editor | Preview cap toggle. | No biology, but affects what the user sees while tuning. |
| `live_preview_max_gdd` | Logical/editor | Preview cap value. | Should be near current calibration target for useful previews. |
| `live_preview_max_growth_steps` | Logical/editor | Preview work cap. | Performance guard. |
| `grid_rows`, `grid_cols`, `grid_spacing` | Logical/editor | Descriptor preview layout. | No biology. |
| `triangle_side_length` | Logical/dataset layout | Three-tree synthetic scene spacing. | No plant biology; affects occlusion/composition. |

## Practical Tuning Order

1. Fit geometry anchors: `target_gdd`, `gdd_per_day`, calendar cap, `plastochron_gdd`, `max_phytomers_per_seasonal_growth`, `internode_length_m`, `leader_internode_thickness_m`.
2. Fit needle cohort scale: `needle_length_m`, `needle_year0_length_multiplier`, `needle_maturation_gdd`, `needle_branching_relax_gdd`.
3. Fit topology only if visible: branching order, whorl count, lateral length/thickness, insertion angle.
4. Fit silhouette/detail: cross-section width/thickness, profiles, curvature/waviness, per-node/per-needle CVs.
5. Fit colors/materials last, after geometry and cohort ages are stable.
6. Keep stress treatment separate: treatment/control differences belong in explicit profiles, not hidden in the base default.

## Known Model Gaps Exposed by This Audit

- First-year Scots pine seedlings can show juvenile single needles, while this minimal generator keeps one fascicle model and only exposes year-0 cohort scaling.
- The current parameter set can express previous-season vigor, but it does not yet encode explicit drought/water-potential physiology.
- Cross-section, stomatal strip, specularity, and color parameters are real-looking image controls, not measured functional traits in the current data. The explicit fascicle sheath geometry is morphology-driven, but its exact palette is still image-calibrated.
- The best future calibration loop should export synthetic height, stem diameter proxy, mature needle length proxy, cohort counts, and color-label summaries for every candidate default.
