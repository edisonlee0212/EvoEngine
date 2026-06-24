# Scots Pine Growth Evidence Anchors

This note records the first default-calibration target for the Scots pine L-system. It is not a full drought or provenance model; it is the baseline used to keep future parameter tuning tied to measured seedlings.

## Experiment-Average Anchors

Primary calibration target: pooled Scots pine seedlings from the 2025 drought-response experiment.

| Trait | Experiment-average anchor | Model knobs used first |
| --- | ---: | --- |
| Seedling height | about 50 mm mean, about 10 mm SD | `target_gdd`, `gdd_per_day`, calendar cap, `plastochron_gdd`, `max_phytomers_per_seasonal_growth`, `internode_length_m` |
| Stem/root-collar diameter | about 3 mm mean, about 0.8 mm SD | `leader_internode_thickness_m`, per-node thickness CV, maturation curves |
| Mature needle length | about 104 mm mean, about 23 mm SD | `needle_length_m`, `needle_year0_length_multiplier`, needle maturation timing |

Control seedlings were longer/thicker than drought seedlings, while height was much less separated. The base default therefore targets the pooled mean; drought/control profiles should be explicit treatment modifiers later.

## Phenology Guidance

The Scots pine literature supports separating calendar phenology from heat-driven expansion:

- Calendar day/year should drive cohort age, senescence, and year-0 vs later-cohort transitions.
- GDD should drive elongation and maturation rates at constant calendar activity.
- Previous-season completion should influence the next-season shoot budget as a bud-preformation/vigor proxy.
- Drought/stress should not be hidden inside the baseline default; add it later as a named treatment/profile.

## Current Default Intent

The canonical default snapshot is a first-pass experiment-average seedling:

- target GDD near two growing seasons, not the older long-run render value,
- GDD/day in a greenhouse-scale range,
- first-season needle maturation in hundreds of GDD rather than thousands,
- later-cohort needles near the measured 100 mm scale,
- conservative previous-season vigor coupling enabled without a full stress physiology model.

Future calibration iterations should report synthetic bounding boxes, stem diameter proxies, needle length proxies, cohort proportions, and render sheets before changing this snapshot again.
