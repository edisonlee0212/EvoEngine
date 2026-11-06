# Matched-scene G1706994 HDR rerun

`Scripts/run_soybean_hdr_experiments.py` reuses the existing soybean library and
copies the original experiment case definitions. Scene geometry, soil/materials,
plant subset seeds, field layout, ROI, transforms, camera setup and ray settings
remain unchanged. The sky changes from the original procedural Skydome to the
G1706994 Scene HDR. The user selected an environment intensity multiplier of 2.
No separate procedural sun or ambient fallback is added.

Outputs live in `out/soybean-experiments/20261017_G1706994_HDR`, separately from
the original `20261016_robust_v1` study. The copied 22-slide English deck is
`presentation/soybean_G1706994_HDR_en.pptx`. The original deck and data remain
untouched. Only retained experiments run: 64 isolated-plant cases, 990 main field
cases, and 8 outer/central cases. Holdout and field-size sensitivity were deleted
from the current source deck and are omitted.

Run the stages with the installed engine's Python 3.12:

```powershell
python Scripts/run_soybean_hdr_experiments.py prepare
python Scripts/run_soybean_hdr_experiments.py smoke
python Scripts/run_soybean_hdr_experiments.py pilot
python Scripts/run_soybean_hdr_experiments.py single
python Scripts/run_soybean_hdr_experiments.py field
python Scripts/run_soybean_hdr_experiments.py edge
python Scripts/run_soybean_hdr_experiments.py capture
python Scripts/run_soybean_hdr_experiments.py analyze
```

Do not rerun a failed stage automatically. Logs and timing receipts preserve each
attempt. Field runs use separate processes for each leaf type and subset seed,
then assemble the original run IDs without changing the experimental cases.
`Scripts/validate_soybean_hdr_experiments.py` uses the bundled scientific Python
to check intensity scaling, sampling, and unchanged selected models/organ areas.

The HDR is a linearized single GoPro exposure, not an absolutely calibrated
radiometric measurement. Results remain relative RGB light scores, not W/m² or
PAR. Camera output gamma is separate from the linear incident-light scores.
