#!/usr/bin/env python3
"""Validate localized DDGI convergence and dynamic recovery with the installed editor."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

from compare_reference_render import compare_hdr_images
from run_ddgi_emissive_m2_validation import capture
from run_ddgi_small_emitter_baseline import SEED, json_safe


FIXTURES = {
    "emissive-small-equal-power": "none",
    "emissive-enable": "enable-emission",
    "emissive-moving-rigid": "translate-x-one-spacing",
    "geometry-moving": "translate-occluder-x",
}
REFRESH_INTERVAL = 120
SCHEDULE_HORIZON = 360


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=root / "out/ddgi-small-emitter/m3")
    parser.add_argument("--editor", type=Path, help="Installed EvoEngineEditor executable override.")
    parser.add_argument("--timeout", type=float, default=900.0, help="Per-launch timeout in seconds.")
    parser.add_argument("--guided-rays", type=int, default=0, help="Guided irradiance rays per probe.")
    parser.add_argument("--guided-emitters", type=int, default=4, help="Maximum emissive guide records.")
    return parser.parse_args()


def main() -> int:
    try:
        args = parse_args()
        if args.timeout <= 0:
            raise ValueError("--timeout must be positive.")
        if args.guided_rays < 0 or args.guided_rays > 4096 or args.guided_emitters < 1 or args.guided_emitters > 8:
            raise ValueError("Guided rays must be 0-4096 and guided emitters must be 1-8.")
        root = repo_root()
        editor = (args.editor or root / "out/install/vs2026-x64/bin/EvoEngineEditor.exe").resolve()
        if not editor.is_file():
            raise FileNotFoundError(f"Installed editor not found: {editor}")
        output_dir = (args.output_dir if args.output_dir.is_absolute() else root / args.output_dir).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        environment = os.environ.copy()
        environment["EVOENGINE_SHADER_CACHE_DIR"] = str(output_dir / "shader-cache")
        environment["EVOENGINE_IMGUI_INI_PATH"] = str(output_dir / "imgui.ini")

        images = {}
        reports = {}
        for fixture in FIXTURES:
            images[fixture], reports[fixture] = capture(
                editor, output_dir, environment, fixture, timeout=args.timeout,
                guided_rays=args.guided_rays, guided_emitters=args.guided_emitters
            )
        repeat, repeat_report = capture(
            editor, output_dir, environment, "emissive-small-equal-power", "repeat", args.timeout,
            args.guided_rays, args.guided_emitters
        )
        repeatability = compare_hdr_images(images["emissive-small-equal-power"], repeat)

        checks = {"rare_emitter_repeat": repeatability["relative_l2_error"] <= 1.0e-6}
        projected_rays_avoided = {}
        for fixture, transition_kind in FIXTURES.items():
            report = reports[fixture]
            convergence = report.get("convergence", {})
            capture_contract = report.get("capture", {})
            maximum = float(convergence.get("variability_maximum", math.nan))
            unstable_fraction = float(convergence.get("unstable_fraction", math.nan))
            checks[f"{fixture}_converged"] = convergence.get("observed") is True and 0 < int(
                convergence.get("frames", 0)
            ) <= 128
            checks[f"{fixture}_localized_metrics"] = (
                math.isfinite(maximum)
                and maximum >= 0.0
                and math.isfinite(unstable_fraction)
                and 0.0 <= unstable_fraction <= 1.0
            )
            checks[f"{fixture}_transition"] = capture_contract.get("transition_kind") == transition_kind
            if transition_kind != "none":
                expected_history_reset = fixture != "emissive-enable"
                checks[f"{fixture}_history_reset"] = (
                    capture_contract.get("history_reset_after_transition") is expected_history_reset
                )
            active_probes = int(report.get("ddgi", {}).get("active_probes", 0))
            rays_per_probe = int(report.get("ddgi", {}).get("rays_per_probe", 0))
            refreshes = SCHEDULE_HORIZON // REFRESH_INTERVAL
            skipped_frames = SCHEDULE_HORIZON - refreshes
            projected_rays_avoided[fixture] = skipped_frames * active_probes * rays_per_probe

        if not all(checks.values()):
            raise RuntimeError(
                "M3 acceptance checks failed: "
                + ", ".join(name for name, value in checks.items() if not value)
            )
        evidence = {
            "schema_version": 1,
            "contract": {
                "resolution": [1920, 1080],
                "measure_frames": 120,
                "deterministic_seed": SEED,
                "refresh_interval": REFRESH_INTERVAL,
                "schedule_horizon": SCHEDULE_HORIZON,
                "guided_rays_per_probe": args.guided_rays,
                "guided_emitter_limit": args.guided_emitters,
            },
            "checks": checks,
            "repeatability": repeatability,
            "projected_post_convergence_rays_avoided": projected_rays_avoided,
            "reports": reports,
            "repeat_report": repeat_report,
            "passed": True,
        }
        evidence_path = output_dir / "evidence.json"
        evidence_path.write_text(json.dumps(json_safe(evidence), indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(f"DDGI M3 localized convergence validation passed; evidence={evidence_path}")
        return 0
    except Exception as error:
        print(f"DDGI M3 localized convergence validation failed: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
