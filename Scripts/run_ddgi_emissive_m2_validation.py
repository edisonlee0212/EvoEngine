#!/usr/bin/env python3
"""Validate M2 DDGI emissive-mesh sampling fixtures with the installed editor."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from compare_reference_render import compare_hdr_images, read_image
from run_ddgi_small_emitter_baseline import SEED, common_command, json_safe, require_hdr, run_command


FIXTURES = (
    "emissive-small-equal-power",
    "emissive-multi",
    "emissive-textured-uv0",
    "emissive-textured-uv3",
    "emissive-alpha-cutout",
    "emissive-one-sided",
    "emissive-double-sided",
)
OUTCOMES = (
    "zero_pdf_rejects",
    "emitter_backface_rejects",
    "alpha_mask_rejects",
    "invalid_sample_rejects",
    "receiver_backface_rejects",
    "shadowed_samples",
    "zero_radiance_samples",
    "nonzero_contributions",
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=root / "out/ddgi-small-emitter/m2")
    parser.add_argument("--editor", type=Path, help="Installed EvoEngineEditor executable override.")
    parser.add_argument("--timeout", type=float, default=900.0, help="Per-launch timeout in seconds.")
    return parser.parse_args()


def capture(
    editor: Path,
    output_dir: Path,
    environment: dict[str, str],
    fixture: str,
    suffix: str = "enabled",
    timeout: float = 900.0,
    guided_rays: int = 0,
    guided_emitters: int = 4,
):
    hdr_fixtures = {
        "emissive-small-equal-power",
        "emissive-enable",
        "emissive-disable",
        "emissive-enable-hdr",
        "emissive-empty",
        "analytic-light",
        "furnace",
    }
    image_path = output_dir / f"{fixture}-{suffix}{'.hdr' if fixture in hdr_fixtures else '.png'}"
    report_path = output_dir / f"{fixture}-{suffix}.json"
    command = common_command(editor, image_path, fixture, 1920, 1080)
    command.extend(
        (
            "--preview-render-mode",
            "rasterization",
            "--preview-warmup-frames",
            "8",
            "--preview-ddgi-report",
            str(report_path),
            "--preview-ddgi-measure-frames",
            "120",
            "--preview-ddgi-guided-rays",
            str(guided_rays),
            "--preview-ddgi-guided-emitters",
            str(guided_emitters),
        )
    )
    run_command(command, editor.parent, environment, timeout, output_dir / f"{fixture}-{suffix}.log")
    image = require_hdr(image_path, 1920, 1080) if image_path.suffix == ".hdr" else read_image(image_path)
    if (image.width, image.height) != (1920, 1080):
        raise RuntimeError(f"Unexpected M2 capture dimensions: {image_path}")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("schema_version") != 7 or report.get("fixture_id") != fixture:
        raise RuntimeError(f"Unexpected M2 report contract: {report_path}")
    sampling = report.get("emissive_sampling", {})
    attempts = int(sampling.get("nee_attempts", 0))
    if (
        sampling.get("stats_available") is not True
        or sum(int(sampling.get(name, 0)) for name in OUTCOMES) != attempts
        or int(sampling.get("inventory_triangles", 0)) <= 0
        or int(sampling.get("eligible_instances", 0)) <= 0
        or int(sampling.get("excluded_emissive_instances", -1)) != 0
        or int(sampling.get("unrepresentable_probabilities", -1)) != 0
    ):
        raise RuntimeError(f"Unhealthy M2 emissive sampling report: {report_path}")
    return image, report


def main() -> int:
    try:
        args = parse_args()
        if args.timeout <= 0:
            raise ValueError("--timeout must be positive.")
        root = repo_root()
        editor = (args.editor or root / "out/install/vs2026-x64/bin/EvoEngineEditor.exe").resolve()
        if not editor.is_file():
            raise FileNotFoundError(f"Installed editor not found: {editor}")
        output_dir = (args.output_dir if args.output_dir.is_absolute() else root / args.output_dir).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        environment = os.environ.copy()
        environment["EVOENGINE_SHADER_CACHE_DIR"] = str(output_dir / "shader-cache")
        environment["EVOENGINE_IMGUI_INI_PATH"] = str(output_dir / "imgui.ini")

        reports = {}
        images = {}
        for fixture in FIXTURES:
            images[fixture], reports[fixture] = capture(editor, output_dir, environment, fixture, timeout=args.timeout)
        repeat, repeat_report = capture(
            editor, output_dir, environment, "emissive-small-equal-power", "repeat", args.timeout
        )
        repeatability = compare_hdr_images(images["emissive-small-equal-power"], repeat)

        checks = {
            "repeat_relative_l2": repeatability["relative_l2_error"] <= 1.0e-6,
            "multi_emitter_inventory": reports["emissive-multi"]["emissive_sampling"]["eligible_instances"] >= 2,
            "textured_uv0_contributes": reports["emissive-textured-uv0"]["emissive_sampling"][
                "nonzero_contributions"
            ] > 0,
            "textured_uv3_contributes": reports["emissive-textured-uv3"]["emissive_sampling"][
                "nonzero_contributions"
            ] > 0,
            "alpha_cutout_rejects": reports["emissive-alpha-cutout"]["emissive_sampling"]["alpha_mask_rejects"] > 0,
            "one_sided_rejects": reports["emissive-one-sided"]["emissive_sampling"][
                "emitter_backface_rejects"
            ] > 0,
            "double_sided_contributes": reports["emissive-double-sided"]["emissive_sampling"][
                "nonzero_contributions"
            ] > 0,
        }
        if not all(checks.values()):
            raise RuntimeError(
                "M2 acceptance checks failed: "
                + ", ".join(name for name, value in checks.items() if not value)
            )
        evidence = {
            "schema_version": 1,
            "contract": {"resolution": [1920, 1080], "measure_frames": 120, "deterministic_seed": SEED},
            "checks": checks,
            "repeatability": repeatability,
            "reports": reports,
            "repeat_report": repeat_report,
            "passed": True,
        }
        evidence_path = output_dir / "evidence.json"
        evidence_path.write_text(json.dumps(json_safe(evidence), indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(f"DDGI M2 emissive validation passed; evidence={evidence_path}")
        return 0
    except Exception as error:
        print(f"DDGI M2 emissive validation failed: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
