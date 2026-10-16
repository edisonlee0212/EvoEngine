#!/usr/bin/env python3
"""Capture opt-in SSS fixtures in Sponza at 1080p and compare linear output."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
from pathlib import Path

import numpy as np

from compare_reference_render import read_hdr


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--editor", type=Path, default=root / "out/install/vs2026-x64/bin/EvoEngineEditor.exe")
    parser.add_argument("--output", type=Path, default=root / "out/sss-validation")
    parser.add_argument("--pilot", action="store_true", help="Run the raster/raw-PT smoke cases only.")
    parser.add_argument("--case", action="append", default=[], help="Run only the named case; repeat for multiple cases.")
    args = parser.parse_args()
    editor = args.editor.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    cases = [
        ("raster", "rasterization", "mixed", "disabled", "reblur", 96, 1, False),
        ("pt-raw", "ray-query", "mixed", "disabled", "reblur", 64, 1, False),
        ("no-sss-raster", "rasterization", "disabled", "disabled", "reblur", 96, 1, False),
        ("no-sss-pt", "ray-query", "disabled", "disabled", "reblur", 64, 1, False),
    ]
    reuse_integrators = {
        "restir-candidate": "candidate",
        "restir-temporal": "temporal",
        "restir-spatial": "spatial",
        "restir-enhanced": "enhanced",
        "restir-temporal-adaptive": "temporal",
        "restir-enhanced-adaptive": "enhanced",
        "restir-enhanced-delayed": "enhanced",
    }
    if not args.pilot:
        cases.extend([
            ("no-sss-reblur", "ray-query", "disabled", "enabled", "reblur", 64, 1, False),
            ("no-sss-relax", "ray-query", "disabled", "enabled", "relax", 64, 1, False),
            ("reference", "ray-query", "mixed", "disabled", "reblur", 256, 8, False),
            ("pt-rtx", "ray-tracing", "mixed", "disabled", "reblur", 64, 1, False),
            ("energy-query", "ray-query", "mixed-energy", "disabled", "reblur", 128, 4, False),
            ("energy-rtx", "ray-tracing", "mixed-energy", "disabled", "reblur", 128, 4, False),
            ("energy-rtx-warm", "ray-tracing", "mixed-energy", "disabled", "reblur", 128, 4, False),
            ("reblur", "ray-query", "mixed", "enabled", "reblur", 64, 1, False),
            ("relax", "ray-query", "mixed", "enabled", "relax", 64, 1, False),
            ("reblur-moving", "ray-query", "mixed", "enabled", "reblur", 64, 1, True),
            ("relax-moving", "ray-query", "mixed", "enabled", "relax", 64, 1, True),
            ("reblur-rotating", "ray-query", "mixed", "enabled", "reblur", 64, 1, 2),
            ("relax-rotating", "ray-query", "mixed", "enabled", "relax", 64, 1, 2),
        ])
        cases.extend((name, "ray-query", "mixed", "disabled", "reblur", 64, 2, False)
                     for name in reuse_integrators)
    if args.case:
        unknown = set(args.case) - {case[0] for case in cases}
        if unknown:
            parser.error(f"Unknown cases: {', '.join(sorted(unknown))}")
        cases = [case for case in cases if case[0] in args.case]
    results = {}
    images = {}
    for name, mode, fixture, denoising, denoiser, frames, spp, moving in cases:
        capture = output / f"{name}.hdr"
        report = output / f"{name}.json"
        capture.unlink(missing_ok=True)
        report.unlink(missing_ok=True)
        command = [
            str(editor), "--demo", "rendering", "--editor", "--capture-demo-preview", str(capture),
            "--preview-width", "1920", "--preview-height", "1080", "--preview-warmup-frames", str(frames),
            "--preview-render-mode", mode, "--preview-sss-fixture", fixture,
            "--preview-camera-position", "0,0,1", "--preview-camera-look-at", "0,0,-1.5",
            "--preview-sample-size", str(spp), "--preview-bounces", "4",
            "--preview-accumulate-samples", "disabled" if moving else "enabled",
            "--preview-nrd-denoising", denoising, "--preview-nrd-denoiser", denoiser,
            "--preview-nrd-temporal-aa", denoising, "--preview-deterministic",
            "--preview-raster-profile-report" if mode == "rasterization" else "--preview-ray-profile-report", str(report),
        ]
        if mode != "rasterization":
            command.extend(["--preview-ray-integrator", reuse_integrators.get(name, "pathtracing")])
        if name.startswith("energy-"):
            command[command.index("--preview-bounces") + 1] = "1"
            command.extend(["--preview-sss-weight", "1", "--preview-firefly-clamp-threshold", "1e30"])
        if name in reuse_integrators:
            command.extend([
                "--preview-restir-temporal-adaptive-cap",
                "enabled" if name.endswith(("-adaptive", "-delayed")) else "disabled",
                "--preview-sss-delayed-reconnection", "enabled" if name.endswith("-delayed") else "disabled",
            ])
        if moving:
            command.extend(["--preview-camera-rotate-each-frame", "0,0.05,0"] if moving == 2 else
                           ["--preview-camera-translate-each-frame", "0.003,0,0"])
        environment = os.environ.copy()
        environment["EVOENGINE_IMGUI_INI_PATH"] = str(output / "imgui.ini")
        environment.setdefault("EVOENGINE_SHADER_CACHE_DIR", str(output / "shader-cache"))
        environment.setdefault("EVOENGINE_PIPELINE_CACHE_DIR", str(output / "pipeline-cache"))
        print(f"Capturing {name}: {frames} frames, {spp} spp/frame", flush=True)
        with (output / f"{name}.stdout.log").open("w") as stdout, (output / f"{name}.stderr.log").open("w") as stderr:
            subprocess.run(command, cwd=editor.parent, env=environment, stdout=stdout, stderr=stderr,
                           timeout=1800, check=True)
        log = "".join((output / f"{name}.{stream}.log").read_text(encoding="utf-8", errors="replace")
                      for stream in ("stdout", "stderr"))
        if any(marker in log for marker in ("[EvoEngine]Error", "VK_ERROR_DEVICE_LOST", "VUID-")):
            raise RuntimeError(f"Rendering errors in {name}; inspect {output / (name + '.stderr.log')}")
        image = read_hdr(capture)
        pixels = np.asarray(image.rgb).reshape(image.height, image.width, 3)
        if (image.width, image.height) != (1920, 1080) or not np.isfinite(pixels).all() or pixels.max() <= 0:
            raise RuntimeError(f"Invalid SSS capture: {capture}")
        if np.max(np.std(pixels, axis=(0, 1))) < 1e-4:
            raise RuntimeError(f"SSS capture has no useful spatial contrast: {capture}")
        images[name] = pixels
        if name.startswith("energy-") and pixels[520:560, 635:670].mean() < 0.05:
            raise RuntimeError(f"Solid SSS direct lighting lost energy: {name}")
        profile = json.loads(report.read_text())
        if mode != "rasterization" and profile["capture"]["rendered_frames"] < frames:
            raise RuntimeError(f"Incomplete SSS capture: {name}")
        if mode != "rasterization":
            variant = profile["variant_readiness"]
            if not variant["ready"] or variant["active_key"] != variant["requested_key"]:
                raise RuntimeError(f"SSS capture did not use the requested shader variant: {name}")
        if name in reuse_integrators:
            integrator = reuse_integrators[name]
            expected_bytes = {"candidate": 96, "temporal": 224, "spatial": 288, "enhanced": 288}[integrator]
            expected_bytes += 4 if name.endswith(("-adaptive", "-delayed")) else 0
            if profile["restir_sss_storage_bytes_per_pixel"] != expected_bytes:
                raise RuntimeError(f"Requested SSS mode was not exercised: {name}")
        if (reuse_integrators.get(name) in ("spatial", "enhanced") and
                not (profile.get("restir_sss_spatial") or {}).get("supported_neighbors", 0)):
            raise RuntimeError(f"SSS reuse was not exercised: {name}")
        results[name] = {"frames": frames, "spp_per_frame": spp, "moving": bool(moving),
                         "camera_motion": "rotation" if moving == 2 else "translation" if moving else "static",
                         "mean": float(pixels.mean()), "maximum": float(pixels.max()),
                         "profile": profile}
        (output / "results.json").write_text(json.dumps(results, indent=2))
    for case in ("energy-rtx", "energy-rtx-warm"):
        if "energy-query" not in images or case not in images:
            continue
        patches = {"block": (520, 560, 635, 670), "sphere": (565, 580, 795, 825),
                   "wedge": (520, 560, 945, 980), "paper": (520, 560, 1095, 1130)}
        ratios = {}
        for name, (y0, y1, x0, x1) in patches.items():
            reference = images["energy-query"][y0:y1, x0:x1].mean(axis=(0, 1))
            measured = images[case][y0:y1, x0:x1].mean(axis=(0, 1))
            ratios[name] = (measured / np.maximum(reference, 1e-8)).tolist()
        results[case]["diffuse_patch_rgb_ratios_to_query"] = ratios
        (output / "results.json").write_text(json.dumps(results, indent=2))
        if any(abs(ratio - 1.0) > 0.03 for channels in ratios.values() for ratio in channels):
            raise RuntimeError(f"SSS direct-light backend energy differs by more than 3% in {case}; inspect results.json")
    if "reference" in images:
        reference = images["reference"][400:680, 200:1720]
        for name in ("raster", "pt-raw", "pt-rtx", "reblur", "relax", *reuse_integrators):
            if name not in images:
                continue
            pixels = images[name][400:680, 200:1720]
            results[name]["static_roi_rmse"] = float(np.sqrt(np.mean((pixels - reference) ** 2)))
            results[name]["static_roi_mean_ratio"] = float(pixels.mean() / max(float(reference.mean()), 1e-8))
        for name in ("no-sss-reblur", "no-sss-relax"):
            if name in images and "no-sss-pt" in images:
                results[name]["mean_ratio_to_raw_control"] = float(images[name].mean() / images["no-sss-pt"].mean())
    (output / "results.json").write_text(json.dumps(results, indent=2))
    print(f"Results: {output / 'results.json'}")


if __name__ == "__main__":
    main()
