"""Repeat only experiments retained in the current deck, changing the sky and nothing in the scene.

The existing Helios library and original experiment configs remain read-only.
Each field subset runs in a separate process to bound GPU resource lifetime.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys

from run_soybean_experiments import ROOT, run_search, save_json, timed_command
from search_prefab_field_illumination import build_cases

SOURCE = ROOT / "out/soybean-experiments/20261016_robust_v1"
STUDY = ROOT / "out/soybean-experiments/20261017_G1706994_HDR"
HDR = ROOT / "out/gopro-sky/G1706994/G1706994_latlong.hdr"
ANALYSIS_PYTHON = Path("C:/Users/sdjkl/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe")


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def hdr_config(name):
    config = json.loads((SOURCE / "configs" / f"{name}.json").read_text(encoding="utf-8"))
    original_cases = build_cases(config)
    config["lighting"].update(type="hdr", hdr_path=str(HDR), skylight_intensity=2.0,
                              rotation_degrees=0.0, sun_intensity=0.0, ambient_light_intensity=0.0)
    config["capture_run_ids"] = []
    config.pop("save_scene_prefix", None)
    config["output_directory"] = str(STUDY / "raw" / name)
    assert build_cases(config) == original_cases, "Sky replacement must not change any field case"
    return config


def prepare():
    configs = {name: hdr_config(name) for name in ["01_single_plant", "03_field_search", "06_edge_control"]}
    configs["03_field_search"]["capture_run_ids"] = [245, 740]
    for name, config in configs.items():
        save_json(STUDY / "configs" / f"{name}.json", config)
    inputs = [SOURCE / "presentation/soybean_experiments_initial_v1_en.pptx", HDR,
              ROOT / "Resources/DigitalAgricultureProject/Assets" / configs["03_field_search"]["base_scene"],
              ROOT / "out/install/vs2026-x64/python/PyEvoEngine.cp312-win_amd64.pyd",
              ROOT / "out/install/vs2026-x64/bin/DigitalAgricultureApp.exe"]
    for asset in configs["03_field_search"]["assets"]:
        inputs.append(ROOT / "Resources/DigitalAgricultureProject/Assets" / asset["folder"] / "manifest.json")
    save_json(STUDY / "provenance/inputs.json", {str(p): digest(p) for p in inputs})
    for name in ["library_split.json"]:
        shutil.copy2(SOURCE / name, STUDY / name)
    for name in ["geometry_library", "geometry_representative"]:
        shutil.copytree(SOURCE / "analysis" / name, STUDY / "analysis" / name, dirs_exist_ok=True)
    for name in ["search_prefab_field_illumination.py", "run_soybean_hdr_experiments.py", "analyze_soybean_experiments.py", "capture_soybean_study.py"]:
        shutil.copy2(ROOT / "Scripts" / name, STUDY / "provenance" / name)
    save_json(STUDY / "protocol.json", {
        "source_study": str(SOURCE), "source_slides": 22, "library_regenerated": False,
        "same_scene": "Same base scene, soil, meshes/materials, subset seeds, spacing, ROI, transforms and cameras",
        "changed": "OptiX environment: original Skydome to G1706994 Scene HDR, user-selected multiplier 2",
        "lighting": configs["03_field_search"]["lighting"],
        "experiments": {name: len(build_cases(config)) for name, config in configs.items()},
        "omitted_deleted_slides": ["holdout validation", "field-size sensitivity"],
        "sampling": "Original settings: isolated 256, main/edge 16 samples per face, 2 bounces",
        "units": "Relative RGB light, not calibrated irradiance/PAR",
        "monitoring": "Per-case logs and timings, process alive, 7200-second hard timeout per subprocess, no crash auto-retry",
    })


def run_stage(name):
    config = json.loads((STUDY / "configs" / f"{name}.json").read_text())
    if name in {"03_field_search", "06_edge_control"}:
        # Keep original run IDs and subset seeds; only split process lifetime, not experimental cases.
        parts = []
        cases = build_cases(config)
        for asset in config["assets"]:
            for seed in config["search"]["seed"]:
                chunk = copy.deepcopy(config)
                label = f"{name}_{asset['name']}_{seed}"
                chunk.update(run_ids=[i for i, c in enumerate(cases) if c["asset_name"] == asset["name"] and c["seed"] == seed],
                             include_in_study_analysis=False)
                chunk["capture_run_ids"] = [i for i in chunk["run_ids"] if name == "03_field_search" and cases[i]["row_spacing_inches"] == 22.5
                                             and cases[i]["column_spacing_inches"] == 3 and seed == 101]
                chunk["save_scene_prefix"] = "GeneratedAssets/Scenes/20261017_G1706994_HDR_field"
                run_search(label, chunk, STUDY)
                parts.append(f"raw/{label}")
        timed_command("assemble_" + name, [sys.executable, str(ROOT / "Scripts/resume_soybean_study.py"),
                      "--assemble-only", "--study", str(STUDY), "--name", name, "--parts", *parts], STUDY)
    else:
        if name == "01_single_plant":
            config.update(capture_run_ids=[0, 32], capture_position=[1.05, .8, 1.15], capture_target=[0, .45, 0],
                          save_scene_prefix="GeneratedAssets/Scenes/20261017_G1706994_HDR_single")
        run_search(name, config, STUDY)


def pilot():
    config = hdr_config("03_field_search")
    config["search"].update(spacing_pairs_inches=[[10, 1], [35, 5]], seed=[101], samples=[16, 256])
    config.update(include_in_study_analysis=False)
    run_search("00_hdr_pilot", config, STUDY)


def smoke():
    config = hdr_config("01_single_plant")
    config["search"]["seed"] = [101]
    config.update(include_in_study_analysis=False)
    run_search("00_hdr_smoke_intensity2", config, STUDY)
    check = copy.deepcopy(config)
    check["lighting"]["skylight_intensity"] = 1.0
    run_search("00_hdr_smoke_intensity1", check, STUDY)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=["prepare", "smoke", "pilot", "single", "field", "edge", "capture", "analyze"])
    stage = parser.parse_args().stage
    if stage == "prepare":
        prepare()
    elif stage == "smoke":
        smoke()
    elif stage == "pilot":
        pilot()
    elif stage == "capture":
        for run_id in [245, 740]:
            timed_command(f"capture_field_{run_id}", [sys.executable, str(ROOT / "Scripts/capture_soybean_study.py"),
                          "--study", str(STUDY), "--run-ids", str(run_id), "--sky-view"], STUDY)
    elif stage == "analyze":
        timed_command("analyze", [str(ANALYSIS_PYTHON), str(ROOT / "Scripts/analyze_soybean_experiments.py"),
                      "--study", str(STUDY), "--no-plots"], STUDY)
    else:
        run_stage({"single": "01_single_plant", "field": "03_field_search", "edge": "06_edge_control"}[stage])


if __name__ == "__main__":
    main()
