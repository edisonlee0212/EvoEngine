#!/usr/bin/env python3
"""Run the versioned soybean study. Raw outputs are immutable between named stages."""

from __future__ import annotations

import argparse
import copy
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUDY = "20261016_robust_v1"
ASSETS = ROOT / "Resources/DigitalAgricultureProject/Assets"
LIBRARY = ASSETS / "GeneratedAssets/Soybean/Experiments" / STUDY


def save_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def timed_command(stage, command, output):
    """Preserve a command log and completion record, including failed runs."""
    receipt = output / "timings" / f"{stage}.json"
    if receipt.exists():
        previous = json.loads(receipt.read_text(encoding="utf-8"))
        waiting_since = time.perf_counter()
        while previous["exit_code"] is None:
            if previous["command"] != command:
                raise RuntimeError(f"{stage} is running with different arguments")
            if time.perf_counter() - waiting_since > 7200:
                raise TimeoutError(f"Timed out waiting for the other {stage} worker; no retry")
            print(f"[study] waiting for the existing {stage} worker", flush=True)
            time.sleep(30)
            previous = json.loads(receipt.read_text(encoding="utf-8"))
        if previous["exit_code"] == 0:
            print(f"[study] already completed: {stage}", flush=True)
            return
        raise RuntimeError(f"Previous {stage} failed; inspect its log before selecting a new study directory")
    if stage.startswith("generate_") and "--output-dir" in command:
        folder = Path(command[command.index("--output-dir") + 1])
        if folder.exists() and any(folder.glob("*.obj")):
            raise RuntimeError(f"Refusing to overwrite an existing model library: {folder}")
    started = time.perf_counter()
    record = {"stage": stage, "command": command, "working_directory": str(ROOT),
              "started_at_utc": datetime.now(timezone.utc).isoformat(), "timeout_seconds": 7200}
    log = output / "logs" / f"{stage}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ)
    environment["Path"] = environment.pop("PATH", environment.get("Path", ""))
    environment["MSBUILDDISABLENODEREUSE"] = "1"
    environment["OMP_NUM_THREADS"] = "4"
    print(f"[study] starting {stage}: {subprocess.list2cmdline(command)}", flush=True)
    with log.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=stream, stderr=subprocess.STDOUT)
        record["pid"] = process.pid
        save_json(receipt, {**record, "exit_code": None})
        # Bounded waits provide a live heartbeat without retrying a failed experiment.
        while process.poll() is None:
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                print(f"[study] {stage}: {time.perf_counter() - started:.0f}s, log {log.stat().st_size} bytes", flush=True)
                if time.perf_counter() - started > record["timeout_seconds"]:
                    print(f"[study] hard timeout for {stage}; terminating PID {process.pid}, no retry", flush=True)
                    process.terminate()
                    process.wait(timeout=30)
    record.update(completed_at_utc=datetime.now(timezone.utc).isoformat(),
                  elapsed_seconds=time.perf_counter() - started, exit_code=process.returncode)
    save_json(receipt, record)
    if process.returncode:
        raise RuntimeError(f"{stage} failed ({process.returncode}); see {log}")
    print(f"[study] completed {stage} in {record['elapsed_seconds']:.1f}s", flush=True)


def link_library(source, destination, entries):
    """Disjoint library views share immutable files, rather than duplicating gigabytes."""
    destination.mkdir(parents=True, exist_ok=True)
    names = {entry["obj"] for entry in entries} | {entry["mtl"] for entry in entries}
    names |= {Path(entry["obj"]).with_suffix(".geometry.json").name for entry in entries}
    names |= {path.name for path in source.iterdir() if path.suffix.lower() in {".png", ".jpg", ".jpeg"}}
    for name in names:
        if not (destination / name).exists():
            os.link(source / name, destination / name)
    manifest = json.loads((source / "manifest.json").read_text(encoding="utf-8"))
    manifest["models"] = entries
    manifest["source_manifest"] = str(source / "manifest.json")
    save_json(destination / "manifest.json", manifest)


def make_config(base, split="Training", rows=8, columns=20, roi=0.5, samples=64):
    config = copy.deepcopy(base)
    config["project"] = str(ROOT / "Resources/DigitalAgricultureProject/DigitalAgricultureProject.eveproj")
    config.pop("helios_generation", None)
    for asset in config["assets"]:
        asset["folder"] = (LIBRARY / split / asset["name"].capitalize()).relative_to(ASSETS).as_posix()
    config["search"].update(rows=[rows], columns=[columns], roi=[roi], seed=[101, 202, 303, 404, 505])
    config["ray_tracing"].update(samples=samples, bounces=2)
    return config


def run_search(name, config, output, chunk_by_asset=False):
    config_path = output / "configs" / f"{name}.json"
    save_json(config_path, config)
    if chunk_by_asset:
        from search_prefab_field_illumination import build_cases
        cases = build_cases(config)
        parts = []
        for asset in config["assets"]:
            chunk = copy.deepcopy(config)
            chunk.update(run_ids=[index for index, case in enumerate(cases) if case["asset_name"] == asset["name"]],
                         include_in_study_analysis=False)
            chunk_name = f"{name}_{asset['name']}"
            run_search(chunk_name, chunk, output)
            parts.append(f"raw/{chunk_name}")
        timed_command(f"assemble_{name}", [sys.executable, str(ROOT / "Scripts/resume_soybean_study.py"),
                      "--assemble-only", "--study", str(output), "--name", name, "--parts", *parts], output)
        return
    timed_command(name, [sys.executable, str(ROOT / "Scripts/search_prefab_field_illumination.py"),
                        "--module-dir", str(ROOT / "out/install/vs2026-x64/python"),
                        "--config", str(config_path), "--output-dir", str(output / "raw" / name)], output)


def generation_command(kind, count, folder):
    ratio = "1.0" if kind == "Broad" else "0.3"
    return [sys.executable, str(ROOT / "Scripts/export_helios_soybeans.py"),
            "--exporter", str(ROOT / "out/helios-soybean-exporter/build/helios_soybean_exporter.exe"),
            "--output-dir", str(folder), "--count", str(count), "--seed-start", "2100000",
            "--age-days", "45", "--max-nodes", "25", "--internode-length-m", "0.035",
            "--leaf-size-range-m", "0.10", "0.14", "--leaf-aspect-ratio-range", ratio, ratio,
            "--petiole-length-range-m", "0.12", "0.16", "--leaf-subdivisions", "8"]


def correct_manifest_provenance(folder):
    """Helios' embedded git label uses the including project's cwd, not HELIOS_ROOT."""
    import hashlib
    target = folder / "manifest.json"
    manifest = json.loads(target.read_text(encoding="utf-8"))
    if "exporter_reported_version" in manifest:
        return
    shutil.copy2(target, folder / "manifest.original-version-label.json")
    manifest["exporter_reported_version"] = manifest["helios_commit"]
    manifest["helios_commit"] = subprocess.run(["git", "-C", str(ROOT.parent / "Helios"), "rev-parse", "HEAD"],
                                                capture_output=True, text=True, check=True).stdout.strip()
    manifest["exporter_sha256"] = hashlib.sha256((ROOT / "out/helios-soybean-exporter/build/helios_soybean_exporter.exe").read_bytes()).hexdigest()
    manifest["provenance_correction"] = "Embedded version was the bridge repository commit. Original label is preserved separately. Source path and commit verified before measurements."
    save_json(target, manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "out/soybean-experiments" / STUDY)
    parser.add_argument("--stage", choices=["build", "library", "finalize-library", "preview", "single", "convergence", "field", "validation"], required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    base = json.loads((ROOT / "Scripts/prefab_field_illumination_search.example.json").read_text(encoding="utf-8"))
    if args.stage == "build":
        timed_command("build_engine", ["cmake", "--build", "out/build/vs2026-x64", "--config", "RelWithDebInfo",
                                      "--parallel", "4", "--target", "EvoEngine_Tests", "--", "/nr:false"], output)
        timed_command("build_app", ["cmake", "--build", "out/build/vs2026-x64", "--config", "RelWithDebInfo",
                                   "--target", "DigitalAgricultureApp", "--", "/nr:false", "/p:BuildProjectReferences=false"], output)
        timed_command("test_engine", ["ctest", "--test-dir", "out/build/vs2026-x64", "-C", "RelWithDebInfo",
                                     "-R", "^(PrefabField[.]|SerializationRegistry[.]PrefabFieldRoi)", "--no-tests=error", "--output-on-failure"], output)
        timed_command("install_engine", ["cmake", "--install", "out/build/vs2026-x64", "--config", "RelWithDebInfo"], output)
        timed_command("build_exporter", ["cmake", "--build", "out/helios-soybean-exporter/build", "--config", "Release", "--parallel", "1"], output)
        timed_command("install_exporter", ["cmake", "--install", "out/helios-soybean-exporter/build", "--config", "Release",
                                          "--prefix", str(output / "tools/helios"), "--component", "HeliosSoybeanExporter"], output)
        snapshots = output / "provenance"
        snapshots.mkdir(exist_ok=True)
        for name, repo in [("engine", ROOT), ("helios", ROOT.parent / "Helios")]:
            for label, command in [("commit", ["rev-parse", "HEAD"]), ("status", ["status", "--short"]), ("diff", ["diff", "--binary"])]:
                result = subprocess.run(["git", "-C", str(repo), *command], capture_output=True, check=True)
                (snapshots / f"{name}_{label}.txt").write_bytes(result.stdout)
        for source in [ROOT / "Tools/HeliosSoybeanExporter/main.cpp", ROOT.parent / "Helios/plugins/plantarchitecture/src/PlantLibrary.cpp",
                       ROOT / "EvoEngine_Services/CudaModule/src/ptx/IlluminationEstimation.cu"]:
            shutil.copy2(source, snapshots / source.name)
        for name in ["Scripts/run_soybean_experiments.py", "Scripts/export_helios_soybeans.py", "Scripts/search_prefab_field_illumination.py",
                     "Scripts/analyze_soybean_experiments.py", "Scripts/describe_soybean_library.py", "Scripts/summarize_soybean_study.py",
                     "Scripts/continue_soybean_study.py", "Scripts/execute_soybean_notebook.py", "Scripts/capture_soybean_study.py",
                     "Scripts/resume_soybean_study.py", "Scripts/tests/test_cuda_image_lifetime.py",
                     "Scripts/tests/test_soybean_recovery.py",
                     "Scripts/tests/test_prefab_field_illumination_search.py", "Scripts/tests/test_soybean_analysis.py",
                     "EvoEngine_Services/CudaModule/src/VulkanInterlop.cpp",
                     "EvoEngine_Services/CudaModule/include/RayTracer/VulkanInterlop.hpp",
                     "Scripts/soybean_experiment_analysis.ipynb", "EvoEngine_SDK/include/Utilities/PrefabField.hpp",
                     "EvoEngine_SDK/include/Utilities/SurfaceIlluminationResult.hpp",
                     "EvoEngine_SDK/src/PrefabField.cpp", "EvoEngine_Services/CudaModule/src/PrefabFieldIllumination.cpp",
                     "EvoEngine_Services/CudaModule/include/Utilities/PrefabFieldIllumination.hpp",
                     "EvoEngine_Services/CudaModule/src/TriangleIlluminationEstimator.cpp",
                     "EvoEngine_Services/CudaModule/include/Utilities/TriangleIlluminationEstimator.hpp",
                     "EvoEngine_Services/CudaModule/include/RayTracer/RayFunctions.cuh",
                     "EvoEngine_Services/CudaModule/include/RayTracer/RayTracerUtilities.cuh",
                     "Tools/HeliosSoybeanExporter/CMakeLists.txt", "PythonBinding/include/PyEvoEngine.hpp",
                     "PythonBinding/src/PyEvoEngine.cpp"]:
            destination = snapshots / "source" / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / name, destination)
        builder = output / ".deck-build/build_deck.mjs"
        if builder.is_file():
            shutil.copy2(builder, snapshots / "presentation_builder.mjs")
        return
    if args.stage in {"library", "finalize-library"}:
        for kind in ["Broad", "Narrow"]:
            if args.stage == "library":
                timed_command(f"generate_{kind.lower()}", generation_command(kind, 1024, LIBRARY / "Library" / kind), output)
            source = LIBRARY / "Library" / kind
            correct_manifest_provenance(source)
            entries = json.loads((source / "manifest.json").read_text(encoding="utf-8"))["models"]
            link_library(source, LIBRARY / "Training" / kind, entries[:768])
            link_library(source, LIBRARY / "Holdout" / kind, entries[768:])
        save_json(output / "library_split.json", {"root": str(LIBRARY), "per_type": 1024,
                  "training": [2100000, 2100767], "holdout": [2100768, 2101023]})
        return
    if args.stage == "preview":
        for kind in ["Broad", "Narrow"]:
            timed_command(f"generate_preview_{kind.lower()}", generation_command(kind, 1, LIBRARY / "Representative" / kind), output)
        config = make_config(base, split="Representative", rows=1, columns=1, roi=1, samples=256)
        config["search"].update(seed=[101], row_spacing_inches=[20], column_spacing_inches=[3])
        config.update(capture_run_ids=[0, 1], capture_position=[1.05, .8, 1.15], capture_target=[0, .45, 0],
                      save_scene_prefix=f"GeneratedAssets/Scenes/{STUDY}_representative")
        run_search("00_representative_pair", config, output)
        return
    if args.stage == "single":
        config = make_config(base, rows=1, columns=1, roi=1, samples=256)
        config["search"].update(seed=list(range(101, 133)), row_spacing_inches=[20], column_spacing_inches=[3])
        config.update(capture_run_ids=[0, 32], capture_position=[1.05, 0.8, 1.15], capture_target=[0, 0.45, 0],
                      save_scene_prefix=f"GeneratedAssets/Scenes/{STUDY}_single")
        run_search("01_single_plant", config, output)
    elif args.stage == "convergence":
        config = make_config(base)
        config["search"].update(seed=[101], row_spacing_inches=[10, 22.5, 35], column_spacing_inches=[1, 3, 5], samples=[16, 64, 256])
        run_search("02_convergence", config, output)
    elif args.stage == "field":
        import csv
        with (output / "analysis/sample_convergence.csv").open(newline="", encoding="utf-8") as stream:
            convergence = list(csv.DictReader(stream))
        errors = {samples: max(abs(float(row["relative_difference_from_256"])) for row in convergence
                               if int(row["samples"]) == samples) for samples in [16, 64]}
        samples = next((value for value in [16, 64] if errors[value] <= 0.01), 256)
        save_json(output / "sampling_decision.json", {"criterion": "maximum relative difference <= 1% across all convergence cases and three metrics",
                  "observed_max_errors": errors, "training_samples": samples, "holdout_samples": 256})
        config = make_config(base, samples=samples)
        config.update(capture_run_ids=[0, 245, 490, 495, 740, 985],
                      save_scene_prefix=f"GeneratedAssets/Scenes/{STUDY}_field")
        run_search("03_field_search", config, output)
    elif args.stage == "validation":
        # Candidate spacings come from training only. Holdout seeds never occur in the training library.
        import csv
        with (output / "analysis/03_field_search/ranked_combinations.csv").open(newline="", encoding="utf-8") as stream:
            ranked = list(csv.DictReader(stream))
        candidates = set()
        for metric in ["leaf_mean_irradiance", "leaf_incident_per_plant", "leaf_incident_per_ground_area"]:
            for kind in ["broad", "narrow"]:
                rows = sorted((row for row in ranked if row["asset_name"] == kind), key=lambda row: float(row[metric]), reverse=True)
                candidates.update((float(row["row_spacing_inches"]), float(row["column_spacing_inches"])) for row in rows[:3])
        timed_command("capture_field_figures", [sys.executable, str(ROOT / "Scripts/capture_soybean_study.py"),
                                              "--study", str(output)], output)
        config = make_config(base, split="Holdout", samples=256)
        config["search"].pop("row_spacing_inches")
        config["search"].pop("column_spacing_inches")
        config["search"].update(seed=[606, 707, 808, 909, 1010], spacing_pairs_inches=sorted(candidates))
        run_search("04_holdout", config, output)
        # Buffer sensitivity uses a larger training field and the same central cell count, not matched surrounding plants.
        baseline = make_config(base, samples=256)
        baseline["search"].update(seed=[101, 202, 303], row_spacing_inches=[10, 35], column_spacing_inches=[1, 5])
        run_search("05_buffer_baseline", baseline, output)
        config = make_config(base, rows=12, columns=30, roi=1 / 3, samples=256)
        config["search"].update(seed=[101, 202, 303], row_spacing_inches=[10, 35], column_spacing_inches=[1, 5])
        run_search("05_buffer_sensitivity", config, output, chunk_by_asset=True)
        samples = json.loads((output / "sampling_decision.json").read_text(encoding="utf-8"))["training_samples"]
        config = make_config(base, roi=1, samples=samples)
        config["search"].update(seed=[101], row_spacing_inches=[10, 35], column_spacing_inches=[1, 5])
        run_search("06_edge_control", config, output)


if __name__ == "__main__":
    main()
