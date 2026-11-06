#!/usr/bin/env python3
"""Continue once the parallel libraries finish, without regenerating or retrying experiments."""
import json
import sys
import time
from pathlib import Path

from run_soybean_experiments import LIBRARY, ROOT, timed_command

STUDY = ROOT / "out/soybean-experiments/20261016_robust_v1"
PYTHON = r"C:/Users/sdjkl/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe"
SCIENCE_PYTHON = r"C:/Users/sdjkl/miniconda3/envs/demeter/python.exe"
NODE = r"C:/Users/sdjkl/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe"


def stage(name, command):
    timed_command(name, command, STUDY)


def analyze(label):
    stage(label, [PYTHON, str(ROOT / "Scripts/analyze_soybean_experiments.py"), "--study", str(STUDY), "--no-plots"])


def finish_study():
    """Shared post-training stages also serve an explicitly authorized continuation."""
    stage("batch_validation", [sys.executable, str(ROOT / "Scripts/run_soybean_experiments.py"), "--stage", "validation"])
    analyze("analyze_after_validation")
    stage("execute_notebook", [SCIENCE_PYTHON, str(ROOT / "Scripts/execute_soybean_notebook.py"), "--study", str(STUDY)])
    stage("archive_sources", [sys.executable, str(ROOT / "Scripts/run_soybean_experiments.py"), "--stage", "build"])
    analyze("analyze_final")
    stage("slides", [NODE, str(STUDY / ".deck-build/build_deck.mjs")])
    stage("refresh_final_analysis", [PYTHON, str(ROOT / "Scripts/analyze_soybean_experiments.py"), "--study", str(STUDY), "--no-plots"])
    stage("html_report", [PYTHON, str(ROOT / "Scripts/summarize_soybean_study.py"), "--study", str(STUDY)])
    print("[continue] study measurements, analysis, notebook, report and initial slides completed", flush=True)


def main():
    started = time.monotonic()
    while True:
        ready = True
        for name in ["generate_broad", "generate_narrow"]:
            receipt = json.loads((STUDY / "timings" / f"{name}.json").read_text(encoding="utf-8"))
            if receipt["exit_code"] not in [None, 0]:
                raise RuntimeError(f"{name} failed. No retry.")
            ready &= receipt["exit_code"] == 0
        if ready:
            break
        if time.monotonic() - started > 7200:
            raise TimeoutError("Library wait timed out. No generators stopped or retried.")
        print("[continue] waiting for both immutable libraries", flush=True)
        time.sleep(30)
    stage("finalize_libraries", [sys.executable, str(ROOT / "Scripts/run_soybean_experiments.py"), "--stage", "finalize-library"])
    stage("describe_library", [PYTHON, str(ROOT / "Scripts/describe_soybean_library.py"), "--library", str(LIBRARY / "Library"),
                               "--output", str(STUDY / "analysis/geometry_library")])
    for name in ["single", "convergence", "field"]:
        stage(f"batch_{name}", [sys.executable, str(ROOT / "Scripts/run_soybean_experiments.py"), "--stage", name])
        analyze(f"analyze_after_{name}")
    finish_study()


if __name__ == "__main__":
    main()
