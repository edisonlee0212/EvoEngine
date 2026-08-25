#!/usr/bin/env python3
"""Capture deterministic SDK shader inventory, reflection, and SPIR-V baselines."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import statistics
import subprocess
import sys
import time


SHADER_EXTENSIONS = {".slang", ".slangh"}
STAGE_PARTS = {
    "Compute": "compute",
    "Vertex": "vertex",
    "Fragment": "fragment",
    "Mesh": "mesh",
    "Task": "task",
    "RayGen": "raygen",
    "Miss": "miss",
    "ClosestHit": "closest_hit",
    "AnyHit": "any_hit",
}
INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"]([^">]+)[">]', re.MULTILINE)
IMPORT_RE = re.compile(r"^\s*import\s+([A-Za-z_][A-Za-z0-9_.]*)\s*;", re.MULTILINE)
MACRO_RE = re.compile(r"^\s*#\s*(define|if|ifdef|ifndef|elif|else|endif)\b", re.MULTILINE)
PREPROCESSOR_RE = re.compile(r"^\s*#\s*[A-Za-z_]", re.MULTILINE)
STATS_PREFIX = "EVOENGINE_SHADER_BASELINE_STATS "


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="RelWithDebInfo", choices=("Debug", "RelWithDebInfo"))
    parser.add_argument("--build-dir", type=Path, default=root / "out" / "build" / "vs2026-x64-tests")
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "EvoEngine_Tests" / "ShaderPolicy" / "Baselines" / "sdk-shader-baseline.json",
    )
    parser.add_argument("--artifact-dir", type=Path, default=root / "out" / "shader-inventory")
    parser.add_argument("--scope", default="EvoEngine_SDK current native Slang shader baseline")
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--skip-compile", action="store_true")
    parser.add_argument("--check", action="store_true", help="Fail if regenerated deterministic data differs.")
    return parser.parse_args()


def resolve(root: Path, path: Path) -> Path:
    return path.resolve() if path.is_absolute() else (root / path).resolve()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def canonical_bytes(value: object) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")


def strip_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", source)


def infer_stage(relative: Path) -> str | None:
    parts = relative.parts
    for part, stage in STAGE_PARTS.items():
        if part in parts:
            return stage
    return None


def ensure_under(path: Path, parent: Path) -> None:
    try:
        path.relative_to(parent)
    except ValueError as error:
        raise RuntimeError(f"Refusing to manage path outside {parent}: {path}") from error


def reset_directory(path: Path, allowed_parent: Path) -> None:
    ensure_under(path, allowed_parent)
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True)


def resolve_dependency(
    source_path: Path,
    name: str,
    shader_root: Path,
    by_name: dict[str, list[Path]],
) -> str:
    local = (source_path.parent / name).resolve()
    if local.is_file():
        return local.relative_to(shader_root).as_posix()
    include_path = (shader_root / "Includes" / name).resolve()
    if include_path.is_file():
        return include_path.relative_to(shader_root).as_posix()
    matches = by_name.get(Path(name).name, [])
    if len(matches) == 1:
        return matches[0].relative_to(shader_root).as_posix()
    return f"unresolved:{name}" if not matches else f"ambiguous:{name}"


def collect_source_inventory(root: Path) -> dict[str, object]:
    shader_root = root / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders"
    paths = sorted(path for path in shader_root.rglob("*") if path.is_file() and path.suffix in SHADER_EXTENSIONS)
    by_name: dict[str, list[Path]] = defaultdict(list)
    for path in paths:
        by_name[path.name].append(path)

    files: list[dict[str, object]] = []
    counts: Counter[str] = Counter()
    graph: dict[str, dict[str, list[str]]] = {}
    for path in paths:
        relative = path.relative_to(shader_root)
        data = path.read_bytes()
        source = data.decode("utf-8", errors="replace")
        uncommented = strip_comments(source)
        includes = INCLUDE_RE.findall(uncommented)
        imports = IMPORT_RE.findall(uncommented)
        macros = MACRO_RE.findall(uncommented)
        preprocessor_directives = PREPROCESSOR_RE.findall(uncommented)
        stage = infer_stage(relative)
        cone = relative.parts[0]
        counts["files"] += 1
        counts[f"extension:{path.suffix}"] += 1
        counts[f"cone:{cone}"] += 1
        if stage:
            counts["entry_points"] += 1
            counts[f"stage:{stage}"] += 1
        counts["includes"] += len(includes)
        counts["imports"] += len(imports)
        counts["macro_controls"] += len(macros)
        counts["preprocessor_directives"] += len(preprocessor_directives)
        if macros:
            counts["files_with_macro_controls"] += 1
        if preprocessor_directives:
            counts["files_with_preprocessor_directives"] += 1
        relative_text = relative.as_posix()
        graph[relative_text] = {
            "includes": [resolve_dependency(path, name, shader_root, by_name) for name in includes],
            "imports": sorted(imports),
        }
        files.append(
            {
                "path": relative_text,
                "extension": path.suffix,
                "stage": stage,
                "cone": cone,
                "bytes": len(data),
                "sha256": sha256_bytes(data),
                "include_count": len(includes),
                "import_count": len(imports),
                "macro_control_count": len(macros),
                "preprocessor_directive_count": len(preprocessor_directives),
            }
        )

    count_output = {key: counts[key] for key in sorted(counts)}
    return {
        "root": "EvoEngine_SDK/Internals/DefaultResources/Shaders",
        "counts": count_output,
        "aggregate_source_sha256": sha256_bytes(canonical_bytes(files)),
        "files": files,
        "dependency_graph": {key: graph[key] for key in sorted(graph)},
    }


def run_command(command: list[str], cwd: Path, log: Path, env: dict[str, str] | None = None) -> tuple[float, str]:
    log.parent.mkdir(parents=True, exist_ok=True)
    run_env = os.environ.copy()
    if env:
        run_env.update(env)
    start = time.perf_counter()
    completed = subprocess.run(
        command,
        cwd=cwd,
        env=run_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    elapsed = time.perf_counter() - start
    log.write_text(completed.stdout or "", encoding="utf-8", newline="")
    if completed.returncode != 0:
        print("\n".join((completed.stdout or "").splitlines()[-120:]), file=sys.stderr)
        raise RuntimeError(f"Command failed with exit code {completed.returncode}; log={log}")
    return elapsed, completed.stdout or ""


def parse_stats(output: str) -> dict[str, int]:
    for line in reversed(output.splitlines()):
        if line.startswith(STATS_PREFIX):
            return json.loads(line[len(STATS_PREFIX) :])
    raise RuntimeError("Shader baseline test did not print compile-cache statistics.")


def test_executable(build_dir: Path, config: str) -> Path:
    suffix = ".exe" if os.name == "nt" else ""
    candidates = (
        build_dir / config / "bin" / f"EvoEngine_Tests{suffix}",
        build_dir / "EvoEngine_Tests" / config / f"EvoEngine_Tests{suffix}",
        build_dir / config / f"EvoEngine_Tests{suffix}",
    )
    return next((candidate for candidate in candidates if candidate.is_file()), candidates[0])


def test_command(executable: Path) -> list[str]:
    return [str(executable), "--gtest_filter=ShaderBaseline.ProductionSdkEntryPointInventoryCompilesAndReflects"]


def run_inventory_test(
    executable: Path,
    artifact_dir: Path,
    cache_dir: Path,
    label: str,
    capture_dir: Path | None = None,
) -> dict[str, object]:
    env = {"EVOENGINE_SHADER_BASELINE_CACHE_DIR": str(cache_dir)}
    if capture_dir:
        env["EVOENGINE_SHADER_BASELINE_CAPTURE_DIR"] = str(capture_dir)
    command = test_command(executable)
    elapsed, output = run_command(command, executable.parent, artifact_dir / "logs" / f"{label}.log", env)
    return {"label": label, "command": command, "elapsed_seconds": elapsed, "stats": parse_stats(output)}


def compiler_version(executable: Path, artifact_dir: Path) -> str:
    name = "slangc.exe" if os.name == "nt" else "slangc"
    candidates = [executable.parent / name]
    if len(executable.parents) >= 3:
        candidates.append(executable.parents[2] / executable.parent.name / "bin" / name)
    slangc = next((candidate for candidate in candidates if candidate.is_file()), None)
    if not slangc:
        return "unavailable"
    _, output = run_command([str(slangc), "-version"], executable.parent, artifact_dir / "logs" / "slang-version.log")
    return output.strip()


def collect_gpu_info() -> list[dict[str, str]]:
    if os.name != "nt":
        return []
    command = [
        "powershell",
        "-NoProfile",
        "-Command",
        "Get-CimInstance Win32_VideoController | Select-Object Name,DriverVersion | ConvertTo-Json -Compress",
    ]
    completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, check=False)
    if completed.returncode != 0 or not completed.stdout.strip():
        return []
    value = json.loads(completed.stdout)
    values = value if isinstance(value, list) else [value]
    return [{"name": item.get("Name", ""), "driver": item.get("DriverVersion", "")} for item in values]


def git_revision(root: Path) -> str:
    completed = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=root, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, check=False
    )
    return completed.stdout.strip() if completed.returncode == 0 else "unavailable"


def collect_compiled_manifest(capture_dir: Path, compiler: str) -> dict[str, object]:
    shaders = []
    total_bytes = 0
    for record_path in sorted((capture_dir / "records").glob("*.json")):
        record = json.loads(record_path.read_text(encoding="utf-8"))
        spirv_path = capture_dir / record.pop("spirv")
        spirv = spirv_path.read_bytes()
        reflection = record.pop("reflection")
        variant_defines = record.pop("variant_defines")
        total_bytes += len(spirv)
        shaders.append(
            {
                **record,
                "variant_defines_sha256": sha256_bytes(variant_defines.encode("utf-8")),
                "spirv_bytes": len(spirv),
                "spirv_sha256": sha256_bytes(spirv),
                "reflection_sha256": sha256_bytes(canonical_bytes(reflection)),
                "reflection_counts": {
                    "descriptor_bindings": len(reflection["descriptor_bindings"]),
                    "push_constant_ranges": len(reflection["push_constant_ranges"]),
                    "stage_inputs": len(reflection["stage_inputs"]),
                    "stage_outputs": len(reflection["stage_outputs"]),
                },
                "compute_thread_group_size": reflection["compute_thread_group_size"],
            }
        )
    if not shaders:
        raise RuntimeError(f"No captured shader records found under {capture_dir}")
    return {
        "compiler": compiler,
        "target": "Vulkan 1.3 / SPIR-V 1.4 / row-major / scalar layout",
        "entry_point_count": len(shaders),
        "aggregate_spirv_bytes": total_bytes,
        "aggregate_manifest_sha256": sha256_bytes(canonical_bytes(shaders)),
        "shaders": shaders,
    }


def collect_compile_evidence(
    root: Path, build_dir: Path, config: str, artifact_dir: Path, runs: int, skip_build: bool
) -> tuple[dict[str, object], dict[str, object]]:
    if runs < 3:
        raise RuntimeError("--runs must be at least 3 for baseline capture.")
    if not skip_build:
        run_command(
            ["cmake", "--build", str(build_dir), "--config", config, "--target", "EvoEngine_Tests", "--", "/m:1"],
            root,
            artifact_dir / "logs" / "build.log",
        )
    executable = test_executable(build_dir, config)
    if not executable.is_file():
        raise RuntimeError(f"EvoEngine_Tests executable not found: {executable}")

    cache_root = artifact_dir / "cache"
    reset_directory(cache_root, root / "out")
    cold_runs = []
    for index in range(runs):
        cache_dir = cache_root / f"cold-{index}"
        cache_dir.mkdir(parents=True)
        cold_runs.append(run_inventory_test(executable, artifact_dir, cache_dir, f"cold-{index}"))

    warm_cache = cache_root / "warm"
    warm_cache.mkdir(parents=True)
    prime = run_inventory_test(executable, artifact_dir, warm_cache, "warm-prime")
    warm_runs = [
        run_inventory_test(executable, artifact_dir, warm_cache, f"warm-{index}") for index in range(runs)
    ]

    capture_dir = artifact_dir / "capture"
    reset_directory(capture_dir, root / "out")
    capture = run_inventory_test(executable, artifact_dir, warm_cache, "manifest-capture", capture_dir)
    slang_version = compiler_version(executable, artifact_dir)
    deterministic = collect_compiled_manifest(capture_dir, slang_version)
    timing = {
        "schema": 1,
        "platform": {
            "vulkan_backend": True,
            "os": platform.platform(),
            "python": sys.version,
            "gpu": collect_gpu_info(),
            "build_config": config,
            "slang_compiler": slang_version,
            "evoengine_revision": git_revision(root),
        },
        "cache_policy": "isolated cold cache per run; one primed persistent cache for warm runs",
        "cold_runs": cold_runs,
        "warm_prime": prime,
        "warm_runs": warm_runs,
        "manifest_capture": capture,
        "cold_median_seconds": statistics.median(run["elapsed_seconds"] for run in cold_runs),
        "warm_median_seconds": statistics.median(run["elapsed_seconds"] for run in warm_runs),
    }
    (artifact_dir / "timing-report.json").write_text(
        json.dumps(timing, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return deterministic, timing


def main() -> int:
    args = parse_args()
    root = repo_root()
    output = resolve(root, args.output)
    artifact_dir = resolve(root, args.artifact_dir)
    build_dir = resolve(root, args.build_dir)
    artifact_dir.mkdir(parents=True, exist_ok=True)

    baseline: dict[str, object] = {
        "schema": 1,
        "scope": args.scope,
        "source_inventory": collect_source_inventory(root),
    }
    if not args.skip_compile:
        compiled, _ = collect_compile_evidence(
            root, build_dir, args.config, artifact_dir, args.runs, args.skip_build
        )
        baseline["compiled_spirv"] = compiled

    serialized = json.dumps(baseline, indent=2, sort_keys=True) + "\n"
    if args.check:
        if not output.is_file():
            raise RuntimeError(f"Baseline does not exist: {output}")
        if output.read_text(encoding="utf-8") != serialized:
            raise RuntimeError(f"SDK shader baseline differs: {output}")
        print(f"SDK shader baseline matches: {output}")
        return 0

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(serialized, encoding="utf-8", newline="")
    print(f"SDK shader baseline: {output}")
    print(f"Local evidence: {artifact_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
