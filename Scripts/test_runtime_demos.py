"""Author isolated demo scenes, export them, and validate relocated runtime apps."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

DEMOS = ("rendering", "rendering-regression", "ddgi", "ecosyslab", "digital-agriculture",
         "lsystem", "procedural-galaxy", "3dgs", "bicycle", "bistro")


def inventory(root: Path) -> dict[str, str]:
    result = {}
    for path in sorted(root.rglob("*")):
        if path.is_file():
            with path.open("rb") as stream:
                result[path.relative_to(root).as_posix()] = hashlib.file_digest(stream, "sha256").hexdigest()
    return result


def run(command: list[str], log: Path, cwd: Path, timeout: int, env=None) -> None:
    with log.open("w", encoding="utf-8") as output:
        result = subprocess.run(command, cwd=cwd, env=env, stdout=output,
                                stderr=subprocess.STDOUT, timeout=timeout, check=False)
    if result.returncode:
        raise RuntimeError(f"Exit {result.returncode}: {command[0]}; see {log}")


def validate(args: argparse.Namespace, demo: str) -> dict:
    started = time.monotonic()
    work = args.work_dir / demo
    work.mkdir(parents=True, exist_ok=False)
    old_reports = {p: p.stat().st_mtime_ns for p in args.resource_root.rglob("Cache/RuntimeAuthoring/report.json")}
    author_command = [str(args.editor), "--demo", demo, "--resource-root", str(args.resource_root),
                      "--author-runtime-scene"]
    if demo in ("ecosyslab", "procedural-galaxy"):
        author_command += ["--author-warmup-frames", "120"]
    run(author_command, work / "author.log", args.editor.parent, args.timeout)
    author_log = (work / "author.log").read_text(encoding="utf-8", errors="replace")
    for marker in (f"EVOENGINE_RUNTIME_SCENE_AUTHORED demo={demo} ", "EVOENGINE_RUNTIME_SCENE_AUTHORING_COMPLETE"):
        if marker not in author_log:
            raise RuntimeError(f"Missing authoring marker: {marker}")
    if demo == "ecosyslab" and "Finished the EcoSysLab demo Acacia eight-year growth animation and generated its mesh." not in author_log:
        raise RuntimeError("EcoSysLab authoring did not finish generating its demo tree")
    reports = [json.loads(p.read_text(encoding="utf-8"))
               for p in args.resource_root.rglob("Cache/RuntimeAuthoring/report.json")
               if old_reports.get(p) != p.stat().st_mtime_ns]
    report, = [r for r in reports if r["demo"] == demo]
    if min(report["camera_resolution"]) <= 1:
        raise RuntimeError("Demo camera was saved at an unusable viewport resolution")
    project = Path(report["project"]).resolve()
    scene = Path(report["scene"]).resolve()
    if not project.is_relative_to(args.resource_root) or not scene.is_relative_to(project.parent / "Assets"):
        raise RuntimeError("Authoring report refers outside the isolated project")
    before = inventory(project.parent / "Assets")
    project_before = project.read_bytes()
    request = {key: report[key] for key in ("schema_version", "application_name", "project",
               "startup_scene_handle", "editor_identity", "loaded_packages", "runtime_config")}
    request_path = work / "request.json"
    request_path.write_text(json.dumps(request, indent=2), encoding="utf-8")
    distribution = work / "distribution"
    run([str(args.exporter), "--request", str(request_path), "--template", str(args.template),
         "--output", str(distribution)], work / "export.log", work, args.timeout)
    if before != inventory(distribution / "Project/Assets"):
        raise RuntimeError("Export changed asset contents")
    if before != inventory(project.parent / "Assets") or project_before != project.read_bytes():
        raise RuntimeError("Export modified the source project")
    config = json.loads((distribution / "runtime.yaml").read_text(encoding="utf-8"))
    if config["packages"] != report["loaded_packages"]:
        raise RuntimeError("Export changed the active package set")
    exported_project_before = (distribution / config["project"]).read_bytes()
    relocated = work / "relocated runtime Ω"
    distribution.rename(relocated)
    unrelated = work / "unrelated working directory"
    decoy = work / "environment decoy"
    unrelated.mkdir()
    decoy.mkdir()
    environment = os.environ.copy()
    windows = Path(environment.get("SystemRoot", r"C:\Windows"))
    environment["PATH"] = os.pathsep.join(map(str, (windows / "System32", windows)))
    for key in ("TEMP", "TMP", "EVOENGINE_SHADER_CACHE_DIR", "EVOENGINE_PIPELINE_CACHE_DIR"):
        environment[key] = str(decoy)
    exe = relocated / (report["application_name"] + ".exe")
    run([str(exe), "--no-error-dialog", "--frames", "60", "--capture", "UserData/demo.png"],
        work / "launch.log", unrelated, args.timeout, environment)
    runtime_log = (relocated / "Logs/runtime.log").read_text(encoding="utf-8", errors="replace")
    for marker in (f"RUNTIME_STARTED scene={report['startup_scene_handle']} packages={len(report['loaded_packages'])}",
                   "RUNTIME_SMOKE_COMPLETE frames=60"):
        if marker not in runtime_log:
            raise RuntimeError(f"Missing runtime marker: {marker}")
    if "RUNTIME_FATAL" in runtime_log or "VUID-" in runtime_log:
        raise RuntimeError("Runtime reported a fatal or Vulkan validation error")
    capture = relocated / "UserData/demo.png"
    with capture.open("rb") as stream:
        if stream.read(8) != b"\x89PNG\r\n\x1a\n":
            raise RuntimeError("Invalid runtime capture")
    if any(unrelated.iterdir()) or any(decoy.iterdir()):
        raise RuntimeError("Runtime wrote outside its distribution")
    if before != inventory(relocated / "Project/Assets"):
        raise RuntimeError("Runtime modified packaged assets")
    if exported_project_before != (relocated / config["project"]).read_bytes():
        raise RuntimeError("Runtime modified its project metadata")
    if project_before != project.read_bytes():
        raise RuntimeError("Runtime modified the source project metadata")
    return {"demo": demo, "passed": True, "seconds": round(time.monotonic() - started, 2),
            "entity_count": report["entity_count"], "packages": report["loaded_packages"],
            "camera_resolution": report["camera_resolution"],
            "project": str(project), "scene": str(scene),
            "executable": str(exe), "capture": str(capture)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("editor", "exporter", "template", "resource-root", "work-dir"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--demo", choices=DEMOS, action="append")
    parser.add_argument("--timeout", type=int, default=900)
    args = parser.parse_args()
    for name in ("editor", "exporter", "template", "resource_root", "work_dir"):
        setattr(args, name, getattr(args, name).resolve())
    if args.resource_root.name != "Resources" or not args.resource_root.is_dir():
        parser.error("--resource-root must be an isolated Resources directory")
    args.work_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for demo in args.demo or DEMOS:
        print(f"Validating {demo}...", flush=True)
        try:
            result = validate(args, demo)
        except Exception as error:
            result = {"demo": demo, "passed": False, "error": str(error)}
        results.append(result)
        (args.work_dir / "results.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
        print(json.dumps(result), flush=True)
    for result in results:
        if result["passed"] and any(not Path(result[key]).is_file() for key in ("project", "scene")):
            result.update(passed=False, error="A later demo deleted an earlier authored project or scene")
    (args.work_dir / "results.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    return int(any(not result["passed"] for result in results))


if __name__ == "__main__":
    raise SystemExit(main())
