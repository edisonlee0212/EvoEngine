#!/usr/bin/env python3
"""Run the local CSM milestone closeout gate."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shlex
import subprocess
import sys
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

ERROR_PATTERNS = (
    re.compile(r"\bfatal\b", re.IGNORECASE),
    re.compile(r"\bunhandled exception\b", re.IGNORECASE),
    re.compile(r"\baccess violation\b", re.IGNORECASE),
    re.compile(r"\bassert(?:ion)? failed\b", re.IGNORECASE),
    re.compile(r"\bvalidation error\b", re.IGNORECASE),
    re.compile(r"\bdevice(?:[- ]| has been )lost\b", re.IGNORECASE),
    re.compile(r"\bVK_ERROR_DEVICE_LOST\b", re.IGNORECASE),
    re.compile(r"\bEVOENGINE_(?:APP_TEST_RESULT failed|FATAL|ERROR)\b", re.IGNORECASE),
)

SHADOW_DEBUG_CAPTURE_MODES = ("cascade-index", "light-uv", "light-depth", "atlas-uv", "texel-density")
P4_SHADOW_DEBUG_CAPTURE_SCENARIOS = (("bistro", "cascade-index", 1), ("rendering", "atlas-uv", 4))
P4_FIT_CAPTURE_SCENARIOS = (("tight", "tight-aabb"),)
RENDERER_LAUNCH_CAP = 16


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def executable_name(name: str) -> str:
    return f"{name}.exe" if os.name == "nt" else name


def default_editor_path(root: Path) -> Path:
    return root / "out" / "install" / "vs2026-x64" / "bin" / executable_name("EvoEngineEditor")


def default_build_dir(root: Path) -> Path:
    return root / "out" / "build" / "vs2026-x64"


def format_command(command: list[str]) -> str:
    return subprocess.list2cmdline(command) if os.name == "nt" else shlex.join(command)


def run_step(name: str, command: list[str], cwd: Path) -> None:
    print(f"\n==> {name}", flush=True)
    print(format_command(command), flush=True)
    completed = subprocess.run(command, cwd=cwd)
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)


def run_logged_step(name: str, command: list[str], cwd: Path, log_path: Path, timeout: int | None = None) -> str:
    print(f"\n==> {name}", flush=True)
    print(format_command(command), flush=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        log_path.write_text(f"$ {format_command(command)}\n\n{output}", encoding="utf-8")
        raise SystemExit(f"Timed out after {timeout} seconds; see {log_path}") from error
    output = completed.stdout or ""
    log_path.write_text(f"$ {format_command(command)}\n\n{output}", encoding="utf-8")
    if output:
        print(output, end="" if output.endswith("\n") else "\n")
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)
    assert_log_is_clean(log_path, output)
    return output


def assert_log_is_clean(log_path: Path, output: str) -> None:
    for line in output.splitlines():
        if any(pattern.search(line) for pattern in ERROR_PATTERNS):
            raise SystemExit(f"Error marker found in {log_path}: {line}")


def log_error_line(log_path: Path) -> str | None:
    if not log_path.exists():
        return None
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if any(pattern.search(line) for pattern in ERROR_PATTERNS):
            return line
    return None


def logged_command(log_path: Path) -> str:
    if not log_path.exists():
        return ""
    first_line = log_path.read_text(encoding="utf-8", errors="replace").splitlines()[:1]
    return first_line[0][2:] if first_line and first_line[0].startswith("$ ") else ""


def command_fingerprint(command: list[str] | str, context: str = "") -> str:
    command_line = command if isinstance(command, str) else format_command(command)
    return hashlib.sha256(f"{command_line}\n{context}".encode("utf-8")).hexdigest()


class RendererAttemptTracker:
    def __init__(self, manifest_path: Path, log_dir: Path, resume: bool):
        self.manifest_path = manifest_path
        self.attempts: list[dict[str, object]] = []
        existing_logs = sorted(log_dir.glob("*.log"), key=lambda path: path.stat().st_mtime_ns)
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            if manifest.get("version") != 1 or not isinstance(manifest.get("attempts"), list):
                raise SystemExit(f"Unsupported renderer-attempt manifest: {manifest_path}")
            self.attempts = manifest["attempts"]
        elif existing_logs:
            if not resume:
                raise SystemExit(f"Existing renderer logs require --resume or a new milestone name: {log_dir}")
            for log_path in existing_logs:
                self.import_log(log_path)
            self.save()
        if (self.attempts or existing_logs) and not resume:
            raise SystemExit(f"Existing renderer attempts require --resume or a new milestone name: {manifest_path}")
        referenced_logs = {str(attempt.get("log_path", "")) for attempt in self.attempts}
        recovered_logs = [path for path in existing_logs if str(path.resolve()) not in referenced_logs]
        for log_path in recovered_logs:
            self.import_log(log_path)
        if recovered_logs:
            self.save()
        interrupted = False
        for attempt in self.attempts:
            if attempt.get("status") != "running":
                continue
            attempt["status"] = "failed"
            attempt["error"] = "Renderer attempt was interrupted before completion."
            log_path = Path(str(attempt["log_path"]))
            if not log_path.exists():
                log_path.parent.mkdir(parents=True, exist_ok=True)
                log_path.write_text(
                    f"$ {attempt.get('command', '')}\n\nEVOENGINE_ERROR: Renderer attempt was interrupted.\n",
                    encoding="utf-8",
                )
            elif log_error_line(log_path) is None:
                with log_path.open("a", encoding="utf-8") as log:
                    log.write("\nEVOENGINE_APP_TEST_RESULT failed: Renderer attempt was interrupted.\n")
            interrupted = True
        if interrupted:
            self.save()

    @property
    def count(self) -> int:
        return len(self.attempts)

    def import_log(self, log_path: Path) -> None:
        command_line = logged_command(log_path)
        failed = "failed" in log_path.stem or not command_line or log_error_line(log_path) is not None
        self.attempts.append(
            {
                "id": self.count + 1,
                "command": command_line,
                "fingerprint": command_fingerprint(command_line),
                "log_path": str(log_path.resolve()),
                "status": "failed" if failed else "succeeded",
                "legacy_import": True,
            }
        )

    def save(self) -> None:
        self.manifest_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.manifest_path.with_suffix(f"{self.manifest_path.suffix}.tmp")
        temporary_path.write_text(
            json.dumps({"version": 1, "attempts": self.attempts}, indent=2) + "\n", encoding="utf-8"
        )
        temporary_path.replace(self.manifest_path)

    def completed(self, log_path: Path, command: list[str], fingerprint_context: str = "") -> bool:
        fingerprint = command_fingerprint(command, fingerprint_context)
        resolved_log_path = str(log_path.resolve())
        return any(
            attempt.get("status") == "succeeded"
            and attempt.get("fingerprint") == fingerprint
            and attempt.get("log_path") == resolved_log_path
            for attempt in self.attempts
        ) and logged_command(log_path) == format_command(command) and log_error_line(log_path) is None

    def start(
        self,
        command: list[str],
        log_path: Path,
        preview_path: Path | None = None,
        metrics_path: Path | None = None,
        fingerprint_context: str = "",
    ) -> dict[str, object]:
        if self.count >= RENDERER_LAUNCH_CAP:
            raise SystemExit(f"Renderer launch cap reached before {log_path}.")
        resolved_log_path = str(log_path.resolve())
        if log_path.exists():
            prior_id = next(
                (
                    attempt.get("id", self.count)
                    for attempt in reversed(self.attempts)
                    if attempt.get("log_path") == resolved_log_path
                ),
                self.count,
            )
            archived_path = log_path.with_name(f"{log_path.stem}-attempt-{prior_id}.log")
            suffix = 2
            while archived_path.exists():
                archived_path = log_path.with_name(f"{log_path.stem}-attempt-{prior_id}-{suffix}.log")
                suffix += 1
            log_path.replace(archived_path)
            for attempt in self.attempts:
                if attempt.get("log_path") == resolved_log_path:
                    attempt["log_path"] = str(archived_path.resolve())
            self.save()
        command_line = format_command(command)
        attempt: dict[str, object] = {
            "id": self.count + 1,
            "command": command_line,
            "fingerprint": command_fingerprint(command, fingerprint_context),
            "fingerprint_context": fingerprint_context,
            "log_path": resolved_log_path,
            "preview_path": str(preview_path.resolve()) if preview_path else None,
            "metrics_path": str(metrics_path.resolve()) if metrics_path else None,
            "status": "running",
            "started": datetime.now().isoformat(timespec="seconds"),
        }
        self.attempts.append(attempt)
        self.save()
        return attempt

    def finish(self, attempt: dict[str, object], status: str, error: str | None = None) -> None:
        log_path = Path(str(attempt["log_path"]))
        if status == "succeeded" and not log_path.exists():
            status = "failed"
            error = "Renderer attempt completed without a log."
        attempt["status"] = status
        attempt["finished"] = datetime.now().isoformat(timespec="seconds")
        if error:
            attempt["error"] = error
        if not log_path.exists():
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_path.write_text(
                f"$ {attempt.get('command', '')}\n\nEVOENGINE_ERROR: {error or status}\n", encoding="utf-8"
            )
        elif status == "failed" and log_error_line(log_path) is None:
            failure = str(error or status).replace("\n", " ")
            with log_path.open("a", encoding="utf-8") as log:
                log.write(f"\nEVOENGINE_APP_TEST_RESULT failed: {failure}\n")
        self.save()

    def log_paths(self, status: str) -> list[Path]:
        paths = [Path(str(attempt["log_path"])) for attempt in self.attempts if attempt.get("status") == status]
        return [path for path in paths if path.exists()]

    def status_count(self, status: str) -> int:
        return sum(attempt.get("status") == status for attempt in self.attempts)

    def invalidate_completed(self, log_path: Path, command: list[str], error: BaseException) -> None:
        fingerprint = command_fingerprint(command)
        resolved_log_path = str(log_path.resolve())
        attempt = next(
            (
                attempt
                for attempt in reversed(self.attempts)
                if attempt.get("status") == "succeeded"
                and attempt.get("fingerprint") == fingerprint
                and attempt.get("log_path") == resolved_log_path
            ),
            None,
        )
        if attempt:
            self.finish(attempt, "failed", str(error))


def capture_is_complete(
    tracker: RendererAttemptTracker,
    log_path: Path,
    preview_path: Path,
    metrics_path: Path | None,
    command: list[str],
) -> bool:
    return (
        tracker.completed(log_path, command)
        and preview_path.exists()
        and preview_path.stat().st_size > 0
        and (metrics_path is None or (metrics_path.exists() and metrics_path.stat().st_size > 0))
    )


def ensure_bistro_resources(root: Path, skip_generate: bool) -> None:
    bistro_project = root / "Resources" / ".generated" / "EvoEngine-DemoProjects" / "Bistro" / "Bistro.eveproj"
    if bistro_project.exists():
        print(f"Bistro project: {bistro_project}", flush=True)
        return
    if skip_generate:
        raise SystemExit(f"Missing Bistro project: {bistro_project}")
    run_step(
        "Generate Bistro demo resources",
        [
            sys.executable,
            str(root / "Scripts" / "prepare_demos.py"),
            "--demo",
            "bistro",
            "--validate",
            "--prepare",
            "--no-previews",
        ],
        root,
    )


def smoke_demo(
    editor: Path,
    demo_id: str,
    seconds: int,
    log_path: Path,
    tracker: RendererAttemptTracker,
    resume: bool,
    extra_args: list[str] | None = None,
) -> None:
    command = [str(editor), "--demo", demo_id, "--editor"]
    if extra_args:
        command.extend(extra_args)
    fingerprint_context = f"smoke-seconds={seconds}"
    if resume and tracker.completed(log_path, command, fingerprint_context):
        print(f"\n==> Reuse completed {demo_id} smoke test", flush=True)
        return
    attempt = tracker.start(command, log_path, fingerprint_context=fingerprint_context)
    print(f"\n==> Smoke {demo_id} for {seconds} seconds", flush=True)
    print(format_command(command), flush=True)
    try:
        process = subprocess.Popen(
            command,
            cwd=editor.parent,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        timed_out = False
        try:
            output, _ = process.communicate(timeout=seconds)
        except subprocess.TimeoutExpired:
            timed_out = True
            process.terminate()
            try:
                output, _ = process.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                output, _ = process.communicate(timeout=10)

        output = output or ""
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(f"$ {format_command(command)}\n\n{output}", encoding="utf-8")
        if output:
            print(output, end="" if output.endswith("\n") else "\n")
        assert_log_is_clean(log_path, output)
        if not timed_out:
            raise SystemExit(
                f"{demo_id} exited after less than {seconds} seconds with code {process.returncode}; see {log_path}"
            )
    except BaseException as error:
        tracker.finish(attempt, "failed", str(error))
        raise
    tracker.finish(attempt, "succeeded")
    print(f"{demo_id} stayed up for {seconds} seconds.", flush=True)


def capture_preview_command(
    editor: Path,
    demo_id: str,
    render_mode: str,
    output_path: Path,
    width: int,
    height: int,
    warmup_frames: int,
    log_path: Path,
    metrics_path: Path | None = None,
    extra_args: list[str] | None = None,
) -> list[str]:
    command = [
        str(editor),
        "--demo",
        demo_id,
        "--editor",
        "--capture-demo-preview",
        str(output_path),
        "--preview-render-mode",
        render_mode,
        "--preview-width",
        str(width),
        "--preview-height",
        str(height),
        "--preview-warmup-frames",
        str(warmup_frames),
        "--preview-deterministic",
    ]
    if metrics_path:
        command.extend(["--preview-metrics-json", str(metrics_path)])
    if extra_args:
        command.extend(extra_args)
    return command


def capture_preview(
    editor: Path,
    demo_id: str,
    render_mode: str,
    output_path: Path,
    width: int,
    height: int,
    warmup_frames: int,
    log_path: Path,
    tracker: RendererAttemptTracker,
    metrics_path: Path | None = None,
    extra_args: list[str] | None = None,
    validate_capture: Callable[[], None] | None = None,
) -> None:
    command = capture_preview_command(
        editor,
        demo_id,
        render_mode,
        output_path,
        width,
        height,
        warmup_frames,
        log_path,
        metrics_path,
        extra_args,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    attempt = tracker.start(command, log_path, output_path, metrics_path)
    output_path.unlink(missing_ok=True)
    if metrics_path:
        metrics_path.unlink(missing_ok=True)
    try:
        run_logged_step(f"Capture {demo_id} {render_mode}", command, editor.parent, log_path, timeout=900)
        if not output_path.exists() or output_path.stat().st_size == 0:
            raise SystemExit(f"Preview was not written: {output_path}")
        if metrics_path and (not metrics_path.exists() or metrics_path.stat().st_size == 0):
            raise SystemExit(f"Preview metrics were not written: {metrics_path}")
        if validate_capture:
            validate_capture()
    except BaseException as error:
        tracker.finish(attempt, "failed", str(error))
        raise
    tracker.finish(attempt, "succeeded")


def capture_preview_or_resume(
    editor: Path,
    demo_id: str,
    render_mode: str,
    output_path: Path,
    width: int,
    height: int,
    warmup_frames: int,
    log_path: Path,
    metrics_path: Path | None,
    extra_args: list[str],
    resume: bool,
    tracker: RendererAttemptTracker,
    validate_capture: Callable[[], None] | None = None,
) -> int:
    command = capture_preview_command(
        editor,
        demo_id,
        render_mode,
        output_path,
        width,
        height,
        warmup_frames,
        log_path,
        metrics_path,
        extra_args,
    )
    if resume and capture_is_complete(tracker, log_path, output_path, metrics_path, command):
        try:
            if validate_capture:
                validate_capture()
        except BaseException as error:
            tracker.invalidate_completed(log_path, command, error)
            raise
        print(f"\n==> Reuse completed capture {output_path}", flush=True)
        return tracker.count
    capture_preview(
        editor,
        demo_id,
        render_mode,
        output_path,
        width,
        height,
        warmup_frames,
        log_path,
        tracker,
        metrics_path,
        extra_args,
        validate_capture,
    )
    return tracker.count


def assert_p4_primary_shadow_paths(metrics_path: Path) -> None:
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    shadow = metrics.get("directional_shadow", {})
    draws = shadow.get("draws", {})
    casters = draws.get("directional_shadow_casters", {})
    if draws.get("indirect_draw_calls", 0) <= 0:
        raise SystemExit(f"P4 primary capture did not exercise directional indirect draws: {metrics_path}")
    if shadow.get("mesh_shader_enabled") and casters.get("mesh_shader", 0) <= 0:
        raise SystemExit(f"P4 primary capture enabled mesh shaders but recorded no mesh-shader draw: {metrics_path}")


def assert_p4_caster_fixture(metrics_path: Path) -> None:
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    shadow = metrics.get("directional_shadow", {})
    draws = shadow.get("draws", {})
    casters = draws.get("directional_shadow_casters", {})
    if not metrics.get("csm_caster_fixture"):
        raise SystemExit(f"P4 caster fixture was not recorded: {metrics_path}")
    if not shadow.get("indirect_rendering_enabled") or draws.get("indirect_draw_calls", 0) <= 0:
        raise SystemExit(f"P4 caster fixture did not exercise indexed-indirect directional draws: {metrics_path}")
    if draws.get("direct_draw_calls", 0) <= 0:
        raise SystemExit(f"P4 caster fixture did not exercise direct directional draws: {metrics_path}")
    required = ["regular", "instanced", "skinned", "external"]
    missing = [name for name in required if casters.get(name, 0) <= 0]
    if missing:
        raise SystemExit(f"P4 caster fixture missed {', '.join(missing)} directional paths: {metrics_path}")
    if casters.get("strands", 0) != 0:
        raise SystemExit(f"P4 caster fixture unexpectedly rendered directional strand shadows: {metrics_path}")


def shadow_metrics_summary(metrics_paths: list[Path]) -> str:
    lines: list[str] = []
    for path in metrics_paths:
        metrics = json.loads(path.read_text(encoding="utf-8"))
        shadow = metrics.get("directional_shadow", {})
        lights = shadow.get("lights", [])
        draws = shadow.get("draws", {})
        gpu_timing = next(
            (timing for timing in metrics.get("gpu_sections", []) if timing.get("name") == "Directional Shadow"),
            {},
        )
        deferred_timing = next(
            (timing for timing in metrics.get("gpu_sections", []) if timing.get("name") == "Deferred Lighting"),
            {},
        )
        lines.append(
            f"- `{path}`: lights={len(lights)}, median_gpu_ms={gpu_timing.get('median_ms', 'unavailable')}, "
            f"p95_gpu_ms={gpu_timing.get('p95_ms', 'unavailable')}, "
            f"deferred_lighting_median_gpu_ms={deferred_timing.get('median_ms', 'unavailable')}, "
            f"deferred_lighting_p95_gpu_ms={deferred_timing.get('p95_ms', 'unavailable')}, "
            f"draw_calls={draws.get('total_draw_calls', 'unavailable')}, "
            f"indirect_commands={draws.get('indirect_draw_commands', 'unavailable')}, "
            f"indirect_enabled={shadow.get('indirect_rendering_enabled', 'unavailable')}, "
            f"mesh_shader_enabled={shadow.get('mesh_shader_enabled', 'unavailable')}, "
            f"caster_draws={draws.get('directional_shadow_casters', 'unavailable')}, "
            f"frames_per_second={metrics.get('frames_per_second', 'unavailable')}"
        )
        for light in lights:
            radius_summary = (
                f"radius_world={light['pcf_radius_world']}"
                if "pcf_radius_world" in light
                else f"radius_texels={light['pcf_radius_texels']}"
            )
            lines.append(
                f"  - light {light['index']}: {radius_summary}, "
                f"bias={light['bias']}, slope_bias={light['slope_bias']}, normal_offset={light['normal_offset']}"
            )
            for cascade in light.get("cascades", []):
                radius_texels = ""
                if "pcf_radius_texels_x" in cascade:
                    radius_texels = (
                        f", radius_texels=[{cascade['pcf_radius_texels_x']}, {cascade['pcf_radius_texels_y']}]"
                    )
                lines.append(
                    f"    - cascade {cascade['index']}: split=[{cascade['split_start']}, {cascade['split_end']}], "
                    f"half_extent={cascade['orthographic_half_extent']}, viewport={cascade['viewport']}, "
                    f"world_units_per_texel={cascade['world_units_per_texel']}, "
                    f"light_space_depth_span={cascade['light_space_depth_span']}{radius_texels}"
                )
    return "\n".join(lines) if lines else "- No metrics captures were requested."


def write_report(
    report_path: Path,
    milestone: str,
    editor: Path,
    smoke_seconds: int,
    shadow_split_lambda: float,
    shadow_cascade_transition_width: float | None,
    shadow_distance_fade: float | None,
    shadow_fit: str,
    shadow_map_resolution: str,
    preview_paths: list[Path],
    metrics_paths: list[Path],
    log_paths: list[Path],
    failed_log_paths: list[Path],
    renderer_launches: int,
    successful_renderer_launches: int,
    failed_renderer_launches: int,
) -> None:
    preview_lines = "\n".join(f"- `{path}`" for path in preview_paths)
    log_lines = "\n".join(f"- `{path}`" for path in log_paths)
    failed_log_lines = "\n".join(f"- `{path}`" for path in failed_log_paths) or "- None."
    metrics_lines = shadow_metrics_summary(metrics_paths)
    sampling_policy = "PCF, radius_world=directional light_size"
    shadow_resolution_command_suffix = ""
    if not shadow_map_resolution.startswith("default"):
        shadow_resolution_command_suffix = f" --shadow-map-resolution {shadow_map_resolution}"
    report_path.write_text(
        f"""# {milestone} CSM Validation Report

Generated: {datetime.now().isoformat(timespec="seconds")}

Editor:
`{editor}`

Smoke launch commands:
- `{editor} --demo bistro --editor{shadow_resolution_command_suffix}`
- `{editor} --demo rendering --editor{shadow_resolution_command_suffix}`

Smoke duration:
{smoke_seconds} seconds per demo.

Shadow fit policy:
{shadow_fit}

Shadow split policy:
Practical Log/Uniform, lambda={shadow_split_lambda:.3f}

Shadow sampling:
{sampling_policy}

Shadow map resolution:
{shadow_map_resolution}

Renderer launch budget:
{renderer_launches} of {RENDERER_LAUNCH_CAP} nonredundant attempts ({successful_renderer_launches} successful; {failed_renderer_launches} failed).

Shadow fade override:
{f"cascade_transition_width={shadow_cascade_transition_width:.3f}" if shadow_cascade_transition_width is not None else "default"}{f" distance_fade={shadow_distance_fade:.3f}" if shadow_distance_fade is not None else ""}

Preview captures:
{preview_lines}

Logs:
{log_lines}

Failed-attempt logs retained as recovery evidence:
{failed_log_lines}

Directional shadow measurements:
{metrics_lines}

Visual check checklist:
- Bistro cascade seams: road curb, street edge, building facade transitions.
- Bistro near/far sharpness: curb stones, plant pot, motorbike front, distant building shadows.
- Bistro grazing surfaces: curb edges and shallow-angle road surfaces.
- Rendering scene coverage: near primitives, far surfaces, camera rotation, and camera translation.
- Alpha-tested foliage: Bistro trees and plant leaves where visible in the captured view.
""",
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--milestone", default="M1", help="Milestone label used for output folders.")
    parser.add_argument("--config", default="RelWithDebInfo", help="CMake build configuration.")
    parser.add_argument("--build-dir", type=Path, default=default_build_dir(root), help="CMake build directory.")
    parser.add_argument("--editor", type=Path, default=default_editor_path(root), help="Installed EvoEngineEditor path.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=root / "out" / "csm-validation",
        help="Directory where validation logs and captures are written.",
    )
    parser.add_argument("--width", type=int, default=1920, help="Preview capture width.")
    parser.add_argument("--height", type=int, default=1080, help="Preview capture height.")
    parser.add_argument("--warmup-frames", type=int, default=1800, help="Raster preview warmup frames.")
    parser.add_argument("--shadow-debug-warmup-frames", type=int, default=120, help="Shadow diagnostic warmup frames.")
    parser.add_argument("--shadow-debug-cascade", type=int, default=0, help="Selected cascade for shadow diagnostics.")
    parser.add_argument("--shadow-debug-light", type=int, default=0, help="Selected directional light for diagnostics.")
    parser.add_argument("--shadow-split-lambda", type=float, default=0.5, help="Practical split lambda.")
    parser.add_argument(
        "--shadow-fit",
        choices=("stable-sphere", "tight-aabb"),
        help="Camera/frustum cascade fit used for captures. Defaults to stable sphere.",
    )
    parser.add_argument(
        "--shadow-map-resolution",
        choices=("low", "medium", "high", "very-high"),
        help="Optional startup shadow map resolution quality for editor smoke and capture commands.",
    )
    parser.add_argument(
        "--shadow-pcf-samples",
        type=int,
        help="Optional directional PCF sample count for capture comparison (1-64).",
    )
    parser.add_argument(
        "--shadow-cascade-transition-width",
        type=float,
        help="Optional cascade transition width in positive linear view-depth units.",
    )
    parser.add_argument(
        "--shadow-distance-fade",
        type=float,
        help="Optional final max-shadow-distance fade width in positive linear view-depth units.",
    )
    parser.add_argument("--smoke-seconds", type=int, default=30, help="Smoke-test duration per demo.")
    parser.add_argument("--skip-format", action="store_true", help="Skip C++ format check.")
    parser.add_argument("--skip-build", action="store_true", help="Skip EvoEngineEditor build.")
    parser.add_argument("--skip-install", action="store_true", help="Skip app install step.")
    parser.add_argument("--skip-smoke", action="store_true", help="Skip 30-second demo smoke tests.")
    parser.add_argument("--skip-capture", action="store_true", help="Skip preview captures.")
    parser.add_argument("--resume", action="store_true", help="Reuse complete captures and count existing logs as attempts.")
    parser.add_argument("--capture-shadow-diagnostics", action="store_true", help="Capture CSM diagnostic previews.")
    parser.add_argument("--skip-bistro-generate", action="store_true", help="Do not auto-generate missing Bistro assets.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = repo_root()
    milestone_key = args.milestone.upper()
    if milestone_key == "CSM-P0":
        raise SystemExit("CSM-P0 used the retired Legacy Stable fit and is historical-only; use a new milestone label.")
    milestone_dir = args.output_dir.resolve() / args.milestone.lower()
    log_dir = milestone_dir / "logs"
    preview_dir = milestone_dir / "previews"
    editor = args.editor.resolve()
    build_dir = args.build_dir.resolve()
    if args.width <= 0 or args.height <= 0:
        raise SystemExit("--width and --height must be positive.")
    if args.warmup_frames < 0:
        raise SystemExit("--warmup-frames must be non-negative.")
    if args.shadow_debug_warmup_frames < 0:
        raise SystemExit("--shadow-debug-warmup-frames must be non-negative.")
    args.shadow_debug_cascade = max(0, min(args.shadow_debug_cascade, 3))
    args.shadow_debug_light = max(0, args.shadow_debug_light)
    args.shadow_split_lambda = max(0.0, min(args.shadow_split_lambda, 1.0))
    shadow_fit = args.shadow_fit or "stable-sphere"
    shadow_fit_report = {
        "stable-sphere": "Stable Sphere",
        "tight-aabb": "Tight Light-Space AABB",
    }[shadow_fit]
    if args.shadow_cascade_transition_width is not None:
        args.shadow_cascade_transition_width = max(0.0, args.shadow_cascade_transition_width)
    if args.shadow_distance_fade is not None:
        args.shadow_distance_fade = max(0.0, args.shadow_distance_fade)
    if args.shadow_pcf_samples is not None:
        args.shadow_pcf_samples = max(1, min(args.shadow_pcf_samples, 64))
    if args.smoke_seconds <= 0:
        raise SystemExit("--smoke-seconds must be positive.")
    shadow_map_resolution_args: list[str] = []
    shadow_map_resolution_report = "default (directional/point/spot 4096 x 4096)"
    if args.shadow_map_resolution:
        shadow_map_resolution_args = ["--shadow-map-resolution", args.shadow_map_resolution]
        shadow_map_resolution_report = args.shadow_map_resolution

    if not args.skip_format:
        run_step(
            "C++ format check",
            [
                sys.executable,
                str(root / "Scripts" / "format_cpp.py"),
                "--check",
                "--root",
                "EvoEngine_SDK",
                "--root",
                "EvoEngine_App",
                "--root",
                "EvoEngine_Tests",
            ],
            root,
        )
    if not args.skip_build:
        run_step(
            "Build EvoEngineEditor",
            ["cmake", "--build", str(build_dir), "--config", args.config, "--target", "EvoEngineEditor"],
            root,
        )
    if not args.skip_install:
        run_step(
            "Install apps",
            [
                sys.executable,
                str(root / "Scripts" / "install_apps.py"),
                "--config",
                args.config,
                "--no-open",
                "--incremental",
            ],
            root,
        )
    if not editor.exists():
        raise SystemExit(f"Missing EvoEngineEditor: {editor}")
    ensure_bistro_resources(root, args.skip_bistro_generate)

    capture_shadow_light_lanes = milestone_key == "CSM-P4"
    capture_p4_delivery_lanes = milestone_key == "CSM-P4"
    planned_renderer_launches = (0 if args.skip_smoke else 2) + (0 if args.skip_capture else 2)
    if not args.skip_capture and args.capture_shadow_diagnostics:
        planned_renderer_launches += (
            len(P4_SHADOW_DEBUG_CAPTURE_SCENARIOS)
            if capture_p4_delivery_lanes
            else 2 * len(SHADOW_DEBUG_CAPTURE_MODES)
        )
    if not args.skip_capture and capture_shadow_light_lanes:
        planned_renderer_launches += 2
    if not args.skip_capture and capture_p4_delivery_lanes:
        planned_renderer_launches += len(P4_FIT_CAPTURE_SCENARIOS) + 2
    if not args.resume and planned_renderer_launches > RENDERER_LAUNCH_CAP:
        raise SystemExit(
            f"Planned renderer launches ({planned_renderer_launches}) exceed the CSM cap ({RENDERER_LAUNCH_CAP})."
        )

    tracker = RendererAttemptTracker(milestone_dir / "renderer-attempts.json", log_dir, args.resume)
    renderer_launches = tracker.count
    if renderer_launches > RENDERER_LAUNCH_CAP:
        raise SystemExit(
            f"Existing renderer attempts ({renderer_launches}) exceed the CSM cap ({RENDERER_LAUNCH_CAP})."
        )
    failed_log_paths = tracker.log_paths("failed")
    log_paths: list[Path] = tracker.log_paths("succeeded")
    if not args.skip_smoke:
        for demo_id in ("bistro", "rendering"):
            log_path = log_dir / f"{demo_id}-smoke.log"
            smoke_demo(
                editor, demo_id, args.smoke_seconds, log_path, tracker, args.resume, shadow_map_resolution_args
            )
            renderer_launches = tracker.count
            log_paths.append(log_path)

    preview_paths: list[Path] = []
    metrics_paths: list[Path] = []
    if not args.skip_capture:
        preview_policy_args = [
            "--preview-shadow-fit",
            shadow_fit,
            "--preview-shadow-split-lambda",
            str(args.shadow_split_lambda),
        ]
        if args.shadow_cascade_transition_width is not None:
            preview_policy_args.extend(
                ["--preview-shadow-cascade-transition-width", str(args.shadow_cascade_transition_width)]
            )
        if args.shadow_distance_fade is not None:
            preview_policy_args.extend(["--preview-shadow-distance-fade", str(args.shadow_distance_fade)])
        if args.shadow_pcf_samples is not None:
            preview_policy_args.extend(["--preview-shadow-pcf-samples", str(args.shadow_pcf_samples)])
        preview_override_args = [*shadow_map_resolution_args, *preview_policy_args]
        for demo_id in ("bistro", "rendering"):
            preview_path = preview_dir / f"{demo_id}-rasterization-{args.width}x{args.height}.png"
            metrics_path = preview_dir / f"{demo_id}-rasterization-metrics.json"
            log_path = log_dir / f"{demo_id}-rasterization-capture.log"
            capture_args = list(preview_override_args)
            if capture_shadow_light_lanes:
                capture_args.extend(["--preview-shadow-light-count", "1"])
            renderer_launches = capture_preview_or_resume(
                editor,
                demo_id,
                "rasterization",
                preview_path,
                args.width,
                args.height,
                args.warmup_frames,
                log_path,
                metrics_path,
                capture_args,
                args.resume,
                tracker,
                (lambda path=metrics_path: assert_p4_primary_shadow_paths(path))
                if capture_p4_delivery_lanes and demo_id == "rendering"
                else None,
            )
            preview_paths.append(preview_path)
            metrics_paths.append(metrics_path)
            log_paths.append(log_path)
        if capture_shadow_light_lanes:
            for light_count in (2, 4):
                preview_path = preview_dir / f"rendering-shadow-lights-{light_count}-{args.width}x{args.height}.png"
                metrics_path = preview_dir / f"rendering-shadow-lights-{light_count}-metrics.json"
                log_path = log_dir / f"rendering-shadow-lights-{light_count}-capture.log"
                renderer_launches = capture_preview_or_resume(
                    editor,
                    "rendering",
                    "rasterization",
                    preview_path,
                    args.width,
                    args.height,
                    args.warmup_frames,
                    log_path,
                    metrics_path,
                    [*preview_override_args, "--preview-shadow-light-count", str(light_count)],
                    args.resume,
                    tracker,
                )
                preview_paths.append(preview_path)
                metrics_paths.append(metrics_path)
                log_paths.append(log_path)
        if capture_p4_delivery_lanes:
            for fit_name, fit_value in P4_FIT_CAPTURE_SCENARIOS:
                preview_path = preview_dir / f"bistro-shadow-fit-{fit_name}-{args.width}x{args.height}.png"
                metrics_path = preview_dir / f"bistro-shadow-fit-{fit_name}-metrics.json"
                log_path = log_dir / f"bistro-shadow-fit-{fit_name}-capture.log"
                renderer_launches = capture_preview_or_resume(
                    editor,
                    "bistro",
                    "rasterization",
                    preview_path,
                    args.width,
                    args.height,
                    args.warmup_frames,
                    log_path,
                    metrics_path,
                    [
                        *shadow_map_resolution_args,
                        *preview_policy_args[2:],
                        "--preview-shadow-fit",
                        fit_value,
                    ],
                    args.resume,
                    tracker,
                )
                preview_paths.append(preview_path)
                metrics_paths.append(metrics_path)
                log_paths.append(log_path)
            for demo_id in ("bistro", "rendering"):
                combined_diagnostic = demo_id == "rendering" and args.capture_shadow_diagnostics
                capture_stem = (
                    f"rendering-shadow-texel-density-c{args.shadow_debug_cascade}"
                    if combined_diagnostic
                    else f"{demo_id}-shadow-medium"
                )
                preview_path = preview_dir / f"{capture_stem}-{args.width}x{args.height}.png"
                metrics_path = preview_dir / f"{capture_stem}-metrics.json"
                log_path = log_dir / f"{capture_stem.replace(f'-c{args.shadow_debug_cascade}', '')}-capture.log"
                capture_args = [
                    "--shadow-map-resolution",
                    "medium",
                    *preview_policy_args,
                    *(["--preview-shadow-caster-fixture"] if demo_id == "rendering" else []),
                ]
                if combined_diagnostic:
                    capture_args.extend(
                        [
                            "--preview-shadow-debug",
                            "texel-density",
                            "--preview-shadow-debug-cascade",
                            str(args.shadow_debug_cascade),
                            "--preview-shadow-debug-light",
                            str(args.shadow_debug_light),
                        ]
                    )
                renderer_launches = capture_preview_or_resume(
                    editor,
                    demo_id,
                    "rasterization",
                    preview_path,
                    args.width,
                    args.height,
                    args.shadow_debug_warmup_frames if combined_diagnostic else args.warmup_frames,
                    log_path,
                    metrics_path,
                    capture_args,
                    args.resume,
                    tracker,
                    (lambda path=metrics_path: assert_p4_caster_fixture(path))
                    if demo_id == "rendering"
                    else None,
                )
                preview_paths.append(preview_path)
                metrics_paths.append(metrics_path)
                log_paths.append(log_path)
        if args.capture_shadow_diagnostics:
            diagnostic_scenarios = (
                P4_SHADOW_DEBUG_CAPTURE_SCENARIOS
                if capture_p4_delivery_lanes
                else tuple(
                    (demo_id, debug_mode, None)
                    for demo_id in ("bistro", "rendering")
                    for debug_mode in SHADOW_DEBUG_CAPTURE_MODES
                )
            )
            for demo_id, debug_mode, light_count in diagnostic_scenarios:
                light_suffix = f"-lights-{light_count}" if light_count and light_count > 1 else ""
                capture_stem = f"{demo_id}-shadow-{debug_mode}{light_suffix}"
                preview_path = (
                    preview_dir
                    / f"{capture_stem}-c{args.shadow_debug_cascade}-{args.width}x{args.height}.png"
                )
                log_path = log_dir / f"{capture_stem}-capture.log"
                diagnostic_args = [
                    *preview_override_args,
                    "--preview-shadow-debug",
                    debug_mode,
                    "--preview-shadow-debug-cascade",
                    str(args.shadow_debug_cascade),
                    "--preview-shadow-debug-light",
                    str(args.shadow_debug_light),
                ]
                if light_count is not None:
                    diagnostic_args.extend(["--preview-shadow-light-count", str(light_count)])
                renderer_launches = capture_preview_or_resume(
                    editor,
                    demo_id,
                    "rasterization",
                    preview_path,
                    args.width,
                    args.height,
                    args.shadow_debug_warmup_frames,
                    log_path,
                    None,
                    diagnostic_args,
                    args.resume,
                    tracker,
                )
                preview_paths.append(preview_path)
                log_paths.append(log_path)

    report_path = milestone_dir / "validation-report.md"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    failed_log_paths = tracker.log_paths("failed")
    log_paths = list(dict.fromkeys([*log_paths, *tracker.log_paths("succeeded")]))
    renderer_launches = tracker.count
    write_report(
        report_path,
        args.milestone,
        editor,
        args.smoke_seconds,
        args.shadow_split_lambda,
        args.shadow_cascade_transition_width,
        args.shadow_distance_fade,
        shadow_fit_report,
        shadow_map_resolution_report,
        preview_paths,
        metrics_paths,
        log_paths,
        failed_log_paths,
        renderer_launches,
        tracker.status_count("succeeded"),
        tracker.status_count("failed"),
    )
    print(f"\nValidation report: {report_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
