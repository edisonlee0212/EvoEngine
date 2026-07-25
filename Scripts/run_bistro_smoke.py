#!/usr/bin/env python3
"""Run the installed Bistro editor with DDGI for a validated 30-second interval."""

from __future__ import annotations

import argparse
import ctypes
from ctypes import wintypes
from datetime import datetime
import os
from pathlib import Path
import queue
import re
import subprocess
import sys
import threading
import time


WM_CLOSE = 0x0010
FATAL_PATTERNS = (
    re.compile(r"Vulkan\s+\[Validation\]-\[Error\]", re.IGNORECASE),
    re.compile(r"\bVK_ERROR_DEVICE_LOST\b", re.IGNORECASE),
    re.compile(r"\bdevice\s+lost\b", re.IGNORECASE),
    re.compile(r"\bunhandled\s+exception\b", re.IGNORECASE),
    re.compile(r"\bfatal(?:\s+error)?\b", re.IGNORECASE),
    re.compile(r"\bassertion\s+failed\b", re.IGNORECASE),
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="RelWithDebInfo", help="Installed build configuration label.")
    parser.add_argument("--duration", type=float, default=30.0, help="Validated run duration after readiness.")
    parser.add_argument("--width", type=int, default=1920, help="Scene-camera render width; must be 1920.")
    parser.add_argument("--height", type=int, default=1080, help="Scene-camera render height; must be 1080.")
    parser.add_argument("--startup-timeout", type=float, default=300.0, help="Seconds allowed to reach readiness.")
    parser.add_argument("--shutdown-timeout", type=float, default=60.0, help="Seconds allowed for graceful shutdown.")
    parser.add_argument("--heartbeat-timeout", type=float, default=5.0, help="Maximum heartbeat silence after readiness.")
    parser.add_argument("--editor", type=Path, help="Installed EvoEngineEditor executable override.")
    return parser.parse_args()


def find_window_for_pid(pid: int) -> int:
    user32 = ctypes.windll.user32
    result = ctypes.c_void_p()

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def enum_window(hwnd: int, _lparam: int) -> bool:
        if not user32.IsWindowVisible(hwnd):
            return True
        window_pid = wintypes.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(window_pid))
        if window_pid.value == pid:
            result.value = hwnd
            return False
        return True

    user32.EnumWindows(enum_window, 0)
    return int(result.value or 0)


def post_close(hwnd: int) -> None:
    if not hwnd or not ctypes.windll.user32.IsWindow(hwnd):
        raise RuntimeError("Bistro editor window disappeared before graceful shutdown.")
    if not ctypes.windll.user32.PostMessageW(hwnd, WM_CLOSE, 0, 0):
        raise ctypes.WinError()


def start_output_reader(
    process: subprocess.Popen[str], output_queue: queue.Queue[tuple[float, str | None]], log_file
) -> threading.Thread:
    def read_output() -> None:
        if process.stdout is not None:
            for line in process.stdout:
                print(line, end="", flush=True)
                log_file.write(line)
                log_file.flush()
                output_queue.put((time.monotonic(), line.rstrip()))
        output_queue.put((time.monotonic(), None))

    thread = threading.Thread(target=read_output, name="bistro-smoke-output", daemon=True)
    thread.start()
    return thread


def force_cleanup(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=10.0)


def run_smoke(args: argparse.Namespace, log_path: Path) -> None:
    if sys.platform != "win32":
        raise RuntimeError("The Bistro smoke runner requires Windows for WM_CLOSE lifecycle validation.")
    if (args.width, args.height) != (1920, 1080):
        raise ValueError("The DDGI Bistro closeout gate requires exactly 1920x1080.")
    if args.duration <= 0 or args.startup_timeout <= 0 or args.shutdown_timeout <= 0 or args.heartbeat_timeout <= 0:
        raise ValueError("Duration and timeout values must be positive.")

    root = repo_root()
    editor = args.editor or root / "out/install/vs2026-x64/bin/EvoEngineEditor.exe"
    editor = editor.resolve()
    if not editor.is_file():
        raise FileNotFoundError(f"Installed editor not found: {editor}")

    installed_imgui_path = editor.parent / "imgui.ini"
    installed_imgui_before = installed_imgui_path.read_bytes() if installed_imgui_path.is_file() else None
    isolated_imgui_path = log_path.with_suffix(".imgui.ini")
    isolated_imgui_path.unlink(missing_ok=True)
    environment = os.environ.copy()
    environment["EVOENGINE_IMGUI_INI_PATH"] = str(isolated_imgui_path)

    command = [
        str(editor),
        "--demo",
        "bistro",
        "--editor",
        "--bistro-smoke",
        "--preview-width",
        str(args.width),
        "--preview-height",
        str(args.height),
    ]
    print(f"Bistro DDGI smoke ({args.config}): {' '.join(command)}")
    print(f"Log: {log_path}")
    output_queue: queue.Queue[tuple[float, str | None]] = queue.Queue()
    with log_path.open("w", encoding="utf-8", newline="") as log_file:
        process = subprocess.Popen(
            command,
            cwd=editor.parent,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )
        reader = start_output_reader(process, output_queue, log_file)
        validation_ready = False
        ddgi_ready = False
        frame_ready = False
        shutdown_complete = False
        last_heartbeat: float | None = None
        fatal_line: str | None = None
        hwnd = 0

        def drain_output() -> None:
            nonlocal validation_ready, ddgi_ready, frame_ready, shutdown_complete, last_heartbeat, fatal_line
            while True:
                try:
                    timestamp, line = output_queue.get_nowait()
                except queue.Empty:
                    return
                if line is None:
                    continue
                validation_ready |= "EVOENGINE_VULKAN_VALIDATION enabled" in line
                ddgi_ready |= "EVOENGINE_BISTRO_DDGI_READY" in line
                frame_ready |= f"EVOENGINE_BISTRO_FRAME_READY resolution={args.width}x{args.height}" in line
                shutdown_complete |= "EVOENGINE_BISTRO_SMOKE_SHUTDOWN_COMPLETE" in line
                if "EVOENGINE_BISTRO_SMOKE_HEARTBEAT" in line:
                    last_heartbeat = timestamp
                if fatal_line is None and any(pattern.search(line) for pattern in FATAL_PATTERNS):
                    fatal_line = line

        try:
            startup_deadline = time.monotonic() + args.startup_timeout
            while True:
                drain_output()
                if fatal_line:
                    raise RuntimeError(f"Fatal runtime output before readiness: {fatal_line}")
                if process.poll() is not None:
                    reader.join(timeout=2.0)
                    drain_output()
                    raise RuntimeError(f"Bistro editor exited before readiness with code {process.returncode}.")
                if not hwnd:
                    hwnd = find_window_for_pid(process.pid)
                if validation_ready and ddgi_ready and frame_ready and hwnd:
                    break
                if time.monotonic() >= startup_deadline:
                    missing = []
                    if not validation_ready:
                        missing.append("Vulkan validation marker")
                    if not ddgi_ready:
                        missing.append("DDGI readiness marker")
                    if not frame_ready:
                        missing.append("1920x1080 frame readiness marker")
                    if not hwnd:
                        missing.append("visible editor window")
                    raise TimeoutError("Bistro smoke startup timed out waiting for " + ", ".join(missing) + ".")
                time.sleep(0.05)

            ready_time = time.monotonic()
            last_heartbeat = last_heartbeat or ready_time
            run_deadline = ready_time + args.duration
            print(f"Bistro DDGI ready; starting {args.duration:g}-second validated interval.")
            while time.monotonic() < run_deadline:
                drain_output()
                if fatal_line:
                    raise RuntimeError(f"Fatal runtime output during smoke interval: {fatal_line}")
                if process.poll() is not None:
                    raise RuntimeError(f"Bistro editor exited during smoke interval with code {process.returncode}.")
                now = time.monotonic()
                if now - last_heartbeat > args.heartbeat_timeout:
                    raise TimeoutError(f"Bistro smoke heartbeat was silent for more than {args.heartbeat_timeout:g} seconds.")
                if not ctypes.windll.user32.IsWindow(hwnd):
                    hwnd = find_window_for_pid(process.pid)
                    if not hwnd:
                        raise RuntimeError("Bistro editor window disappeared during the smoke interval.")
                time.sleep(min(0.05, max(0.0, run_deadline - now)))

            drain_output()
            if fatal_line:
                raise RuntimeError(f"Fatal runtime output at smoke closeout: {fatal_line}")
            post_close(hwnd)
            shutdown_deadline = time.monotonic() + args.shutdown_timeout
            while process.poll() is None and time.monotonic() < shutdown_deadline:
                drain_output()
                if fatal_line:
                    raise RuntimeError(f"Fatal runtime output during shutdown: {fatal_line}")
                time.sleep(0.05)
            if process.poll() is None:
                raise TimeoutError("Bistro editor did not shut down after WM_CLOSE.")
            reader.join(timeout=5.0)
            drain_output()
            if process.returncode != 0:
                raise RuntimeError(f"Bistro editor exited with code {process.returncode} after WM_CLOSE.")
            if fatal_line:
                raise RuntimeError(f"Fatal runtime output during smoke run: {fatal_line}")
            if not shutdown_complete:
                raise RuntimeError("Bistro editor exited without the shutdown-complete marker.")
        except Exception:
            force_cleanup(process)
            reader.join(timeout=5.0)
            drain_output()
            raise

    installed_imgui_after = installed_imgui_path.read_bytes() if installed_imgui_path.is_file() else None
    if installed_imgui_after != installed_imgui_before:
        raise RuntimeError(f"Installed editor ImGui layout changed during Bistro smoke: {installed_imgui_path}")

    print(f"Bistro DDGI smoke passed: {args.width}x{args.height} for {args.duration:g} seconds; log={log_path}")


def main() -> int:
    args = parse_args()
    log_dir = repo_root() / "out/ddgi-smoke"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"bistro-smoke-{datetime.now():%Y%m%d-%H%M%S}.log"
    try:
        run_smoke(args, log_path)
    except Exception as error:
        print(f"Bistro DDGI smoke failed: {error}", file=sys.stderr)
        print(f"Log: {log_path}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
