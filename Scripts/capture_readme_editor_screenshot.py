#!/usr/bin/env python3
"""Capture the DemoApp editor layout used by the README gallery."""

from __future__ import annotations

import argparse
import ctypes
from ctypes import wintypes
import subprocess
import sys
import textwrap
import threading
import time
from pathlib import Path


DEFAULT_BUILD_DIR = Path("out/build/vs2026-x64")
DEFAULT_ARTIFACT = Path("out/readme-screenshots/RenderingDemo.png")
README_IMAGE = Path("Resources/GitHub/RenderingDemo.png")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="overwrite Resources/GitHub/RenderingDemo.png")
    parser.add_argument("--build", dest="build", action="store_true", default=True, help="build DemoApp first")
    parser.add_argument("--no-build", dest="build", action="store_false", help="skip building DemoApp")
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--config", default="RelWithDebInfo")
    parser.add_argument("--output", type=Path, help="capture output path")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--warmup-frames", type=int, default=30)
    return parser.parse_args()


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def run_build(root: Path, build_dir: Path, config: str) -> None:
    command = ["cmake", "--build", str(root / build_dir), "--config", config, "--target", "DemoApp"]
    subprocess.run(command, cwd=root, check=True)


def demo_app_path(root: Path, build_dir: Path, config: str) -> Path:
    path = root / build_dir / "EvoEngine_App" / config / "DemoApp.exe"
    if not path.exists():
        raise FileNotFoundError(f"DemoApp executable was not found: {path}")
    return path


def yaml_path(path: Path) -> str:
    return path.resolve().as_posix()


def write_run_config(path: Path, ready_file: Path, done_file: Path, width: int, height: int, warmup_frames: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        textwrap.dedent(
            f"""\
            mode: editor_screenshot
            demo_setup: Rendering
            screenshot_width: {width}
            screenshot_height: {height}
            warmup_frames: {warmup_frames}
            max_load_frames: 30000
            ready_file: "{yaml_path(ready_file)}"
            done_file: "{yaml_path(done_file)}"
            """
        ),
        encoding="utf-8",
    )


def read_process_output(process: subprocess.Popen[str], lines: list[str]) -> threading.Thread:
    def reader() -> None:
        if process.stdout is None:
            return
        for line in process.stdout:
            lines.append(line.rstrip())
            print(line, end="")

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    return thread


def wait_for_file(path: Path, process: subprocess.Popen[str], timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.exists():
            return
        if process.poll() is not None:
            raise RuntimeError(f"DemoApp exited before creating {path}")
        time.sleep(0.1)
    raise TimeoutError(f"Timed out waiting for {path}")


def find_main_window(process_id: int, timeout: float):
    user32 = ctypes.windll.user32
    user32.EnumWindows.argtypes = [ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM), wintypes.LPARAM]
    user32.EnumWindows.restype = wintypes.BOOL
    user32.IsWindowVisible.argtypes = [wintypes.HWND]
    user32.IsWindowVisible.restype = wintypes.BOOL
    user32.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
    user32.GetWindowThreadProcessId.restype = wintypes.DWORD
    user32.GetWindowTextLengthW.argtypes = [wintypes.HWND]
    user32.GetWindowTextLengthW.restype = ctypes.c_int
    user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    user32.GetWindowTextW.restype = ctypes.c_int

    windows: list[tuple[int, str]] = []

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def callback(hwnd, _):
        window_process_id = wintypes.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(window_process_id))
        if window_process_id.value == process_id and user32.IsWindowVisible(hwnd):
            title_length = user32.GetWindowTextLengthW(hwnd)
            title = ctypes.create_unicode_buffer(title_length + 1)
            user32.GetWindowTextW(hwnd, title, title_length + 1)
            windows.append((int(hwnd), title.value))
        return True

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        windows.clear()
        user32.EnumWindows(callback, 0)
        if windows:
            return next((hwnd for hwnd, title in windows if title), windows[0][0])
        time.sleep(0.1)
    raise TimeoutError(f"Timed out finding DemoApp window for pid {process_id}")


def work_area_origin() -> tuple[int, int]:
    user32 = ctypes.windll.user32

    class Rect(ctypes.Structure):
        _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long), ("right", ctypes.c_long), ("bottom", ctypes.c_long)]

    rect = Rect()
    if user32.SystemParametersInfoW(0x0030, 0, ctypes.byref(rect), 0):
        return rect.left, rect.top
    return 0, 0


def position_window(hwnd, width: int, height: int) -> tuple[int, int, int, int]:
    user32 = ctypes.windll.user32
    user32.SetProcessDPIAware()
    user32.ShowWindow.argtypes = [wintypes.HWND, ctypes.c_int]
    user32.SetWindowPos.argtypes = [
        wintypes.HWND,
        wintypes.HWND,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_uint,
    ]
    user32.SetForegroundWindow.argtypes = [wintypes.HWND]
    x, y = work_area_origin()
    user32.ShowWindow(wintypes.HWND(hwnd), 9)
    user32.SetWindowPos(wintypes.HWND(hwnd), wintypes.HWND(-1), x, y, width, height, 0x0040)
    user32.SetForegroundWindow(wintypes.HWND(hwnd))
    time.sleep(1.0)
    return x, y, width, height


def capture_region(output_path: Path, x: int, y: int, width: int, height: int) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    script = textwrap.dedent(
        f"""
        Add-Type -AssemblyName System.Drawing
        $bitmap = New-Object System.Drawing.Bitmap {width}, {height}
        $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
        $graphics.CopyFromScreen({x}, {y}, 0, 0, $bitmap.Size)
        $bitmap.Save('{str(output_path.resolve()).replace("'", "''")}', [System.Drawing.Imaging.ImageFormat]::Png)
        $graphics.Dispose()
        $bitmap.Dispose()
        """
    )
    subprocess.run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", script], check=True)


def main() -> int:
    if sys.platform != "win32":
        print("README editor screenshot capture is Windows-only because it captures a visible desktop window.")
        return 0

    args = parse_args()
    root = repo_root()
    output = root / (README_IMAGE if args.apply else args.output or DEFAULT_ARTIFACT)
    build_dir = args.build_dir if args.build_dir.is_absolute() else root / args.build_dir

    if args.build:
        run_build(root, build_dir, args.config)

    app_path = demo_app_path(root, build_dir, args.config)
    run_dir = root / "out/readme-screenshots/run"
    ready_file = run_dir / "ready.txt"
    done_file = run_dir / "done.txt"
    run_config = run_dir / "DemoApp.editor-screenshot.yaml"
    for marker in (ready_file, done_file):
        marker.unlink(missing_ok=True)
    write_run_config(run_config, ready_file, done_file, args.width, args.height, args.warmup_frames)

    process = subprocess.Popen(
        [str(app_path), "--run-config", str(run_config)],
        cwd=root,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    output_lines: list[str] = []
    read_process_output(process, output_lines)
    try:
        wait_for_file(ready_file, process, args.timeout)
        hwnd = find_main_window(process.pid, 15.0)
        x, y, width, height = position_window(hwnd, args.width, args.height)
        capture_region(output, x, y, width, height)
        done_file.write_text("done\n", encoding="utf-8")
        exit_code = process.wait(timeout=30.0)
        if exit_code != 0:
            raise RuntimeError(f"DemoApp exited with code {exit_code}")
    except Exception:
        process.terminate()
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            process.kill()
        raise

    print(f"Captured README editor screenshot: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
