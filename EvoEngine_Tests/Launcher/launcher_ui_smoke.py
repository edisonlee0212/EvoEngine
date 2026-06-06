#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ctypes
import os
import subprocess
import tempfile
import time
from ctypes import wintypes
from pathlib import Path


user32 = ctypes.windll.user32
gdi32 = ctypes.windll.gdi32


class RECT(ctypes.Structure):
    _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long), ("right", ctypes.c_long), ("bottom", ctypes.c_long)]


class BITMAPINFOHEADER(ctypes.Structure):
    _fields_ = [
        ("biSize", wintypes.DWORD),
        ("biWidth", ctypes.c_long),
        ("biHeight", ctypes.c_long),
        ("biPlanes", wintypes.WORD),
        ("biBitCount", wintypes.WORD),
        ("biCompression", wintypes.DWORD),
        ("biSizeImage", wintypes.DWORD),
        ("biXPelsPerMeter", ctypes.c_long),
        ("biYPelsPerMeter", ctypes.c_long),
        ("biClrUsed", wintypes.DWORD),
        ("biClrImportant", wintypes.DWORD),
    ]


class BITMAPINFO(ctypes.Structure):
    _fields_ = [("bmiHeader", BITMAPINFOHEADER), ("bmiColors", wintypes.DWORD * 1)]


user32.GetWindowDC.argtypes = [wintypes.HWND]
user32.GetWindowDC.restype = wintypes.HDC
user32.ReleaseDC.argtypes = [wintypes.HWND, wintypes.HDC]
gdi32.CreateCompatibleDC.argtypes = [wintypes.HDC]
gdi32.CreateCompatibleDC.restype = wintypes.HDC
gdi32.CreateCompatibleBitmap.argtypes = [wintypes.HDC, ctypes.c_int, ctypes.c_int]
gdi32.CreateCompatibleBitmap.restype = wintypes.HBITMAP
gdi32.SelectObject.argtypes = [wintypes.HDC, wintypes.HGDIOBJ]
gdi32.SelectObject.restype = wintypes.HGDIOBJ
gdi32.BitBlt.argtypes = [
    wintypes.HDC,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    wintypes.HDC,
    ctypes.c_int,
    ctypes.c_int,
    wintypes.DWORD,
]
gdi32.GetDIBits.argtypes = [
    wintypes.HDC,
    wintypes.HBITMAP,
    wintypes.UINT,
    wintypes.UINT,
    wintypes.LPVOID,
    ctypes.POINTER(BITMAPINFO),
    wintypes.UINT,
]
gdi32.DeleteObject.argtypes = [wintypes.HGDIOBJ]
gdi32.DeleteDC.argtypes = [wintypes.HDC]


def wait_for_window(process: subprocess.Popen[bytes], timeout: float = 15.0) -> tuple[int, RECT]:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Launcher exited early with code {process.returncode}.")
        hwnd = find_window_for_pid(process.pid)
        if hwnd:
            rect = RECT()
            user32.GetWindowRect(hwnd, ctypes.byref(rect))
            return hwnd, rect
        time.sleep(0.25)
    raise RuntimeError("Timed out waiting for launcher window.")


def find_window_for_pid(pid: int) -> int:
    hwnd_result = ctypes.c_void_p()

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def enum_proc(hwnd: int, lparam: int) -> bool:
        if not user32.IsWindowVisible(hwnd):
            return True
        window_pid = wintypes.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(window_pid))
        if window_pid.value == pid:
            hwnd_result.value = hwnd
            return False
        return True

    user32.EnumWindows(enum_proc, 0)
    return int(hwnd_result.value or 0)


def click(x: int, y: int) -> None:
    user32.SetCursorPos(x, y)
    user32.mouse_event(0x0002, 0, 0, 0, None)
    time.sleep(0.1)
    user32.mouse_event(0x0004, 0, 0, 0, None)


def capture_window_pixels(hwnd: int) -> tuple[int, int, bytes]:
    rect = RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(rect))
    width = rect.right - rect.left
    height = rect.bottom - rect.top
    if width <= 0 or height <= 0:
        return width, height, b""

    window_dc = user32.GetWindowDC(hwnd)
    memory_dc = gdi32.CreateCompatibleDC(window_dc)
    bitmap = gdi32.CreateCompatibleBitmap(window_dc, width, height)
    old_bitmap = gdi32.SelectObject(memory_dc, bitmap)
    try:
        gdi32.BitBlt(memory_dc, 0, 0, width, height, window_dc, 0, 0, 0x00CC0020)
        info = BITMAPINFO()
        info.bmiHeader.biSize = ctypes.sizeof(BITMAPINFOHEADER)
        info.bmiHeader.biWidth = width
        info.bmiHeader.biHeight = -height
        info.bmiHeader.biPlanes = 1
        info.bmiHeader.biBitCount = 32
        info.bmiHeader.biCompression = 0
        buffer = ctypes.create_string_buffer(width * height * 4)
        rows = gdi32.GetDIBits(memory_dc, bitmap, 0, height, buffer, ctypes.byref(info), 0)
        if rows != height:
            raise RuntimeError("Failed to capture launcher window pixels.")
        return width, height, bytes(buffer)
    finally:
        gdi32.SelectObject(memory_dc, old_bitmap)
        gdi32.DeleteObject(bitmap)
        gdi32.DeleteDC(memory_dc)
        user32.ReleaseDC(hwnd, window_dc)


def assert_nonblank_window(hwnd: int, label: str) -> None:
    width, height, pixels = capture_window_pixels(hwnd)
    if width <= 100 or height <= 100 or not pixels:
        raise RuntimeError(f"{label} screenshot dimensions are invalid: {width}x{height}.")
    stride = max(4, len(pixels) // 4096 // 4 * 4)
    samples = {pixels[i:i + 3] for i in range(0, len(pixels), stride)}
    if len(samples) < 8:
        raise RuntimeError(f"{label} screenshot appears blank.")


def log_contains(path: Path, needle: str) -> bool:
    return path.exists() and needle in path.read_text(encoding="utf-8", errors="ignore")


def wait_for_log(path: Path, needle: str, timeout: float = 8.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if log_contains(path, needle):
            return True
        time.sleep(0.2)
    return False


def start_launcher(launcher_path: Path, env: dict[str, str]) -> subprocess.Popen[bytes]:
    return subprocess.Popen([str(launcher_path)], cwd=launcher_path.parent, env=env.copy())


def terminate(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is None:
        subprocess.run(["taskkill", "/PID", str(process.pid), "/F", "/T"], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--launcher", required=True, type=Path)
    args = parser.parse_args()

    temp_dir = Path(tempfile.mkdtemp(prefix="EvoEngineLauncherUiSmoke_"))
    log_path = temp_dir / "launcher.log"
    env = os.environ.copy()
    env["LOCALAPPDATA"] = str(temp_dir / "LocalAppData")
    env["EVOENGINE_LAUNCHER_TEST_LOG"] = str(log_path)
    env["EVOENGINE_LAUNCHER_TEST_PARENT_FOLDER"] = str(temp_dir)
    recent_project = temp_dir / "RecentProject.eveproj"
    recent_project.write_text("application_name: Recent Smoke Project\n", encoding="utf-8")
    settings_path = Path(env["LOCALAPPDATA"]) / "EvoEngine" / "EditorSettings.yaml"
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    settings_path.write_text(f"recent_projects:\n  - {recent_project}\n", encoding="utf-8")

    launcher = start_launcher(args.launcher, env)
    try:
        hwnd, rect = wait_for_window(launcher)
        user32.SetForegroundWindow(hwnd)
        if not wait_for_log(log_path, "mode:hub"):
            print("Launcher did not log project-hub mode.")
            return 1
        if not wait_for_log(log_path, "recent-count:1"):
            print("Launcher did not load seeded recent project.")
            return 1
        assert_nonblank_window(hwnd, "Launcher project hub")
        center_x = (rect.left + rect.right) // 2
        center_y = (rect.top + rect.bottom) // 2
        click(center_x, center_y)
        time.sleep(1)
        if launcher.poll() is not None:
            print(f"Launcher exited after workspace click with code {launcher.returncode}.")
            return 1

        if not log_contains(log_path, "template:Generic:available"):
            print("Launcher did not log Generic template availability.")
            return 1
        if "template:" not in log_path.read_text(encoding="utf-8", errors="ignore"):
            print("Launcher did not log template availability.")
            return 1
        click((rect.left + rect.right) // 2 + 70, (rect.top + rect.bottom) // 2 + 135)
        time.sleep(1)
        if launcher.poll() is not None:
            print(f"Launcher exited after project-hub interaction with code {launcher.returncode}.")
            return 1
        return 0
    finally:
        terminate(launcher)


if __name__ == "__main__":
    raise SystemExit(main())
