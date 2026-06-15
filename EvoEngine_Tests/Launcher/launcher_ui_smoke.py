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
if ctypes.sizeof(ctypes.c_void_p) == 8:
    LONG_PTR = ctypes.c_longlong
else:
    LONG_PTR = ctypes.c_long

GWL_STYLE = -16
WS_CAPTION = 0x00C00000
WS_THICKFRAME = 0x00040000


class RECT(ctypes.Structure):
    _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long), ("right", ctypes.c_long), ("bottom", ctypes.c_long)]


user32.GetWindowLongPtrW.argtypes = [wintypes.HWND, ctypes.c_int]
user32.GetWindowLongPtrW.restype = LONG_PTR


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


def assert_custom_title_bar_window(hwnd: int, label: str) -> None:
    rect = RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(rect))
    width = rect.right - rect.left
    height = rect.bottom - rect.top
    if width <= 100 or height <= 100:
        raise RuntimeError(f"{label} window dimensions are invalid: {width}x{height}.")
    style = user32.GetWindowLongPtrW(hwnd, GWL_STYLE)
    if style & WS_CAPTION:
        raise RuntimeError(f"{label} still has a native Windows caption.")
    if not style & WS_THICKFRAME:
        raise RuntimeError(f"{label} is missing a resizable frame.")


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
        assert_custom_title_bar_window(hwnd, "Launcher project hub")
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
