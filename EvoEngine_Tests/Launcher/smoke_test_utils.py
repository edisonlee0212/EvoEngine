#!/usr/bin/env python3
from __future__ import annotations

import csv
import ctypes
import subprocess
import time
from ctypes import wintypes
from pathlib import Path
from typing import Callable, TypeVar


T = TypeVar("T")

user32 = ctypes.windll.user32
if ctypes.sizeof(ctypes.c_void_p) == 8:
    LONG_PTR = ctypes.c_longlong
else:
    LONG_PTR = ctypes.c_long

GWL_STYLE = -16
WS_CAPTION = 0x00C00000
WS_POPUP = 0x80000000
WS_THICKFRAME = 0x00040000
WS_MAXIMIZEBOX = 0x00010000


class RECT(ctypes.Structure):
    _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long), ("right", ctypes.c_long), ("bottom", ctypes.c_long)]


user32.GetWindowLongPtrW.argtypes = [wintypes.HWND, ctypes.c_int]
user32.GetWindowLongPtrW.restype = LONG_PTR


def run_subtest(name: str, action: Callable[[], T]) -> T:
    print(f"[ RUN      ] {name}", flush=True)
    start_time = time.monotonic()
    try:
        result = action()
    except Exception as error:
        elapsed = time.monotonic() - start_time
        print(f"[  FAILED  ] {name} ({elapsed:.2f}s)", flush=True)
        print(error, flush=True)
        raise
    elapsed = time.monotonic() - start_time
    print(f"[       OK ] {name} ({elapsed:.2f}s)", flush=True)
    return result


def wait_until(description: str, predicate: Callable[[], T | None | bool], timeout: float = 10.0) -> T:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value  # type: ignore[return-value]
        time.sleep(0.2)
    raise RuntimeError(f"Timed out waiting for {description}.")


def process_ids(image_name: str) -> set[int]:
    output = subprocess.check_output(
        ["tasklist", "/FI", f"IMAGENAME eq {image_name}", "/FO", "CSV", "/NH"],
        text=True,
        stderr=subprocess.DEVNULL,
    )
    ids: set[int] = set()
    for row in csv.reader(output.splitlines()):
        if len(row) >= 2 and row[0].lower() == image_name.lower():
            ids.add(int(row[1]))
    return ids


def wait_for_new_process(image_name: str, before: set[int], timeout: float = 10.0) -> int:
    return wait_until(
        f"new {image_name} process",
        lambda: next(iter(process_ids(image_name) - before), None),
        timeout,
    )


def kill_new_processes(image_name: str, before: set[int]) -> None:
    for pid in process_ids(image_name) - before:
        subprocess.run(["taskkill", "/PID", str(pid), "/F", "/T"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def kill_process(process: subprocess.Popen[object]) -> None:
    if process.poll() is None:
        subprocess.run(["taskkill", "/PID", str(process.pid), "/F", "/T"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass


def assert_process_stays_alive(process: subprocess.Popen[object], label: str, duration: float = 1.0) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"{label} exited early with code {process.returncode}.")
        time.sleep(0.1)


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


def wait_for_window(process: subprocess.Popen[object], label: str, timeout: float = 15.0) -> tuple[int, RECT]:
    def find_visible_window() -> tuple[int, RECT] | None:
        if process.poll() is not None:
            raise RuntimeError(f"{label} exited early with code {process.returncode}.")
        hwnd = find_window_for_pid(process.pid)
        if not hwnd:
            return None
        rect = RECT()
        user32.GetWindowRect(hwnd, ctypes.byref(rect))
        return hwnd, rect

    return wait_until(f"{label} window", find_visible_window, timeout)


def click(x: int, y: int) -> None:
    user32.SetCursorPos(x, y)
    user32.mouse_event(0x0002, 0, 0, 0, None)
    time.sleep(0.1)
    user32.mouse_event(0x0004, 0, 0, 0, None)


def assert_custom_title_bar_window(hwnd: int, label: str, resizable: bool = True) -> None:
    rect = RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(rect))
    width = rect.right - rect.left
    height = rect.bottom - rect.top
    if width <= 100 or height <= 100:
        raise RuntimeError(f"{label} window dimensions are invalid: {width}x{height}.")
    style = user32.GetWindowLongPtrW(hwnd, GWL_STYLE)
    if not style & WS_CAPTION:
        raise RuntimeError(f"{label} is missing the native caption style required for Windows animations.")
    if style & WS_POPUP:
        raise RuntimeError(f"{label} still has popup window styling.")
    if resizable and not style & WS_THICKFRAME:
        raise RuntimeError(f"{label} is missing a resizable frame.")
    if not resizable and style & WS_THICKFRAME:
        raise RuntimeError(f"{label} unexpectedly has a resizable frame.")
    if not resizable and style & WS_MAXIMIZEBOX:
        raise RuntimeError(f"{label} unexpectedly has a maximize box.")


def log_contains(path: Path, needle: str) -> bool:
    return path.exists() and needle in path.read_text(encoding="utf-8", errors="ignore")


def wait_for_log(path: Path, needle: str, timeout: float = 8.0) -> None:
    wait_until(f"log entry {needle}", lambda: log_contains(path, needle), timeout)
