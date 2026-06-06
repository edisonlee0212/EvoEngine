#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path


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


def write_metadata_only_project(root: Path) -> Path:
    project_dir = root / "MetadataOnlyProject"
    (project_dir / "Assets").mkdir(parents=True)
    project_path = project_dir / "MetadataOnlyProject.eveproj"
    project_path.write_text(
        "application_name: Metadata Only Project\n"
        "preferred_editor: EvoEngineEditor\n"
        "startup_runtime_packages: []\n",
        encoding="utf-8",
    )
    return project_path


def has_start_scene_handle(project_path: Path) -> bool:
    for line in project_path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition(":")
        if key.strip() == "start_scene_handle" and separator:
            try:
                return int(value.strip()) != 0
            except ValueError:
                return False
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--launcher", required=True, type=Path)
    parser.add_argument("--editor", required=True, type=Path)
    parser.add_argument("--project", required=True, type=Path)
    args = parser.parse_args()

    launcher_before = process_ids("EvoEngineLauncher.exe")
    editor_before = process_ids("EvoEngineEditor.exe")
    project_path = args.project.resolve()

    try:
      completed = subprocess.run([str(args.editor)], cwd=args.editor.parent, timeout=20)
      if completed.returncode != 0:
          print(f"Editor without project returned {completed.returncode}, expected 0.")
          return 1
      time.sleep(2)
      if not process_ids("EvoEngineLauncher.exe") - launcher_before:
          print("Editor without project did not spawn EvoEngineLauncher.")
          return 1
      kill_new_processes("EvoEngineLauncher.exe", launcher_before)

      editor = subprocess.Popen([str(args.editor), "--project", str(project_path)], cwd=args.editor.parent)
      time.sleep(4)
      if editor.poll() is not None:
          print(f"Editor with project exited early with code {editor.returncode}.")
          return 1
      kill_process(editor)

      with tempfile.TemporaryDirectory(prefix="EvoEngineLauncherProcessSmoke_") as temp_dir:
          metadata_only_project = write_metadata_only_project(Path(temp_dir))
          editor = subprocess.Popen([str(args.editor), "--project", str(metadata_only_project)], cwd=args.editor.parent)
          time.sleep(6)
          if editor.poll() is not None:
              print(f"Editor with metadata-only project exited early with code {editor.returncode}.")
              return 1
          kill_process(editor)
          if not has_start_scene_handle(metadata_only_project):
              print("Editor did not persist start_scene_handle for metadata-only project.")
              return 1

      env = os.environ.copy()
      env["EVOENGINE_LAUNCHER_TEST_OPEN_PROJECT"] = str(project_path)
      launcher = subprocess.Popen([str(args.launcher)], cwd=args.launcher.parent, env=env)
      try:
          launcher.wait(timeout=20)
      except subprocess.TimeoutExpired:
          print("Launcher did not exit after test open-project hook.")
          return 1
      time.sleep(2)
      if not process_ids("EvoEngineEditor.exe") - editor_before:
          print("Launcher open-project hook did not spawn EvoEngineEditor.")
          return 1
      return 0
    finally:
      kill_new_processes("EvoEngineLauncher.exe", launcher_before)
      kill_new_processes("EvoEngineEditor.exe", editor_before)


if __name__ == "__main__":
    raise SystemExit(main())
