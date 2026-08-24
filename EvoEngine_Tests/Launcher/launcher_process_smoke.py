#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import subprocess
import tempfile
from pathlib import Path

from smoke_test_utils import (
    find_window_for_pid,
    kill_new_processes,
    kill_process,
    process_ids,
    run_subtest,
    wait_for_new_process,
    wait_for_window,
    wait_until,
)


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
        def editor_without_project_spawns_launcher() -> None:
            completed = subprocess.run([str(args.editor)], cwd=args.editor.parent, timeout=60)
            if completed.returncode != 0:
                raise RuntimeError(f"Editor without project returned {completed.returncode}, expected 0.")
            wait_for_new_process("EvoEngineLauncher.exe", launcher_before)
            kill_new_processes("EvoEngineLauncher.exe", launcher_before)

        def editor_opens_existing_project() -> None:
            editor = subprocess.Popen([str(args.editor), "--project", str(project_path)], cwd=args.editor.parent)
            try:
                wait_for_window(editor, "Editor with project", timeout=60)
            finally:
                kill_process(editor)

        def editor_player_mode_opens_existing_project() -> None:
            editor = subprocess.Popen([str(args.editor), "--project", str(project_path), "--player"],
                                      cwd=args.editor.parent)
            try:
                wait_for_window(editor, "Player project", timeout=60)
            finally:
                kill_process(editor)

        def editor_headless_mode_initializes_project() -> None:
            with tempfile.TemporaryDirectory(prefix="EvoEngineLauncherHeadlessSmoke_") as temp_dir:
                metadata_only_project = write_metadata_only_project(Path(temp_dir))
                editor = subprocess.Popen([str(args.editor), "--project", str(metadata_only_project), "--headless"],
                                          cwd=args.editor.parent)
                try:
                    wait_until(
                        "headless metadata-only project start_scene_handle",
                        lambda: has_start_scene_handle(metadata_only_project),
                        timeout=60,
                    )
                finally:
                    kill_process(editor)

        def editor_creates_metadata_only_start_scene() -> None:
            with tempfile.TemporaryDirectory(prefix="EvoEngineLauncherProcessSmoke_") as temp_dir:
                metadata_only_project = write_metadata_only_project(Path(temp_dir))
                editor = subprocess.Popen([str(args.editor), "--project", str(metadata_only_project)],
                                          cwd=args.editor.parent)
                try:
                    wait_for_window(editor, "Editor with metadata-only project", timeout=60)
                    wait_until(
                        "metadata-only project start_scene_handle",
                        lambda: has_start_scene_handle(metadata_only_project),
                        timeout=60,
                    )
                finally:
                    kill_process(editor)

        def launcher_open_project_hook_spawns_editor() -> None:
            env = os.environ.copy()
            env["EVOENGINE_LAUNCHER_TEST_OPEN_PROJECT"] = str(project_path)
            editor_before_launch = process_ids("EvoEngineEditor.exe")
            try:
                launcher = subprocess.Popen([str(args.launcher)], cwd=args.launcher.parent, env=env)
                launcher.wait(timeout=60)
                editor_pid = wait_for_new_process("EvoEngineEditor.exe", editor_before_launch)
                wait_until("editor window spawned by launcher", lambda: find_window_for_pid(editor_pid), timeout=60)
            except subprocess.TimeoutExpired as error:
                raise RuntimeError("Launcher did not exit after test open-project hook.") from error
            finally:
                kill_new_processes("EvoEngineEditor.exe", editor_before_launch)

        def launcher_open_demo_hook_spawns_editor_profile() -> None:
            with tempfile.TemporaryDirectory(prefix="EvoEngineLauncherForeignCwd_") as temp_dir:
                foreign_working_directory = Path(temp_dir)
                (foreign_working_directory / "Resources").mkdir()
                env = os.environ.copy()
                env["EVOENGINE_LAUNCHER_TEST_OPEN_DEMO"] = "rendering"
                editor_before_launch = process_ids("EvoEngineEditor.exe")
                demo_app_before_launch = process_ids("DemoApp.exe")
                try:
                    launcher = subprocess.Popen([str(args.launcher)], cwd=foreign_working_directory, env=env)
                    launcher.wait(timeout=60)
                    editor_pid = wait_for_new_process("EvoEngineEditor.exe", editor_before_launch)
                    wait_until("demo profile editor window spawned by launcher", lambda: find_window_for_pid(editor_pid),
                               timeout=60)
                except subprocess.TimeoutExpired as error:
                    raise RuntimeError("Launcher did not exit after test open-demo hook.") from error
                finally:
                    kill_new_processes("EvoEngineEditor.exe", editor_before_launch)
                demo_app_after_launch = process_ids("DemoApp.exe")
                if set(demo_app_after_launch) - set(demo_app_before_launch):
                    raise RuntimeError("Launcher demo hook spawned DemoApp.exe instead of EvoEngineEditor.exe.")

        def launcher_open_ddgi_demo_hook_spawns_editor_profile() -> None:
            with tempfile.TemporaryDirectory(prefix="EvoEngineLauncherDdgiDemoSmoke_") as temp_dir:
                log_path = Path(temp_dir) / "launcher.log"
                env = os.environ.copy()
                env["EVOENGINE_LAUNCHER_TEST_OPEN_DEMO"] = "ddgi"
                env["EVOENGINE_LAUNCHER_TEST_LOG"] = str(log_path)
                editor_before_launch = process_ids("EvoEngineEditor.exe")
                ddgi_app_before_launch = process_ids("DDGIApp.exe")
                try:
                    launcher = subprocess.Popen([str(args.launcher)], cwd=args.launcher.parent, env=env)
                    launcher.wait(timeout=60)
                    editor_pid = wait_for_new_process("EvoEngineEditor.exe", editor_before_launch)
                    wait_until("DDGI demo editor window spawned by launcher", lambda: find_window_for_pid(editor_pid),
                               timeout=60)
                except subprocess.TimeoutExpired as error:
                    raise RuntimeError("Launcher did not exit after DDGI test open-demo hook.") from error
                finally:
                    kill_new_processes("EvoEngineEditor.exe", editor_before_launch)
                ddgi_app_after_launch = process_ids("DDGIApp.exe")
                if set(ddgi_app_after_launch) - set(ddgi_app_before_launch):
                    raise RuntimeError("Launcher DDGI demo hook spawned DDGIApp.exe instead of EvoEngineEditor.exe.")
                log_text = log_path.read_text(encoding="utf-8", errors="ignore")
                if "demo-open:ddgi:EvoEngineEditor:Editor" not in log_text:
                    raise RuntimeError("Launcher DDGI demo hook did not log EvoEngineEditor editor-mode launch.")

        run_subtest("EditorWithoutProject.SpawnsLauncher", editor_without_project_spawns_launcher)
        run_subtest("EditorWithProject.OpensWindow", editor_opens_existing_project)
        run_subtest("EditorPlayerMode.OpensWindow", editor_player_mode_opens_existing_project)
        run_subtest("EditorHeadlessMode.InitializesProject", editor_headless_mode_initializes_project)
        run_subtest("MetadataOnlyProject.PersistsStartScene", editor_creates_metadata_only_start_scene)
        run_subtest("LauncherOpenProjectHook.SpawnsEditor", launcher_open_project_hook_spawns_editor)
        run_subtest("LauncherOpenDemoHook.SpawnsEditorProfile", launcher_open_demo_hook_spawns_editor_profile)
        run_subtest("LauncherOpenDdgiDemoHook.SpawnsEditorProfile", launcher_open_ddgi_demo_hook_spawns_editor_profile)
        return 0
    except Exception:
        return 1
    finally:
        kill_new_processes("EvoEngineLauncher.exe", launcher_before)
        kill_new_processes("EvoEngineEditor.exe", editor_before)


if __name__ == "__main__":
    raise SystemExit(main())
