#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import subprocess
import tempfile
from pathlib import Path

from smoke_test_utils import (
    assert_process_stays_alive,
    assert_custom_title_bar_window,
    click,
    kill_process,
    log_contains,
    run_subtest,
    user32,
    wait_for_log,
    wait_for_window,
)


def start_launcher(launcher_path: Path, env: dict[str, str]) -> subprocess.Popen[bytes]:
    return subprocess.Popen([str(launcher_path)], cwd=launcher_path.parent, env=env.copy())


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
        hwnd, rect = run_subtest("LauncherWindow.Opens", lambda: wait_for_window(launcher, "Launcher"))
        user32.SetForegroundWindow(hwnd)

        def verify_project_hub_state() -> None:
            wait_for_log(log_path, "mode:hub")
            wait_for_log(log_path, "recent-count:1")
            assert_custom_title_bar_window(hwnd, "Launcher project hub")

        run_subtest("LauncherProjectHub.LoadsRecentProject", verify_project_hub_state)
        center_x = (rect.left + rect.right) // 2
        center_y = (rect.top + rect.bottom) // 2
        click(center_x, center_y)
        run_subtest("LauncherWorkspace.ClickDoesNotExit", lambda: assert_process_stays_alive(launcher, "Launcher"))

        def verify_new_project_packages() -> None:
            if not log_contains(log_path, "package-count:"):
                raise RuntimeError("Launcher did not log package availability.")
            if "template:" in log_path.read_text(encoding="utf-8", errors="ignore"):
                raise RuntimeError("Launcher still logged template availability.")

        run_subtest("LauncherNewProject.ListsPackagesWithoutTemplates", verify_new_project_packages)
        click((rect.left + rect.right) // 2 + 70, (rect.top + rect.bottom) // 2 + 135)
        run_subtest("LauncherProjectHub.InteractionDoesNotExit", lambda: assert_process_stays_alive(launcher, "Launcher"))
        return 0
    except Exception:
        return 1
    finally:
        kill_process(launcher)


if __name__ == "__main__":
    raise SystemExit(main())
