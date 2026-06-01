#!/usr/bin/env python3
"""Configure, build, run, and summarize local EvoEngine tests without opening an IDE."""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_generator() -> str:
    if os.name == "nt":
        return "Visual Studio 18 2026"
    return "Ninja"


def default_build_dir(root: Path) -> Path:
    if os.name == "nt":
        return root / "out" / "build" / "vs2026-x64-tests"
    return root / "out" / "build" / "tests"


def default_artifact_dir(root: Path) -> Path:
    return root / "out" / "test-artifacts" / "latest"


def format_command(command: list[str]) -> str:
    if os.name == "nt":
        return subprocess.list2cmdline(command)
    return shlex.join(command)


def run_step(
    name: str, command: list[str], check: bool = True, env: dict[str, str] | None = None
) -> int:
    print(f"\n==> {name}", flush=True)
    print(format_command(command), flush=True)
    run_env = None
    if env:
        run_env = os.environ.copy()
        run_env.update(env)
    completed = subprocess.run(command, cwd=repo_root(), env=run_env)
    if check and completed.returncode != 0:
        raise SystemExit(completed.returncode)
    return completed.returncode


def clean_build_dir(root: Path, build_dir: Path) -> None:
    if not build_dir.exists():
        return

    allowed_root = (root / "out" / "build").resolve()
    resolved_build_dir = build_dir.resolve()
    try:
        resolved_build_dir.relative_to(allowed_root)
    except ValueError:
        raise SystemExit(
            f"Refusing to clean build directory outside {allowed_root}. "
            "Pass a path under out/build or delete it manually."
        )

    print(f"Cleaning {resolved_build_dir}")
    shutil.rmtree(resolved_build_dir)


def prepare_artifact_dir(root: Path, artifact_dir: Path) -> None:
    resolved_root = root.resolve()
    resolved_artifact_dir = artifact_dir.resolve()
    dangerous_dirs = {
        resolved_root,
        resolved_root.parent,
        Path.home().resolve(),
        Path(resolved_artifact_dir.anchor).resolve(),
    }
    if resolved_artifact_dir in dangerous_dirs:
        raise SystemExit(f"Refusing to clean unsafe artifact directory: {resolved_artifact_dir}")

    if resolved_artifact_dir.exists():
        shutil.rmtree(resolved_artifact_dir)
    resolved_artifact_dir.mkdir(parents=True, exist_ok=True)


def configure(args: argparse.Namespace, root: Path, build_dir: Path) -> None:
    command = [
        "cmake",
        "-S",
        str(root),
        "-B",
        str(build_dir),
        "-G",
        args.generator,
        "-DBUILD_TESTING=ON",
    ]
    if args.architecture and "Visual Studio" in args.generator:
        command.extend(["-A", args.architecture])
    command.extend(args.cmake_arg)
    run_step("Configure", command)


def should_run_render_tests(args: argparse.Namespace) -> bool:
    return args.render_only or not (args.all or args.exclude_render or args.label)


def build(args: argparse.Namespace, build_dir: Path, run_render_tests: bool) -> None:
    target = args.target
    if target is None and run_render_tests:
        target = "EvoEngine_RenderTests"

    command = [
        "cmake",
        "--build",
        str(build_dir),
        "--config",
        args.config,
    ]
    if target:
        command.extend(["--target", target])
    run_step("Build", command)


def ctest_command(
    args: argparse.Namespace, build_dir: Path, report_path: Path, run_render_tests: bool
) -> list[str]:
    command = [
        "ctest",
        "--test-dir",
        str(build_dir),
        "-C",
        args.config,
    ]

    if args.list:
        command.append("-N")
    else:
        command.extend(
            [
                "--output-on-failure",
                "--output-junit",
                str(report_path),
                "--test-output-size-passed",
                "1048576",
                "--test-output-size-failed",
                "1048576",
            ]
        )

    if args.verbose:
        command.append("-V")
    if run_render_tests:
        command.extend(["-L", "render"])
    if args.exclude_render:
        command.extend(["-LE", "render|gpu"])
    if args.label:
        command.extend(["-L", args.label])
    if args.exclude_label:
        command.extend(["-LE", args.exclude_label])

    command.extend(args.ctest_arg)
    return command


def testcase_name(testcase: ET.Element) -> str:
    name = testcase.attrib.get("name", "")
    classname = testcase.attrib.get("classname", "")
    if classname and name and not name.startswith(classname):
        return f"{classname}.{name}"
    return name or classname or "<unnamed>"


def summarize_junit(report_path: Path) -> None:
    if not report_path.exists():
        print("\nNo JUnit report was written.")
        return

    root = ET.parse(report_path).getroot()
    testcases = root.findall(".//testcase")
    if not testcases:
        tests = root.attrib.get("tests", "0")
        print(f"\nTest summary: no test cases reported, tests={tests}")
        print(f"JUnit report: {report_path}")
        return

    rows: list[tuple[str, float, str]] = []
    for testcase in testcases:
        status = "PASS"
        if testcase.find("skipped") is not None:
            status = "SKIP"
        if testcase.find("failure") is not None or testcase.find("error") is not None:
            status = "FAIL"
        elapsed = float(testcase.attrib.get("time", "0") or 0)
        rows.append((status, elapsed, testcase_name(testcase)))

    failed = sum(1 for status, _, _ in rows if status == "FAIL")
    skipped = sum(1 for status, _, _ in rows if status == "SKIP")
    passed = sum(1 for status, _, _ in rows if status == "PASS")

    print("\nTest summary:")
    for status, elapsed, name in rows:
        print(f"  {status:<4} {elapsed:>7.2f}s  {name}")
    print(f"Total: {len(rows)}, passed: {passed}, failed: {failed}, skipped: {skipped}")
    print(f"JUnit report: {report_path}")


def summarize_render_metrics(report_path: Path) -> None:
    if not report_path.exists():
        return

    root = ET.parse(report_path).getroot()
    metric_lines: list[str] = []
    for testcase in root.findall(".//testcase"):
        system_out = testcase.findtext("system-out", default="")
        for line in system_out.splitlines():
            if line.startswith("Render comparison "):
                metric_lines.append(line)

    if not metric_lines:
        return

    print("\nRender metrics:")
    for line in metric_lines:
        print(f"  {line}")


def summarize_visual_artifacts(artifact_dir: Path) -> None:
    print("\nVisual artifacts:")
    if not artifact_dir.exists():
        print("  none")
        return

    artifacts = sorted(path for path in artifact_dir.rglob("*.png") if path.is_file())
    if not artifacts:
        print("  none")
        return

    for artifact in artifacts:
        print(f"  {artifact} ({artifact.stat().st_size} bytes)")


def parse_args(root: Path) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Configure, build, run, and summarize local EvoEngine tests. "
            "By default this runs render/GPU tests; GitHub Actions own format and compilation checks."
        )
    )
    parser.add_argument(
        "-C",
        "--config",
        default="RelWithDebInfo",
        choices=["Debug", "Release", "RelWithDebInfo", "MinSizeRel"],
        help="CMake build configuration.",
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=default_build_dir(root),
        help="CMake build directory.",
    )
    parser.add_argument(
        "--generator",
        default=default_generator(),
        help="CMake generator.",
    )
    parser.add_argument(
        "-A",
        "--architecture",
        default="x64" if os.name == "nt" else "",
        help="CMake generator architecture for Visual Studio generators.",
    )
    parser.add_argument(
        "--target",
        default=None,
        help="Optional build target. Defaults to the generator's normal all target.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=None,
        help="JUnit XML report path. Defaults to <build-dir>/Testing/ctest-results.xml.",
    )
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=default_artifact_dir(root),
        help="Directory for visual test artifacts. Defaults to out/test-artifacts/latest.",
    )
    parser.add_argument("--clean", action="store_true", help="Delete the build directory first.")
    parser.add_argument("--no-configure", action="store_true", help="Skip CMake configure.")
    parser.add_argument("--configure-only", action="store_true", help="Stop after CMake configure.")
    parser.add_argument("--no-build", action="store_true", help="Skip CMake build.")
    parser.add_argument("--build-only", action="store_true", help="Stop after CMake build.")
    parser.add_argument("--list", action="store_true", help="List tests without running them.")
    parser.add_argument("--verbose", action="store_true", help="Pass -V to CTest.")
    parser.add_argument(
        "--all",
        action="store_true",
        help="Run all CTest tests instead of the default local render/GPU tests.",
    )
    parser.add_argument(
        "--render-only",
        action="store_true",
        help="Run only render-labeled tests. This is the default local behavior.",
    )
    parser.add_argument(
        "--accept-render-baseline",
        action="store_true",
        help="Update render golden baselines from the current render output.",
    )
    parser.add_argument(
        "--exclude-render",
        action="store_true",
        help='Exclude render/GPU tests with CTest label regex "render|gpu".',
    )
    parser.add_argument("--label", default="", help="Additional CTest -L label regex.")
    parser.add_argument("--exclude-label", default="", help="Additional CTest -LE label regex.")
    parser.add_argument(
        "--cmake-arg",
        action="append",
        default=[],
        help="Extra argument passed to CMake configure. May be repeated.",
    )
    parser.add_argument(
        "--ctest-arg",
        action="append",
        default=[],
        help="Extra argument passed to CTest. May be repeated.",
    )
    return parser.parse_args()


def main() -> int:
    root = repo_root()
    args = parse_args(root)
    if args.all and args.render_only:
        raise SystemExit("--all and --render-only cannot be used together.")
    if args.render_only and args.exclude_render:
        raise SystemExit("--render-only and --exclude-render cannot be used together.")

    run_render_tests = should_run_render_tests(args)
    build_dir = args.build_dir
    if not build_dir.is_absolute():
        build_dir = root / build_dir
    build_dir = build_dir.resolve()

    report_path = args.report
    if report_path is None:
        report_path = build_dir / "Testing" / "ctest-results.xml"
    elif not report_path.is_absolute():
        report_path = (root / report_path).resolve()

    artifact_dir = args.artifact_dir
    if not artifact_dir.is_absolute():
        artifact_dir = root / artifact_dir
    artifact_dir = artifact_dir.resolve()

    if args.clean:
        clean_build_dir(root, build_dir)

    if not args.no_configure:
        configure(args, root, build_dir)
    if args.configure_only:
        return 0

    if not args.no_build:
        build(args, build_dir, run_render_tests)
    if args.build_only:
        return 0

    report_path.parent.mkdir(parents=True, exist_ok=True)
    if not args.list:
        prepare_artifact_dir(root, artifact_dir)
    command = ctest_command(args, build_dir, report_path, run_render_tests)
    ctest_env = {"EVOENGINE_TEST_ARTIFACT_DIR": str(artifact_dir)}
    if args.accept_render_baseline:
        ctest_env["EVOENGINE_ACCEPT_RENDER_BASELINE"] = "1"
    exit_code = run_step(
        "CTest",
        command,
        check=False,
        env=ctest_env if not args.list else None,
    )
    if not args.list:
        summarize_junit(report_path)
        summarize_render_metrics(report_path)
        summarize_visual_artifacts(artifact_dir)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
