"""Helpers for locating vcpkg and enabling optional packages such as CGAL."""

from __future__ import annotations

import os
import re
from pathlib import Path

DEFAULT_VCPKG_ROOT_CANDIDATES = (
    Path(r"C:\src\vcpkg"),
    Path(r"C:\vcpkg"),
)


def resolve_vcpkg_root(explicit: Path | None = None) -> Path | None:
    """Return a vcpkg root that contains scripts/buildsystems/vcpkg.cmake, if any."""
    candidates: list[Path] = []
    if explicit is not None:
        candidates.append(explicit)
    env_root = os.environ.get("VCPKG_ROOT", "").strip()
    if env_root:
        candidates.append(Path(env_root))
    candidates.extend(DEFAULT_VCPKG_ROOT_CANDIDATES)

    seen: set[Path] = set()
    for candidate in candidates:
        try:
            root = candidate.expanduser().resolve()
        except OSError:
            continue
        if root in seen:
            continue
        seen.add(root)
        if (root / "scripts" / "buildsystems" / "vcpkg.cmake").is_file():
            return root
    return None


def vcpkg_toolchain_file(vcpkg_root: Path) -> Path:
    return vcpkg_root / "scripts" / "buildsystems" / "vcpkg.cmake"


def vcpkg_has_cgal(vcpkg_root: Path, triplet: str = "x64-windows") -> bool:
    share = vcpkg_root / "installed" / triplet / "share" / "cgal"
    return (share / "CGALConfig.cmake").is_file() or (share / "cgal-config.cmake").is_file()


def cmake_cache_cgal_dir(build_dir: Path) -> str | None:
    cache = build_dir / "CMakeCache.txt"
    if not cache.is_file():
        return None
    match = re.search(r"^CGAL_DIR:PATH=(.*)$", cache.read_text(encoding="utf-8", errors="ignore"), re.MULTILINE)
    if not match:
        return None
    return match.group(1).strip()


def cmake_cache_needs_cgal_reconfigure(build_dir: Path, vcpkg_root: Path | None) -> bool:
    """True when the build cache lacks CGAL but the local vcpkg install provides it."""
    if vcpkg_root is None or not vcpkg_has_cgal(vcpkg_root):
        return False
    cgal_dir = cmake_cache_cgal_dir(build_dir)
    return cgal_dir is None or cgal_dir == "" or cgal_dir == "CGAL_DIR-NOTFOUND"


def cmake_cache_toolchain_file(build_dir: Path) -> str | None:
    cache = build_dir / "CMakeCache.txt"
    if not cache.is_file():
        return None
    match = re.search(
        r"^CMAKE_TOOLCHAIN_FILE:FILEPATH=(.*)$",
        cache.read_text(encoding="utf-8", errors="ignore"),
        re.MULTILINE,
    )
    if not match:
        return None
    return match.group(1).strip()


def cmake_vcpkg_configure_args(vcpkg_root: Path) -> list[str]:
    toolchain = vcpkg_toolchain_file(vcpkg_root)
    return [f"-DCMAKE_TOOLCHAIN_FILE={toolchain}"]


def resolve_optional_cgal_vcpkg(
    explicit: Path | None = None,
) -> tuple[Path | None, list[str]]:
    """
    Resolve vcpkg only when it can provide CGAL.

    Returns ``(vcpkg_root, configure_args)``. When vcpkg or CGAL is missing, both are
    empty/None so configure stays unchanged; CMake's existing ``find_package(CGAL)``
    WARNING remains the only notification.
    """
    vcpkg_root = resolve_vcpkg_root(explicit)
    if vcpkg_root is None or not vcpkg_has_cgal(vcpkg_root):
        return None, []
    return vcpkg_root, cmake_vcpkg_configure_args(vcpkg_root)


def prepare_build_dir_for_toolchain(build_dir: Path, toolchain: Path) -> bool:
    """
    Drop CMakeCache.txt when the toolchain file changes so configure can succeed.

    Returns True when the cache was removed.
    """
    cache = build_dir / "CMakeCache.txt"
    if not cache.is_file():
        return False
    previous = cmake_cache_toolchain_file(build_dir)
    desired = str(toolchain.resolve())
    if previous is None or Path(previous).resolve() != Path(desired).resolve():
        cache.unlink()
        return True
    return False
