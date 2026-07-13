import argparse
import csv
from pathlib import Path


DATE_ORDER = ("2021-07-01", "2021-07-14", "2021-08-18", "2021-08-30", "2021-09-02")
GROWTH_STAGE_BY_DATE = {date: f"GrowthStage{index:02d}" for index, date in enumerate(DATE_ORDER, 1)}

MANUAL_ASSET_ROOT = Path("ManualAssets")
GENERATED_ASSET_ROOT = Path("GeneratedAssets")
MANUAL_4X10_SCENE = MANUAL_ASSET_ROOT / "Scenes" / "Sorghum_4x10_PARBAR.evescene"
MANUAL_10X10_SCENE = MANUAL_ASSET_ROOT / "Scenes" / "Sorghum_10x10.evescene"
GENERATED_DESCRIPTOR_ROOT = GENERATED_ASSET_ROOT / "Descriptors"
GENERATED_SCENE_ROOT = GENERATED_ASSET_ROOT / "Scenes"
GENERATED_REPORT_ROOT = GENERATED_ASSET_ROOT / "Reports"
DEFAULT_PROJECT_ASSETS = Path(__file__).resolve().parents[1] / "Resources" / "DigitalAgricultureProject" / "Assets"
PROJECT_SCENES = {
    "test_lsystem_sorghum.eveproj": (MANUAL_4X10_SCENE, 40),
    "test_lsystem_sorghum_10x10_overlap.eveproj": (MANUAL_10X10_SCENE, 200),
}


def growth_stage(date: str) -> str:
    return GROWTH_STAGE_BY_DATE[date]


def descriptor_path(root: Path, date: str, cultivar: str) -> Path:
    return root / growth_stage(date) / f"{cultivar}.sorghumls"


def four_by_ten_scene(date: str) -> Path:
    return GENERATED_SCENE_ROOT / f"Sorghum_4x10_{growth_stage(date)}.evescene"


def duplicate_asset_handles(assets_root: Path) -> dict[int, list[Path]]:
    handles: dict[int, list[Path]] = {}
    for metadata_path in sorted(assets_root.rglob("*.evefilemeta")):
        for line in metadata_path.read_text(encoding="utf-8").splitlines():
            if line.startswith("asset_handle_:"):
                handles.setdefault(int(line.partition(":")[2]), []).append(metadata_path)
                break
    return {handle: paths for handle, paths in handles.items() if len(paths) > 1}


def asset_handles(assets_root: Path) -> dict[int, Path]:
    handles: dict[int, Path] = {}
    for metadata_path in sorted(assets_root.rglob("*.evefilemeta")):
        for line in metadata_path.read_text(encoding="utf-8").splitlines():
            if line.startswith("asset_handle_:"):
                handles[int(line.partition(":")[2])] = metadata_path
                break
    return handles


def validate_unique_asset_handles(assets_root: Path) -> None:
    duplicates = duplicate_asset_handles(assets_root)
    if duplicates:
        details = "\n".join(
            f"{handle}: " + ", ".join(str(path.relative_to(assets_root)) for path in paths)
            for handle, paths in duplicates.items()
        )
        raise ValueError(f"duplicate asset handles under {assets_root}:\n{details}")


def validate_sidecars(assets_root: Path) -> None:
    roots = (assets_root / MANUAL_ASSET_ROOT, assets_root / GENERATED_ASSET_ROOT)
    missing = []
    for root in roots:
        if not root.is_dir():
            missing.append(root)
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.suffix not in {".evefilemeta", ".evefoldermeta"}:
                sidecar = Path(f"{path}.evefilemeta")
                if not sidecar.is_file():
                    missing.append(sidecar)
                else:
                    declared_name = next(
                        line.partition(":")[2].strip()
                        for line in sidecar.read_text(encoding="utf-8").splitlines()
                        if line.startswith("asset_file_name_:")
                    )
                    if declared_name != path.stem:
                        raise ValueError(f"asset metadata name mismatch: {sidecar}")
            elif path.is_dir():
                sidecar = path.with_name(f"{path.name}.evefoldermeta")
                if not sidecar.is_file():
                    missing.append(sidecar)
    if missing:
        raise ValueError("missing asset sidecar(s): " + ", ".join(str(path) for path in missing))


def metadata_handle(path: Path) -> int:
    for line in Path(f"{path}.evefilemeta").read_text(encoding="utf-8").splitlines():
        if line.startswith("asset_handle_:"):
            return int(line.partition(":")[2])
    raise ValueError(f"missing asset handle: {path}.evefilemeta")


def validate_projects(project_root: Path, assets_root: Path) -> None:
    for project_name, (scene, marker_count) in PROJECT_SCENES.items():
        project = project_root / project_name
        start_handle = next(
            int(line.partition(":")[2])
            for line in project.read_text(encoding="utf-8").splitlines()
            if line.startswith("start_scene_handle:")
        )
        scene_path = assets_root / scene
        if start_handle != metadata_handle(scene_path):
            raise ValueError(f"{project_name} does not start {scene.as_posix()}")
        text = scene_path.read_text(encoding="utf-8")
        if "tn: SorghumLS" in text or "_cluster_" in text:
            raise ValueError(f"manual scene contains SorghumLS plants: {scene.as_posix()}")
        markers = sum(
            line.lstrip().startswith(("- n: BTX_LSystem_", "- n: Pawaga_LSystem_"))
            for line in text.splitlines()
        )
        if markers != marker_count:
            raise ValueError(f"{scene.as_posix()} has {markers} planting markers; expected {marker_count}")


def validate_asset_references(assets_root: Path) -> None:
    handles = asset_handles(assets_root)
    dangling = []
    for path in assets_root.rglob("*"):
        if not path.is_file() or path.suffix in {".evefilemeta", ".evefoldermeta", ".png", ".evemesh", ".evetexture2d"}:
            continue
        lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
        for index, line in enumerate(lines):
            if not line.lstrip().startswith("asset_handle_:"):
                continue
            handle = int(line.partition(":")[2])
            type_name = lines[index + 1].partition(":")[2].strip().strip('"') if index + 1 < len(lines) else ""
            if handle and handle not in handles and type_name != "PostProcessingStack":
                dangling.append(f"{path.relative_to(assets_root)}:{index + 1} -> {handle}")
    if dangling:
        raise ValueError("dangling asset reference(s):\n" + "\n".join(dangling))


def validate_generated_outputs(assets_root: Path) -> None:
    scenes = {four_by_ten_scene(date): 40 for date in DATE_ORDER}
    scenes[GENERATED_SCENE_ROOT / "Sorghum_10x10_Mature.evescene"] = 200
    for relative, expected_count in scenes.items():
        text = (assets_root / relative).read_text(encoding="utf-8")
        count = text.count("tn: SorghumLS")
        if count != expected_count or "_cluster_" in text:
            raise ValueError(f"{relative.as_posix()} has {count} plants; expected {expected_count} without clusters")
    manifest = assets_root / GENERATED_REPORT_ROOT / "field_manifest.csv"
    with manifest.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            for column in ("descriptor_asset_path", "scene_asset_path"):
                path = assets_root / row[column]
                if not path.is_file():
                    raise ValueError(f"manifest references missing asset: {row[column]}")


def validate_asset_layout(assets_root: Path) -> None:
    assets_root = assets_root.resolve()
    validate_unique_asset_handles(assets_root)
    validate_sidecars(assets_root)
    validate_projects(assets_root.parent, assets_root)
    validate_asset_references(assets_root)
    validate_generated_outputs(assets_root)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Validate the sorghum project asset layout.")
    parser.add_argument("assets_root", type=Path, nargs="?", default=DEFAULT_PROJECT_ASSETS)
    args = parser.parse_args()
    validate_asset_layout(args.assets_root)
    print(f"Sorghum asset layout is self-contained under {args.assets_root.resolve()}")
