"""Check a delivered file manifest, or compare the six native exports of two runs."""
import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--compare", type=Path, help="Compare native outputs with this other dataset")
    args = parser.parse_args()
    if args.compare:
        names = ["skeleton.csv", "nodes.yaml", "flows.yaml", "growth.csv", "leaves.obj", "leaf_instances.csv"]
        folders = sorted(p for p in args.dataset.glob("tree_*") if p.is_dir())
        if not folders or [p.name for p in folders] != sorted(p.name for p in args.compare.glob("tree_*") if p.is_dir()):
            raise ValueError("The two runs have different tree folders")
        for folder in folders:
            for base in [folder, args.compare / folder.name]:
                if not all((base / name).is_file() for name in names[:4]):
                    raise ValueError(f"Missing required native export: {base}")
            if {n for n in names if (folder / n).exists()} != {n for n in names if (args.compare / folder.name / n).exists()}:
                raise ValueError(f"The two runs have different native export files: {folder.name}")
        files = [p / name for p in folders for name in names if (p / name).exists()]
        for path in files:
            relative = path.relative_to(args.dataset)
            if digest(path) != digest(args.compare / relative):
                raise ValueError(f"Native output differs: {relative}")
        print(f"MATCH: {len(files)} native files across {len(folders)} trees are byte-identical.")
    else:
        manifest = json.loads((args.dataset / "sha256.json").read_text())
        for name, expected in manifest.items():
            path = args.dataset / name.replace("\\", "/")
            if digest(path) != expected:
                raise ValueError(f"Hash mismatch: {name}")
        print(f"PASS: all {len(manifest)} listed file hashes match.")


if __name__ == "__main__":
    main()
