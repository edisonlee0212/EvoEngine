"""One command: native headless growth, meshes, validation and scientific figures."""
import argparse
import csv
import hashlib
import importlib.metadata
import json
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml
import numpy as np

from dataset import process_tree, figures


def sha256(path):
    with Path(path).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def main():
    here = Path(__file__).resolve().parent
    installed = (here / "Oak.shoot").exists()
    repo = here.parents[1]
    defaults = here if installed else repo / "Resources" / "ZachQSM"
    default_exe = here.parent / "ZachQSMTreeDatasetGenerator.exe" if installed else repo / "out/install/zach-headless/bin/ZachQSMTreeDatasetGenerator.exe"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New output directory; existing data is never overwritten")
    parser.add_argument("--count", type=int, default=12)
    parser.add_argument("--seed", type=int, default=1001, help="First seed; each subsequent tree increments it")
    parser.add_argument("--years", type=float, default=5)
    parser.add_argument("--step-days", type=float, default=30)
    parser.add_argument("--shoot", type=Path, default=defaults / "Oak.shoot")
    parser.add_argument("--foliage", type=Path, default=defaults / "Oak.foliage")
    parser.add_argument("--pruning", type=Path, default=defaults / "Oak.pruning")
    parser.add_argument("--leaves", action="store_true", help="Also export basic EcoSysLab leaf planes")
    parser.add_argument("--radial-segments", type=int, default=16, help="Cylinder/junction surface resolution; 32 gives finer surfaces")
    parser.add_argument("--exe", type=Path, default=default_exe)
    parser.add_argument("--postprocess-only", action="store_true", help="Finish mesh/figure export for an existing native run")
    args = parser.parse_args()
    output = args.output.resolve()
    if args.radial_segments < 8 or args.radial_segments > 128:
        parser.error("radial-segments must be between 8 and 128")
    config_path = output / "run.yaml"
    if not args.postprocess_only:
        if output.exists(): parser.error("Output already exists; choose a new folder or use --postprocess-only")
        if not args.exe.is_file(): parser.error(f"Build/install the app first: {args.exe}")
        inputs = {key: getattr(args,key).resolve() for key in ["shoot","foliage","pruning"]}
        for path in inputs.values():
            if not path.is_file(): parser.error(f"Missing descriptor: {path}")
        output.mkdir(parents=True)
        (output / "parameters").mkdir()
        config = dict(output=str(output), count=args.count, seed=args.seed, years=args.years,
                      step_days=args.step_days, leaves=args.leaves)
        for key, path in inputs.items():
            destination = output / "parameters" / path.name
            shutil.copy2(path, destination)
            config[key] = str(destination)
        config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
        provenance = dict(created_utc=datetime.now(timezone.utc).isoformat(), executable_sha256=sha256(args.exe),
                          descriptor_sha256={key:sha256(path) for key,path in inputs.items()},
                          python=sys.version, packages={key:importlib.metadata.version(key) for key in
                                                       ["numpy","trimesh","manifold3d","matplotlib","PyYAML"]})
        provenance["package_sha256"] = {path.name:sha256(path) for path in sorted((args.exe.resolve().parent / "Packages").glob("*.dll"))}
        provenance["script_sha256"] = {path.name:sha256(path) for path in sorted(here.glob("*.py"))}
        identity = args.exe.resolve().parent / "evoengine-build.json"
        if identity.exists(): provenance["engine_build"] = json.loads(identity.read_text(encoding="utf-8"))
        (output / "provenance.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
        command = [str(args.exe.resolve()), "--config", str(config_path)]
        with (output / "native.log").open("w", encoding="utf-8") as log:
            run = subprocess.Popen(command, cwd=args.exe.resolve().parent, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, errors="replace")
            for line in run.stdout:
                print(line, end="", flush=True)
                log.write(line); log.flush()
            if run.wait(): raise RuntimeError(f"Native app failed ({run.returncode}); see {output / 'native.log'}")
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    folders = [output / f"tree_{i+1:03d}" for i in range(config["count"])]
    metrics = []
    for index, folder in enumerate(folders):
        history = np.atleast_1d(np.genfromtxt(folder / "growth.csv", delimiter=",", names=True))
        if not np.isclose(history["age_days"][-1], config["years"]*365, atol=.01):
            raise ValueError(f"{folder.name} did not reach the requested age")
        result = process_tree(folder, args.radial_segments)
        result["seed"] = config["seed"] + index
        result["age_years"] = config["years"]
        metrics.append(result)
    with (output / "summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(metrics[0])); writer.writeheader(); writer.writerows(metrics)
    figures(output, metrics)
    validation = dict(requested=config["count"], completed=len(metrics), all_meshes_watertight=all(m["watertight"] for m in metrics),
                      all_wood_connected=all(m["wood_components"] == 1 for m in metrics),
                      radial_segments=args.radial_segments, units="metres", up_axis="Y", trees=metrics,
                      postprocess_script_sha256={p.name:sha256(p) for p in sorted(here.glob("*.py"))},
                      geometry="Union of native cylinders, labelled parent-child connectors, and endpoint spheres")
    (output / "validation.json").write_text(json.dumps(validation, indent=2), encoding="utf-8")
    hashes = {str(path.relative_to(output)):sha256(path) for path in sorted(output.rglob("*")) if path.is_file() and path.name != "sha256.json"}
    (output / "sha256.json").write_text(json.dumps(hashes, indent=2), encoding="utf-8")
    print(f"Completed {len(metrics)} trees: {output}")


if __name__ == "__main__":
    main()
