"""Copy the supplied descriptors and change selected scalar parameters."""
import argparse
import math
from pathlib import Path

import yaml


def main():
    here = Path(__file__).resolve().parent
    defaults = here if (here / "Oak.shoot").exists() else here.parents[1] / "Resources/ZachQSM"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=defaults)
    parser.add_argument("--output", type=Path, required=True, help="New folder for three modified descriptors")
    parser.add_argument("--set", action="append", required=True, metavar="FILE.FIELD=VALUE",
                        help="For example shoot.growth_rate=0.16 or pruning.low_branch_pruning=0.2")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output exists; choose a new folder to preserve earlier settings")
    documents = {kind: yaml.safe_load((args.source / f"Oak.{kind}").read_text())
                 for kind in ["shoot", "foliage", "pruning"]}
    for change in args.set:
        try:
            target, text = change.split("=", 1)
            kind, field = target.split(".", 1)
            old = documents[kind][field]
            value = yaml.safe_load(text)
            if type(old) not in (bool, int, float) or type(value) not in (bool, int, float):
                raise ValueError("Only existing scalar numeric or boolean fields can be changed")
            if isinstance(old, bool) != isinstance(value, bool) or not math.isfinite(value):
                raise ValueError("Use a finite number for a numeric field, or true/false for a boolean field")
            if type(old) is int and type(value) is not int:
                raise ValueError("This field requires an integer")
            documents[kind][field] = value
        except (KeyError, ValueError, yaml.YAMLError) as error:
            parser.error(f"Invalid setting {change!r}: {error}")
    args.output.mkdir(parents=True)
    for kind, data in documents.items():
        (args.output / f"Oak.{kind}").write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    print(f"Saved a separate parameter set: {args.output.resolve()}")
    print("Generate with --shoot, --foliage and --pruning pointing to these three files.")


if __name__ == "__main__":
    main()
