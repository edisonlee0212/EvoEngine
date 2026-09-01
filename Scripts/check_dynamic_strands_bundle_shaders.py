#!/usr/bin/env python3
"""Verify the deterministic opt-in dynamic-strand bundle shader inventory."""

from pathlib import Path
import sys


EXPECTED = {
    "ApplySlices.slang",
    "BuildCoarseEdges.slang",
    "BuildSliceMembers.slang",
    "BuildSliceRanges.slang",
    "FitSlices.slang",
    "GatherCoupledPairs.slang",
    "ReduceCoarseEdges.slang",
    "SolveCoarseEdges.slang",
    "SolveCoupledPairs.slang",
    "SortCoarseEdges.slang",
    "SortSliceMembers.slang",
}


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    bundle = root / (
        "EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Compute/"
        "DynamicStrands/Constraints/Position/Bundle"
    )
    source = (root / "EvoEngine_Packages/EcoSysLab/src/DsConstraints.cpp").read_text(encoding="utf-8")
    missing = sorted(name for name in EXPECTED if not (bundle / name).is_file())
    unreferenced = sorted(name for name in EXPECTED if f'"{name}"' not in source)
    if missing or unreferenced:
        if missing:
            print("Missing bundle shaders: " + ", ".join(missing))
        if unreferenced:
            print("Unreferenced bundle shaders: " + ", ".join(unreferenced))
        return 1
    print(f"Dynamic-strand bundle shader inventory: {len(EXPECTED)} files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
