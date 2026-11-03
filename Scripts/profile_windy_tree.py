"""Profile the scripted Windy Tree demo and summarize its CPU/GPU timing captures."""

import argparse
import csv
import os
from pathlib import Path
import statistics
import subprocess


def summarize(directory):
    results = {}
    for path in sorted(directory.glob("*.csv")):
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        wall = [float(row["ms"]) for row in rows if row["domain"] == "frame"]
        if wall:
            results[path.stem] = (rows, wall)
    lines = ["# Windy Tree performance", "", "```text",
             (directory / "configuration.txt").read_text().strip(), "```", "",
             "| Configuration | Frames | Mean frame ms | Median frame ms | FPS |",
             "|---|---:|---:|---:|---:|"]
    order = ["full", "without_plant_visual", "physics_without_branches"]
    for name in order:
        rows, wall = results[name]
        mean = statistics.mean(wall)
        lines.append(f"| {name} | {len(wall)} | {mean:.3f} | {statistics.median(wall):.3f} | {1000 / mean:.2f} |")
    lines += ["", "CPU scopes are inclusive and GPU scopes can nest; do not add these rows together.",
              "GPU pass calls per frame reveal fixed-update duty cycles. Mean pass time is accumulated work divided",
              "by all measured frames; mean per call describes the cost when that pass runs."]
    for name in order:
        rows, wall = results[name]
        lines += ["", f"## {name}"]
        for domain in ("cpu", "gpu"):
            totals = {}
            for row in rows:
                if row["domain"] == domain:
                    total = totals.setdefault(row["name"], [0.0, 0])
                    total[0] += float(row["ms"])
                    total[1] += int(row["calls"])
            lines += ["", f"{domain.upper()}:", "",
                      "| Scope | Mean ms/frame | Calls/frame | Mean ms/call |",
                      "|---|---:|---:|---:|"]
            for label, (milliseconds, calls) in sorted(totals.items(), key=lambda item: -item[1][0])[:18]:
                lines.append(f"| {label} | {milliseconds / len(wall):.3f} | {calls / len(wall):.2f} | {milliseconds / calls:.3f} |")
        spans = [float(row["ms"]) for row in rows if row["domain"] == "gpu_span"]
        lines += ["", f"GPU result coverage: {len(spans)}/{len(wall)} frames."]
    report = directory / "report.md"
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(report)
    print("\n".join(lines[lines.index("| Configuration | Frames | Mean frame ms | Median frame ms | FPS |"):
                                lines.index("CPU scopes are inclusive and GPU scopes can nest; do not add these rows together.")]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--editor", type=Path, help="Installed EvoEngineEditor executable")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--analyze-only", action="store_true")
    parser.add_argument("--canopy-texels", type=float, default=16, help="Maximum projected canopy diameter in shadow texels; zero disables")
    parser.add_argument("--impostor-pixels", type=float, default=3.0, help="Camera branch impostor diameter threshold; zero disables")
    parser.add_argument("--debug-impostor-pixels", type=float, help="Override Plant Visual threshold independently")
    parser.add_argument("--geometry-reference", action="store_true", help="Use original per-node tasks and triangle-local vertices")
    parser.add_argument("--geometry-details", action="store_true", help="Capture procedural camera branch GPU time")
    parser.add_argument("--shadow-reference", action="store_true", help="Use unbatched branch shadows without task culling")
    parser.add_argument("--shadow-details", action="store_true", help="Capture branch shadow GPU time per cascade")
    parser.add_argument("--full-shadows", action="store_true", help="Benchmark full branch shadow geometry")
    parser.add_argument("--gather", action="store_true", help="Benchmark the reference cell-driven coupling")
    parser.add_argument("--full-physics-writes", action="store_true", help="Benchmark full-record solver writes")
    parser.add_argument("--physics-phase-timings", action="store_true", help="Add per-dispatch solver GPU scopes")
    parser.add_argument("--physics-reference", action="store_true", help="Benchmark AoS physics with uncached coefficients and broad barriers")
    parser.add_argument("--full-tessellation", action="store_true", help="Use configured full branch subdivisions")
    parser.add_argument("--physics-tolerance", type=float, default=0.0, help="Physics LOD error as a fraction of branch radius; zero uses all rods")
    parser.add_argument("--cluster-order", type=int, help="Enable subtree physics LOD at this minimum branch order")
    parser.add_argument("--validate-cluster-switches", action="store_true", help="Exercise live coarse/fine switches during warmup")
    args = parser.parse_args()
    if args.cluster_order is not None and not 0 <= args.cluster_order <= 64:
        parser.error("--cluster-order must be between 0 and 64")
    if not 0 <= args.physics_tolerance <= 2:
        parser.error("--physics-tolerance must be between 0 and 2")
    if not 0 <= args.impostor_pixels <= 8:
        parser.error("--impostor-pixels must be between 0 and 8")
    if args.debug_impostor_pixels is not None and not 0 <= args.debug_impostor_pixels <= 8:
        parser.error("--debug-impostor-pixels must be between 0 and 8")
    if not 0 <= args.canopy_texels <= 512:
        parser.error("--canopy-texels must be between 0 and 512")
    directory = args.output.resolve()
    if not args.analyze_only:
        if args.editor is None:
            parser.error("--editor is required unless --analyze-only is used")
        if directory.exists() and any(directory.glob("*.csv")):
            parser.error("Choose a fresh output directory to avoid mixing captures")
        directory.mkdir(parents=True, exist_ok=True)
        editor = args.editor.resolve(strict=True)
        environment = os.environ.copy()
        environment.pop("EVOENGINE_TREE_DEBUG_IMPOSTOR_PIXELS", None)
        if args.debug_impostor_pixels is not None:
            environment["EVOENGINE_TREE_DEBUG_IMPOSTOR_PIXELS"] = str(args.debug_impostor_pixels)
        environment["EVOENGINE_TREE_CANOPY_TEXELS"] = str(args.canopy_texels)
        environment["EVOENGINE_TREE_IMPOSTOR_PIXELS"] = str(args.impostor_pixels)
        environment["EVOENGINE_WINDY_TREE_PROFILE"] = str(directory)
        environment["EVOENGINE_TREE_PHYSICS_TOLERANCE"] = str(args.physics_tolerance)
        environment.pop("EVOENGINE_TREE_CLUSTER_ORDER", None)
        environment.pop("EVOENGINE_TREE_CLUSTER_VALIDATE", None)
        if args.cluster_order is not None:
            environment["EVOENGINE_TREE_CLUSTER_ORDER"] = str(args.cluster_order)
        if args.validate_cluster_switches:
            environment["EVOENGINE_TREE_CLUSTER_VALIDATE"] = "1"
        for name, enabled in (("EVOENGINE_TREE_GEOMETRY_REFERENCE", args.geometry_reference),
                              ("EVOENGINE_TREE_GEOMETRY_PROFILE", args.geometry_details),
                              ("EVOENGINE_TREE_SHADOW_REFERENCE", args.shadow_reference),
                              ("EVOENGINE_TREE_SHADOW_PROFILE", args.shadow_details)):
            environment.pop(name, None)
            if enabled:
                environment[name] = "1"
        environment.pop("EVOENGINE_FULL_BRANCH_SHADOWS", None)
        if args.full_shadows:
            environment["EVOENGINE_FULL_BRANCH_SHADOWS"] = "1"
        environment.pop("EVOENGINE_ROD_AIR_GATHER", None)
        if args.gather:
            environment["EVOENGINE_ROD_AIR_GATHER"] = "1"
        for name, enabled in (("EVOENGINE_TREE_PHYSICS_FULL_WRITES", args.full_physics_writes),
                              ("EVOENGINE_TREE_PHYSICS_PHASE_TIMINGS", args.physics_phase_timings),
                              ("EVOENGINE_TREE_FULL_TESSELLATION", args.full_tessellation),
                              ("EVOENGINE_TREE_PHYSICS_REFERENCE", args.physics_reference)):
            environment.pop(name, None)
            if enabled:
                environment[name] = "1"
        with (directory / "editor.log").open("w", encoding="utf-8") as log:
            subprocess.run([str(editor), "--demo", "windy-tree"], cwd=editor.parent, env=environment,
                           stdout=log, stderr=subprocess.STDOUT, timeout=1800, check=True)
        with (directory / "configuration.txt").open("a", encoding="utf-8") as configuration:
            configuration.write(f"geometry_reference={int(args.geometry_reference)}\ngeometry_details={int(args.geometry_details)}\n")
            configuration.write(f"shadow_reference={int(args.shadow_reference)}\nshadow_details={int(args.shadow_details)}\n")
            configuration.write(f"canopy_texels={args.canopy_texels}\n")
            configuration.write(f"debug_impostor_pixels={args.debug_impostor_pixels if args.debug_impostor_pixels is not None else args.impostor_pixels}\n")
            configuration.write(f"impostor_pixels={args.impostor_pixels}\n")
            configuration.write(f"physics_reference={int(args.physics_reference)}\n")
            configuration.write(f"tessellation={'full' if args.full_tessellation else 'adaptive'}\n")
            configuration.write(f"physics_writes={'full' if args.full_physics_writes else 'motion'}\n")
            configuration.write(f"physics_phase_timings={int(args.physics_phase_timings)}\n")
        if "GPU timestamp capacity exhausted" in (directory / "editor.log").read_text(encoding="utf-8"):
            raise RuntimeError("GPU timestamp scopes were skipped; capture fewer passes before comparing timings")
    summarize(directory)


if __name__ == "__main__":
    main()
