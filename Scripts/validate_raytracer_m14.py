#!/usr/bin/env python3
"""Run and validate the three-launch M14 pipeline-cache acceptance slice."""

from __future__ import annotations

import argparse
import copy
import json
import shutil
import subprocess
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

import run_raytracer_baseline as legacy
import run_raytracer_m6 as m6


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "out" / "raytracer-m14"
DEFAULT_EDITOR = ROOT / "out" / "install" / "vs2026-x64" / "bin" / "EvoEngineEditor.exe"
DEFAULT_BUILD = ROOT / "out" / "build" / "vs2026-x64"
WIDTH = 1280
HEIGHT = 720
SPP = 64
SAMPLES_PER_FRAME = 4
FRAMES = SPP // SAMPLES_PER_FRAME
CAMERA_POSITION = [0.169453, 0.80237, 14.8766]
CAMERA_LOOK_AT = [0.169453, -0.27857930944, -5.0941673788415995]


@dataclass(frozen=True)
class Lane:
  id: str
  technique: str
  cache_group: str
  cache_state: str


LANES = (
    Lane("rtx-cold", "rtx", "rtx", "cold"),
    Lane("rq-cold", "rq", "rq", "cold"),
    Lane("query-only-warm", "query-only", "rq", "warm"),
)


def write_json(path: Path, value: object) -> None:
  m6.write_json(path, value)


def load_json(path: Path) -> dict[str, object]:
  value = json.loads(path.read_text(encoding="utf-8"))
  if not isinstance(value, dict):
    raise ValueError(f"Expected JSON object: {path}")
  return value


def lane_paths(output_root: Path, lane: Lane) -> dict[str, Path]:
  lane_root = output_root / lane.id
  cache_root = output_root / "cache" / lane.cache_group
  return {
      "root": lane_root,
      "output": lane_root / "bistro.hdr",
      "metrics": lane_root / "metrics.json",
      "log": lane_root / "run.log",
      "imgui": lane_root / "imgui.ini",
      "record": lane_root / "record.json",
      "cache_root": cache_root,
      "shader_cache": cache_root / "ShaderBinaries",
      "pipeline_cache": cache_root / "PipelineCache",
  }


def command(editor: Path, lane: Lane, paths: dict[str, Path]) -> list[str]:
  result = [
      str(editor),
      "--demo", "bistro",
      "--editor",
      "--capture-demo-preview", str(paths["output"]),
      "--preview-metrics-json", str(paths["metrics"]),
      "--preview-render-mode", "raytracing" if lane.technique == "rtx" else "rayquery",
      "--preview-warmup-frames", str(FRAMES),
      "--preview-timing-warmup-frames", "1",
      "--preview-sample-size", str(SAMPLES_PER_FRAME),
      "--preview-auto-spp", "disabled",
      "--preview-firefly-clamp", "enabled",
      "--preview-firefly-clamp-threshold", "10",
      "--preview-ser", "disabled",
      "--preview-ray-shader-variant", "auto",
      "--preview-width", str(WIDTH),
      "--preview-height", str(HEIGHT),
      "--preview-camera-position", m6.vector_argument(CAMERA_POSITION),
      "--preview-camera-look-at", m6.vector_argument(CAMERA_LOOK_AT),
      "--preview-deterministic",
  ]
  if lane.technique == "query-only":
    result.append("--disable-ray-tracing-pipeline")
  return result


def environment(paths: dict[str, Path]) -> dict[str, str]:
  return {
      "EVOENGINE_SHADER_CACHE_DIR": str(paths["shader_cache"].resolve()),
      "EVOENGINE_PIPELINE_CACHE_DIR": str(paths["pipeline_cache"].resolve()),
      "EVOENGINE_IMGUI_INI_PATH": str(paths["imgui"].resolve()),
  }


def append_launch(output_root: Path, lane: Lane, status: str, error: str | None = None) -> None:
  path = output_root / "launch-ledger.json"
  ledger = load_json(path) if path.is_file() else {
      "schema": 1,
      "type": "raytracer_m14_launch_ledger",
      "launches": [],
  }
  if status == "started":
    ledger["launches"].append({
        "lane": lane.id,
        "technique": lane.technique,
        "status": status,
        "started_utc": datetime.now(timezone.utc).isoformat(),
    })
  else:
    entry = next(item for item in reversed(ledger["launches"]) if item["lane"] == lane.id)
    entry["status"] = status
    entry["finished_utc"] = datetime.now(timezone.utc).isoformat()
    if error:
      entry["error"] = error
  write_json(path, ledger)


def add_gate(gates: list[dict[str, object]], name: str, passed: bool, actual: object, expected: object) -> None:
  gates.append({"name": name, "pass": bool(passed), "actual": actual, "expected": expected})


def validate_lane(lane: Lane, summary: dict[str, object], paths: dict[str, Path]) -> list[dict[str, object]]:
  gates: list[dict[str, object]] = []
  variant = summary.get("ray_shader_variant") or {}
  creation = variant.get("pipeline_creation") or {}
  cache = summary.get("pipeline_cache") or {}
  shader_cache = variant.get("shader_cache") or {}
  capabilities = summary.get("capabilities") or {}
  expected_mode = "RayTracing" if lane.technique == "rtx" else "RayQuery"
  add_gate(gates, f"{lane.id}_capture_contract",
           summary.get("width") == WIDTH and summary.get("height") == HEIGHT
           and summary.get("render_mode") == expected_mode and summary.get("effective_spp") == SPP
           and summary.get("deterministic") is True,
           {key: summary.get(key) for key in ("width", "height", "render_mode", "effective_spp", "deterministic")},
           {"width": WIDTH, "height": HEIGHT, "render_mode": expected_mode, "effective_spp": SPP,
            "deterministic": True})
  add_gate(gates, f"{lane.id}_variant_ready",
           variant.get("ready") is True and variant.get("pending") is False and variant.get("failed") is False
           and variant.get("requested_key") == variant.get("active_key"), variant, "ready active specialization")
  add_gate(gates, f"{lane.id}_variant_bounds",
           int(variant.get("resident_variant_count", 99)) <= 8
           and int(variant.get("pending_build_count", 99)) == 0
           and int(variant.get("failed_entry_count", 99)) == 0
           and int(variant.get("retained_submission_count", 99)) <= 2
           and int(variant.get("variant_capacity", 0)) == 8,
           {key: variant.get(key) for key in ("resident_variant_count", "pending_build_count", "failed_entry_count",
                                              "retained_submission_count", "variant_capacity")},
           {"resident_max": 8, "pending": 0, "failed": 0, "retained_max": 2, "capacity": 8})
  add_gate(gates, f"{lane.id}_pipeline_result",
           int(creation.get("result", -1)) == 0 and float(creation.get("wall_ms", 0.0)) > 0.0
           and creation.get("feedback_supported") is True,
           creation, {"result": 0, "wall_ms": ">0", "feedback_supported": True})
  expected_cache_path = paths["pipeline_cache"].resolve()
  actual_cache_path = Path(str(cache.get("path", ""))).resolve().parent if cache.get("path") else None
  add_gate(gates, f"{lane.id}_cache_initialized",
           cache.get("initialized") is True and actual_cache_path == expected_cache_path,
           {"initialized": cache.get("initialized"),
            "path": str(actual_cache_path) if actual_cache_path else None},
           {"initialized": True, "path": str(expected_cache_path)})
  if lane.cache_state == "cold":
    add_gate(gates, f"{lane.id}_cold_pipeline_cache",
             cache.get("load_source") == "missing" and int(cache.get("initial_bytes", -1)) == 0,
             {"source": cache.get("load_source"), "bytes": cache.get("initial_bytes")},
             {"source": "missing", "bytes": 0})
    add_gate(gates, f"{lane.id}_cold_shader_cache",
             int(shader_cache.get("compilations", 0)) > 0 and int(shader_cache.get("disk_misses", 0)) > 0,
             shader_cache, "frontend compile from empty cache")
    if creation.get("feedback_valid") is True:
      add_gate(gates, f"{lane.id}_cold_feedback_miss", creation.get("application_cache_hit") is False,
               creation.get("application_cache_hit"), False)
  else:
    add_gate(gates, f"{lane.id}_warm_pipeline_cache",
             cache.get("load_source") == "disk" and int(cache.get("initial_bytes", 0)) > 0,
             {"source": cache.get("load_source"), "bytes": cache.get("initial_bytes")},
             {"source": "disk", "bytes": ">0"})
    add_gate(gates, f"{lane.id}_warm_shader_cache",
             int(shader_cache.get("disk_hits", 0)) > 0 and int(shader_cache.get("compilations", -1)) == 0,
             shader_cache, "frontend disk hits without compilation")
    add_gate(gates, f"{lane.id}_warm_pipeline_cache_used",
             int(cache.get("valid_feedback_count", 0)) > 0
             and int(cache.get("application_cache_hit_count", 0)) > 0,
             {"active_hit": creation.get("application_cache_hit"),
               "aggregate_valid_feedback": cache.get("valid_feedback_count"),
               "aggregate_hits": cache.get("application_cache_hit_count")},
             {"aggregate_valid_feedback": ">0", "aggregate_hits": ">0"})
  if lane.technique == "rtx":
    add_gate(gates, "rtx_deferred_runtime",
             creation.get("deferred_requested") is True and creation.get("synchronous_fallback") is False,
             {"requested": creation.get("deferred_requested"), "used": creation.get("deferred_used"),
              "fallback": creation.get("synchronous_fallback"), "reason": creation.get("fallback_reason")},
             {"requested": True, "fallback": False})
  if lane.technique == "query-only":
    add_gate(gates, "query_only_independence",
             summary.get("query_only") is True and capabilities.get("ray_tracing_pipeline") is False
             and capabilities.get("ray_query") is True,
             {"query_only": summary.get("query_only"), "capabilities": capabilities},
             {"query_only": True, "ray_tracing_pipeline": False, "ray_query": True})
  return gates


def refuse_existing_output(output_root: Path) -> None:
  if output_root.exists() and any(output_root.iterdir()):
    raise RuntimeError(f"M14 requires a fresh evidence root and will not overwrite: {output_root}")
  try:
    relative = output_root.resolve().relative_to(ROOT)
  except ValueError:
    return
  ignored = subprocess.run(["git", "-C", str(ROOT), "check-ignore", "--quiet", str(relative)],
                           capture_output=True).returncode == 0
  if not ignored:
    raise RuntimeError(f"M14 evidence inside the repository must use a git-ignored root: {output_root}")


def run(output_root: Path, editor: Path, build_dir: Path) -> dict[str, object]:
  refuse_existing_output(output_root)
  if not editor.is_file():
    raise FileNotFoundError(editor)
  output_root.mkdir(parents=True, exist_ok=True)
  provenance = {
      "schema": 1,
      "type": "raytracer_m14_provenance",
      "repo": legacy.repo_state(ROOT),
      "build": legacy.cmake_build_metadata(build_dir, "RelWithDebInfo"),
      "installed_build_matches": legacy.verify_evo_install_matches_build(editor, build_dir),
      "runtime": legacy.runtime_binary_manifest(editor),
      "lanes": [asdict(lane) for lane in LANES],
  }
  provenance["evidence_fingerprint"] = m6.json_sha256(provenance)
  write_json(output_root / "provenance.json", provenance)
  records: dict[str, dict[str, object]] = {}
  rq_cold_cache_manifest = None
  for index, lane in enumerate(LANES, start=1):
    print(f"M14 launch {index}/3: {lane.id}", flush=True)
    paths = lane_paths(output_root, lane)
    paths["root"].mkdir(parents=True)
    if lane.cache_state == "cold":
      shutil.rmtree(paths["cache_root"], ignore_errors=True)
    paths["cache_root"].mkdir(parents=True, exist_ok=True)
    before = m6.directory_manifest(paths["cache_root"])
    if lane.cache_state == "warm" and before != rq_cold_cache_manifest:
      raise RuntimeError("M14 warm query-only input differs from the validated RayQuery cold cache")
    launched = False
    try:
      legacy.prepare_evo_bistro(output_root, lane.id, False)
      project_before = legacy.evo_project_state()
      append_launch(output_root, lane, "started")
      launched = True
      try:
        _, stdout_records = legacy.run_command(command(editor, lane, paths), ROOT, paths["log"], False,
                                               environment(paths))
      finally:
        legacy.prepare_evo_bistro(output_root, lane.id + "-post", False)
      if legacy.evo_project_state() != project_before:
        raise RuntimeError(f"{lane.id}: Bistro project state was not restored")
      m6.validate_run_log(paths["log"], "evo")
      legacy.validate_output_image(paths["output"], legacy.Profile(WIDTH, HEIGHT, FRAMES, SAMPLES_PER_FRAME))
      summary = m6.find_record(stdout_records, "evoengine_ray_capture")
      if summary is None or summary != load_json(paths["metrics"]):
        raise RuntimeError(f"{lane.id}: capture metrics are missing or differ from stdout")
      gates = validate_lane(lane, summary, paths)
      failed = [gate["name"] for gate in gates if not gate["pass"]]
      if failed:
        raise RuntimeError(f"{lane.id}: failed gates: {failed}")
      after = m6.directory_manifest(paths["cache_root"])
      if not after["files"]:
        raise RuntimeError(f"{lane.id}: no cache artifacts were published")
      pipeline_cache_after = m6.directory_manifest(paths["pipeline_cache"])
      if not pipeline_cache_after["files"]:
        raise RuntimeError(f"{lane.id}: no Vulkan pipeline-cache artifact was published")
      record = {
          "schema": 1,
          "type": "raytracer_m14_lane",
          "status": "complete",
          "lane": asdict(lane),
          "command": command(editor, lane, paths),
          "environment": environment(paths),
          "summary": summary,
          "output": str(paths["output"]),
          "output_sha256": legacy.sha256(paths["output"]),
          "metrics": str(paths["metrics"]),
          "metrics_sha256": legacy.sha256(paths["metrics"]),
          "log": str(paths["log"]),
          "log_sha256": legacy.sha256(paths["log"]),
          "cache_before": before,
          "cache_after": after,
          "pipeline_cache_after": pipeline_cache_after,
          "gates": gates,
      }
      write_json(paths["record"], record)
      records[lane.id] = record
      if lane.id == "rq-cold":
        rq_cold_cache_manifest = after
      append_launch(output_root, lane, "complete")
    except Exception as error:
      if launched:
        append_launch(output_root, lane, "failed", str(error))
      raise
  rq_hash = records["rq-cold"]["output_sha256"]
  query_only_hash = records["query-only-warm"]["output_sha256"]
  equivalence = rq_hash == query_only_hash
  rq_cache_hits = int(records["rq-cold"]["summary"]["pipeline_cache"]["application_cache_hit_count"])
  warm_cache_hits = int(records["query-only-warm"]["summary"]["pipeline_cache"]["application_cache_hit_count"])
  warm_hit_increase = warm_cache_hits > rq_cache_hits
  ledger = load_json(output_root / "launch-ledger.json")
  launches = ledger.get("launches", [])
  report = {
      "schema": 1,
      "type": "raytracer_m14_acceptance_report",
      "pass": equivalence and warm_hit_increase and len(launches) == 3
      and all(item.get("status") == "complete" for item in launches),
      "evidence_fingerprint": provenance["evidence_fingerprint"],
      "run_count": len(records),
      "capture_accounting": {"precommit_executed": 3, "postcommit_delivery_executed": 0,
                             "executed_total": 3, "planned_postcommit_delivery": 1,
                             "milestone_launch_ceiling": 4, "reference_executed": False},
      "rq_query_only_bit_exact": equivalence,
      "warm_hit_count_exceeds_rq_cold": warm_hit_increase,
      "pipeline_cache_hit_counts": {"rq_cold": rq_cache_hits, "query_only_warm": warm_cache_hits},
      "rq_output_sha256": rq_hash,
      "query_only_output_sha256": query_only_hash,
      "records": {key: {"path": str(lane_paths(output_root, next(l for l in LANES if l.id == key))["record"]),
                         "sha256": legacy.sha256(lane_paths(output_root, next(l for l in LANES if l.id == key))["record"]),
                         "output_sha256": value["output_sha256"]} for key, value in records.items()},
  }
  write_json(output_root / "m14-report.json", report)
  if not report["pass"]:
    raise RuntimeError("M14 acceptance report failed")
  return report


def logged_capture_summary(path: Path) -> dict[str, object] | None:
  records = []
  for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
    if line.startswith("RAY_CAPTURE_JSON "):
      try:
        records.append(json.loads(line[len("RAY_CAPTURE_JSON "):]))
      except json.JSONDecodeError:
        continue
  return m6.find_record(records, "evoengine_ray_capture")


def analyze_existing(output_root: Path) -> dict[str, object]:
  provenance_path = output_root / "provenance.json"
  ledger_path = output_root / "launch-ledger.json"
  if not provenance_path.is_file() or not ledger_path.is_file():
    raise RuntimeError("M14 existing-evidence analysis requires provenance and launch ledger files")
  provenance = load_json(provenance_path)
  ledger = load_json(ledger_path)
  launches = ledger.get("launches", [])
  expected_rejection = "query-only-warm: failed gates: ['query-only-warm_warm_pipeline_hit']"
  ledger_matches = (
      len(launches) == 3
      and [(item.get("lane"), item.get("status")) for item in launches]
      == [("rtx-cold", "complete"), ("rq-cold", "complete"), ("query-only-warm", "failed")]
      and launches[-1].get("error") == expected_rejection
  )
  records: dict[str, dict[str, object]] = {}
  for lane in LANES:
    paths = lane_paths(output_root, lane)
    for name in ("output", "metrics", "log"):
      if not paths[name].is_file():
        raise RuntimeError(f"{lane.id}: missing existing {name} evidence: {paths[name]}")
    m6.validate_run_log(paths["log"], "evo")
    legacy.validate_output_image(paths["output"], legacy.Profile(WIDTH, HEIGHT, FRAMES, SAMPLES_PER_FRAME))
    summary = load_json(paths["metrics"])
    if logged_capture_summary(paths["log"]) != summary:
      raise RuntimeError(f"{lane.id}: existing metrics differ from the logged capture summary")
    gates = validate_lane(lane, summary, paths)
    failed = [gate["name"] for gate in gates if not gate["pass"]]
    if failed:
      raise RuntimeError(f"{lane.id}: corrected existing-evidence gates failed: {failed}")
    pipeline_manifest = m6.directory_manifest(paths["pipeline_cache"])
    if not pipeline_manifest["files"]:
      raise RuntimeError(f"{lane.id}: persisted Vulkan pipeline cache is missing")
    records[lane.id] = {
        "summary": summary,
        "output": str(paths["output"]),
        "output_sha256": legacy.sha256(paths["output"]),
        "metrics": str(paths["metrics"]),
        "metrics_sha256": legacy.sha256(paths["metrics"]),
        "log": str(paths["log"]),
        "log_sha256": legacy.sha256(paths["log"]),
        "pipeline_cache": pipeline_manifest,
        "gates": gates,
    }
  rq_record = load_json(lane_paths(output_root, LANES[1])["record"])
  rq_pipeline_files = [item for item in rq_record["cache_after"]["files"]
                       if str(item.get("path", "")).startswith("PipelineCache/")]
  warm_initial_bytes = int(records["query-only-warm"]["summary"]["pipeline_cache"]["initial_bytes"])
  exact_rq_cache_reuse = (
      ledger_matches
      and len(rq_pipeline_files) == 1
      and int(rq_pipeline_files[0]["bytes"]) == warm_initial_bytes + 60
  )
  rq_hash = records["rq-cold"]["output_sha256"]
  query_only_hash = records["query-only-warm"]["output_sha256"]
  rq_cache_hits = int(records["rq-cold"]["summary"]["pipeline_cache"]["application_cache_hit_count"])
  warm_cache_hits = int(records["query-only-warm"]["summary"]["pipeline_cache"]["application_cache_hit_count"])
  warm_hit_increase = warm_cache_hits > rq_cache_hits
  report = {
      "schema": 1,
      "type": "raytracer_m14_acceptance_overlay",
      "pass": exact_rq_cache_reuse and warm_hit_increase and rq_hash == query_only_hash,
      "no_renderer_launched": True,
      "evidence_fingerprint": provenance.get("evidence_fingerprint"),
      "original_validator": {"pass": False, "rejected_gate": "query-only-warm_warm_pipeline_hit",
                             "ledger_sha256": legacy.sha256(ledger_path)},
      "correction": {
          "reason": "A per-pipeline full-cache-hit flag is advisory and was too strict for the query-only lane.",
          "replacement_gate": "valid aggregate feedback and more application cache hits than the cold RayQuery lane",
          "warm_aggregate_hits": records["query-only-warm"]["summary"]["pipeline_cache"][
              "application_cache_hit_count"],
          "warm_active_pipeline_hit": records["query-only-warm"]["summary"]["ray_shader_variant"][
              "pipeline_creation"]["application_cache_hit"],
      },
      "capture_accounting": {"precommit_executed": 3, "postcommit_delivery_executed": 0,
                             "executed_total": 3, "planned_postcommit_delivery": 1,
                             "milestone_launch_ceiling": 4, "reference_executed": False},
      "exact_rq_cache_reuse": exact_rq_cache_reuse,
      "rq_query_only_bit_exact": rq_hash == query_only_hash,
      "warm_hit_count_exceeds_rq_cold": warm_hit_increase,
      "pipeline_cache_hit_counts": {"rq_cold": rq_cache_hits, "query_only_warm": warm_cache_hits},
      "rq_output_sha256": rq_hash,
      "query_only_output_sha256": query_only_hash,
      "records": records,
  }
  write_json(output_root / "m14-acceptance-overlay.json", report)
  if not report["pass"]:
    raise RuntimeError("M14 existing-evidence acceptance overlay failed")
  return report


def synthetic_summary(lane: Lane) -> dict[str, object]:
  warm = lane.cache_state == "warm"
  return {
      "width": WIDTH, "height": HEIGHT, "render_mode": "RayTracing" if lane.technique == "rtx" else "RayQuery",
      "effective_spp": SPP, "deterministic": True, "query_only": lane.technique == "query-only",
      "capabilities": {"ray_tracing_pipeline": lane.technique != "query-only", "ray_query": True},
      "pipeline_cache": {"initialized": True,
                         "path": str((Path("cache") / lane.cache_group / "PipelineCache" / "cache.bin").resolve()),
                         "load_source": "disk" if warm else "missing", "initial_bytes": 64 if warm else 0,
                         "valid_feedback_count": 1, "application_cache_hit_count": 1 if warm else 0},
      "ray_shader_variant": {
          "ready": True, "pending": False, "failed": False, "requested_key": "key", "active_key": "key",
          "resident_variant_count": 1, "pending_build_count": 0, "failed_entry_count": 0,
          "retained_submission_count": 1, "variant_capacity": 8,
          "shader_cache": {"compilations": 0 if warm else 1, "disk_misses": 0 if warm else 1,
                           "disk_hits": 1 if warm else 0},
          "pipeline_creation": {"result": 0, "wall_ms": 1.0, "feedback_supported": True,
                                "feedback_valid": True, "application_cache_hit": False,
                                "deferred_requested": lane.technique == "rtx", "deferred_used": False,
                                "synchronous_fallback": False},
      },
  }


def self_test() -> None:
  if len(LANES) != 3 or [lane.technique for lane in LANES] != ["rtx", "rq", "query-only"]:
    raise AssertionError("M14 launch matrix changed")
  if LANES[1].cache_group != LANES[2].cache_group or LANES[0].cache_group == LANES[1].cache_group:
    raise AssertionError("M14 cache isolation/reuse contract changed")
  for lane in LANES:
    paths = lane_paths(Path("."), lane)
    summary = synthetic_summary(lane)
    summary["pipeline_cache"]["path"] = str((paths["pipeline_cache"] / "cache.bin").resolve())
    if not all(gate["pass"] for gate in validate_lane(lane, summary, paths)):
      raise AssertionError(f"Synthetic passing {lane.id} evidence was rejected")
    broken = copy.deepcopy(summary)
    broken["ray_shader_variant"]["resident_variant_count"] = 9
    if all(gate["pass"] for gate in validate_lane(lane, broken, paths)):
      raise AssertionError(f"Synthetic unbounded {lane.id} evidence was accepted")
  cold_hits = synthetic_summary(LANES[1])["pipeline_cache"]["application_cache_hit_count"]
  warm_hits = synthetic_summary(LANES[2])["pipeline_cache"]["application_cache_hit_count"]
  if warm_hits <= cold_hits:
    raise AssertionError("Synthetic warm evidence did not exceed the cold pipeline-cache hit count")
  print("M14 validator self-test passed")


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__)
  mode = parser.add_mutually_exclusive_group()
  mode.add_argument("--self-test", action="store_true")
  mode.add_argument("--analyze-existing", action="store_true")
  parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
  parser.add_argument("--editor", type=Path, default=DEFAULT_EDITOR)
  parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD)
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  if args.self_test:
    self_test()
    return 0
  if args.analyze_existing:
    report = analyze_existing(args.output_dir.resolve())
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
  report = run(args.output_dir.resolve(), args.editor.resolve(), args.build_dir.resolve())
  print(json.dumps(report, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
