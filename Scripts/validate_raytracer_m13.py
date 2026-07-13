#!/usr/bin/env python3
"""Run and validate the three-capture M13 dynamic-AS acceptance slice."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import subprocess
import tempfile
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import run_raytracer_baseline as legacy
import run_raytracer_m6 as m6


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "out" / "raytracer-m13"
DEFAULT_EDITOR = ROOT / "out" / "install" / "vs2026-x64" / "bin" / "EvoEngineEditor.exe"
DEFAULT_BUILD = ROOT / "out" / "build" / "vs2026-x64"
M6_REPORT = ROOT / "out" / "raytracer-m6" / "report.json"
M6_REPORT_SHA256 = "4794d83ec764c2477e598ce66e4ed3d6bf89c214fe7d00e62f74029739a924a2"
M6_RECORD_SHA256 = {
    "rtx": "c1a93fa2cdba3254c2c40e76b101f5e43dadc48d9981fe738da9379d6fed575b",
    "rq": "9882cf9a9d6866f7bf477e5f5468d97137ddae54b01fd525c8d7c3bf46ade298",
    "query-only": "73bbeb3d0cecaed418baf80622a050a382082dcb95dc7f2827f95688dbef8f8c",
}
TECHNIQUES = ("rtx", "rq", "query-only")
TARGETS = {
    "blas_update_samples": 30,
    "blas_update_total_ms_max": 3.20,
    "tlas_update_samples": 15,
    "tlas_update_total_ms_max": 0.60,
    "wall_seconds_max": {"rtx": 0.48, "rq": 0.43, "query-only": 0.43},
}
M13_SOURCE_FILES = (
    "EvoEngine_App/include/DemoScene.hpp",
    "EvoEngine_App/src/DemoScene.cpp",
    "EvoEngine_App/src/EvoEngineEditor.cpp",
    "EvoEngine_SDK/include/Layers/RenderLayer.hpp",
    "EvoEngine_SDK/include/Rendering/Geometry/Mesh.hpp",
    "EvoEngine_SDK/include/Rendering/Geometry/MikkTangentSpace.hpp",
    "EvoEngine_SDK/include/Rendering/Geometry/MorphTarget.hpp",
    "EvoEngine_SDK/include/Rendering/Geometry/SkinnedMesh.hpp",
    "EvoEngine_SDK/include/Rendering/Platform/GraphicsResources.hpp",
    "EvoEngine_SDK/include/Rendering/RenderInstances/RenderInstanceStorage.hpp",
    "EvoEngine_SDK/include/Rendering/Renderer/MeshRenderer.hpp",
    "EvoEngine_SDK/include/Rendering/Renderer/SkinnedMeshRenderer.hpp",
    "EvoEngine_SDK/src/Application.cpp",
    "EvoEngine_SDK/src/GraphicsResources.cpp",
    "EvoEngine_SDK/src/Mesh.cpp",
    "EvoEngine_SDK/src/MeshRenderer.cpp",
    "EvoEngine_SDK/src/MikkTangentSpace.cpp",
    "EvoEngine_SDK/src/Prefab.cpp",
    "EvoEngine_SDK/src/RenderInstanceStorage.cpp",
    "EvoEngine_SDK/src/RenderLayer.cpp",
    "EvoEngine_SDK/src/SkinnedMesh.cpp",
    "EvoEngine_SDK/src/SkinnedMeshRenderer.cpp",
    "EvoEngine_Tests/Core/GltfMaterialConversionTest.cpp",
    "EvoEngine_Tests/Core/GltfRayTracingMaterialShaderTest.cpp",
    "EvoEngine_Tests/Core/RayTracingSkinnedInstanceTest.cpp",
    "EvoEngine_Tests/Core/SerializationRegistryTest.cpp",
    "docs/raytracer-m13-dynamic-geometry.md",
    "Scripts/run_raytracer_baseline.py",
    "Scripts/run_raytracer_m6.py",
    "Scripts/validate_raytracer_m13.py",
)
VALIDATOR_SOURCE = "Scripts/validate_raytracer_m13.py"
APPROVED_VALIDATOR_NORMALIZED_SHA256 = "294b863d4abe559414fec2b698ce290a0c4ec55fddf8fa6e1622d8666b030967"
POLICY_DOCUMENT = "docs/raytracer-m13-dynamic-geometry.md"
CAPTURED_POLICY_SOURCE_SETS = {
    "original": {
        VALIDATOR_SOURCE: {"bytes": 21932, "sha256": "280bd69a2ac3aa3da41caad98c7279279c44ad54af94905c79d9b3287eda37e6"},
        POLICY_DOCUMENT: {"bytes": 2973, "sha256": "7090ee1b6b576b51e28437885f5ce3150ce4a8c92c786e5d645a47ac7e1eeeac"},
    },
    "replacement": {
        VALIDATOR_SOURCE: {"bytes": 27081, "sha256": "6346700166e99001f9c2e8a5012015a2d669cde85a3e29351931c7a15201ba98"},
        POLICY_DOCUMENT: {"bytes": 3629, "sha256": "e84e889d8ffdfff76ada0cb1f837d834a386548f0907683f34bc6dc766e1700f"},
    },
}
POLICY_SOURCE_PATHS = (VALIDATOR_SOURCE, POLICY_DOCUMENT)
APPROVED_POLICY_DOCUMENT = {
    "bytes": 3956,
    "sha256": "37869968247bb931feee0a3a458011bcdeb82a8f71d5d6dd99b4b04f054aaaab",
}
APPROVED_POLICY_DELTA = {
    "rtx_wall_seconds_max": {"replacement_captured": 0.45, "approved": 0.48},
    "rayquery_wall_seconds_max": {"original_captured": 0.38, "replacement_captured": 0.40, "approved": 0.43},
    "query_only_wall_seconds_max": {"original_captured": 0.38, "replacement_captured": 0.40, "approved": 0.43},
    "zero_instance_upload_update_count": {"original_captured": 0, "replacement_captured": 1, "approved": 1},
}


def load_json(path: Path) -> dict[str, object]:
  with path.open("r", encoding="utf-8") as file:
    value = json.load(file)
  if not isinstance(value, dict):
    raise ValueError(f"Expected a JSON object: {path}")
  return value


def write_json(path: Path, value: object) -> None:
  m6.write_json(path, value)


def normalized_validator_sha256(source: str) -> str:
  marker = 'APPROVED_VALIDATOR_NORMALIZED_SHA256 = "'
  begin = source.index(marker) + len(marker)
  end = source.index('"', begin)
  normalized = source[:begin] + "<normalized>" + source[end:]
  return hashlib.sha256(normalized.encode("utf-8")).hexdigest()


def validate_current_validator() -> None:
  source = (ROOT / VALIDATOR_SOURCE).read_text(encoding="utf-8")
  actual = normalized_validator_sha256(source)
  if actual != APPROVED_VALIDATOR_NORMALIZED_SHA256:
    raise RuntimeError(f"M13 validator revision changed: {actual}")


def suite() -> dict[str, object]:
  return {"order_seed": 6132026, "variants": [{"id": "specialized", "evo": "auto"}]}


def specs() -> list[m6.RunSpec]:
  return [
      m6.RunSpec(
          "measure",
          "measure",
          "evo-only",
          "motion-as",
          "default",
          "evo",
          technique,
          "specialized",
          "cold",
          1,
          1280,
          720,
          64,
          4,
          "rendering-regression",
          True,
      )
      for technique in TECHNIQUES
  ]


def source_manifest() -> list[dict[str, object]]:
  result = []
  for relative in M13_SOURCE_FILES:
    path = ROOT / relative
    if not path.is_file():
      raise FileNotFoundError(f"Missing M13 provenance input: {path}")
    result.append({"path": relative, "bytes": path.stat().st_size, "sha256": legacy.sha256(path)})
  return result


def provenance(editor: Path, build_dir: Path) -> dict[str, object]:
  value = {
      "schema": 1,
      "type": "raytracer_m13_provenance",
      "repo": legacy.repo_state(ROOT),
      "build": legacy.cmake_build_metadata(build_dir, "RelWithDebInfo"),
      "installed_build_matches": legacy.verify_evo_install_matches_build(editor, build_dir),
      "runtime": legacy.runtime_binary_manifest(editor),
      "sources": source_manifest(),
      "static_m6_report": {"path": str(M6_REPORT), "sha256": legacy.sha256(M6_REPORT)},
      "suite": suite(),
      "specs": [asdict(spec) for spec in specs()],
  }
  value["evidence_fingerprint"] = m6.json_sha256(value)
  return value


def validate_analysis_provenance(recorded: dict[str, object], current: dict[str, object]) -> dict[str, object] | None:
  validate_current_validator()
  if recorded == current:
    return None

  recorded_sources = {entry["path"]: entry for entry in recorded["sources"]}
  current_sources = {entry["path"]: entry for entry in current["sources"]}
  captured_policy = next(
      (
          name
          for name, sources in CAPTURED_POLICY_SOURCE_SETS.items()
          if all(
              {key: recorded_sources.get(path, {}).get(key) for key in ("bytes", "sha256")} == expected
              for path, expected in sources.items()
          )
      ),
      None,
  )
  if captured_policy is None:
    raise RuntimeError("M13 captured policy sources do not match an approved revision")
  if {key: current_sources[POLICY_DOCUMENT].get(key) for key in ("bytes", "sha256")} != APPROVED_POLICY_DOCUMENT:
    raise RuntimeError("M13 approved policy document changed after target ratification")
  if current_sources[VALIDATOR_SOURCE]["sha256"] == recorded_sources[VALIDATOR_SOURCE]["sha256"]:
    raise RuntimeError("M13 source/build provenance changed after the three captures")

  def comparable(value: dict[str, object]) -> dict[str, object]:
    value = copy.deepcopy(value)
    value.pop("evidence_fingerprint")
    value["sources"] = [entry for entry in value["sources"] if entry["path"] not in POLICY_SOURCE_PATHS]
    return value

  if comparable(recorded) != comparable(current):
    raise RuntimeError("M13 source/build provenance changed after the three captures")
  return {
      "captured_policy": captured_policy,
      "sources": {
          path: {"captured": recorded_sources[path], "analysis": current_sources[path]}
          for path in POLICY_SOURCE_PATHS
      },
      "approved_delta": APPROVED_POLICY_DELTA,
  }


def validate_static_m6() -> dict[str, object]:
  actual_hash = legacy.sha256(M6_REPORT)
  if actual_hash != M6_REPORT_SHA256:
    raise RuntimeError(f"Approved M6 report changed: {actual_hash} != {M6_REPORT_SHA256}")
  report = load_json(M6_REPORT)
  rows = report.get("runs", [])
  rows_by_technique = {str(row.get("technique")): row for row in rows}
  if report.get("type") != "raytracer_m6_compact_report" or len(rows) != 3 or set(rows_by_technique) != set(TECHNIQUES):
    raise RuntimeError("Approved M6 report is not the exact three-lane compact baseline")
  records = []
  for technique in TECHNIQUES:
    row = rows_by_technique[technique]
    if any(
        int(row.get(field, -1)) != 0
        for field in (
            "capture_blas_build_count",
            "capture_blas_update_count",
            "capture_tlas_build_count",
            "capture_tlas_update_count",
        )
    ):
      raise RuntimeError(f"Approved M6 {technique} lane no longer proves static zero AS work")
    output = Path(str(row["output"])).resolve()
    record_path = output.with_suffix(".record.json")
    if M6_REPORT.parent.resolve() not in record_path.parents:
      raise RuntimeError(f"Approved M6 record escaped its evidence root: {record_path}")
    record_hash = legacy.sha256(record_path)
    if record_hash != M6_RECORD_SHA256[technique]:
      raise RuntimeError(f"Approved M6 {technique} record changed: {record_hash}")
    record = load_json(record_path)
    spec = m6.RunSpec(**record["spec"])
    if record.get("status") != "complete" or record.get("run_id") != spec.run_id:
      raise RuntimeError(f"Approved M6 {technique} record is incomplete")
    if (
        spec.renderer != "evo" or spec.technique != technique or spec.scene != "bistro"
        or spec.width != 1280 or spec.height != 720 or spec.spp != 512 or spec.motion
    ):
      raise RuntimeError(f"Approved M6 {technique} record has the wrong static specification")
    for path_field, hash_field in (
        ("output", "output_sha256"), ("log", "log_sha256"), ("metrics_path", "metrics_sha256")
    ):
      path = Path(str(record[path_field])).resolve()
      if not path.is_file() or legacy.sha256(path) != record.get(hash_field):
        raise RuntimeError(f"Approved M6 {technique} {path_field} is missing or changed")
    output_path = Path(str(record["output"]))
    metrics_path = Path(str(record["metrics_path"]))
    m6.validate_run_log(Path(str(record["log"])), "evo")
    legacy.validate_output_image(output_path, legacy.Profile(spec.width, spec.height, spec.frames, spec.samples_per_frame))
    if load_json(metrics_path) != record.get("summary"):
      raise RuntimeError(f"Approved M6 {technique} metrics differ from the immutable record")
    records.append({"technique": technique, "record": str(record_path), "record_sha256": record_hash})
  return {"report": str(M6_REPORT), "report_sha256": actual_hash, "records": records, "gate": "zero capture AS work"}


def add_gate(gates: list[dict[str, object]], name: str, passed: bool, actual: object, expected: object) -> None:
  gates.append({"name": name, "pass": bool(passed), "actual": actual, "expected": expected})


def section(summary: dict[str, object], name: str) -> dict[str, object]:
  matches = [entry for entry in summary.get("gpu_sections", []) if entry.get("name") == name]
  return matches[0] if len(matches) == 1 else {}


def evaluate_lanes(lanes: dict[str, dict[str, object]]) -> tuple[list[dict[str, object]], dict[str, object]]:
  gates: list[dict[str, object]] = []
  results: dict[str, object] = {}
  for technique in TECHNIQUES:
    lane = lanes[technique]
    summary = lane["summary"]
    sections = summary.get("gpu_sections", [])
    blas = section(summary, "BLAS Update")
    tlas = section(summary, "TLAS Update")
    build_samples = sum(
        int(entry.get("sample_count", 0))
        for entry in sections
        if entry.get("name") in ("BLAS Build", "TLAS Build")
    )
    upload = summary.get("tlas_upload")
    startup_upload = summary.get("startup_tlas_upload")
    wall = float(summary.get("accumulation_wall_seconds", math.inf))
    add_gate(gates, f"{technique}_temporal_motion", summary.get("temporal_motion_capture") is True,
             summary.get("temporal_motion_capture"), True)
    add_gate(gates, f"{technique}_measure_window",
             summary.get("measured_frames") == 15 and summary.get("measured_spp") == 60,
             {"frames": summary.get("measured_frames"), "spp": summary.get("measured_spp")},
             {"frames": 15, "spp": 60})
    add_gate(gates, f"{technique}_zero_builds", build_samples == 0, build_samples, 0)
    add_gate(gates, f"{technique}_blas_updates",
             blas.get("sample_count") == TARGETS["blas_update_samples"]
             and 0.0 < float(blas.get("total_ms", math.inf)) <= TARGETS["blas_update_total_ms_max"],
             {"samples": blas.get("sample_count"), "total_ms": blas.get("total_ms")},
             {"samples": 30, "total_ms_max": 3.20})
    add_gate(gates, f"{technique}_tlas_updates",
             tlas.get("sample_count") == TARGETS["tlas_update_samples"]
             and 0.0 < float(tlas.get("total_ms", math.inf)) <= TARGETS["tlas_update_total_ms_max"],
             {"samples": tlas.get("sample_count"), "total_ms": tlas.get("total_ms")},
             {"samples": 15, "total_ms_max": 0.60})
    add_gate(gates, f"{technique}_wall", math.isfinite(wall) and 0.0 < wall <= TARGETS["wall_seconds_max"][technique], wall,
             {"maximum": TARGETS["wall_seconds_max"][technique]})
    add_gate(gates, f"{technique}_upload_schema", isinstance(upload, dict) and isinstance(startup_upload, dict),
             {"capture": isinstance(upload, dict), "startup": isinstance(startup_upload, dict)}, True)
    if not isinstance(upload, dict):
      upload = {}
    source_bytes = int(upload.get("source_bytes", 0))
    uploaded_bytes = int(upload.get("uploaded_bytes", 0))
    ratio = float(upload.get("upload_ratio", math.inf))
    expected_ratio = uploaded_bytes / source_bytes if source_bytes else math.inf
    add_gate(gates, f"{technique}_dirty_upload_accounting",
             upload.get("build_count") == 0
             and upload.get("update_count") == 15
             and upload.get("operation_count") == 15
             and upload.get("full_upload_count") == 0
             and upload.get("zero_instance_upload_update_count") == 1,
             upload,
             {"build_count": 0, "update_count": 15, "operation_count": 15, "full_upload_count": 0,
              "zero_instance_upload_update_count": 1})
    add_gate(gates, f"{technique}_dirty_upload_reduction",
             0 < uploaded_bytes < source_bytes and 15 <= int(upload.get("range_count", 0)) <= 30
             and math.isclose(ratio, expected_ratio, rel_tol=0.0, abs_tol=1.0e-12),
             {"source_bytes": source_bytes, "uploaded_bytes": uploaded_bytes,
              "range_count": upload.get("range_count"), "ratio": ratio},
             {"uploaded": "0 < uploaded < source", "range_count": "15..30", "ratio": "exact"})
    gpu = summary.get("gpu") or {}
    add_gate(gates, f"{technique}_approved_gpu",
             {key: gpu.get(key) for key in ("vendor_id", "device_id", "driver_version")}
             == {"vendor_id": 4318, "device_id": 12036, "driver_version": 2496774144},
             {key: gpu.get(key) for key in ("vendor_id", "device_id", "driver_version")},
             {"vendor_id": 4318, "device_id": 12036, "driver_version": 2496774144})
    results[technique] = {
        "accumulation_wall_seconds": wall,
        "blas_update": blas,
        "tlas_update": tlas,
        "tlas_upload": upload,
        "startup_tlas_upload": startup_upload,
        "output_sha256": lane.get("output_sha256"),
        "record": lane.get("record"),
        "record_sha256": lane.get("record_sha256"),
    }
  rq_hash = lanes["rq"].get("output_sha256")
  query_only_hash = lanes["query-only"].get("output_sha256")
  add_gate(gates, "rq_query_only_bit_exact", bool(rq_hash) and rq_hash == query_only_hash,
           {"rq": rq_hash, "query_only": query_only_hash}, "identical SHA-256")
  return gates, results


def record_lanes(records: list[dict[str, object]], output_root: Path) -> dict[str, dict[str, object]]:
  result = {}
  for record in records:
    technique = str(record["spec"]["technique"])
    record_path = m6.spec_paths(output_root, m6.RunSpec(**record["spec"]))["record"]
    result[technique] = {
        "summary": record["summary"],
        "output_sha256": record["output_sha256"],
        "record": str(record_path),
        "record_sha256": legacy.sha256(record_path),
    }
  return result


def refuse_existing_output(output_root: Path) -> None:
  try:
    relative = output_root.resolve().relative_to(ROOT)
  except ValueError:
    relative = None
  if relative is not None:
    ignored = subprocess.run(
        ["git", "-C", str(ROOT), "check-ignore", "--quiet", str(relative)], capture_output=True
    ).returncode == 0
    if not ignored:
      raise RuntimeError(f"M13 evidence inside the repository must use a git-ignored root: {output_root}")
  if output_root.exists() and any(output_root.iterdir()):
    raise RuntimeError(f"M13 requires a fresh evidence root and will not overwrite: {output_root}")


def append_launch(output_root: Path, spec: m6.RunSpec, status: str, error: str | None = None) -> None:
  path = output_root / "launch-ledger.json"
  ledger = load_json(path) if path.is_file() else {"schema": 1, "type": "raytracer_m13_launch_ledger", "launches": []}
  launches = ledger["launches"]
  if status == "started":
    launches.append({"run_id": spec.run_id, "technique": spec.technique, "status": status,
                     "started_utc": datetime.now(timezone.utc).isoformat()})
  else:
    entry = next(item for item in reversed(launches) if item["run_id"] == spec.run_id)
    entry["status"] = status
    entry["finished_utc"] = datetime.now(timezone.utc).isoformat()
    if error:
      entry["error"] = error
  write_json(path, ledger)


def run_measure(output_root: Path, editor: Path, build_dir: Path) -> tuple[dict[str, object], list[dict[str, object]]]:
  validate_current_validator()
  refuse_existing_output(output_root)
  static = validate_static_m6()
  evidence = provenance(editor, build_dir)
  output_root.mkdir(parents=True, exist_ok=True)
  write_json(output_root / "provenance.json", evidence)
  records = []
  for index, spec in enumerate(specs(), start=1):
    print(f"M13 launch {index}/3: {spec.run_id}", flush=True)
    append_launch(output_root, spec, "started")
    try:
      record = m6.execute_spec(
          spec, suite(), output_root, editor, ROOT, ROOT, ROOT, {}, str(evidence["evidence_fingerprint"]),
          False, False, False
      )
      m6.verify_record_integrity(record, spec, suite(), str(evidence["evidence_fingerprint"]))
      records.append(record)
      append_launch(output_root, spec, "complete")
    except Exception as error:
      append_launch(output_root, spec, "failed", str(error))
      raise
  write_json(output_root / "static-m6-binding.json", static)
  return evidence, records


def analyze(output_root: Path, editor: Path, build_dir: Path) -> dict[str, object]:
  current_evidence = provenance(editor, build_dir)
  recorded = load_json(output_root / "provenance.json")
  policy_revision = validate_analysis_provenance(recorded, current_evidence)
  static = validate_static_m6()
  fingerprint = str(recorded["evidence_fingerprint"])
  records = m6.require_records(output_root, specs(), suite(), fingerprint, "M13 motion-AS")
  gpu = m6.require_matched_gpu(records, "M13 motion-AS")
  ledger = load_json(output_root / "launch-ledger.json")
  launches = ledger.get("launches", [])
  if len(launches) != 3 or any(entry.get("status") != "complete" for entry in launches):
    raise RuntimeError("M13 launch ledger is not exactly three completed renderer processes")
  gates, lanes = evaluate_lanes(record_lanes(records, output_root))
  report = {
      "schema": 1,
      "type": "raytracer_m13_acceptance_report",
      "pass": all(gate["pass"] for gate in gates),
      "evidence_fingerprint": fingerprint,
      "analysis_policy_revision": policy_revision,
      "gpu": {"vendor_id": gpu[0], "device_id": gpu[1], "driver_version": gpu[2]},
      "run_count": len(records),
      "record_set_digest": m6.record_set_digest(records),
      "capture_accounting": {"observed_fresh_precommit": 9, "accepted_current_source_precommit": 3,
                             "superseded_precommit": 6, "planned_postcommit_delivery": 1,
                             "total_milestone": 10, "reference_executed": False},
      "targets": TARGETS,
      "static_m6": static,
      "lanes": lanes,
      "gates": gates,
      "failed_gates": [gate["name"] for gate in gates if not gate["pass"]],
  }
  write_json(output_root / "m13-report.json", report)
  return report


def synthetic_summary() -> dict[str, object]:
  return {
      "temporal_motion_capture": True,
      "measured_frames": 15,
      "measured_spp": 60,
      "accumulation_wall_seconds": 0.35,
      "gpu": {"vendor_id": 4318, "device_id": 12036, "driver_version": 2496774144},
      "gpu_sections": [
          {"name": "BLAS Update", "sample_count": 30, "total_ms": 3.0},
          {"name": "TLAS Update", "sample_count": 15, "total_ms": 0.4},
      ],
      "startup_tlas_upload": {},
      "tlas_upload": {"source_bytes": 96000, "uploaded_bytes": 1920, "upload_ratio": 0.02,
                      "range_count": 30, "operation_count": 15, "build_count": 0, "update_count": 15,
                      "no_op_count": 0, "full_upload_count": 0, "zero_instance_upload_update_count": 1},
  }


def run_self_test() -> None:
  validator_source = (ROOT / VALIDATOR_SOURCE).read_text(encoding="utf-8")
  if normalized_validator_sha256(validator_source) != APPROVED_VALIDATOR_NORMALIZED_SHA256:
    raise AssertionError("M13 validator does not match its pinned normalized revision")
  if normalized_validator_sha256(validator_source + "\n# drift") == APPROVED_VALIDATOR_NORMALIZED_SHA256:
    raise AssertionError("M13 validator self-hash accepted content drift")
  if {"bytes": (ROOT / POLICY_DOCUMENT).stat().st_size,
      "sha256": legacy.sha256(ROOT / POLICY_DOCUMENT)} != APPROVED_POLICY_DOCUMENT:
    raise AssertionError("M13 approved policy document does not match its pinned revision")
  for captured_policy, captured_sources in CAPTURED_POLICY_SOURCE_SETS.items():
    recorded_provenance = {
        "evidence_fingerprint": "captured",
        "repo": {"working_diff_sha256": "renderer"},
        "sources": [
            {"path": path, **value} for path, value in captured_sources.items()
        ] + [{"path": "renderer.cpp", "bytes": 8, "sha256": "renderer"}],
    }
    current_provenance = copy.deepcopy(recorded_provenance)
    current_provenance["evidence_fingerprint"] = "analysis"
    current_provenance["sources"][0].update(bytes=1, sha256="analysis-validator")
    current_provenance["sources"][1].update(APPROVED_POLICY_DOCUMENT)
    policy_revision = validate_analysis_provenance(recorded_provenance, current_provenance)
    if policy_revision is None or policy_revision["captured_policy"] != captured_policy:
      raise AssertionError("M13 approved policy-only provenance revision was not recorded")
    for name, mutate in {
        "tracked_diff": lambda value: value["repo"].update(working_diff_sha256="changed"),
        "renderer_source": lambda value: value["sources"][2].update(sha256="changed"),
    }.items():
      changed = copy.deepcopy(current_provenance)
      mutate(changed)
      try:
        validate_analysis_provenance(recorded_provenance, changed)
      except RuntimeError:
        pass
      else:
        raise AssertionError(f"M13 policy exception accepted {name} drift")
  passing = {
      technique: {"summary": synthetic_summary(), "output_sha256": "rq" if technique != "rtx" else "rtx"}
      for technique in TECHNIQUES
  }
  gates, _ = evaluate_lanes(passing)
  if not all(gate["pass"] for gate in gates):
    raise AssertionError("Synthetic passing M13 evidence was rejected")
  mutations = {
      "build": lambda value: value["gpu_sections"].append({"name": "TLAS Build", "sample_count": 1}),
      "blas_count": lambda value: value["gpu_sections"][0].update(sample_count=29),
      "wall": lambda value: value.update(accumulation_wall_seconds=0.5),
      "invalid_wall": lambda value: value.update(accumulation_wall_seconds=-1.0),
      "full_upload": lambda value: value["tlas_upload"].update(full_upload_count=1),
      "zero_copy_update": lambda value: value["tlas_upload"].update(zero_instance_upload_update_count=0),
      "no_reduction": lambda value: value["tlas_upload"].update(uploaded_bytes=96000, upload_ratio=1.0),
  }
  for name, mutate in mutations.items():
    evidence = copy.deepcopy(passing)
    mutate(evidence["rtx"]["summary"])
    if all(gate["pass"] for gate in evaluate_lanes(evidence)[0]):
      raise AssertionError(f"Synthetic M13 {name} failure was accepted")
  mismatch = copy.deepcopy(passing)
  mismatch["query-only"]["output_sha256"] = "different"
  if all(gate["pass"] for gate in evaluate_lanes(mismatch)[0]):
    raise AssertionError("Synthetic M13 RayQuery mismatch was accepted")
  with tempfile.TemporaryDirectory(prefix="raytracer-m13-self-test-") as temporary:
    root = Path(temporary)
    root.mkdir(exist_ok=True)
    write_json(root / "used.json", {})
    try:
      refuse_existing_output(root)
    except RuntimeError:
      pass
    else:
      raise AssertionError("M13 fresh-root guard accepted an existing artifact")
  try:
    refuse_existing_output(ROOT / "m13-nonignored-self-test")
  except RuntimeError:
    pass
  else:
    raise AssertionError("M13 provenance guard accepted a non-ignored in-repository root")


def print_plan(output_root: Path, editor: Path) -> None:
  validate_current_validator()
  plan = {
      "schema": 1,
      "type": "raytracer_m13_plan",
      "output_root": str(output_root),
      "editor": str(editor),
      "measure_phase_capture_count": 3,
      "approved_precommit_capture_count": 9,
      "superseded_precommit_capture_count": 6,
      "postcommit_delivery_capture_count": 1,
      "total_milestone_capture_count": 10,
      "reference_capture_count": 0,
      "specs": [asdict(spec) | {"paths": {key: str(value) for key, value in m6.spec_paths(output_root, spec).items()}}
                for spec in specs()],
  }
  print(json.dumps(plan, indent=2, sort_keys=True))


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--phase", choices=("self-test", "plan", "measure", "analyze", "all"), default="plan")
  parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
  parser.add_argument("--editor", type=Path, default=DEFAULT_EDITOR)
  parser.add_argument("--evo-build-dir", type=Path, default=DEFAULT_BUILD)
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  output_root = args.output_dir.resolve()
  editor = args.editor.resolve()
  build_dir = args.evo_build_dir.resolve()
  if args.phase == "self-test":
    run_self_test()
    print("M13 validator self-test passed")
    return 0
  if args.phase == "plan":
    print_plan(output_root, editor)
    return 0
  if args.phase in ("measure", "all"):
    run_measure(output_root, editor, build_dir)
  if args.phase in ("analyze", "all"):
    report = analyze(output_root, editor, build_dir)
    print(f"M13 acceptance: {'PASS' if report['pass'] else 'FAIL'} -> {output_root / 'm13-report.json'}")
    return 0 if report["pass"] else 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
