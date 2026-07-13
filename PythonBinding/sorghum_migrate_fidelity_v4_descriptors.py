#!/usr/bin/env python3
"""Migrate the promoted sorghum descriptors to the fidelity-v4 morphology contract."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "Resources" / "DigitalAgricultureProject" / "Assets"
DESCRIPTORS = ASSETS / "GeneratedAssets" / "Descriptors"
MANUAL = ASSETS / "ManualAssets" / "Descriptors"

STAGES = {
    "GrowthStage01": (0.0120, 0.0140, 0.00030, 0.00015),
    "GrowthStage02": (0.0135, 0.0252, 0.00035, 0.00020),
    "GrowthStage03": (0.0150, 0.0420, 0.00045, 0.00030),
    "GrowthStage04": (0.0165, 0.1260, 0.00050, 0.00040),
    "GrowthStage05": (0.0165, 0.1260, 0.00045, 0.00045),
}


def plotted(name: str, value: float) -> str:
    formatted = f"{value:.8g}"
    return f"""{name}:
  mean:
    min_value: {formatted}
    max_value: {formatted}
    curve:
      tangent_: false
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [0, 0.5]
        - [1, 0.5]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: false
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [0, 0]
        - [1, 0]
"""


def tapered_plotted(name: str, basal: float, tip: float) -> str:
    return f"""{name}:
  mean:
    min_value: {tip:.8g}
    max_value: {basal:.8g}
    curve:
      tangent_: false
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [0, 1]
        - [0.6, 1]
        - [1, 0]
  deviation:
    min_value: 0
    max_value: 0
    curve:
      tangent_: false
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [0, 0]
        - [1, 0]
"""


def single(name: str, mean: float, deviation: float = 0.0) -> str:
    return f"{name}:\n  mean: {mean:.8g}\n  deviation: {deviation:.8g}\n"


def replace_block(text: str, name: str, replacement: str) -> str:
    pattern = re.compile(rf"(?ms)^{re.escape(name)}:\n.*?(?=^[A-Za-z_][A-Za-z0-9_]*:|\Z)")
    if not pattern.search(text):
        raise ValueError(f"missing descriptor field: {name}")
    return pattern.sub(replacement, text, count=1)


def insert_after(text: str, name: str, addition: str) -> str:
    pattern = re.compile(rf"(?ms)^{re.escape(name)}:\n.*?(?=^[A-Za-z_][A-Za-z0-9_]*:|\Z)")
    match = pattern.search(text)
    if not match:
        match = re.search(rf"(?m)^{re.escape(name)}:.*\n", text)
    if not match:
        raise ValueError(f"missing descriptor field: {name}")
    return text[: match.end()] + addition + text[match.end() :]


def upsert_after(text: str, name: str, field: str, block: str) -> str:
    return replace_block(text, field, block) if re.search(rf"(?m)^{re.escape(field)}:", text) else insert_after(text, name, block)


def scalar(text: str, name: str, value: str, after: str) -> str:
    line = f"{name}: {value}\n"
    if re.search(rf"(?m)^{re.escape(name)}:", text):
        return re.sub(rf"(?m)^{re.escape(name)}:.*$", line.rstrip(), text)
    return insert_after(text, after, line)


def asset_ref(name: str, handle: int) -> str:
    return f'{name}:\n  asset_handle_: {handle}\n  type_name_: Texture2D\n'


def migrate(path: Path, values: tuple[float, float, float, float]) -> None:
    diameter, blade_width, blade_thickness, sheath_thickness = values
    text = path.read_text(encoding="utf-8")
    text = replace_block(text, "internode_thickness", tapered_plotted("internode_thickness", diameter, diameter * 0.8))
    text = replace_block(text, "leaf_blade_max_width", plotted("leaf_blade_max_width", blade_width))
    text = upsert_after(
        text,
        "leaf_blade_max_width",
        "leaf_blade_thickness",
        plotted("leaf_blade_thickness", blade_thickness),
    )
    text = upsert_after(text, "leaf_blade_thickness", "leaf_sheath_thickness", plotted("leaf_sheath_thickness", sheath_thickness))
    text = re.sub(r"(?m)^leaf_width_scale:.*$", "leaf_width_scale: 1", text)
    text = replace_block(text, "leaf_waviness_frequency", single("leaf_waviness_frequency", 3.0, 0.75))
    text = replace_block(text, "leaf_sheath_radius_ratio", single("leaf_sheath_radius_ratio", 1.05))
    text = upsert_after(text, "leaf_sheath_radius_ratio", "leaf_sheath_wrap_angle", single("leaf_sheath_wrap_angle", 390.0))
    text = re.sub(r"(?m)^tiller_model_version:.*$", "tiller_model_version: 4", text)
    text = replace_block(text, "tiller_final_lean_angle", single("tiller_final_lean_angle", 15.0, 5.0))
    text = replace_block(text, "tiller_azimuth_jitter", single("tiller_azimuth_jitter", 0.0, 10.0))
    text = re.sub(r"(?m)^tiller_recovery_phytomer_count:.*\n", "", text)
    text = scalar(text, "tiller_recovery_axis_fraction", "1", "tiller_azimuth_jitter")
    text = scalar(text, "culm_radial_segments", "24", "stem_material_specular")
    text = scalar(text, "culm_node_radius_scale", "1.08", "culm_radial_segments")
    text = scalar(text, "culm_texture_repeat_m", "0.25", "culm_node_radius_scale")
    text = re.sub(r"(?m)^leaf_material_albedo_color:.*$", "leaf_material_albedo_color: [1, 1, 1]", text)
    text = re.sub(r"(?m)^stem_material_albedo_color:.*$", "stem_material_albedo_color: [1, 1, 1]", text)
    handles = {
        "stem_albedo_texture": 11677113829394799981,
        "stem_normal_texture": 15939859271237366179,
        "stem_roughness_texture": 13467941576064353144,
        "stem_metallic_texture": 7946515516638111996,
        "stem_ao_texture": 9007124116920053463,
    }
    anchor = "leaf_material_specular"
    for field, handle in handles.items():
        text = upsert_after(text, anchor, field, asset_ref(field, handle))
        anchor = field
    path.write_text(text, encoding="utf-8", newline="\n")


def main() -> None:
    for stage, values in STAGES.items():
        for cultivar in ("BTX", "Pawaga"):
            migrate(DESCRIPTORS / stage / f"{cultivar}.sorghumls", values)
    manual_values = (0.0210, 0.1260, 0.00050, 0.00040)
    for cultivar in ("BTX", "Pawaga"):
        migrate(MANUAL / f"{cultivar}.sorghumls", manual_values)
    for root in (ASSETS / "ManualAssets" / "Scenes", ASSETS / "GeneratedAssets" / "Scenes"):
        for scene in root.glob("*.evescene"):
            text = re.sub(r"(?m)^\s+leaf_thickness:.*\n", "", scene.read_text(encoding="utf-8"))
            scene.write_text(text, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
