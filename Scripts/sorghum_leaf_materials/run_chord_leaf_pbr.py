#!/usr/bin/env python3
"""Run Ubisoft Chord on one flat sorghum leaf albedo and save model PBR maps.

This script intentionally has no heuristic fallback path. If Chord cannot load
or cannot infer, it exits with an error.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

from PIL import Image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--albedo", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--chord-repo-root", type=Path, required=True)
    parser.add_argument("--chord-config", type=Path, required=True)
    parser.add_argument("--chord-checkpoint", type=Path, required=True)
    args = parser.parse_args()

    if not args.albedo.exists():
        raise FileNotFoundError(args.albedo)
    if not args.chord_repo_root.exists():
        raise FileNotFoundError(args.chord_repo_root)
    if not args.chord_config.exists():
        raise FileNotFoundError(args.chord_config)
    if not args.chord_checkpoint.exists():
        raise FileNotFoundError(args.chord_checkpoint)

    repo_root = str(args.chord_repo_root.resolve())
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

    import torch
    from omegaconf import OmegaConf
    from torchvision.transforms import v2
    from torchvision.transforms.functional import to_pil_image

    from chord import ChordModel
    from chord.io import load_torch_file

    args.out_dir.mkdir(parents=True, exist_ok=True)

    source = Image.open(args.albedo).convert("RGB")
    to_tensor = v2.Compose([v2.ToImage(), v2.ToDtype(torch.float32, scale=True)])
    image_tensor = to_tensor(source)
    original_h, original_w = image_tensor.shape[-2:]

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    config = OmegaConf.load(str(args.chord_config))
    model = ChordModel(config)
    state_dict = load_torch_file(str(args.chord_checkpoint))
    model.load_state_dict(state_dict)
    model.eval()
    model.to(device)

    x = v2.Resize(size=(1024, 1024), antialias=True)(image_tensor).unsqueeze(0).to(device)
    with torch.no_grad():
        if device.type == "cuda":
            with torch.autocast(device_type="cuda"):
                output = model(x)
        else:
            output = model(x)

    resize_back = v2.Resize(size=(original_h, original_w), antialias=True)
    saved: dict[str, str] = {}
    name_map = {
        "basecolor": "albedo.png",
        "normal": "normal.png",
        "roughness": "roughness.png",
        "metalness": "metallic.png",
    }
    for key, filename in name_map.items():
        tensor = output.get(key)
        if tensor is None:
            raise RuntimeError(f"Chord output did not include required map: {key}")
        tensor = resize_back(tensor).squeeze(0).detach().cpu()
        if key in {"roughness", "metalness"} and tensor.dim() == 3 and tensor.shape[0] > 1:
            tensor = tensor[:1]
        image = to_pil_image(tensor)
        out_path = args.out_dir / filename
        image.save(out_path)
        saved[key] = str(out_path)

    report = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "source_albedo": str(args.albedo),
        "chord_repo_root": str(args.chord_repo_root),
        "chord_config": str(args.chord_config),
        "chord_checkpoint": str(args.chord_checkpoint),
        "device": str(device),
        "outputs": saved,
    }
    (args.out_dir / "chord_leaf_pbr_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    for key, path in saved.items():
        print(f"{key}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
