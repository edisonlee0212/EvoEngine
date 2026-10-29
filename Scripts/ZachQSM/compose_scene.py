"""Place generated trees in a scene, preserving individual meshes and species labels.

This places independently grown trees; it does not simulate canopy competition.
"""
import argparse
import json
from pathlib import Path

import trimesh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="JSON with trees: [{mesh, position_m: [x,y,z], species, id}]")
    parser.add_argument("--output", required=True, type=Path, help="New .glb path")
    args = parser.parse_args()
    if args.output.exists(): parser.error("Output already exists")
    config = json.loads(args.config.read_text(encoding="utf-8"))
    scene = trimesh.Scene()
    for tree in config["trees"]:
        path = (args.config.resolve().parent / tree["mesh"]).resolve()
        mesh = trimesh.load_mesh(path, process=False)
        mesh.apply_translation(tree["position_m"])
        mesh.visual.vertex_colors = [115, 79, 43, 255]
        scene.add_geometry(mesh, node_name=tree["id"], geom_name=tree["id"], metadata={"species": tree["species"]})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    scene.export(args.output)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
