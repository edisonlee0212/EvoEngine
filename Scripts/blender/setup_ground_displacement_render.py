"""Add Cycles ground displacement to an authored LSystem sorghum Blender scene."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import bpy


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BLEND = ROOT / "out" / "exports" / "lsystem_sorghum_paper_smoke" / "lsystem_sorghum_adult_cycles.blend"
DEFAULT_OUTPUT_BLEND = (
    ROOT
    / "out"
    / "exports"
    / "lsystem_sorghum_paper_smoke"
    / "lsystem_sorghum_adult_cycles_ground_displacement.blend"
)
DEFAULT_RENDER = (
    ROOT
    / "out"
    / "exports"
    / "lsystem_sorghum_paper_smoke"
    / "lsystem_sorghum_adult_cycles_ground_displacement.png"
)
DEFAULT_SOIL_HEIGHT = (
    ROOT / "Resources" / "DigitalAgricultureProject" / "Assets" / "2026-06-04_Sorghum" / "soil_PBRv2" / "height.png"
)


def parse_args() -> argparse.Namespace:
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--blend", type=Path, default=DEFAULT_BLEND)
    parser.add_argument("--output-blend", type=Path, default=DEFAULT_OUTPUT_BLEND)
    parser.add_argument("--render-output", type=Path, default=DEFAULT_RENDER)
    parser.add_argument("--height-texture", type=Path, default=DEFAULT_SOIL_HEIGHT)
    parser.add_argument("--samples", type=int, default=512)
    parser.add_argument("--resolution-x", type=int, default=3000)
    parser.add_argument("--resolution-y", type=int, default=2000)
    parser.add_argument("--displacement-strength", type=float, default=0.5)
    parser.add_argument("--material-displacement-strength", type=float, default=0.08)
    parser.add_argument("--height-midpoint", type=float, default=0.5)
    parser.add_argument("--label", default="")
    parser.add_argument("--skip-render", action="store_true")
    return parser.parse_args(argv)


def load_image(path: Path, colorspace: str) -> bpy.types.Image:
    image = bpy.data.images.load(str(path.resolve()), check_existing=True)
    try:
        image.colorspace_settings.name = colorspace
    except TypeError:
        pass
    return image


def texture_dir_for_blend(path: Path) -> Path:
    return path.resolve().parent / "textures"


def ensure_soil_height_texture(output_blend: Path, source: Path) -> Path:
    if not source.exists():
        raise FileNotFoundError(f"Soil height texture not found: {source}")
    texture_dir = texture_dir_for_blend(output_blend)
    texture_dir.mkdir(parents=True, exist_ok=True)
    target = texture_dir / "soil_height.png"
    if source.resolve() != target.resolve():
        shutil.copy2(source, target)
    return target


def find_ground_object() -> bpy.types.Object:
    candidates = [obj for obj in bpy.context.scene.objects if obj.type == "MESH" and obj.name.startswith("Ground Mesh")]
    if not candidates:
        candidates = [obj for obj in bpy.context.scene.objects if obj.type == "MESH" and "ground" in obj.name.lower()]
    if not candidates:
        raise RuntimeError("Could not find a ground mesh object")
    return max(candidates, key=lambda obj: len(obj.data.polygons))


def new_principled_material(name: str) -> tuple[bpy.types.Material, bpy.types.Node]:
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    for node in list(nodes):
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (520, 0)
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (260, 0)
    material.node_tree.links.new(bsdf.outputs["BSDF"], output.inputs["Surface"])
    return material, bsdf


def set_input_default(node: bpy.types.Node, name: str, value) -> None:
    if name in node.inputs:
        node.inputs[name].default_value = value


def add_image_node(material: bpy.types.Material, image: bpy.types.Image, label: str, location: tuple[int, int]) -> bpy.types.Node:
    node = material.node_tree.nodes.new("ShaderNodeTexImage")
    node.label = label
    node.image = image
    node.location = location
    return node


def first_existing(paths: list[Path]) -> Path | None:
    return next((path for path in paths if path.exists()), None)


def rebuild_ground_material(
    ground: bpy.types.Object,
    texture_dir: Path,
    height_path: Path,
    material_displacement_strength: float,
) -> dict[str, str | None]:
    material, bsdf = new_principled_material("SWEEP_Ground_Soil_Displaced_Cycles")
    if hasattr(material, "displacement_method"):
        material.displacement_method = "DISPLACEMENT"
    if hasattr(material, "max_vertex_displacement"):
        material.max_vertex_displacement = max(0.5, material_displacement_strength)
    set_input_default(bsdf, "Metallic", 0.0)
    set_input_default(bsdf, "Roughness", 0.72)

    paths = {
        "albedo": first_existing([texture_dir / "albedo.png"]),
        "normal": first_existing([texture_dir / "normal.png"]),
        "roughness": first_existing([texture_dir / "roughness.png"]),
        "ao": first_existing([texture_dir / "ao.png"]),
        "height": height_path,
    }
    images = {
        "albedo": load_image(paths["albedo"], "sRGB") if paths["albedo"] else None,
        "normal": load_image(paths["normal"], "Non-Color") if paths["normal"] else None,
        "roughness": load_image(paths["roughness"], "Non-Color") if paths["roughness"] else None,
        "ao": load_image(paths["ao"], "Non-Color") if paths["ao"] else None,
        "height": load_image(paths["height"], "Non-Color"),
    }
    links = material.node_tree.links
    if images["albedo"]:
        albedo = add_image_node(material, images["albedo"], "soil albedo", (-760, 180))
        color_output = albedo.outputs["Color"]
        if images["ao"]:
            ao = add_image_node(material, images["ao"], "soil ao", (-760, -40))
            multiply = material.node_tree.nodes.new("ShaderNodeMix")
            multiply.data_type = "RGBA"
            multiply.factor_mode = "UNIFORM"
            multiply.blend_type = "MULTIPLY"
            multiply.inputs["Factor"].default_value = 0.22
            multiply.location = (-420, 130)
            links.new(albedo.outputs["Color"], multiply.inputs["A"])
            links.new(ao.outputs["Color"], multiply.inputs["B"])
            color_output = multiply.outputs["Result"]
        links.new(color_output, bsdf.inputs["Base Color"])
    if images["roughness"] and "Roughness" in bsdf.inputs:
        roughness = add_image_node(material, images["roughness"], "soil roughness", (-420, -230))
        links.new(roughness.outputs["Color"], bsdf.inputs["Roughness"])
    normal_output = None
    if images["normal"]:
        normal_tex = add_image_node(material, images["normal"], "soil normal", (-760, -500))
        normal = material.node_tree.nodes.new("ShaderNodeNormalMap")
        normal.inputs["Strength"].default_value = 0.42
        normal.location = (-420, -500)
        links.new(normal_tex.outputs["Color"], normal.inputs["Color"])
        normal_output = normal.outputs["Normal"]
    if images["height"]:
        height = add_image_node(material, images["height"], "soil height micro-bump", (-760, -720))
        bump = material.node_tree.nodes.new("ShaderNodeBump")
        bump.inputs["Strength"].default_value = 0.08
        bump.inputs["Distance"].default_value = 0.018
        bump.location = (-150, -590)
        links.new(height.outputs["Color"], bump.inputs["Height"])
        if normal_output:
            links.new(normal_output, bump.inputs["Normal"])
        normal_output = bump.outputs["Normal"]
        displacement = material.node_tree.nodes.new("ShaderNodeDisplacement")
        displacement.location = (260, -740)
        displacement.inputs["Scale"].default_value = material_displacement_strength
        displacement.inputs["Midlevel"].default_value = 0.5
        output = next(node for node in material.node_tree.nodes if node.type == "OUTPUT_MATERIAL")
        links.new(height.outputs["Color"], displacement.inputs["Height"])
        links.new(displacement.outputs["Displacement"], output.inputs["Displacement"])
    if normal_output and "Normal" in bsdf.inputs:
        links.new(normal_output, bsdf.inputs["Normal"])

    ground.data.materials.clear()
    ground.data.materials.append(material)
    return {key: str(value.resolve()) if value else None for key, value in paths.items()}


def add_socket(group: bpy.types.NodeTree, name: str, in_out: str, socket_type: str) -> None:
    group.interface.new_socket(name=name, in_out=in_out, socket_type=socket_type)


def build_ground_displacement_group(
    height_image: bpy.types.Image,
    strength: float,
    midpoint: float,
) -> bpy.types.NodeTree:
    name = "SWEEP_Ground_Height_Displacement_GN"
    old = bpy.data.node_groups.get(name)
    if old:
        bpy.data.node_groups.remove(old)
    group = bpy.data.node_groups.new(name, "GeometryNodeTree")
    add_socket(group, "Geometry", "INPUT", "NodeSocketGeometry")
    add_socket(group, "Geometry", "OUTPUT", "NodeSocketGeometry")

    nodes = group.nodes
    links = group.links
    group_input = nodes.new("NodeGroupInput")
    group_input.location = (-920, 0)
    group_output = nodes.new("NodeGroupOutput")
    group_output.location = (680, 0)

    uv = nodes.new("GeometryNodeInputNamedAttribute")
    uv.data_type = "FLOAT_VECTOR"
    uv.inputs["Name"].default_value = "UVMap"
    uv.location = (-920, -230)

    image = nodes.new("GeometryNodeImageTexture")
    image.inputs["Image"].default_value = height_image
    image.extension = "EXTEND"
    image.interpolation = "Cubic"
    image.location = (-650, -180)

    separate = nodes.new("FunctionNodeSeparateColor")
    separate.mode = "RGB"
    separate.location = (-390, -180)

    subtract = nodes.new("ShaderNodeMath")
    subtract.operation = "SUBTRACT"
    subtract.inputs[1].default_value = midpoint
    subtract.location = (-160, -170)

    scale = nodes.new("ShaderNodeMath")
    scale.operation = "MULTIPLY"
    scale.inputs[1].default_value = strength
    scale.location = (55, -170)

    normal = nodes.new("GeometryNodeInputNormal")
    normal.location = (-160, -390)

    vector_scale = nodes.new("ShaderNodeVectorMath")
    vector_scale.operation = "SCALE"
    vector_scale.location = (280, -250)

    set_position = nodes.new("GeometryNodeSetPosition")
    set_position.location = (430, 0)

    links.new(group_input.outputs["Geometry"], set_position.inputs["Geometry"])
    links.new(uv.outputs["Attribute"], image.inputs["Vector"])
    links.new(image.outputs["Color"], separate.inputs["Color"])
    links.new(separate.outputs["Red"], subtract.inputs[0])
    links.new(subtract.outputs["Value"], scale.inputs[0])
    links.new(normal.outputs["Normal"], vector_scale.inputs["Vector"])
    links.new(scale.outputs["Value"], vector_scale.inputs["Scale"])
    links.new(vector_scale.outputs["Vector"], set_position.inputs["Offset"])
    links.new(set_position.outputs["Geometry"], group_output.inputs["Geometry"])
    return group


def apply_ground_displacement_modifier(
    ground: bpy.types.Object,
    height_path: Path,
    strength: float,
    midpoint: float,
) -> None:
    height_image = load_image(height_path, "Non-Color")
    node_group = build_ground_displacement_group(height_image, strength, midpoint)
    for modifier in list(ground.modifiers):
        if modifier.name == "SWEEP Ground Height Displacement":
            ground.modifiers.remove(modifier)
    modifier = ground.modifiers.new("SWEEP Ground Height Displacement", "NODES")
    modifier.node_group = node_group
    modifier.show_render = True
    modifier.show_viewport = True


def configure_cycles(samples: int, resolution_x: int, resolution_y: int) -> None:
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = True
    scene.cycles.max_bounces = 10
    scene.cycles.diffuse_bounces = 4
    scene.cycles.glossy_bounces = 4
    scene.cycles.transparent_max_bounces = 12
    scene.render.resolution_x = resolution_x
    scene.render.resolution_y = resolution_y
    scene.view_settings.exposure = -0.2
    scene.view_settings.gamma = 1.0
    try:
        preferences = bpy.context.preferences.addons["cycles"].preferences
        preferences.compute_device_type = "OPTIX"
        for device in preferences.devices:
            device.use = True
        scene.cycles.device = "GPU"
    except Exception:
        scene.cycles.device = "CPU"


def add_camera_label(label: str) -> None:
    if not label:
        return
    camera = bpy.context.scene.camera
    if not camera:
        return
    material = bpy.data.materials.new("SWEEP_Render_Label_Black")
    material.diffuse_color = (0.0, 0.0, 0.0, 1.0)
    material.use_nodes = True
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    if bsdf and "Base Color" in bsdf.inputs:
        bsdf.inputs["Base Color"].default_value = (0.0, 0.0, 0.0, 1.0)
    bpy.ops.object.text_add()
    text = bpy.context.object
    text.name = "SWEEP Render Date Label"
    text.data.body = label
    text.data.align_x = "LEFT"
    text.data.align_y = "CENTER"
    text.data.materials.append(material)
    frame = camera.data.view_frame(scene=bpy.context.scene)
    distance = 2.0
    scale = distance / max(1.0e-6, abs(frame[0].z))
    min_x = min(point.x for point in frame) * scale
    max_x = max(point.x for point in frame) * scale
    min_y = min(point.y for point in frame) * scale
    max_y = max(point.y for point in frame) * scale
    text.data.size = 0.045 * (max_y - min_y)
    text.parent = camera
    text.location = (
        min_x + 0.04 * (max_x - min_x),
        max_y - 0.06 * (max_y - min_y),
        -distance,
    )
    text.rotation_euler = (0.0, 0.0, 0.0)


def write_report(
    path: Path,
    ground: bpy.types.Object,
    texture_paths: dict[str, str | None],
    args: argparse.Namespace,
) -> None:
    camera = bpy.context.scene.camera
    report = {
        "blend": str(args.output_blend.resolve()),
        "render": None if args.skip_render else str(args.render_output.resolve()),
        "ground_object": ground.name,
        "ground_vertices": len(ground.data.vertices),
        "ground_polygons": len(ground.data.polygons),
        "ground_modifiers": [modifier.name for modifier in ground.modifiers],
        "ground_textures": texture_paths,
        "displacement_strength_m": args.displacement_strength,
        "material_displacement_strength_m": args.material_displacement_strength,
        "height_midpoint": args.height_midpoint,
        "label": args.label,
        "camera": {
            "name": camera.name if camera else None,
            "location": list(camera.location) if camera else None,
            "rotation_euler": list(camera.rotation_euler) if camera else None,
            "lens": camera.data.lens if camera else None,
        },
        "cycles": {
            "samples": bpy.context.scene.cycles.samples,
            "device": bpy.context.scene.cycles.device,
            "resolution": [bpy.context.scene.render.resolution_x, bpy.context.scene.render.resolution_y],
        },
    }
    path.write_text(json.dumps(report, indent=2), encoding="utf-8")


def main() -> None:
    args = parse_args()
    args.blend = args.blend.resolve()
    args.output_blend = args.output_blend.resolve()
    args.render_output = args.render_output.resolve()
    args.height_texture = args.height_texture.resolve()
    bpy.ops.wm.open_mainfile(filepath=str(args.blend))
    if not bpy.context.scene.camera:
        raise RuntimeError("The blend file has no active camera; refusing to invent one")

    height_path = ensure_soil_height_texture(args.output_blend, args.height_texture)
    ground = find_ground_object()
    texture_paths = rebuild_ground_material(
        ground,
        texture_dir_for_blend(args.output_blend),
        height_path,
        args.material_displacement_strength,
    )
    apply_ground_displacement_modifier(ground, height_path, args.displacement_strength, args.height_midpoint)
    configure_cycles(args.samples, args.resolution_x, args.resolution_y)

    args.output_blend.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output_blend))
    if not args.skip_render:
        add_camera_label(args.label)
        args.render_output.parent.mkdir(parents=True, exist_ok=True)
        bpy.context.scene.render.filepath = str(args.render_output)
        bpy.ops.render.render(write_still=True)

    write_report(args.output_blend.with_name(args.output_blend.stem + "_report.json"), ground, texture_paths, args)


if __name__ == "__main__":
    main()
