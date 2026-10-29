"""Mesh and measure the actual EcoSysLab skeleton, in metres with Y up."""
import csv
import json
from pathlib import Path

import manifold3d as m3d
import numpy as np
import trimesh
import yaml


def read_skeleton(path):
    a = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    start = np.column_stack([a[f"start_{c}"] for c in "xyz"])
    end = np.column_stack([a[f"end_{c}"] for c in "xyz"])
    radius = a["diameter_m"] / 2
    ids = a["id"].astype(int)
    parents = a["parent_id"].astype(int)
    lookup = {node: i for i, node in enumerate(ids)}
    if len(lookup) != len(ids) or np.count_nonzero(parents == -1) != 1:
        raise ValueError("Skeleton needs unique node IDs and exactly one root")
    if not np.isfinite(np.column_stack([start, end, radius])).all() or np.any(radius <= 0):
        raise ValueError("Invalid coordinates or nonpositive radius")
    lengths = np.linalg.norm(end - start, axis=1)
    if not np.allclose(lengths, a["length_m"], atol=1e-5, rtol=1e-4):
        raise ValueError("Exported lengths disagree with endpoints")
    seen = set()
    gaps = []
    for i, (node, parent) in enumerate(zip(ids, parents)):
        # EcoSysLab emits parents before their children.
        if parent != -1:
            if parent not in seen:
                raise ValueError("Missing parent, cycle, or non-topological node order")
            parent_index = lookup[parent]
            gap = float(np.linalg.norm(start[i] - end[parent_index]))
            # EcoSysLab's branch_push offsets lateral starts radially and forward.
            if gap > np.sqrt(2)*radius[parent_index] + 1e-4:
                raise ValueError(f"Native attachment gap exceeds the branch_push bound: {gap:.6g} m")
            gaps.append(gap)
        seen.add(node)
    max_gap = max(gaps, default=0.)
    return a, start, end, radius, lengths, lookup, max_gap


def solid_from_cylinders(start, end, radius, radial_segments):
    solids = []
    for p, q, r in zip(start, end, radius):
        d = q - p
        length = np.linalg.norm(d)
        if length > 1e-8:
            z = d / length
            helper = np.array([1., 0, 0]) if abs(z[0]) < .9 else np.array([0., 1, 0])
            x = np.cross(helper, z)
            x /= np.linalg.norm(x)
            y = np.cross(z, x)
            transform = np.column_stack([x, y, z, p])
            solids.append(m3d.Manifold.cylinder(float(length), float(r), circular_segments=radial_segments).transform(transform))
        # These balls join angled segments without leaving open rings or cracks.
        sphere = m3d.Manifold.sphere(float(r), circular_segments=radial_segments)
        solids.extend([sphere.translate(p), sphere.translate(q)])
    union = m3d.Manifold.batch_boolean(solids, m3d.OpType.Add)
    return closed_exterior(union)


def closed_exterior(union):
    if union.status() != m3d.Error.NoError or union.is_empty():
        raise ValueError(f"Mesh union failed: {union.status()}")
    parts = union.decompose()
    exteriors = [p for p in parts if p.volume() > 0]
    cavities = [p for p in parts if p.volume() < 0]
    if len(exteriors) != 1 or len(exteriors) + len(cavities) != len(parts):
        raise ValueError(f"Wood mesh has {len(exteriors)} disconnected exterior components")
    # Negative closed shells bound internal voids, not detached wood branches.
    union = exteriors[0]
    raw = union.to_mesh()
    mesh = trimesh.Trimesh(vertices=np.asarray(raw.vert_properties)[:, :3],
                           faces=np.asarray(raw.tri_verts), process=False)
    if not mesh.is_watertight or not mesh.is_winding_consistent or mesh.volume <= 0:
        raise ValueError("Wood mesh did not pass closed-surface checks")
    mesh.metadata["filled_internal_cavities"] = len(cavities)
    mesh.metadata["filled_internal_cavity_volume_m3"] = -sum(p.volume() for p in cavities)
    return mesh


def process_tree(folder, segments=16):
    folder = Path(folder)
    a, start, end, radius, lengths, lookup, max_gap = read_skeleton(folder / "skeleton.csv")
    loader = getattr(yaml, "CSafeLoader", yaml.SafeLoader)
    nodes = yaml.load((folder / "nodes.yaml").read_text(), Loader=loader)["Nodes"]
    flows = yaml.load((folder / "flows.yaml").read_text(), Loader=loader)["Flows"]
    if len(nodes) != len(a) or {n["I"] for n in nodes} != set(lookup):
        raise ValueError("Node graph and cylinder node IDs disagree")
    for node in nodes:
        i = lookup[node["I"]]
        labels_match = node["PI"] == int(a["parent_id"][i]) and node["FI"] == int(a["flow_id"][i])
        positions_match = np.allclose(node["SP"], start[i], atol=1e-7) and np.allclose(node["EP"], end[i], atol=1e-7)
        if not labels_match or not positions_match or not np.isclose(node["T"], radius[i]*2):
            raise ValueError("Native YAML and CSV skeleton exports disagree")
    flow_ids = {f["I"] for f in flows}
    if len(flow_ids) != len(flows) or flow_ids != set(a["flow_id"].astype(int)):
        raise ValueError("Branch graph and node flow IDs disagree")
    flow_parents = {f["I"]:f["PI"] for f in flows}
    if list(flow_parents.values()).count(-1) != 1:
        raise ValueError("Branch graph must have exactly one root")
    for flow in flow_ids:
        visited = set()
        while flow != -1:
            if flow in visited or flow not in flow_parents:
                raise ValueError("Branch graph contains a cycle or missing parent")
            visited.add(flow)
            flow = flow_parents[flow]
    leaf_count = 0
    if (folder / "leaf_instances.csv").exists():
        with (folder / "leaf_instances.csv").open() as f:
            for row in csv.DictReader(f):
                if int(row["node_id"]) not in lookup or int(row["first_vertex"]) != leaf_count*4+1:
                    raise ValueError("Invalid leaf-to-node labels")
                leaf_count += 1
    with (folder / "cylinders.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["node_id", "parent_id", "flow_id", "order", "start_x", "start_y", "start_z",
                         "end_x", "end_y", "end_z", "radius_m", "length_m"])
        for i in range(len(a)):
            writer.writerow([int(a[i][k]) for k in ["id", "parent_id", "flow_id", "order"]] +
                            list(start[i]) + list(end[i]) + [radius[i], lengths[i]])
    connector_starts, connector_ends, connector_radii = [], [], []
    with (folder / "junction_connectors.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["child_node_id", "parent_node_id", "start_x", "start_y", "start_z",
                         "end_x", "end_y", "end_z", "radius_m", "length_m"])
        for i, parent in enumerate(a["parent_id"].astype(int)):
            if parent == -1: continue
            p = end[lookup[parent]]
            gap = float(np.linalg.norm(start[i]-p))
            if gap <= 1e-7: continue
            connector_starts.append(p); connector_ends.append(start[i]); connector_radii.append(radius[i])
            writer.writerow([int(a["id"][i]), parent, *p, *start[i], radius[i], gap])
    branch_rows = []
    angles = []
    for flow in np.unique(a["flow_id"]).astype(int):
        ii = np.flatnonzero(a["flow_id"] == flow)
        base = int(ii[0])
        parent = int(a["parent_id"][base])
        angle = None
        parent_flow = -1
        if parent != -1:
            j = lookup[parent]
            parent_flow = int(a["flow_id"][j])
            if lengths[base] > 1e-8 and lengths[j] > 1e-8:
                angle = float(np.degrees(np.arccos(np.clip(np.dot(end[base]-start[base], end[j]-start[j]) /
                                                         (lengths[base]*lengths[j]), -1, 1))))
                angles.append(angle)
        branch_rows.append(dict(flow_id=flow, parent_flow_id=parent_flow, order=int(a["order"][base]),
                                nodes=len(ii), length_m=float(lengths[ii].sum()),
                                basal_diameter_m=float(radius[base]*2), attachment_angle_deg=angle))
    with (folder / "branches.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(branch_rows[0]))
        writer.writeheader()
        writer.writerows(branch_rows)
    print(f"Meshing {folder.name}: {len(a)} cylinders", flush=True)
    mesh = solid_from_cylinders(np.vstack([start, np.array(connector_starts).reshape(-1,3)]),
                                np.vstack([end, np.array(connector_ends).reshape(-1,3)]),
                                np.r_[radius, connector_radii], segments)
    mesh.export(folder / "wood.obj")
    mesh.export(folder / "wood.ply")
    # Read the delivered PLY back; validate export as well as the in-memory mesh.
    reopened = trimesh.load_mesh(folder / "wood.ply", process=False)
    if not reopened.is_watertight or len(reopened.faces) != len(mesh.faces):
        raise ValueError("PLY export did not preserve the mesh")
    points = np.vstack([start, end])
    trunk_crossings = np.flatnonzero((a["order"] == 0) & (np.minimum(start[:,1], end[:,1]) <= 1.3) &
                                    (np.maximum(start[:,1], end[:,1]) >= 1.3))
    metrics = dict(tree=folder.name, nodes=len(a), branches=len(branch_rows), max_order=int(a["order"].max()),
                   leaf_planes=leaf_count, native_graphs_consistent=True,
                   height_m=float(points[:,1].max()-points[:,1].min()),
                   crown_width_x_m=float(np.ptp(points[:,0])), crown_width_z_m=float(np.ptp(points[:,2])),
                   total_axis_length_m=float(lengths.sum()),
                   trunk_diameter_at_1_3m_m=float(2*radius[trunk_crossings[0]]) if len(trunk_crossings) else None,
                   cylinder_volume_sum_m3=float(np.sum(np.pi*radius**2*lengths)),
                   closed_mesh_volume_m3=float(mesh.volume), surface_area_m2=float(mesh.area),
                   median_branch_angle_deg=float(np.median(angles)) if angles else None,
                   triangles=len(mesh.faces), vertices=len(mesh.vertices), watertight=bool(mesh.is_watertight),
                   consistent_winding=bool(mesh.is_winding_consistent), wood_components=1,
                   native_max_attachment_offset_m=max_gap, junction_connectors=len(connector_radii),
                   filled_internal_cavities=mesh.metadata["filled_internal_cavities"],
                   filled_internal_cavity_volume_m3=mesh.metadata["filled_internal_cavity_volume_m3"],
                   zero_length_nodes=int(np.count_nonzero(lengths <= 1e-8)))
    (folder / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    render_tree(mesh, folder / "wood_preview.png", folder.name)
    return metrics


def render_tree(mesh, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig = plt.figure(figsize=(7, 8), facecolor="#f6f5f1")
    ax = fig.add_subplot(projection="3d", facecolor="#f6f5f1")
    # Plotting convention swaps Y and Z so that native Y is shown vertically.
    triangles = mesh.triangles[:, :, [0, 2, 1]]
    normals = mesh.face_normals[:, [0, 2, 1]]
    light = np.array([-.4, -.6, .7]); light /= np.linalg.norm(light)
    shade = .35 + .65*np.clip(normals @ light, 0, 1)
    colors = np.array([.45, .30, .17])[None, :] * shade[:, None]
    ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, edgecolors="none", linewidths=0))
    bounds = mesh.bounds[:, [0, 2, 1]]
    extent = np.maximum(bounds[1]-bounds[0], .1)
    ax.set(xlim=bounds[:,0], ylim=bounds[:,1], zlim=bounds[:,2], title=title)
    ax.set_box_aspect(extent, zoom=1.3)
    ax.view_init(elev=12, azim=-65)
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)


def figures(output, metrics):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    output = Path(output)
    n = len(metrics)
    cols = min(4, n)
    rows = (n + cols - 1)//cols
    fig, axes = plt.subplots(rows, cols, figsize=(4*cols, 4.7*rows), squeeze=False, facecolor="#f6f5f1")
    for ax, metric in zip(axes.flat, metrics):
        ax.imshow(plt.imread(output / metric["tree"] / "wood_preview.png"))
        ax.set_title(f"{metric['tree']} · {metric['height_m']:.2f} m · {metric['branches']} branches", fontsize=10)
        ax.axis("off")
    for ax in list(axes.flat)[n:]: ax.axis("off")
    fig.suptitle("EcoSysLab oak trial | closed wood meshes", fontsize=18)
    fig.tight_layout(rect=(0,0,1,.97))
    fig.savefig(output / "tree_gallery.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(rows, cols, figsize=(4*cols, 4.7*rows), squeeze=False)
    max_height = max(t["height_m"] for t in metrics)
    width = max(max(t["crown_width_x_m"], t["crown_width_z_m"]) for t in metrics)
    for ax, metric in zip(axes.flat, metrics):
        a, start, end, radius, *_ = read_skeleton(output / metric["tree"] / "skeleton.csv")
        lines = np.stack([start[:, [0,1]], end[:, [0,1]]], axis=1)
        collection = LineCollection(lines, cmap="viridis", linewidths=np.clip(radius*70, .25, 3))
        collection.set_array(a["order"])
        collection.set_clim(0, max(t["max_order"] for t in metrics))
        ax.add_collection(collection)
        ax.set(xlim=(-width*.65,width*.65), ylim=(-.1,max_height*1.05), title=metric["tree"], xlabel="X (m)", ylabel="Y (m)")
        ax.set_aspect("equal")
    for ax in list(axes.flat)[n:]: ax.axis("off")
    fig.suptitle("Native skeletons | same scale | colour = native branch order", fontsize=16)
    fig.tight_layout(rect=(0,0,.92,.96))
    color_axis = fig.add_axes([.94,.22,.012,.55])
    fig.colorbar(collection, cax=color_axis, label="Native branch order")
    fig.savefig(output / "skeleton_gallery.png", dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    x = np.arange(1,n+1)
    for ax, key, label in zip(axes.flat[:4], ["height_m", "branches", "closed_mesh_volume_m3", "trunk_diameter_at_1_3m_m"],
                              ["Height (m)", "Native branch count", "Closed mesh volume (m³)", "Trunk diameter at 1.3 m (m)"]):
        ax.bar(x, [t[key] if t[key] is not None else np.nan for t in metrics], color="#397e76")
        ax.set(xlabel="Tree", ylabel=label, xticks=x)
        ax.grid(axis="y", alpha=.2)
    for metric in metrics:
        history = np.genfromtxt(output / metric["tree"] / "growth.csv", delimiter=",", names=True)
        axes[1,1].plot(history["age_days"]/365, history["height_m"], alpha=.8, linewidth=1)
    axes[1,1].set(xlabel="Simulated age (years)", ylabel="Height (m)")
    for metric in metrics:
        with (output / metric["tree"] / "branches.csv").open() as f:
            branches = list(csv.DictReader(f))
        orders = np.array([int(row["order"]) for row in branches])
        counts = np.bincount(orders)
        axes[1,2].plot(np.arange(len(counts)), counts, marker=".", alpha=.7)
    axes[1,2].set(xlabel="Native branch order", ylabel="Branch count", yscale="log")
    fig.suptitle("12-tree trial measurements" if n == 12 else f"{n}-tree measurements", fontsize=18)
    fig.tight_layout(rect=(0,0,1,.96))
    fig.savefig(output / "measurements.png", dpi=180)
    fig.savefig(output / "measurements.pdf")
    plt.close(fig)
