"""Derive a Position-Y scene with supports fitted to its raised reference bars."""
import argparse
import base64
import hashlib
import json
import re
import uuid
from pathlib import Path

import numpy as np


SUPPORTED_SCENE = 'ManualAssets/Scenes/Sorghum_PositionY_PARBAR45_Supported.evescene'
MIDDLE_HEIGHT_M = 0.8


def north_order_transform(reference, cultivar):
    bottom = reference[f'{cultivar}:bottom']['center']
    top = reference[f'{cultivar}:top']['center']
    transform = np.diag([-1., 1., -1., 1.])
    transform[0, 3] = bottom[0] + top[0]
    transform[2, 3] = bottom[2] + top[2]
    return transform


def north_order_reference(reference):
    result = {}
    for key, panel in reference.items():
        transform = north_order_transform(reference, key.split(':')[0])
        center = (transform @ np.array([*panel['center'], 1.]))[:3].tolist()
        result[key] = dict(panel, center=center, axis_deg=panel['axis_deg'] + 180.)
    return result


def supported_bar_reference(reference):
    result = north_order_reference(reference)
    for cultivar in ('BTX', 'Pawaga'):
        result[f'{cultivar}:middle']['center'][1] = MIDDLE_HEIGHT_M
    return result


def matrix_entry(handle):
    return re.compile(rf'(- h: {handle}\s*\n\s*dc:\s*\n\s*- d: !!binary ")([^\"]+)'
                      r'("\s*\n\s*- d: !!binary ")([^\"]+)(")')


def read_scene(path):
    text = path.read_text(encoding='utf-8')
    entities = {}
    for match in re.finditer(r'  - n: ([^\n]+)\n(.*?)(?=  - n: |systems_:)', text, re.S):
        block = match[2]
        handle = int(re.search(r'    h: (\d+)', block)[1])
        parent = re.search(r'    p: (\d+)', block)
        mesh = re.search(r'        mesh:\s+asset_handle_: (\d+)', block)
        entry = matrix_entry(handle).search(text)
        entities[handle] = dict(name=match[1].strip(), parent=int(parent[1]) if parent else None,
                                mesh=int(mesh[1]) if mesh else None,
                                local=np.frombuffer(base64.b64decode(entry[2]), dtype='<f4').reshape(4, 4).T.astype(float))

    def world(handle):
        entity = entities[handle]
        return world(entity['parent']) @ entity['local'] if entity['parent'] else entity['local']

    for handle, entity in entities.items():
        entity['world'] = world(handle)
        ancestor = handle
        while entities[ancestor]['parent']:
            ancestor = entities[ancestor]['parent']
        entity['root'] = ancestor
    return text, entities


def extend_pole(bottom, top, rise):
    if top + rise <= bottom:
        raise ValueError('Raised pole must retain positive height')
    transform = np.eye(4)
    transform[1, 1] = (top + rise - bottom) / (top - bottom)
    transform[1, 3] = bottom * (1 - transform[1, 1])
    return transform


def fit_supports(project):
    assets = project / 'Assets'
    scenes = assets / 'ManualAssets/Scenes'
    original_path = scenes / 'Sorghum_4x10_PARBAR.evescene'
    source_path = scenes / 'Sorghum_PositionY_PARBAR45.evescene'
    _, original = read_scene(original_path)
    text, entities = read_scene(source_path)
    reference = json.loads((Path(__file__).parent / 'tau_diurnal/measured_bars.json').read_text())
    meshes = {}

    def vertices(handle):
        if handle not in meshes:
            path = assets / f'ManualAssets/Context/Mesh_{handle}.evemesh'
            payload = re.search(r'^vertices_: !!binary "([^\"]+)"', path.read_text(), re.M)[1]
            positions = np.frombuffer(base64.b64decode(payload), dtype='<f4').reshape(-1, 20)[:, :3]
            meshes[handle] = np.column_stack((positions, np.ones(len(positions)))).T
        return meshes[handle]

    report = dict(source=str(source_path), source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(), rigs={})
    for cultivar in ('BTX', 'Pawaga'):
        root = next(h for h, e in entities.items() if e['name'] == f'PARBAR_{cultivar}')

        def part(name):
            matches = [h for h, e in entities.items() if e['root'] == root and e['name'] == name and e['mesh']]
            if len(matches) != 1:
                raise ValueError(f'Expected one {cultivar}/{name}, got {matches}')
            return matches[0]

        bar = part(f'PARBAR_{cultivar}_TopSensorBarMesh')
        points = vertices(entities[bar]['mesh'])
        baseline = entities[root]['world'] @ np.linalg.inv(original[root]['world']) @ original[bar]['world']
        rise = float(np.mean((entities[bar]['world'] @ points)[1] - (baseline @ points)[1]))
        changes = []
        for name in ('modular_metal_gutter_section.004', 'modular_metal_gutter_coupler.001', 'model.003'):
            handle = part(name)
            entity = entities[handle]
            before = entity['world'] @ vertices(entity['mesh'])
            adjustment = np.eye(4)
            adjustment[1, 3] = rise
            if name == 'modular_metal_gutter_section.004':
                adjustment = extend_pole(float(before[1].min()), float(before[1].max()), rise)
            world = adjustment @ entity['world']
            local = np.linalg.inv(entities[entity['parent']]['world']) @ world
            entity.update(local=local, world=world)
            after = world @ vertices(entity['mesh'])
            assert np.allclose(before[[0, 2]], after[[0, 2]])
            changes.append(dict(entity=handle, name=name, before_y=[float(before[1].min()), float(before[1].max())],
                                after_y=[float(after[1].min()), float(after[1].max())]))
        middle = entities[part(f'PARBAR_{cultivar}_MiddleSensorBarMesh')]
        middle_world = middle['world'].copy()
        middle_world[1, 3] += MIDDLE_HEIGHT_M - reference[f'{cultivar}:middle']['center'][1]
        middle.update(world=middle_world,
                      local=np.linalg.inv(entities[middle['parent']]['world']) @ middle_world)
        rotation = north_order_transform(reference, cultivar)
        for handle, entity in entities.items():
            if entity['root'] != root:
                continue
            world = rotation @ entity['world']
            local = world if handle == root else entity['local']
            blobs = [base64.b64encode(m.T.astype('<f4').tobytes()).decode() for m in (local, world)]
            text, count = matrix_entry(handle).subn(lambda m: m[1] + blobs[0] + m[3] + blobs[1] + m[5], text)
            assert count == 1
        report['rigs'][cultivar] = dict(rise_m=rise, changed_parts=changes,
                                       middle_height_m=MIDDLE_HEIGHT_M,
                                       horizontal_rotation_degrees=180,
                                       pivot_xz_m=[rotation[0, 3] / 2, rotation[2, 3] / 2])

    report['expected_bars'] = supported_bar_reference(reference)

    output = assets / SUPPORTED_SCENE
    output.write_text(text, encoding='utf-8')
    sidecar = Path(str(output) + '.evefilemeta')
    if not sidecar.exists():
        handle = uuid.uuid4().int & ((1 << 63) - 1)
        assert not any(re.search(rf'asset_handle_:\s*{handle}\s*$', p.read_text(), re.M)
                       for p in assets.rglob('*.evefilemeta'))
        sidecar.write_text(f'asset_extension_: .evescene\nasset_file_name_: {output.stem}\n'
                           f'asset_type_name_: Scene\nasset_handle_: {handle}\n', encoding='utf-8')
    report.update(output=str(output), output_sha256=hashlib.sha256(output.read_bytes()).hexdigest())
    return report


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=root / 'Resources/DigitalAgricultureProject')
    parser.add_argument('--report', type=Path, default=root / 'out/tau_current/parbar_support_fit.json')
    args = parser.parse_args()
    report = fit_supports(args.project.resolve())
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
