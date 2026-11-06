"""Run the Position-Y PARbar experiment with the current PyDigitalAgriculture runtime."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = Path(__file__).resolve().parent / 'tau_diurnal'
SCENE = 'ManualAssets/Scenes/Sorghum_PositionY_PARBAR45_Supported.evescene'
GENOTYPES = ['Pawaga'] * 4 + ['BTX'] * 4 + ['Pawaga'] * 3 + ['BTX'] * 4 + ['Pawaga'] * 2
PROBES = 50


def summarize(records):
    panels = defaultdict(list)
    for record in records:
        if not math.isfinite(record.scalar) or record.scalar < 0:
            raise ValueError('Nonfinite or negative probe energy')
        panels[record.cultivar, record.sensor_bar_level].append(record.scalar)
    expected = {(g, level) for g in ('BTX', 'Pawaga') for level in ('top', 'middle', 'bottom')}
    if set(panels) != expected or any(len(v) != PROBES for v in panels.values()):
        raise ValueError('Expected six complete 50-probe panels')
    result = {}
    for genotype in ('BTX', 'Pawaga'):
        top = sum(panels[genotype, 'top']) / PROBES
        values = panels[genotype, 'bottom']
        bottom = sum(values) / PROBES
        if top <= 0:
            raise ValueError('Above-canopy reference must be positive')
        result.update({f'{genotype}_top': top, f'{genotype}_bottom': bottom,
                       f'{genotype}_tau': bottom / top,
                       f'{genotype}_bottom_cv': math.sqrt(sum((x-bottom)**2 for x in values)/PROBES)/bottom if bottom else 0.0})
    return result


def check_probe_positions(records, reference):
    panels = defaultdict(list)
    for record in records:
        panels[f'{record.cultivar}:{record.sensor_bar_level}'].append(record)
    report = {}
    for key, expected in reference.items():
        samples = panels[key]
        if len(samples) != PROBES:
            raise ValueError(f'Wrong probe count for {key}')
        center = [sum(getattr(r.position, axis) for r in samples)/PROBES for axis in 'xyz']
        error = max(abs(a-b) for a,b in zip(center, expected['center']))
        span = math.sqrt(sum((getattr(samples[-1].position, a)-getattr(samples[0].position, a))**2 for a in 'xyz'))
        axis = math.degrees(math.atan2(samples[-1].position.z-samples[0].position.z,
                                      samples[-1].position.x-samples[0].position.x))
        angle_error = abs((axis-expected['axis_deg']+90) % 180-90)
        if error > 1e-4 or abs(span - expected['span_m']) > 1e-4 or angle_error > .01:
            raise ValueError(f'Probe layout differs from reference: {key}, center={center}, span={span}')
        if any(r.normal.y < .99 for r in samples):
            raise ValueError(f'Non-upward probe in {key}')
        report[key] = dict(center=center, span_m=span, axis_deg=axis, count=len(samples))
    return report


def main():
    from fit_tau_parbar_supports import supported_bar_reference

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--module-dir', type=Path, default=ROOT / 'out/install/vs2026-x64/python')
    parser.add_argument('--project', type=Path, default=ROOT / 'Resources/DigitalAgricultureProject/test_lsystem_sorghum.eveproj')
    parser.add_argument('--sun-csv', type=Path, default=DATA / 'sun_positions_20210726.csv')
    parser.add_argument('--btx-descriptor', default='BTX623_20210726_v6.sorghumls')
    parser.add_argument('--pawaga-descriptor', default='Pawaga_20210726_v8.sorghumls')
    parser.add_argument('--seed', type=int, default=7101)
    parser.add_argument('--samples', type=int, default=128)
    parser.add_argument('--bounces', type=int, default=4)
    parser.add_argument('--leaf-vsub', type=float, default=.02)
    parser.add_argument('--leaf-hsub', type=int, default=4)
    parser.add_argument('--output', type=Path, default=ROOT / 'out/tau_current/july26_s7101')
    args = parser.parse_args()
    if args.samples <= 0 or args.bounces < 0 or args.seed < 0:
        parser.error('Samples must be positive; bounces and seed must be nonnegative')
    module_dir, project, output, sun_csv = (p.resolve() for p in (args.module_dir, args.project, args.output, args.sun_csv))
    if (output / 'tau.csv').exists():
        raise FileExistsError(f'Choose a new output directory; existing results: {output}')
    output.mkdir(parents=True, exist_ok=True)
    with sun_csv.open(newline='') as f:
        bins = [r for r in csv.DictReader(f) if r['valid'] == 'True']
    if not bins or len({r['time_mst'] for r in bins}) != len(bins):
        raise ValueError('Sun table has no valid times or duplicate times')
    descriptors = {'BTX': f'ManualAssets/Descriptors/TauSeasonal/{args.btx_descriptor}',
                   'Pawaga': f'ManualAssets/Descriptors/TauSeasonal/{args.pawaga_descriptor}'}
    dll_dirs = [os.add_dll_directory(str(p)) for p in (module_dir, module_dir / 'Packages') if os.name == 'nt' and p.is_dir()]
    sys.path.insert(0, str(module_dir))
    os.chdir(module_dir)
    import PyDigitalAgriculture as evo
    if Path(evo.__file__).resolve().parent != module_dir:
        raise RuntimeError(f'Unexpected binding: {evo.__file__}')
    print(f'Binding: {evo.__file__}\nProject: {project}\nSeed: {args.seed}; {len(bins)} valid times', flush=True)
    start = time.monotonic()
    manifest = dict(status='running', binding=evo.__file__, project=str(project), sun_csv=str(sun_csv), seed=args.seed,
                    descriptors=descriptors, descriptor_schedule='fixed_for_day', parbar_order='south_to_north',
                    samples=args.samples, bounces=args.bounces, push_normal_distance=.001,
                    leaf_vsub=args.leaf_vsub, leaf_hsub=args.leaf_hsub,
                    measurement='Current engine CudaModule::EstimateIlluminationRayTracing with PARSensorGroup probes; unchanged CUDA integrator',
                    hashes={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                            [Path(evo.__file__), sun_csv, DATA/'measured_bars.json', module_dir/'EvoEngine_SDK.dll',
                             module_dir/'Packages/LSystemPackage.dll', module_dir/'Packages/DigitalAgriculturePackage.dll',
                             project.parent/'Assets'/SCENE]
                            + [project.parent / 'Assets' / p for p in descriptors.values()]})
    handle = None
    try:
        if not evo.RunLSystemSorghumProject(str(project), str(module_dir/'Packages'),
                                           SCENE):
            raise RuntimeError('Project initialization failed')
        if evo.ConfigureSorghumLsPlantingGrid(GENOTYPES, 12, .3, 1.0, 0.0, 4.0) != 204:
            raise RuntimeError('Planting grid count differs')
        if evo.InstantiateSorghumLsPlantsFromPlantingMarkers() != 204:
            raise RuntimeError('Plant instantiation count differs')
        manifest['ground_before_fit'] = evo.GetIlluminationGroundState()
        if not evo.EnsureIlluminationSoilContext() or not evo.ValidateIlluminationContext():
            raise RuntimeError('Ground coverage or soil texture context is incomplete')
        if evo.ConformSorghumLsPlantsToGroundMesh() != 204:
            raise RuntimeError('Ground placement count differs')
        if evo.SetSorghumLsGenotypeDescriptors(descriptors, False, args.seed) != 204:
            raise RuntimeError('Descriptor assignment count differs')
        if evo.ConfigureSorghumLsLeafMeshQuality(args.leaf_vsub, args.leaf_hsub, False, True, False) != 204:
            raise RuntimeError('Leaf mesh configuration count differs')
        print('Growing 204 plants...', flush=True)
        if evo.GrowSorghumLsPlantsToAdulthood(args.seed, '', False, True) != 204 or not evo.WaitForProjectIdle():
            raise RuntimeError('Plant growth did not complete')
        if not evo.ValidateIlluminationContext():
            raise RuntimeError('Grown canopy extends outside the soil footprint')
        manifest['ground_after_fit'] = evo.GetIlluminationGroundState()
        plants = list(evo.GetSorghumLsFieldState())
        if len(plants) != 204 or Counter(p['genotype'] for p in plants) != {'BTX': 96, 'Pawaga': 108}:
            raise RuntimeError('Unexpected final plant composition')
        if not all(p['has_geometry'] and p['leaf_count'] > 0 and p['pending_geometry_updates'] == 0
                   and p['published_triangles'] == p['snapshot_triangles'] > 0 for p in plants):
            raise RuntimeError('Plant geometry is incomplete')
        if not all(p['textures_loaded'] for p in plants):
            raise RuntimeError('Leaf or stem textures did not resolve')
        expected_xz = {(round((col-5.5)*.3, 4), float(row-4)) for row in range(17) for col in range(12)}
        if {tuple(round(v, 4) for v in p['position'][::2]) for p in plants} != expected_xz:
            raise RuntimeError('Plant coordinates do not match the full layout')
        (output/'plants.json').write_text(json.dumps(plants, indent=2))
        handle = evo.CreateParbarTopFaceSensorGroup(PROBES)
        records = list(evo.GetParbarTopFaceSensorResults(handle, PROBES))
        reference = supported_bar_reference(json.loads((DATA/'measured_bars.json').read_text()))
        manifest['probe_layout'] = check_probe_positions(records, reference)
        manifest['plant_count'] = len(plants)
        print('Verified 204 plants and 300 probes; measuring...', flush=True)
        with (output/'tau.csv').open('w', newline='') as summary_file, (output/'probes.csv').open('w', newline='') as probe_file:
            columns = ['time_mst', 'elevation_deg', 'azimuth_deg', 'valid']
            columns += [f'{g}_{quantity}' for g in ('BTX', 'Pawaga') for quantity in ('top', 'bottom', 'tau', 'bottom_cv')]
            summary = csv.DictWriter(summary_file, fieldnames=columns)
            summary.writeheader()
            probes = csv.writer(probe_file)
            probes.writerow(['time_mst','cultivar','level','column','x','y','z','scalar'])
            for index, b in enumerate(bins):
                angles = evo.Vec3()
                angles.x, angles.y, angles.z = float(b['elevation_deg']), 270-float(b['azimuth_deg']), 0
                evo.SetSunDirection(angles)
                evo.EstimatePARSensors(handle, args.samples, args.bounces, .001, index+1)
                records = list(evo.GetParbarTopFaceSensorResults(handle, PROBES))
                values = summarize(records)
                summary.writerow({**b, **values})
                probes.writerows([b['time_mst'], r.cultivar, r.sensor_bar_level, r.column,
                                  r.position.x, r.position.y, r.position.z, r.scalar] for r in records)
                summary_file.flush()
                probe_file.flush()
                print(f"[{index+1}/{len(bins)}] {b['time_mst']}: BTX={values['BTX_tau']:.5f}, Pawaga={values['Pawaga_tau']:.5f}", flush=True)
        manifest.update(status='complete', valid_times=len(bins), probe_rows=len(bins)*6*PROBES,
                        elapsed_seconds=time.monotonic()-start)
    except BaseException as error:
        manifest.update(status='failed', error=str(error))
        raise
    finally:
        (output/'run.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
        if handle is not None:
            evo.DeleteRuntimeAsset(handle)
        evo.Terminate()
    print(f'Complete: {output}', flush=True)


if __name__ == '__main__':
    main()
