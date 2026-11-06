"""Matched July-noon sensitivity to descriptor standard deviations; production assets stay unchanged."""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from run_tau_diurnal import ROOT, DATA, SCENE, GENOTYPES, PROBES, summarize, check_probe_positions

OUTPUT = ROOT / 'out/tau_current/july1230_variance_diagnostic'
ARCHIVE = ROOT / 'out/tau_current/tau_1000_campaign/july_september_no_panicle/runs/2021-07-26'
SCALES = {'original': 1., 'half_sd': .5, 'quarter_sd': .25}
SEEDS = list(range(7101, 7370, 9))
PYTHON = Path('C:/Users/sdjkl/miniconda3/envs/evoengine/python.exe')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def scaled_descriptor(source, scale):
    if not 0 < scale <= 1:
        raise ValueError('Use positive standard-deviation scales no larger than one')
    result = copy.deepcopy(source)
    for value in result.values():
        if isinstance(value, dict) and 'deviation' in value:
            if isinstance(value['deviation'], dict):
                for key in ('min_value', 'max_value'):
                    value['deviation'][key] *= scale
            else:
                value['deviation'] *= scale
    return result


def prepare():
    from ruamel.yaml import YAML
    yaml = YAML(typ='safe')
    reference = json.loads((ARCHIVE/'s7101/run.json').read_text())
    assert reference['status'] == 'complete'
    for path, expected in reference['hashes'].items():
        assert sha(path) == expected, path
    OUTPUT.mkdir(parents=True, exist_ok=False)
    project_dir = OUTPUT/'project'
    shutil.copytree(Path(reference['project']).parent, project_dir)
    audit, descriptors = {}, {}
    for label, scale in SCALES.items():
        descriptors[label] = {}
        for genotype, relative in reference['descriptors'].items():
            original = project_dir/'Assets'/relative
            source = yaml.load(original)
            assert source['enable_panicle'] is False
            audit[genotype] = {key: value for key, value in source.items()
                               if isinstance(value, dict) and 'deviation' in value}
            if scale == 1:
                target = original
            else:
                target = original.with_name(original.stem+'_'+label+original.suffix)
                modified = scaled_descriptor(source, scale)
                with target.open('w') as stream:
                    yaml.dump(modified, stream)
                assert yaml.load(target) == modified
            descriptors[label][genotype] = target.relative_to(project_dir/'Assets').as_posix()
    manifest = dict(status='prepared', date='2021-07-26', time_mst='12:30', seeds=SEEDS,
                    scales=SCALES, samples=2048, ray_seed=18, bounces=4,
                    project=str(project_dir/Path(reference['project']).name),
                    runtime=str(Path(reference['binding']).parent), descriptors=descriptors,
                    archive_hashes=reference['hashes'],
                    project_hashes={str(p): sha(p) for p in project_dir.rglob('*') if p.is_file()},
                    script_sha256=sha(__file__),
                    scope='Scale all explicitly serialized descriptor deviations, preserve means and curves; '
                          'retain intrinsic random phyllotaxis phase, wind direction, leaf phases and atlas variants. '
                          'Absent descriptor fields retain inline defaults (including disabled panicle parameters).',
                    pairing_limit='Same field seed and plant assignments; changed rounded counts can change '
                                  'subsequent random draws, so this does not lock individual leaf azimuths.',
                    seed_limit='Predetermined subset of original campaign, not selected by tau; field seeds '
                               'reuse overlapping per-plant seed sequences, so no independence-based inference.')
    (OUTPUT/'experiment.json').write_text(json.dumps(manifest, indent=2))
    (OUTPUT/'descriptor_audit.json').write_text(json.dumps(audit, indent=2))
    print(f'Prepared {len(SEEDS)} seeds x {len(SCALES)} scales in {OUTPUT}')


def worker(seed, condition, output_root=OUTPUT):
    from fit_tau_parbar_supports import supported_bar_reference
    config = json.loads((output_root/'experiment.json').read_text())
    output = output_root/'runs'/f's{seed}'/condition
    output.mkdir(parents=True, exist_ok=False)
    runtime = Path(config['runtime'])
    dll_dirs = [os.add_dll_directory(str(p)) for p in (runtime, runtime/'Packages')]
    sys.path.insert(0, str(runtime))
    os.chdir(runtime)
    import PyDigitalAgriculture as evo
    sensor = None
    start = time.monotonic()
    manifest = dict(status='running', seed=seed, condition=condition, executable=sys.executable, script_sha256=sha(__file__))
    try:
        assert evo.RunLSystemSorghumProject(config['project'], str(runtime/'Packages'), SCENE)
        assert evo.ConfigureSorghumLsPlantingGrid(GENOTYPES,12,.3,1.,0.,4.) == 204
        assert evo.InstantiateSorghumLsPlantsFromPlantingMarkers() == 204
        assert evo.EnsureIlluminationSoilContext() and evo.ValidateIlluminationContext()
        assert evo.ConformSorghumLsPlantsToGroundMesh() == 204
        assert evo.ConfigureSorghumLsLeafMeshQuality(.02,4,False,True,False) == 204
        sensor = evo.CreateParbarTopFaceSensorGroup(PROBES)
        manifest['probes'] = check_probe_positions(list(evo.GetParbarTopFaceSensorResults(sensor,PROBES)),
            supported_bar_reference(json.loads((DATA/'measured_bars.json').read_text())))
        sun = next(row for row in csv.DictReader((DATA/'sun_positions_20210726.csv').open())
                   if row['time_mst'] == '12:30')
        angles = evo.Vec3()
        angles.x, angles.y, angles.z = float(sun['elevation_deg']),270-float(sun['azimuth_deg']),0
        evo.SetSunDirection(angles)
        archived_plants = json.loads((ARCHIVE/f's{seed}/plants.json').read_text())
        archived_tau = next(row for row in csv.DictReader((ARCHIVE/f's{seed}/tau.csv').open())
                            if row['time_mst'] == '12:30')
        with (output/'measurements.csv').open('w',newline='') as stream, (output/'probes.csv').open('w',newline='') as pf:
            writer = csv.DictWriter(stream,fieldnames=['seed','condition','sd_scale','samples','ray_seed']+
                [f'{g}_{k}' for g in ('BTX','Pawaga') for k in ('top','bottom','tau','bottom_cv')])
            writer.writeheader()
            pw = csv.writer(pf)
            pw.writerow(['seed','condition','samples','ray_seed','genotype','level','column','x','y','z','scalar'])
            for label, scale in [(condition,config['scales'][condition])]:
                assert evo.SetSorghumLsGenotypeDescriptors(config['descriptors'][label],False,seed) == 204
                assert evo.GrowSorghumLsPlantsToAdulthood(seed,'',False,True) == 204
                assert evo.WaitForProjectIdle() and evo.ValidateIlluminationContext()
                plants = json.loads(json.dumps(list(evo.GetSorghumLsFieldState())))
                assert len(plants) == 204 and all(p['textures_loaded'] and p['has_geometry'] and
                    p['pending_geometry_updates'] == 0 and p['published_triangles'] == p['snapshot_triangles'] > 0
                    for p in plants)
                assert [(p['name'],p['seed'],p['position']) for p in plants] == [
                    (p['name'],p['seed'],p['position']) for p in archived_plants]
                if label == 'original':
                    assert plants == archived_plants
                if 'initial_leaf_azimuth' in config or 'azimuth_arcs' in config:
                    phases = dict(evo.GetSorghumLsInitialLeafAzimuths())
                    assert set(phases) == {p['name'] for p in plants}
                    if 'azimuth_arcs' in config:
                        arc_start, width = config['azimuth_arcs'][label]
                        reference_phases = json.loads((Path(config['prior_results'])/'runs'/f's{seed}'/
                                                       'original/initial_leaf_azimuths.json').read_text())
                        errors = [abs((value-(arc_start+reference_phases[name]/360*width)+180)%360-180)
                                  for name,value in phases.items()]
                        assert max(errors) < 0.0001
                        manifest['max_phase_error_degrees'] = max(errors)
                        if width == 0:
                            assert all(value == arc_start for value in phases.values())
                    else:
                        expected_phase = config['initial_leaf_azimuth'][label]
                        if expected_phase is not None:
                            assert all(v == expected_phase for v in phases.values())
                        else:
                            assert max(phases.values())-min(phases.values()) > 1
                    assert [p['leaf_count'] for p in plants] == [p['leaf_count'] for p in archived_plants]
                    (output/'initial_leaf_azimuths.json').write_text(json.dumps(phases,indent=2))
                (output/f'plants_{label}.json').write_text(json.dumps(plants,indent=2))
                settings = [128,2048] if label == 'original' else [2048]
                for samples in settings:
                    evo.EstimatePARSensors(sensor,samples,4,.001,18)
                    probes = list(evo.GetParbarTopFaceSensorResults(sensor,PROBES))
                    values = summarize(probes)
                    writer.writerow(dict(seed=seed,condition=label,sd_scale=scale,samples=samples,ray_seed=18,**values))
                    pw.writerows([seed,label,samples,18,p.cultivar,p.sensor_bar_level,p.column,
                                  p.position.x,p.position.y,p.position.z,p.scalar] for p in probes)
                    stream.flush()
                    pf.flush()
                    if samples == 128:
                        manifest['baseline_error'] = {g: values[g+'_tau']-float(archived_tau[g+'_tau'])
                                                      for g in ('BTX','Pawaga')}
                        assert max(abs(e) for e in manifest['baseline_error'].values()) < 1e-7
                    print(f'{seed} {label} samples={samples} BTX={values["BTX_tau"]:.6f} Pawaga={values["Pawaga_tau"]:.6f}',flush=True)
                assert json.loads(json.dumps(list(evo.GetSorghumLsFieldState()))) == plants
        manifest.update(status='complete', elapsed_seconds=time.monotonic()-start)
    except BaseException as error:
        manifest.update(status='failed',error=str(error),elapsed_seconds=time.monotonic()-start)
        raise
    finally:
        (output/'run.json').write_text(json.dumps(manifest,indent=2))
        if sensor is not None:
            evo.DeleteRuntimeAsset(sensor)
        evo.Terminate()


def run():
    config = json.loads((OUTPUT/'experiment.json').read_text())
    config.update(status='running',script_sha256=sha(__file__),execution='One fresh process per seed and condition')
    (OUTPUT/'experiment.json').write_text(json.dumps(config,indent=2))
    for i, seed in enumerate(config['seeds']):
        for label in config['scales']:
            with (OUTPUT/f's{seed}_{label}.log').open('w') as log:
                subprocess.run([str(PYTHON),str(Path(__file__).resolve()),'worker','--seed',str(seed),
                                '--condition',label],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=600)
            print(f'Completed seed {seed}: {label}',flush=True)
        print(f'Completed {i+1}/{len(config["seeds"])}: seed {seed}',flush=True)
    for path, expected in config['archive_hashes'].items():
        assert sha(path) == expected, path
    for path, expected in config['project_hashes'].items():
        assert sha(path) == expected, path
    config['status'] = 'complete'
    (OUTPUT/'experiment.json').write_text(json.dumps(config,indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['prepare','run','worker'])
    parser.add_argument('--seed',type=int)
    parser.add_argument('--condition',choices=list(SCALES))
    args = parser.parse_args()
    if args.mode == 'prepare':
        prepare()
    elif args.mode == 'run':
        run()
    else:
        worker(args.seed,args.condition)
