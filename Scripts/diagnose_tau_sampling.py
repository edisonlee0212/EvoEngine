"""Measure a fixed July canopy repeatedly and capture the BTx623 low-bar surroundings."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

from run_tau_diurnal import DATA, PROBES, ROOT, SCENE, GENOTYPES, check_probe_positions, summarize
from fit_tau_parbar_supports import supported_bar_reference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--label', choices=['low', 'middle', 'high'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    reference = ROOT/f'out/tau_current/tau_1000_campaign/july_september_no_panicle/runs/2021-07-26/s{args.seed}'
    old = json.loads((reference/'run.json').read_text())
    assert old['status'] == 'complete'
    for name, sha in old['hashes'].items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == sha, name
    output.mkdir(parents=True, exist_ok=False)
    project = Path(old['project'])
    runtime = Path(old['binding']).parent
    dll_dirs = [os.add_dll_directory(str(p)) for p in (runtime, runtime/'Packages')]
    sys.path.insert(0, str(runtime))
    os.chdir(runtime)
    import PyDigitalAgriculture as evo

    def vec(values):
        v = evo.Vec3()
        v.x, v.y, v.z = values
        return v

    def field_state():
        return json.loads(json.dumps(list(evo.GetSorghumLsFieldState())))

    sun_rows = list(csv.DictReader((DATA/'sun_positions_20210726.csv').open()))
    valid_rows = [r for r in sun_rows if r['valid'] == 'True']
    sun_index, sun = next((i,r) for i,r in enumerate(valid_rows) if r['time_mst'] == '12:30')
    baseline = next(r for r in csv.DictReader((reference/'tau.csv').open()) if r['time_mst'] == '12:30')
    manifest = dict(status='running', plant_seed=args.seed, label=args.label, reference=str(reference),
                    time_mst='12:30', date='2021-07-26', samples=[128,512,2048], ray_seeds=list(range(1001,1011)),
                    bounces=4, hashes=old['hashes'], baseline_ray_seed=sun_index+1,
                    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    executable=sys.executable, captures=[])
    sensor = None
    start = time.monotonic()
    try:
        assert evo.RunLSystemSorghumProject(str(project), str(runtime/'Packages'), SCENE)
        assert evo.ConfigureSorghumLsPlantingGrid(GENOTYPES,12,.3,1.,0.,4.) == 204
        assert evo.InstantiateSorghumLsPlantsFromPlantingMarkers() == 204
        assert evo.EnsureIlluminationSoilContext() and evo.ValidateIlluminationContext()
        assert evo.ConformSorghumLsPlantsToGroundMesh() == 204
        assert evo.SetSorghumLsGenotypeDescriptors(old['descriptors'],False,args.seed) == 204
        assert evo.ConfigureSorghumLsLeafMeshQuality(.02,4,False,True,False) == 204
        assert evo.GrowSorghumLsPlantsToAdulthood(args.seed,'',False,True) == 204
        assert evo.WaitForProjectIdle() and evo.ValidateIlluminationContext()
        plants = field_state()
        assert plants == json.loads((reference/'plants.json').read_text()), 'Archived plant state differs'
        (output/'plants.json').write_text(json.dumps(plants,indent=2))
        manifest['plant_state_matches_reference'] = True
        manifest['ground'] = evo.GetIlluminationGroundState()
        sensor = evo.CreateParbarTopFaceSensorGroup(PROBES)
        layout = check_probe_positions(list(evo.GetParbarTopFaceSensorResults(sensor,PROBES)),
                                       supported_bar_reference(json.loads((DATA/'measured_bars.json').read_text())))
        manifest['probe_layout'] = layout
        evo.SetSunDirection(vec([float(sun['elevation_deg']),270-float(sun['azimuth_deg']),0]))
        with (output/'measurements.csv').open('w',newline='') as f, (output/'probes.csv').open('w',newline='') as pf:
            columns = ['plant_seed','samples','ray_seed','baseline']
            columns += [f'{g}_{k}' for g in ['BTX','Pawaga'] for k in ['top','bottom','tau','bottom_cv']]
            writer = csv.DictWriter(f, fieldnames=columns)
            writer.writeheader()
            pw = csv.writer(pf)
            pw.writerow(['plant_seed','samples','ray_seed','baseline','genotype','level','column','x','y','z','scalar'])
            settings = [(128,sun_index+1,True)]+[(samples,ray_seed,False) for samples in [128,512,2048] for ray_seed in range(1001,1011)]
            for samples, ray_seed, is_baseline in settings:
                evo.EstimatePARSensors(sensor,samples,4,.001,ray_seed)
                probes = list(evo.GetParbarTopFaceSensorResults(sensor,PROBES))
                values = summarize(probes)
                writer.writerow(dict(plant_seed=args.seed,samples=samples,ray_seed=ray_seed,baseline=is_baseline,**values))
                pw.writerows([args.seed,samples,ray_seed,is_baseline,p.cultivar,p.sensor_bar_level,p.column,
                              p.position.x,p.position.y,p.position.z,p.scalar] for p in probes)
                f.flush()
                pf.flush()
                if is_baseline:
                    error = {g:values[g+'_tau']-float(baseline[g+'_tau']) for g in ['BTX','Pawaga']}
                    manifest['baseline_tau_error'] = error
                    assert max(abs(v) for v in error.values()) < .01, 'Baseline reproduction differs materially'
                print(f'seed={args.seed} samples={samples} ray_seed={ray_seed} BTX={values["BTX_tau"]:.6f} Pawaga={values["Pawaga_tau"]:.6f}',flush=True)
            assert field_state() == plants, 'Plants changed during measurements'
        center = layout['BTX:bottom']['center']
        elev, az = math.radians(float(sun['elevation_deg'])), math.radians(float(sun['azimuth_deg']))
        sunlight = [math.cos(elev)*math.cos(az), math.sin(elev), math.cos(elev)*math.sin(az)]
        views = [
            ('bar_oblique', [2.2,1.2,-.2], center, [0,1,0], 50),
            ('bar_from_sun', [a+4*b for a,b in zip(center,sunlight)], center, [0,0,1], 43),
            ('bar_looking_up', [center[0],center[1]+.012,center[2]],
             [center[0],center[1]+3,center[2]], [0,0,1], 110),
        ]
        for name, position, target, up, fov in views:
            assert evo.SetMainCameraLookAt(vec(position),vec(target),vec(up),fov)
            path = output/f'{name}.png'
            assert evo.CaptureCurrentSceneCuda(1440,1080,str(path),128,4,2.2,0.)
            manifest['captures'].append(dict(name=name,path=str(path),position=position,target=target,up=up,fov=fov))
            print(f'Captured {path}',flush=True)
        assert field_state() == plants
        for name, sha in old['hashes'].items():
            assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == sha, name
        manifest.update(status='complete',fixed_plant_state_verified=True,elapsed_seconds=time.monotonic()-start)
    except BaseException as e:
        manifest.update(status='failed',error=str(e),elapsed_seconds=time.monotonic()-start)
        raise
    finally:
        (output/'run.json').write_text(json.dumps(manifest,indent=2))
        if sensor is not None:
            evo.DeleteRuntimeAsset(sensor)
        evo.Terminate()


if __name__ == '__main__':
    main()
