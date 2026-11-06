"""Paired July-noon initial leaf azimuth arc and center sensitivity."""
import argparse
import csv
import json
from pathlib import Path
import subprocess

import diagnose_tau_variance as variance

OUTPUT = variance.ROOT/'out/tau_current/july1230_azimuth_ranges'
PRIOR = OUTPUT.parent/'july1230_fixed_phase_diagnostic'
# Arc start and total width, wrapped to [0, 360).
ARCS = {'original': (0,360), 'fixed_phase': (0,0), 'c0_w180': (270,180),
        'c0_w90': (315,90), 'c0_w30': (345,30), 'c90_w0': (90,0), 'c90_w90': (45,90)}
NEW = list(ARCS)[2:]
CONTROL_SEEDS = [7101,7155,7362]


def prepare():
    assert not (OUTPUT/'experiment.json').exists()
    prior = json.loads((PRIOR/'experiment.json').read_text())
    assert prior['status'] == 'complete'
    assert json.loads((PRIOR/'validation.json').read_text())['status'] == 'verified'
    runtime = Path(prior['runtime'])
    for path, expected in prior['hashes'].items():
        path = Path(path)
        checked = OUTPUT/'runtime_before_change'/path.relative_to(runtime) if path.is_relative_to(runtime) else path
        assert variance.sha(checked) == expected, checked
    project = OUTPUT/'project'/Path(prior['project']).name
    descriptors = {k: prior['descriptors'][k] for k in ('original','fixed_phase')}
    for condition in NEW:
        start, width = ARCS[condition]
        prefix = (f'randomize_initial_leaf_azimuth: {str(width > 0).lower()}\n'
                  f'initial_leaf_azimuth_degrees: {start}\n'
                  f'initial_leaf_azimuth_range_degrees: {width}\n').encode()
        descriptors[condition] = {}
        for genotype, relative in descriptors['original'].items():
            source = project.parent/'Assets'/relative
            data = source.read_bytes()
            assert b'initial_leaf_azimuth' not in data
            target = source.with_name(source.stem+'_'+condition+source.suffix)
            target.write_bytes(prefix+data)
            assert target.read_bytes()[len(prefix):] == data
            descriptors[condition][genotype] = target.relative_to(project.parent/'Assets').as_posix()
    inputs = [p for p in runtime.rglob('*') if p.suffix in ('.pyd','.dll','.ptx')]
    inputs += [p for p in project.parent.rglob('*') if p.is_file()]
    inputs += [p for p in (PRIOR/'runs').rglob('*') if p.suffix in ('.json','.csv')]
    inputs += [PRIOR/'experiment.json',PRIOR/'validation.json',
               variance.DATA/'sun_positions_20210726.csv',variance.DATA/'measured_bars.json']
    config = dict(status='prepared',date='2021-07-26',time_mst='12:30',seeds=variance.SEEDS,
        scales={k:1.0 for k in ARCS},azimuth_arcs=ARCS,samples=2048,bounces=4,ray_seed=18,
        project=str(project),runtime=str(runtime),descriptors=descriptors,prior_results=str(PRIOR),
        script_sha256=variance.sha(variance.__file__),driver_sha256=variance.sha(__file__),
        hashes={str(p):variance.sha(p) for p in inputs},control_seeds=CONTROL_SEEDS,
        scope='Same 30 field seeds and original trait distributions. Uniform initial azimuth within '
              'each circular arc. All arc conditions reuse the original per-plant uniform draw; '
              'only the main-axis initial phyllotaxis phase changes. No panicles. One July noon time.',
        limitations='Descriptive paired sensitivity, overlapping per-plant seed sequences; no '
                    'independence-based inference or assertion of converged population extrema.')
    (OUTPUT/'experiment.json').write_text(json.dumps(config,indent=2))
    print('Prepared 150 new configurations, six runtime controls, and 60 reused configurations',flush=True)


def check_control(seed, condition):
    current = OUTPUT/'runs'/f's{seed}'/condition
    old = PRIOR/'runs'/f's{seed}'/condition
    assert json.loads((current/f'plants_{condition}.json').read_text()) == json.loads((old/f'plants_{condition}.json').read_text())
    assert json.loads((current/'initial_leaf_azimuths.json').read_text()) == json.loads((old/'initial_leaf_azimuths.json').read_text())
    a = list(csv.DictReader((current/'measurements.csv').open()))
    b = list(csv.DictReader((old/'measurements.csv').open()))
    assert a == b, (seed,condition)
    print(f'Exact runtime control passed: {seed} {condition}',flush=True)


def run():
    config = json.loads((OUTPUT/'experiment.json').read_text())
    assert config['script_sha256'] == variance.sha(variance.__file__)
    assert config['driver_sha256'] == variance.sha(__file__)
    for path, expected in config['hashes'].items():
        assert variance.sha(path) == expected,path
    config['status'] = 'running'
    (OUTPUT/'experiment.json').write_text(json.dumps(config,indent=2))
    jobs = [(seed,c) for seed in CONTROL_SEEDS for c in ('original','fixed_phase')]
    jobs += [(seed,c) for seed in config['seeds'] for c in NEW]
    try:
        for index,(seed,condition) in enumerate(jobs):
            result = OUTPUT/'runs'/f's{seed}'/condition/'run.json'
            if result.exists():
                previous = json.loads(result.read_text())
                assert previous['status'] == 'complete' and previous['script_sha256'] == config['script_sha256']
            else:
                with (OUTPUT/f's{seed}_{condition}.log').open('w') as log:
                    subprocess.run([str(variance.PYTHON),str(Path(__file__).resolve()),'worker',
                        '--seed',str(seed),'--condition',condition],stdout=log,stderr=subprocess.STDOUT,
                        check=True,timeout=600)
            if condition in ('original','fixed_phase'):
                check_control(seed,condition)
            config['completed_jobs'] = index+1
            (OUTPUT/'experiment.json').write_text(json.dumps(config,indent=2))
            print(f'Completed {index+1}/{len(jobs)}: {seed} {condition}',flush=True)
        for path, expected in config['hashes'].items():
            assert variance.sha(path) == expected,path
        config['status'] = 'complete'
    except BaseException as error:
        config.update(status='failed',error=str(error))
        raise
    finally:
        (OUTPUT/'experiment.json').write_text(json.dumps(config,indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['prepare','run','worker'])
    parser.add_argument('--seed',type=int)
    parser.add_argument('--condition',choices=list(ARCS))
    args = parser.parse_args()
    if args.mode == 'prepare':
        prepare()
    elif args.mode == 'run':
        run()
    else:
        variance.worker(args.seed,args.condition,OUTPUT)
