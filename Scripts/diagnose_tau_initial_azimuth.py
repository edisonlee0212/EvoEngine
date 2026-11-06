"""Compare random versus fixed initial leaf azimuth, retaining original trait distributions."""
import argparse
import json
from pathlib import Path
import subprocess

import diagnose_tau_variance as variance

OUTPUT = variance.ROOT/'out/tau_current/july1230_fixed_phase_diagnostic'
CONDITIONS = {'original': None, 'fixed_phase': 0.0}


def prepare():
    reference = json.loads((variance.ARCHIVE/'s7101/run.json').read_text())
    assert not (OUTPUT/'experiment.json').exists()
    runtime = Path(reference['binding']).parent
    project = OUTPUT/'project'/Path(reference['project']).name
    descriptors = {'original':reference['descriptors'], 'fixed_phase':{}}
    prefix = b'randomize_initial_leaf_azimuth: false\ninitial_leaf_azimuth_degrees: 0\n'
    for genotype, relative in reference['descriptors'].items():
        source = project.parent/'Assets'/relative
        data = source.read_bytes()
        assert b'randomize_initial_leaf_azimuth' not in data and b'initial_leaf_azimuth_degrees' not in data
        target = source.with_name(source.stem+'_fixed_initial_azimuth_0.sorghumls')
        target.write_bytes(prefix+data)
        assert target.read_bytes()[len(prefix):] == data
        descriptors['fixed_phase'][genotype] = target.relative_to(project.parent/'Assets').as_posix()
    for path, expected in reference['hashes'].items():
        source = Path(path)
        checked = OUTPUT/'runtime_before_change'/source.relative_to(runtime) if source.is_relative_to(runtime) else source
        assert variance.sha(checked) == expected, checked
    runtime_inputs = [p for p in runtime.rglob('*') if p.suffix in ('.pyd','.dll','.ptx')]
    project_inputs = [p for p in project.parent.rglob('*') if p.is_file()]
    config = dict(status='prepared',date='2021-07-26',time_mst='12:30',seeds=variance.SEEDS,
        scales={label:1.0 for label in CONDITIONS},initial_leaf_azimuth=CONDITIONS,
        samples=2048,bounces=4,ray_seed=18,project=str(project),runtime=str(runtime),descriptors=descriptors,
        script_sha256=variance.sha(variance.__file__),driver_sha256=variance.sha(__file__),
        hashes={str(p):variance.sha(p) for p in runtime_inputs+project_inputs+[variance.DATA/'sun_positions_20210726.csv',variance.DATA/'measured_bars.json']},
        prior_results=str(variance.OUTPUT),
        scope='Fix only initial main-axis phyllotaxis phase to local 0 degrees; retain original descriptor deviations, '
              'branch azimuth jitter, per-leaf roll, tiller jitter, twist, wind and all random draws. '
              'A single fixed angle is a sensitivity condition, not an estimate of the real field direction.')
    (OUTPUT/'experiment.json').write_text(json.dumps(config,indent=2))
    print('Prepared 30 matched seeds, random phase versus fixed local 0-degree phase',flush=True)


def run():
    config = json.loads((OUTPUT/'experiment.json').read_text())
    assert config['script_sha256'] == variance.sha(variance.__file__)
    assert config['driver_sha256'] == variance.sha(__file__)
    config['status'] = 'running'
    (OUTPUT/'experiment.json').write_text(json.dumps(config,indent=2))
    try:
        for index, seed in enumerate(config['seeds']):
            for condition in CONDITIONS:
                with (OUTPUT/f's{seed}_{condition}.log').open('w') as log:
                    subprocess.run([str(variance.PYTHON),str(Path(__file__).resolve()),'worker',
                        '--seed',str(seed),'--condition',condition],stdout=log,stderr=subprocess.STDOUT,
                        check=True,timeout=600)
                print(f'Completed {index+1}/30 seed {seed}: {condition}',flush=True)
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
    parser.add_argument('--condition',choices=list(CONDITIONS))
    args = parser.parse_args()
    if args.mode == 'prepare':
        prepare()
    elif args.mode == 'run':
        run()
    else:
        variance.worker(args.seed,args.condition,OUTPUT)
