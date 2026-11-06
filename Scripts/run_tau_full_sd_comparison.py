"""Full July comparison at the baseline's 270 seeds and 25 times per day."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from pathlib import Path
import shutil
import subprocess
import time

import pandas as pd

from diagnose_tau_variance import ROOT, DATA, PYTHON, sha
from run_tau_ensemble import load_run

OUTPUT = ROOT/'out/tau_current/july_full_sd_comparison'
BASE = ROOT/'out/tau_current/tau_1000_campaign/july_september_no_panicle'
FINAL = BASE.parent/'final_results'
PILOT = ROOT/'out/tau_current/july1230_variance_diagnostic'
WORKER = ROOT/'Scripts/run_tau_diurnal.py'
DATE = '2021-07-26'
SCALES = {'original':1.0,'half_sd':.5,'quarter_sd':.25}
SEEDS = list(range(7101,7371))


def save(config):
    temporary = OUTPUT/'experiment.tmp'
    temporary.write_text(json.dumps(config,indent=2),encoding='utf-8')
    temporary.replace(OUTPUT/'experiment.json')


def prepare():
    assert not OUTPUT.exists()
    baseline = json.loads((BASE/'ensemble.json').read_text())
    assert baseline['status'] == 'complete' and baseline['completed_pairs'] == 270
    pilot = json.loads((PILOT/'experiment.json').read_text())
    assert pilot['status'] == 'complete'
    for path,expected in pilot['project_hashes'].items():
        assert sha(path) == expected,path
    times = pd.read_csv(DATA/'sun_positions_20210726.csv').query('valid').time_mst.tolist()
    assert len(times) == 25 and times[0] == '08:15' and times[-1] == '14:15'
    prior_frames = [load_run(BASE/'runs'/DATE/f's{s}',DATE,s,times) for s in SEEDS]
    prior = pd.concat(prior_frames,ignore_index=True)
    saved = pd.read_csv(BASE/'simulation_seed_values.csv').query('date == @DATE')
    keys = ['date','seed','time_mst','genotype']
    pd.testing.assert_frame_equal(prior.sort_values(keys).reset_index(drop=True),
                                  saved[prior.columns].sort_values(keys).reset_index(drop=True))
    OUTPUT.mkdir()
    project = OUTPUT/'project'/Path(pilot['project']).name
    shutil.copytree(Path(pilot['project']).parent,project.parent)
    runtime = OUTPUT/'runtime'
    shutil.copytree(ROOT/'out/install/vs2026-x64/python',runtime)
    shutil.copy2(WORKER,OUTPUT/'worker_used.py')
    shutil.copy2(__file__,OUTPUT/'driver_used.py')
    prior.to_csv(OUTPUT/'baseline_seed_values.csv',index=False)
    descriptors = pilot['descriptors']
    inputs = [p for p in project.parent.rglob('*') if p.is_file()]
    inputs += [p for p in runtime.rglob('*') if p.suffix in ('.pyd','.dll','.ptx')]
    inputs += [p for p in (BASE/'runs'/DATE).rglob('*') if p.suffix in ('.csv','.json')]
    inputs += [WORKER,Path(__file__),ROOT/'Scripts/fit_tau_parbar_supports.py',
               DATA/'sun_positions_20210726.csv',DATA/'measured_bars.json',
               BASE/'ensemble.json',BASE/'simulation_seed_values.csv',
               FINAL/'july26_no_panicles_field_minutes.csv',FINAL/'july26_no_panicles_quantiles.csv',
               FINAL/'selection.json']
    config = dict(status='prepared',date=DATE,seeds=SEEDS,times=times,scales=SCALES,
        project=str(project),runtime=str(runtime),descriptors=descriptors,samples=128,bounces=4,
        sun_csv=str(DATA/'sun_positions_20210726.csv'),ray_seeds=list(range(1,26)),
        panicles=False,baseline=str(BASE),prior_noon_pilot=str(PILOT),early_stopping=False,
        comparison='Same 270 seeds and 25 valid solar times as the archived July trend. '
                   'All 270 days per reduced-SD condition; reuse verified baseline. '
                   'Keep original real/sim valid-window intersection and min-max bands.',
        pairing_limit='Same plant assignments and field seeds. Rounded topology can change '
                      'subsequent random draws, so per-plant azimuth is not locked across SD scales.',
        hashes={str(p):sha(p) for p in inputs},worker_sha256=sha(WORKER),driver_sha256=sha(__file__),
        completed_days=0)
    save(config)
    print('Prepared 540 new full July days; 270 baseline days verified and reused.',flush=True)


def execute(config, condition, seed, controls=False):
    root = OUTPUT/('controls' if controls else 'runs')/condition/f's{seed}'
    if (root/'run.json').exists():
        load_run(root,DATE,seed,config['times'])
        return root
    root.parent.mkdir(parents=True,exist_ok=True)
    desc = config['descriptors'][condition]
    command = [str(PYTHON),str(WORKER),'--module-dir',config['runtime'],'--project',config['project'],
               '--sun-csv',config['sun_csv'],'--seed',str(seed),'--samples','128','--bounces','4',
               '--btx-descriptor',Path(desc['BTX']).name,'--pawaga-descriptor',Path(desc['Pawaga']).name,
               '--output',str(root)]
    with root.with_suffix('.log').open('w') as log:
        subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=600)
    load_run(root,DATE,seed,config['times'])
    old = BASE/'runs'/DATE/f's{seed}'
    plants = json.loads((root/'plants.json').read_text())
    original = json.loads((old/'plants.json').read_text())
    assert [(p['name'],p['seed'],p['position']) for p in plants] == [
        (p['name'],p['seed'],p['position']) for p in original]
    if condition == 'original':
        assert plants == original
        for name in ('tau.csv','probes.csv'):
            pd.testing.assert_frame_equal(pd.read_csv(root/name),pd.read_csv(old/name),check_exact=True)
    return root


def controls(config):
    begin = time.monotonic()
    execute(config,'original',7101,True)
    serial = time.monotonic()-begin
    print(f'Exact full-day baseline passed: 7101; {serial:.1f}s',flush=True)
    begin = time.monotonic()
    with ThreadPoolExecutor(max_workers=2) as pool:
        jobs = {pool.submit(execute,config,'original',s,True):s for s in (7235,7370)}
        for result in as_completed(jobs):
            result.result()
            print(f'Exact concurrent full-day baseline passed: {jobs[result]}',flush=True)
    parallel = time.monotonic()-begin
    config['controls'] = dict(status='verified',seeds=[7101,7235,7370],
                              serial_wall_seconds=serial,two_concurrent_wall_seconds=parallel)
    config['workers'] = 2 if parallel < 1.8*serial else 1
    config['status'] = 'controls_verified'
    save(config)
    print(f'Controls verified. Using {config["workers"]} worker(s).',flush=True)


def run(config):
    assert config['controls']['status'] == 'verified'
    for path,expected in config['hashes'].items():
        assert sha(path) == expected,path
    config['status'] = 'running'
    save(config)
    begin = time.monotonic()
    jobs = [(c,s) for s in SEEDS for c in ('half_sd','quarter_sd')]
    completed = 0
    try:
        with ThreadPoolExecutor(max_workers=config['workers']) as pool:
            pending = {pool.submit(execute,config,c,s):(c,s) for c,s in jobs}
            try:
                for result in as_completed(pending):
                    result.result()
                    completed += 1
                    condition,seed = pending[result]
                    config.update(completed_days=completed,elapsed_seconds=time.monotonic()-begin,
                                  last_completed=dict(condition=condition,seed=seed))
                    save(config)
                    print(f'Completed {completed}/540 full days: {condition} seed {seed}',flush=True)
            except BaseException:
                for result in pending:
                    result.cancel()
                raise
        for path,expected in config['hashes'].items():
            assert sha(path) == expected,path
        config['status'] = 'complete'
    except BaseException as error:
        config.update(status='failed',error=str(error))
        raise
    finally:
        save(config)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['prepare','controls','run'])
    args = parser.parse_args()
    if args.mode == 'prepare':
        prepare()
    else:
        config = json.loads((OUTPUT/'experiment.json').read_text())
        assert sha(WORKER) == config['worker_sha256'] and sha(__file__) == config['driver_sha256']
        controls(config) if args.mode == 'controls' else run(config)
