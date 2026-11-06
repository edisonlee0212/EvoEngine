"""Run paired July/September stands and stop at the agreed pointwise band stability."""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
DATES = ('2021-07-26', '2021-09-21')
QUANTILES = ('q025', 'median', 'q975')
DESCRIPTORS = ('BTX623_20210726_v6_no_panicle.sorghumls', 'Pawaga_20210726_v8_no_panicle.sorghumls')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_run_configuration(folder, project, descriptors, hashes):
    run = json.loads((folder/'run.json').read_text())
    expected = dict(project=str(project), samples=128, bounces=4, leaf_vsub=.02,
                    leaf_hsub=4, push_normal_distance=.001,
                    descriptors=dict(zip(('BTX', 'Pawaga'),
                        [f'ManualAssets/Descriptors/TauSeasonal/{d}' for d in descriptors])))
    if any(run.get(k) != v for k, v in expected.items()):
        raise ValueError(f'Run configuration differs: {folder}')
    if run['hashes'] != {p: hashes.get(p) for p in run['hashes']}:
        raise ValueError(f'Run input/runtime hashes differ: {folder}')


def write_manifest(output, manifest):
    temporary = output/'ensemble.tmp'
    temporary.write_text(json.dumps(manifest, indent=2))
    temporary.replace(output/'ensemble.json')


def summarize(sim):
    if sim.duplicated(['date', 'genotype', 'time_mst', 'seed']).any() or not np.isfinite(sim.tau).all():
        raise ValueError('Duplicate or nonfinite simulation values')
    return sim.groupby(['date', 'genotype', 'time_mst']).tau.agg(
        n='size', q025=lambda x: x.quantile(.025), median='median',
        q975=lambda x: x.quantile(.975)).reset_index()


def stability(previous, current, tolerance):
    keys = ['date', 'genotype', 'time_mst']
    a, b = (x.set_index(keys).sort_index() for x in (previous, current))
    if not a.index.equals(b.index):
        raise ValueError('Checkpoint time grids differ')
    change = (b[list(QUANTILES)] - a[list(QUANTILES)]).abs()
    flat = change.stack()
    worst = flat.idxmax()
    return dict(max_change=float(flat.max()), passed=bool((change < tolerance).all().all()),
                worst=dict(zip([*keys, 'statistic'], worst)),
                changes={key: float(change[key].max()) for key in QUANTILES})


def load_run(folder, date, seed, expected_times):
    manifest = json.loads((folder/'run.json').read_text())
    if manifest['status'] != 'complete' or manifest['seed'] != seed or manifest['plant_count'] != 204:
        raise ValueError(f'Incomplete or mismatched run: {folder}')
    data = pd.read_csv(folder/'tau.csv', dtype={'time_mst': str})
    if data.time_mst.tolist() != expected_times or not data.valid.all():
        raise ValueError(f'Unexpected solar time grid: {folder}')
    for genotype in ('BTX', 'Pawaga'):
        if not np.allclose(data[f'{genotype}_tau'], data[f'{genotype}_bottom']/data[f'{genotype}_top']):
            raise ValueError(f'Incorrect tau: {folder}')
    probes = pd.read_csv(folder/'probes.csv')
    if len(probes) != len(data)*300 or not np.isfinite(probes.scalar).all():
        raise ValueError(f'Incomplete probe results: {folder}')
    data['date'], data['seed'] = date, seed
    return data.melt(id_vars=['date', 'seed', 'time_mst'], value_vars=['BTX_tau', 'Pawaga_tau'],
                     var_name='genotype', value_name='tau').assign(
                         genotype=lambda x: x.genotype.map({'BTX_tau': 'BTx623', 'Pawaga_tau': 'Pawaga'}))


def plot_results(output, band, field_path, count, panicles=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.dates as mdates
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    dates = sorted(band.date.unique())
    field = pd.read_csv(field_path, parse_dates=['minute'])
    field = field.loc[field.date.isin(dates) & field.valid].copy()
    field.to_csv(output/'field_minutes_used.csv', index=False)
    band = band.copy()
    band['minute'] = pd.to_datetime(band.date + ' ' + band.time_mst)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(2, len(dates), figsize=(6.5*len(dates), 8), sharey=True, squeeze=False)
    upper = max(.75, float(field.tau.max())*1.06, float(band.q975.max())*1.06)
    for row, (genotype, color) in enumerate([('BTx623', '#2777b5'), ('Pawaga', '#d98124')]):
        for col, date in enumerate(dates):
            ax = axes[row, col]
            f = field.loc[(field.date == date) & (field.genotype == genotype)].sort_values('minute')
            b = band.loc[(band.date == date) & (band.genotype == genotype)].sort_values('minute')
            ax.fill_between(b.minute, b.q025, b.q975, color=color, alpha=.22, linewidth=0)
            ax.plot(b.minute, b['median'], color=color, linewidth=2, marker='o', markersize=3)
            curve = f.set_index('minute').tau.reindex(pd.date_range(f.minute.min(), f.minute.max(), freq='min'))
            ax.plot(curve.index, curve.values, color='#242424', linewidth=1.1, marker='.', markersize=2)
            ax.set_title(f'{date} | {genotype}', loc='left', fontweight='bold')
            ax.text(.02, .97, f'Field: {len(f)} valid minutes\nSimulation: {len(b)} times x {count} seeds',
                    transform=ax.transAxes, va='top', fontsize=9, color='#555555')
            ax.xaxis.set_major_locator(mdates.HourLocator())
            ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
            ax.set_xlim(pd.Timestamp(date+' 08:00'), pd.Timestamp(date+' 16:00'))
            ax.set_ylim(0, upper)
            ax.grid(axis='y', alpha=.17)
            if col == 0:
                ax.set_ylabel('Canopy transmittance, τ (below / above)')
            if row == 1:
                ax.set_xlabel('Local time (MST, UTC−7)')
    fig.suptitle('Diurnal comparison | '+('with panicles' if panicles else 'no panicles'), fontsize=18, y=.98)
    fig.legend(handles=[Line2D([], [], color='#242424', label='Field: valid 1-minute values'),
                        Line2D([], [], color='#2777b5', label='Simulation: median across seeds'),
                        Patch(facecolor='#2777b5', alpha=.22, label='Simulation: pointwise 2.5–97.5% quantiles')],
               loc='upper center', bbox_to_anchor=(.5, .945), ncol=3 if len(dates)>1 else 1, frameon=False, fontsize=9)
    fig.text(.5, .025, 'Fixed base descriptors; simulation every 15 minutes. Lines/shading connect sampled times.\n'
             'Bands show variation across generated stands, not confidence intervals of the mean.', ha='center', fontsize=9)
    fig.subplots_adjust(left=.08, right=.985, top=.86, bottom=.12, hspace=.3, wspace=.12)
    for extension in ('png', 'pdf', 'svg'):
        fig.savefig(output/f'diurnal_comparison.{extension}', dpi=180)
    plt.close(fig)
    history = output/'convergence_history.csv'
    if history.exists() and history.stat().st_size > 2:
        checks = pd.read_csv(history)
        fig, ax = plt.subplots(figsize=(8, 4))
        for key in QUANTILES:
            ax.plot(checks.n, checks[key], marker='.', label=key)
        ax.axhline(.01, color='black', linestyle='--', label='Stopping threshold')
        ax.set(xlabel='Number of plant seeds', ylabel='Maximum change in tau',
               title='Change from preceding 10-seed checkpoint')
        ax.legend()
        fig.tight_layout()
        fig.savefig(output/'convergence.png', dpi=180)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--project', type=Path, default=ROOT/'Resources/DigitalAgricultureProject/test_lsystem_sorghum.eveproj')
    parser.add_argument('--field-minutes', type=Path, default=ROOT/'out/tau_fork_repro/diurnal_comparison/field_minutes_screening.csv')
    parser.add_argument('--max-seeds', type=int, default=100)
    parser.add_argument('--dates', nargs='+', choices=DATES, default=list(DATES))
    parser.add_argument('--panicles', action='store_true')
    parser.add_argument('--reuse', type=Path, help='Copy verified complete runs from an earlier ensemble')
    args = parser.parse_args()
    if args.max_seeds < 30 or args.max_seeds % 10 or len(set(args.dates)) != len(args.dates):
        parser.error('Use at least 30 seeds, a multiple of 10, and distinct dates')
    output = args.output.resolve()
    field_path = args.field_minutes.resolve()
    project = args.project.resolve()
    dates = sorted(args.dates)
    descriptors = tuple(d.replace('_no_panicle', '') for d in DESCRIPTORS) if args.panicles else DESCRIPTORS
    reuse = args.reuse.resolve() if args.reuse else None
    inputs = [Path(__file__), ROOT/'Scripts/run_tau_diurnal.py', field_path,
              ROOT/'Scripts/fit_tau_parbar_supports.py', ROOT/'Scripts/tau_diurnal/measured_bars.json', project]
    inputs += [ROOT/f'Scripts/tau_diurnal/sun_positions_{d.replace("-", "")}.csv' for d in dates]
    runtime = ROOT/'out/install/vs2026-x64/python'
    inputs += [runtime/p for p in ('PyDigitalAgriculture.cp312-win_amd64.pyd', 'EvoEngine_SDK.dll',
                                   'Packages/LSystemPackage.dll', 'Packages/DigitalAgriculturePackage.dll')]
    inputs += sorted(p for p in (project.parent/'Assets').rglob('*') if p.is_file())
    hashes = {str(p): digest(p) for p in inputs}
    for name in descriptors:
        path = project.parent/'Assets/ManualAssets/Descriptors/TauSeasonal'/name
        if ('enable_panicle: false' in path.read_text()) == args.panicles:
            raise ValueError(f'Incorrect panicle setting: {path}')
    if reuse:
        old = json.loads((reuse/'ensemble.json').read_text())
        if old['descriptors'] != list(descriptors) or old['seed_start'] != 7101:
            raise ValueError('Reused ensemble has different descriptors or seeds')
        for path, sha in old['hashes'].items():
            if path != str(Path(__file__)) and (path not in hashes or hashes[path] != sha):
                raise ValueError(f'Reused experiment input differs: {path}')
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(status='running', dates=dates, seed_start=7101, max_seeds=args.max_seeds,
                    quantiles=[.025, .5, .975], minimum_seeds=30, check_every=10,
                    tolerance=.01, required_consecutive_passes=3, samples=128, bounces=4,
                    project=str(project), descriptors=descriptors, panicles=args.panicles,
                    reuse_from=str(reuse) if reuse else None, reused_day_runs=0,
                    field_processing='Reuse existing screened minute values unchanged', hashes=hashes,
                    checkpoints=[], commands=[], completed_pairs=0, completed_day_runs=0)
    start = time.monotonic()
    parts, previous, streak = [], None, 0
    write_manifest(output, manifest)
    try:
        for count, seed in enumerate(range(7101, 7101+args.max_seeds), 1):
            for date in dates:
                folder = output/'runs'/date/f's{seed}'
                folder.parent.mkdir(parents=True, exist_ok=True)
                sun = ROOT/f'Scripts/tau_diurnal/sun_positions_{date.replace("-", "")}.csv'
                command = [sys.executable, str(ROOT/'Scripts/run_tau_diurnal.py'), '--seed', str(seed),
                           '--project', str(project), '--sun-csv', str(sun), '--btx-descriptor', descriptors[0],
                           '--pawaga-descriptor', descriptors[1], '--output', str(folder)]
                times = pd.read_csv(sun).query('valid').time_mst.tolist()
                source = reuse/'runs'/date/f's{seed}' if reuse else None
                if source and source.exists():
                    check_run_configuration(source, project, descriptors, hashes)
                    load_run(source, date, seed, times)
                    shutil.copytree(source, folder)
                    if (source.parent/f's{seed}.log').exists():
                        shutil.copy2(source.parent/f's{seed}.log', folder.parent/f's{seed}.log')
                    manifest['reused_day_runs'] += 1
                    print(f'REUSED {count}/{args.max_seeds} {date} seed={seed}', flush=True)
                else:
                    print(f'START {count}/{args.max_seeds} {date} seed={seed}', flush=True)
                    manifest['commands'].append(command)
                    manifest['active_run'] = dict(date=date, seed=seed)
                    write_manifest(output, manifest)
                    with (folder.parent/f's{seed}.log').open('w') as log:
                        subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=1800)
                check_run_configuration(folder, project, descriptors, hashes)
                parts.append(load_run(folder, date, seed, times))
                manifest['completed_day_runs'] += 1
                manifest.pop('active_run', None)
                write_manifest(output, manifest)
            if len(dates) == 2:
                a, b = [json.loads((output/'runs'/d/f's{seed}'/'plants.json').read_text()) for d in dates]
                if a != b:
                    raise ValueError(f'Paired plant states differ: seed {seed}')
            manifest['completed_pairs'] = count
            print(f'COMPLETE seed {count}/{args.max_seeds}, elapsed={time.monotonic()-start:.1f}s', flush=True)
            if count % 10 == 0:
                sim = pd.concat(parts, ignore_index=True)
                band = summarize(sim)
                if not (band.n == count).all():
                    raise ValueError('Unequal seed counts')
                band.to_csv(output/f'quantiles_n{count:03}.csv', index=False)
                if count >= 30:
                    check = dict(n=count, **stability(previous, band, .01))
                    streak = streak+1 if check['passed'] else 0
                    check['consecutive_passes'] = streak
                    manifest['checkpoints'].append(check)
                    print('CHECK '+json.dumps(check), flush=True)
                previous = band
                sim.to_csv(output/'simulation_seed_values.csv', index=False)
                band.to_csv(output/'simulation_time_quantiles.csv', index=False)
                pd.DataFrame([dict(n=c['n'], max_change=c['max_change'], **c['changes'],
                                   passed=c['passed'], consecutive_passes=c['consecutive_passes'])
                              for c in manifest['checkpoints']]).to_csv(output/'convergence_history.csv', index=False)
                plot_results(output, band, field_path, count, args.panicles)
            manifest['elapsed_seconds'] = time.monotonic()-start
            write_manifest(output, manifest)
            if streak >= 3:
                break
        sim.to_csv(output/'simulation_seed_values.csv', index=False)
        band.to_csv(output/'simulation_time_quantiles.csv', index=False)
        if any(digest(p) != sha for p, sha in hashes.items()):
            raise ValueError('An input or runtime file changed during the ensemble')
        plot_results(output, band, field_path, count, args.panicles)
        manifest.update(status='complete', stop_reason='band_stability' if streak >= 3 else 'max_seeds',
                        completed_pairs=count, elapsed_seconds=time.monotonic()-start)
        (output/'validation.json').write_text(json.dumps(dict(
            status='verified', seeds_per_date=count, completed_day_runs=count*len(dates),
            simulation_tau_values=len(sim), time_genotype_groups=len(band),
            raw_probe_rows=len(sim)*150, panicles=args.panicles,
            matching_plant_states_between_dates=True if len(dates)==2 else None,
            stable_runtime_and_input_hashes=True, stop_reason=manifest['stop_reason']), indent=2))
    except BaseException as error:
        manifest.update(status='failed', error=str(error), elapsed_seconds=time.monotonic()-start)
        raise
    finally:
        write_manifest(output, manifest)
    print('FINISHED '+json.dumps({k: manifest[k] for k in ('status', 'stop_reason', 'completed_pairs', 'elapsed_seconds')}), flush=True)


if __name__ == '__main__':
    main()
