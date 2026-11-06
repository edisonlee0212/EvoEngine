"""Verify and summarize matched initial-azimuth width and center experiments."""
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from diagnose_tau_azimuth_ranges import OUTPUT, PRIOR, ARCS, CONTROL_SEEDS, check_control
from diagnose_tau_variance import sha


def plot(data, order, labels, name, title):
    fig, axes = plt.subplots(1,2,figsize=(13,5),sharey=True)
    for ax, genotype in zip(axes,('BTX','Pawaga')):
        for i,condition in enumerate(order):
            values = data[data.condition == condition].sort_values('seed')[genotype+'_tau'].to_numpy()
            color = '#d78838' if condition.startswith('c90') else '#3675ad'
            ax.scatter(i+np.linspace(-.12,.12,len(values)),values,color=color,s=18,alpha=.6,
                       label='Field seeds' if i == 0 else None,zorder=2)
            median = np.median(values)
            ax.errorbar(i,median,yerr=[[median-values.min()],[values.max()-median]],fmt='D',
                        color='black',capsize=7,ms=5,lw=1.6,zorder=3,
                        label='Median and full range' if i == 0 else None)
            ax.scatter(i,values.mean(),facecolors='white',edgecolors='#c73333',marker='o',s=43,
                       zorder=4,label='Mean' if i == 0 else None)
        ax.set_xticks(range(len(order)),labels,fontsize=10)
        ax.set_xlim(-.5,len(order)-.5)
        ax.set_ylim(0,1)
        ax.set_title('BTx623' if genotype == 'BTX' else genotype)
        ax.grid(axis='y',alpha=.2)
    axes[0].set_ylabel('Transmittance (tau)')
    fig.suptitle(title+'\nJuly 26, 12:30 | 30 matched seeds | No panicles',fontsize=12)
    fig.legend(*axes[0].get_legend_handles_labels(),loc='lower center',ncol=3,frameon=False)
    fig.tight_layout(rect=[0,.08,1,.96])
    for suffix in ('png','pdf'):
        fig.savefig(OUTPUT/f'{name}.{suffix}',dpi=180)
    plt.close(fig)


def main():
    config = json.loads((OUTPUT/'experiment.json').read_text())
    prior = json.loads((PRIOR/'experiment.json').read_text())
    assert config['status'] == 'complete' and config['completed_jobs'] == 156
    assert sha(OUTPUT/'worker_used.py') == config['script_sha256']
    assert sha(OUTPUT/'driver_used.py') == config['driver_sha256']
    rows,phases = [],[]
    probe_rows = measurements = 0
    maximum_phase_error = 0
    for seed in config['seeds']:
        reference = json.loads((PRIOR/'runs'/f's{seed}'/'original/plants_original.json').read_text())
        reference_phases = json.loads((PRIOR/'runs'/f's{seed}'/'original/initial_leaf_azimuths.json').read_text())
        for condition,(start,width) in ARCS.items():
            reused = condition in ('original','fixed_phase')
            root = (PRIOR if reused else OUTPUT)/'runs'/f's{seed}'/condition
            run = json.loads((root/'run.json').read_text())
            assert run['status'] == 'complete' and run['seed'] == seed
            assert run['script_sha256'] == (prior if reused else config)['script_sha256']
            state = json.loads((root/f'plants_{condition}.json').read_text())
            assert [(p['name'],p['seed'],p['position'],p['leaf_count']) for p in state] == [
                (p['name'],p['seed'],p['position'],p['leaf_count']) for p in reference]
            actual = json.loads((root/'initial_leaf_azimuths.json').read_text())
            assert set(actual) == set(reference_phases) and len(actual) == 204
            for plant,value in actual.items():
                error = abs((value-(start+reference_phases[plant]/360*width)+180)%360-180)
                assert error < .0001
                maximum_phase_error = max(maximum_phase_error,error)
                if width == 0:
                    assert value == start
                phases.append(dict(seed=seed,condition=condition,plant=plant,phase=value))
            data = pd.read_csv(root/'measurements.csv')
            probes = pd.read_csv(root/'probes.csv')
            assert len(data) == (2 if condition == 'original' else 1)
            assert len(probes) == 300*len(data) and np.isfinite(probes.scalar).all() and (probes.scalar >= 0).all()
            means = probes.groupby(['samples','genotype','level']).scalar.agg(['mean','count'])
            assert (means['count'] == 50).all()
            for row in data.itertuples():
                assert row.seed == seed and row.condition == condition and row.ray_seed == 18 and row.sd_scale == 1
                for genotype in ('BTX','Pawaga'):
                    top = means.loc[(row.samples,genotype,'top'),'mean']
                    bottom = means.loc[(row.samples,genotype,'bottom'),'mean']
                    assert np.isclose(top,getattr(row,genotype+'_top'),rtol=0,atol=1e-12)
                    assert np.isclose(bottom,getattr(row,genotype+'_bottom'),rtol=0,atol=1e-12)
                    assert np.isclose(bottom/top,getattr(row,genotype+'_tau'),rtol=0,atol=1e-12)
            primary = data[data.samples == 2048].copy()
            assert len(primary) == 1
            primary['source'] = str(root)
            primary['reused'] = reused
            rows.append(primary)
            probe_rows += len(probes)
            measurements += len(data)
    for seed in CONTROL_SEEDS:
        for condition in ('original','fixed_phase'):
            check_control(seed,condition)
    data = pd.concat(rows,ignore_index=True)
    assert len(data) == 210 and probe_rows == 72000 and measurements == 240
    summary = []
    for genotype in ('BTX','Pawaga'):
        baseline = data[data.condition == 'original'][genotype+'_tau']
        for condition,(start,width) in ARCS.items():
            group = data[data.condition == condition]
            x,top = group[genotype+'_tau'],group[genotype+'_top']
            summary.append(dict(genotype=genotype,condition=condition,n=len(x),width=width,
                center=None if width == 360 else (start+width/2)%360,
                mean=x.mean(),median=x.median(),std=x.std(),minimum=x.min(),maximum=x.max(),
                range_width=x.max()-x.min(),tau_cv=x.std()/x.mean(),top_cv=top.std()/top.mean(),
                range_reduction_pct=100*(1-(x.max()-x.min())/(baseline.max()-baseline.min()))))
    summary = pd.DataFrame(summary)
    data.to_csv(OUTPUT/'all_measurements.csv',index=False)
    summary.to_csv(OUTPUT/'range_summary.csv',index=False)
    pd.DataFrame(phases).to_csv(OUTPUT/'initial_leaf_azimuths.csv',index=False)
    plot(data,['original','c0_w180','c0_w90','c0_w30','fixed_phase'],
         ['Full circle\n360 deg','0 +/- 90 deg\n180 deg wide','0 +/- 45 deg\n90 deg wide',
          '0 +/- 15 deg\n30 deg wide','Fixed 0 deg\n0 deg wide'],
         'azimuth_width_comparison','Initial azimuth range: narrowing around 0 degrees')
    plot(data,['original','fixed_phase','c90_w0','c0_w90','c90_w90'],
         ['Full circle\nreference','Fixed 0 deg','Fixed 90 deg','0 +/- 45 deg','90 +/- 45 deg'],
         'azimuth_center_comparison','Initial azimuth center: 0 versus 90 degrees')
    for path,expected in config['hashes'].items():
        assert sha(path) == expected,path
    validation = dict(status='verified',primary_configurations=210,new_configurations=150,
        reused_configurations=60,runtime_controls=6,analyzed_probe_rows=probe_rows,
        analyzed_measurements=measurements,exact_runtime_controls=True,
        verified_phase_records=len(phases),max_phase_error_degrees=maximum_phase_error,
        plant_names_seeds_positions_leaf_counts_identical=True,runtime_project_and_reused_inputs_unchanged=True,
        timing_note='New worker elapsed_seconds is invalid: arc start shadowed the timer. '
                    'Use filesystem timestamps; numerical inputs and measurements are unaffected. '
                    'Executed worker/driver snapshots are preserved with verified hashes.',
        analysis_sha256=sha(__file__))
    (OUTPUT/'validation.json').write_text(json.dumps(validation,indent=2))
    print(summary.to_string(index=False))
    print(json.dumps(validation,indent=2))


if __name__ == '__main__':
    main()
