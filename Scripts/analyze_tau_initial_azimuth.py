"""Verify fixed-initial-phase controls and compare full noon tau ranges."""
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from diagnose_tau_initial_azimuth import OUTPUT, CONDITIONS
from diagnose_tau_variance import sha


def main():
    config = json.loads((OUTPUT/'experiment.json').read_text())
    assert config['status'] == 'complete'
    all_rows, baseline_errors, phases = [], [], []
    for seed in config['seeds']:
        stands = {}
        for condition, expected in CONDITIONS.items():
            root = OUTPUT/'runs'/f's{seed}'/condition
            run = json.loads((root/'run.json').read_text())
            assert run['status'] == 'complete' and run['seed'] == seed
            assert run['script_sha256'] == config['script_sha256']
            baseline_errors.extend(run.get('baseline_error',{}).values())
            data, probes = pd.read_csv(root/'measurements.csv'), pd.read_csv(root/'probes.csv')
            assert len(data) == (2 if condition == 'original' else 1)
            assert len(probes) == 300*len(data) and np.isfinite(probes.scalar).all()
            assert (probes.scalar >= 0).all()
            means = probes.groupby(['samples','genotype','level']).scalar.agg(['mean','count'])
            assert (means['count'] == 50).all()
            for row in data.itertuples():
                assert row.ray_seed == 18 and row.sd_scale == 1 and row.condition == condition
                for genotype in ('BTX','Pawaga'):
                    top = means.loc[(row.samples,genotype,'top'),'mean']
                    bottom = means.loc[(row.samples,genotype,'bottom'),'mean']
                    assert np.isclose(bottom/top,getattr(row,genotype+'_tau'),rtol=0,atol=1e-12)
            all_rows.append(data[data.samples == 2048])
            stands[condition] = json.loads((root/f'plants_{condition}.json').read_text())
            azimuths = json.loads((root/'initial_leaf_azimuths.json').read_text())
            assert len(azimuths) == 204
            if expected is not None:
                assert all(value == expected for value in azimuths.values())
            phases.extend(dict(seed=seed,condition=condition,plant=name,phase=value) for name,value in azimuths.items())
        assert [(p['name'],p['seed'],p['position'],p['leaf_count']) for p in stands['original']] == [
            (p['name'],p['seed'],p['position'],p['leaf_count']) for p in stands['fixed_phase']]
    data = pd.concat(all_rows,ignore_index=True)
    assert len(data) == 60
    data.to_csv(OUTPUT/'all_measurements.csv',index=False)
    pd.DataFrame(phases).to_csv(OUTPUT/'initial_leaf_azimuths.csv',index=False)
    summary = []
    for genotype in ('BTX','Pawaga'):
        original = data[data.condition == 'original'][genotype+'_tau']
        for condition in CONDITIONS:
            x = data[data.condition == condition][genotype+'_tau']
            top = data[data.condition == condition][genotype+'_top']
            summary.append(dict(genotype=genotype,condition=condition,n=len(x),mean=x.mean(),median=x.median(),
                std=x.std(),minimum=x.min(),maximum=x.max(),range_width=x.max()-x.min(),
                q025=x.quantile(.025),q975=x.quantile(.975),top_cv=top.std()/top.mean(),
                range_reduction_pct=100*(1-(x.max()-x.min())/(original.max()-original.min())),
                std_reduction_pct=100*(1-x.std()/original.std())))
    summary = pd.DataFrame(summary)
    summary.to_csv(OUTPUT/'range_summary.csv',index=False)
    previous = pd.read_csv(OUTPUT.parent/'july1230_variance_diagnostic/all_measurements.csv')
    previous = previous[previous.condition == 'original'].set_index('seed').sort_index()
    current = data[data.condition == 'original'].set_index('seed').sort_index()
    baseline_2048_error = max(float((current[g+'_tau']-previous[g+'_tau']).abs().max()) for g in ('BTX','Pawaga'))
    assert baseline_2048_error < 1e-7
    fig, axes = plt.subplots(1,2,figsize=(11,4.7),sharey=True)
    for ax, genotype in zip(axes,('BTX','Pawaga')):
        pairs = data.pivot(index='seed',columns='condition',values=genotype+'_tau').reindex(columns=list(CONDITIONS))
        for values in pairs.to_numpy():
            ax.plot([0,1],values,color='#b7b7b7',alpha=.45,lw=.7,zorder=1)
        for i,(condition,color) in enumerate(zip(CONDITIONS,['#3675ad','#49906a'])):
            values = pairs[condition].to_numpy()
            ax.scatter(np.full(len(values),i),values,color=color,s=20,alpha=.7,zorder=3,
                       label='Field seeds' if i == 0 else None)
            median = np.median(values)
            ax.errorbar(i,median,yerr=np.array([[median-values.min()],[values.max()-median]]),fmt='D',
                        ms=6,color='black',capsize=8,lw=1.8,zorder=4,label='Median and full range' if i == 0 else None)
        ax.set_title('BTx623' if genotype == 'BTX' else genotype)
        ax.set_xticks([0,1],['Random initial phase','Fixed initial phase (0 deg)'])
        ax.set_xlim(-.3,1.3)
        ax.grid(axis='y',alpha=.2)
    axes[0].set_ylabel('Transmittance (tau)')
    fig.suptitle('July 26, 12:30 | Original trait distributions | No panicles')
    fig.legend(*axes[0].get_legend_handles_labels(),loc='lower center',ncol=2,fontsize=9,frameon=False)
    fig.tight_layout(rect=[0,.08,1,.98])
    fig.savefig(OUTPUT/'initial_phase_comparison.png',dpi=180)
    fig.savefig(OUTPUT/'initial_phase_comparison.pdf')
    for path, expected in config['hashes'].items():
        assert sha(path) == expected,path
    validation = dict(status='verified',field_seeds=30,primary_measurements=60,total_measurements=90,probe_rows=27000,
        fixed_phase_plants_verified=30*204,plant_assignments_and_leaf_counts_identical=True,
        max_baseline_128_error=max(abs(v) for v in baseline_errors),max_baseline_2048_error=baseline_2048_error,
        runtime_and_project_hashes_unchanged=True,analysis_sha256=sha(__file__))
    (OUTPUT/'validation.json').write_text(json.dumps(validation,indent=2))
    print(summary.to_string(index=False))
    print(json.dumps(validation,indent=2))


if __name__ == '__main__':
    main()
