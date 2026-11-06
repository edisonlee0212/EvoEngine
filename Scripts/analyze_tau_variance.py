"""Validate and plot the matched descriptor-dispersion experiment."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from diagnose_tau_variance import OUTPUT, SCALES, sha


def parameter_audit():
    audit = json.loads((OUTPUT/'descriptor_audit.json').read_text())
    profiles = []
    for genotype, distributions in audit.items():
        for name, value in distributions.items():
            deviation = value['deviation']
            if isinstance(deviation, dict):
                if deviation['min_value'] == deviation['max_value'] == 0:
                    continue
                ts = np.linspace(0,1,101)
                evaluated = []
                for part in ('mean','deviation'):
                    plot = value[part]
                    assert not plot['curve']['tangent_']
                    points = np.array(plot['curve']['values_'])
                    curve = np.clip(np.interp(ts,points[:,0],points[:,1]),0,1)
                    evaluated.append(plot['min_value']+(plot['max_value']-plot['min_value'])*curve)
                for t, mean, sd in zip(ts,*evaluated):
                    profiles.append(dict(genotype=genotype,parameter=name,rank_fraction=t,mean=mean,sd=sd))
            elif deviation:
                profiles.append(dict(genotype=genotype,parameter=name,rank_fraction=np.nan,
                                     mean=value['mean'],sd=deviation))
    profiles = pd.DataFrame(profiles)
    profiles.to_csv(OUTPUT/'parameter_profiles.csv',index=False)
    result = profiles.groupby(['genotype','parameter']).agg(
        mean_min=('mean','min'),mean_max=('mean','max'),sd_min=('sd','min'),sd_max=('sd','max'))
    result.to_csv(OUTPUT/'parameter_summary.csv')
    return result


def main():
    parameter_audit()
    config = json.loads((OUTPUT/'experiment.json').read_text())
    assert config['status'] == 'complete'
    frames, plants, baseline_errors = [], [], []
    for seed in config['seeds']:
        root = OUTPUT/'runs'/f's{seed}'
        for label in SCALES:
            run = json.loads((root/label/'run.json').read_text())
            assert run['status'] == 'complete'
            assert run['seed'] == seed and run['condition'] == label
            assert run['script_sha256'] == config['script_sha256']
            baseline_errors.extend(run.get('baseline_error',{}).values())
        data = pd.concat([pd.read_csv(root/label/'measurements.csv') for label in SCALES],ignore_index=True)
        probes = pd.concat([pd.read_csv(root/label/'probes.csv') for label in SCALES],ignore_index=True)
        assert len(data) == 4 and len(probes) == 1200
        assert np.isfinite(probes['scalar']).all() and (probes['scalar'] >= 0).all()
        means = probes.groupby(['condition','samples','genotype','level'])['scalar'].agg(['mean','count'])
        assert (means['count'] == 50).all()
        for row in data.itertuples():
            assert row.seed == seed and row.ray_seed == 18
            for genotype in ('BTX','Pawaga'):
                a = means.loc[(row.condition,row.samples,genotype,'top'),'mean']
                b = means.loc[(row.condition,row.samples,genotype,'bottom'),'mean']
                assert np.isclose(b/a,getattr(row,genotype+'_tau'),rtol=0,atol=1e-12)
        primary = data[data.samples == 2048]
        assert set(primary.condition) == set(SCALES) and len(primary) == 3
        frames.append(primary)
        for label in SCALES:
            stand = json.loads((root/label/f'plants_{label}.json').read_text())
            assert len(stand) == 204
            plants.extend(dict(field_seed=seed,condition=label,**p) for p in stand)
    data = pd.concat(frames,ignore_index=True)
    data.to_csv(OUTPUT/'all_measurements.csv',index=False)
    summaries = []
    for label, scale in SCALES.items():
        subset = data[data.condition == label]
        for genotype in ('BTX','Pawaga'):
            x = subset[genotype+'_tau']
            summaries.append(dict(condition=label,sd_scale=scale,input_variance_scale=scale**2,
                genotype=genotype,n=len(x),mean=x.mean(),median=x.median(),std=x.std(),
                minimum=x.min(),maximum=x.max(),range_width=x.max()-x.min(),
                q025=x.quantile(.025),q975=x.quantile(.975),q95_width=x.quantile(.975)-x.quantile(.025),
                top_mean=subset[genotype+'_top'].mean(),
                top_cv=subset[genotype+'_top'].std()/subset[genotype+'_top'].mean(),
                bottom_cv_across_stands=subset[genotype+'_bottom'].std()/subset[genotype+'_bottom'].mean()))
    summary = pd.DataFrame(summaries)
    for genotype in ('BTX','Pawaga'):
        baseline = summary[(summary.genotype == genotype)&(summary.condition == 'original')].iloc[0]
        for metric in ('range_width','std','q95_width'):
            mask = summary.genotype == genotype
            summary.loc[mask,metric+'_reduction_pct'] = 100*(1-summary.loc[mask,metric]/baseline[metric])
    summary.to_csv(OUTPUT/'range_summary.csv',index=False)
    plant_df = pd.DataFrame(plants)
    plant_df.groupby(['condition','genotype']).agg(
        n=('leaf_count','size'),leaf_count_mean=('leaf_count','mean'),leaf_count_std=('leaf_count','std'),
        leaf_count_min=('leaf_count','min'),leaf_count_max=('leaf_count','max')).to_csv(OUTPUT/'plant_leaf_count_summary.csv')
    fig, axes = plt.subplots(1,2,figsize=(12,4.7),sharey=True)
    colors = ['#3675ad','#dc8833','#49906a']
    for ax, genotype in zip(axes,('BTX','Pawaga')):
        paired = data.pivot(index='seed',columns='condition',values=genotype+'_tau').reindex(columns=list(SCALES))
        for values in paired.to_numpy():
            ax.plot(range(3),values,color='#bbbbbb',alpha=.45,lw=.65,zorder=1)
        for i,(label,color) in enumerate(zip(SCALES,colors)):
            values = paired[label].to_numpy()
            ax.scatter(np.full(len(values),i),values,s=18,color=color,alpha=.65,zorder=3,
                       label='Field seeds' if i == 0 else None)
            median = np.median(values)
            ax.errorbar(i,median,yerr=np.array([[median-values.min()],[values.max()-median]]),
                        fmt='D',ms=6,color='black',capsize=8,lw=1.8,zorder=4,
                        label='Median and full range' if i == 0 else None)
        ax.set_xticks(range(3),['Original','SD x 0.5','SD x 0.25'])
        ax.set_title('BTx623' if genotype == 'BTX' else genotype)
        ax.set_xlabel('Descriptor parameter dispersion')
        ax.grid(axis='y',alpha=.2)
        ax.set_xlim(-.35,2.35)
    axes[0].set_ylabel('Transmittance (tau)')
    axes[1].legend(fontsize=8,loc='upper left')
    fig.suptitle('July 26, 12:30 | No panicles')
    fig.tight_layout()
    fig.savefig(OUTPUT/'variance_comparison.png',dpi=180)
    fig.savefig(OUTPUT/'variance_comparison.pdf')
    for path, expected in config['archive_hashes'].items():
        assert sha(path) == expected,path
    for path, expected in config['project_hashes'].items():
        assert sha(path) == expected,path
    validation = dict(status='verified',field_seeds=len(config['seeds']),primary_measurements=len(data),
        total_measurements=4*len(config['seeds']),probe_rows=1200*len(config['seeds']),
        max_archived_baseline_error=max(abs(x) for x in baseline_errors),
        probe_ratios_recomputed=True,original_inputs_unchanged=True,experiment_inputs_unchanged=True,
        analysis_sha256=sha(__file__))
    (OUTPUT/'validation.json').write_text(json.dumps(validation,indent=2))
    print(summary.to_string(index=False))
    print(json.dumps(validation,indent=2))


if __name__ == '__main__':
    main()
