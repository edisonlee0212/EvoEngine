"""Verify full July SD ensembles and plot matched min-max diurnal comparisons."""
import json
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np
import pandas as pd

from run_tau_full_sd_comparison import OUTPUT, BASE, FINAL, PILOT, DATE, SCALES, SEEDS, sha

LABELS = {'original':'Original SD','half_sd':'SD x 0.5','quarter_sd':'SD x 0.25'}
COLORS = {'original':'#666666','half_sd':'#317bb4','quarter_sd':'#c87b28'}


def summarize(data, seeds, times):
    keys = ['condition','genotype','time_mst']
    assert not data.duplicated(keys+['seed']).any() and np.isfinite(data.tau).all()
    assert set(data.condition) == set(SCALES)
    expected = {(c,g,t) for c in SCALES for g in ('BTx623','Pawaga') for t in times}
    groups = data.groupby(keys)
    assert set(groups.groups) == expected
    assert all(set(group.seed) == set(seeds) for _,group in groups)
    result = groups.tau.agg(n='size',minimum='min',median='median',maximum='max',mean='mean',std='std').reset_index()
    result['range_width'] = result.maximum-result.minimum
    return result


def panel(ax, band, field, genotype, conditions, upper):
    for condition in conditions:
        b = band[(band.genotype == genotype)&(band.condition == condition)].sort_values('time_mst')
        time = pd.to_datetime(DATE+' '+b.time_mst)
        color = COLORS[condition] if len(conditions)>1 else ('#2777b5' if genotype=='BTx623' else '#d98124')
        ax.fill_between(time,b.minimum,b.maximum,color=color,alpha=.19,linewidth=0)
        ax.plot(time,b['median'],color=color,lw=2,marker='o',ms=2.5)
    f = field[field.genotype == genotype].sort_values('minute')
    ax.plot(f.minute,f.tau,color='#222222',lw=1.2)
    ax.set(xlim=(pd.Timestamp(DATE+' 08:00'),pd.Timestamp(DATE+' 16:00')),
           ylim=(0,upper),xlabel='Local time (MST, UTC-7)')
    ax.xaxis.set_major_locator(mdates.HourLocator())
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    ax.grid(axis='y',alpha=.17)


def figures(band, field, upper):
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,
                         'axes.spines.top':False,'axes.spines.right':False})
    for condition in SCALES:
        fig,axes = plt.subplots(1,2,figsize=(13,5.2),sharey=True)
        for ax,genotype in zip(axes,['BTx623','Pawaga']):
            panel(ax,band,field,genotype,[condition],upper)
            ax.set_title(genotype,loc='left',fontweight='bold',fontsize=13)
        axes[0].set_ylabel('Canopy transmittance, tau (below / above)')
        fig.suptitle(f'{DATE} | No panicles | {LABELS[condition]}',fontsize=17,y=.975)
        fig.legend(handles=[Line2D([],[],color='#222222',label='Field'),
                            Line2D([],[],color='#666666',label='Simulation: median'),
                            Patch(facecolor='#999999',alpha=.22,label='Simulation: min-max (270 seeds)')],
                   loc='upper center',bbox_to_anchor=(.5,.91),ncol=3,frameon=False,fontsize=10)
        fig.subplots_adjust(left=.075,right=.97,top=.76,bottom=.14,wspace=.12)
        for suffix in ('png','pdf','svg'):
            fig.savefig(OUTPUT/f'july26_{condition}.{suffix}',dpi=180)
        plt.close(fig)
    fig,axes = plt.subplots(2,2,figsize=(13,8.5),sharex=True,sharey=True)
    for row,condition in enumerate(('half_sd','quarter_sd')):
        for col,genotype in enumerate(('BTx623','Pawaga')):
            ax = axes[row,col]
            panel(ax,band,field,genotype,['original',condition],upper)
            ax.set_title(f'{genotype} | Original vs {LABELS[condition]}',loc='left',fontsize=12)
            if col == 0:
                ax.set_ylabel('Canopy transmittance, tau')
    fig.suptitle('July 26 | Parameter SD comparison | 270 matched seeds',fontsize=17,y=.985)
    handles = [Line2D([],[],color='#222222',label='Field')]
    handles += [Line2D([],[],color=COLORS[c],lw=2,label=LABELS[c]+': median') for c in SCALES]
    handles += [Patch(facecolor='#999999',alpha=.22,label='Bands: min-max')]
    fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.945),ncol=5,frameon=False,fontsize=9)
    fig.subplots_adjust(left=.075,right=.97,top=.85,bottom=.08,hspace=.32,wspace=.12)
    for suffix in ('png','pdf','svg'):
        fig.savefig(OUTPUT/f'july26_sd_trend_comparison.{suffix}',dpi=180)
    plt.close(fig)


def main():
    config = json.loads((OUTPUT/'experiment.json').read_text())
    assert config['status']=='complete' and config['completed_days']==540
    assert sha(OUTPUT/'worker_used.py')==config['worker_sha256']
    assert sha(OUTPUT/'driver_used.py')==config['driver_sha256']
    frames,plants = [],[]
    probe_rows = 0
    pilot_matches = 0
    for seed in SEEDS:
        reference = json.loads((BASE/'runs'/DATE/f's{seed}'/'plants.json').read_text())
        for condition in SCALES:
            root = BASE/'runs'/DATE/f's{seed}' if condition=='original' else OUTPUT/'runs'/condition/f's{seed}'
            manifest = json.loads((root/'run.json').read_text())
            assert manifest['status']=='complete' and manifest['seed']==seed
            assert manifest['samples']==128 and manifest['bounces']==4 and manifest['plant_count']==204
            assert manifest['valid_times']==25 and manifest['probe_rows']==7500
            assert manifest['descriptors']==config['descriptors'][condition]
            if condition!='original':
                assert manifest['project']==config['project']
                for path,expected in manifest['hashes'].items():
                    assert config['hashes'][path]==expected,path
            stand = json.loads((root/'plants.json').read_text())
            assert [(p['name'],p['seed'],p['position']) for p in stand] == [
                (p['name'],p['seed'],p['position']) for p in reference]
            pilot = PILOT/'runs'/f's{seed}'/condition/f'plants_{condition}.json'
            if pilot.exists():
                assert stand == json.loads(pilot.read_text()),(seed,condition)
                pilot_matches += 1
            plants.extend(dict(condition=condition,field_seed=seed,**p) for p in stand)
            data = pd.read_csv(root/'tau.csv')
            probes = pd.read_csv(root/'probes.csv')
            assert data.time_mst.tolist()==config['times'] and data.valid.all()
            assert len(probes)==7500 and np.isfinite(probes.scalar).all() and (probes.scalar>=0).all()
            means = probes.groupby(['time_mst','cultivar','level']).scalar.agg(['mean','count'])
            assert len(means)==150 and (means['count']==50).all()
            for row in data.itertuples():
                for genotype in ('BTX','Pawaga'):
                    top = means.loc[(row.time_mst,genotype,'top'),'mean']
                    bottom = means.loc[(row.time_mst,genotype,'bottom'),'mean']
                    assert np.isclose(top,getattr(row,genotype+'_top'),rtol=0,atol=1e-12)
                    assert np.isclose(bottom,getattr(row,genotype+'_bottom'),rtol=0,atol=1e-12)
                    assert np.isclose(bottom/top,getattr(row,genotype+'_tau'),rtol=0,atol=1e-12)
            data['seed'],data['condition']=seed,condition
            long = data.melt(id_vars=['condition','seed','time_mst'],value_vars=['BTX_tau','Pawaga_tau'],
                             var_name='genotype',value_name='tau')
            long['genotype']=long.genotype.map({'BTX_tau':'BTx623','Pawaga_tau':'Pawaga'})
            frames.append(long)
            probe_rows += len(probes)
        if (seed-SEEDS[0]+1)%30 == 0:
            print(f'Validated {seed-SEEDS[0]+1}/270 paired seeds',flush=True)
    data = pd.concat(frames,ignore_index=True)
    band = summarize(data,SEEDS,config['times'])
    old = pd.read_csv(FINAL/'july26_no_panicles_quantiles.csv').set_index(['genotype','time_mst']).sort_index()
    check = band[band.condition=='original'].set_index(['genotype','time_mst']).sort_index()
    for key in ('minimum','median','maximum'):
        assert np.allclose(check[key],old[key],rtol=0,atol=1e-12)
    field = pd.read_csv(FINAL/'july26_no_panicles_field_minutes.csv',parse_dates=['minute'])
    assert field.valid.all() and field.date.eq(DATE).all()
    for genotype in ('BTx623','Pawaga'):
        f = field[field.genotype==genotype].sort_values('minute')
        assert len(f)==361 and f.minute.diff().dropna().eq(pd.Timedelta(minutes=1)).all()
        assert pd.to_datetime(DATE+' '+pd.Series(config['times'])).isin(f.minute).all()
    data.to_csv(OUTPUT/'simulation_seed_values.csv',index=False)
    band.to_csv(OUTPUT/'simulation_time_summary.csv',index=False)
    field.to_csv(OUTPUT/'field_minutes_used.csv',index=False)
    pd.DataFrame(plants).groupby(['condition','genotype']).leaf_count.agg(
        n='size',mean='mean',std='std',minimum='min',maximum='max').to_csv(OUTPUT/'plant_leaf_count_summary.csv')
    comparisons = []
    for genotype in ('BTx623','Pawaga'):
        original = band[(band.genotype==genotype)&(band.condition=='original')].set_index('time_mst')
        for condition in ('half_sd','quarter_sd'):
            b = band[(band.genotype==genotype)&(band.condition==condition)].set_index('time_mst')
            comparisons.append(dict(genotype=genotype,condition=condition,times=len(b),
                narrower_times=int((b.range_width<original.range_width).sum()),
                original_mean_width=original.range_width.mean(),mean_width=b.range_width.mean(),
                mean_width_change_pct=100*(b.range_width.mean()/original.range_width.mean()-1),
                mean_median_shift=(b['median']-original['median']).mean(),
                min_width_change=(b.range_width-original.range_width).min(),
                max_width_change=(b.range_width-original.range_width).max()))
    comparison = pd.DataFrame(comparisons)
    comparison.to_csv(OUTPUT/'range_comparison.csv',index=False)
    upper = max(.75,math.ceil(max(data.tau.max(),field.tau.max())*1.05*10)/10)
    figures(band,field,upper)
    for path,expected in config['hashes'].items():
        assert sha(path)==expected,path
    validation = dict(status='verified',new_days=540,reused_days=270,full_day_runtime_controls=3,
        seeds_per_condition=270,times_per_day=25,probe_rows=probe_rows,seed_genotype_time_rows=len(data),
        pilot_plant_states_matched=pilot_matches,baseline_min_median_max_reproduced=True,
        field_minutes_unchanged=722,input_hashes_unchanged=True,early_stopping=False,
        analysis_sha256=sha(__file__),common_y_upper=upper)
    assert probe_rows==6075000 and len(data)==40500 and pilot_matches==90
    (OUTPUT/'validation.json').write_text(json.dumps(validation,indent=2))
    print(comparison.to_string(index=False))
    print(json.dumps(validation,indent=2))


if __name__=='__main__':
    main()
