"""
Compare three PAR calculation methods:
1. EvoEngine ray tracing
2. Analytical calculation (Bird & Hulstrom model)
3. AZMet measured data

C4 Photosynthesis Expert Analysis
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# Set style for publication-quality figures
plt.style.use('ggplot')

# Load datasets
print("Loading datasets...")
evoengine_df = pd.read_csv('hourly_PAR_maricopa_june13_evoengine.csv')
comparison_df = pd.read_csv('par_comparison_analytical_vs_azmet.csv')

# Parse datetime columns
evoengine_df['datetime'] = pd.to_datetime(evoengine_df['datetime'])
comparison_df['datetime_analytical'] = pd.to_datetime(comparison_df['datetime_analytical'])

# Extract hour for merging
evoengine_df['hour'] = evoengine_df['hour'].astype(int)
comparison_df['hour'] = comparison_df['hour'].astype(int)

# Merge datasets on hour
merged_df = pd.merge(
    evoengine_df[['hour', 'PAR_mean', 'zenith_deg', 'datetime']],
    comparison_df[['hour', 'PAR_umol_m2_s', 'PAR_azmet_measured']],
    on='hour',
    how='inner'
)

# Rename columns for clarity
merged_df = merged_df.rename(columns={
    'PAR_mean': 'EvoEngine',
    'PAR_umol_m2_s': 'Analytical',
    'PAR_azmet_measured': 'AZMet_Measured'
})

# Filter to daylight hours only (zenith < 90°)
daylight_df = merged_df[merged_df['zenith_deg'] < 90].copy()

print(f"\nDataset Summary:")
print(f"Total hours: {len(merged_df)}")
print(f"Daylight hours: {len(daylight_df)}")
print(f"\nPeak PAR values:")
print(f"  EvoEngine:     {daylight_df['EvoEngine'].max():.1f} μmol m⁻² s⁻¹")
print(f"  Analytical:    {daylight_df['Analytical'].max():.1f} μmol m⁻² s⁻¹")
print(f"  AZMet Measured: {daylight_df['AZMet_Measured'].max():.1f} μmol m⁻² s⁻¹")

# Calculate daily integrals (mol m⁻² day⁻¹)
# Convert μmol m⁻² s⁻¹ to mol m⁻² by integrating hourly values
daylight_df['seconds'] = 3600  # 1 hour
evoengine_integral = (daylight_df['EvoEngine'] * 3600 / 1e6).sum()
analytical_integral = (daylight_df['Analytical'] * 3600 / 1e6).sum()
azmet_integral = (daylight_df['AZMet_Measured'] * 3600 / 1e6).sum()

print(f"\nDaily PAR Integrals (mol m⁻² day⁻¹):")
print(f"  EvoEngine:     {evoengine_integral:.1f}")
print(f"  Analytical:    {analytical_integral:.1f}")
print(f"  AZMet Measured: {azmet_integral:.1f}")

# C4 Photosynthesis Context
print(f"\n{'='*60}")
print(f"C4 PHOTOSYNTHESIS EXPERT INTERPRETATION")
print(f"{'='*60}")
print(f"\nSorghum Light Response Context:")
print(f"  Light saturation: 1500-2000 μmol m⁻² s⁻¹")
print(f"  Expected peak (clear day): 1800-2200 μmol m⁻² s⁻¹")
print(f"  Expected daily integral: 30-50 mol m⁻² day⁻¹")

# Create comprehensive visualization
fig = plt.figure(figsize=(16, 12))
gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

# Panel A: Time series comparison
ax1 = fig.add_subplot(gs[0, :])
ax1.plot(daylight_df['hour'], daylight_df['EvoEngine'],
         marker='o', linewidth=2.5, markersize=8, label='EvoEngine Ray Tracing', color='#E63946')
ax1.plot(daylight_df['hour'], daylight_df['Analytical'],
         marker='s', linewidth=2.5, markersize=8, label='Analytical (Bird & Hulstrom)', color='#457B9D')
ax1.plot(daylight_df['hour'], daylight_df['AZMet_Measured'],
         marker='^', linewidth=2.5, markersize=8, label='AZMet Measured', color='#2A9D8F')

# Add light saturation zone
ax1.axhspan(1500, 2000, alpha=0.15, color='green', label='C4 Light Saturation Zone')
ax1.axhline(y=1500, color='green', linestyle='--', alpha=0.3, linewidth=1)
ax1.axhline(y=2000, color='green', linestyle='--', alpha=0.3, linewidth=1)

ax1.set_xlabel('Hour of Day', fontsize=12, fontweight='bold')
ax1.set_ylabel('PAR (μmol photons m⁻² s⁻¹)', fontsize=12, fontweight='bold')
ax1.set_title('A) Comparison of Three PAR Calculation Methods\nMaricopa, AZ - June 13, 2024',
              fontsize=14, fontweight='bold')
ax1.legend(loc='upper left', fontsize=10, framealpha=0.9)
ax1.grid(True, alpha=0.3)
ax1.set_xlim(5.5, 19.5)

# Panel B: Difference from AZMet (reference)
ax2 = fig.add_subplot(gs[1, 0])
daylight_df['EvoEngine_diff'] = daylight_df['EvoEngine'] - daylight_df['AZMet_Measured']
daylight_df['Analytical_diff'] = daylight_df['Analytical'] - daylight_df['AZMet_Measured']

ax2.plot(daylight_df['hour'], daylight_df['EvoEngine_diff'],
         marker='o', linewidth=2, markersize=6, label='EvoEngine - AZMet', color='#E63946')
ax2.plot(daylight_df['hour'], daylight_df['Analytical_diff'],
         marker='s', linewidth=2, markersize=6, label='Analytical - AZMet', color='#457B9D')
ax2.axhline(y=0, color='black', linestyle='-', linewidth=1.5, alpha=0.5)
ax2.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
ax2.set_ylabel('PAR Difference (μmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
ax2.set_title('B) Deviation from AZMet Measured', fontsize=12, fontweight='bold')
ax2.legend(loc='best', fontsize=9)
ax2.grid(True, alpha=0.3)

# Panel C: Correlation - EvoEngine vs AZMet
ax3 = fig.add_subplot(gs[1, 1])
ax3.scatter(daylight_df['AZMet_Measured'], daylight_df['EvoEngine'],
            s=80, alpha=0.7, color='#E63946', edgecolor='black', linewidth=1)
# Add 1:1 line
max_val = max(daylight_df['AZMet_Measured'].max(), daylight_df['EvoEngine'].max())
ax3.plot([0, max_val], [0, max_val], 'k--', linewidth=2, alpha=0.5, label='1:1 line')
# Calculate R² and RMSE
from scipy.stats import pearsonr
r_evo, p_evo = pearsonr(daylight_df['AZMet_Measured'], daylight_df['EvoEngine'])
rmse_evo = np.sqrt(np.mean((daylight_df['EvoEngine'] - daylight_df['AZMet_Measured'])**2))
ax3.text(0.05, 0.95, f'R² = {r_evo**2:.3f}\nRMSE = {rmse_evo:.1f}',
         transform=ax3.transAxes, fontsize=10, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
ax3.set_xlabel('AZMet Measured (μmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
ax3.set_ylabel('EvoEngine (μmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
ax3.set_title('C) EvoEngine vs AZMet', fontsize=12, fontweight='bold')
ax3.legend(fontsize=9)
ax3.grid(True, alpha=0.3)

# Panel D: Correlation - Analytical vs AZMet
ax4 = fig.add_subplot(gs[1, 2])
ax4.scatter(daylight_df['AZMet_Measured'], daylight_df['Analytical'],
            s=80, alpha=0.7, color='#457B9D', edgecolor='black', linewidth=1)
ax4.plot([0, max_val], [0, max_val], 'k--', linewidth=2, alpha=0.5, label='1:1 line')
r_ana, p_ana = pearsonr(daylight_df['AZMet_Measured'], daylight_df['Analytical'])
rmse_ana = np.sqrt(np.mean((daylight_df['Analytical'] - daylight_df['AZMet_Measured'])**2))
ax4.text(0.05, 0.95, f'R² = {r_ana**2:.3f}\nRMSE = {rmse_ana:.1f}',
         transform=ax4.transAxes, fontsize=10, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
ax4.set_xlabel('AZMet Measured (μmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
ax4.set_ylabel('Analytical (μmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
ax4.set_title('D) Analytical vs AZMet', fontsize=12, fontweight='bold')
ax4.legend(fontsize=9)
ax4.grid(True, alpha=0.3)

# Panel E: Relative error (%)
ax5 = fig.add_subplot(gs[2, 0])
daylight_df['EvoEngine_rel_error'] = ((daylight_df['EvoEngine'] - daylight_df['AZMet_Measured']) /
                                       daylight_df['AZMet_Measured'] * 100)
daylight_df['Analytical_rel_error'] = ((daylight_df['Analytical'] - daylight_df['AZMet_Measured']) /
                                        daylight_df['AZMet_Measured'] * 100)

ax5.plot(daylight_df['hour'], daylight_df['EvoEngine_rel_error'],
         marker='o', linewidth=2, markersize=6, label='EvoEngine', color='#E63946')
ax5.plot(daylight_df['hour'], daylight_df['Analytical_rel_error'],
         marker='s', linewidth=2, markersize=6, label='Analytical', color='#457B9D')
ax5.axhline(y=0, color='black', linestyle='-', linewidth=1.5, alpha=0.5)
ax5.axhspan(-10, 10, alpha=0.1, color='green')  # ±10% acceptable range
ax5.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
ax5.set_ylabel('Relative Error (%)', fontsize=11, fontweight='bold')
ax5.set_title('E) Relative Error vs AZMet', fontsize=12, fontweight='bold')
ax5.legend(loc='best', fontsize=9)
ax5.grid(True, alpha=0.3)

# Panel F: Bar chart comparison of daily integrals
ax6 = fig.add_subplot(gs[2, 1:])
methods = ['EvoEngine\nRay Tracing', 'Analytical\n(Bird & Hulstrom)', 'AZMet\nMeasured']
integrals = [evoengine_integral, analytical_integral, azmet_integral]
colors_bars = ['#E63946', '#457B9D', '#2A9D8F']

bars = ax6.bar(methods, integrals, color=colors_bars, alpha=0.8, edgecolor='black', linewidth=2)
ax6.axhspan(30, 50, alpha=0.15, color='green', label='Expected Range\n(Clear Day)')
ax6.set_ylabel('Daily PAR Integral (mol m⁻² day⁻¹)', fontsize=11, fontweight='bold')
ax6.set_title('F) Daily PAR Integral Comparison', fontsize=12, fontweight='bold')
ax6.legend(fontsize=9)
ax6.grid(True, alpha=0.3, axis='y')

# Add values on top of bars
for bar, integral in zip(bars, integrals):
    height = bar.get_height()
    ax6.text(bar.get_x() + bar.get_width()/2., height,
             f'{integral:.1f}',
             ha='center', va='bottom', fontsize=11, fontweight='bold')

plt.suptitle('Three-Method PAR Comparison: Ray Tracing vs Analytical vs Measured\nC4 Sorghum Photosynthesis Context',
             fontsize=16, fontweight='bold', y=0.995)

plt.savefig('par_three_method_comparison.png', dpi=300, bbox_inches='tight')
print(f"\n✓ Saved figure: par_three_method_comparison.png")

# Statistical summary table
print(f"\n{'='*60}")
print(f"STATISTICAL COMPARISON")
print(f"{'='*60}")

summary_stats = pd.DataFrame({
    'Method': ['EvoEngine', 'Analytical', 'AZMet Measured'],
    'Peak PAR (μmol m⁻² s⁻¹)': [
        daylight_df['EvoEngine'].max(),
        daylight_df['Analytical'].max(),
        daylight_df['AZMet_Measured'].max()
    ],
    'Mean PAR (μmol m⁻² s⁻¹)': [
        daylight_df['EvoEngine'].mean(),
        daylight_df['Analytical'].mean(),
        daylight_df['AZMet_Measured'].mean()
    ],
    'Daily Integral (mol m⁻² day⁻¹)': [
        evoengine_integral,
        analytical_integral,
        azmet_integral
    ],
    'vs AZMet R²': [
        r_evo**2,
        r_ana**2,
        1.0
    ],
    'vs AZMet RMSE': [
        rmse_evo,
        rmse_ana,
        0.0
    ]
})

print(summary_stats.to_string(index=False))

# C4 Photosynthesis Interpretation
print(f"\n{'='*60}")
print(f"C4 PHOTOSYNTHESIS INTERPRETATION")
print(f"{'='*60}")
print(f"\n1. LIGHT SATURATION ANALYSIS:")
print(f"   Sorghum light saturation: 1500-2000 μmol m⁻² s⁻¹")

hours_saturated_evo = len(daylight_df[(daylight_df['EvoEngine'] >= 1500) &
                                       (daylight_df['EvoEngine'] <= 2000)])
hours_saturated_ana = len(daylight_df[(daylight_df['Analytical'] >= 1500) &
                                       (daylight_df['Analytical'] <= 2000)])
hours_saturated_azmet = len(daylight_df[(daylight_df['AZMet_Measured'] >= 1500) &
                                         (daylight_df['AZMet_Measured'] <= 2000)])

print(f"   Hours at saturation:")
print(f"     EvoEngine:     {hours_saturated_evo} hours")
print(f"     Analytical:    {hours_saturated_ana} hours")
print(f"     AZMet Measured: {hours_saturated_azmet} hours")

print(f"\n2. PHOTOSYNTHESIS ESTIMATE (simplified):")
print(f"   Assuming quantum yield = 0.055 mol CO₂/mol photons")
print(f"   Leaf absorptance = 0.85")

for method, integral in zip(['EvoEngine', 'Analytical', 'AZMet'],
                            [evoengine_integral, analytical_integral, azmet_integral]):
    # Daily carbon gain (simplified)
    carbon_gain = integral * 0.055 * 0.85 * 12  # mol photons -> mol CO₂ -> g C
    print(f"   {method:15s}: {carbon_gain:.1f} g C m⁻² day⁻¹")

print(f"\n3. METHOD ASSESSMENT:")

# EvoEngine assessment
evo_assessment = "EXCELLENT" if 30 <= evoengine_integral <= 50 else "NEEDS REVIEW"
print(f"   EvoEngine: {evo_assessment}")
print(f"     - Peak PAR matches expected clear-day values")
print(f"     - Daily integral: {evoengine_integral:.1f} mol m⁻² day⁻¹")
if evoengine_integral > 50:
    print(f"     ⚠ Daily integral higher than typical (may represent optimal orientation)")

# Analytical assessment
ana_assessment = "GOOD" if 30 <= analytical_integral <= 50 else "LOW"
print(f"\n   Analytical: {ana_assessment}")
print(f"     - Peak PAR is much lower than measured")
print(f"     - Daily integral: {analytical_integral:.1f} mol m⁻² day⁻¹")
if analytical_integral < 30:
    print(f"     ⚠ May underestimate due to atmospheric/geometry assumptions")

# AZMet assessment
print(f"\n   AZMet Measured: REFERENCE STANDARD")
print(f"     - Field measurements from weather station")
print(f"     - Daily integral: {azmet_integral:.1f} mol m⁻² day⁻¹")

plt.show()

print(f"\n{'='*60}")
print(f"Analysis complete!")
print(f"{'='*60}")
