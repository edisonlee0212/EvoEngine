#!/usr/bin/env python3
"""
Plot June 12, 2024 Analytical vs AZMet Comparison
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Load the comparison data
comparison_file = Path('./analytical_par_results/par_comparison_analytical_vs_azmet_june12.csv')
df = pd.read_csv(comparison_file)

# Create figure
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# 1. Time series comparison
ax1 = axes[0, 0]
ax1.plot(df['hour'], df['PAR_umol_m2_s'],
        'o-', linewidth=2.5, markersize=8, color='#2ca02c',
        label='Analytical (30° leaf)')
ax1.plot(df['hour'], df['PAR_azmet_measured'],
        's--', linewidth=2, markersize=7, color='#ff7f0e',
        label='AZMet measured (horizontal)')
ax1.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
ax1.set_ylabel('PAR (μmol photons m⁻² s⁻¹)', fontsize=11, fontweight='bold')
ax1.set_title('A. PAR Comparison: Analytical vs AZMet', fontsize=12, fontweight='bold', loc='left')
ax1.legend(loc='upper right', fontsize=9)
ax1.grid(alpha=0.3)
ax1.set_xlim(5, 21)

# Add light saturation reference
ax1.axhline(y=1500, color='red', linestyle=':', linewidth=2, alpha=0.5,
           label='C4 light saturation')
ax1.legend(loc='upper right', fontsize=9)

# 2. Scatter plot
ax2 = axes[0, 1]
valid_data = df.dropna(subset=['PAR_umol_m2_s', 'PAR_azmet_measured'])
valid_data = valid_data[valid_data['PAR_azmet_measured'] > 0]

ax2.scatter(valid_data['PAR_azmet_measured'], valid_data['PAR_umol_m2_s'],
           s=100, alpha=0.7, edgecolors='black', linewidth=1.5, c=valid_data['hour'],
           cmap='viridis')

# Add 1:1 line
max_val = max(valid_data['PAR_azmet_measured'].max(), valid_data['PAR_umol_m2_s'].max())
ax2.plot([0, max_val], [0, max_val], 'k--', linewidth=2, alpha=0.5, label='1:1 line')

# Add expected line for 30° tilt (~cos(30°) = 0.866 reduction)
ax2.plot([0, max_val], [0, max_val * 0.866], 'r:', linewidth=2, alpha=0.5,
        label='Expected (cos 30°)')

ax2.set_xlabel('AZMet PAR (horizontal, μmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
ax2.set_ylabel('Analytical PAR (30° leaf, μmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
ax2.set_title('B. Analytical vs Measured', fontsize=12, fontweight='bold', loc='left')
ax2.legend(loc='upper left', fontsize=9)
ax2.grid(alpha=0.3)

if len(valid_data) > 0:
    correlation = valid_data['PAR_umol_m2_s'].corr(valid_data['PAR_azmet_measured'])
    ax2.text(0.98, 0.02, f'R = {correlation:.3f}',
            transform=ax2.transAxes, ha='right', va='bottom',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9),
            fontsize=10, fontweight='bold')

# 3. PAR vs Solar Elevation (light response curve)
ax3 = axes[1, 0]
daylight = df[df['elevation_deg'] > 0]
scatter3 = ax3.scatter(daylight['elevation_deg'], daylight['PAR_umol_m2_s'],
                      s=120, c=daylight['hour'], cmap='viridis',
                      edgecolors='black', linewidth=1.5, label='Analytical')

# Add AZMet data
ax3.scatter(daylight['elevation_deg'], daylight['PAR_azmet_measured'],
           s=120, c=daylight['hour'], cmap='plasma', marker='s',
           edgecolors='black', linewidth=1.5, label='AZMet')

ax3.set_xlabel('Solar Elevation (°)', fontsize=11, fontweight='bold')
ax3.set_ylabel('PAR (μmol photons m⁻² s⁻¹)', fontsize=11, fontweight='bold')
ax3.set_title('C. Light Response to Solar Angle', fontsize=12, fontweight='bold', loc='left')
ax3.legend(loc='upper left', fontsize=9)
ax3.grid(alpha=0.3)

# 4. Cumulative comparison
ax4 = axes[1, 1]
cumulative_analytical = np.cumsum(df['PAR_umol_m2_s']) * 3600 / 1e6
cumulative_azmet = np.cumsum(df['PAR_azmet_measured'].fillna(0)) * 3600 / 1e6

ax4.plot(df['hour'], cumulative_analytical, 'o-',
        linewidth=2.5, markersize=7, color='darkgreen', label='Analytical')
ax4.plot(df['hour'], cumulative_azmet, 's--',
        linewidth=2, markersize=6, color='darkorange', label='AZMet')
ax4.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
ax4.set_ylabel('Cumulative PAR (mol photons m⁻²)', fontsize=11, fontweight='bold')
ax4.set_title('D. Daily PAR Accumulation', fontsize=12, fontweight='bold', loc='left')
ax4.legend(loc='upper left', fontsize=9)
ax4.grid(alpha=0.3)
ax4.set_xlim(5, 21)

# Add final values
final_analytical = cumulative_analytical.iloc[-1]
final_azmet = cumulative_azmet.iloc[-1]
ax4.text(0.98, 0.95, f'Analytical: {final_analytical:.1f} mol m⁻² day⁻¹\nAZMet: {final_azmet:.1f} mol m⁻² day⁻¹',
        transform=ax4.transAxes, ha='right', va='top',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9),
        fontsize=10, fontweight='bold')

# Add expected range shading
ax4.axhspan(30, 50, alpha=0.1, color='green', label='Expected (horizontal)')
ax4.legend(loc='upper left', fontsize=9)

# Overall title
fig.suptitle(f'Sorghum Leaf PAR - Analytical Model vs AZMet Measurements\n' +
             f'Maricopa, AZ - June 12, 2024',
             fontsize=13, fontweight='bold', y=0.98)

plt.tight_layout(rect=[0, 0, 1, 0.96])

# Save
output_path = Path('./analytical_par_results/june12_analytical_vs_azmet_comparison.png')
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"✓ Saved: {output_path}")

# Print summary statistics
print("\n" + "="*80)
print("JUNE 12, 2024 - COMPARISON SUMMARY")
print("="*80)
print(f"Peak PAR (Analytical): {df['PAR_umol_m2_s'].max():.1f} μmol m⁻² s⁻¹")
print(f"Peak PAR (AZMet):      {df['PAR_azmet_measured'].max():.1f} μmol m⁻² s⁻¹")
print(f"Daily PAR (Analytical): {final_analytical:.2f} mol m⁻² day⁻¹")
print(f"Daily PAR (AZMet):      {final_azmet:.2f} mol m⁻² day⁻¹")
print(f"Difference:             {final_azmet - final_analytical:.2f} mol m⁻² day⁻¹ ({(final_azmet/final_analytical - 1)*100:.1f}%)")
if len(valid_data) > 0:
    print(f"Correlation:            {correlation:.3f}")
print("="*80)
