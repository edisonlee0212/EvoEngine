#!/usr/bin/env python3
"""
Comprehensive Comparison: EvoEngine vs Analytical vs AZMET
June 12-13, 2024 - 7-Leaf Sorghum PAR Analysis

Compares three methods for calculating PAR:
1. EvoEngine GPU Ray Tracing (7-leaf sorghum)
2. Analytical Model (Beer's Law + atmospheric attenuation)
3. AZMET Field Measurements (Maricopa, AZ station)

Author: Claude Code (C4 Expert)
Date: November 4, 2025
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# =============================================================================
# Configuration
# =============================================================================

# File paths
CLAUDE_DIR = Path(r'C:\Users\Brenda\code\EvoEngine\claude')
EVOENGINE_DIR = Path(r'C:\Users\Brenda\code\EvoEngine\out\build\x64-Release\PythonBinding\evoengine_par_results')

# Input files
EVOENGINE_JUNE12 = EVOENGINE_DIR / 'hourly_PAR_7leaf_plant_maricopa_june12.csv'
EVOENGINE_JUNE13 = EVOENGINE_DIR / 'hourly_PAR_7leaf_plant_maricopa_june13.csv'
ANALYTICAL_JUNE12 = CLAUDE_DIR / 'par_comparison_analytical_vs_azmet_june12.csv'
ANALYTICAL_JUNE13 = CLAUDE_DIR / 'par_comparison_analytical_vs_azmet.csv'

# Output
OUTPUT_DIR = CLAUDE_DIR / 'three_method_comparison'
OUTPUT_DIR.mkdir(exist_ok=True)

# =============================================================================
# Data Loading
# =============================================================================

print("="*80)
print("THREE-METHOD PAR COMPARISON: EvoEngine vs Analytical vs AZMET")
print("="*80)
print()

print("Loading data files...")

# Load EvoEngine data
evoengine_june12 = pd.read_csv(EVOENGINE_JUNE12)
evoengine_june13 = pd.read_csv(EVOENGINE_JUNE13)

# Load Analytical + AZMET data (already merged)
analytical_june12 = pd.read_csv(ANALYTICAL_JUNE12)
analytical_june13 = pd.read_csv(ANALYTICAL_JUNE13)

print(f"  EvoEngine June 12: {len(evoengine_june12)} records")
print(f"  EvoEngine June 13: {len(evoengine_june13)} records")
print(f"  Analytical June 12: {len(analytical_june12)} records")
print(f"  Analytical June 13: {len(analytical_june13)} records")
print()

# =============================================================================
# Data Merging
# =============================================================================

print("Merging datasets...")

# June 12
june12 = pd.DataFrame({
    'hour': evoengine_june12['hour'],
    'elevation_deg': evoengine_june12['elevation_deg'],
    'PAR_EvoEngine': evoengine_june12['PAR_mean'],
    'PAR_Analytical': analytical_june12['PAR_umol_m2_s'],
    'PAR_AZMET': analytical_june12['PAR_azmet_measured']
})
june12['date'] = 'June 12, 2024'

# June 13
june13 = pd.DataFrame({
    'hour': evoengine_june13['hour'],
    'elevation_deg': evoengine_june13['elevation_deg'],
    'PAR_EvoEngine': evoengine_june13['PAR_mean'],
    'PAR_Analytical': analytical_june13['PAR_umol_m2_s'],
    'PAR_AZMET': analytical_june13['PAR_azmet_measured']
})
june13['date'] = 'June 13, 2024'

# Combine both days
combined = pd.concat([june12, june13], ignore_index=True)

print(f"  Combined dataset: {len(combined)} records")
print()

# =============================================================================
# Statistical Analysis
# =============================================================================

print("="*80)
print("STATISTICAL COMPARISON")
print("="*80)
print()

def calculate_statistics(df, day_name):
    """Calculate comprehensive statistics for one day"""

    # Filter daylight hours only
    daylight = df[df['PAR_EvoEngine'] > 0].copy()

    stats = {
        'Day': day_name,
        'Records': len(daylight),

        # Peak values
        'Peak_EvoEngine': daylight['PAR_EvoEngine'].max(),
        'Peak_Analytical': daylight['PAR_Analytical'].max(),
        'Peak_AZMET': daylight['PAR_AZMET'].max(),

        # Mean values (daylight hours)
        'Mean_EvoEngine': daylight['PAR_EvoEngine'].mean(),
        'Mean_Analytical': daylight['PAR_Analytical'].mean(),
        'Mean_AZMET': daylight['PAR_AZMET'].mean(),

        # Daily integrals (mol m-2 day-1)
        'Daily_EvoEngine': df['PAR_EvoEngine'].sum() * 3600 / 1e6,
        'Daily_Analytical': df['PAR_Analytical'].sum() * 3600 / 1e6,
        'Daily_AZMET': df['PAR_AZMET'].sum() * 3600 / 1e6,
    }

    # Calculate differences and ratios
    stats['EvoEngine_vs_Analytical_mean_diff'] = stats['Mean_EvoEngine'] - stats['Mean_Analytical']
    stats['EvoEngine_vs_AZMET_mean_diff'] = stats['Mean_EvoEngine'] - stats['Mean_AZMET']
    stats['EvoEngine_vs_Analytical_ratio'] = stats['Mean_EvoEngine'] / stats['Mean_Analytical'] if stats['Mean_Analytical'] > 0 else np.nan
    stats['EvoEngine_vs_AZMET_ratio'] = stats['Mean_EvoEngine'] / stats['Mean_AZMET'] if stats['Mean_AZMET'] > 0 else np.nan

    return stats

# Calculate for both days
stats_june12 = calculate_statistics(june12, 'June 12')
stats_june13 = calculate_statistics(june13, 'June 13')

# Print summary
for stats in [stats_june12, stats_june13]:
    print(f"{stats['Day']} - Summary Statistics")
    print("-" * 80)
    print(f"  Peak PAR (umol m-2 s-1):")
    print(f"    EvoEngine:  {stats['Peak_EvoEngine']:8.1f}")
    print(f"    Analytical: {stats['Peak_Analytical']:8.1f}")
    print(f"    AZMET:      {stats['Peak_AZMET']:8.1f}")
    print()
    print(f"  Mean PAR (daylight hours, umol m-2 s-1):")
    print(f"    EvoEngine:  {stats['Mean_EvoEngine']:8.1f}")
    print(f"    Analytical: {stats['Mean_Analytical']:8.1f}")
    print(f"    AZMET:      {stats['Mean_AZMET']:8.1f}")
    print()
    print(f"  Daily PAR Integral (mol m-2 day-1):")
    print(f"    EvoEngine:  {stats['Daily_EvoEngine']:8.2f}")
    print(f"    Analytical: {stats['Daily_Analytical']:8.2f}")
    print(f"    AZMET:      {stats['Daily_AZMET']:8.2f}")
    print()
    print(f"  EvoEngine vs Analytical:")
    print(f"    Mean difference: {stats['EvoEngine_vs_Analytical_mean_diff']:+8.1f} umol m-2 s-1")
    print(f"    Ratio:           {stats['EvoEngine_vs_Analytical_ratio']:8.3f}")
    print()
    print(f"  EvoEngine vs AZMET:")
    print(f"    Mean difference: {stats['EvoEngine_vs_AZMET_mean_diff']:+8.1f} umol m-2 s-1")
    print(f"    Ratio:           {stats['EvoEngine_vs_AZMET_ratio']:8.3f}")
    print()
    print()

# =============================================================================
# Visualization
# =============================================================================

print("Creating visualizations...")

# Create comprehensive 4-panel figure for each day
for df, day_name in [(june12, 'June 12'), (june13, 'June 13')]:

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(f'PAR Comparison: Three Methods - {day_name}, 2024\n' +
                 f'Maricopa, AZ (33.07°N, 111.97°W)',
                 fontsize=14, fontweight='bold')

    # Panel 1: Time series comparison
    ax1 = axes[0, 0]
    ax1.plot(df['hour'], df['PAR_EvoEngine'], 'o-', linewidth=2.5, markersize=8,
            color='#e74c3c', label='EvoEngine (Ray Tracing)', zorder=3)
    ax1.plot(df['hour'], df['PAR_Analytical'], 's--', linewidth=2, markersize=7,
            color='#3498db', label='Analytical Model', alpha=0.8, zorder=2)
    ax1.plot(df['hour'], df['PAR_AZMET'], '^:', linewidth=2, markersize=7,
            color='#2ecc71', label='AZMET Measured', alpha=0.8, zorder=1)

    ax1.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
    ax1.set_ylabel('PAR (µmol photons m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax1.set_title('A. Hourly PAR Time Series', fontsize=12, fontweight='bold', loc='left')
    ax1.legend(loc='upper left', fontsize=10, framealpha=0.9)
    ax1.grid(alpha=0.3, linestyle='--')
    ax1.set_xlim(5, 21)
    ax1.set_ylim(0, None)

    # Add light saturation reference
    ax1.axhline(y=1500, color='purple', linestyle=':', linewidth=2, alpha=0.5,
               label='C4 Light Saturation (~1500)')
    ax1.legend(loc='upper left', fontsize=10, framealpha=0.9)

    # Panel 2: Method comparison scatter plots
    ax2 = axes[0, 1]
    daylight = df[df['PAR_EvoEngine'] > 0]

    # EvoEngine vs Analytical
    ax2.scatter(daylight['PAR_Analytical'], daylight['PAR_EvoEngine'],
               s=100, alpha=0.7, edgecolors='black', linewidth=1.5,
               c=daylight['hour'], cmap='viridis', label='EvoEngine vs Analytical')

    # 1:1 line
    max_val = max(daylight['PAR_Analytical'].max(), daylight['PAR_EvoEngine'].max())
    ax2.plot([0, max_val], [0, max_val], 'k--', linewidth=2, alpha=0.5, label='1:1 Line')

    ax2.set_xlabel('Analytical PAR (µmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('EvoEngine PAR (µmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax2.set_title('B. EvoEngine vs Analytical', fontsize=12, fontweight='bold', loc='left')
    ax2.legend(fontsize=10)
    ax2.grid(alpha=0.3, linestyle='--')
    ax2.set_aspect('equal')

    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap='viridis',
                               norm=plt.Normalize(vmin=daylight['hour'].min(),
                                                vmax=daylight['hour'].max()))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax2)
    cbar.set_label('Hour', fontsize=10)

    # Panel 3: EvoEngine vs AZMET scatter
    ax3 = axes[1, 0]
    ax3.scatter(daylight['PAR_AZMET'], daylight['PAR_EvoEngine'],
               s=100, alpha=0.7, edgecolors='black', linewidth=1.5,
               c=daylight['hour'], cmap='plasma', label='EvoEngine vs AZMET')

    # 1:1 line
    max_val = max(daylight['PAR_AZMET'].max(), daylight['PAR_EvoEngine'].max())
    ax3.plot([0, max_val], [0, max_val], 'k--', linewidth=2, alpha=0.5, label='1:1 Line')

    ax3.set_xlabel('AZMET PAR (µmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax3.set_ylabel('EvoEngine PAR (µmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax3.set_title('C. EvoEngine vs AZMET Measurements', fontsize=12, fontweight='bold', loc='left')
    ax3.legend(fontsize=10)
    ax3.grid(alpha=0.3, linestyle='--')
    ax3.set_aspect('equal')

    # Add colorbar
    sm2 = plt.cm.ScalarMappable(cmap='plasma',
                                norm=plt.Normalize(vmin=daylight['hour'].min(),
                                                 vmax=daylight['hour'].max()))
    sm2.set_array([])
    cbar2 = plt.colorbar(sm2, ax=ax3)
    cbar2.set_label('Hour', fontsize=10)

    # Panel 4: Residuals (differences)
    ax4 = axes[1, 1]

    # Calculate residuals
    evo_minus_analytical = df['PAR_EvoEngine'] - df['PAR_Analytical']
    evo_minus_azmet = df['PAR_EvoEngine'] - df['PAR_AZMET']

    ax4.plot(df['hour'], evo_minus_analytical, 'o-', linewidth=2, markersize=7,
            color='#3498db', label='EvoEngine - Analytical')
    ax4.plot(df['hour'], evo_minus_azmet, 's-', linewidth=2, markersize=7,
            color='#2ecc71', label='EvoEngine - AZMET')

    ax4.axhline(y=0, color='black', linestyle='-', linewidth=1.5, alpha=0.7)
    ax4.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
    ax4.set_ylabel('PAR Difference (µmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax4.set_title('D. Method Differences (Residuals)', fontsize=12, fontweight='bold', loc='left')
    ax4.legend(fontsize=10)
    ax4.grid(alpha=0.3, linestyle='--')
    ax4.set_xlim(5, 21)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    # Save figure
    filename = f'three_method_comparison_{day_name.replace(" ", "_").replace(",", "").lower()}.png'
    filepath = OUTPUT_DIR / filename
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    print(f"  Saved: {filepath}")
    plt.close()

# =============================================================================
# Combined Comparison (Both Days)
# =============================================================================

print()
print("Creating combined two-day comparison...")

fig, axes = plt.subplots(1, 2, figsize=(16, 6))
fig.suptitle('Two-Day PAR Comparison: EvoEngine vs Analytical vs AZMET\n' +
             'June 12-13, 2024 - Maricopa, AZ',
             fontsize=14, fontweight='bold')

# June 12
ax1 = axes[0]
ax1.plot(june12['hour'], june12['PAR_EvoEngine'], 'o-', linewidth=2.5, markersize=8,
        color='#e74c3c', label='EvoEngine', zorder=3)
ax1.plot(june12['hour'], june12['PAR_Analytical'], 's--', linewidth=2, markersize=7,
        color='#3498db', label='Analytical', alpha=0.8, zorder=2)
ax1.plot(june12['hour'], june12['PAR_AZMET'], '^:', linewidth=2, markersize=7,
        color='#2ecc71', label='AZMET', alpha=0.8, zorder=1)
ax1.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
ax1.set_ylabel('PAR (µmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
ax1.set_title('June 12, 2024', fontsize=12, fontweight='bold')
ax1.legend(loc='upper left', fontsize=10)
ax1.grid(alpha=0.3)
ax1.set_xlim(5, 21)
ax1.set_ylim(0, 2300)

# June 13
ax2 = axes[1]
ax2.plot(june13['hour'], june13['PAR_EvoEngine'], 'o-', linewidth=2.5, markersize=8,
        color='#e74c3c', label='EvoEngine', zorder=3)
ax2.plot(june13['hour'], june13['PAR_Analytical'], 's--', linewidth=2, markersize=7,
        color='#3498db', label='Analytical', alpha=0.8, zorder=2)
ax2.plot(june13['hour'], june13['PAR_AZMET'], '^:', linewidth=2, markersize=7,
        color='#2ecc71', label='AZMET', alpha=0.8, zorder=1)
ax2.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
ax2.set_ylabel('PAR (µmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
ax2.set_title('June 13, 2024', fontsize=12, fontweight='bold')
ax2.legend(loc='upper left', fontsize=10)
ax2.grid(alpha=0.3)
ax2.set_xlim(5, 21)
ax2.set_ylim(0, 2300)

plt.tight_layout(rect=[0, 0, 1, 0.95])
filepath = OUTPUT_DIR / 'two_day_combined_comparison.png'
plt.savefig(filepath, dpi=300, bbox_inches='tight')
print(f"  Saved: {filepath}")
plt.close()

# =============================================================================
# Save Statistical Summary
# =============================================================================

print()
print("Saving statistical summary...")

stats_df = pd.DataFrame([stats_june12, stats_june13])
stats_filepath = OUTPUT_DIR / 'three_method_statistics.csv'
stats_df.to_csv(stats_filepath, index=False)
print(f"  Saved: {stats_filepath}")

# Save combined data
combined_filepath = OUTPUT_DIR / 'three_method_combined_data.csv'
combined.to_csv(combined_filepath, index=False)
print(f"  Saved: {combined_filepath}")

print()
print("="*80)
print("ANALYSIS COMPLETE")
print("="*80)
print(f"Output directory: {OUTPUT_DIR}")
print()
