#!/usr/bin/env python3
"""
Sorghum Leaf Angle Comparison: 0° (Horizontal) vs 30° (Tilted)

Compares PAR calculation for the SAME sorghum leaf at two different angles:
- 0° (horizontal) - maximum PAR exposure
- 30° (tilted) - typical vegetative stage angle

Uses EvoEngine ray tracing with SetEntityRotation API to control leaf angle.
Also includes analytical PAR calculation for validation.

Author: Claude Code (EvoEngine Expert)
Date: 2025-11-04
Location: Maricopa, AZ (33.07°N, -111.97°W)
Date simulated: June 13, 2024
"""

import os
from pathlib import Path
import sys
import numpy as np
import pandas as pd
from datetime import datetime, timedelta
from tqdm import tqdm
import matplotlib.pyplot as plt

# Solar position calculation
try:
    import pvlib
    from pvlib import solarposition
    print("✓ Using pvlib for solar position calculations")
except ImportError:
    print("ERROR: pvlib not available. Installing...")
    os.system("pip install pvlib")
    import pvlib
    from pvlib import solarposition

# =============================================================================
# Configuration
# =============================================================================

# Location: Maricopa, Arizona
LATITUDE = 33.07
LONGITUDE = -111.97
ELEVATION = 361  # meters
TIMEZONE = 'America/Phoenix'  # Arizona doesn't observe DST

# Simulation date and time range
SIMULATION_DATE = '2024-06-13'
START_HOUR = 6  # 6 AM
END_HOUR = 20   # 8 PM
HOUR_STEP = 1   # hourly

# Leaf geometry (single Stage 3 sorghum leaf)
LEAF_LENGTH_M = 0.50  # 50 cm
LEAF_WIDTH_M = 0.06   # 6 cm
LEAF_HEIGHT_M = 1.0   # 1 meter above ground

# Angles to test
ANGLES_TO_TEST = [0, 30]  # degrees from horizontal

# Output
OUTPUT_DIR = Path('./par_comparison_results')
OUTPUT_CSV_0DEG = 'par_horizontal_0deg.csv'
OUTPUT_CSV_30DEG = 'par_30deg.csv'
OUTPUT_COMPARISON = 'par_0deg_vs_30deg_comparison.csv'

# =============================================================================
# EvoEngine Path Configuration
# =============================================================================

EVOENGINE_DIRECTORY = Path('C:/Users/Brenda/code/EvoEngine')
BUILD_CONFIG = 'x64-Release'
LIBRARY_DIRECTORY = EVOENGINE_DIRECTORY / 'out' / 'build' / BUILD_CONFIG / 'PythonBinding'

ALTERNATIVE_LIBRARY_PATHS = [
    EVOENGINE_DIRECTORY / 'build' / 'PythonBinding',
    EVOENGINE_DIRECTORY / 'build' / 'Release' / 'PythonBinding',
    EVOENGINE_DIRECTORY / 'out' / 'build' / 'Release' / 'PythonBinding',
]

# =============================================================================
# Helper Functions
# =============================================================================

def find_library_directory():
    """Find the Python binding library directory"""
    if LIBRARY_DIRECTORY.exists():
        return LIBRARY_DIRECTORY

    print(f"⚠ Warning: Primary library path not found: {LIBRARY_DIRECTORY}")
    print("  Trying alternative locations...")

    for alt_path in ALTERNATIVE_LIBRARY_PATHS:
        if alt_path.exists():
            print(f"  ✓ Found at: {alt_path}")
            return alt_path

    print("\nERROR: Could not find Python bindings!")
    print("Searched locations:")
    print(f"  - {LIBRARY_DIRECTORY}")
    for alt in ALTERNATIVE_LIBRARY_PATHS:
        print(f"  - {alt}")
    sys.exit(1)

# =============================================================================
# Analytical PAR Calculation
# =============================================================================

def calculate_analytical_par(zenith_deg, leaf_angle_deg):
    """
    Calculate PAR using cosine law with atmospheric correction

    Args:
        zenith_deg: Solar zenith angle in degrees
        leaf_angle_deg: Leaf angle from horizontal in degrees

    Returns:
        PAR in μmol m⁻² s⁻¹
    """
    if 90 - zenith_deg <= 0:  # Sun below horizon
        return 0.0

    zenith_rad = np.radians(zenith_deg)
    cos_zenith = np.cos(zenith_rad)

    # Atmospheric transmittance (Kasten-Young formula)
    air_mass = 1.0 / (cos_zenith + 0.50572 * (96.07995 - zenith_deg)**(-1.6364))
    transmittance = 0.7**(air_mass**0.678)

    # Base intensity at extraterrestrial level
    base_intensity = 40000.0  # Same as EvoEngine

    # Direct beam on horizontal surface
    par_horizontal = base_intensity * transmittance * cos_zenith

    # For tilted surface, adjust by leaf angle
    # Incident angle = zenith - leaf_angle (simplified for comparison)
    incident_angle_rad = np.radians(abs(zenith_deg - leaf_angle_deg))
    cos_incident = np.cos(incident_angle_rad)

    # Apply tilt factor
    par_tilted = par_horizontal * (cos_incident / cos_zenith) if cos_zenith > 0 else 0

    # Convert to μmol m⁻² s⁻¹ (empirical factor)
    par_umol = par_tilted * 0.05

    return max(0, par_umol)

# =============================================================================
# EvoEngine Setup
# =============================================================================

print("="*80)
print("SORGHUM LEAF ANGLE COMPARISON - EvoEngine Ray Tracing")
print("="*80)
print(f"Location: {LATITUDE}°N, {LONGITUDE}°W (Maricopa, AZ)")
print(f"Date: {SIMULATION_DATE}")
print(f"Time range: {START_HOUR}:00 - {END_HOUR}:00 (hourly)")
print(f"Leaf: {LEAF_LENGTH_M*100:.0f} cm × {LEAF_WIDTH_M*100:.0f} cm")
print(f"Angles: {ANGLES_TO_TEST}° from horizontal")
print("="*80)
print()

# Find library
library_directory = find_library_directory()
print(f"EvoEngine directory: {EVOENGINE_DIRECTORY}")
print(f"Library directory: {library_directory}")

# Store current directory to restore later
current_directory = Path.cwd()

# Add to path and change directory (required for resource loading)
sys.path.append(str(library_directory))
os.chdir(library_directory)

try:
    import PyDigitalAgriculture as sorghum_framework
    print("✓ EvoEngine PyDigitalAgriculture loaded successfully")
except ImportError as e:
    print(f"ERROR: Could not load EvoEngine: {e}")
    print("Make sure EvoEngine is properly built and the library exists")
    print(f"Expected location: {library_directory}")
    sys.exit(1)

# Check API compatibility
print("\nChecking API compatibility...")
required_apis = [
    'IlluminationEstimationOnSorghum',
    'GetAllIlluminationEstimationResultsOnSorghum',
    'SetSunDirection',
    'SetEntityRotation',  # NEW API
]

missing_apis = [api for api in required_apis if not hasattr(sorghum_framework, api)]
if missing_apis:
    print("\nERROR: MISSING REQUIRED APIs:")
    for api in missing_apis:
        print(f"  ✗ {api}")
    if 'SetEntityRotation' in missing_apis:
        print("\n  ⚠ SetEntityRotation API not found!")
        print("  Please rebuild EvoEngine Python bindings with the new API.")
    sys.exit(1)

print("✓ All required APIs present")
print()

# =============================================================================
# Solar Position Calculations
# =============================================================================

def calculate_solar_position(date, hour, latitude, longitude, timezone):
    """Calculate solar position for given time and location"""
    dt = pd.Timestamp(f'{date} {hour:02d}:00:00', tz=timezone)
    solar_pos = solarposition.get_solarposition(dt, latitude, longitude)

    elevation = solar_pos['elevation'].values[0]
    azimuth = solar_pos['azimuth'].values[0]
    zenith = solar_pos['zenith'].values[0]

    return {
        'datetime': dt,
        'elevation_deg': elevation,
        'azimuth_deg': azimuth,
        'zenith_deg': zenith,
        'hour': hour
    }

def generate_sun_path(date, start_hour, end_hour, hour_step, latitude, longitude, timezone):
    """Generate solar positions for all timesteps"""
    sun_positions = []

    for hour in range(start_hour, end_hour + 1, hour_step):
        pos = calculate_solar_position(date, hour, latitude, longitude, timezone)
        sun_positions.append(pos)

    return pd.DataFrame(sun_positions)

# =============================================================================
# EvoEngine Scene Setup
# =============================================================================

def initialize_evoengine():
    """Initialize EvoEngine framework with ray tracing"""
    print("Initializing EvoEngine framework...")

    # Find project file
    project_paths = [
        library_directory / 'Resources' / 'DigitalAgricultureProject' / 'test.eveproj',
        EVOENGINE_DIRECTORY / 'Resources' / 'DigitalAgricultureProject' / 'test.eveproj',
        Path('./Resources/DigitalAgricultureProject/test.eveproj'),
    ]

    project_path = None
    for p in project_paths:
        if p.exists():
            project_path = p
            print(f"  Using project file: {p}")
            break

    if not project_path:
        print(f"ERROR: Project file not found!")
        print("Searched locations:")
        for p in project_paths:
            print(f"  - {p}")
        sys.exit(1)

    # Enable GPU ray tracing
    sorghum_framework.PushRayTracerLayer()
    sorghum_framework.RegisterClasses()
    sorghum_framework.PushSorghumLayer()
    sorghum_framework.PushRayTracerLayer()
    sorghum_framework.Run(str(project_path))

    print("✓ EvoEngine initialized with ray tracing enabled")

def setup_leaf_geometry():
    """Create single sorghum leaf geometry"""
    print("Setting up single sorghum leaf geometry...")

    # Enable BTF if available
    if hasattr(sorghum_framework, 'EnableBTF'):
        sorghum_framework.EnableBTF()

    # Configure mesh generation for single leaf
    data_gen_params = sorghum_framework.SorghumDataGenerationParameters()
    data_gen_params.generate_ground_mesh = False
    data_gen_params.avoid_occlusion = False

    # Leaf settings - single leaf only
    mesh_settings = data_gen_params.sorghum_mesh_generator_settings
    mesh_settings.enable_panicle = False
    mesh_settings.enable_stem = False
    mesh_settings.enable_leaves = True
    mesh_settings.enable_leaf_sheath = False
    mesh_settings.single_leaf_index = 2  # 3rd leaf
    mesh_settings.bottom_face = True
    mesh_settings.leaf_separated = True
    mesh_settings.leaf_thickness = 0.001

    # Load sorghum descriptor
    sorghum_paths = [
        "./SorghumGenerator/Sample1.sorghum",
        "./Resources/SorghumGenerator/Sample1.sorghum",
    ]

    sorghum_state_handle = None
    for p in sorghum_paths:
        try:
            sorghum_state_handle = sorghum_framework.GetAssetHandle(p)
            if sorghum_state_handle:
                print(f"  Using sorghum descriptor: {p}")
                break
        except:
            continue

    if not sorghum_state_handle:
        print("ERROR: Could not find sorghum descriptor!")
        sys.exit(1)

    sorghum_entity = sorghum_framework.CreateEntityFromSorghumDescriptor(sorghum_state_handle)

    # Generate mesh
    print("  Generating leaf mesh...")
    sorghum_framework.GenerateSorghumMesh(mesh_settings)
    print(f"  ✓ Single leaf mesh generated")

    return sorghum_entity

# =============================================================================
# Ray-Traced Illumination Calculation
# =============================================================================

def run_raytraced_illumination(sun_positions_df, sorghum_entity, leaf_angle_deg):
    """
    Run ray-traced illumination estimation for given leaf angle

    Args:
        sun_positions_df: DataFrame with solar positions
        sorghum_entity: Entity handle for the sorghum leaf
        leaf_angle_deg: Leaf angle from horizontal (0=horizontal, 30=tilted)

    Returns:
        DataFrame with PAR values from ray tracing
    """
    print()
    print(f"Running ray-traced illumination for {leaf_angle_deg}° leaf angle...")
    print(f"  Timesteps: {len(sun_positions_df)}")
    print(f"  Method: EvoEngine GPU ray tracing")

    # Set leaf angle using new API
    print(f"  Setting leaf rotation to {leaf_angle_deg}° (pitch angle)...")
    sorghum_framework.SetEntityRotation(sorghum_entity, leaf_angle_deg, 0, 0)
    print(f"  ✓ Leaf rotated to {leaf_angle_deg}° from horizontal")
    print()

    results = []

    pbar = tqdm(sun_positions_df.iterrows(), total=len(sun_positions_df),
                desc=f"Ray tracing {leaf_angle_deg}°", unit="hour")

    for idx, row in pbar:
        # Skip nighttime
        if row['elevation_deg'] <= 0:
            results.append({
                'PAR_mean': 0.0,
                'PAR_analytical': 0.0,
            })
            pbar.set_postfix(time=row['datetime'].strftime('%H:%M'),
                           zenith=f"{row['zenith_deg']:.1f}°",
                           PAR="Night")
            continue

        # Set sun direction
        sorghum_framework.SetSunDirection(row['azimuth_deg'], row['elevation_deg'])

        # Run ray-traced illumination
        sorghum_framework.IlluminationEstimationOnSorghum()

        # Get results
        result = sorghum_framework.GetAllIlluminationEstimationResultsOnSorghum()

        # Extract PAR
        par_value = 0.0
        for entity_result in result:
            if len(entity_result) >= 5:
                area = entity_result[2].x
                if area > 0:
                    par_value = entity_result[4].x
                    break

        # Calculate analytical PAR for comparison
        par_analytical = calculate_analytical_par(row['zenith_deg'], leaf_angle_deg)

        results.append({
            'PAR_mean': par_value,
            'PAR_analytical': par_analytical,
        })

        pbar.set_postfix(time=row['datetime'].strftime('%H:%M'),
                       zenith=f"{row['zenith_deg']:.1f}°",
                       PAR=f"{par_value:.0f}",
                       Ana=f"{par_analytical:.0f}")

    print()
    print(f"✓ Ray-traced illumination complete for {leaf_angle_deg}°")

    # Combine with sun position data
    results_df = pd.DataFrame(results)
    combined_df = pd.concat([sun_positions_df.reset_index(drop=True),
                            results_df.reset_index(drop=True)], axis=1)

    return combined_df

# =============================================================================
# Comparison and Visualization
# =============================================================================

def create_comparison_visualization(par_0deg, par_30deg, output_dir):
    """Create comprehensive comparison visualization"""
    fig, axes = plt.subplots(2, 3, figsize=(18, 11))

    # Panel A: Time series comparison - EvoEngine
    ax1 = axes[0, 0]
    ax1.plot(par_0deg['hour'], par_0deg['PAR_mean'],
             marker='o', linewidth=3, markersize=10, label='0° (Horizontal)', color='#E63946')
    ax1.plot(par_30deg['hour'], par_30deg['PAR_mean'],
             marker='s', linewidth=2.5, markersize=8, label='30° (Tilted)',
             color='#457B9D', linestyle='--')
    ax1.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
    ax1.set_ylabel('PAR (μmol photons m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax1.set_title('A. EvoEngine Ray Tracing Comparison', fontsize=12, fontweight='bold', loc='left')
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(alpha=0.3)
    ax1.set_xlim(5, 21)
    ax1.axhline(y=1500, color='red', linestyle=':', linewidth=2, alpha=0.5)

    # Panel B: Time series comparison - Analytical
    ax2 = axes[0, 1]
    ax2.plot(par_0deg['hour'], par_0deg['PAR_analytical'],
             marker='o', linewidth=3, markersize=10, label='0° (Analytical)', color='#E63946', alpha=0.7)
    ax2.plot(par_30deg['hour'], par_30deg['PAR_analytical'],
             marker='s', linewidth=2.5, markersize=8, label='30° (Analytical)',
             color='#457B9D', linestyle='--', alpha=0.7)
    ax2.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
    ax2.set_ylabel('PAR (μmol photons m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax2.set_title('B. Analytical Model Comparison', fontsize=12, fontweight='bold', loc='left')
    ax2.legend(loc='upper right', fontsize=10)
    ax2.grid(alpha=0.3)
    ax2.set_xlim(5, 21)

    # Panel C: Ratio over time
    ax3 = axes[0, 2]
    ratio = par_0deg['PAR_mean'] / par_30deg['PAR_mean'].replace(0, np.nan)
    ax3.plot(par_0deg['hour'], ratio,
             marker='o', linewidth=2.5, markersize=8, color='#2A9D8F')
    ax3.axhline(y=1.0, color='black', linestyle='--', linewidth=2, alpha=0.5)
    ax3.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
    ax3.set_ylabel('PAR Ratio (0° / 30°)', fontsize=11, fontweight='bold')
    ax3.set_title('C. Horizontal vs 30° Ratio', fontsize=12, fontweight='bold', loc='left')
    ax3.grid(alpha=0.3)
    ax3.set_xlim(5, 21)

    # Panel D: Cumulative PAR
    ax4 = axes[1, 0]
    cumulative_0 = np.cumsum(par_0deg['PAR_mean']) * 3600 / 1e6
    cumulative_30 = np.cumsum(par_30deg['PAR_mean']) * 3600 / 1e6
    ax4.plot(par_0deg['hour'], cumulative_0, 'o-', linewidth=2.5,
            markersize=7, color='#E63946', label='0° (Horizontal)')
    ax4.plot(par_30deg['hour'], cumulative_30, 's-', linewidth=2.5,
            markersize=7, color='#457B9D', label='30° (Tilted)', linestyle='--')
    ax4.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
    ax4.set_ylabel('Cumulative PAR (mol photons m⁻²)', fontsize=11, fontweight='bold')
    ax4.set_title('D. Daily PAR Accumulation', fontsize=12, fontweight='bold', loc='left')
    ax4.legend(loc='upper left', fontsize=10)
    ax4.grid(alpha=0.3)
    ax4.set_xlim(5, 21)

    # Panel E: EvoEngine vs Analytical - 0°
    ax5 = axes[1, 1]
    daylight_0 = par_0deg[par_0deg['elevation_deg'] > 0]
    ax5.plot(daylight_0['hour'], daylight_0['PAR_mean'],
             marker='o', linewidth=2.5, markersize=8, label='EvoEngine', color='#E63946')
    ax5.plot(daylight_0['hour'], daylight_0['PAR_analytical'],
             marker='s', linewidth=2, markersize=6, label='Analytical',
             color='#E63946', linestyle=':', alpha=0.7)
    ax5.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
    ax5.set_ylabel('PAR (μmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax5.set_title('E. Validation: 0° Horizontal', fontsize=12, fontweight='bold', loc='left')
    ax5.legend(loc='upper right', fontsize=10)
    ax5.grid(alpha=0.3)

    # Panel F: EvoEngine vs Analytical - 30°
    ax6 = axes[1, 2]
    daylight_30 = par_30deg[par_30deg['elevation_deg'] > 0]
    ax6.plot(daylight_30['hour'], daylight_30['PAR_mean'],
             marker='o', linewidth=2.5, markersize=8, label='EvoEngine', color='#457B9D')
    ax6.plot(daylight_30['hour'], daylight_30['PAR_analytical'],
             marker='s', linewidth=2, markersize=6, label='Analytical',
             color='#457B9D', linestyle=':', alpha=0.7)
    ax6.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
    ax6.set_ylabel('PAR (μmol m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax6.set_title('F. Validation: 30° Tilted', fontsize=12, fontweight='bold', loc='left')
    ax6.legend(loc='upper right', fontsize=10)
    ax6.grid(alpha=0.3)

    # Overall title
    fig.suptitle(f'Sorghum Leaf Angle Comparison: 0° vs 30°\n' +
                 f'Maricopa, AZ - {SIMULATION_DATE} - EvoEngine Ray Tracing',
                 fontsize=14, fontweight='bold', y=0.98)

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    # Save figure
    fig_path = output_dir / 'par_0deg_vs_30deg_comparison.png'
    plt.savefig(fig_path, dpi=300, bbox_inches='tight')
    print(f"✓ Saved visualization: {fig_path}")

    return fig

# =============================================================================
# Main Workflow
# =============================================================================

def main():
    """Main workflow for leaf angle comparison"""

    # Create output directories
    OUTPUT_DIR.mkdir(exist_ok=True)

    # Step 1: Calculate solar positions
    print("Step 1: Calculating solar positions...")
    sun_positions = generate_sun_path(SIMULATION_DATE, START_HOUR, END_HOUR, HOUR_STEP,
                                     LATITUDE, LONGITUDE, TIMEZONE)
    print(f"✓ Generated {len(sun_positions)} solar positions")
    daylight_hours = len(sun_positions[sun_positions['elevation_deg'] > 0])
    print(f"  Daylight hours: {daylight_hours} (elevation > 0°)")
    print()

    # Step 2: Initialize EvoEngine
    print("Step 2: Initializing EvoEngine...")
    initialize_evoengine()
    print()

    # Step 3: Setup leaf geometry
    print("Step 3: Setting up leaf geometry...")
    sorghum_entity = setup_leaf_geometry()
    print()

    # Step 4: Run ray-traced illumination for 0° (horizontal)
    print("Step 4: Running ray-traced illumination for 0° (horizontal)...")
    par_0deg = run_raytraced_illumination(sun_positions, sorghum_entity, 0)
    print(f"  Peak PAR: {par_0deg['PAR_mean'].max():.1f} μmol m⁻² s⁻¹")
    print()

    # Step 5: Run ray-traced illumination for 30° (tilted)
    print("Step 5: Running ray-traced illumination for 30° (tilted)...")
    par_30deg = run_raytraced_illumination(sun_positions, sorghum_entity, 30)
    print(f"  Peak PAR: {par_30deg['PAR_mean'].max():.1f} μmol m⁻² s⁻¹")
    print()

    # Step 6: Save results
    print("Step 6: Saving results...")
    par_0deg.to_csv(OUTPUT_DIR / OUTPUT_CSV_0DEG, index=False)
    print(f"✓ Saved: {OUTPUT_DIR / OUTPUT_CSV_0DEG}")
    par_30deg.to_csv(OUTPUT_DIR / OUTPUT_CSV_30DEG, index=False)
    print(f"✓ Saved: {OUTPUT_DIR / OUTPUT_CSV_30DEG}")

    # Create comparison dataframe
    comparison_df = pd.DataFrame({
        'hour': par_0deg['hour'],
        'elevation_deg': par_0deg['elevation_deg'],
        'zenith_deg': par_0deg['zenith_deg'],
        'PAR_0deg_evoengine': par_0deg['PAR_mean'],
        'PAR_30deg_evoengine': par_30deg['PAR_mean'],
        'PAR_0deg_analytical': par_0deg['PAR_analytical'],
        'PAR_30deg_analytical': par_30deg['PAR_analytical'],
        'PAR_diff_evoengine': par_0deg['PAR_mean'] - par_30deg['PAR_mean'],
        'PAR_ratio': par_0deg['PAR_mean'] / par_30deg['PAR_mean'].replace(0, np.nan),
    })
    comparison_df.to_csv(OUTPUT_DIR / OUTPUT_COMPARISON, index=False)
    print(f"✓ Saved: {OUTPUT_DIR / OUTPUT_COMPARISON}")
    print()

    # Step 7: Analysis
    print("Step 7: Analyzing results...")
    print("="*80)
    print("COMPARISON RESULTS")
    print("="*80)

    # Daily integrals
    daily_0deg = par_0deg['PAR_mean'].sum() * 3600 / 1e6
    daily_30deg = par_30deg['PAR_mean'].sum() * 3600 / 1e6

    print(f"\nDaily PAR Integrals (EvoEngine):")
    print(f"  0° (Horizontal):  {daily_0deg:.2f} mol m⁻² day⁻¹")
    print(f"  30° (Tilted):     {daily_30deg:.2f} mol m⁻² day⁻¹")
    print(f"  Difference:       {daily_0deg - daily_30deg:.2f} mol m⁻² day⁻¹ ({(daily_0deg/daily_30deg - 1)*100:+.1f}%)")

    # Peak values
    peak_0deg = par_0deg['PAR_mean'].max()
    peak_30deg = par_30deg['PAR_mean'].max()
    print(f"\nPeak PAR (EvoEngine):")
    print(f"  0° (Horizontal):  {peak_0deg:.1f} μmol m⁻² s⁻¹")
    print(f"  30° (Tilted):     {peak_30deg:.1f} μmol m⁻² s⁻¹")
    print(f"  Difference:       {peak_0deg - peak_30deg:.1f} μmol m⁻² s⁻¹ ({(peak_0deg/peak_30deg - 1)*100:+.1f}%)")

    print("="*80)
    print()

    # Step 8: Visualization
    print("Step 8: Creating visualization...")
    create_comparison_visualization(par_0deg, par_30deg, OUTPUT_DIR)
    print()

    # Step 9: Cleanup
    print("Step 9: Cleaning up...")
    sorghum_framework.Terminate()
    print("✓ EvoEngine terminated")

    # Restore original directory
    os.chdir(current_directory)
    print()

    # Summary
    print("="*80)
    print("SUMMARY")
    print("="*80)
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Horizontal (0°):  {daily_0deg:.2f} mol m⁻² day⁻¹ (peak: {peak_0deg:.0f} μmol m⁻² s⁻¹)")
    print(f"Tilted (30°):     {daily_30deg:.2f} mol m⁻² day⁻¹ (peak: {peak_30deg:.0f} μmol m⁻² s⁻¹)")
    print(f"Horizontal receives {(daily_0deg/daily_30deg - 1)*100:+.1f}% more PAR than 30° leaf")
    print("="*80)

    return par_0deg, par_30deg, comparison_df

if __name__ == '__main__':
    try:
        par_0deg, par_30deg, comparison = main()
    except Exception as e:
        print()
        print("="*80)
        print("ERROR OCCURRED")
        print("="*80)
        print(f"{e}")
        print()
        import traceback
        traceback.print_exc()
        print("="*80)

        # Try to cleanup
        try:
            import PyDigitalAgriculture as sorghum_framework
            sorghum_framework.Terminate()
        except:
            pass

        # Restore directory
        try:
            os.chdir(current_directory)
        except:
            pass

        sys.exit(1)
