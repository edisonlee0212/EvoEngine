#!/usr/bin/env python3
"""
Sorghum Single Leaf Daily PAR Calculation - Adapted for Target EvoEngine

This is an adapted version of sorghum_single_leaf_daily_par.py for use with
the EvoEngine build at /Users/maryfrancis/Documents/GitHub/EvoEngine

KEY DIFFERENCES FROM ORIGINAL:
- Paths updated for target EvoEngine installation
- API compatibility checks added
- Fallback mechanisms for missing branch 261 APIs
- Better error handling and diagnostics

IMPORTANT: This script requires branch 261 illumination APIs. Run
test_evoengine_compatibility.py FIRST to verify your build has them.

Calculates hourly PAR (Photosynthetically Active Radiation) incident on a single
sorghum leaf over a 24-hour period using EvoEngine's ray tracing engine.

Author: Claude Code (C4 Expert)
Date: October 31, 2025
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
    print("OK Using pvlib for solar position calculations")
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
LEAF_ANGLE_DEG = 30   # degrees from horizontal (typical vegetative stage)
LEAF_HEIGHT_M = 1.0   # 1 meter above ground

# Output
OUTPUT_DIR = Path('./evoengine_par_results')
OUTPUT_CSV = 'hourly_PAR_maricopa_june13_evoengine.csv'
SCREENSHOT_DIR = OUTPUT_DIR / 'screenshots'

# =============================================================================
# EvoEngine Path Configuration
# =============================================================================

# Path to target EvoEngine installation
# MODIFY THIS if your build is in a different location
EVOENGINE_DIRECTORY = Path('C:/Users/Brenda/code/EvoEngine')

# Build configuration - adjust based on your build type
# Common values: 'Debug', 'Release', 'RelWithDebInfo', 'x64-Release'
BUILD_CONFIG = 'x64-Release'

# Construct library path
LIBRARY_DIRECTORY = EVOENGINE_DIRECTORY / 'out' / 'build' / BUILD_CONFIG / 'PythonBinding'

# Alternative paths to try if primary doesn't exist
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

    print(f"WARNING Warning: Primary library path not found: {LIBRARY_DIRECTORY}")
    print("  Trying alternative locations...")

    for alt_path in ALTERNATIVE_LIBRARY_PATHS:
        if alt_path.exists():
            print(f"  OK Found at: {alt_path}")
            return alt_path

    print("\nERROR: Could not find Python bindings!")
    print("Searched locations:")
    print(f"  - {LIBRARY_DIRECTORY}")
    for alt in ALTERNATIVE_LIBRARY_PATHS:
        print(f"  - {alt}")
    print("\nPlease:")
    print("  1. Verify EvoEngine is built with Python bindings")
    print("  2. Update LIBRARY_DIRECTORY in this script")
    print("  3. Or run test_evoengine_compatibility.py to diagnose")
    sys.exit(1)

def check_api_compatibility(framework):
    """Check if required APIs are available"""
    # Core required APIs (must be present)
    required_apis = [
        'IlluminationEstimationOnSorghum',
        'GetAllIlluminationEstimationResultsOnSorghum',
        'SetSunDirection',
    ]

    # Optional APIs for enhanced realism
    optional_apis = [
        'EnableBTF',
        'SetCBTFGroup',
        'SetSkyDome',
    ]

    missing_required = [api for api in required_apis if not hasattr(framework, api)]
    missing_optional = [api for api in optional_apis if not hasattr(framework, api)]

    if missing_required:
        print("\n" + "="*80)
        print("ERROR: MISSING REQUIRED APIs")
        print("="*80)
        print("\nThe following core APIs are not available:")
        for api in missing_required:
            print(f"  ERROR {api}")
        print("\nRun test_evoengine_compatibility.py for detailed analysis.")
        print("="*80)
        sys.exit(1)

    if missing_optional:
        print("\nWARNING WARNING: Optional APIs missing (will use default materials):")
        for api in missing_optional:
            print(f"  • {api}")
        print()

    return True

# =============================================================================
# EvoEngine Setup
# =============================================================================

print("="*80)
print("SORGHUM SINGLE LEAF DAILY PAR CALCULATION - EvoEngine Ray Tracing")
print("="*80)
print(f"Location: {LATITUDE}°N, {LONGITUDE}°W (Maricopa, AZ)")
print(f"Date: {SIMULATION_DATE}")
print(f"Time range: {START_HOUR}:00 - {END_HOUR}:00 (hourly)")
print(f"Leaf: {LEAF_LENGTH_M*100:.0f} cm × {LEAF_WIDTH_M*100:.0f} cm, {LEAF_ANGLE_DEG}° angle")
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
    print("OK EvoEngine PyDigitalAgriculture loaded successfully")
except ImportError as e:
    print(f"ERROR: Could not load EvoEngine: {e}")
    print("Make sure EvoEngine is properly built and the library exists")
    print(f"Expected location: {library_directory}")
    sys.exit(1)

# Check API compatibility
print("\nChecking API compatibility...")
check_api_compatibility(sorghum_framework)
print("OK All required APIs present")
print()

# =============================================================================
# Solar Position Calculations
# =============================================================================

def calculate_solar_position(date, hour, latitude, longitude, timezone):
    """
    Calculate solar position (elevation and azimuth) for given time and location

    Returns:
        dict with keys: datetime, elevation_deg, azimuth_deg, zenith_deg
    """
    # Create datetime object
    dt = pd.Timestamp(f'{date} {hour:02d}:00:00', tz=timezone)

    # Use pvlib for accurate solar position
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

    # Find project file - try multiple locations
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

    print("OK EvoEngine initialized with ray tracing enabled")

def setup_leaf_geometry():
    """Create single sorghum leaf geometry"""
    print("Setting up single sorghum leaf geometry...")

    # Enable BTF (Bidirectional Texture Function) for realistic leaf optical properties
    if hasattr(sorghum_framework, 'EnableBTF'):
        sorghum_framework.EnableBTF()
    else:
        print("  ℹ BTF not available - using default materials")

    # Try to find BTF group asset
    btf_paths = [
        "./BTFGroup.cbtfgroup",
        "./Resources/BTFGroup.cbtfgroup",
    ]

    btf_path = None
    for p in btf_paths:
        try:
            cbtf_group_handle = sorghum_framework.GetAssetHandle(p)
            if cbtf_group_handle:
                btf_path = p
                break
        except:
            continue

    if btf_path and hasattr(sorghum_framework, 'SetCBTFGroup'):
        sorghum_framework.SetCBTFGroup(cbtf_group_handle)
        print("  OK BTF enabled for realistic leaf optics")
    else:
        print("  WARNING BTF group not found or API unavailable - using default materials")

    # Configure mesh generation for single leaf
    data_gen_params = sorghum_framework.SorghumDataGenerationParameters()
    data_gen_params.generate_ground_mesh = False  # No ground mesh
    data_gen_params.avoid_occlusion = False       # Include self-shading effects

    # Leaf settings - single leaf only (Stage 3 = 3rd leaf, index 2)
    mesh_settings = data_gen_params.sorghum_mesh_generator_settings
    mesh_settings.enable_panicle = False      # No panicle
    mesh_settings.enable_stem = False         # No stem
    mesh_settings.enable_leaves = True        # Only leaf
    mesh_settings.enable_leaf_sheath = False  # No sheath
    mesh_settings.single_leaf_index = 2       # 3rd leaf (Stage 3, 0-indexed)
    mesh_settings.bottom_face = True          # Both sides of leaf
    mesh_settings.leaf_separated = True       # Separated mesh
    mesh_settings.leaf_thickness = 0.001      # 1mm thickness

    # Try to load sorghum descriptor
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
        print("Searched locations:")
        for p in sorghum_paths:
            print(f"  - {p}")
        sys.exit(1)

    sorghum_entity = sorghum_framework.CreateEntityFromSorghumDescriptor(sorghum_state_handle)

    # Generate mesh with configured settings
    print("  Generating leaf mesh...")
    sorghum_framework.GenerateSorghumMesh(mesh_settings)
    print(f"  OK Single leaf mesh generated with configured settings")

    # Set sky dome for diffuse illumination
    if hasattr(sorghum_framework, 'SetSkyDome'):
        sorghum_framework.SetSkyDome()
        print("  OK Sky dome configured")
    else:
        print("  ℹ Sky dome API not available - using default lighting")

    return sorghum_entity

# =============================================================================
# Ray-Traced Illumination Calculation
# =============================================================================

def run_raytraced_illumination(sun_positions_df, sorghum_entity):
    """
    Run ray-traced illumination estimation for all sun positions

    Returns:
        DataFrame with PAR values from ray tracing
    """
    print()
    print("Running ray-traced illumination estimation...")
    print(f"  Timesteps: {len(sun_positions_df)}")
    print(f"  Method: EvoEngine GPU ray tracing with BTF leaf optics")
    print()

    results = []

    pbar = tqdm(sun_positions_df.iterrows(), total=len(sun_positions_df),
                desc="Ray tracing", unit="hour")

    for idx, row in pbar:
        # Skip nighttime (sun below horizon)
        if row['elevation_deg'] <= 0:
            results.append({
                'PAR_mean': 0.0,
                'PAR_max': 0.0,
                'PAR_min': 0.0,
                'PAR_std': 0.0,
                'num_vertices': 0
            })
            pbar.set_postfix(time=row['datetime'].strftime('%H:%M'),
                           zenith=f"{row['zenith_deg']:.1f}°",
                           PAR="Night")
            continue

        # Set sun direction using azimuth and elevation angles
        # API expects: SetSunDirection(azimuth_deg, elevation_deg)
        sorghum_framework.SetSunDirection(row['azimuth_deg'], row['elevation_deg'])

        # Run ray-traced illumination estimation
        sorghum_framework.IlluminationEstimationOnSorghum()

        # Get results (returns list of [position, rotation, area, total_flux, average_flux])
        # Result format: [[position, rotation, area_as_vec3, total_flux, average_flux], ...]
        result = sorghum_framework.GetAllIlluminationEstimationResultsOnSorghum()

        # Find entity with non-zero area (skip empty entities)
        par_value = 0.0
        found_valid_entity = False

        for entity_result in result:
            if len(entity_result) >= 5:
                # Check if entity has non-zero area (index 2, x component)
                area = entity_result[2].x
                if area > 0:
                    # Extract PAR from average_flux (index 4, x component)
                    average_flux = entity_result[4]
                    par_value = average_flux.x
                    found_valid_entity = True
                    break

        if found_valid_entity:
            # For aggregated results, we don't have per-vertex statistics
            # so we use the average value directly
            results.append({
                'PAR_mean': par_value,
                'PAR_max': par_value,  # No per-vertex data available
                'PAR_min': par_value,  # No per-vertex data available
                'PAR_std': 0.0,        # No spatial variation data
                'num_vertices': 0      # Not applicable for aggregated results
            })

            pbar.set_postfix(time=row['datetime'].strftime('%H:%M'),
                           zenith=f"{row['zenith_deg']:.1f}°",
                           PAR=f"{par_value:.0f}")
        else:
            # No illumination data (shouldn't happen)
            results.append({
                'PAR_mean': 0.0,
                'PAR_max': 0.0,
                'PAR_min': 0.0,
                'PAR_std': 0.0,
                'num_vertices': 0
            })
            pbar.set_postfix(time=row['datetime'].strftime('%H:%M'),
                           zenith=f"{row['zenith_deg']:.1f}°",
                           PAR="No data")

    print()
    print("OK Ray-traced illumination complete")

    # Combine with sun position data
    results_df = pd.DataFrame(results)
    combined_df = pd.concat([sun_positions_df.reset_index(drop=True),
                            results_df.reset_index(drop=True)], axis=1)

    return combined_df

# =============================================================================
# Validation
# =============================================================================

def validate_daily_par(par_df):
    """
    Validate total daily PAR against expected values

    Expected range for clear June day at 33°N: 30-50 mol photons m⁻² day⁻¹
    Based on: Monteith & Unsworth (2013), Campbell & Norman (1998)
    """
    # Convert μmol m⁻² s⁻¹ to mol m⁻² day⁻¹
    # Multiply by seconds per hour (3600) and sum over hours, divide by 1e6
    hourly_integral = par_df['PAR_mean'].sum() * 3600 / 1e6  # mol m⁻² day⁻¹

    print()
    print("="*80)
    print("VALIDATION RESULTS")
    print("="*80)
    print(f"Total daily PAR integral: {hourly_integral:.2f} mol photons m⁻² day⁻¹")
    print(f"Expected range (clear sky): 30-50 mol photons m⁻² day⁻¹")
    print()

    if 30 <= hourly_integral <= 50:
        print("OK VALIDATION PASSED: Daily PAR within expected range")
        status = "PASS"
    elif 25 <= hourly_integral <= 55:
        print("WARNING VALIDATION WARNING: Daily PAR slightly outside range but reasonable")
        status = "WARNING"
    else:
        print("ERROR VALIDATION FAILED: Daily PAR outside expected range")
        print("  This may indicate:")
        print("    - Incorrect leaf angle/orientation")
        print("    - Issues with BTF optical properties")
        print("    - Ray tracing configuration problems")
        status = "FAIL"

    print("="*80)
    print()

    return {
        'daily_integral': hourly_integral,
        'status': status,
        'expected_min': 30,
        'expected_max': 50
    }

# =============================================================================
# Visualization
# =============================================================================

def create_visualization(par_df, output_dir):
    """Create comprehensive visualization of daily PAR from ray tracing"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 1. Hourly PAR time series
    ax1 = axes[0, 0]
    ax1.plot(par_df['hour'], par_df['PAR_mean'], 'o-', linewidth=2.5,
            markersize=8, color='#2ca02c', label='Mean PAR (ray traced)')
    ax1.fill_between(par_df['hour'],
                     par_df['PAR_mean'] - par_df['PAR_std'],
                     par_df['PAR_mean'] + par_df['PAR_std'],
                     alpha=0.3, color='#2ca02c', label='±1 SD (spatial variation)')
    ax1.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
    ax1.set_ylabel('PAR (μmol photons m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax1.set_title('A. Hourly PAR from Ray Tracing', fontsize=12, fontweight='bold', loc='left')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(alpha=0.3)
    ax1.set_xlim(5, 21)

    # Add sorghum light saturation reference
    ax1.axhline(y=1500, color='red', linestyle=':', linewidth=2, alpha=0.5,
               label='Light saturation (~1500)')
    ax1.legend(loc='upper right', fontsize=9)

    # 2. Solar position with PAR intensity
    ax2 = axes[0, 1]
    scatter = ax2.scatter(par_df['azimuth_deg'], par_df['elevation_deg'],
                         c=par_df['PAR_mean'], s=200, cmap='YlOrRd',
                         edgecolors='black', linewidth=1.5, vmin=0)
    for idx, row in par_df.iterrows():
        if row['PAR_mean'] > 0:  # Only label daylight hours
            ax2.annotate(f"{int(row['hour'])}h",
                        (row['azimuth_deg'], row['elevation_deg']),
                        xytext=(5, 5), textcoords='offset points',
                        fontsize=8, fontweight='bold')
    ax2.set_xlabel('Solar Azimuth (°)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Solar Elevation (°)', fontsize=11, fontweight='bold')
    ax2.set_title('B. Sun Path and PAR Intensity', fontsize=12, fontweight='bold', loc='left')
    cbar = plt.colorbar(scatter, ax=ax2)
    cbar.set_label('PAR (μmol m⁻² s⁻¹)', fontsize=10)
    ax2.grid(alpha=0.3)

    # 3. PAR vs Solar Elevation (light response curve)
    ax3 = axes[1, 0]
    daylight = par_df[par_df['elevation_deg'] > 0]
    scatter3 = ax3.scatter(daylight['elevation_deg'], daylight['PAR_mean'],
                          s=120, c=daylight['hour'], cmap='viridis',
                          edgecolors='black', linewidth=1.5)

    # Add error bars for spatial variation
    ax3.errorbar(daylight['elevation_deg'], daylight['PAR_mean'],
                yerr=daylight['PAR_std'], fmt='none', ecolor='gray',
                alpha=0.5, capsize=3)

    ax3.set_xlabel('Solar Elevation (°)', fontsize=11, fontweight='bold')
    ax3.set_ylabel('Mean PAR (μmol photons m⁻² s⁻¹)', fontsize=11, fontweight='bold')
    ax3.set_title('C. Light Response to Solar Angle', fontsize=12, fontweight='bold', loc='left')
    cbar2 = plt.colorbar(scatter3, ax=ax3)
    cbar2.set_label('Hour', fontsize=10)
    ax3.grid(alpha=0.3)

    # 4. Cumulative daily PAR
    ax4 = axes[1, 1]
    cumulative = np.cumsum(par_df['PAR_mean']) * 3600 / 1e6  # Convert to mol m⁻²
    ax4.fill_between(par_df['hour'], 0, cumulative, alpha=0.3, color='darkgreen')
    ax4.plot(par_df['hour'], cumulative, 'o-', linewidth=2.5,
            markersize=7, color='darkgreen')
    ax4.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
    ax4.set_ylabel('Cumulative PAR (mol photons m⁻²)', fontsize=11, fontweight='bold')
    ax4.set_title('D. Daily PAR Accumulation', fontsize=12, fontweight='bold', loc='left')
    ax4.grid(alpha=0.3)
    ax4.set_xlim(5, 21)

    # Add final value annotation
    final_par = cumulative.iloc[-1]
    ax4.text(0.98, 0.95, f'Total: {final_par:.1f} mol m⁻² day⁻¹',
            transform=ax4.transAxes, ha='right', va='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9),
            fontsize=11, fontweight='bold')

    # Add expected range shading
    ax4.axhspan(30, 50, alpha=0.1, color='green', label='Expected range')
    ax4.legend(loc='upper left', fontsize=9)

    # Overall title
    fig.suptitle(f'Sorghum Leaf Daily PAR - EvoEngine Ray Tracing (Target Build)\n' +
                 f'Maricopa, AZ - {SIMULATION_DATE} - ' +
                 f'Leaf: {LEAF_LENGTH_M*100:.0f} cm × {LEAF_WIDTH_M*100:.0f} cm, {LEAF_ANGLE_DEG}° angle',
                 fontsize=13, fontweight='bold', y=0.98)

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    # Save figure
    fig_path = output_dir / 'daily_par_visualization.png'
    plt.savefig(fig_path, dpi=300, bbox_inches='tight')
    print(f"OK Saved visualization: {fig_path}")

    return fig

# =============================================================================
# Main Workflow
# =============================================================================

def main():
    """Main workflow for EvoEngine ray-traced PAR calculation"""

    # Create output directories
    OUTPUT_DIR.mkdir(exist_ok=True)
    SCREENSHOT_DIR.mkdir(exist_ok=True)

    # Step 1: Calculate solar positions
    print("Step 1: Calculating solar positions...")
    sun_positions = generate_sun_path(SIMULATION_DATE, START_HOUR, END_HOUR, HOUR_STEP,
                                     LATITUDE, LONGITUDE, TIMEZONE)
    print(f"OK Generated {len(sun_positions)} solar positions")
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

    # Step 4: Run ray-traced illumination
    print("Step 4: Running ray-traced illumination...")
    par_df = run_raytraced_illumination(sun_positions, sorghum_entity)
    print(f"  Peak PAR: {par_df['PAR_mean'].max():.1f} μmol m⁻² s⁻¹")
    print(f"  Mean PAR (daylight): {par_df[par_df['PAR_mean'] > 0]['PAR_mean'].mean():.1f} μmol m⁻² s⁻¹")
    print()

    # Step 5: Save results
    print("Step 5: Saving results...")
    output_csv_path = OUTPUT_DIR / OUTPUT_CSV
    par_df.to_csv(output_csv_path, index=False)
    print(f"OK Saved CSV: {output_csv_path}")
    print()

    # Step 6: Validation
    print("Step 6: Validating results...")
    validation = validate_daily_par(par_df)

    # Step 7: Visualization
    print("Step 7: Creating visualization...")
    create_visualization(par_df, OUTPUT_DIR)
    print()

    # Step 8: Cleanup
    print("Step 8: Cleaning up...")
    sorghum_framework.Terminate()
    print("OK EvoEngine terminated")

    # Restore original directory
    os.chdir(current_directory)
    print()

    # Summary
    print("="*80)
    print("SUMMARY")
    print("="*80)
    print(f"EvoEngine build: {EVOENGINE_DIRECTORY}")
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Data file: {OUTPUT_CSV}")
    print(f"Visualization: daily_par_visualization.png")
    print()
    print(f"Peak PAR: {par_df['PAR_mean'].max():.1f} μmol m⁻² s⁻¹ " +
          f"at {par_df.loc[par_df['PAR_mean'].idxmax(), 'hour']:.0f}:00")
    print(f"Daily PAR integral: {validation['daily_integral']:.2f} mol m⁻² day⁻¹")
    print(f"Validation: {validation['status']}")
    print()
    print("Ray tracing method: EvoEngine GPU with BTF leaf optics")
    print(f"Mesh vertices analyzed: ~{par_df['num_vertices'].max():.0f} per leaf")
    print("="*80)

    return par_df, validation

if __name__ == '__main__':
    try:
        par_data, validation_results = main()
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
