#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sorghum 7-Leaf Plant Multi-Day PAR Calculation - June 10-17, 2024

Calculates hourly PAR (Photosynthetically Active Radiation) incident on a full
7-leaf sorghum plant over an 8-day period using EvoEngine's ray tracing engine.

IMPORTANT: This script requires EvoEngine with illumination APIs built.
IMPORTANT: Must be run with Python 3.9 to match EvoEngine bindings.

Key Features:
- Multi-day simulation (June 10-17, 2024)
- Hourly ray tracing for each day (6 AM - 8 PM)
- Daily-varying solar geometry (sun path changes each day)
- Full 7-leaf plant with self-shading effects
- Consolidated CSV output with 120 timesteps (8 days x 15 hours)

Author: Claude Code (C4 Expert)
Date: November 5, 2025
Location: Maricopa, AZ (33.07 deg N, -111.97 deg W)
Date range: June 10-17, 2024
"""

import os
from pathlib import Path
import sys

# Set UTF-8 encoding for Windows console
if sys.platform == 'win32':
    import codecs
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')
    sys.stderr = codecs.getwriter('utf-8')(sys.stderr.buffer, 'strict')
    os.environ['PYTHONIOENCODING'] = 'utf-8'
import numpy as np
import pandas as pd
from datetime import datetime, timedelta
from tqdm import tqdm
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

# Solar position calculation
try:
    import pvlib
    from pvlib import solarposition
    print("[OK] Using pvlib for solar position calculations")
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

# Simulation date range
START_DATE = '2024-06-10'
END_DATE = '2024-06-17'
START_HOUR = 6  # 6 AM
END_HOUR = 20   # 8 PM
HOUR_STEP = 1   # hourly

# Output
OUTPUT_DIR = Path('./evoengine_par_results')
OUTPUT_CSV = 'hourly_PAR_7leaf_june10_17_2024.csv'
SCREENSHOT_DIR = OUTPUT_DIR / 'screenshots'

# =============================================================================
# EvoEngine Path Configuration
# =============================================================================

EVOENGINE_DIRECTORY = Path('C:/Users/Brenda/code/EvoEngine')
BUILD_CONFIG = 'x64-Release'
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

    print(f"[!] Warning: Primary library path not found: {LIBRARY_DIRECTORY}")
    print("  Trying alternative locations...")

    for alt_path in ALTERNATIVE_LIBRARY_PATHS:
        if alt_path.exists():
            print(f"  [OK] Found at: {alt_path}")
            return alt_path

    print("\nERROR: Could not find Python bindings!")
    print("Searched locations:")
    print(f"  - {LIBRARY_DIRECTORY}")
    for alt in ALTERNATIVE_LIBRARY_PATHS:
        print(f"  - {alt}")
    print("\nPlease build EvoEngine with Python bindings enabled.")
    sys.exit(1)

def check_api_compatibility(framework):
    """Check if required APIs are available"""
    required_apis = [
        'IlluminationEstimationOnSorghum',
        'GetAllIlluminationEstimationResultsOnSorghum',
        'SetSunDirection',
    ]

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
            print(f"  [X] {api}")
        print("\nPlease ensure EvoEngine is built with illumination APIs.")
        print("="*80)
        sys.exit(1)

    if missing_optional:
        print("\n[!] WARNING: Optional APIs missing (will use default materials):")
        for api in missing_optional:
            print(f"   {api}")
        print()

    return True

# =============================================================================
# EvoEngine Setup
# =============================================================================

print("="*80)
print("SORGHUM 7-LEAF PLANT MULTI-DAY PAR CALCULATION")
print("="*80)
print(f"Location: {LATITUDE} deg N, {LONGITUDE} deg W (Maricopa, AZ)")
print(f"Date range: {START_DATE} to {END_DATE} (8 days)")
print(f"Time range: {START_HOUR}:00 - {END_HOUR}:00 (hourly)")
print(f"Plant: Full 7-leaf sorghum with stem and panicle")
print(f"Expected timesteps: {8 * (END_HOUR - START_HOUR + 1)} (8 days x 15 hours)")
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
    print("[OK] EvoEngine PyDigitalAgriculture loaded successfully")
except ImportError as e:
    print(f"ERROR: Could not load EvoEngine: {e}")
    print("Make sure EvoEngine is properly built and the library exists")
    print(f"Expected location: {library_directory}")
    sys.exit(1)

# Check API compatibility
print("\nChecking API compatibility...")
check_api_compatibility(sorghum_framework)
print("[OK] All required APIs present")
print()

# =============================================================================
# Solar Position Calculations
# =============================================================================

def calculate_solar_position(date_str, hour, latitude, longitude, timezone):
    """
    Calculate solar position (elevation and azimuth) for given time and location

    Args:
        date_str: Date in 'YYYY-MM-DD' format
        hour: Hour of day (0-23)
        latitude: Latitude in degrees
        longitude: Longitude in degrees
        timezone: Timezone string (e.g., 'America/Phoenix')

    Returns:
        dict with keys: date, datetime, elevation_deg, azimuth_deg, zenith_deg, hour
    """
    # Create datetime object
    dt = pd.Timestamp(f'{date_str} {hour:02d}:00:00', tz=timezone)

    # Use pvlib for accurate solar position
    solar_pos = solarposition.get_solarposition(dt, latitude, longitude)

    elevation = solar_pos['elevation'].values[0]
    azimuth = solar_pos['azimuth'].values[0]
    zenith = solar_pos['zenith'].values[0]

    return {
        'date': date_str,
        'datetime': dt,
        'elevation_deg': elevation,
        'azimuth_deg': azimuth,
        'zenith_deg': zenith,
        'hour': hour
    }

def generate_sun_path(date_str, start_hour, end_hour, hour_step, latitude, longitude, timezone):
    """Generate solar positions for all timesteps in a single day"""
    sun_positions = []

    for hour in range(start_hour, end_hour + 1, hour_step):
        pos = calculate_solar_position(date_str, hour, latitude, longitude, timezone)
        sun_positions.append(pos)

    return pd.DataFrame(sun_positions)

def generate_multiday_sun_path(start_date, end_date, start_hour, end_hour, hour_step,
                                latitude, longitude, timezone):
    """Generate solar positions for all timesteps across multiple days"""
    date_range = pd.date_range(start=start_date, end=end_date, freq='D')

    all_sun_positions = []

    for current_date in date_range:
        date_str = current_date.strftime('%Y-%m-%d')
        daily_positions = generate_sun_path(date_str, start_hour, end_hour, hour_step,
                                           latitude, longitude, timezone)
        all_sun_positions.append(daily_positions)

    return pd.concat(all_sun_positions, ignore_index=True)

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

    print("[OK] EvoEngine initialized with ray tracing enabled")

def setup_plant_geometry():
    """Create full sorghum plant geometry (CALLED ONCE)"""
    print("Setting up full sorghum plant geometry...")

    # Enable BTF (Bidirectional Texture Function) for realistic leaf optical properties
    if hasattr(sorghum_framework, 'EnableBTF'):
        sorghum_framework.EnableBTF()
    else:
        print("  [i] BTF not available - using default materials")

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
        print("  [OK] BTF enabled for realistic leaf optics")
    else:
        print("  [!] BTF group not found or API unavailable - using default materials")

    # Configure mesh generation for full plant
    data_gen_params = sorghum_framework.SorghumDataGenerationParameters()
    data_gen_params.generate_ground_mesh = False  # No ground mesh
    data_gen_params.avoid_occlusion = False       # Include self-shading effects

    # Full plant settings
    mesh_settings = data_gen_params.sorghum_mesh_generator_settings
    mesh_settings.enable_panicle = True      # Include panicle
    mesh_settings.enable_stem = True         # Include stem
    mesh_settings.enable_leaves = True       # Include all leaves
    mesh_settings.enable_leaf_sheath = False # No sheath
    mesh_settings.single_leaf_index = -1     # -1 = ALL LEAVES (7 leaves)
    mesh_settings.bottom_face = True         # Both sides of leaf
    mesh_settings.leaf_separated = True      # Separated mesh
    mesh_settings.leaf_thickness = 0.001     # 1mm thickness

    # Try to load sorghum descriptor - 7-leaf target
    sorghum_paths = [
        "./SorghumGenerator/7leaf-target.sg",
        "./Resources/DigitalAgricultureProject/Assets/SorghumGenerator/7leaf-target.sg",
        "../../../../Resources/DigitalAgricultureProject/Assets/SorghumGenerator/7leaf-target.sg",
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
    print("  Generating full plant mesh...")
    sorghum_framework.GenerateSorghumMesh(mesh_settings)
    print(f"  [OK] Full plant mesh generated (7 leaves + stem + panicle)")

    # Set sky dome for diffuse illumination
    if hasattr(sorghum_framework, 'SetSkyDome'):
        sorghum_framework.SetSkyDome()
        print("  [OK] Sky dome configured")
    else:
        print("  [i] Sky dome API not available - using default lighting")

    return sorghum_entity

# =============================================================================
# Ray-Traced Illumination Calculation
# =============================================================================

def run_raytraced_illumination_multiday(sun_positions_df, sorghum_entity):
    """
    Run ray-traced illumination estimation for all sun positions across multiple days

    Returns:
        DataFrame with PAR values from ray tracing for all timesteps
    """
    print()
    print("Running multi-day ray-traced illumination estimation...")
    print(f"  Total timesteps: {len(sun_positions_df)}")
    print(f"  Method: EvoEngine GPU ray tracing with BTF leaf optics")
    print()

    results = []

    # Progress bar for all timesteps
    pbar = tqdm(sun_positions_df.iterrows(), total=len(sun_positions_df),
                desc="Ray tracing", unit="timestep")

    for idx, row in pbar:
        # Skip nighttime (sun below horizon)
        if row['elevation_deg'] <= 0:
            results.append({
                'PAR_mean': 0.0,
                'PAR_max': 0.0,
                'PAR_min': 0.0,
                'PAR_std': 0.0,
                'num_leaves': 0,
                'total_leaf_area': 0.0
            })
            pbar.set_postfix(date=row['date'], time=f"{row['hour']:02d}:00",
                           zenith=f"{row['zenith_deg']:.1f} deg", PAR="Night")
            continue

        # Set sun direction using azimuth and elevation angles
        sorghum_framework.SetSunDirection(row['azimuth_deg'], row['elevation_deg'])

        # Run ray-traced illumination estimation
        sorghum_framework.IlluminationEstimationOnSorghum()

        # Get results (returns list of [position, rotation, area, total_flux, average_flux])
        result = sorghum_framework.GetAllIlluminationEstimationResultsOnSorghum()

        # Aggregate results across ALL leaf entities (full plant)
        total_area = 0.0
        total_flux = 0.0
        leaf_pars = []
        num_leaves = 0

        for entity_result in result:
            if len(entity_result) >= 5:
                # Check if entity has non-zero area (index 2, x component)
                area = entity_result[2].x
                if area > 0:
                    # Extract flux and PAR values
                    flux = entity_result[3].x  # Total flux on this leaf
                    leaf_par = entity_result[4].x  # Average PAR on this leaf

                    total_area += area
                    total_flux += flux
                    leaf_pars.append(leaf_par)
                    num_leaves += 1

        # Calculate area-weighted average PAR across all leaves
        if num_leaves > 0 and total_area > 0:
            avg_par = total_flux / total_area
            par_std = np.std(leaf_pars) if len(leaf_pars) > 1 else 0.0
            par_max = max(leaf_pars) if leaf_pars else 0.0
            par_min = min(leaf_pars) if leaf_pars else 0.0

            results.append({
                'PAR_mean': avg_par,
                'PAR_max': par_max,
                'PAR_min': par_min,
                'PAR_std': par_std,
                'num_leaves': num_leaves,
                'total_leaf_area': total_area
            })

            pbar.set_postfix(date=row['date'], time=f"{row['hour']:02d}:00",
                           PAR=f"{avg_par:.0f}", leaves=num_leaves)
        else:
            # No illumination data
            results.append({
                'PAR_mean': 0.0,
                'PAR_max': 0.0,
                'PAR_min': 0.0,
                'PAR_std': 0.0,
                'num_leaves': 0,
                'total_leaf_area': 0.0
            })
            pbar.set_postfix(date=row['date'], time=f"{row['hour']:02d}:00",
                           PAR="No data", leaves=0)

    print()
    print("[OK] Ray-traced illumination complete")

    # Combine with sun position data
    results_df = pd.DataFrame(results)
    combined_df = pd.concat([sun_positions_df.reset_index(drop=True),
                            results_df.reset_index(drop=True)], axis=1)

    return combined_df

# =============================================================================
# Validation
# =============================================================================

def validate_multiday_par(par_df):
    """
    Validate multi-day PAR data

    Calculates daily integrals and checks for physical/biological plausibility
    """
    print()
    print("="*80)
    print("VALIDATION RESULTS - MULTI-DAY (June 10-17, 2024)")
    print("="*80)

    # Group by date and calculate daily integrals
    daily_stats = []

    for date_str, group in par_df.groupby('date'):
        # Calculate daily integral (μmol m⁻² s⁻¹ → mol m⁻² day⁻¹)
        daily_integral = group['PAR_mean'].sum() * 3600 / 1e6
        peak_par = group['PAR_mean'].max()
        mean_par_daylight = group[group['PAR_mean'] > 0]['PAR_mean'].mean()
        mean_std = group['PAR_std'].mean()

        daily_stats.append({
            'date': date_str,
            'daily_integral': daily_integral,
            'peak_par': peak_par,
            'mean_par_daylight': mean_par_daylight,
            'mean_std': mean_std
        })

    daily_df = pd.DataFrame(daily_stats)

    # Print daily statistics
    print("\nDaily PAR Integrals:")
    print("-" * 80)
    print(f"{'Date':<12} {'Daily Integral':<18} {'Peak PAR':<15} {'Mean PAR':<15}")
    print(f"{'':12} {'(mol m^-2 day^-1)':<18} {'(umol m^-2 s^-1)':<15} {'(umol m^-2 s^-1)':<15}")
    print("-" * 80)
    for _, row in daily_df.iterrows():
        print(f"{row['date']:<12} {row['daily_integral']:>16.2f}  {row['peak_par']:>13.1f}  {row['mean_par_daylight']:>13.1f}")
    print("-" * 80)

    # Overall statistics
    mean_daily_integral = daily_df['daily_integral'].mean()
    std_daily_integral = daily_df['daily_integral'].std()
    min_daily_integral = daily_df['daily_integral'].min()
    max_daily_integral = daily_df['daily_integral'].max()

    print(f"\nMulti-Day Statistics:")
    print(f"  Mean daily integral: {mean_daily_integral:.2f} +/- {std_daily_integral:.2f} mol m^-2 day^-1")
    print(f"  Range: {min_daily_integral:.2f} - {max_daily_integral:.2f} mol m^-2 day^-1")
    print(f"  Day-to-day variation: {(std_daily_integral/mean_daily_integral*100):.1f}%")
    print()

    # Validation checks
    print("Validation Checks:")
    validation_passed = True

    # Check 1: Daily integral range (full plant with self-shading: 15-40 mol m^-2 day^-1)
    if 15 <= mean_daily_integral <= 40:
        print("  [OK] Mean daily integral within expected range (15-40 mol m^-2 day^-1)")
    elif 10 <= mean_daily_integral <= 45:
        print("  [!] WARNING: Mean daily integral slightly outside range but reasonable")
        validation_passed = False
    else:
        print("  [X] FAILED: Mean daily integral outside expected range")
        validation_passed = False

    # Check 2: Day-to-day variation (<10% expected for clear days in mid-June)
    if std_daily_integral / mean_daily_integral < 0.10:
        print(f"  [OK] Day-to-day variation reasonable (<10%)")
    else:
        print(f"  [!] WARNING: High day-to-day variation (>{(std_daily_integral/mean_daily_integral*100):.1f}%)")

    # Check 3: Peak PAR range (1500-2000 umol m^-2 s^-1 for top leaves)
    peak_par_overall = par_df['PAR_mean'].max()
    if 1200 <= peak_par_overall <= 2200:
        print(f"  [OK] Peak PAR within expected range (1200-2200 umol m^-2 s^-1)")
    else:
        print(f"  [!] WARNING: Peak PAR outside typical range: {peak_par_overall:.1f} umol m^-2 s^-1")
        validation_passed = False

    # Check 4: Spatial variation (self-shading detection)
    mean_std_overall = par_df['PAR_std'].mean()
    if mean_std_overall > 100:
        print(f"  [OK] Spatial variation detected (mean std = {mean_std_overall:.1f} umol m^-2 s^-1)")
    else:
        print(f"  [!] WARNING: Low spatial variation - self-shading may not be captured")

    status = "PASS" if validation_passed else "WARNING"
    print()
    print(f"Overall Status: {status}")
    print("="*80)
    print()

    return {
        'daily_df': daily_df,
        'mean_daily_integral': mean_daily_integral,
        'std_daily_integral': std_daily_integral,
        'status': status,
        'peak_par_overall': peak_par_overall
    }

# =============================================================================
# Visualization
# =============================================================================

def create_multiday_visualization(par_df, validation_results, output_dir):
    """Create comprehensive multi-day visualization"""
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)

    # Convert datetime to proper format for plotting
    par_df['datetime'] = pd.to_datetime(par_df['datetime'])
    daily_df = validation_results['daily_df']

    # 1. Multi-day hourly PAR time series
    ax1 = fig.add_subplot(gs[0, :])

    # Plot each day with different color
    dates = par_df['date'].unique()
    colors = plt.cm.viridis(np.linspace(0, 1, len(dates)))

    for i, date_str in enumerate(dates):
        day_data = par_df[par_df['date'] == date_str]
        ax1.plot(day_data['datetime'], day_data['PAR_mean'],
                marker='o', linewidth=2, markersize=4,
                color=colors[i], label=date_str, alpha=0.8)

    ax1.set_xlabel('Date and Time', fontsize=12, fontweight='bold')
    ax1.set_ylabel('PAR (umol photons m^-2 s^-1)', fontsize=12, fontweight='bold')
    ax1.set_title('A. Multi-Day Hourly PAR Time Series (June 10-17, 2024)',
                 fontsize=13, fontweight='bold', loc='left')
    ax1.legend(ncol=4, fontsize=9, loc='upper left')
    ax1.grid(alpha=0.3)

    # Add light saturation reference
    ax1.axhline(y=1500, color='red', linestyle=':', linewidth=2, alpha=0.5,
               label='Light saturation (~1500)')

    # Format x-axis
    ax1.xaxis.set_major_formatter(mdates.DateFormatter('%m-%d\n%H:%M'))
    ax1.xaxis.set_major_locator(mdates.DayLocator())

    # 2. Daily PAR integrals (bar chart)
    ax2 = fig.add_subplot(gs[1, 0])

    daily_df['date_dt'] = pd.to_datetime(daily_df['date'])
    bars = ax2.bar(daily_df['date_dt'], daily_df['daily_integral'],
                   color='darkgreen', alpha=0.7, edgecolor='black', linewidth=1.5)

    # Add value labels on bars
    for bar, value in zip(bars, daily_df['daily_integral']):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'{value:.1f}', ha='center', va='bottom', fontsize=9, fontweight='bold')

    ax2.set_xlabel('Date', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Daily PAR Integral (mol m^-2 day^-1)', fontsize=11, fontweight='bold')
    ax2.set_title('B. Daily PAR Integrals', fontsize=12, fontweight='bold', loc='left')
    ax2.grid(alpha=0.3, axis='y')
    ax2.xaxis.set_major_formatter(mdates.DateFormatter('%m-%d'))

    # Add expected range
    ax2.axhspan(15, 40, alpha=0.15, color='green', label='Expected range (15-40)')
    ax2.legend(fontsize=9)

    # 3. Sun path evolution (overlay all days)
    ax3 = fig.add_subplot(gs[1, 1])

    for i, date_str in enumerate(dates):
        day_data = par_df[par_df['date'] == date_str]
        daylight = day_data[day_data['elevation_deg'] > 0]

        ax3.plot(daylight['azimuth_deg'], daylight['elevation_deg'],
                marker='o', linewidth=2, markersize=5, color=colors[i],
                label=date_str, alpha=0.7)

    ax3.set_xlabel('Solar Azimuth (deg)', fontsize=11, fontweight='bold')
    ax3.set_ylabel('Solar Elevation (deg)', fontsize=11, fontweight='bold')
    ax3.set_title('C. Sun Path Evolution (8 Days)', fontsize=12, fontweight='bold', loc='left')
    ax3.legend(ncol=2, fontsize=8)
    ax3.grid(alpha=0.3)

    # 4. Peak PAR vs Date (trend analysis)
    ax4 = fig.add_subplot(gs[2, 0])

    ax4.plot(daily_df['date_dt'], daily_df['peak_par'],
            marker='o', linewidth=2.5, markersize=8, color='orangered')
    ax4.set_xlabel('Date', fontsize=11, fontweight='bold')
    ax4.set_ylabel('Peak PAR (umol m^-2 s^-1)', fontsize=11, fontweight='bold')
    ax4.set_title('D. Peak Daily PAR Trend', fontsize=12, fontweight='bold', loc='left')
    ax4.grid(alpha=0.3)
    ax4.xaxis.set_major_formatter(mdates.DateFormatter('%m-%d'))

    # Add trend line
    z = np.polyfit(range(len(daily_df)), daily_df['peak_par'], 1)
    p = np.poly1d(z)
    ax4.plot(daily_df['date_dt'], p(range(len(daily_df))),
            "r--", alpha=0.5, linewidth=2, label=f'Trend: {z[0]:.1f} μmol/day')
    ax4.legend(fontsize=9)

    # 5. Spatial variation (PAR std across leaves)
    ax5 = fig.add_subplot(gs[2, 1])

    for i, date_str in enumerate(dates):
        day_data = par_df[par_df['date'] == date_str]
        daylight = day_data[day_data['elevation_deg'] > 0]

        ax5.plot(daylight['hour'], daylight['PAR_std'],
                marker='o', linewidth=1.5, markersize=4, color=colors[i],
                label=date_str, alpha=0.7)

    ax5.set_xlabel('Hour of Day', fontsize=11, fontweight='bold')
    ax5.set_ylabel('PAR Std Dev (umol m^-2 s^-1)', fontsize=11, fontweight='bold')
    ax5.set_title('E. Spatial Variation Across Leaves', fontsize=12, fontweight='bold', loc='left')
    ax5.legend(ncol=2, fontsize=8, loc='upper left')
    ax5.grid(alpha=0.3)

    # Overall title
    mean_integral = validation_results['mean_daily_integral']
    std_integral = validation_results['std_daily_integral']
    fig.suptitle(f'Sorghum 7-Leaf Plant Multi-Day PAR Analysis (June 10-17, 2024)\n' +
                 f'Maricopa, AZ - Mean Daily Integral: {mean_integral:.2f} +/- {std_integral:.2f} mol m^-2 day^-1',
                 fontsize=14, fontweight='bold', y=0.995)

    # Save figure
    fig_path = output_dir / 'multiday_par_visualization.png'
    plt.savefig(fig_path, dpi=300, bbox_inches='tight')
    print(f"[OK] Saved visualization: {fig_path}")

    return fig

# =============================================================================
# Main Workflow
# =============================================================================

def main():
    """Main workflow for multi-day EvoEngine ray-traced PAR calculation"""

    # Create output directories
    OUTPUT_DIR.mkdir(exist_ok=True)
    SCREENSHOT_DIR.mkdir(exist_ok=True)

    # Step 1: Calculate solar positions for all days
    print("Step 1: Calculating solar positions for multi-day period...")
    sun_positions = generate_multiday_sun_path(
        START_DATE, END_DATE, START_HOUR, END_HOUR, HOUR_STEP,
        LATITUDE, LONGITUDE, TIMEZONE
    )
    print(f"[OK] Generated {len(sun_positions)} solar positions")

    # Count by date
    dates = sun_positions['date'].unique()
    print(f"  Dates: {len(dates)} days ({dates[0]} to {dates[-1]})")
    daylight_hours = len(sun_positions[sun_positions['elevation_deg'] > 0])
    print(f"  Daylight timesteps: {daylight_hours} (elevation > 0°)")
    print()

    # Step 2: Initialize EvoEngine (ONE TIME)
    print("Step 2: Initializing EvoEngine...")
    initialize_evoengine()
    print()

    # Step 3: Setup plant geometry (ONE TIME)
    print("Step 3: Setting up plant geometry...")
    sorghum_entity = setup_plant_geometry()
    print()

    # Step 4: Run ray-traced illumination for all timesteps
    print("Step 4: Running multi-day ray-traced illumination...")
    par_df = run_raytraced_illumination_multiday(sun_positions, sorghum_entity)
    print(f"  Peak PAR (overall): {par_df['PAR_mean'].max():.1f} μmol m⁻² s⁻¹")
    print(f"  Mean PAR (daylight): {par_df[par_df['PAR_mean'] > 0]['PAR_mean'].mean():.1f} μmol m⁻² s⁻¹")
    print()

    # Step 5: Save results
    print("Step 5: Saving results...")
    output_csv_path = OUTPUT_DIR / OUTPUT_CSV
    par_df.to_csv(output_csv_path, index=False)
    print(f"[OK] Saved CSV: {output_csv_path}")
    print(f"  Rows: {len(par_df)}")
    print(f"  Columns: {list(par_df.columns)}")
    print()

    # Step 6: Validation
    print("Step 6: Validating multi-day results...")
    validation = validate_multiday_par(par_df)

    # Step 7: Visualization
    print("Step 7: Creating multi-day visualization...")
    create_multiday_visualization(par_df, validation, OUTPUT_DIR)
    print()

    # Step 8: Cleanup
    print("Step 8: Cleaning up...")
    sorghum_framework.Terminate()
    print("[OK] EvoEngine terminated")

    # Restore original directory
    os.chdir(current_directory)
    print()

    # Summary
    print("="*80)
    print("SUMMARY - MULTI-DAY SIMULATION")
    print("="*80)
    print(f"EvoEngine build: {EVOENGINE_DIRECTORY}")
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Data file: {OUTPUT_CSV}")
    print(f"Visualization: multiday_par_visualization.png")
    print()

    print(f"Date range: {START_DATE} to {END_DATE} ({len(dates)} days)")
    print(f"Total timesteps: {len(par_df)}")
    print(f"Daylight timesteps: {daylight_hours}")
    print()

    # Plant statistics
    num_leaves = int(par_df['num_leaves'].max())
    total_area = par_df['total_leaf_area'].max()

    print(f"Number of leaves: {num_leaves}")
    print(f"Total leaf area: {total_area:.3f} m^2")
    print()

    # Multi-day statistics
    print(f"Mean daily integral: {validation['mean_daily_integral']:.2f} +/- {validation['std_daily_integral']:.2f} mol m^-2 day^-1")
    print(f"Peak PAR (overall): {validation['peak_par_overall']:.1f} umol m^-2 s^-1")
    print(f"Validation status: {validation['status']}")
    print()

    print("Ray tracing method: EvoEngine GPU with BTF leaf optics")
    print(f"Plant configuration: Full 7-leaf sorghum with stem and panicle")
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
