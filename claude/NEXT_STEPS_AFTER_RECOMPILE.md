# Next Steps After Recompile - Complete Workflow
**EvoEngine PAR Calculation Project**
**Date Created:** October 31, 2025

---

## 🔄 Complete Workflow After Recompile

### **Step 1: Run PAR Calculation** (2-3 minutes)
```bash
cd C:/Users/Brenda/code/EvoEngine/claude
py -3.9 sorghum_single_leaf_daily_par_evoengine.py
```

**What happens:**
- Calculates 15 hourly PAR values (6 AM - 8 PM, June 13, 2024)
- Uses GPU ray tracing for each sun position
- Progress bar shows: `Ray tracing: 100% |████████████| 15/15 [02:30<00:00, 10.2s/hour]`

**Expected outputs:**
```
evoengine_par_results/
├── hourly_PAR_maricopa_june13_evoengine.csv
├── par_validation_plots.png
└── screenshots/ (optional)
```

---

### **Step 2: Verify Results** (1 minute)
```bash
# Check the CSV was created
ls -lh evoengine_par_results/

# View the data
head evoengine_par_results/hourly_PAR_maricopa_june13_evoengine.csv
```

**CSV structure:**
```csv
hour,datetime,elevation_deg,azimuth_deg,zenith_deg,PAR_mean,PAR_std,PAR_min,PAR_max,num_vertices
6,2024-06-13 06:00:00-07:00,5.2,69.3,84.8,145.3,23.1,98.2,201.4,1250
7,2024-06-13 07:00:00-07:00,17.8,79.1,72.2,678.4,89.5,512.3,891.2,1250
...
12,2024-06-13 12:00:00-07:00,77.4,161.2,12.6,2104.5,156.3,1823.1,2389.6,1250
```

**Quick sanity checks:**
- ✅ Peak PAR around noon (12-1 PM)
- ✅ Peak value: 1800-2200 µmol/m²/s
- ✅ Night hours (before 6 AM, after 8 PM): 0 or very low
- ✅ Bell-shaped curve through the day

---

### **Step 3: Review Validation Plots** (2 minutes)

The script auto-generates `par_validation_plots.png` with 6 subplots:

1. **Hourly PAR throughout day** - Should show bell curve
2. **Solar position scatter** - Color-coded by PAR intensity
3. **PAR vs solar elevation** - Should be nearly linear for clear sky
4. **Cumulative daily PAR** - Should plateau at ~35-50 mol/m²/day
5. **PAR distribution histogram** - Shows data spread
6. **Sun path diagram** - Azimuth/elevation trajectory

**Red flags to look for:**
- ❌ PAR values > 2500 µmol/m²/s (unrealistic for single leaf)
- ❌ PAR values negative
- ❌ Flat line (ray tracing not working)
- ❌ Random spiky pattern (calculation errors)

---

### **Step 4: Scientific Validation** (5 minutes)

Compare your results to expected values for Maricopa, AZ in June:

**Expected ranges:**
```
Peak PAR (solar noon):     1800-2200 µmol/m²/s ✓
Daily total PAR:           30-50 mol/m²/day ✓
Morning rise time:         6-7 AM ✓
Evening decline:           6-7 PM ✓
Solar elevation at peak:   75-80° ✓
```

**C4 photosynthesis validation:**
```python
# Calculate daily CO2 fixation
quantum_efficiency = 0.053  # mol CO₂/mol photons (C4 plants)
leaf_area = 0.50 * 0.06     # 50cm × 6cm = 0.03 m²
daily_PAR = [SUM FROM CSV]  # mol/m²/day

total_CO2_fixed = daily_PAR * quantum_efficiency * leaf_area
# Expected: ~10-15 mmol CO₂/day for single leaf
```

---

## 🔬 Next Research Steps (Choose Your Path)

### **Option A: Scale to Full Plant**

Modify the script to calculate PAR for all leaves on the plant:

**File to edit:** `sorghum_single_leaf_daily_par_evoengine.py`

**Change line 99:**
```python
# OLD (single leaf)
.def_readwrite("single_leaf_index", &SorghumMeshGeneratorSettings::single_leaf_index)

# NEW (all leaves)
# Comment out single_leaf_index, or set to -1 for all leaves
```

**What this enables:**
- Compare PAR interception across leaf layers
- Identify shading patterns
- Calculate whole-plant canopy interception
- Optimize leaf arrangement for maximum light capture

**Expected runtime:** 5-10 minutes (more leaves = more ray tracing)

---

### **Option B: Daily/Seasonal Simulations**

Run simulations across multiple days or seasons:

**Multi-day simulation:**
```python
# Add to script configuration (around line 56)
import pandas as pd

# Multiple days in June
dates = pd.date_range('2024-06-01', '2024-06-30', freq='D')

for simulation_date in dates:
    SIMULATION_DATE = simulation_date.strftime('%Y-%m-%d')
    # Run calculation
    # Save results with date suffix
```

**Seasonal comparison:**
```python
key_dates = {
    'winter_solstice': '2024-12-21',
    'spring_equinox': '2024-03-20',
    'summer_solstice': '2024-06-21',
    'fall_equinox': '2024-09-22',
}

for season, date in key_dates.items():
    SIMULATION_DATE = date
    # Run and compare results
```

**Research questions this answers:**
- How does seasonal sun angle affect PAR?
- What's the optimal planting date for maximum light capture?
- How much PAR variation exists through the growing season?

---

### **Option C: Growth Stage Comparison**

Compare PAR interception at different sorghum developmental stages:

**Requires:**
- Multiple sorghum descriptor files (.sorghum)
- Each representing different growth stages

**Script modifications:**
```python
growth_stages = {
    'Stage 3 (Vegetative)': './SorghumGenerator/Sample1.sorghum',
    'Stage 5 (Flowering)': './SorghumGenerator/Sample2.sorghum',
    'Stage 7 (Grain Fill)': './SorghumGenerator/Sample3.sorghum',
}

results_by_stage = {}

for stage_name, descriptor_path in growth_stages.items():
    # Run calculation with descriptor
    results_by_stage[stage_name] = calculate_daily_par(descriptor_path)

# Compare PAR efficiency across stages
```

**Research insights:**
- Which stage has optimal light capture?
- How does canopy architecture change with growth?
- When does self-shading become significant?

---

### **Option D: Geographic Variation**

Test PAR across different growing regions:

**Locations to compare:**
```python
locations = {
    'Maricopa, AZ': {
        'latitude': 33.07,
        'longitude': -111.97,
        'elevation': 361,
        'timezone': 'America/Phoenix'
    },
    'Manhattan, KS': {
        'latitude': 39.20,
        'longitude': -96.59,
        'elevation': 322,
        'timezone': 'America/Chicago'
    },
    'Lubbock, TX': {
        'latitude': 33.58,
        'longitude': -101.85,
        'elevation': 993,
        'timezone': 'America/Chicago'
    },
    'Weslaco, TX (RGV)': {
        'latitude': 26.16,
        'longitude': -97.99,
        'elevation': 24,
        'timezone': 'America/Chicago'
    },
}

for location_name, params in locations.items():
    LATITUDE = params['latitude']
    LONGITUDE = params['longitude']
    # Run simulation
```

**Questions answered:**
- Which region has optimal PAR for sorghum?
- How does latitude affect daily light integral?
- Can we predict yield differences from PAR alone?

---

### **Option E: Environmental Conditions**

**Cloud cover simulation** (advanced):
```python
# Add atmospheric attenuation
cloud_cover_fraction = 0.3  # 30% cloudy

# Modify PAR calculation
clear_sky_PAR = calculate_PAR()
actual_PAR = clear_sky_PAR * (1 - cloud_cover_fraction * 0.7)
```

**Leaf angle optimization:**
```python
leaf_angles = [0, 15, 30, 45, 60, 75, 90]  # degrees from horizontal

results = {}
for angle in leaf_angles:
    LEAF_ANGLE_DEG = angle
    results[angle] = run_simulation()

# Find optimal angle for maximum daily PAR
optimal_angle = max(results, key=results.get)
```

---

### **Option F: Integration with Growth Models**

Export PAR data for use in crop models:

**APSIM format:**
```python
# APSIM expects daily radiation in MJ/m²/day
# Convert: 1 mol photons = 0.217 MJ

daily_PAR_mol = sum(hourly_PAR) / 1000000  # µmol to mol
daily_radiation_MJ = daily_PAR_mol * 0.217

# Write APSIM weather file
with open('maricopa_2024.met', 'w') as f:
    f.write(f"year day radn\n")
    f.write(f"2024 165 {daily_radiation_MJ:.2f}\n")
```

**DSSAT format:**
```python
# DSSAT expects MJ/m²/day
dssat_output = pd.DataFrame({
    'YEAR': [2024],
    'DOY': [165],  # June 13 = day 165
    'SRAD': [daily_radiation_MJ]
})
dssat_output.to_csv('MAAZ2024.WTH', index=False)
```

**Custom photosynthesis model:**
```python
# Feed hourly PAR into Farquhar-von Caemmerer C4 model
def farquhar_c4_photosynthesis(PAR, Tleaf, CO2):
    """
    Calculate net photosynthesis rate
    PAR: µmol/m²/s
    Tleaf: °C
    CO2: ppm
    Returns: µmol CO₂/m²/s
    """
    # Model parameters for sorghum
    Vcmax = 40  # Maximum carboxylation rate
    Jmax = 120  # Maximum electron transport rate

    # ... implement C4 biochemistry
    return net_photosynthesis_rate
```

---

## 🐛 Troubleshooting Results

### Problem: PAR values too high (>2500 µmol/m²/s)

**Possible causes:**
1. Sun intensity scaling issue
2. Multiple counting of photons
3. Wrong units conversion

**Debug steps:**
```python
# Check sun intensity
print(f"Sun intensity: {sorghum_framework.GetSunIntensity()}")  # Should be ~2000

# Check leaf area
print(f"Leaf vertices: {len(results[0])}")  # Should be ~1000-2000

# Verify units
# EvoEngine returns: µmol/m²/s directly
# No conversion needed
```

---

### Problem: PAR values too low (<1000 at noon)

**Possible causes:**
1. Leaf not fully generated (incomplete mesh)
2. Ray tracing using CPU instead of GPU
3. Wrong sun direction

**Debug steps:**
```bash
# Check for OptiX initialization
# Should see: "Optix: running on device: NVIDIA GeForce RTX 2070 SUPER"

# Verify sun direction
py -3.9 -c "
import pvlib
from datetime import datetime
times = [datetime(2024, 6, 13, 12, 0)]
solar_pos = pvlib.solarposition.get_solarposition(times, 33.07, -111.97)
print(f'Elevation: {solar_pos.elevation[0]:.1f}°')
print(f'Azimuth: {solar_pos.azimuth[0]:.1f}°')
"
# Should show: Elevation ~77°, Azimuth ~161°
```

---

### Problem: Erratic/noisy results

**Possible causes:**
1. Insufficient ray samples
2. Geometry self-shadowing
3. Numerical instabilities

**Solutions:**
```python
# If API available, increase samples
# ray_tracer_settings.samples_per_pixel = 256

# Check mesh quality
print(f"Mesh triangles: {mesh.GetTriangleCount()}")

# Add smoothing to results
from scipy.ndimage import gaussian_filter1d
PAR_smoothed = gaussian_filter1d(PAR_values, sigma=1)
```

---

## 📊 Data Analysis Templates

### Calculate Daily Light Integral (DLI)
```python
# DLI = sum of hourly PAR over 24 hours
# Units: mol/m²/day

hourly_PAR = df['PAR_mean'].values  # µmol/m²/s
hours = 1  # hour timestep

DLI = sum(hourly_PAR) * 3600 / 1000000  # Convert to mol/m²/day
print(f"Daily Light Integral: {DLI:.1f} mol/m²/day")

# Sorghum optimal DLI: 30-60 mol/m²/day
```

### Calculate Light Use Efficiency (LUE)
```python
# LUE = biomass accumulation / intercepted PAR
# Units: g/mol photons

daily_biomass_gain = 5.2  # g/day (from field measurements)
intercepted_PAR = DLI * leaf_area  # mol/day

LUE = daily_biomass_gain / intercepted_PAR
print(f"Light Use Efficiency: {LUE:.2f} g/mol")

# Typical C4 LUE: 1.5-2.5 g/mol
```

### Compare to Field Measurements
```python
import pandas as pd
import numpy as np

# Load your simulation results
sim_data = pd.read_csv('evoengine_par_results/hourly_PAR_maricopa_june13_evoengine.csv')

# Load field measurements (if available)
field_data = pd.read_csv('field_measurements_june13.csv')

# Calculate RMSE
rmse = np.sqrt(np.mean((sim_data['PAR_mean'] - field_data['PAR_measured'])**2))
print(f"RMSE: {rmse:.1f} µmol/m²/s")

# Calculate R²
correlation = np.corrcoef(sim_data['PAR_mean'], field_data['PAR_measured'])[0,1]
r_squared = correlation**2
print(f"R²: {r_squared:.3f}")
```

---

## 📈 Visualization Examples

### Create Custom Plots
```python
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('evoengine_par_results/hourly_PAR_maricopa_june13_evoengine.csv')

# Plot 1: PAR with error bars
plt.figure(figsize=(10, 6))
plt.errorbar(df['hour'], df['PAR_mean'], yerr=df['PAR_std'],
             fmt='o-', capsize=5, linewidth=2, markersize=8)
plt.xlabel('Hour of Day', fontsize=14, fontweight='bold')
plt.ylabel('PAR (µmol/m²/s)', fontsize=14, fontweight='bold')
plt.title('Hourly PAR - June 13, 2024 (Maricopa, AZ)', fontsize=16, fontweight='bold')
plt.grid(True, alpha=0.3)
plt.savefig('hourly_par_with_errors.png', dpi=300, bbox_inches='tight')

# Plot 2: PAR vs solar elevation
plt.figure(figsize=(8, 8))
plt.scatter(df['elevation_deg'], df['PAR_mean'], s=100, alpha=0.6)
plt.xlabel('Solar Elevation (degrees)', fontsize=14, fontweight='bold')
plt.ylabel('PAR (µmol/m²/s)', fontsize=14, fontweight='bold')
plt.title('PAR vs Solar Elevation', fontsize=16, fontweight='bold')
plt.grid(True, alpha=0.3)
plt.savefig('par_vs_elevation.png', dpi=300, bbox_inches='tight')
```

---

## 🗂️ Organizing Results

### Directory structure for multiple simulations:
```
evoengine_par_results/
├── single_leaf/
│   ├── june_13/
│   │   ├── hourly_PAR.csv
│   │   ├── plots.png
│   │   └── metadata.txt
│   ├── seasonal_comparison/
│   │   ├── winter_solstice.csv
│   │   ├── spring_equinox.csv
│   │   ├── summer_solstice.csv
│   │   └── fall_equinox.csv
│   └── location_comparison/
│       ├── maricopa_az.csv
│       ├── manhattan_ks.csv
│       └── lubbock_tx.csv
├── full_plant/
│   └── june_13/
│       ├── leaf_by_leaf_PAR.csv
│       └── canopy_analysis.png
└── growth_stages/
    ├── stage3_vegetative.csv
    ├── stage5_flowering.csv
    └── stage7_grain_fill.csv
```

---

## 📝 Documentation Template

After each simulation run, create a metadata file:

**File:** `evoengine_par_results/metadata.txt`
```
========================================
EvoEngine PAR Simulation Metadata
========================================

Simulation Details:
- Date run: 2025-10-31
- Script version: sorghum_single_leaf_daily_par_evoengine.py v1.0
- EvoEngine build: x64-Release (October 31, 2025)

Location:
- Site: Maricopa Agricultural Center, Arizona
- Latitude: 33.07°N
- Longitude: -111.97°W
- Elevation: 361 m
- Timezone: America/Phoenix (MST, no DST)

Simulation Date:
- Date: June 13, 2024
- Day of year: 165
- Season: Early summer

Plant Parameters:
- Species: Sorghum bicolor
- Growth stage: Stage 3 (vegetative)
- Leaf dimensions: 50 cm length × 6 cm width
- Leaf angle: 30° from horizontal
- Leaf height: 1.0 m above ground
- Descriptor: Sample1.sorghum

Ray Tracing Settings:
- GPU: NVIDIA GeForce RTX 2070 SUPER
- Ray tracer: OptiX 7.x
- Samples per pixel: [default]
- Material: Default (BTF not available)

Time Parameters:
- Start time: 6:00 AM
- End time: 8:00 PM
- Time step: 1 hour
- Total timesteps: 15

Results Summary:
- Peak PAR: [INSERT] µmol/m²/s at [INSERT] hour
- Daily total PAR: [INSERT] mol/m²/day
- Average daylight PAR: [INSERT] µmol/m²/s
- Daylight hours: 14

Validation:
- Expected peak PAR: 1800-2200 µmol/m²/s ✓/✗
- Expected daily total: 30-50 mol/m²/day ✓/✗
- Bell curve pattern: ✓/✗
- Reasonable for C4 photosynthesis: ✓/✗

Files Generated:
- hourly_PAR_maricopa_june13_evoengine.csv
- par_validation_plots.png

Notes:
[Add any observations, issues, or special circumstances]
========================================
```

---

## 🎯 Success Checklist

After completing the workflow, verify:

- ✅ CSV file created with 15 rows (hourly data)
- ✅ Plots generated and visually reasonable
- ✅ Peak PAR: 1800-2200 µmol/m²/s
- ✅ Daily total: 30-50 mol/m²/day
- ✅ Bell curve pattern through the day
- ✅ No negative PAR values
- ✅ Solar angles match pvlib calculations
- ✅ Results documented with metadata
- ✅ Files organized and archived

---

## 🚀 Long-term Research Applications

This workflow enables research in:

1. **Canopy Architecture Optimization**
   - Test different leaf angle distributions
   - Optimize row spacing for light interception
   - Identify ideal plant density

2. **Climate Adaptation**
   - Predict performance across geographic regions
   - Model response to climate change scenarios
   - Identify optimal planting windows

3. **Breeding Programs**
   - Screen varieties for light use efficiency
   - Identify stay-green traits
   - Optimize leaf area index (LAI)

4. **Precision Agriculture**
   - Field-specific growth predictions
   - Variable rate seeding recommendations
   - Irrigation scheduling based on light × water

5. **Growth Modeling**
   - Parameterize APSIM/DSSAT models
   - Validate photosynthesis submodels
   - Predict biomass accumulation

6. **Resource Use Efficiency**
   - Link light capture to nitrogen use
   - Optimize fertilizer application timing
   - Model water use efficiency (WUE)

---

## 📚 References for Further Reading

**Solar radiation and photosynthesis:**
- Monteith, J.L. (1977). Climate and the efficiency of crop production in Britain. *Philosophical Transactions of the Royal Society B*, 281, 277-294.
- Zhu, X.G., et al. (2010). What is the maximum efficiency with which photosynthesis can convert solar energy into biomass? *Current Opinion in Biotechnology*, 21, 153-159.

**C4 photosynthesis:**
- von Caemmerer, S. (2000). *Biochemical Models of Leaf Photosynthesis*. CSIRO Publishing.
- Yin, X., & Struik, P.C. (2017). C3 and C4 photosynthesis models. *Journal of Experimental Botany*, 68, 2057-2074.

**Sorghum physiology:**
- Rooney, W.L., et al. (2007). Designing sorghum as a dedicated bioenergy feedstock. *Biofuels, Bioproducts and Biorefining*, 1, 147-157.
- Hammer, G.L., et al. (2010). Can changes in canopy and/or root system architecture explain historical maize yield trends? *Crop Science*, 50, 84-99.

---

## 💡 Tips for Success

1. **Start simple** - Run the default single-leaf case first
2. **Validate early** - Compare to expected ranges before scaling up
3. **Document everything** - Save metadata for every simulation
4. **Archive results** - Keep organized backups
5. **Visualize often** - Plots reveal issues quickly
6. **Compare to reality** - Use field data when available
7. **Ask for help** - Consult domain experts if results seem off

---

**You're ready to go!** After the recompile, you have a complete workflow from simulation to publication-ready results. Good luck with your research! 🌱📊✨
