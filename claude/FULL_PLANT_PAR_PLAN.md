# Plan: Ray-Traced PAR Calculation for Entire Sorghum Plant

## Overview

We'll run GPU ray tracing on a complete sorghum plant (not just a single leaf) to calculate PAR across all leaves throughout a full day. This will give us realistic self-shading and spatial PAR distribution across the canopy.

## Current Status

✓ **Working Components:**
- Single leaf PAR calculation validated (40.83 mol m⁻² day⁻¹)
- Ray tracer operational (OptiX on RTX 2070 SUPER)
- Solar position calculation (pvlib)
- Atmospheric correction (Beer-Lambert)
- CSV export and visualization

✓ **Available APIs:**
- `CreateEntityFromSorghumDescriptor()` - Creates full plant from descriptor
- `GenerateSorghumMesh()` - Generates mesh for entire plant
- `SetSunDirection()` - Controls sun position
- `IlluminationEstimationOnSorghum()` - Ray tracing calculation
- `GetAllIlluminationEstimationResultsOnSorghum()` - Retrieves results

## Key Differences: Single Leaf vs Full Plant

| Aspect | Single Leaf | Full Plant |
|--------|-------------|------------|
| **Geometry** | ~300 vertices | ~10,000-50,000 vertices |
| **Self-shading** | None | Significant (upper leaves shade lower) |
| **Ray tracing time** | ~0.5 sec/timestep | ~10-30 sec/timestep |
| **PAR distribution** | Uniform across leaf | Spatial gradient (top > bottom) |
| **Memory** | ~50 KB | ~5-20 MB |
| **Biological realism** | Low | High |

## Plan Details

### Step 1: Modify Mesh Generation Settings

**Change from single leaf to full plant:**

```python
# Current (single leaf):
mesh_settings.enable_panicle = False
mesh_settings.enable_stem = False
mesh_settings.enable_leaves = True
mesh_settings.single_leaf_index = 2  # Only leaf #2

# New (full plant):
mesh_settings.enable_panicle = True   # Include seed head
mesh_settings.enable_stem = True      # Include stem
mesh_settings.enable_leaves = True    # All leaves
mesh_settings.single_leaf_index = -1  # -1 = all leaves (not just one)
mesh_settings.leaf_separated = True   # Each leaf as separate entity
```

**Rationale:**
- `single_leaf_index = -1` generates all leaves (typically 8-12 leaves per plant)
- `leaf_separated = True` creates separate entities per leaf (better for per-leaf PAR)
- Include panicle and stem for realistic shading

### Step 2: Update Result Retrieval

**Current approach returns single value per entity:**
```python
result = GetAllIlluminationEstimationResultsOnSorghum()
# Returns: [[position, rotation, total_area, total_flux, average_flux]]
```

**For full plant, we need to aggregate across all leaf entities:**
```python
results = GetAllIlluminationEstimationResultsOnSorghum()

# Sum across all leaves
total_area = sum(r[2].x for r in results if r[2].x > 0)
total_flux = sum(r[3].x for r in results if r[2].x > 0)
average_par = total_flux / total_area if total_area > 0 else 0

# Per-leaf breakdown
leaf_pars = [r[4].x for r in results if r[2].x > 0]
```

### Step 3: Adjust Expected PAR Values

**Single leaf expectations:**
- Peak PAR: ~1291 μmol m⁻² s⁻¹ (no self-shading)
- Daily integral: ~40 mol m⁻² day⁻¹

**Full plant expectations:**
- **Upper leaves**: 1200-1800 μmol m⁻² s⁻¹ (high light)
- **Mid-canopy**: 600-1000 μmol m⁻² s⁻¹ (moderate shading)
- **Lower leaves**: 200-500 μmol m⁻² s⁻¹ (heavy shading)
- **Whole-plant average**: 500-900 μmol m⁻² s⁻¹
- **Daily integral (average)**: 20-35 mol m⁻² day⁻¹

**Validation criteria:**
```python
# Spatial variation should be significant
par_std > 200  # Expect high variability due to self-shading

# Gradient from top to bottom
upper_leaf_par > lower_leaf_par * 2  # Upper leaves get >2x more light

# Total plant light interception
total_daily_par > 200 mol/day  # Sum across all leaves
```

### Step 4: Performance Considerations

**Ray tracing time estimate:**
- Single leaf: ~0.5 sec/timestep × 15 timesteps = **~7 seconds**
- Full plant: ~15 sec/timestep × 15 timesteps = **~4 minutes**

**Optimization strategies:**
1. **Reduce timesteps** for testing:
   ```python
   # Instead of hourly (15 timesteps):
   TIMESTEP_HOURS = 2  # Every 2 hours → 8 timesteps
   ```

2. **Use coarser mesh** for initial testing:
   ```python
   mesh_settings.leaf_thickness = 0.002  # Coarser (faster)
   ```

3. **Test with subset of leaves**:
   ```python
   # For debugging, generate only upper leaves:
   mesh_settings.single_leaf_index = 8  # Top leaf only
   ```

### Step 5: Create New Script

**File: `sorghum_full_plant_daily_par.py`**

Based on `sorghum_single_leaf_daily_par_evoengine.py` with these changes:

```python
def setup_plant_geometry():
    """Generate full sorghum plant with all leaves."""
    mesh_settings = sorghum_framework.SorghumMeshGeneratorSettings()

    # FULL PLANT (not single leaf)
    mesh_settings.enable_panicle = True
    mesh_settings.enable_stem = True
    mesh_settings.enable_leaves = True
    mesh_settings.single_leaf_index = -1  # ALL LEAVES
    mesh_settings.bottom_face = True
    mesh_settings.leaf_separated = True
    mesh_settings.leaf_thickness = 0.001

    # Create sorghum entity
    sorghum_state_handle = sorghum_framework.GetAssetHandle(
        "./SorghumGenerator/Sample1.sorghum"
    )
    sorghum_entity = sorghum_framework.CreateEntityFromSorghumDescriptor(
        sorghum_state_handle
    )

    # Generate mesh
    sorghum_framework.GenerateSorghumMesh(mesh_settings)

    return sorghum_entity

def aggregate_plant_results(results):
    """Aggregate PAR across all leaves."""
    if not results:
        return 0.0, 0.0, []

    # Filter valid leaves (area > 0)
    valid_leaves = [r for r in results if len(r) >= 5 and r[2].x > 0]

    if not valid_leaves:
        return 0.0, 0.0, []

    # Calculate totals
    total_area = sum(r[2].x for r in valid_leaves)
    total_flux = sum(r[3].x for r in valid_leaves)

    # Area-weighted average PAR
    avg_par = total_flux / total_area if total_area > 0 else 0.0

    # Per-leaf PAR values
    leaf_pars = [r[4].x for r in valid_leaves]

    return avg_par, total_area, leaf_pars
```

**Updated validation:**
```python
def validate_full_plant_results(par_df):
    """Validate full plant PAR results."""
    peak_par = par_df['PAR_mean'].max()
    daily_integral = (par_df['PAR_mean'].sum() * 3600) / 1e6

    # Validation checks
    checks = {
        'peak_par_range': 400 <= peak_par <= 1500,  # Lower than single leaf
        'daily_integral': 15 <= daily_integral <= 40,  # Lower due to shading
        'spatial_variation': par_df['PAR_std'].mean() > 100,  # High variability
    }

    return all(checks.values()), checks
```

### Step 6: Output Enhancements

**Add spatial analysis:**

```python
# Per-leaf statistics
leaf_stats = {
    'leaf_id': range(len(leaf_pars)),
    'par_mean': leaf_pars,
    'height': [get_leaf_height(i) for i in range(len(leaf_pars))],
}
leaf_df = pd.DataFrame(leaf_stats)
leaf_df.to_csv('per_leaf_par_distribution.csv')

# Visualization: PAR vs height
plt.figure(figsize=(10, 6))
plt.scatter(leaf_df['height'], leaf_df['par_mean'])
plt.xlabel('Leaf Height (m)')
plt.ylabel('Daily Average PAR (umol m-2 s-1)')
plt.title('PAR Distribution by Leaf Height')
```

**Enhanced output:**
- `hourly_PAR_full_plant.csv` - Whole-plant average PAR over time
- `per_leaf_par_distribution.csv` - PAR by leaf position
- `canopy_par_gradient.png` - Visualization of vertical PAR gradient

## Implementation Steps

### Phase 1: Test Run (Quick validation)
```bash
# Use reduced timesteps for fast testing
TIMESTEP_HOURS = 3  # 6:00, 9:00, 12:00, 15:00, 18:00 -> 5 timesteps
# Expected runtime: ~1-2 minutes
```

**Success criteria:**
- Multiple leaf entities detected (8-12 typical)
- PAR values vary significantly between leaves
- No crashes or CUDA errors

### Phase 2: Full Day Run (Production)
```bash
# Full hourly resolution
TIMESTEP_HOURS = 1  # 15 timesteps
# Expected runtime: ~4-5 minutes
```

**Success criteria:**
- Smooth PAR curves over time
- Daily integral in expected range (15-40 mol m-2 day-1)
- Clear vertical gradient (upper > lower leaves)

### Phase 3: Analysis & Visualization
- Generate per-leaf PAR distributions
- Create canopy gradient plots
- Compare with single leaf results
- Export for photosynthesis modeling

## Expected Outputs

### 1. Hourly Whole-Plant PAR
```csv
datetime,elevation_deg,azimuth_deg,PAR_mean,PAR_std,num_leaves,total_leaf_area_m2
2024-06-13 06:00:00,6.97,66.73,350.2,125.4,10,0.48
2024-06-13 07:00:00,18.80,73.98,720.5,230.1,10,0.48
...
```

### 2. Per-Leaf Spatial Distribution
```csv
leaf_id,height_m,par_mean,par_max,par_min,shading_fraction
0,0.15,245.3,380.2,120.5,0.75
1,0.35,420.8,680.1,210.3,0.55
2,0.55,650.2,950.4,380.7,0.35
...
```

### 3. Summary Statistics
```
Full Plant PAR Summary (June 13, 2024)
======================================
Number of leaves: 10
Total leaf area: 0.48 m2

Daily Average PAR: 680 umol m-2 s-1
Peak PAR (noon): 1050 umol m-2 s-1
Daily Integral: 28.5 mol m-2 day-1

Canopy Gradient:
  Upper leaves (top 3): 950 umol m-2 s-1
  Mid leaves (4-7): 680 umol m-2 s-1
  Lower leaves (8-10): 340 umol m-2 s-1

Self-shading efficiency: 35% light reduction vs single leaf
```

## Risk Mitigation

**Potential issues:**

1. **Out of GPU memory** (10-50K vertices)
   - **Mitigation**: Use coarser mesh, reduce leaf count for testing

2. **Long ray tracing time** (4-5 min total)
   - **Mitigation**: Start with 3-hour timesteps (5 timesteps)

3. **Multiple entities complicate aggregation**
   - **Mitigation**: Pre-built aggregation function handles multiple leaves

4. **Rotation limitation still applies**
   - **Note**: Leaf angles from descriptor, not runtime rotation

## Next Steps After This Plan

1. **Create `sorghum_full_plant_daily_par.py`** script
2. **Run test with 5 timesteps** (~2 min)
3. **Validate spatial PAR distribution** (upper > lower)
4. **Run full hourly simulation** (~5 min)
5. **Generate canopy gradient analysis**
6. **Compare with single-leaf results** (expect ~30-40% lower due to shading)

---

**Date**: 2024-11-04
**EvoEngine Build**: x64-Release
**Ray Tracer**: OptiX on RTX 2070 SUPER
