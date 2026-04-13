# Full Plant PAR Calculation - Quick Start

## Files Created

1. **FULL_PLANT_PAR_PLAN.md** - Detailed plan and rationale
2. **sorghum_full_plant_daily_par.py** - Full plant PAR calculation script

## Key Differences from Single Leaf

### Mesh Generation
```python
# Full plant settings (all leaves + stem + panicle)
mesh_settings.enable_panicle = True    # Include seed head
mesh_settings.enable_stem = True       # Include stem
mesh_settings.enable_leaves = True     # All leaves
mesh_settings.single_leaf_index = -1   # -1 = ALL LEAVES
```

### Result Aggregation
- Aggregates PAR across ALL leaf entities
- Calculates area-weighted average
- Tracks per-leaf statistics (min, max, std)
- Reports number of leaves and total leaf area

### Expected Results
| Metric | Single Leaf | Full Plant |
|--------|-------------|------------|
| **Peak PAR** | ~1291 μmol m⁻² s⁻¹ | ~800-1200 μmol m⁻² s⁻¹ |
| **Daily integral** | ~41 mol m⁻² day⁻¹ | ~20-35 mol m⁻² day⁻¹ |
| **Spatial variation** | None (uniform) | High (self-shading) |
| **Ray trace time** | ~7 seconds | ~4-5 minutes |

## How to Run

### Quick Test (5 timesteps, ~2 min)
Edit line 60 in the script:
```python
HOUR_STEP = 3  # Every 3 hours: 6:00, 9:00, 12:00, 15:00, 18:00
```

Then run:
```bash
cd C:\Users\Brenda\code\EvoEngine\claude
py -3.9 sorghum_full_plant_daily_par.py
```

### Full Simulation (15 timesteps, ~5 min)
Use default settings (HOUR_STEP = 1):
```bash
cd C:\Users\Brenda\code\EvoEngine\claude
py -3.9 sorghum_full_plant_daily_par.py
```

## Expected Output

### Console Output
```
================================================================================
SORGHUM FULL PLANT DAILY PAR CALCULATION - EvoEngine Ray Tracing
================================================================================
Location: 33.07degN, -111.97degW (Maricopa, AZ)
Date: 2024-06-13
Time range: 6:00 - 20:00 (hourly)
Plant: Full sorghum with all leaves, stem, panicle
================================================================================

...

Ray tracing:  100%|##########| 15/15 [04:30<00:00, leaves=10, PAR=850]

[OK] Ray-traced illumination complete
  Peak PAR: 1050.2 umol m-2 s-
  Mean PAR (daylight): 680.5 umol m-2 s-

================================================================================
VALIDATION RESULTS - FULL PLANT
================================================================================
Total daily PAR integral: 28.50 mol photons m-2 day-
Expected range (full plant with self-shading): 15-40 mol photons m-2 day-
Peak PAR: 1050.2 umol m-2 s-
Average spatial variation (std): 245.3 umol m-2 s-

[OK] Daily PAR integral within expected range
[OK] Spatial variation detected (self-shading present)
================================================================================

================================================================================
SUMMARY - FULL PLANT
================================================================================
Number of leaves: 10
Total leaf area: 0.480 m2

Peak PAR: 1050.2 umol m-2 s- at 11:00
Daily PAR integral: 28.50 mol m-2 day-
Spatial variation (avg std): 245.3 umol m-2 s-
Validation: PASS

Ray tracing method: EvoEngine GPU with BTF leaf optics
Plant configuration: Full sorghum with stem, leaves, and panicle
================================================================================
```

### Output Files
- **CSV**: `evoengine_par_results/hourly_PAR_full_plant_maricopa_june13.csv`
- **Visualization**: `evoengine_par_results/daily_par_visualization.png`

### CSV Format
```csv
datetime,elevation_deg,azimuth_deg,zenith_deg,hour,PAR_mean,PAR_max,PAR_min,PAR_std,num_vertices,total_leaf_area
2024-06-13 06:00:00-07:00,6.97,66.73,83.03,6,350.2,480.5,210.3,125.4,10,0.480
2024-06-13 07:00:00-07:00,18.80,73.98,71.20,7,720.5,980.2,420.8,230.1,10,0.480
...
```

**New columns for full plant:**
- `num_vertices` - Number of leaves detected
- `total_leaf_area` - Total leaf area in m²
- `PAR_std` - Standard deviation across leaves (self-shading indicator)
- `PAR_max` - Highest PAR among leaves (upper canopy)
- `PAR_min` - Lowest PAR among leaves (lower canopy)

## Validation Criteria

### PASS Conditions
1. ✓ Daily PAR integral: 15-40 mol m⁻² day⁻¹
2. ✓ Spatial variation (std) > 100 μmol m⁻² s⁻¹
3. ✓ Multiple leaves detected (typically 8-12)

### Common Issues

**Issue**: "No leaves detected (num_vertices = 0)"
- **Cause**: Mesh generation failed
- **Fix**: Check that sorghum descriptor exists at `./SorghumGenerator/Sample1.sorghum`

**Issue**: "Low spatial variation (std < 100)"
- **Cause**: Self-shading not captured (single entity instead of multiple leaves)
- **Fix**: Verify `mesh_settings.leaf_separated = True`

**Issue**: "Daily PAR too high (> 40 mol m⁻² day⁻¹)"
- **Cause**: May have generated single leaf instead of full plant
- **Fix**: Verify `mesh_settings.single_leaf_index = -1`

**Issue**: "CUDA out of memory"
- **Cause**: Too many vertices for GPU
- **Fix**: Reduce mesh resolution: `mesh_settings.leaf_thickness = 0.002`

## Comparison with Single Leaf

To compare full plant vs single leaf:
```python
# Load both results
single_leaf = pd.read_csv('evoengine_par_results/hourly_PAR_maricopa_june13_evoengine.csv')
full_plant = pd.read_csv('evoengine_par_results/hourly_PAR_full_plant_maricopa_june13.csv')

# Compare daily integrals
single_leaf_daily = single_leaf['PAR_mean'].sum() * 3600 / 1e6  # ~41 mol m-2 day-1
full_plant_daily = full_plant['PAR_mean'].sum() * 3600 / 1e6   # ~28 mol m-2 day-1

# Self-shading effect
shading_reduction = (1 - full_plant_daily / single_leaf_daily) * 100
print(f"Self-shading reduces PAR by {shading_reduction:.1f}%")  # Expected: ~30-35%
```

## Next Steps

1. **Run full plant simulation** (~5 min)
2. **Compare with single leaf results** from previous run
3. **Analyze spatial PAR distribution** (upper vs lower leaves)
4. **Use for photosynthesis modeling** (C4 biochemical model input)

---

**Created**: 2024-11-04
**Location**: C:\Users\Brenda\code\EvoEngine\claude\
**Status**: Ready to run
