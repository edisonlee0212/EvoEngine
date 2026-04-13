# Ray Tracer Validation Plan: Horizontal Leaf Test

## Objective

Determine if the EvoEngine ray tracer is correctly calculating PAR by testing with a **flat horizontal reference surface** that should match analytical predictions.

## Problem Statement

Current sorghum leaf PAR shows:
- Peak at 10am (elevation 56°) instead of solar noon (elevation 78°)
- Asymmetric morning/afternoon pattern
- Evening PAR increases (17:00-18:00)

**Question**: Is this due to:
1. **Ray tracer bug** (incorrect physics implementation), OR
2. **Leaf geometry** (realistic sorghum leaf angle/curvature)?

## Phase 1: Horizontal Reference Test

### Test Setup

Create a flat horizontal quad with:
- **Normal vector**: [0, 1, 0] (pointing straight up)
- **No curvature**: Perfectly flat plane
- **Size**: Similar to sorghum leaf (50 cm × 6 cm)

### Expected Results (Analytical Model)

For a horizontal surface, PAR should follow:

```
PAR(t) = I₀ × τ(zenith) × cos(zenith)

Where:
  I₀ = Base intensity at zenith (40000.0)
  τ = Atmospheric transmittance = 0.7^(AM^0.678)
  AM = Air mass = 1 / [cos(zenith) + 0.50572 × (96.08 - zenith)^-1.6364]
  cos(zenith) = Cosine factor for surface orientation
```

**Key predictions:**
- **Peak at solar noon** (12:00-13:00, elevation ~78°)
- **Symmetric bell curve** (morning = afternoon for same zenith angle)
- **No evening bumps** (monotonic decline after noon)

| Time | Elevation | Zenith | cos(θ) | Trans. | Expected PAR |
|------|-----------|--------|--------|--------|--------------|
| 10:00 | 56° | 34° | 0.829 | 0.667 | ~1325 |
| 12:00 | 78° | 12° | 0.978 | 0.697 | ~1733 ← PEAK |
| 14:00 | 68° | 22° | 0.927 | 0.687 | ~1527 |

### Decision Tree

```
Run horizontal leaf test
    │
    ├─→ R² > 0.95, RMSE < 200, Peak at noon?
    │   │
    │   YES → ✓ RAY TRACER IS CORRECT
    │         → Original asymmetry is due to LEAF GEOMETRY
    │         → Sorghum leaf angle/curvature causes observed pattern
    │         → This is actually MORE REALISTIC than analytical model!
    │
    └─→ R² < 0.9, High error, Peak not at noon?
        │
        NO → ✗ RAY TRACER HAS BUG
             → Proceed to Phase 2: Diagnostic Testing
```

## Phase 2: Diagnostic Testing (If Phase 1 Fails)

### Test 2.1: Verify Mesh Normals

**Check**: Are mesh normals actually [0, 1, 0]?

**How to check**:
1. Export mesh vertices and normals from illumination estimator
2. Calculate average normal: should be [0, 1, 0] ± 0.01
3. Check normal variance: should be near zero for flat surface

**Python code**:
```python
# In GetAllIlluminationEstimationResultsOnSorghum
# Add normal vector to returned data
# Check: np.allclose(avg_normal, [0, 1, 0], atol=0.01)
```

**If normals are wrong**:
→ Issue in mesh generation (sorghum descriptor creates curved leaf)
→ Need to create custom flat quad mesh

### Test 2.2: Verify cos(zenith) Calculation

**Check**: Is the ray tracer correctly applying `NdotL = dot(normal, sunDir)`?

**Location**: `IlluminationEstimation.cu` lines 84, 151

**Code to verify**:
```cuda
float NdotL = glm::dot(avgNormal, sunDir);
// Should match cos(zenith_rad) for horizontal surface
```

**Test**:
1. Add logging of NdotL values
2. Compare with analytical cos(zenith)
3. Should match within 1%

**If mismatch**:
→ Sun direction calculation error in `SetSunDirection()`
→ Check azimuth/elevation to direction vector conversion

### Test 2.3: Verify Atmospheric Transmittance

**Check**: Is transmittance calculation matching analytical model?

**Location**: `SorghumLayer.cpp` lines 408-421

**Test**:
1. Log calculated transmittance values
2. Compare with analytical: τ = 0.7^(AM^0.678)
3. Should match within 0.5%

**If mismatch**:
→ Air mass formula error
→ Transmittance exponent error

### Test 2.4: Verify Intensity Application

**Check**: Is attenuated intensity correctly set?

**Location**: `SorghumLayer.cpp` line 443

**Test**:
```cpp
// Log these values
EVOENGINE_LOG("Base: " + std::to_string(base_intensity) +
              ", Trans: " + std::to_string(transmittance) +
              ", Attenuated: " + std::to_string(attenuated_intensity));
```

**If intensity not being applied**:
→ Check if environment_properties is being copied correctly to GPU
→ Verify OptiX kernel is reading correct values

## Phase 3: Geometry Effect Confirmation (If Phase 1 Passes)

### Test 3.1: 30° Angled Leaf

Create leaf tilted 30° from horizontal, facing east:
- **Normal**: [sin(30°), cos(30°), 0] = [0.5, 0.866, 0]
- **Expected**: Peak in morning (east-facing catches morning sun)
- **Expected**: Lower afternoon PAR (sun from west misses leaf)

**If this matches original asymmetric pattern**:
→ Confirms that realistic sorghum leaf geometry causes the effect
→ EvoEngine is MORE ACCURATE than analytical horizontal assumption

### Test 3.2: Curved Leaf Profile

Create leaf with realistic sorghum midrib curvature:
- **Variable normals**: Different parts face different directions
- **Expected**: Complex PAR pattern
- **Expected**: Spatial variation (PAR_std > 0)

**If spatial variation appears**:
→ Confirms ray tracer captures self-shading
→ Validates spatially-explicit PAR for Photo3 model

## Success Criteria

### Ray Tracer is Correct if:

1. **Horizontal leaf test**:
   - R² > 0.95 with analytical model
   - RMSE < 200 μmol m⁻² s⁻¹
   - Peak PAR within 1 hour of solar noon
   - Symmetric morning/afternoon pattern (asymmetry < 10%)

2. **Angled leaf test** (if horizontal passes):
   - Shows expected directional bias
   - Peak shifts toward leaf orientation
   - Confirms geometry effects

3. **Physical constraints met**:
   - PAR ≥ 0 (no negative values)
   - Daily integral 30-50 mol m⁻² day⁻¹
   - cos(zenith) factor correctly applied

### Ray Tracer has Bug if:

1. **Horizontal leaf test fails**:
   - Poor correlation (R² < 0.8)
   - Large systematic error (RMSE > 500)
   - Peak not at noon (off by >2 hours)
   - Non-symmetric pattern for symmetric geometry

2. **Diagnostic tests reveal**:
   - Mesh normals are correct but PAR doesn't match
   - cos(zenith) not being applied
   - Atmospheric transmittance mismatch
   - Intensity not modulated by sun angle

## Implementation Notes

### Python Script Features

**`test_horizontal_leaf_par.py`** includes:
- Automatic solar position calculation
- Analytical PAR prediction for horizontal surface
- Statistical comparison (R², RMSE, correlation)
- Bell curve symmetry check
- Automated diagnostic decision tree
- Visualization with 4 panels:
  - Time series comparison
  - Scatter plot (1:1 line)
  - Absolute error
  - Percent error

### Running the Test

```bash
cd C:\Users\Brenda\code\EvoEngine\claude
env PYTHONIOENCODING=utf-8 py -3.9 test_horizontal_leaf_par.py
```

**Output**:
- Console: Statistical comparison + diagnostic conclusion
- `horizontal_leaf_validation.png`: Visualization
- `horizontal_leaf_par_validation.csv`: Detailed results

## Expected Outcomes

### Most Likely Scenario: Ray Tracer is Correct

Based on physics:
- Atmospheric attenuation is correctly implemented ✓
- cos(zenith) factor is in `NdotL` calculation ✓
- Original asymmetry is from **realistic sorghum leaf geometry** ✓

**Interpretation**: The "anomaly" isn't a bug—it's the ray tracer correctly modeling a non-horizontal leaf!

**Implications for C4 modeling**:
- EvoEngine provides MORE accurate PAR than analytical methods
- Leaf angle and curvature significantly affect light interception
- Spatially-explicit PAR is essential for Photo3 photosynthesis model

### Alternative Scenario: Ray Tracer has Bug

If horizontal test fails, likely causes:
1. **Sun direction error**: Azimuth/elevation → vector conversion wrong
2. **Normal calculation**: Mesh normals not being read correctly
3. **GPU data transfer**: Environment properties not reaching OptiX kernel
4. **Cosine factor**: `NdotL` not being applied or applied incorrectly

**Fix priority**:
1. Verify sun direction vector calculation
2. Check mesh normal orientation
3. Add GPU-side logging to CUDA kernel
4. Validate OptiX ray intersection data

## Documentation

After completing validation:

Create `RAY_TRACER_VALIDATION_REPORT.md` with:
- Test results (horizontal leaf)
- Statistical metrics (R², RMSE)
- Diagnostic conclusions
- Next steps (if validation passed: explain geometry effects)
- Figures: horizontal validation + geometry comparison

## Timeline

- **Phase 1**: 30 minutes (run horizontal test)
- **Phase 2**: 1-2 hours (if diagnostic needed)
- **Phase 3**: 30 minutes (geometry confirmation)
- **Documentation**: 30 minutes

**Total**: 1-3 hours depending on results
