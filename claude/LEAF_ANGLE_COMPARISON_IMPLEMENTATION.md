# Sorghum Leaf Angle Comparison Implementation

**Date:** 2025-11-04
**Purpose:** Compare PAR (Photosynthetically Active Radiation) for horizontal (0°) vs tilted (30°) sorghum leaves

---

## What Was Added

### 1. New C++ API: `SetEntityRotation`

**Purpose:** Rotate any entity in the scene to control leaf angle for experiments

**Files Modified:**
- `PythonBinding/include/PyDigitalAgriculture.hpp` (added declaration)
- `PythonBinding/src/PyDigitalAgriculture.cpp` (added implementation)
- `PythonBinding/src/PyDigitalAgricultureModule.cpp` (added Python binding)

**API Signature:**
```cpp
void SetEntityRotation(Entity entity, float pitch, float yaw, float roll);
```

**Parameters:**
- `entity`: The sorghum leaf entity to rotate
- `pitch`: Rotation around X axis in degrees (controls leaf angle from horizontal)
- `yaw`: Rotation around Y axis in degrees (azimuthal orientation)
- `roll`: Rotation around Z axis in degrees (leaf roll)

**Python Usage:**
```python
import PyDigitalAgriculture

# Create sorghum leaf
sorghum_entity = PyDigitalAgriculture.CreateEntityFromSorghumDescriptor(...)

# Set to horizontal (0°)
PyDigitalAgriculture.SetEntityRotation(sorghum_entity, 0, 0, 0)

# Set to 30° tilt
PyDigitalAgriculture.SetEntityRotation(sorghum_entity, 30, 0, 0)
```

---

## 2. New Comparison Script

**File:** `claude/compare_horizontal_vs_30deg_leaf.py`

**Features:**
- Compares PAR for 0° (horizontal) vs 30° (tilted) leaf
- Uses EvoEngine GPU ray tracing for both angles
- Includes analytical PAR calculation for validation
- Generates comprehensive 6-panel comparison visualization
- Calculates daily PAR integrals and peak values

**Output Files:**
- `par_comparison_results/par_horizontal_0deg.csv` - Hourly data for 0° leaf
- `par_comparison_results/par_30deg.csv` - Hourly data for 30° leaf
- `par_comparison_results/par_0deg_vs_30deg_comparison.csv` - Combined comparison
- `par_comparison_results/par_0deg_vs_30deg_comparison.png` - Visualization

**Visualization Panels:**
- **Panel A:** EvoEngine ray tracing comparison (0° vs 30°)
- **Panel B:** Analytical model comparison (0° vs 30°)
- **Panel C:** PAR ratio over time (horizontal / tilted)
- **Panel D:** Cumulative daily PAR accumulation
- **Panel E:** EvoEngine vs Analytical validation (0° leaf)
- **Panel F:** EvoEngine vs Analytical validation (30° leaf)

---

## Build Instructions

### Step 1: Rebuild Python Bindings

**Option A: Using build.cmd (Recommended)**

Open **"x64 Native Tools Command Prompt for VS 2022"** and run:

```cmd
cd C:\Users\Brenda\code\EvoEngine
build.cmd --no-test
```

**Expected time:** ~3-5 minutes

**Option B: Using rebuild_python_bindings.cmd**

```cmd
cd C:\Users\Brenda\code\EvoEngine\claude
rebuild_python_bindings.cmd
```

**Option C: Manual CMake Build**

```cmd
cd C:\Users\Brenda\code\EvoEngine\out\build\x64-Release
ninja PyDigitalAgriculture
```

### Step 2: Verify Build

Check that the Python module was rebuilt:

```cmd
dir C:\Users\Brenda\code\EvoEngine\out\build\x64-Release\PythonBinding\*.pyd
```

Expected output:
```
PyDigitalAgriculture.cp39-win_amd64.pyd  (should show recent timestamp)
```

### Step 3: Test New API

```python
import sys
sys.path.append(r'C:\Users\Brenda\code\EvoEngine\out\build\x64-Release\PythonBinding')

import PyDigitalAgriculture

# Check if new API is available
assert hasattr(PyDigitalAgriculture, 'SetEntityRotation')
print("✓ SetEntityRotation API available!")
```

---

## Running the Comparison

### Full Comparison (Recommended)

```cmd
cd C:\Users\Brenda\code\EvoEngine\claude
py -3.9 compare_horizontal_vs_30deg_leaf.py
```

**Expected Runtime:** ~5-10 minutes (15 timesteps × 2 angles)

**Expected Output:**
```
================================================================================
SORGHUM LEAF ANGLE COMPARISON - EvoEngine Ray Tracing
================================================================================
Location: 33.07°N, -111.97°W (Maricopa, AZ)
Date: 2024-06-13
Time range: 6:00 - 20:00 (hourly)
Leaf: 50 cm × 6 cm
Angles: [0, 30]° from horizontal
================================================================================

Step 1: Calculating solar positions...
✓ Generated 15 solar positions
  Daylight hours: 13 (elevation > 0°)

Step 2: Initializing EvoEngine...
✓ EvoEngine initialized with ray tracing enabled

Step 3: Setting up leaf geometry...
✓ Single leaf mesh generated

Step 4: Running ray-traced illumination for 0° (horizontal)...
  Setting leaf rotation to 0° (pitch angle)...
  ✓ Leaf rotated to 0° from horizontal

Ray tracing 0°: 100%|████████| 15/15 [02:30<00:00, PAR=2104]
  Peak PAR: 2104.5 μmol m⁻² s⁻¹

Step 5: Running ray-traced illumination for 30° (tilted)...
  Setting leaf rotation to 30° (pitch angle)...
  ✓ Leaf rotated to 30° from horizontal

Ray tracing 30°: 100%|████████| 15/15 [02:30<00:00, PAR=1823]
  Peak PAR: 1823.4 μmol m⁻² s⁻¹

================================================================================
COMPARISON RESULTS
================================================================================

Daily PAR Integrals (EvoEngine):
  0° (Horizontal):  48.23 mol m⁻² day⁻¹
  30° (Tilted):     43.51 mol m⁻² day⁻¹
  Difference:       4.72 mol m⁻² day⁻¹ (+10.9%)

Peak PAR (EvoEngine):
  0° (Horizontal):  2104.5 μmol m⁻² s⁻¹
  30° (Tilted):     1823.4 μmol m⁻² s⁻¹
  Difference:       281.1 μmol m⁻² s⁻¹ (+15.4%)

Horizontal receives +10.9% more PAR than 30° leaf
================================================================================
```

---

## Expected Results

### Physical Expectations

**Horizontal Leaf (0°):**
- **Peak PAR:** 2000-2200 μmol m⁻² s⁻¹ (at solar noon)
- **Daily integral:** 45-50 mol m⁻² day⁻¹
- **Peak timing:** 12:00 (solar noon)
- **Symmetry:** High (morning ≈ afternoon)

**30° Tilted Leaf:**
- **Peak PAR:** 1700-1900 μmol m⁻² s⁻¹
- **Daily integral:** 40-45 mol m⁻² day⁻¹
- **Peak timing:** May shift slightly from noon
- **Symmetry:** Moderate

**Comparison:**
- Horizontal leaf should receive **10-15% more PAR** than 30° leaf
- Peak PAR difference: **~200-300 μmol m⁻² s⁻¹**
- Both should follow cosine law (with corrections for tilt angle)

---

## Validation Criteria

### ✅ Success Criteria

1. **Non-zero PAR values**
   - Both 0° and 30° should have realistic PAR (>1500 μmol m⁻² s⁻¹ at peak)

2. **Horizontal > Tilted**
   - 0° leaf should receive more total PAR than 30° leaf

3. **Reasonable difference**
   - Daily integral difference: 10-15%
   - Peak PAR difference: 15-20%

4. **EvoEngine vs Analytical agreement**
   - Correlation (R²) > 0.90 for both angles
   - Pattern should match (peak at noon, bell curve)

5. **Ray tracing timing**
   - Should take ~10 seconds per timestep (not instant)
   - Total runtime: ~5-10 minutes

### ⚠️ Warning Signs

- **All zeros:** Mesh not generated or rotation failed
- **No difference between angles:** Rotation API not working
- **Instant execution:** Ray tracing not running
- **Negative PAR:** Physics violation
- **Poor analytical correlation:** Implementation error

---

## Troubleshooting

### Issue: "SetEntityRotation not found"

**Cause:** Python bindings not rebuilt with new API

**Solution:**
```cmd
cd C:\Users\Brenda\code\EvoEngine
build.cmd --no-test
```

Verify timestamp on `.pyd` file is recent.

### Issue: "All PAR values are zero"

**Cause:** Mesh not generated or rotation broke mesh

**Solution:**
1. Check mesh generation output for errors
2. Try without rotation first to verify mesh works
3. Check entity validity in logs

### Issue: "No difference between 0° and 30°"

**Cause:** Rotation not applied or not affecting ray tracing

**Solution:**
1. Verify `SetEntityRotation` is called before ray tracing
2. Check that entity handle is valid
3. Add debug output to verify rotation is applied

### Issue: "Ray tracing too fast (instant)"

**Cause:** Mesh has no geometry

**Solution:**
1. Verify `GenerateSorghumMesh()` completed successfully
2. Check for mesh generation errors in console
3. Try with existing working script first

---

## Scientific Context

### Why This Comparison Matters

**C4 Photosynthesis in Sorghum:**
- Sorghum is a C4 plant with high photosynthetic efficiency
- Light saturation: ~1500-2000 μmol m⁻² s⁻¹
- Leaf angle affects PAR interception and thus photosynthesis

**Leaf Angle Trade-offs:**
- **Horizontal leaves (0°):** Maximum PAR interception, risk of photoinhibition
- **Tilted leaves (30-45°):** Reduced PAR but better canopy penetration
- **Vertical leaves (>60°):** Minimal midday PAR, avoids heat stress

**Agricultural Implications:**
- Leaf angle is a key trait for sorghum breeding
- Affects whole-canopy photosynthesis and water use efficiency
- Important for optimizing planting density

---

## Next Steps

### Additional Experiments

1. **More angles:** Test 0°, 15°, 30°, 45°, 60° to create response curve

2. **Time of day effects:** Compare morning vs noon vs afternoon differences

3. **Multiple leaves:** Test how leaf angle affects whole-plant PAR distribution

4. **Different dates:** Compare summer solstice vs equinox

5. **Cloudy conditions:** Test effect of diffuse radiation

### Code Extensions

**Multi-angle sweep:**
```python
angles = [0, 15, 30, 45, 60, 75, 90]
results = {}
for angle in angles:
    results[angle] = run_raytraced_illumination(sun_positions, entity, angle)
```

**Optimization:**
```python
# Find optimal leaf angle for maximum daily PAR
optimal_angle = find_angle_maximizing_par(sun_positions, entity)
```

---

## Files Modified/Created

### Modified (C++ API):
1. `PythonBinding/include/PyDigitalAgriculture.hpp`
2. `PythonBinding/src/PyDigitalAgriculture.cpp`
3. `PythonBinding/src/PyDigitalAgricultureModule.cpp`

### Created (Python):
1. `claude/compare_horizontal_vs_30deg_leaf.py` - Main comparison script
2. `claude/LEAF_ANGLE_COMPARISON_IMPLEMENTATION.md` - This documentation

### Output (Generated by script):
1. `par_comparison_results/par_horizontal_0deg.csv`
2. `par_comparison_results/par_30deg.csv`
3. `par_comparison_results/par_0deg_vs_30deg_comparison.csv`
4. `par_comparison_results/par_0deg_vs_30deg_comparison.png`

---

## Git Commit Message

```
Add SetEntityRotation API for leaf angle experiments

- Add SetEntityRotation() C++ API to PyDigitalAgriculture
- Expose rotation control (pitch, yaw, roll) to Python
- Create comparison script for 0° vs 30° leaf angles
- Include analytical PAR calculation for validation
- Generate 6-panel comparison visualization

This enables systematic study of leaf angle effects on PAR interception
for sorghum photosynthesis modeling.

Modified:
  PythonBinding/include/PyDigitalAgriculture.hpp
  PythonBinding/src/PyDigitalAgriculture.cpp
  PythonBinding/src/PyDigitalAgricultureModule.cpp

Added:
  claude/compare_horizontal_vs_30deg_leaf.py
  claude/LEAF_ANGLE_COMPARISON_IMPLEMENTATION.md
```

---

**Ready to build and test!** 🚀

Run the rebuild, then execute the comparison script to see how leaf angle affects PAR interception in sorghum.
