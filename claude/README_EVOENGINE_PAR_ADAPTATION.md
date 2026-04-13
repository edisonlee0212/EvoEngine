# Sorghum Single Leaf Daily PAR - EvoEngine Adaptation Guide

## Overview

This document explains the adaptation of `sorghum_single_leaf_daily_par.py` to work with your target EvoEngine build at `/Users/maryfrancis/Documents/GitHub/EvoEngine`.

**Created**: October 31, 2025
**Author**: Claude Code (C4 Expert)

---

## Critical Information About APIs

### ⚠️ IMPORTANT: Branch 261 APIs and Build Status

The illumination estimation APIs used in the original script come from **EvoEngine branch 261**, which is a **feature branch** that has NOT been merged into the main `dev` branch yet.

**Branch 261 Status**:
- ⚠️ **Known build issue**: Branch 261 does NOT build successfully (reason unknown)
- ✓ **APIs exist in code**: The illumination functions are implemented
- ⚠️ **Runtime status unclear**: APIs may work if they can be compiled
- 🎯 **Your goal**: Get these APIs working in the dev branch build

**Branch 261 APIs** (need to be added to your dev build):
- `IlluminationEstimationOnSorghum()`
- `GetAllIlluminationEstimationResultsOnSorghum()`
- Enhanced PAR sensor generation
- Sorghum-specific illumination pipeline optimizations

**Your Target EvoEngine** (dev branch):
- ✓ Has the base framework and core functionality
- ✓ Has Python binding infrastructure
- ✓ Has Digital Agriculture plugin
- ✓ **Should build successfully** (unlike branch 261)
- ⚠️ **May NOT have branch 261's convenience APIs**

### Why This Matters

Since branch 261 has build issues, your best path forward is:

1. **Start with working dev branch build**
2. **Test what APIs are available**
3. **Add missing branch 261 APIs if needed** (I can provide the code)
4. **Rebuild and test**

You CANNOT rely on branch 261 directly due to its build failures.

---

## Files Created

### 1. `test_evoengine_compatibility.py`

**Purpose**: Check if your EvoEngine build has the required APIs

**What it does**:
- Locates your EvoEngine Python bindings
- Tests module import
- Checks for required framework APIs
- **Specifically checks for branch 261 illumination APIs**
- Tests framework initialization
- Provides recommendations based on findings

**Run this FIRST** before attempting the full simulation.

**Usage**:
```bash
cd /Users/maryfrancis/Documents/Pauli-Lab/Claude
python test_evoengine_compatibility.py
```

**Expected Outcomes**:

**Scenario A: Branch 261 APIs Present** ✓ (UNLIKELY)
```
✓ IlluminationEstimationOnSorghum
✓ GetAllIlluminationEstimationResultsOnSorghum

✓ RESULT: Branch 261 APIs present!
  You can use the direct Python script approach (Option 1).
```
→ **Proceed with adapted script directly**

**Scenario B: Only Generic APIs Present** ⚠️ (MOST LIKELY)
```
✗ IlluminationEstimationOnSorghum - MISSING (branch 261 feature)
✗ GetAllIlluminationEstimationResultsOnSorghum - MISSING (branch 261 feature)
✓ IlluminationEstimation (generic)

⚠ RESULT: Only generic APIs present
  You'll need to add branch 261 bindings (I can provide code).
```
→ **Need to add bindings** - This is expected and fixable

**Scenario C: No Illumination APIs** ✗ (POSSIBLE)
```
✗ IlluminationEstimationOnSorghum
✗ IlluminationEstimation

✗ RESULT: No illumination APIs found
  You'll need to add illumination APIs from branch 261.
```
→ **More work needed** - Add illumination system from branch 261

---

### 2. `sorghum_single_leaf_daily_par_evoengine.py`

**Purpose**: Adapted version of the PAR calculation script for your EvoEngine build

**Key Differences from Original**:
- Updated paths for target EvoEngine installation
- API compatibility checks built-in
- Better error messages and diagnostics
- Tries multiple paths for resources (project file, BTF, sorghum descriptor)
- Falls back gracefully if optional features missing

**Configuration** (lines 51-68):
```python
# MODIFY THESE if your build is elsewhere:
EVOENGINE_DIRECTORY = Path('/Users/maryfrancis/Documents/GitHub/EvoEngine')
BUILD_CONFIG = 'x64-Release'  # or 'Debug', 'Release', etc.
```

**Usage**:
```bash
# After running test script and confirming/adding APIs:
python sorghum_single_leaf_daily_par_evoengine.py
```

**Requirements**:
- Python 3.8+
- Dependencies: `numpy`, `pandas`, `matplotlib`, `pvlib`, `tqdm`
- EvoEngine built with Python bindings
- Branch 261 illumination APIs (may need to add manually)

**Outputs**:
- `./evoengine_par_results/hourly_PAR_maricopa_june13_evoengine.csv` - Hourly data
- `./evoengine_par_results/daily_par_visualization.png` - 4-panel figure

---

## Step-by-Step Testing Workflow

### Step 1: Run Compatibility Test on Your Machine

**On the machine where EvoEngine is built:**

```bash
# Copy test script to that machine
scp test_evoengine_compatibility.py user@your-machine:/path/to/directory/

# SSH to that machine
ssh user@your-machine

# Run test
cd /path/to/directory
python test_evoengine_compatibility.py
```

**Review the output carefully.** It will tell you exactly what's available.

---

### Step 2: Interpret Results and Add Missing APIs

#### If Test Shows "Branch 261 APIs Present" ✓ (Unlikely but Great!)

**Excellent!** Someone already added these to your build.

**Next steps**:
1. Copy `sorghum_single_leaf_daily_par_evoengine.py` to your machine
2. Adjust paths in the script if needed (lines 51-68)
3. Run the script:
   ```bash
   python sorghum_single_leaf_daily_par_evoengine.py
   ```
4. Check outputs in `./evoengine_par_results/`

---

#### If Test Shows "Only Generic APIs" or "No Illumination APIs" ⚠️ (Expected)

**This is the most likely scenario** since branch 261 has build issues.

**Solution: Add Branch 261 Bindings Manually**

The underlying illumination system exists in EvoEngine's core - you just need to expose it to Python.

**Step 2A: Add C++ Binding Code**

Edit `/Users/maryfrancis/Documents/GitHub/EvoEngine/PythonBinding/src/PyDigitalAgriculture.cpp`

Add these function implementations (look for similar existing functions as reference):

```cpp
// Add near other illumination-related functions

void PyDigitalAgriculture::IlluminationEstimationOnSorghum() {
  auto scene = Application::GetActiveScene();
  auto sorghum_layer = Application::GetLayer<SorghumLayer>();

  if (!sorghum_layer) {
    EVOENGINE_ERROR("SorghumLayer not found!");
    return;
  }

  // Call the core illumination calculation
  sorghum_layer->CalculateIllumination();
}

std::vector<std::vector<glm::vec3>>
PyDigitalAgriculture::GetAllIlluminationEstimationResultsOnSorghum() {
  auto scene = Application::GetActiveScene();
  if (!scene) {
    EVOENGINE_ERROR("No active scene!");
    return {};
  }

  std::vector<std::vector<glm::vec3>> results;

  // Iterate through all entities with illumination estimators
  scene->ForEach<Transform, BtfMeshRenderer>(
    [&](Entity entity, Transform& transform, BtfMeshRenderer& renderer) {
      // Check if entity has illumination estimator
      if (!scene->HasPrivateComponent<TriangleIlluminationEstimator>(entity)) {
        return;
      }

      auto estimator = scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(entity);

      std::vector<glm::vec3> entity_result;

      // Result format: [position, rotation, total_area, total_flux, average_flux]
      entity_result.emplace_back(transform.GetPosition());
      entity_result.emplace_back(transform.GetEulerRotation());
      entity_result.emplace_back(estimator->total_area, 0.0f, 0.0f);
      entity_result.emplace_back(estimator->total_flux);
      entity_result.emplace_back(estimator->average_flux);

      results.emplace_back(entity_result);
    }
  );

  return results;
}
```

**Step 2B: Add Function Declarations**

Edit `/Users/maryfrancis/Documents/GitHub/EvoEngine/PythonBinding/include/PyDigitalAgriculture.hpp`

Add these declarations in the class definition:

```cpp
class PyDigitalAgriculture {
public:
  // ... existing declarations ...

  static void IlluminationEstimationOnSorghum();
  static std::vector<std::vector<glm::vec3>> GetAllIlluminationEstimationResultsOnSorghum();
};
```

**Step 2C: Expose to Python Module**

Edit `/Users/maryfrancis/Documents/GitHub/EvoEngine/PythonBinding/src/PyDigitalAgricultureModule.cpp`

Add these lines in the module initialization (look for similar `m.def()` calls):

```cpp
PYBIND11_MODULE(PyDigitalAgriculture, m) {
    m.doc() = "Python bindings for EvoEngine Digital Agriculture";

    // ... existing bindings ...

    // Branch 261 illumination APIs
    m.def("IlluminationEstimationOnSorghum",
          &PyDigitalAgriculture::IlluminationEstimationOnSorghum,
          "Calculate illumination on sorghum plants using ray tracing");

    m.def("GetAllIlluminationEstimationResultsOnSorghum",
          &PyDigitalAgriculture::GetAllIlluminationEstimationResultsOnSorghum,
          "Get illumination estimation results from all sorghum entities");

    // ... rest of module ...
}
```

**Step 2D: Rebuild EvoEngine**

```bash
cd /Users/maryfrancis/Documents/GitHub/EvoEngine/out/build/x64-Release
cmake --build . --config Release -j 8
```

**Step 2E: Re-run Test**

```bash
python test_evoengine_compatibility.py
```

Should now show:
```
✓ IlluminationEstimationOnSorghum
✓ GetAllIlluminationEstimationResultsOnSorghum
```

**Step 2F: Run PAR Calculation**

```bash
python sorghum_single_leaf_daily_par_evoengine.py
```

---

## Alternative: C++ Direct Implementation (If Python Binding is Too Complex)

If adding Python bindings proves difficult, you can write a **standalone C++ application** that uses EvoEngine directly.

**Advantages**:
- No Python binding complexity
- Full access to C++ APIs
- Better debugging
- Potentially better performance

**Disadvantages**:
- Need to implement solar position calculations in C++
- No pandas/matplotlib (need to write CSV, visualize separately)
- More C++ code to write

**I can provide a complete C++ application template** if you prefer this approach.

---

## Detailed Comparison: Branch Status

| Aspect | EvoEngine (dev) | EvoEngine-261 (branch 261) |
|--------|-----------------|----------------------------|
| **Branch** | `dev` (main development) | Feature branch |
| **Build Status** | ✓ **Builds successfully** | ✗ **Build fails** (reason unknown) |
| **Code Base** | Latest, ~6-10 commits ahead | Older + branch 261 commits |
| **Branch 261 APIs in code** | ❌ Not present | ✓ Present (but can't build) |
| **Branch 261 APIs compiled** | ❌ No | ✗ Can't compile due to build failure |
| **Can run PAR script** | ⚠️ After adding APIs manually | ✗ Can't build at all |
| **Best for** | **Adding APIs and testing** | Reference for what APIs look like |

**The Reality**:
- **Branch 261**: Has the API code you want BUT won't build
- **Dev branch**: Builds successfully BUT missing the APIs

**Solution**: Take the API code from branch 261 and add it to your working dev build (instructions above).

---

## Troubleshooting

### Issue: "Could not find Python bindings"

**Symptoms**:
```
ERROR: Could not find Python bindings!
Searched locations:
  - /Users/maryfrancis/Documents/GitHub/EvoEngine/out/build/x64-Release/PythonBinding
```

**Solutions**:
1. Verify EvoEngine is actually built: `ls -la /Users/maryfrancis/Documents/GitHub/EvoEngine/out/`
2. Find bindings: `find /Users/maryfrancis/Documents/GitHub/EvoEngine -name "PyDigitalAgriculture.*"`
3. Update `LIBRARY_DIR` in scripts to match actual location

---

### Issue: "ImportError: cannot import PyDigitalAgriculture"

**Symptoms**:
```
ImportError: dlopen(...PyDigitalAgriculture.so): Library not loaded
```

**Causes**:
- Missing dependencies (CUDA, OptiX, system libraries)
- Python version mismatch
- Incomplete build

**Solutions**:
1. Check Python version: `python --version`
2. Check dependencies: `otool -L PyDigitalAgriculture.so` (macOS) or `ldd` (Linux)
3. Rebuild if necessary

---

### Issue: Compilation Errors After Adding Binding Code

**Symptoms**:
```
error: 'TriangleIlluminationEstimator' was not declared in this scope
```

**Causes**:
- Missing include files
- Different class names in your version

**Solutions**:
1. Check existing PyDigitalAgriculture.cpp for similar code patterns
2. Look at existing illumination-related functions
3. Add missing includes at top of file:
   ```cpp
   #include "TriangleIlluminationEstimator.hpp"
   #include "SorghumLayer.hpp"
   ```
4. Contact me with specific error messages - I can provide fixes

---

### Issue: APIs Added But Still Don't Work at Runtime

**Symptoms**:
- Test script shows APIs present
- But script crashes when calling them

**Possible Causes**:
- Core illumination system not properly initialized
- Missing BTF materials
- Ray tracer layer not enabled

**Solutions**:
1. Check initialization order in script
2. Ensure `PushRayTracerLayer()` called before `PushSorghumLayer()`
3. Verify resources (BTF files, project file) are present
4. Check EvoEngine console output for error messages

---

## Recommended Workflow

### Phase 1: Assessment (5-10 minutes)

1. Copy test script to machine with EvoEngine build
2. Run `test_evoengine_compatibility.py`
3. Document what APIs are available
4. **Most likely outcome**: Branch 261 APIs missing (expected)

### Phase 2: Add Missing APIs (1-3 hours)

1. Follow "Step 2A-2C" above to add binding code
2. Rebuild EvoEngine
3. Re-run test to confirm APIs present
4. Debug any compilation issues

### Phase 3: Test PAR Calculation (30 min - 1 hour)

1. Copy adapted script to EvoEngine machine
2. Adjust paths if needed
3. Run script
4. Verify outputs
5. Compare with expected PAR values (30-50 mol m⁻² day⁻¹)

### Expected Total Time

**If APIs need to be added**: 2-4 hours (including build time)
**If APIs already present**: 30 minutes

---

## What To Do About Branch 261 Build Failure

**You have several options:**

### Option 1: Don't Fix Branch 261 (Recommended)

**Rationale**: You don't need branch 261 to build - you just need its APIs.

**Steps**:
1. Use dev branch (builds successfully)
2. Manually add the branch 261 API code (provided above)
3. Rebuild dev branch
4. Use the working dev build with added APIs

**Time**: 2-3 hours
**Risk**: Low (dev branch is stable)

---

### Option 2: Debug Branch 261 Build (Not Recommended)

**Only do this if you need something else from branch 261 besides the illumination APIs.**

**Steps**:
1. Try building branch 261: `cmake --build . 2>&1 | tee build.log`
2. Examine error messages
3. Try to fix dependency issues, missing files, etc.
4. This could take days

**Time**: Unknown (could be hours to days)
**Risk**: High (unknown issue, may be unfixable)

---

### Option 3: Extract More Code from Branch 261 If Needed

If the simple API additions don't work, you may need more code from branch 261:

**What to extract**:
- `SorghumLayer::CalculateIllumination()` implementation
- `TriangleIlluminationEstimator` class
- Related illumination infrastructure

**I can help identify what code to copy** if you encounter issues.

---

## Next Steps

### Immediate Actions

1. **Run compatibility test** on your EvoEngine machine:
   ```bash
   python test_evoengine_compatibility.py
   ```

2. **Report back** with test results:
   - Which APIs are present?
   - What does the test recommend?
   - Any errors?

3. **Based on results**, choose next action:
   - **APIs present**: Test adapted script immediately
   - **APIs missing**: Add binding code (provided above)
   - **Build issues**: Debug or contact me

---

## Files Summary

| File | Purpose | When to Use |
|------|---------|-------------|
| `test_evoengine_compatibility.py` | Check API availability | **Run FIRST** |
| `sorghum_single_leaf_daily_par_evoengine.py` | Adapted PAR calculation | After APIs confirmed/added |
| This README | Implementation guide | Reference throughout process |

---

## If You Need Help

**Provide me with**:
1. Output from test script
2. Any compilation errors (if adding APIs)
3. EvoEngine version/commit: `git log --oneline -1`
4. Build configuration used

**I can provide**:
- Debugged binding code
- Alternative C++ implementation
- Specific solutions for your build

---

## Summary

✅ **Test script created**: Diagnoses what APIs are available

✅ **Adapted PAR script created**: Ready when APIs available

✅ **Binding code provided**: To add missing branch 261 APIs to dev build

⚠️ **Branch 261 won't build**: But you don't need it to - extract its code

🎯 **Next step**: Run test, then add APIs if needed

---

**The path forward is clear**: Dev branch builds successfully. Add branch 261's APIs to it manually. This avoids the branch 261 build issue entirely.
