# Rotation Issue Fix

**Problem:** 0° and 30° leaves showed identical PAR values, indicating rotation was not working.

**Root Cause:** The `SetEntityRotation` function was setting `GlobalTransform` instead of `Transform`.

- **GlobalTransform** is a computed/read-only transform that represents the world-space position
- **Transform** is the local transform that should be modified

**Fix Applied:**

Changed line 112 in `PythonBinding/src/PyDigitalAgriculture.cpp`:

```cpp
// BEFORE (incorrect):
auto& global_transform = scene->GetDataComponent<GlobalTransform>(entity);
global_transform.SetEulerRotation(...);

// AFTER (correct):
auto& transform = scene->GetDataComponent<Transform>(entity);
transform.SetEulerRotation(...);
```

**Additional Improvements:**
1. Added `Application::Loop()` call after setting rotation to ensure scene updates
2. Added logging to verify rotation values are being set

---

## Rebuild and Test

### Step 1: Rebuild

```cmd
cd C:\Users\Brenda\code\EvoEngine
build.cmd --no-test
```

### Step 2: Run Diagnostic Test

First, test with extreme rotation (0° vs 90° vertical) to verify rotation works:

```cmd
cd C:\Users\Brenda\code\EvoEngine\claude
py -3.9 debug_rotation_issue.py
```

**Expected output if working:**
```
PAR at 0° (horizontal):  ~1800-2000 μmol m⁻² s⁻¹
PAR at 90° (vertical):   ~200-400 μmol m⁻² s⁻¹
Difference:              ~1500-1700 μmol m⁻² s⁻¹
✅ ROTATION IS WORKING!
```

**If NOT working (same values):**
- Mesh might be on child entity
- Need to rotate all child entities in hierarchy
- Ray tracer might use mesh-local coordinates

### Step 3: Run Full Comparison

Once rotation is verified working:

```cmd
py -3.9 compare_horizontal_vs_30deg_leaf.py
```

**Expected output:**
```
Daily PAR Integrals (EvoEngine):
  0° (Horizontal):  ~48 mol m⁻² day⁻¹
  30° (Tilted):     ~43 mol m⁻² day⁻¹
  Difference:       ~5 mol m⁻² day⁻¹ (+10-12%)

Peak PAR (EvoEngine):
  0° (Horizontal):  ~2100 μmol m⁻² s⁻¹
  30° (Tilted):     ~1800 μmol m⁻² s⁻¹
  Difference:       ~300 μmol m⁻² s⁻¹ (+15%)
```

---

## If Still Not Working

If rotation still shows no difference, the issue might be:

### Issue 1: Mesh on Child Entity

The sorghum entity might have a hierarchy where the actual leaf mesh is on a child entity:

```
SorghumEntity (root) <- We're rotating this
  └─ LeafEntity (child) <- But mesh is here
      └─ Mesh component
```

**Solution:** Need to find and rotate ALL entities with meshes:

```cpp
// Pseudocode - would need C++ implementation
void SetEntityRotationRecursive(Entity entity, pitch, yaw, roll) {
  SetRotation(entity);
  for (child : GetChildren(entity)) {
    SetEntityRotationRecursive(child, pitch, yaw, roll);
  }
}
```

### Issue 2: Ray Tracer Uses Mesh-Local Coordinates

The illumination calculation might be using the mesh's local vertex positions without applying entity transforms.

**Solution:** Would need to modify `SorghumLayer::CalculateIllumination()` to apply entity transforms to meshes before ray tracing.

### Issue 3: Mesh Generated After Rotation

If the mesh is generated after setting rotation, it might override the transform.

**Solution:** Try setting rotation AFTER mesh generation:

```python
# Create and generate mesh first
sorghum_entity = setup_leaf_geometry()  # Generates mesh

# THEN set rotation
PyDigitalAgriculture.SetEntityRotation(sorghum_entity, 30, 0, 0)
```

---

## Verification Steps

1. **Check log output** - Look for "SetEntityRotation: pitch=..." messages
2. **Test extreme angles** - 0° vs 90° should show huge difference
3. **Verify mesh exists** - Ensure PAR values are non-zero
4. **Check timing** - Ray tracing should take ~10 sec/timestep

---

## Files Modified

1. `PythonBinding/src/PyDigitalAgriculture.cpp` - Fixed Transform vs GlobalTransform
2. `claude/debug_rotation_issue.py` - New diagnostic script
3. `claude/ROTATION_FIX.md` - This document
