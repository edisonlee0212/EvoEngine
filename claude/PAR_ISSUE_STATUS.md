# PAR Calculation Status - FINAL DIAGNOSIS

## ROOT CAUSE: Illumination Estimation Doesn't Sample Direct Sunlight

**The existing ray tracer CAN calculate direct sunlight, but illumination estimation doesn't use it.**

### Why PAR Values Are Constant

Illumination estimation (`IlluminationEstimation.cu`) shoots random hemisphere rays from leaf surfaces. When rays miss geometry, they call `MissFunc` which returns only environmental/ambient light - NO direct sun contribution.

Direct sun calculation EXISTS in `ClosestHitFunc` (RayFunctions.cuh:57-79) for SingleLightSource mode, BUT it only triggers when rays HIT geometry. A single leaf in empty space has most rays missing everything.

### Existing Direct Sun Implementation

```cpp
// RayFunctions.cuh:57-79
if (environment.environmental_lighting_type == EnvironmentalLightingType::SingleLightSource) {
    // Sample toward sun with shadow ray
    glm::vec3 newRayDirection = RandomSampleHemisphere(..., environment.sun_direction, 1.0f - environment.light_size);
    // Trace shadow ray to check occlusion
    optixTrace(..., OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT, ...);
    // Add direct sun if not occluded
    energy += perRayData.energy * NdotL * albedoColor;
}
```

**This code works for camera rendering (surfaces hit each other), but not for single-leaf illumination (most rays miss).**

## Fixed (2025-01-03)
- ✅ Python script extracts PAR from `result[i][4].x` (average_flux)
- ✅ Skips empty entities (checks `area > 0`)
- ✅ Added `Application::Loop()` calls after `IlluminationEstimationOnSorghum()`
- ✅ Results successfully retrieved and exported to CSV
- ✅ **Found the ray tracer!** OptiX uses `RayTracerLayer::environment_properties.sun_direction`, NOT DirectionalLight
- ✅ Now updating `environment_properties.sun_direction` in `SetSunDirection()` (line 402)
- ✅ PAR increased 35x (from 0.21 to 7.5 μmol m⁻² s⁻¹)

## Required Fix

**Add direct sun sampling to `IlluminationEstimation.cu`**

### Option 1: Explicit Sun Sampling (Recommended)
Modify `IlluminationEstimation.cu` to shoot explicit shadow rays toward the sun:

```cuda
// After hemisphere sampling loop, add:
if (environment.environmental_lighting_type == EnvironmentalLightingType::SingleLightSource) {
    // Explicit sun shadow ray
    glm::vec3 sunDir = -environment.sun_direction;  // Point TO sun
    float NdotL = glm::dot(normal, sunDir);
    if (NdotL > 0.0f) {
        // Trace shadow ray
        optixTrace(..., sunDir, OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT, ...);
        // Add direct sun contribution
        pointEnergy += perRayData.energy * NdotL * environment.skylight_intensity;
    }
}
```

### Option 2: Modify MissFunc
Add direct sun calculation to `MissFunc` for SingleLightSource mode.

### Option 3: Use Scene Lights
Implement scene-based light sampling that queries DirectionalLight entities.

### Files Modified
```
SorghumLayer.cpp (line 379-424)
  - SetSunDirection() now updates environment_properties.sun_direction
  - Sets environment_properties.skylight_intensity = 2000.0f

PyDigitalAgriculture.cpp (line 75-76)
  - Added Application::Loop() calls

sorghum_single_leaf_daily_par_evoengine.py (line 414-442)
  - Fixed PAR extraction, skips empty entities
```

### Key Discovery
**The ray tracer uses a separate lighting system!**
- `DirectionalLight` entity → for rendering/display only
- `RayTracerLayer::environment_properties` → for OptiX ray tracing
- Must update BOTH for consistent visuals + accurate simulation

### Test
```bash
cd C:\Users\Brenda\code\EvoEngine\claude
env PYTHONIOENCODING=utf-8 py -3.9 sorghum_single_leaf_daily_par_evoengine.py
# Check: Peak PAR should be ~1500-2000, currently 7.5 (constant)
```

## Summary

**Ray tracing works, but illumination estimation only captures ambient/diffuse light.**

Current PAR values (0.8-0.9 μmol m⁻² s⁻¹):
- ✅ Update with sun angle (small variation)
- ❌ 2000x too low (should be 1500-2000 at noon)
- ❌ No direct sun contribution

**To fix:** Modify `IlluminationEstimation.cu` to explicitly sample sun direction with shadow rays, OR use existing camera rendering path which already has this functionality.

**Alternative:** Check if there's a different illumination estimation mode or parameter that enables direct light sampling that we haven't found yet.
