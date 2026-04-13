#!/usr/bin/env python3
"""
Debug script to diagnose rotation issue (no Unicode)
"""

import os
from pathlib import Path
import sys

EVOENGINE_DIRECTORY = Path('C:/Users/Brenda/code/EvoEngine')
BUILD_CONFIG = 'x64-Release'
LIBRARY_DIRECTORY = EVOENGINE_DIRECTORY / 'out' / 'build' / BUILD_CONFIG / 'PythonBinding'

sys.path.append(str(LIBRARY_DIRECTORY))
os.chdir(LIBRARY_DIRECTORY)

import PyDigitalAgriculture as sorghum_framework

print("="*80)
print("ROTATION DEBUG TEST")
print("="*80)

# Initialize
print("\n1. Initializing EvoEngine...")
sorghum_framework.PushRayTracerLayer()
sorghum_framework.RegisterClasses()
sorghum_framework.PushSorghumLayer()
sorghum_framework.PushRayTracerLayer()

project_path = str(EVOENGINE_DIRECTORY / 'Resources' / 'DigitalAgricultureProject' / 'test.eveproj')
sorghum_framework.Run(project_path)
print("   OK Initialized")

# Create leaf
print("\n2. Creating sorghum leaf...")
data_gen_params = sorghum_framework.SorghumDataGenerationParameters()
mesh_settings = data_gen_params.sorghum_mesh_generator_settings
mesh_settings.enable_panicle = False
mesh_settings.enable_stem = False
mesh_settings.enable_leaves = True
mesh_settings.enable_leaf_sheath = False
mesh_settings.single_leaf_index = 2
mesh_settings.bottom_face = True
mesh_settings.leaf_separated = True
mesh_settings.leaf_thickness = 0.001

sorghum_state_handle = sorghum_framework.GetAssetHandle("./SorghumGenerator/Sample1.sorghum")
sorghum_entity = sorghum_framework.CreateEntityFromSorghumDescriptor(sorghum_state_handle)
print(f"   OK Created entity: {sorghum_entity}")

print("\n3. Generating mesh...")
sorghum_framework.GenerateSorghumMesh(mesh_settings)
print("   OK Mesh generated")

# Test rotation at 0 degrees
print("\n4. Testing rotation at 0 degrees...")
sorghum_framework.SetEntityRotation(sorghum_entity, 0, 0, 0)
print("   OK Rotation set to 0 degrees")

# Run one illumination test at 0 degrees
print("\n5. Running illumination at 0 degrees (sun at high angle)...")
sorghum_framework.SetSunDirection(180, 80)  # High sun
sorghum_framework.IlluminationEstimationOnSorghum()
result_0deg = sorghum_framework.GetAllIlluminationEstimationResultsOnSorghum()

par_0deg = 0.0
if len(result_0deg) > 0:
    for entity_result in result_0deg:
        if len(entity_result) >= 5:
            area = entity_result[2].x
            if area > 0:
                par_0deg = entity_result[4].x
                break

print(f"   PAR at 0 degrees: {par_0deg:.1f} umol/m2/s")

# Test rotation at 90 degrees (vertical - should be very different!)
print("\n6. Testing rotation at 90 degrees (vertical leaf)...")
sorghum_framework.SetEntityRotation(sorghum_entity, 90, 0, 0)
print("   OK Rotation set to 90 degrees")

# Run same illumination test at 90 degrees
print("\n7. Running illumination at 90 degrees (same sun position)...")
sorghum_framework.SetSunDirection(180, 80)  # Same high sun
sorghum_framework.IlluminationEstimationOnSorghum()
result_90deg = sorghum_framework.GetAllIlluminationEstimationResultsOnSorghum()

par_90deg = 0.0
if len(result_90deg) > 0:
    for entity_result in result_90deg:
        if len(entity_result) >= 5:
            area = entity_result[2].x
            if area > 0:
                par_90deg = entity_result[4].x
                break

print(f"   PAR at 90 degrees: {par_90deg:.1f} umol/m2/s")

# Analysis
print("\n" + "="*80)
print("RESULTS")
print("="*80)
print(f"PAR at 0 deg (horizontal):  {par_0deg:.1f} umol/m2/s")
print(f"PAR at 90 deg (vertical):   {par_90deg:.1f} umol/m2/s")
print(f"Difference:                 {abs(par_0deg - par_90deg):.1f} umol/m2/s")
if par_90deg > 0:
    print(f"Ratio (0deg/90deg):         {par_0deg/par_90deg:.2f}x")

print("\n" + "="*80)
if abs(par_0deg - par_90deg) < 10:
    print("FAIL - ROTATION NOT WORKING")
    print("\nPossible causes:")
    print("  1. Mesh is on a child entity, not the entity we're rotating")
    print("  2. Ray tracer uses mesh-local coordinates, ignoring entity transform")
    print("  3. Rotation is being reset somewhere")
else:
    print("SUCCESS - ROTATION IS WORKING!")
    if par_90deg > 0:
        print(f"\nVertical leaf receives {(1 - par_90deg/par_0deg)*100:.1f}% less PAR than horizontal")
    print("This is physically correct!")
print("="*80)

# Cleanup
sorghum_framework.Terminate()
