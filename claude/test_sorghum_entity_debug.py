#!/usr/bin/env python3
"""
Debug script to check if sorghum entities are being created properly
"""
import sys
import os
from pathlib import Path

# Setup paths
EVOENGINE_DIRECTORY = Path('C:/Users/Brenda/code/EvoEngine')
LIBRARY_DIRECTORY = EVOENGINE_DIRECTORY / 'out' / 'build' / 'x64-Release' / 'PythonBinding'

sys.path.append(str(LIBRARY_DIRECTORY))
os.chdir(LIBRARY_DIRECTORY)

import PyDigitalAgriculture as sorghum_framework

print("="*80)
print("SORGHUM ENTITY DEBUG TEST")
print("="*80)

# Initialize
project_path = EVOENGINE_DIRECTORY / 'Resources' / 'DigitalAgricultureProject' / 'test.eveproj'
sorghum_framework.PushRayTracerLayer()
sorghum_framework.RegisterClasses()
sorghum_framework.PushSorghumLayer()
sorghum_framework.PushRayTracerLayer()
sorghum_framework.Run(str(project_path))

print("\n✓ EvoEngine initialized\n")

# Create sorghum entity
sorghum_handle = sorghum_framework.GetAssetHandle("./SorghumGenerator/Sample1.sorghum")
print(f"Sorghum asset handle: {sorghum_handle}")

sorghum_entity = sorghum_framework.CreateEntityFromSorghumDescriptor(sorghum_handle)
print(f"Created entity: {sorghum_entity}")

# Generate mesh
print("\nGenerating mesh...")
mesh_settings = sorghum_framework.SorghumMeshGeneratorSettings()
mesh_settings.enable_panicle = False
mesh_settings.enable_stem = False
mesh_settings.enable_leaves = True
mesh_settings.enable_leaf_sheath = False
mesh_settings.single_leaf_index = 2
mesh_settings.bottom_face = True
mesh_settings.leaf_separated = True
mesh_settings.leaf_thickness = 0.001

sorghum_framework.GenerateSorghumMesh(mesh_settings)
print("✓ Mesh generated")

# Set sun direction
print("\nSetting sun direction (azimuth=180, elevation=45)...")
sorghum_framework.SetSunDirection(180.0, 45.0)
print("✓ Sun direction set")

# Run illumination
print("\nRunning illumination estimation...")
sorghum_framework.IlluminationEstimationOnSorghum()
print("✓ Illumination estimation complete")

# Get results
print("\nRetrieving results...")
results = sorghum_framework.GetAllIlluminationEstimationResultsOnSorghum()
print(f"Number of result entities: {len(results)}")

if len(results) > 0:
    print(f"\nFirst entity results:")
    print(f"  Number of data items: {len(results[0])}")
    for i, item in enumerate(results[0]):
        print(f"  Item {i} ({type(item).__name__}): x={item.x}, y={item.y}, z={item.z}")

    print(f"\nSecond entity results (if exists):")
    if len(results) > 1:
        print(f"  Number of data items: {len(results[1])}")
        for i, item in enumerate(results[1]):
            print(f"  Item {i} ({type(item).__name__}): x={item.x}, y={item.y}, z={item.z}")

    print(f"\nInterpreting results for first entity:")
    print(f"  Position: ({results[0][0].x:.3f}, {results[0][0].y:.3f}, {results[0][0].z:.3f})")
    print(f"  Rotation: ({results[0][1].x:.1f}°, {results[0][1].y:.1f}°, {results[0][1].z:.1f}°)")
    print(f"  Total area: {results[0][2].x:.6f} m²")
    print(f"  Total flux: ({results[0][3].x:.3f}, {results[0][3].y:.3f}, {results[0][3].z:.3f})")
    print(f"  Average flux (PAR): {results[0][4].x:.3f} μmol m⁻² s⁻¹")
else:
    print("\n❌ NO RESULTS RETURNED!")
    print("\nPossible issues:")
    print("  1. Entity doesn't have Sorghum component")
    print("  2. Entity doesn't have TriangleIlluminationEstimator component")
    print("  3. Illumination calculation failed silently")

# Cleanup
sorghum_framework.Terminate()
print("\n" + "="*80)
