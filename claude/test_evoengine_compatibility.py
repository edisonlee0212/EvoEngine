#!/usr/bin/env python3
"""
EvoEngine API Compatibility Test Script

This script checks whether your built EvoEngine has the necessary APIs
for sorghum single leaf daily PAR calculation.

IMPORTANT: The IlluminationEstimationOnSorghum() API is from branch 261,
which is a feature branch not yet merged into dev. If your build is from
the dev branch, this API will likely be MISSING.

Run this script FIRST before attempting the full PAR calculation to determine
which implementation approach you need to use.

Author: Claude Code (C4 Expert)
Date: October 31, 2025
"""

import sys
import os
from pathlib import Path

# =============================================================================
# Configuration - MODIFY THESE PATHS FOR YOUR SYSTEM
# =============================================================================

# Path to your EvoEngine installation
EVOENGINE_DIR = Path('C:/Users/Brenda/code/EvoEngine')

# Build configuration (adjust based on your build)
# Options: 'Debug', 'Release', 'RelWithDebInfo', 'x64-Release', etc.
BUILD_CONFIG = 'x64-Release'

# Full path to Python bindings
LIBRARY_DIR = EVOENGINE_DIR / 'out' / 'build' / BUILD_CONFIG / 'PythonBinding'

# Alternative common paths if above doesn't work
ALTERNATIVE_PATHS = [
    EVOENGINE_DIR / 'build' / 'PythonBinding',
    EVOENGINE_DIR / 'build' / 'Release' / 'PythonBinding',
    EVOENGINE_DIR / 'out' / 'build' / 'Release' / 'PythonBinding',
]

# =============================================================================
# Test Functions
# =============================================================================

def find_library_directory():
    """Find the Python binding library directory"""
    print("="*70)
    print("SEARCHING FOR EVOENGINE PYTHON BINDINGS")
    print("="*70)

    # Try primary path
    if LIBRARY_DIR.exists():
        print(f"✓ Found at: {LIBRARY_DIR}")
        return LIBRARY_DIR

    print(f"✗ Not found at: {LIBRARY_DIR}")
    print("\nTrying alternative locations...")

    # Try alternatives
    for alt_path in ALTERNATIVE_PATHS:
        if alt_path.exists():
            print(f"✓ Found at: {alt_path}")
            return alt_path
        print(f"  ✗ {alt_path}")

    print("\n" + "="*70)
    print("ERROR: Could not find Python bindings!")
    print("="*70)
    print("\nPlease ensure EvoEngine is built with Python bindings enabled.")
    print("Update LIBRARY_DIR in this script to match your build location.")
    print("\nExpected directory should contain:")
    print("  - PyDigitalAgriculture.so (macOS/Linux)")
    print("  - PyDigitalAgriculture.pyd (Windows)")
    print("  - Resources/ directory")
    sys.exit(1)

def check_module_file(library_dir):
    """Check if Python module file exists"""
    print("\n" + "="*70)
    print("TEST 1: PYTHON MODULE FILE")
    print("="*70)

    # Check for platform-specific extensions (including versioned names like .cp39-win_amd64.pyd)
    import glob

    patterns = [
        str(library_dir / 'PyDigitalAgriculture*.so'),
        str(library_dir / 'PyDigitalAgriculture*.pyd'),
        str(library_dir / 'PyDigitalAgriculture*.dylib'),
    ]

    found = False
    for pattern in patterns:
        matches = glob.glob(pattern)
        if matches:
            module_path = Path(matches[0])
            print(f"✓ Module file found: {module_path.name}")
            print(f"  Size: {module_path.stat().st_size / 1024:.1f} KB")
            found = True
            break

    if not found:
        print("✗ Python module file not found!")
        print(f"  Searched for: PyDigitalAgriculture*.{{so,pyd,dylib}}")
        print(f"  In directory: {library_dir}")
        sys.exit(1)

    return True

def import_module(library_dir):
    """Import PyDigitalAgriculture module"""
    print("\n" + "="*70)
    print("TEST 2: MODULE IMPORT")
    print("="*70)

    # Add to path and change directory (required for resource loading)
    sys.path.insert(0, str(library_dir))
    os.chdir(library_dir)

    try:
        import PyDigitalAgriculture as sda
        print("✓ PyDigitalAgriculture imported successfully")
        return sda
    except ImportError as e:
        print(f"✗ Failed to import PyDigitalAgriculture")
        print(f"  Error: {e}")
        print("\nPossible causes:")
        print("  1. Missing dependencies (CUDA, OptiX, system libraries)")
        print("  2. Incorrect Python version (check build Python version)")
        print("  3. Incomplete build")
        sys.exit(1)

def check_core_api(sda):
    """Check core framework APIs"""
    print("\n" + "="*70)
    print("TEST 3: CORE FRAMEWORK API")
    print("="*70)

    core_functions = [
        'PushRayTracerLayer',
        'RegisterClasses',
        'PushSorghumLayer',
        'Run',
        'Terminate',
    ]

    all_present = True
    for func in core_functions:
        if hasattr(sda, func):
            print(f"  ✓ {func}")
        else:
            print(f"  ✗ {func} - MISSING")
            all_present = False

    if not all_present:
        print("\n✗ Core framework APIs incomplete!")
        print("  This build may not be functional.")
        sys.exit(1)

    print("\n✓ Core framework APIs complete")
    return True

def check_sorghum_api(sda):
    """Check sorghum-specific APIs"""
    print("\n" + "="*70)
    print("TEST 4: SORGHUM API")
    print("="*70)

    sorghum_functions = [
        'GetAssetHandle',
        'CreateEntityFromSorghumDescriptor',
        'InitiateSorghumEntity',
        'EnableBTF',
        'SetCBTFGroup',
        'SetSkyDome',
        'SetSunDirection',
    ]

    all_present = True
    for func in sorghum_functions:
        if hasattr(sda, func):
            print(f"  ✓ {func}")
        else:
            print(f"  ✗ {func} - MISSING")
            all_present = False

    if not all_present:
        print("\n⚠ Some sorghum APIs missing - reduced functionality")
    else:
        print("\n✓ Sorghum APIs complete")

    return all_present

def check_illumination_api(sda):
    """Check illumination estimation APIs (branch 261 specific)"""
    print("\n" + "="*70)
    print("TEST 5: ILLUMINATION ESTIMATION API (BRANCH 261)")
    print("="*70)
    print("\n⚠ IMPORTANT: These APIs are from branch 261 feature branch.")
    print("  If your build is from 'dev' branch, these will be MISSING.\n")

    # Branch 261 specific APIs
    branch_261_functions = [
        'IlluminationEstimationOnSorghum',
        'GetAllIlluminationEstimationResultsOnSorghum',
    ]

    has_branch_261 = True
    for func in branch_261_functions:
        if hasattr(sda, func):
            print(f"  ✓ {func}")
        else:
            print(f"  ✗ {func} - MISSING (branch 261 feature)")
            has_branch_261 = False

    # Check for alternative generic APIs
    generic_functions = [
        'IlluminationEstimation',
        'CalculateIllumination',
    ]

    has_generic = False
    print("\nChecking for generic illumination APIs:")
    for func in generic_functions:
        if hasattr(sda, func):
            print(f"  ✓ {func}")
            has_generic = True
        else:
            print(f"  ✗ {func}")

    # Determine outcome
    print("\n" + "-"*70)
    if has_branch_261:
        print("✓ RESULT: Branch 261 APIs present!")
        print("  You can use the direct Python script approach (Option 1).")
        return 'branch_261'
    elif has_generic:
        print("⚠ RESULT: Only generic APIs present")
        print("  You'll need to add branch 261 bindings or use C++ wrapper (Option 2).")
        return 'generic'
    else:
        print("✗ RESULT: No illumination APIs found")
        print("  You'll need to implement C++ integration (Option 2 or 3).")
        return 'none'

def check_resources(library_dir):
    """Check if required resources exist"""
    print("\n" + "="*70)
    print("TEST 6: RESOURCE FILES")
    print("="*70)

    # Key resources needed
    resources_to_check = [
        'Resources/DigitalAgricultureProject/test.eveproj',
        'BTFGroup.cbtfgroup',
        'SorghumGenerator/Sample1.sorghum',
    ]

    all_present = True
    for resource in resources_to_check:
        resource_path = library_dir / resource
        # Also check in parent directories
        if not resource_path.exists():
            resource_path = library_dir.parent.parent.parent / 'Resources' / 'DigitalAgricultureProject' / resource.split('/')[-1]

        if resource_path.exists():
            print(f"  ✓ {resource}")
        else:
            print(f"  ⚠ {resource} - NOT FOUND")
            print(f"    Searched: {resource_path}")
            all_present = False

    if not all_present:
        print("\n⚠ Some resources missing - may cause runtime errors")
        print("  Resources should be copied to build directory during build.")
    else:
        print("\n✓ All required resources found")

    return all_present

def test_initialization(sda):
    """Test basic framework initialization"""
    print("\n" + "="*70)
    print("TEST 7: FRAMEWORK INITIALIZATION")
    print("="*70)
    print("\nAttempting to initialize EvoEngine framework...")
    print("(This may take a few seconds)\n")

    try:
        # Find project file
        project_paths = [
            LIBRARY_DIR / 'Resources' / 'DigitalAgricultureProject' / 'test.eveproj',
            EVOENGINE_DIR / 'Resources' / 'DigitalAgricultureProject' / 'test.eveproj',
        ]

        project_path = None
        for p in project_paths:
            if p.exists():
                project_path = str(p)
                print(f"Using project file: {p}")
                break

        if not project_path:
            print("⚠ Project file not found, using default...")
            project_path = "./Resources/DigitalAgricultureProject/test.eveproj"

        # Initialize
        sda.PushRayTracerLayer()
        print("  ✓ Ray tracer layer pushed")

        sda.RegisterClasses()
        print("  ✓ Classes registered")

        sda.PushSorghumLayer()
        print("  ✓ Sorghum layer pushed")

        sda.Run(project_path)
        print("  ✓ Framework started")

        # Clean shutdown
        sda.Terminate()
        print("  ✓ Framework terminated cleanly")

        print("\n✓ Initialization test PASSED")
        return True

    except Exception as e:
        print(f"\n✗ Initialization test FAILED")
        print(f"  Error: {e}")
        import traceback
        traceback.print_exc()
        return False

def list_all_functions(sda):
    """List all available functions in module"""
    print("\n" + "="*70)
    print("BONUS: ALL AVAILABLE FUNCTIONS")
    print("="*70)

    all_attrs = dir(sda)
    functions = [attr for attr in all_attrs if not attr.startswith('_') and callable(getattr(sda, attr, None))]

    print(f"\nTotal functions available: {len(functions)}\n")
    for func in sorted(functions):
        print(f"  • {func}")

def generate_recommendations(api_status):
    """Generate recommendations based on test results"""
    print("\n" + "="*70)
    print("RECOMMENDATIONS")
    print("="*70)

    if api_status == 'branch_261':
        print("""
✓ GOOD NEWS: Your EvoEngine has branch 261 illumination APIs!

You can proceed with Option 1 (direct Python script):
  1. Use the adapted script: sorghum_single_leaf_daily_par_evoengine.py
  2. It should work with minimal modifications
  3. Only path changes needed

Next steps:
  → Run: python sorghum_single_leaf_daily_par_evoengine.py
""")

    elif api_status == 'generic':
        print("""
⚠ PARTIAL SUPPORT: Generic illumination APIs found, but not branch 261 APIs.

You have two options:

OPTION A: Add branch 261 Python bindings (RECOMMENDED)
  1. Merge branch 261 into your dev build
  2. Or add the missing Python bindings manually
     → I can provide the C++ code to add to PyDigitalAgriculture.cpp

OPTION B: Use C++ wrapper approach
  1. Write C++ code that calls the underlying illumination functions
  2. Create thin Python script that uses the wrapper
  3. More work but gives full control

Recommendation: Try Option A first - it's simpler.
""")

    else:  # 'none'
        print("""
✗ NO ILLUMINATION API: You'll need to implement from scratch.

Your options:

OPTION 1: Add branch 261 code to your build
  1. Checkout branch 261 from EvoEngine repository
  2. Merge the illumination estimation code
  3. Rebuild with Python bindings

OPTION 2: Write C++ application directly
  1. Create standalone C++ application
  2. Use EvoEngine core APIs directly
  3. Skip Python wrapper entirely

OPTION 3: Use EvoEngine-261 instead
  1. Use the known-working EvoEngine-261 build
  2. Update it to latest dev features if needed

Recommendation: Option 3 is fastest if you just need results now.
                Option 1 is best for long-term integration.
""")

# =============================================================================
# Main Test Sequence
# =============================================================================

def main():
    print("""
╔══════════════════════════════════════════════════════════════════════╗
║                                                                      ║
║        EVOENGINE API COMPATIBILITY TEST FOR SORGHUM PAR              ║
║                                                                      ║
║  This script checks if your EvoEngine build has the necessary APIs  ║
║  for sorghum single leaf daily PAR calculation.                     ║
║                                                                      ║
╚══════════════════════════════════════════════════════════════════════╝
""")

    print(f"Target EvoEngine: {EVOENGINE_DIR}")
    print(f"Expected build config: {BUILD_CONFIG}")
    print()

    # Run tests
    library_dir = find_library_directory()
    check_module_file(library_dir)
    sda = import_module(library_dir)
    check_core_api(sda)
    check_sorghum_api(sda)
    api_status = check_illumination_api(sda)
    check_resources(library_dir)
    init_success = test_initialization(sda)

    # Optional: list all functions
    print("\n" + "="*70)
    response = input("List all available functions? (y/N): ").strip().lower()
    if response == 'y':
        list_all_functions(sda)

    # Generate recommendations
    generate_recommendations(api_status)

    # Final summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"  Library location: {library_dir}")
    print(f"  Module import: ✓")
    print(f"  Core APIs: ✓")
    print(f"  Illumination API status: {api_status.upper()}")
    print(f"  Initialization test: {'✓' if init_success else '✗'}")
    print("="*70)

    if api_status == 'branch_261' and init_success:
        print("\n🎉 SUCCESS! Your build is ready for PAR calculation.")
        return 0
    elif api_status == 'generic':
        print("\n⚠ PARTIAL: You need to add branch 261 bindings.")
        return 1
    else:
        print("\n✗ NOT READY: Significant work needed.")
        return 2

if __name__ == '__main__':
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
        sys.exit(130)
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
