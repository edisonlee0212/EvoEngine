#!/usr/bin/env python3
"""
Apply realistic sorghum leaf shape characteristics to .sg files.

This script updates .sg files with realistic parameters from 7leaf-target.sg
while preserving leaf length and internode length from the original files.

Issues fixed:
1. Leaf bending (adds realistic droop)
2. Leaf waviness (reduces from 20 to 0.1)
3. Width along leaf (adds realistic taper with bulge in middle)
4. Width along stem (adds variation)
5. Roll angle deviation (increases for natural variation)
6. Leaf roll angle deviation (adds natural variation)

Usage:
    python apply_realistic_sorghum_shape.py [directory]
"""

import os
import sys
import yaml
import shutil
from pathlib import Path


# Realistic parameters from 7leaf-target.sg
REALISTIC_PARAMS = {
    'leaf_roll_angle': {
        'deviation': {
            'max_value': 12,  # Was 6, increase for variation
            'curve': {
                'values_': [
                    [-0.1, 0],
                    [0, 0.3],
                    [0.1, 0],
                    [-0.1, 0],
                    [1, 1],
                    [0.1, 0]
                ]
            }
        }
    },

    'leaf_bending': {
        'mean': {
            'curve': {
                'values_': [
                    [-0.1, 0],
                    [0, 0.829],  # Realistic droop
                    [0.9, -0.0097],
                    [-0.1, 0.0069],
                    [1, 0.521],
                    [0.1, 0]
                ]
            }
        }
    },

    'leaf_bending_acceleration': {
        'mean': {
            'curve': {
                'values_': [
                    [-0.1, 0],
                    [0, 0.781],
                    [0.1, 0],
                    [-0.1, 0],
                    [1, 0.812],
                    [0.1, 0]
                ]
            }
        }
    },

    'leaf_waviness': {
        'mean': {
            'max_value': 0.1,  # Was 20! Way too high
            'curve': {
                'values_': [
                    [-0.1, 0],
                    [0, 0.799],
                    [0.1, 0],
                    [-0.102, 0.389],
                    [1, 0.044],
                    [0.1, 0]
                ]
            }
        }
    },

    'width_along_stem': {
        'values_': [
            [-0.1, 0],
            [0, 1],
            [0.1, 0],
            [-0.1, 0],
            [1, 0.688],
            [0.1, 0]
        ]
    },

    'width_along_leaf': {
        'values_': [
            [-0.1, 0],
            [0, 0.315],
            [0.253, 0.233],
            [-0.1, 0],
            [0.594, 0.547],  # Bulge in middle
            [0.2, 0.001],
            [-0.205, 0.290],
            [1, 0.016],  # Pointy tip!
            [0.1, 0]
        ]
    },

    'waviness_along_leaf': {
        'values_': [
            [-0.1, 0],
            [0, 0],
            [0.1, 0],
            [-0.1, 0],
            [0.793, 0.505],
            [0.073, 0],
            [-0.1, 0],
            [1, 0],
            [0.1, 0]
        ]
    }
}


def apply_realistic_shape(data):
    """
    Apply realistic sorghum shape parameters while preserving
    leaf_length and internode_length.
    """

    # Update leaf_roll_angle deviation
    if 'leaf_roll_angle' in data and 'deviation' in data['leaf_roll_angle']:
        data['leaf_roll_angle']['deviation']['max_value'] = \
            REALISTIC_PARAMS['leaf_roll_angle']['deviation']['max_value']
        data['leaf_roll_angle']['deviation']['curve']['values_'] = \
            REALISTIC_PARAMS['leaf_roll_angle']['deviation']['curve']['values_']

    # Update leaf_bending for realistic droop
    if 'leaf_bending' in data and 'mean' in data['leaf_bending']:
        data['leaf_bending']['mean']['curve']['values_'] = \
            REALISTIC_PARAMS['leaf_bending']['mean']['curve']['values_']

    # Update leaf_bending_acceleration
    if 'leaf_bending_acceleration' in data and 'mean' in data['leaf_bending_acceleration']:
        data['leaf_bending_acceleration']['mean']['curve']['values_'] = \
            REALISTIC_PARAMS['leaf_bending_acceleration']['mean']['curve']['values_']

    # Fix leaf_waviness (was way too high at 20!)
    if 'leaf_waviness' in data and 'mean' in data['leaf_waviness']:
        data['leaf_waviness']['mean']['max_value'] = \
            REALISTIC_PARAMS['leaf_waviness']['mean']['max_value']
        data['leaf_waviness']['mean']['curve']['values_'] = \
            REALISTIC_PARAMS['leaf_waviness']['mean']['curve']['values_']

    # Fix width_along_stem (was all zeros)
    if 'width_along_stem' in data:
        data['width_along_stem']['values_'] = \
            REALISTIC_PARAMS['width_along_stem']['values_']

    # Fix width_along_leaf (makes pointy tips and realistic shape)
    if 'width_along_leaf' in data:
        data['width_along_leaf']['values_'] = \
            REALISTIC_PARAMS['width_along_leaf']['values_']

    # Fix waviness_along_leaf
    if 'waviness_along_leaf' in data:
        data['waviness_along_leaf']['values_'] = \
            REALISTIC_PARAMS['waviness_along_leaf']['values_']

    return data


def fix_sg_file(filepath):
    """Apply realistic sorghum shape to a single .sg file."""
    print(f"Processing: {filepath}")

    # Create backup
    backup_path = str(filepath) + ".shape_backup"
    shutil.copy2(filepath, backup_path)
    print(f"  Backup created: {backup_path}")

    # Load YAML
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)

    # Store original values we want to preserve
    orig_leaf_length_max = data.get('leaf_length', {}).get('mean', {}).get('max_value')
    orig_internode_length = data.get('internode_length', {}).get('mean')

    # Apply realistic shape
    fixed_data = apply_realistic_shape(data)

    # Write back
    with open(filepath, 'w') as f:
        yaml.dump(fixed_data, f, default_flow_style=None, sort_keys=False, width=1000)

    print(f"  Applied realistic sorghum shape")
    print(f"    Preserved leaf_length.max_value: {orig_leaf_length_max}")
    print(f"    Preserved internode_length: {orig_internode_length}")
    print()


def main():
    if len(sys.argv) > 1:
        directory = Path(sys.argv[1])
    else:
        # Default to scenegraphcompare directory
        directory = Path(r"C:\Users\Brenda\code\EvoEngine\Resources\DigitalAgricultureProject\Assets\SorghumGenerator\claude\scenegraphcompare")

    if not directory.exists():
        print(f"ERROR: Directory not found: {directory}")
        sys.exit(1)

    print("Applying realistic sorghum shape characteristics")
    print("=" * 60)
    print("Changes:")
    print("  - Leaf bending: Added realistic droop")
    print("  - Leaf waviness: Reduced from 20 to 0.1")
    print("  - Width along leaf: Pointy tips + bulge in middle")
    print("  - Width along stem: Added variation")
    print("  - Roll angle deviation: Increased for natural variation")
    print()
    print(f"Processing directory: {directory}")
    print("=" * 60)

    # Find all .sg files
    sg_files = list(directory.glob("*.sg"))

    if not sg_files:
        print("No .sg files found!")
        sys.exit(1)

    print(f"Found {len(sg_files)} .sg files\n")

    # Process each file
    for sg_file in sorted(sg_files):
        try:
            fix_sg_file(sg_file)
        except Exception as e:
            print(f"  ERROR: Failed to process {sg_file}: {e}")
            print()

    print("=" * 60)
    print(f"Completed! Processed {len(sg_files)} files")
    print("\nBackup files created with .shape_backup extension")
    print("If everything works, you can delete the .shape_backup files")


if __name__ == "__main__":
    main()
