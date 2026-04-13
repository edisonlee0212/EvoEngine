#!/usr/bin/env python3
"""
Fix Sorghum .sg files with 4-element array format to 2-element format.

This script converts curve control point arrays from:
  [tangent_x, tangent_y, x, y]
to:
  [x, y]

Usage:
    python fix_sg_files.py [directory]
"""

import os
import sys
import yaml
import shutil
from pathlib import Path


def fix_curve_values(data):
    """
    Recursively fix curve values_ arrays in the data structure.
    Convert 4-element arrays [tangent_x, tangent_y, x, y] to [x, y].
    """
    if isinstance(data, dict):
        for key, value in data.items():
            if key == "values_" and isinstance(value, list):
                # Check if this is the problematic format
                fixed_values = []
                for item in value:
                    if isinstance(item, list):
                        if len(item) == 4:
                            # Extract last 2 elements (x, y coordinates)
                            fixed_values.append([item[2], item[3]])
                        elif len(item) == 2:
                            # Already correct format
                            fixed_values.append(item)
                        else:
                            print(f"  WARNING: Unexpected array length {len(item)}: {item}")
                            fixed_values.append(item)
                    else:
                        fixed_values.append(item)
                data[key] = fixed_values
            else:
                fix_curve_values(value)
    elif isinstance(data, list):
        for item in data:
            fix_curve_values(item)

    return data


def fix_sg_file(filepath):
    """Fix a single .sg file."""
    print(f"Processing: {filepath}")

    # Create backup
    backup_path = str(filepath) + ".backup"
    shutil.copy2(filepath, backup_path)
    print(f"  Backup created: {backup_path}")

    # Load YAML
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)

    # Fix the data
    fixed_data = fix_curve_values(data)

    # Write back
    with open(filepath, 'w') as f:
        yaml.dump(fixed_data, f, default_flow_style=None, sort_keys=False, width=1000)

    print(f"  Fixed and saved: {filepath}")


def main():
    if len(sys.argv) > 1:
        directory = Path(sys.argv[1])
    else:
        # Default to scenegraphcompare directory
        directory = Path(r"C:\Users\Brenda\code\EvoEngine\Resources\DigitalAgricultureProject\Assets\SorghumGenerator\claude\scenegraphcompare")

    if not directory.exists():
        print(f"ERROR: Directory not found: {directory}")
        sys.exit(1)

    print(f"Fixing .sg files in: {directory}")
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
            print()
        except Exception as e:
            print(f"  ERROR: Failed to process {sg_file}: {e}")
            print()

    print("=" * 60)
    print(f"Completed! Processed {len(sg_files)} files")
    print("\nBackup files created with .backup extension")
    print("If everything works, you can delete the .backup files")


if __name__ == "__main__":
    main()
