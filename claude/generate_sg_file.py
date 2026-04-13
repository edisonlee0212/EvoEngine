#!/usr/bin/env python3
"""
Generate EvoEngine-compatible Sorghum .sg descriptor files.

This script creates properly formatted .sg files with the correct 2-element
array format for curve control points.

Usage:
    python generate_sg_file.py --output my_sorghum.sg --leaves 7 --length 0.6
"""

import argparse
import yaml
from pathlib import Path


def create_curve(values_xy, min_val=0, max_val=1):
    """
    Create a curve dictionary with proper 2-element [x, y] format.

    Args:
        values_xy: List of [x, y] pairs (2-element arrays)
        min_val: Minimum value for this parameter
        max_val: Maximum value for this parameter
    """
    return {
        'min_value': min_val,
        'max_value': max_val,
        'curve': {
            'tangent_': True,
            'min_': [0, 0],
            'max_': [1, 1],
            'values_': values_xy  # CRITICAL: Must be 2-element arrays!
        }
    }


def create_simple_curve(values_xy):
    """Create a simple curve (no min/max_value) for global shape parameters."""
    return {
        'tangent_': True,
        'min_': [0, 0],
        'max_': [1, 1],
        'values_': values_xy
    }


def create_parameter(mean_curve, dev_curve):
    """Create a parameter with mean and deviation curves."""
    return {
        'mean': mean_curve,
        'deviation': dev_curve
    }


def generate_sorghum_sg(
    leaf_count=7,
    leaf_length_max=0.6,
    leaf_width_max=0.07,
    internode_length=0.035,
    stem_width=0.014,
    branching_angle_max=55,
    growth_stage_days=None,
    output_path="sorghum.sg"
):
    """
    Generate a sorghum .sg file with proper EvoEngine format.

    Args:
        leaf_count: Number of leaves (1-15)
        leaf_length_max: Maximum leaf length in meters
        leaf_width_max: Maximum leaf width in meters
        internode_length: Distance between nodes in meters
        stem_width: Stem diameter in meters
        branching_angle_max: Maximum branching angle in degrees
        growth_stage_days: If specified, adjusts parameters for growth day
        output_path: Output file path
    """

    # Adjust parameters based on growth stage if specified
    if growth_stage_days is not None:
        # Simple growth model: scale leaf size with days
        leaf_length_max = 0.3 + growth_stage_days * 0.05  # 0.3-1.0m over 14 days
        leaf_width_max = 0.05 + growth_stage_days * 0.003  # 0.05-0.09m
        internode_length = 0.03 + growth_stage_days * 0.004  # 0.03-0.09m
        stem_width = 0.012 + growth_stage_days * 0.0008  # 0.012-0.023m

    # Define the complete sorghum descriptor
    data = {
        # Panicle (seed head) parameters
        'panicle_size': {'mean': [0, 0], 'deviation': 0},
        'panicle_seed_amount': {'mean': 0, 'deviation': 0},
        'panicle_seed_radius': {'mean': 0.002, 'deviation': 0},

        # Stem parameters
        'stem_tilt_angle': {'mean': 0, 'deviation': 0},
        'internode_length': {'mean': internode_length, 'deviation': 0.0},
        'stem_width': {'mean': stem_width, 'deviation': 0},

        # Leaf count
        'leaf_amount': {'mean': leaf_count, 'deviation': 0},

        # Leaf starting position along stem (0=base, 1=top)
        'leaf_starting_point': create_parameter(
            create_curve([[0, 0.2], [1, 1]], 0, 1),
            create_curve([[0, 0], [1, 0]], 0, 1)
        ),

        # Leaf curling (0-90 degrees)
        'leaf_curling': create_parameter(
            create_curve([[0, 0.3], [1, 0.7]], 0, 90),
            create_curve([[0, 0], [1, 0]], 0, 1)
        ),

        # Leaf roll angle (-1 to 1)
        'leaf_roll_angle': create_parameter(
            create_curve([[0, 0.5], [1, 0.5]], -1, 1),
            create_curve([[0, 0], [1, 0]], 0, 6)
        ),

        # Branching angle (0-55 degrees) - younger leaves more upright
        'leaf_branching_angle': create_parameter(
            create_curve([[0, 1.0], [0.5, 0.7], [1, 0.5]], 0, branching_angle_max),
            create_curve([[0, 0], [1, 0]], 0, 3)
        ),

        # Leaf bending (drooping)
        'leaf_bending': create_parameter(
            create_curve([[0, 0.829], [1, 0.521]], -180, 180),
            create_curve([[0, 0.5], [1, 0.5]], 0, 0)
        ),

        # Bending acceleration
        'leaf_bending_acceleration': create_parameter(
            create_curve([[0, 0.781], [1, 0.812]], 0, 1),
            create_curve([[0, 0.5], [1, 0.5]], 0, 0)
        ),

        # Bending smoothness
        'leaf_bending_smoothness': create_parameter(
            create_curve([[0, 0.5], [1, 0.5]], 0, 1),
            create_curve([[0, 0.5], [1, 0.5]], 0, 0)
        ),

        # Leaf waviness
        'leaf_waviness': create_parameter(
            create_curve([[0, 0.799], [1, 0.044]], 0, 0.1),
            create_curve([[0, 0.5], [1, 0.5]], 0, 0)
        ),

        # Waviness frequency
        'leaf_waviness_frequency': create_parameter(
            create_curve([[0, 0.5], [1, 0.5]], 0, 0.1),
            create_curve([[0, 0.5], [1, 0.5]], 0, 0)
        ),

        # Leaf length - middle leaves largest
        'leaf_length': create_parameter(
            create_curve([[0, 0.578], [0.5, 0.8], [1, 0.519]], 0, leaf_length_max),
            create_curve([[0, 0.5], [1, 0.5]], 0, 0)
        ),

        # Leaf width
        'leaf_width': create_parameter(
            create_curve([[0, 0.5], [1, 0.5]], 0, leaf_width_max),
            create_curve([[0, 0.5], [1, 0.5]], 0, 0)
        ),

        # Global shape curves
        'width_along_stem': create_simple_curve([
            [-0.1, 0], [0, 1], [0.1, 0],
            [-0.1, 0], [1, 0.688], [0.1, 0]
        ]),

        'width_along_leaf': create_simple_curve([
            [-0.1, 0], [0, 0.315], [0.253, 0.233],
            [-0.1, 0], [0.594, 0.547], [0.2, 0.001],
            [-0.205, 0.290], [1, 0.016], [0.1, 0]
        ]),

        'waviness_along_leaf': create_simple_curve([
            [-0.1, 0], [0, 0], [0.1, 0],
            [-0.1, 0], [0.793, 0.505], [0.073, 0],
            [-0.1, 0], [1, 0], [0.1, 0]
        ]),

        'curling_along_leaf': create_simple_curve([
            [-0.1, 0], [0, 0], [0.1, 0],
            [-0.1, 0], [1, 0], [0.1, 0]
        ])
    }

    # Write to file
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=None, sort_keys=False, width=1000)

    print(f"Generated: {output_path}")
    print(f"  - Leaves: {leaf_count}")
    print(f"  - Max leaf length: {leaf_length_max:.3f}m")
    print(f"  - Max leaf width: {leaf_width_max:.3f}m")
    print(f"  - Internode length: {internode_length:.3f}m")
    print(f"  - Stem width: {stem_width:.3f}m")
    if growth_stage_days:
        print(f"  - Growth day: {growth_stage_days}")

    return output_path


def main():
    parser = argparse.ArgumentParser(
        description='Generate EvoEngine-compatible sorghum .sg files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic 7-leaf sorghum
  python generate_sg_file.py --output my_sorghum.sg

  # Single leaf for testing
  python generate_sg_file.py --output single_leaf.sg --leaves 1 --length 0.8

  # Growth series
  python generate_sg_file.py --output day1.sg --growth-day 1
  python generate_sg_file.py --output day7.sg --growth-day 7

  # Custom parameters
  python generate_sg_file.py --output custom.sg --leaves 10 --length 0.9 --width 0.08
        """
    )

    parser.add_argument('--output', '-o', default='sorghum.sg',
                        help='Output .sg file path (default: sorghum.sg)')
    parser.add_argument('--leaves', '-n', type=int, default=7,
                        help='Number of leaves (default: 7)')
    parser.add_argument('--length', '-l', type=float, default=0.6,
                        help='Maximum leaf length in meters (default: 0.6)')
    parser.add_argument('--width', '-w', type=float, default=0.07,
                        help='Maximum leaf width in meters (default: 0.07)')
    parser.add_argument('--internode', '-i', type=float, default=0.035,
                        help='Internode length in meters (default: 0.035)')
    parser.add_argument('--stem-width', '-s', type=float, default=0.014,
                        help='Stem width in meters (default: 0.014)')
    parser.add_argument('--branching-angle', '-b', type=float, default=55,
                        help='Maximum branching angle in degrees (default: 55)')
    parser.add_argument('--growth-day', '-g', type=int, default=None,
                        help='Growth stage day (overrides other parameters)')

    args = parser.parse_args()

    generate_sorghum_sg(
        leaf_count=args.leaves,
        leaf_length_max=args.length,
        leaf_width_max=args.width,
        internode_length=args.internode,
        stem_width=args.stem_width,
        branching_angle_max=args.branching_angle,
        growth_stage_days=args.growth_day,
        output_path=args.output
    )


if __name__ == '__main__':
    main()
