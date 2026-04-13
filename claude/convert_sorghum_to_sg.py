"""
Convert improved .sorghum state files to .sg descriptor format.
Handles conversion from explicit leaf specifications to statistical distributions.
"""

import yaml
import os
from pathlib import Path
import numpy as np


def extract_leaf_width_curve(sorghum_data):
    """Extract the width_along_leaf curve from the first leaf as a template."""
    leaves = sorghum_data.get('leaves', [])
    if not leaves:
        return None

    first_leaf = leaves[0]
    width_curve = first_leaf.get('width_along_leaf', {}).get('curve', {})

    # Convert 4-element format to 2-element format for EvoEngine .sg
    if 'values' in width_curve:
        values_4elem = width_curve['values']
        # Extract [x, y] from [[tangent_x, tangent_y, x, y], ...]
        values_2elem = [[v[2], v[3]] for v in values_4elem]

        return {
            'tangent_': True,
            'min_': [0.0, 0.0],
            'max_': [1.0, 1.0],
            'values_': values_2elem
        }
    return None


def analyze_leaves(sorghum_data):
    """Analyze leaf distribution patterns from state file."""
    leaves = sorghum_data.get('leaves', [])

    if not leaves:
        return None

    # Extract data for all leaves
    starting_points = [leaf['starting_point'] for leaf in leaves]
    lengths = [leaf['length'] for leaf in leaves]
    widths = [leaf['width_along_leaf']['max_value'] for leaf in leaves]
    branching_angles = [leaf['branching_angle'] for leaf in leaves]
    roll_angles = [leaf['roll_angle'] for leaf in leaves]
    curling_max = [leaf['curling_along_leaf']['max_value'] for leaf in leaves]
    bending_min = [leaf['bending_along_leaf']['min_value'] for leaf in leaves]
    bending_max = [leaf['bending_along_leaf']['max_value'] for leaf in leaves]
    waviness_max = [leaf['waviness_along_leaf']['max_value'] for leaf in leaves]
    waviness_freq = [leaf['waviness_frequency'] for leaf in leaves]

    return {
        'starting_points': starting_points,
        'lengths': lengths,
        'widths': widths,
        'branching_angles': branching_angles,
        'roll_angles': roll_angles,
        'curling_max': curling_max,
        'bending_min': bending_min,
        'bending_max': bending_max,
        'waviness_max': waviness_max,
        'waviness_freq': waviness_freq,
    }


def create_linear_curve(y_values):
    """Create a linear interpolation curve from y-values (normalized to 0-1)."""
    n = len(y_values)
    if n == 0:
        return [[0.0, 0.5], [1.0, 0.5]]
    elif n == 1:
        return [[0.0, y_values[0]], [1.0, y_values[0]]]

    # Normalize y_values to [0, 1] range
    min_y = min(y_values)
    max_y = max(y_values)

    if max_y == min_y:
        # All values are the same
        normalized = [0.5] * n
    else:
        normalized = [(y - min_y) / (max_y - min_y) for y in y_values]

    # Create curve points evenly spaced in x
    curve_points = []
    for i, y_norm in enumerate(normalized):
        x = i / (n - 1) if n > 1 else 0.5
        curve_points.append([x, y_norm])

    return curve_points, (min_y, max_y)


def convert_to_sg_format(sorghum_path, output_path):
    """Convert a .sorghum state file to .sg descriptor format."""

    # Load the .sorghum file
    with open(sorghum_path, 'r') as f:
        sorghum_data = yaml.safe_load(f)

    # Convert numpy types to native Python types
    def convert_numpy_types(obj):
        """Recursively convert numpy types to Python native types."""
        if isinstance(obj, (np.floating, np.float64, np.float32)):
            return float(obj)
        elif isinstance(obj, (np.integer, np.int64, np.int32)):
            return int(obj)
        elif isinstance(obj, dict):
            return {k: convert_numpy_types(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [convert_numpy_types(item) for item in obj]
        return obj

    # Extract plant name
    plant_name = sorghum_data.get('name', 'Sorghum Plant')

    # Extract stem data
    stem = sorghum_data.get('stem', {})
    stem_length = stem.get('length', 0.5)
    stem_width_min = stem.get('width_along_stem', {}).get('min_value', 0.01)
    stem_width_max = stem.get('width_along_stem', {}).get('max_value', 0.03)
    stem_width = (stem_width_min + stem_width_max) / 2.0

    # Calculate internode length from stem length and leaf count
    leaves = sorghum_data.get('leaves', [])
    num_leaves = len(leaves)
    internode_length = stem_length / max(num_leaves, 1)

    # Extract panicle data
    panicle = sorghum_data.get('panicle', {})
    panicle_size = panicle.get('panicle_size', [0, 0, 0])
    seed_amount = panicle.get('seed_amount', 0)
    seed_radius = panicle.get('seed_radius', 0.002)

    # Analyze leaf patterns
    leaf_analysis = analyze_leaves(sorghum_data)

    if not leaf_analysis:
        print(f"Warning: No leaves found in {sorghum_path}")
        return

    # Extract width_along_leaf curve (standardized from first leaf)
    width_curve = extract_leaf_width_curve(sorghum_data)

    # Create curves for each parameter
    starting_curve, (start_min, start_max) = create_linear_curve(leaf_analysis['starting_points'])
    length_curve, (length_min, length_max) = create_linear_curve(leaf_analysis['lengths'])
    width_curve_dist, (width_min, width_max) = create_linear_curve(leaf_analysis['widths'])
    branching_curve, (branch_min, branch_max) = create_linear_curve(leaf_analysis['branching_angles'])
    roll_curve, (roll_min, roll_max) = create_linear_curve(leaf_analysis['roll_angles'])
    curling_curve, (curl_min, curl_max) = create_linear_curve(leaf_analysis['curling_max'])

    # Create .sg descriptor
    sg_data = {
        'panicle_size': {
            'mean': [panicle_size[0], panicle_size[1]],
            'deviation': 0.0
        },
        'panicle_seed_amount': {
            'mean': seed_amount,
            'deviation': 0
        },
        'panicle_seed_radius': {
            'mean': seed_radius,
            'deviation': 0.0
        },
        'stem_tilt_angle': {
            'mean': 0.0,
            'deviation': 0.0
        },
        'internode_length': {
            'mean': internode_length,
            'deviation': 0.0
        },
        'stem_width': {
            'mean': stem_width,
            'deviation': 0.0
        },
        'leaf_amount': {
            'mean': num_leaves,
            'deviation': 0
        },

        # Leaf starting point distribution
        'leaf_starting_point': {
            'mean': {
                'min_value': start_min,
                'max_value': start_max,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': starting_curve
                }
            },
            'deviation': {
                'min_value': 0.0,
                'max_value': 0.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            }
        },

        # Leaf curling
        'leaf_curling': {
            'mean': {
                'min_value': 0.0,
                'max_value': curl_max,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': curling_curve
                }
            },
            'deviation': {
                'min_value': 0.0,
                'max_value': 0.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            }
        },

        # Leaf roll angle
        'leaf_roll_angle': {
            'mean': {
                'min_value': roll_min,
                'max_value': roll_max,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': roll_curve
                }
            },
            'deviation': {
                'min_value': 0.0,
                'max_value': 0.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            }
        },

        # Leaf branching angle
        'leaf_branching_angle': {
            'mean': {
                'min_value': branch_min,
                'max_value': branch_max,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': branching_curve
                }
            },
            'deviation': {
                'min_value': 0.0,
                'max_value': 0.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            }
        },

        # Leaf bending
        'leaf_bending': {
            'mean': {
                'min_value': np.mean(leaf_analysis['bending_min']),
                'max_value': np.mean(leaf_analysis['bending_max']),
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            },
            'deviation': {
                'min_value': 0.0,
                'max_value': 0.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            }
        },

        # Leaf bending acceleration
        'leaf_bending_acceleration': {
            'mean': {
                'min_value': 0.0,
                'max_value': 1.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            },
            'deviation': {
                'min_value': 0.0,
                'max_value': 0.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            }
        },

        # Leaf bending smoothness
        'leaf_bending_smoothness': {
            'mean': {
                'min_value': 0.0,
                'max_value': 1.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            },
            'deviation': {
                'min_value': 0.0,
                'max_value': 0.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            }
        },

        # Leaf waviness
        'leaf_waviness': {
            'mean': {
                'min_value': 0.0,
                'max_value': np.mean(leaf_analysis['waviness_max']) / 1000.0,  # Convert to 0-0.1 range
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            },
            'deviation': {
                'min_value': 0.0,
                'max_value': 0.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            }
        },

        # Leaf waviness frequency
        'leaf_waviness_frequency': {
            'mean': {
                'min_value': 0.0,
                'max_value': np.mean(leaf_analysis['waviness_freq']),
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            },
            'deviation': {
                'min_value': 0.0,
                'max_value': 0.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            }
        },

        # Leaf length
        'leaf_length': {
            'mean': {
                'min_value': 0.0,
                'max_value': length_max,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': length_curve
                }
            },
            'deviation': {
                'min_value': 0.0,
                'max_value': 0.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            }
        },

        # Leaf width
        'leaf_width': {
            'mean': {
                'min_value': 0.0,
                'max_value': width_max,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': width_curve_dist
                }
            },
            'deviation': {
                'min_value': 0.0,
                'max_value': 0.0,
                'curve': {
                    'tangent_': True,
                    'min_': [0.0, 0.0],
                    'max_': [1.0, 1.0],
                    'values_': [[0.0, 0.5], [1.0, 0.5]]
                }
            }
        },

        # Global curves
        'width_along_stem': {
            'min_value': 0.0,
            'max_value': 1.0,
            'curve': {
                'tangent_': True,
                'min_': [0.0, 0.0],
                'max_': [1.0, 1.0],
                'values_': [[0.0, 0.5], [1.0, 0.5]]
            }
        },

        'width_along_leaf': {
            'min_value': 0.0,
            'max_value': 1.0,
            'curve': width_curve if width_curve else {
                'tangent_': True,
                'min_': [0.0, 0.0],
                'max_': [1.0, 1.0],
                'values_': [[0.0, 0.5], [0.5, 1.0], [1.0, 0.0]]
            }
        },

        'waviness_along_leaf': {
            'min_value': 0.0,
            'max_value': 1.0,
            'curve': {
                'tangent_': True,
                'min_': [0.0, 0.0],
                'max_': [1.0, 1.0],
                'values_': [[0.0, 0.5], [1.0, 0.5]]
            }
        },

        'curling_along_leaf': {
            'min_value': 0.0,
            'max_value': 1.0,
            'curve': {
                'tangent_': True,
                'min_': [0.0, 0.0],
                'max_': [1.0, 1.0],
                'values_': [[0.0, 0.3], [1.0, 0.7]]
            }
        }
    }

    # Convert all numpy types to native Python types before serialization
    sg_data = convert_numpy_types(sg_data)

    # Write to .sg file
    with open(output_path, 'w') as f:
        yaml.dump(sg_data, f, default_flow_style=None, sort_keys=False, allow_unicode=True)

    print(f"[OK] Converted: {os.path.basename(sorghum_path)} -> {os.path.basename(output_path)}")
    print(f"     Leaves: {num_leaves}, Stem length: {stem_length:.3f}m, Max leaf length: {length_max:.3f}m")


def convert_directory(input_dir, output_dir):
    """Convert all .sorghum files in a directory to .sg format."""

    input_path = Path(input_dir)
    output_path = Path(output_dir)

    # Create output directory if it doesn't exist
    output_path.mkdir(parents=True, exist_ok=True)

    # Find all .sorghum files
    sorghum_files = list(input_path.glob("*.sorghum"))

    if not sorghum_files:
        print(f"No .sorghum files found in {input_dir}")
        return

    print(f"Found {len(sorghum_files)} .sorghum files to convert\n")

    # Convert each file
    for sorghum_file in sorted(sorghum_files):
        # Create output filename
        sg_filename = sorghum_file.stem + ".sg"
        sg_filepath = output_path / sg_filename

        try:
            convert_to_sg_format(str(sorghum_file), str(sg_filepath))
        except Exception as e:
            print(f"[ERROR] Error converting {sorghum_file.name}: {e}")

    print(f"\n[DONE] Conversion complete! Output files in: {output_path}")


if __name__ == "__main__":
    # Input and output directories
    input_dir = r"C:\Users\Brenda\Desktop\mf\claude evo engine\improved_evoengine_scene_graphs"
    output_dir = r"C:\Users\Brenda\Desktop\mf\claude evo engine\improved_evoengine_scene_graphs_sg"

    convert_directory(input_dir, output_dir)
