# EvoEngine Sorghum .sg File Format Reference

## Overview

Sorghum descriptor (.sg) files define the morphological parameters for sorghum plants in EvoEngine. These YAML files control plant structure, leaf geometry, and growth characteristics for visualization and ray tracing simulations.

## Critical Format Requirements

### ⚠️ MOST COMMON ERROR: Array Format

**CORRECT Format (2-element arrays):**
```yaml
values_:
  - [-0.1, 0]      # [x, y]
  - [0, 0.207]     # [x, y]
  - [1, 1]         # [x, y]
```

**INCORRECT Format (4-element arrays - CAUSES CRASH):**
```yaml
values_:
  - - -0.1         # [tangent_x, tangent_y, x, y]
    - 0.0
    - 0.0
    - 0.15
```

**Symptom:** "Debug Assertion Failed: vector subscript out of range"

**Fix:** Use `claude/fix_sg_files.py` to automatically convert 4-element to 2-element format.

## Complete File Structure

```yaml
# Panicle (seed head) parameters
panicle_size:
  mean: [0, 0]              # [width, length] in meters
  deviation: 0

panicle_seed_amount:
  mean: 0                   # Number of seeds
  deviation: 0

panicle_seed_radius:
  mean: 0.002               # Seed radius in meters
  deviation: 0

# Stem parameters
stem_tilt_angle:
  mean: 0                   # Tilt angle in degrees
  deviation: 0

internode_length:
  mean: 0.035               # Distance between leaf nodes (meters)
  deviation: 0.005          # Variation in internode length

stem_width:
  mean: 0.014               # Stem diameter in meters
  deviation: 0

# Leaf count
leaf_amount:
  mean: 7                   # Number of leaves (1-15 typical)
  deviation: 0

# Leaf distribution parameters (all use curve format)
leaf_starting_point:        # Where on stem (0=base, 1=top)
  mean: <curve>
  deviation: <curve>

leaf_curling:               # Curl angle (0-90 degrees)
  mean: <curve>
  deviation: <curve>

leaf_roll_angle:            # Roll around midrib (-1 to 1)
  mean: <curve>
  deviation: <curve>

leaf_branching_angle:       # Angle from vertical (0-55 degrees)
  mean: <curve>
  deviation: <curve>

leaf_bending:               # Drooping angle (-180 to 180 degrees)
  mean: <curve>
  deviation: <curve>

leaf_bending_acceleration:  # Bending rate (0-1)
  mean: <curve>
  deviation: <curve>

leaf_bending_smoothness:    # Bending smoothness (0-1)
  mean: <curve>
  deviation: <curve>

leaf_waviness:              # Waviness magnitude (0-0.1)
  mean: <curve>
  deviation: <curve>

leaf_waviness_frequency:    # Waviness frequency (0-0.1)
  mean: <curve>
  deviation: <curve>

leaf_length:                # Leaf length in meters (0-1.16m typical)
  mean: <curve>
  deviation: <curve>

leaf_width:                 # Leaf width in meters (0-0.1m typical)
  mean: <curve>
  deviation: <curve>

# Global shape curves
width_along_stem:           # How leaf width varies by position on stem
  <single_curve>

width_along_leaf:           # How leaf width varies along leaf length
  <single_curve>

waviness_along_leaf:        # How waviness varies along leaf
  <single_curve>

curling_along_leaf:         # How curling varies along leaf
  <single_curve>
```

## Curve Format

### Standard Parameter Curve (with mean and deviation)

```yaml
parameter_name:
  mean:
    min_value: 0            # Minimum value for this parameter
    max_value: 55           # Maximum value for this parameter
    curve:
      tangent_: true        # Use tangent-based interpolation
      min_: [0, 0]          # Curve domain minimum [x, y]
      max_: [1, 1]          # Curve domain maximum [x, y]
      values_:              # Control points [x, y]
        - [-0.1, 0]         # ⚠️ MUST be 2-element arrays!
        - [0, 0.5]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.2]
        - [0.1, 0]
  deviation:
    min_value: 0
    max_value: 3
    curve:
      tangent_: true
      min_: [0, 0]
      max_: [1, 1]
      values_:
        - [-0.1, 0]
        - [0, 0.67]
        - [0.1, 0]
        - [-0.1, 0]
        - [1, 0.225]
        - [0.1, 0]
```

### Global Shape Curve (single curve, no mean/deviation)

```yaml
width_along_stem:
  tangent_: true
  min_: [0, 0]
  max_: [1, 1]
  values_:
    - [-0.1, 0]
    - [0, 1]
    - [0.1, 0]
    - [-0.1, 0]
    - [1, 0.688]
    - [0.1, 0]
```

## Curve Interpretation

### X-Axis Meaning
- For `leaf_*` parameters: Represents **leaf position along stem**
  - 0.0 = bottom-most (oldest) leaf
  - 1.0 = top-most (youngest) leaf

- For `*_along_leaf` parameters: Represents **position along leaf length**
  - 0.0 = leaf base (attached to stem)
  - 1.0 = leaf tip

- For `*_along_stem`: Represents **height along stem**
  - 0.0 = stem base
  - 1.0 = stem top

### Y-Axis Meaning
- **Normalized value (0-1)** that maps to range `[min_value, max_value]`
- Example: If `leaf_length.max_value = 1.16` and curve `y = 0.5`, actual length = 0.58m

### Tangent Values
- First element in each array pair: **tangent_x** (typically -0.1 or 0.1)
  - Negative = slope down before point
  - Positive = slope up after point
- Used for smooth interpolation between control points

## Realistic Sorghum Parameters

### Growth Stage V7 (7 leaves, ~30 days)
```yaml
leaf_amount: {mean: 7, deviation: 0}
internode_length: {mean: 0.035, deviation: 0.005}
stem_width: {mean: 0.014, deviation: 0}
leaf_length: {mean: {max_value: 0.6}}     # 60cm leaves
leaf_width: {mean: {max_value: 0.07}}     # 7cm width
leaf_branching_angle: {mean: {max_value: 55}}  # 0-55° (older leaves droop more)
```

### Growth Stage V15 (15 leaves, ~60 days)
```yaml
leaf_amount: {mean: 15, deviation: 0}
internode_length: {mean: 0.08, deviation: 0.01}
stem_width: {mean: 0.022, deviation: 0}
leaf_length: {mean: {max_value: 1.0}}     # 1m leaves
leaf_width: {mean: {max_value: 0.09}}     # 9cm width
```

### Single Leaf Testing
```yaml
leaf_amount: {mean: 1, deviation: 0}
internode_length: {mean: 0.08, deviation: 0}
stem_width: {mean: 0.0225, deviation: 0}
leaf_length: {mean: {max_value: 0.79}}
leaf_width: {mean: {max_value: 0.087}}
leaf_branching_angle: {mean: {max_value: 55}}
```

## Typical Value Ranges

| Parameter | Typical Range | Units | Notes |
|-----------|---------------|-------|-------|
| `leaf_amount` | 1-15 | count | 7-10 typical for V7-V10 |
| `internode_length` | 0.03-0.12 | meters | Increases with growth |
| `stem_width` | 0.01-0.025 | meters | Increases with growth |
| `leaf_length` | 0.3-1.2 | meters | Varies by leaf position |
| `leaf_width` | 0.05-0.10 | meters | Varies by leaf position |
| `leaf_branching_angle` | 0-55 | degrees | Younger leaves more upright |
| `leaf_bending` | -180-180 | degrees | Drooping, 0=horizontal |
| `leaf_curling` | 0-90 | degrees | Curling angle |
| `leaf_roll_angle` | -1-1 | normalized | Roll around midrib |
| `leaf_waviness` | 0-0.1 | meters | Surface waviness |

## Common Patterns

### Leaf Size Increases Then Decreases Along Stem
Typical pattern: middle leaves largest, bottom and top smaller

```yaml
leaf_length:
  mean:
    max_value: 1.16
    curve:
      values_:
        - [0, 0.578]      # Bottom leaf: 67cm (0.578 * 1.16)
        - [0.5, 0.8]      # Middle leaf: 93cm (largest)
        - [1, 0.519]      # Top leaf: 60cm (smaller)
```

### Branching Angle Decreases Upward (Younger Leaves More Erect)
```yaml
leaf_branching_angle:
  mean:
    max_value: 55
    curve:
      values_:
        - [0, 1.0]        # Bottom: 55° (drooping)
        - [0.5, 0.7]      # Middle: 38.5°
        - [1, 0.2]        # Top: 11° (upright)
```

### Width Tapers Toward Leaf Tip
```yaml
width_along_leaf:
  values_:
    - [0, 0.315]          # Base: 31.5% of max width
    - [0.6, 0.547]        # Mid: 54.7% (widest point)
    - [1, 0.016]          # Tip: 1.6% (narrow)
```

## Validation Checklist

Before using .sg files in EvoEngine:

- [ ] All `values_` arrays use 2-element `[x, y]` format (NOT 4-element)
- [ ] All curve x-values are in range [0, 1]
- [ ] All curve y-values are in range [0, 1]
- [ ] `min_value < max_value` for all parameters
- [ ] `leaf_amount > 0`
- [ ] `leaf_length.max_value > 0`
- [ ] `leaf_width.max_value > 0`
- [ ] `internode_length > 0`
- [ ] YAML syntax is valid (test with `yaml.safe_load()`)
- [ ] File extension is `.sg`

## Python Generation Template

```python
import yaml

def create_sorghum_sg(
    leaf_count=7,
    leaf_length_max=0.6,
    leaf_width_max=0.07,
    internode_length=0.035,
    output_path="sorghum.sg"
):
    """Generate a basic sorghum .sg file."""

    data = {
        'panicle_size': {'mean': [0, 0], 'deviation': 0},
        'panicle_seed_amount': {'mean': 0, 'deviation': 0},
        'panicle_seed_radius': {'mean': 0.002, 'deviation': 0},
        'stem_tilt_angle': {'mean': 0, 'deviation': 0},
        'internode_length': {'mean': internode_length, 'deviation': 0},
        'stem_width': {'mean': 0.014, 'deviation': 0},
        'leaf_amount': {'mean': leaf_count, 'deviation': 0},

        'leaf_starting_point': {
            'mean': {
                'min_value': 0, 'max_value': 1,
                'curve': {
                    'tangent_': True,
                    'min_': [0, 0], 'max_': [1, 1],
                    'values_': [[0, 0.2], [1, 1]]  # ⚠️ 2-element format!
                }
            },
            'deviation': {
                'min_value': 0, 'max_value': 1,
                'curve': {
                    'tangent_': True,
                    'min_': [0, 0], 'max_': [1, 1],
                    'values_': [[0, 0], [1, 0]]
                }
            }
        },

        'leaf_length': {
            'mean': {
                'min_value': 0, 'max_value': leaf_length_max,
                'curve': {
                    'tangent_': True,
                    'min_': [0, 0], 'max_': [1, 1],
                    'values_': [[0, 0.5], [1, 0.5]]  # Uniform length
                }
            },
            'deviation': {
                'min_value': 0, 'max_value': 0,
                'curve': {
                    'tangent_': True,
                    'min_': [0, 0], 'max_': [1, 1],
                    'values_': [[0, 0.5], [1, 0.5]]
                }
            }
        },

        # ... (add all other required fields)

        'width_along_leaf': {
            'tangent_': True,
            'min_': [0, 0], 'max_': [1, 1],
            'values_': [
                [0, 0.315],
                [0.6, 0.547],
                [1, 0.016]
            ]
        }
    }

    with open(output_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=None, sort_keys=False, width=1000)

    print(f"Generated: {output_path}")

# Usage
create_sorghum_sg(
    leaf_count=7,
    leaf_length_max=0.6,
    leaf_width_max=0.07,
    output_path="my_sorghum.sg"
)
```

## Troubleshooting

### Problem: "Debug Assertion Failed: vector subscript out of range"
**Cause:** 4-element arrays in `values_`

**Solution:**
```bash
python claude/fix_sg_files.py path/to/directory
```

### Problem: Plant doesn't appear in EvoEngine
**Check:**
1. `leaf_amount.mean > 0`
2. `leaf_length.mean.max_value > 0`
3. `leaf_width.mean.max_value > 0`
4. File is in correct directory for project

### Problem: Plant looks unrealistic
**Check:**
1. Physical dimensions reasonable (see Typical Value Ranges)
2. Curve y-values in [0, 1] range
3. Branching angles not inverted
4. Width curves make sense (taper toward tip)

## Reference Files

**Working Examples:**
- `Resources/DigitalAgricultureProject/Assets/SorghumGenerator/7leaf-target.sg`
- `Resources/DigitalAgricultureProject/Assets/SorghumGenerator/claude/single_leaf_growth_day4.sg`

**Fixed Examples:**
- `Resources/DigitalAgricultureProject/Assets/SorghumGenerator/claude/scenegraphcompare/*.sg`

## Tools

**Fix Script:** `claude/fix_sg_files.py`
- Automatically converts 4-element to 2-element format
- Creates .backup files
- Usage: `python fix_sg_files.py [directory]`

**Slash Command:** `/sg-generator`
- Expert guidance for creating .sg files
- Built-in validation rules
- Example templates

**Python API:**
```python
import PyDigitalAgriculture

# Load and instantiate
PyDigitalAgriculture.GenerateSorghumMesh("path/to/file.sg")
```

## Version History

- **v1.0** (2025-11-10): Initial format documentation
- Fixed 21 files in `scenegraphcompare/` directory (4-element → 2-element conversion)

## Questions?

See:
- `CLAUDE.md` - EvoEngine build and architecture
- `TRANSFER_TO_EVOENGINE_MACHINE.md` - PAR calculation context
- EvoEngine source: `EvoEngine_Plugins/DigitalAgriculture/src/lib/SorghumDescriptor.cpp`
