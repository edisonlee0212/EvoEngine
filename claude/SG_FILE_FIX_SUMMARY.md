# Sorghum .sg File Fix Summary

## Issues Fixed

### Issue 1: "Vector subscript out of range" Crash
**Problem:** Files in `scenegraphcompare/` used 4-element arrays `[tangent_x, tangent_y, x, y]` instead of the required 2-element format `[x, y]`.

**Solution:** Created `fix_sg_files.py` to automatically convert all curve `values_` arrays from 4-element to 2-element format.

**Files Fixed:** All 21 files in `scenegraphcompare/` directory
- analytical_day1-7.sg
- azmet_day1-7.sg
- evoengine_day1-7.sg

**Result:** ✅ Files now load without crashing in EvoEngine

---

### Issue 2: Unrealistic Plant Appearance
**Problem:** Plants looked nothing like sorghum:
- Leaves had no droop/bending
- Leaves were not pointy at tips
- No realistic waviness
- Two leaves on each side (not alternating)
- Flat, stiff appearance

**Root Causes Identified:**

1. **Leaf Bending**: Flat at 0.5 (no droop)
   - Should be: 0.829 → 0.521 (realistic drooping curve)

2. **Leaf Waviness**: max_value was 20 (way too high!)
   - Should be: 0.1

3. **Width Along Leaf**: Linear taper (0.5 → 0.1)
   - Should be: Bulge in middle + pointy tip (0.315 → 0.547 → 0.016)

4. **Width Along Stem**: All zeros (no variation)
   - Should be: 1.0 → 0.688 (realistic variation)

5. **Roll Angle Deviation**: Only 6 (too uniform)
   - Should be: 12 (natural variation)

**Solution:** Created `apply_realistic_sorghum_shape.py` to apply realistic parameters from `7leaf-target.sg` while preserving original leaf lengths and stem dimensions.

**Changes Applied:**
- ✅ Realistic leaf droop/bending
- ✅ Pointy leaf tips (1.6% width at tip)
- ✅ Bulge in leaf middle (54.7% width at 59% position)
- ✅ Proper waviness magnitude (0.1 instead of 20)
- ✅ Stem width variation
- ✅ Natural roll angle variation (deviation: 12)

**Files Fixed:** Same 21 files in `scenegraphcompare/`

**Result:** ✅ Plants now look realistic with proper sorghum morphology

---

## Files Created

### 1. `fix_sg_files.py`
**Purpose:** Fix 4-element array format crash

**Usage:**
```bash
python fix_sg_files.py [directory]
# Default: scenegraphcompare directory
```

**What it does:**
- Converts `[tangent_x, tangent_y, x, y]` → `[x, y]`
- Creates `.backup` files
- Processes all .sg files in directory

### 2. `apply_realistic_sorghum_shape.py`
**Purpose:** Apply realistic leaf and stem morphology

**Usage:**
```bash
python apply_realistic_sorghum_shape.py [directory]
# Default: scenegraphcompare directory
```

**What it does:**
- Applies realistic bending curves from 7leaf-target.sg
- Adds pointy leaf tips
- Fixes waviness magnitude
- Preserves original leaf_length and internode_length
- Creates `.shape_backup` files

### 3. `generate_sg_file.py`
**Purpose:** Generate new .sg files with correct format

**Usage:**
```bash
# Basic usage
python generate_sg_file.py --output my_plant.sg

# Single leaf for testing
python generate_sg_file.py --output single_leaf.sg --leaves 1 --length 0.8

# Growth series
python generate_sg_file.py --output day1.sg --growth-day 1

# Custom parameters
python generate_sg_file.py --output custom.sg --leaves 10 --length 0.9 --width 0.08
```

**Features:**
- Always generates 2-element `[x, y]` format (no crashes)
- Includes realistic shape parameters by default
- Supports growth stage simulation
- Full parameter customization

### 4. `.claude/commands/sg-generator.md`
**Purpose:** Slash command for expert .sg file generation guidance

**Usage:**
```
/sg-generator
```

**Provides:**
- Complete format specification
- Common parameters and patterns
- Validation checklist
- Troubleshooting guide
- Example code templates

### 5. `SG_FILE_FORMAT_REFERENCE.md`
**Purpose:** Comprehensive documentation of .sg file format

**Contents:**
- Complete file structure
- All required fields and their meanings
- Curve format specification
- Realistic value ranges
- Common patterns
- Validation checklist
- Python generation templates
- Troubleshooting guide

---

## Key Learnings

### Critical Format Rule
**ALL curve `values_` arrays MUST use 2-element format: `[x, y]`**

❌ **WRONG (causes crash):**
```yaml
values_:
  - - -0.1
    - 0.0
    - 0.0
    - 0.15
```

✅ **CORRECT:**
```yaml
values_:
  - [0.0, 0.15]
```

### Realistic Sorghum Morphology

**Leaf Shape:**
- Width tapers to pointy tip (1.6% at tip)
- Bulges in middle (54.7% at 59% position)
- Base is moderate width (31.5%)

**Leaf Behavior:**
- Realistic droop (bending: 0.829 → 0.521)
- Natural roll variation (deviation: 12)
- Moderate waviness (max: 0.1, not 20!)

**Stem Characteristics:**
- Width varies along stem (1.0 → 0.688)
- Internode length: 0.035-0.084m depending on growth stage

---

## Validation Checklist

Before using .sg files in EvoEngine:

- [x] All `values_` arrays use 2-element `[x, y]` format
- [x] Leaf bending curve is realistic (not flat at 0.5)
- [x] Leaf waviness max_value ≤ 0.1 (not 20!)
- [x] width_along_leaf creates pointy tips (y ≈ 0.016 at x=1)
- [x] width_along_leaf has bulge in middle
- [x] width_along_stem is not all zeros
- [x] leaf_roll_angle deviation ≥ 12 for variation
- [x] YAML syntax is valid
- [x] File extension is `.sg`

---

## How to Use Going Forward

### For New .sg Files:
1. Use `generate_sg_file.py` to create new files
2. Or use `/sg-generator` slash command for guidance
3. Reference `SG_FILE_FORMAT_REFERENCE.md` for format details

### If You Have 4-Element Format Files:
1. Run `fix_sg_files.py` on the directory
2. Then run `apply_realistic_sorghum_shape.py` if appearance is unrealistic

### If Plants Look Unrealistic:
1. Run `apply_realistic_sorghum_shape.py` on the directory
2. Compare with `7leaf-target.sg` for reference
3. Check leaf_bending, width_along_leaf, and leaf_waviness parameters

---

## Backup Files

Both fix scripts create backup files:
- `.backup` - Original before format fix
- `.shape_backup` - Before shape parameter update

**Once you verify everything works, you can safely delete backup files:**
```bash
cd scenegraphcompare
rm *.backup *.shape_backup
```

---

## Reference Files

**Good Examples:**
- `7leaf-target.sg` - Realistic 7-leaf sorghum (gold standard)
- `single_leaf_growth_day4.sg` - Single leaf example

**Fixed Examples:**
- All files in `scenegraphcompare/` - Now have correct format and realistic appearance

---

## Tools Summary

| Tool | Purpose | Creates Backups |
|------|---------|-----------------|
| `fix_sg_files.py` | Fix 4-element → 2-element format | `.backup` |
| `apply_realistic_sorghum_shape.py` | Add realistic morphology | `.shape_backup` |
| `generate_sg_file.py` | Generate new files | N/A |
| `/sg-generator` | Expert guidance | N/A |

---

## Results

✅ **21 files fixed in `scenegraphcompare/`**
- No more crashes
- Realistic sorghum appearance
- Preserved original leaf lengths and stem dimensions
- Ready for PAR calculation and visualization

---

## Questions?

See:
- `SG_FILE_FORMAT_REFERENCE.md` - Complete format documentation
- `CLAUDE.md` - EvoEngine build and architecture
- `/sg-generator` - Interactive expert guidance
- `7leaf-target.sg` - Reference for realistic parameters
