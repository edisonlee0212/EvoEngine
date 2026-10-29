# Oak parameter reference

These values describe the supplied oak, not a calibrated species model. They are read directly by the basic descriptor implementations in this branch. Lengths and diameters use metres. Model time is in days; the command converts years to 365 days. Change copies of the files and pass their paths with `--shoot`, `--pruning`, and `--foliage`.

## Shoot file

| Field | Supplied value | Effect in this implementation |
| --- | ---: | --- |
| `base_internode_count` | 1 | Number of initial axis segments. |
| `straight_trunk` | 0.7 | Below this root-to-node distance, order-zero growth suppresses apical angular deviation and tropism. It does not prune low branches. |
| `growth_rate` | 0.2 | Nominal annual extension scale. Actual height also depends on light, growth allocation, angle and pruning. |
| `internode_length` | 0.03 | Base segment length; growth rate divided by this length sets the segment production rate. |
| `internode_length_thickness_factor` | 0.25 | Exponent scaling a segment's length as its diameter grows relative to the tip diameter. |
| `end_node_thickness` | 0.003 | Base/tip diameter, not radius. |
| `thickness_accumulation_factor` | 0.45 | Controls diameter accumulation toward the base. |
| `thickness_age_factor` | 0.00001 | Adds diameter according to segment age and growth rate. |
| `gravitropism` | 0.01 | Growth rotation toward the direction opposite gravity. |
| `phototropism` | 0.06 | Growth rotation toward the native light direction. |
| `horizontal_tropism` | 0 | Rotation toward the horizontal projection for non-trunk segments. |
| `gravity_bending_strength` | 2 | Strength of the native gravity sagging response. |
| `gravity_bending_thickness_factor` | 3 | Diameter dependence of sagging. |
| `gravity_bending_max` | 0.8 | Limits the sagging response. |
| `internode_shadow_factor` | 0.02 | Shadow contribution used by the native light calculation. |
| `lateral_bud_count` | 1 | Number of possible lateral buds per segment. |
| `max_order` | -1 | Limits creation of lateral buds by order; -1 leaves it unlimited. |
| `apical_bud_extinction_rate` | 0 | Reduces apical flushing; a short initial trunk region has a special always-flush rule. |
| `lateral_bud_flushing_rate` | 0.2 | Base lateral flushing rate; the implementation further suppresses it with an exponential function of the inhibitor sink. |
| `apical_control` | -0.9 | Sigmoid control of growth allocation. Negative values use the native level measure; nonnegative values use branch order. This is not simply a probability. |
| `root_distance_control` | 0 | Multiplies allocation by inverse root distance raised to this exponent; zero disables it. |
| `height_control` | 0 | Multiplies allocation by inverse height raised to this exponent; zero disables it. |
| `apical_dominance` | 0.5 | Sets the inhibitor produced in proportion to light intake. |
| `apical_dominance_loss` | 0.02 | Loss of that inhibitor during transport through the tree. |

`branching_angle_graph`, `roll_angle_graph`, and `apical_angle_graph` are complete procedural graphs, not single scalars. They are sampled with `(x, y, z, root_distance)` and their angular outputs are converted from degrees to radians. The branching graph contains a 65-degree constant together with noise and other operations; that constant alone is not the complete evaluated branch angle. Preserve the graph's node IDs and connections when editing it. Save a modified descriptor through the editor if unfamiliar with its graph serialization. The headless app can then use that file unchanged.

## Pruning file

| Field | Bundled value | Effect in this implementation |
| --- | ---: | --- |
| `low_branch_pruning` | 0 | Removes non-trunk branches below a fraction of the maximum root-to-tip path length, once that maximum exceeds 1 m. Applies at branching points. |
| `trunk_protection` | false | Exempts order-zero segments from the two pruning rules. |
| `max_flow_length` | 0 | If nonzero, prunes when the native chain index exceeds this value. This is a segment count threshold, not metres. |
| `light_pruning_factor` | 0.3 | Removes end segments whose light intake is below this threshold. |
| `branch_strength` | 10 | Multiplier for native resistance to breaking; zero disables that resistance calculation. |
| `branch_strength_thickness_factor` | 3 | Exponent of diameter divided by 0.002 m in the resistance calculation. |
| `branch_strength_lighting_threshold` | 0.4 | Descendant light threshold below which resistance is reduced. |
| `branch_strength_lighting_loss` | 0.9 | Fractional resistance loss under that threshold. |
| `branch_breaking_multiplier` | 1 | Both the multiplier and exponent in the current stress-based pruning expression. |
| `branch_breaking_factor` | 1 | Serialized in the file but not read by the current `PrepareController` calculation. Changing it has no effect there. |

The source implementation is the authority where comments and behaviour differ. These notes describe the checked-out version and should be reviewed when updating EvoEngine.

## Foliage and forest form

The bundled oak uses `leaf_size: [0.05, 0.1]`, `leaf_count: 2`, and `max_end_distance: 0.2`. Leaves are placed near tips subject to the descriptor's thickness and root-distance limits. The exporter uses the actual `GenerateFoliageMatrices` function and emits a simple plane for each placement. Leaf material references are not needed for geometry export.

For a taller clear trunk, start by studying `low_branch_pruning`, light-driven pruning and trunk persistence. For a narrower crown, study departure angles and allocation/directional controls at fixed seeds. Larger ages and growth rates change scale as well as architecture; they are not substitutes for competition. The reference forest crowns developed under crowding. A composed scene of independently grown trees has no such growth history. No fixed settings here are claimed to reproduce the eight named species in the reference figure.
