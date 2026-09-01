# Dynamic-strand bundle solver

`DsBundle` provides three persisted solver modes. Existing and missing scene data defaults to `Legacy`.

- `Legacy` preserves the original random bundle projection schedule.
- `CoupledXpbd` replaces cross-strand averaging with pairwise 6-DOF XPBD. Off-center midpoint anchors produce both
  translation and torque, while quaternion-log constraints correct relative bend and twist.
- `Hybrid` adds live cross-section rigid fitting and coarse longitudinal slice edges before the pairwise solve. This is
  the intended mode for solid, wood-like bundles and remains opt-in.

`pair_iterations`, `coarse_iterations`, compliance scales, and shape-matching strength are live controls. Slice spacing
and minimum slice membership change buffer topology and require dynamic-strand reinitialization. Compliance scales are
dimensionless multipliers over the existing material modulus graphs and segment geometry; they do not replace authored
wood properties.

Hybrid slices use the static branch/node and root-distance bin together with the latest connectivity `group_index`.
Cutting or structural failure therefore separates slice fits and coarse edges on the next `Physics()` call. Pair and
coarse cohesion continuously weakens with connection integrity, carbon/lignin health, rot density, and moisture. A
zero-integrity connection contributes no correction.

The board experiment is the controlled torque/rotation fixture. The large-log experiment covers scale, damage, and
performance. `Capture bundle diagnostics` records rigid-fit rotation, center/far-side motion, constraint RMS, momentum
residuals, dispatch count, owned strand/bundle memory, and the most recent 120 GPU profiler frames. Hybrid GPU time
includes topology rebuild, slice fit/apply, coarse-edge solve, and pair solve/gather.

Current implementation limits:

- Hybrid topology uses GPU bitonic sorts followed by serial GPU range/edge reduction. This avoids CPU readback but is a
  correctness-first path that may be slower than Legacy for large logs.
- Components below `minimum_slice_members` or with ill-conditioned endpoint covariance fall back to pairwise XPBD.
- `DsStiffRod` continues to own per-strand longitudinal mechanics; coarse edges couple adjacent bundle slices.
- Legacy remains the default until board/log Vulkan-validation captures and timestep-variation measurements pass on a
  supported runtime configuration.
