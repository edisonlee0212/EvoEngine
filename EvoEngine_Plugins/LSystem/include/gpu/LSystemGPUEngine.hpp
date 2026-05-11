#pragma once

// =============================================================================
//  LSystemGPUEngine — GPU-resident L-system pipeline orchestrator.
//
//  Phase 0 stub. Compiled into the plugin only when LSYSTEM_GPU_PIPELINE is
//  defined. Nothing here is wired into the runtime yet; this header locks the
//  CPU-side API that subsequent phases will fill in:
//
//      Phase 1 — implement EmitMeshlets() against existing CPU graph state.
//      Phase 2 — implement DispatchGrowAndPropagate() (compute shaders).
//      Phase 3 — implement DispatchDerive() (compute shaders).
//      Phase 4 — extend buffers with an instance dimension.
//      Phase 5 — wire IPlantBundle registry and replace tassel-specific
//                paths with bundle-driven dispatch.
//
//  See docs/gpu_pipeline.md for the full architecture and §10 phase gates.
// =============================================================================

#include "IPlantBundle.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#if defined(LSYSTEM_GPU_PIPELINE)

#include "TasselInstanceSoA.hpp"

#include <vulkan/vulkan.h>

namespace evo_engine {
class Buffer;
class DescriptorSet;
class DescriptorSetLayout;
}  // namespace evo_engine

namespace l_system_plugin::gpu {

// Forward declarations (full types live in their own headers; engine
// methods take them by reference, no need to include here).
struct TasselGrowthSoA;
struct CurveAtlas;

/// Per-instance dirty flags (mirrors EditChannel.dirty_bits in the GPU SSBO).
enum class DirtyBit : uint32_t {
  None     = 0,
  Topology = 1u << 0,  ///< rules or topology-affecting params changed → re-derive
  Curve    = 1u << 1,  ///< any Curve2D changed → re-bake curve atlas
  Scalar   = 1u << 2,  ///< cheap scalar param changed → next frame growth pass
  Tropism  = 1u << 3,  ///< tropism array reshaped → next frame growth pass
  All      = 0xFFFFFFFFu,
};

inline DirtyBit operator|(DirtyBit a, DirtyBit b) {
  return static_cast<DirtyBit>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline bool Any(DirtyBit a, DirtyBit b) {
  return (static_cast<uint32_t>(a) & static_cast<uint32_t>(b)) != 0u;
}

/// Hash-RNG seed components. Combined inside derive.comp / grow.comp via PCG.
/// See docs/gpu_pipeline.md §4.
struct RngKey {
  uint32_t seed = 0;            ///< user-controlled, per instance
  uint32_t instance_id = 0;     ///< stable per plant instance
  uint32_t derivation_step = 0; ///< incremented by derive.comp
};

/// Per-frame profile timestamps surfaced to the editor HUD.
/// All values in milliseconds; -1 if the corresponding pass did not run.
struct GpuFrameProfile {
  double upload_ms       = -1.0;
  double curve_bake_ms   = -1.0;
  double derive_ms       = -1.0;
  double compact_ms      = -1.0;
  double grow_ms         = -1.0;
  double propagate_ms    = -1.0;
  double mesh_submit_ms  = -1.0;
  uint32_t module_count  = 0;
  uint32_t fired_rules   = 0;
};

class LSystemGPUEngine {
 public:
  /// Process-wide singleton. Created on first access; reset only by tests.
  /// All call sites (LSystemLayer::OnCreate, MaizeTassel::RebuildGeometry,
  /// the equivalence harness) share the same engine so per-instance state
  /// (buffers, descriptor sets, shadows) is consistent across the frame.
  static LSystemGPUEngine& Get();

  LSystemGPUEngine();
  ~LSystemGPUEngine();

  LSystemGPUEngine(const LSystemGPUEngine&) = delete;
  LSystemGPUEngine& operator=(const LSystemGPUEngine&) = delete;

  // ---- Bundle registration (Phase 5) --------------------------------------

  void RegisterBundle(std::shared_ptr<IPlantBundle> bundle);
  std::shared_ptr<IPlantBundle> FindBundle(const std::string& name) const;

  // ---- Per-instance lifecycle (Phases 1–4) --------------------------------

  /// Returns a stable instance id used as a key for all subsequent calls.
  /// Phase 1 will call this when MaizeTassel::RebuildGeometry() is the
  /// orchestration point. Phase 4 will allow many instances per bundle.
  uint32_t CreateInstance(const std::string& bundle_name, uint32_t seed);
  void DestroyInstance(uint32_t instance_id);

  /// Mark a per-instance dirty flag. Cheap; coalesced inside the engine.
  void MarkDirty(uint32_t instance_id, DirtyBit bits);

  // ---- Phase 1a: tassel-specific SoA upload --------------------------------
  //
  // Entry point for the CPU packer (TasselInstancePacker.cpp). Copies the
  // packed SoA into a persistent-mapped SSBO associated with the instance.
  // Phase 1a implementation is a no-op that only records size + a local
  // copy for later inspection; Phase 1b replaces it with a VMA-backed
  // Vulkan buffer upload.
  //
  // Called once per ``RebuildGeometry()`` when ``LSYSTEM_GPU_PIPELINE`` is
  // on. Phase 2 will split this into a static-upload half (topology change)
  // and a dynamic-update half (growth fields only) to avoid copying the
  // color / rot channels on every frame.
  void UploadTasselInternodes(uint32_t instance_id,
                              const TasselInternodeInstance* data,
                              uint32_t count);

  /// Current internode instance count for ``instance_id``; 0 if the
  /// instance has no internodes uploaded yet.
  uint32_t GetTasselInternodeCount(uint32_t instance_id) const;

  // ---- Phase 1b: GPU resource wiring --------------------------------------
  //
  // Set the descriptor-set layout that the engine should bind ``set=1`` to
  // for the tassel_internode pipeline. Owned by LSystemLayer and forwarded
  // here from LSystemLayer::OnCreate so the engine can lazily create one
  // DescriptorSet per instance and update its (binding=0) slot to point
  // at the per-instance VMA SSBO.
  ///
  /// Idempotent: calling with the same layout pointer is a no-op.
  void SetTasselInternodeDescriptorLayout(
      const std::shared_ptr<evo_engine::DescriptorSetLayout>& layout);

  /// Returns the per-instance descriptor set bound to the latest VMA buffer
  /// upload, or VK_NULL_HANDLE if either (a) no descriptor layout has been
  /// installed via SetTasselInternodeDescriptorLayout, or (b) no upload
  /// has happened yet for this instance. Phase 1b-final draw recording
  /// will pass this as the ``set=1`` argument of BindDescriptorSet.
  VkDescriptorSet GetTasselInternodeDescriptorSet(uint32_t instance_id) const;

  /// Returns the underlying VMA buffer for diagnostics / debug capture.
  /// Lifetime tied to the engine; do not retain past DestroyInstance.
  std::shared_ptr<evo_engine::Buffer> GetTasselInternodeBuffer(uint32_t instance_id) const;

  // ---- Phase 2a: growth SoA + curve atlas (CPU-shadow only) --------------
  //
  // The CPU packer (TasselGrowthPacker.cpp) hands the engine a fully
  // populated TasselGrowthSoA. CurveBaker hands it a CurveAtlas. Both are
  // shadowed CPU-side in Phase 2a so the equivalence harness can validate
  // grow / propagate against the CPU baseline before any Vulkan dispatch
  // exists. Phase 2b replaces the shadows with persistent-mapped SSBOs.

  void UploadTasselGrowth(uint32_t instance_id, const TasselGrowthSoA& soa);
  void UploadCurveAtlas(uint32_t instance_id, const CurveAtlas& atlas);

  /// Run the CPU mirrors of grow.comp + propagate.comp against the
  /// shadowed SoA. Returns false if either upload is missing for the
  /// instance. Mutates the shadowed SoA's length_thickness, growth_state
  /// (when ``advance_age``), global_position, and global_rotation
  /// channels — exactly what the GLSL passes will mutate in Phase 2b.
  bool DispatchGrowAndPropagateCpuShadow(uint32_t instance_id,
                                         float gdd_step,
                                         float maturity_gdd,
                                         bool advance_age);

  // ---- Phase 2b.1: GPU grow + propagate (SSBO compute) -------------------
  //
  // Uploads the shadow SoA + curve atlas for `instance_id` into per-instance
  // VMA SSBOs, dispatches `tassel_grow.comp` once over the full node range,
  // then dispatches `tassel_propagate.comp` once per depth band (host loop
  // with EverythingBarrier between bands so each child reads its parent's
  // already-written global pose). On return the GPU output channels are
  // downloaded back into the shadow SoA so the rest of the engine can keep
  // reading from a single source of truth.
  //
  // Returns false if the shadow SoA / curve atlas have not been uploaded
  // for this instance, or if the compute pipelines failed to initialize.
  bool DispatchGrowAndPropagateGpu(uint32_t instance_id,
                                   float gdd_step,
                                   float maturity_gdd,
                                   bool advance_age);

  // ---- Phase 2b.1: parity self-test --------------------------------------
  //
  // Build a small synthetic SoA (linear chain + a couple of laterals) +
  // matching curve atlas, run the CPU shadow grow+propagate against a
  // copy, run the GPU compute path against another copy, and compare
  // length_thickness / global_position / global_rotation channels.
  //
  // Returns the maximum absolute difference observed across all output
  // channels and all nodes. A negative return value indicates the test
  // could not be run (shaders failed to load, no Vulkan device, etc.)
  // and `failure_reason` (if non-null) carries a short diagnostic string.
  //
  // Intended to be invoked once at plugin OnCreate so divergences caught
  // at startup never reach a real tassel scene. ~1 ms per call.
  float RunGrowPropagateSelfTest(std::string* failure_reason = nullptr);

  // ---- Phase 2b.2: GPU pack_internodes -----------------------------------
  //
  // Sparse 1:1 pack: one TasselInternodeInstance written per growth-SoA
  // node, with length=0 sentinel for skipped nodes (non-internodes,
  // zero-length, non-finite). The mesh task shader culls sentinels at
  // dispatch time.
  //
  // Preconditions:
  //   * `UploadTasselGrowth` has been called (growth shadow exists)
  //   * The grow+propagate compute pipelines have been initialized
  //     (typically via a prior `DispatchGrowAndPropagateGpu` call, since
  //     the pack pipeline shares the same set=0 layout)
  //   * `SetTasselInternodeDescriptorLayout` has been called by
  //     `LSystemLayer::OnCreate`
  //
  // Side effects:
  //   * Allocates / resizes the per-instance TasselInternodeInstance SSBO
  //     to hold `node_count` entries (power-of-two capacity).
  //   * Updates `GetTasselInternodeCount` to return `node_count` so the
  //     existing draw callback dispatches the right work-group count.
  //   * The shadow vector returned by `GetTasselInternodeCount` source is
  //     resized but NOT populated (GPU is the source of truth for the
  //     packed instance bytes; CPU shadow stays stale until next CPU
  //     `UploadTasselInternodes` call).
  //
  // Returns false if any precondition fails or the pipeline could not be
  // built. Caller falls back to the CPU pack path in that case.
  bool DispatchPackInternodesGpu(uint32_t instance_id,
                                 uint32_t color_mode,
                                 const float instance_color[4]);

  /// Read-only access to the shadowed growth SoA for diagnostics + the
  /// equivalence harness. Lifetime tied to the engine instance; returns
  /// nullptr if no upload has happened yet.
  const TasselGrowthSoA* PeekTasselGrowthShadow(uint32_t instance_id) const;

  // ---- Phase 3a: derive interface stub ------------------------------------
  //
  // Phase 3a does NOT run a GPU derive — derive.comp's per-rule kernels
  // are stubbed. This entry takes a host-supplied callable that runs ONE
  // CPU derivation step (typically wrapping engine_.ApplyTopologyRules),
  // then re-packs the resulting graph into the shadowed growth SoA. It
  // exists so the per-frame round-trip "derive → re-pack → grow →
  // propagate → mesh" can be exercised end-to-end on the CPU shadow,
  // unblocking the equivalence harness for derive output before the
  // GLSL kernels exist.
  //
  // Phase 3b will replace this with VkCmdDispatch(derive.comp) +
  // VkCmdDispatch(compact.comp) and a GPU-side re-pack.
  using DeriveAndRepackFn = std::function<bool(uint32_t instance_id,
                                               TasselGrowthSoA& shadow)>;
  bool DispatchDeriveCpuShadow(uint32_t instance_id,
                               const DeriveAndRepackFn& derive_and_repack);

  // ---- Per-frame entry point ----------------------------------------------

  /// Called once per frame from LSystemLayer::Update(). Internally:
  ///   1. uploads any pending EditChannel writes,
  ///   2. (re-)bakes curves if Curve dirty,
  ///   3. dispatches derive/compact if Topology dirty,
  ///   4. always dispatches grow + propagate,
  ///   5. records indirect mesh-shader draws into the active command buffer.
  /// Phase 0: no-op.
  void TickFrame(double delta_time_seconds);

  // ---- Diagnostics --------------------------------------------------------

  GpuFrameProfile LastFrameProfile(uint32_t instance_id) const;

  // Implementation type — public name only so TU-local helpers in
  // LSystemGPUEngine.cpp can take `Impl&`. The body is defined inside
  // the .cpp so external code still cannot touch its members.
  struct Impl;

 private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
