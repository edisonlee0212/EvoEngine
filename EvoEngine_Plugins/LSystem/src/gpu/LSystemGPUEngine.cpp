// =============================================================================
//  LSystemGPUEngine — Phase 0 stub implementation.
//
//  Compiled into the plugin only when LSYSTEM_GPU_PIPELINE is defined. All
//  methods are no-ops at this stage. Subsequent phases fill in:
//    Phase 1 — mesh-shader emission paths,
//    Phase 2 — grow + propagate compute dispatches,
//    Phase 3 — derive + compact compute dispatches,
//    Phase 4 — multi-instance buffer growth,
//    Phase 5 — IPlantBundle registry resolution.
//
//  See docs/gpu_pipeline.md.
// =============================================================================

#include "gpu/LSystemGPUEngine.hpp"

#if defined(LSYSTEM_GPU_PIPELINE)

#include "gpu/CurveBaker.hpp"
#include "gpu/TasselGrowthSoA.hpp"

#include "Platform/Platform.hpp"
#include "Rendering/Platform/ComputePipeline.hpp"
#include "Rendering/Platform/GraphicsResources.hpp"
#include "Rendering/Platform/Shader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <random>
#include <string>

namespace l_system_plugin::gpu {

namespace {

// Round capacity up to the next power of two so we don't reallocate the VMA
// buffer on every small growth step. Minimum 64 instances to keep the
// initial allocation aligned to a useful power of two for typical small
// tassels.
uint32_t NextPow2Capacity(uint32_t requested) {
  uint32_t cap = 64u;
  while (cap < requested) cap <<= 1u;
  return cap;
}

// CPU-side mirror of grow.comp. Mutates ``soa.length_thickness`` in place
// and optionally advances ``soa.growth_state[*].x`` (age_gdd) by gdd_step.
// Used by the equivalence harness to validate the GLSL implementation
// without a live Vulkan device.
void RunGrowCpu(TasselGrowthSoA& soa,
                const CurveAtlas& atlas,
                float gdd_step,
                float maturity_gdd,
                bool advance_age) {
  const uint32_t n = soa.header.node_count;
  for (uint32_t i = 0; i < n; ++i) {
    glm::vec4& gs = soa.growth_state[i];
    if (advance_age) gs.x += gdd_step;
    const uint32_t tag = soa.type_tag[i];
    const float age = gs.x;
    const float t = (maturity_gdd > 0.0f)
                  ? std::min(std::max(age / maturity_gdd, 0.0f), 1.0f)
                  : 0.0f;
    glm::vec2 out(0.0f);
    if (tag == static_cast<uint32_t>(TasselTypeTag::Internode) ||
        tag == static_cast<uint32_t>(TasselTypeTag::SpikeletPair)) {
      const float lf = atlas.SampleLinear(0u, t);  // CURVE_INTERNODE_LENGTH
      const float tf = atlas.SampleLinear(1u, t);  // CURVE_THICKNESS_GROWTH
      out = glm::vec2(gs.y * lf, gs.z * tf * 0.5f);
    }
    soa.length_thickness[i] = out;
  }
}

// CPU-side mirror of propagate.comp. Walks depth bands in order so each
// child reads its parent's just-written global pose (matches the host
// barrier discipline assumed by the GLSL pass).
void RunPropagateCpu(TasselGrowthSoA& soa) {
  const uint32_t n = soa.header.node_count;
  if (n == 0) return;
  const uint32_t band_count = soa.header.depth_band_count;

  // Band 0: keep host-seeded pose, just refresh w = length.
  {
    const uint32_t end = soa.header.depth_band_offsets[1];
    for (uint32_t s = 0; s < end; ++s) {
      const uint32_t i = soa.depth_sorted_index[s];
      soa.global_position[i].w = soa.length_thickness[i].x;
    }
  }

  for (uint32_t b = 1; b < band_count; ++b) {
    const uint32_t s_begin = soa.header.depth_band_offsets[b];
    const uint32_t s_end   = soa.header.depth_band_offsets[b + 1];
    for (uint32_t s = s_begin; s < s_end; ++s) {
      const uint32_t i = soa.depth_sorted_index[s];
      const int32_t parent = soa.parent_index[i];
      const float length = soa.length_thickness[i].x;
      if (parent < 0) {
        soa.global_position[i].w = length;
        continue;
      }
      const glm::vec4 parent_pos = soa.global_position[parent];
      const glm::vec4 parent_rot = soa.global_rotation[parent];
      const glm::vec4 local_q    = soa.local_rotation[i];

      // Quaternion rotate (0,0,-1) by parent_rot.
      const glm::quat pr(parent_rot.w, parent_rot.x, parent_rot.y, parent_rot.z);
      glm::vec3 dir = pr * glm::vec3(0.0f, 0.0f, -1.0f);
      const float dn = glm::length(dir);
      if (dn > 0.0f) dir /= dn;

      const glm::vec3 child_pos = glm::vec3(parent_pos) + dir * parent_pos.w;

      // Hamilton product parent_rot * local_q.
      const glm::quat lq(local_q.w, local_q.x, local_q.y, local_q.z);
      glm::quat cq = pr * lq;
      const float cn = std::sqrt(cq.x*cq.x + cq.y*cq.y + cq.z*cq.z + cq.w*cq.w);
      if (cn > 0.0f) cq = glm::quat(cq.w / cn, cq.x / cn, cq.y / cn, cq.z / cn);
      else           cq = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

      soa.global_position[i] = glm::vec4(child_pos, length);
      soa.global_rotation[i] = glm::vec4(cq.x, cq.y, cq.z, cq.w);
    }
  }
}

}  // namespace

struct LSystemGPUEngine::Impl {
  // Phase 1+ will hold Vulkan handles, SSBOs, descriptor sets, the bundle
  // registry, per-instance state, and the GPU profile ring buffer.
  std::unordered_map<std::string, std::shared_ptr<IPlantBundle>> bundles;
  uint32_t next_instance_id = 1u;

  // Phase 1a: per-instance shadow copy of the packed tassel internode SoA.
  // Replaced in Phase 1b by a VMA-backed device-local VkBuffer with a
  // staging upload path; the CPU shadow stays as a debug/validation aid.
  std::unordered_map<uint32_t, std::vector<TasselInternodeInstance>>
      tassel_internode_shadows;

  // Phase 1b: GPU resources for the tassel_internode draw path.
  // Layout is owned by LSystemLayer and supplied via
  // SetTasselInternodeDescriptorLayout. Per-instance buffers are grown
  // power-of-two so transient growth/regrowth doesn't thrash the
  // allocator. Capacity is in *instances* (TasselInternodeInstance), not
  // bytes.
  std::shared_ptr<evo_engine::DescriptorSetLayout> tassel_internode_layout;
  std::unordered_map<uint32_t, std::shared_ptr<evo_engine::Buffer>>
      tassel_internode_buffers;
  std::unordered_map<uint32_t, std::shared_ptr<evo_engine::DescriptorSet>>
      tassel_internode_descriptor_sets;
  std::unordered_map<uint32_t, uint32_t> tassel_internode_buffer_capacities;

  // Phase 2a: per-instance shadow copy of the growth SoA + most recent
  // curve atlas. The CPU-shadow grow/propagate runs against these so the
  // equivalence harness can validate against the CPU baseline before any
  // Vulkan dispatch lands. Replaced in Phase 2b by device-local SSBOs.
  std::unordered_map<uint32_t, TasselGrowthSoA> growth_shadows;
  std::unordered_map<uint32_t, CurveAtlas>       curve_shadows;
  std::unordered_map<uint32_t, GpuFrameProfile>  last_profiles;

  // Phase 2b.1: GPU-resident growth resources.
  //
  // One descriptor set + 12 SSBOs per instance: 11 SoA channels + atlas.
  // Layout binding indices match tassel_grow.comp / tassel_propagate.comp:
  //   0 header, 1 parent_index, 2 depth, 3 type_tag, 4 local_rotation,
  //   5 growth_state, 6 node_random_misc, 7 length_thickness,
  //   8 global_position, 9 global_rotation, 10 depth_sorted_index,
  //   11 curve atlas.
  // Buffers are sized for the current node_count exactly; reallocation is
  // cheap because Phase 2b.1 is only exercised by the self-test (one-shot).
  // Phase 2b.2 will switch to power-of-two capacity to handle live growth.
  struct GrowthGpuResources {
    std::shared_ptr<evo_engine::Buffer> header;
    std::shared_ptr<evo_engine::Buffer> parent_index;
    std::shared_ptr<evo_engine::Buffer> depth;
    std::shared_ptr<evo_engine::Buffer> type_tag;
    std::shared_ptr<evo_engine::Buffer> local_rotation;
    std::shared_ptr<evo_engine::Buffer> growth_state;
    std::shared_ptr<evo_engine::Buffer> node_random_misc;
    std::shared_ptr<evo_engine::Buffer> length_thickness;
    std::shared_ptr<evo_engine::Buffer> global_position;
    std::shared_ptr<evo_engine::Buffer> global_rotation;
    std::shared_ptr<evo_engine::Buffer> depth_sorted_index;
    std::shared_ptr<evo_engine::Buffer> atlas;
    std::shared_ptr<evo_engine::DescriptorSet> set;
    uint32_t node_count = 0;        ///< per-node buffer length
    uint32_t atlas_taps_count = 0;  ///< curve atlas tap length
  };

  std::shared_ptr<evo_engine::DescriptorSetLayout> growth_layout;
  std::shared_ptr<evo_engine::ComputePipeline> grow_pipeline;
  std::shared_ptr<evo_engine::ComputePipeline> propagate_pipeline;
  std::unordered_map<uint32_t, GrowthGpuResources> growth_gpu_resources;
  bool growth_pipelines_init_attempted = false;

  // Phase 2b.2: GPU pack_internodes pipeline. Shares set=0 with grow /
  // propagate; binds the existing tassel_internode_layout at set=1.
  std::shared_ptr<evo_engine::ComputePipeline> pack_internodes_pipeline;
  bool pack_pipeline_init_attempted = false;
};

namespace {

// Push constants must match the corresponding GLSL `layout(push_constant)`
// blocks in tassel_grow.comp and tassel_propagate.comp.
struct GrowPushConstant {
  uint32_t node_count;
  float    gdd_step;
  float    maturity_gdd;
  uint32_t advance_age;
};
static_assert(sizeof(GrowPushConstant) == 16, "GrowPushConstant must match GrowPC in tassel_grow.comp");

struct PropagatePushConstant {
  uint32_t band_begin;
  uint32_t band_end;
  uint32_t _pad0 = 0;
  uint32_t _pad1 = 0;
};
static_assert(sizeof(PropagatePushConstant) == 16, "PropagatePushConstant must match PropagatePC in tassel_propagate.comp");

// Mirrors `layout(push_constant) uniform PackPC` in tassel_pack_internodes.comp.
// std430 lays out vec4 on a 16-byte boundary, which combined with two leading
// uints requires an 8-byte pad to push the vec4 to offset 16. Total size 32 B.
struct PackPushConstant {
  uint32_t node_count;
  uint32_t color_mode;
  uint32_t _pad0 = 0;
  uint32_t _pad1 = 0;
  float    instance_color[4]{0.0f, 0.0f, 0.0f, 1.0f};
};
static_assert(sizeof(PackPushConstant) == 32, "PackPushConstant must match PackPC in tassel_pack_internodes.comp");

// Build a Buffer with given byte size, usage, and host access mode. Caller
// owns the lifetime via shared_ptr; the engine retains the only strong ref.
std::shared_ptr<evo_engine::Buffer> MakeStorageBuffer(VkDeviceSize size_bytes,
                                                     VkBufferUsageFlags extra_usage,
                                                     bool host_random_access) {
  if (size_bytes == 0) size_bytes = 16;  // VMA disallows zero-size; guard.
  VkBufferCreateInfo bci{};
  bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  bci.size  = size_bytes;
  bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
            | VK_BUFFER_USAGE_TRANSFER_DST_BIT
            | VK_BUFFER_USAGE_TRANSFER_SRC_BIT
            | extra_usage;
  bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo aci{};
  aci.usage = VMA_MEMORY_USAGE_AUTO;
  // RANDOM lets DownloadData read back via a direct map; SEQUENTIAL is
  // strictly upload-only. Output channels need readback for the parity
  // harness, so pay the cost across the board for Phase 2b.1.
  aci.flags = host_random_access
                  ? VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT
                  : VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
  return std::make_shared<evo_engine::Buffer>(bci, aci);
}

// Compose absolute path under the LSystem plugin Internals shader root.
std::filesystem::path TasselComputeShader(const char* leaf) {
  return std::filesystem::path("./LSystemResources") / "Shaders/Compute/Tassel" / leaf;
}

}  // namespace

LSystemGPUEngine& LSystemGPUEngine::Get() {
  // Function-local static avoids static-init-order issues with the engine
  // SDK. Lifetime ends at process shutdown; per-instance Vulkan resources
  // must be released via DestroyInstance before Platform shuts down.
  static LSystemGPUEngine instance;
  return instance;
}

LSystemGPUEngine::LSystemGPUEngine() : impl_(std::make_unique<Impl>()) {}
LSystemGPUEngine::~LSystemGPUEngine() = default;

void LSystemGPUEngine::RegisterBundle(std::shared_ptr<IPlantBundle> bundle) {
  if (!bundle) return;
  impl_->bundles[bundle->Name()] = std::move(bundle);
}

std::shared_ptr<IPlantBundle> LSystemGPUEngine::FindBundle(const std::string& name) const {
  const auto it = impl_->bundles.find(name);
  return it == impl_->bundles.end() ? nullptr : it->second;
}

uint32_t LSystemGPUEngine::CreateInstance(const std::string& /*bundle_name*/, uint32_t /*seed*/) {
  // Phase 4 will allocate per-instance slots in the SoA buffers.
  return impl_->next_instance_id++;
}

void LSystemGPUEngine::DestroyInstance(uint32_t instance_id) {
  // Phase 1b: release per-instance Vulkan resources. The DescriptorSet /
  // Buffer destructors free their underlying handles via VMA / the engine
  // descriptor pool. Order matters: drop the descriptor set first so its
  // last write to the (now-doomed) buffer is sequenced before the buffer
  // is destroyed.
  impl_->tassel_internode_descriptor_sets.erase(instance_id);
  impl_->tassel_internode_buffers.erase(instance_id);
  impl_->tassel_internode_buffer_capacities.erase(instance_id);
  impl_->tassel_internode_shadows.erase(instance_id);
  impl_->growth_shadows.erase(instance_id);
  impl_->curve_shadows.erase(instance_id);
  impl_->growth_gpu_resources.erase(instance_id);
  impl_->last_profiles.erase(instance_id);
}

void LSystemGPUEngine::MarkDirty(uint32_t /*instance_id*/, DirtyBit /*bits*/) {
  // Phase 7: route into the EditChannel ring buffer.
}

void LSystemGPUEngine::SetTasselInternodeDescriptorLayout(
    const std::shared_ptr<evo_engine::DescriptorSetLayout>& layout) {
  // Idempotent: only act if the layout actually changed. Re-binding the
  // same layout would require recreating every per-instance descriptor
  // set; that's only valid if the underlying Vulkan layout differs.
  if (impl_->tassel_internode_layout == layout) return;
  impl_->tassel_internode_layout = layout;
  // Existing per-instance descriptor sets were built against the old
  // layout; drop them so the next upload rebuilds against the new one.
  impl_->tassel_internode_descriptor_sets.clear();
}

VkDescriptorSet LSystemGPUEngine::GetTasselInternodeDescriptorSet(uint32_t instance_id) const {
  const auto it = impl_->tassel_internode_descriptor_sets.find(instance_id);
  if (it == impl_->tassel_internode_descriptor_sets.end()) return VK_NULL_HANDLE;
  return it->second->GetVkDescriptorSet();
}

std::shared_ptr<evo_engine::Buffer> LSystemGPUEngine::GetTasselInternodeBuffer(uint32_t instance_id) const {
  const auto it = impl_->tassel_internode_buffers.find(instance_id);
  return it == impl_->tassel_internode_buffers.end() ? nullptr : it->second;
}

void LSystemGPUEngine::UploadTasselInternodes(uint32_t instance_id,
                                              const TasselInternodeInstance* data,
                                              uint32_t count) {
  // Always refresh the CPU shadow — the equivalence harness and any
  // diagnostic peek path read from this map.
  auto& shadow = impl_->tassel_internode_shadows[instance_id];
  shadow.assign(data, data + count);

  // If the layout hasn't been installed yet (e.g. running headless tests
  // before LSystemLayer::OnCreate), keep the shadow-only path. The CPU
  // peek API still works; the GPU draw side simply has no buffer to
  // bind. This matches Phase 1a behavior exactly.
  if (!impl_->tassel_internode_layout) return;

  // (Re)allocate the device-local SSBO if capacity is insufficient. A
  // pure capacity check (not a tight equality) lets us skip realloc on
  // every minor growth tick. Empty uploads still drop us through so the
  // descriptor set is bound to *some* buffer.
  const uint32_t needed_capacity = std::max<uint32_t>(count, 1u);
  uint32_t& current_capacity = impl_->tassel_internode_buffer_capacities[instance_id];
  std::shared_ptr<evo_engine::Buffer>& buffer = impl_->tassel_internode_buffers[instance_id];
  bool buffer_rebuilt = false;
  if (!buffer || current_capacity < needed_capacity) {
    current_capacity = NextPow2Capacity(needed_capacity);
    VkBufferCreateInfo buffer_create_info{};
    buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_create_info.size = static_cast<VkDeviceSize>(current_capacity) * sizeof(TasselInternodeInstance);
    buffer_create_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
                             | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo alloc_create_info{};
    // VMA_MEMORY_USAGE_AUTO + HOST_ACCESS_SEQUENTIAL_WRITE_BIT lets VMA
    // pick the best memory type (BAR / system / device-local) for a
    // mostly-write workload. UploadVector below handles the host->device
    // path (either direct map or staging copy) transparently.
    alloc_create_info.usage = VMA_MEMORY_USAGE_AUTO;
    alloc_create_info.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;

    buffer = std::make_shared<evo_engine::Buffer>(buffer_create_info, alloc_create_info);
    buffer_rebuilt = true;
  }

  // Push the actual instance bytes. UploadVector handles persistent-map
  // vs staging internally based on the VMA allocation. For an empty
  // upload we still want to keep the buffer allocation around so the
  // descriptor binding remains valid.
  if (!shadow.empty()) {
    buffer->UploadVector(shadow);
  }

  // Lazily allocate the descriptor set the first time a buffer exists
  // for this instance. After a buffer rebuild we must re-write the
  // (binding=0) slot since its underlying VkBuffer handle changed.
  std::shared_ptr<evo_engine::DescriptorSet>& descriptor_set =
      impl_->tassel_internode_descriptor_sets[instance_id];
  const bool descriptor_set_fresh = !descriptor_set;
  if (descriptor_set_fresh) {
    descriptor_set = std::make_shared<evo_engine::DescriptorSet>(impl_->tassel_internode_layout);
  }
  if (descriptor_set_fresh || buffer_rebuilt) {
    descriptor_set->UpdateBufferDescriptorBinding(0, buffer);
  }
}

uint32_t LSystemGPUEngine::GetTasselInternodeCount(uint32_t instance_id) const {
  const auto it = impl_->tassel_internode_shadows.find(instance_id);
  return it == impl_->tassel_internode_shadows.end()
             ? 0u
             : static_cast<uint32_t>(it->second.size());
}

namespace {
// Forward declarations: the full bodies live in the Phase 2b.1 anonymous
// namespace block further down (alongside DispatchGrowAndPropagateGpu).
// Phase 2b.3 needs them callable from UploadTasselGrowth /
// UploadCurveAtlas which sit above that block — moving the full bodies
// up would entangle the file's chronological phase ordering, so a
// forward declaration is preferred over a reorder.
bool EnsureGrowthPipelines(LSystemGPUEngine::Impl& impl);
void EnsureGrowthBuffersAndUploadSoA(LSystemGPUEngine::Impl& impl,
                                     uint32_t instance_id,
                                     const TasselGrowthSoA& soa);
void UploadCurveAtlasBytes(LSystemGPUEngine::Impl& impl,
                           uint32_t instance_id,
                           const CurveAtlas& atlas);
}  // namespace

void LSystemGPUEngine::UploadTasselGrowth(uint32_t instance_id,
                                          const TasselGrowthSoA& soa) {
  // CPU shadow stays the source of truth for diagnostics + the CPU-shadow
  // dispatch path.
  impl_->growth_shadows[instance_id] = soa;
  // Push the SoA into per-instance SSBOs whenever the compute pipelines
  // are available. The pack-only path needs this even when no atlas has
  // been uploaded yet — the atlas slot is bound to a 16-byte stub.
  if (EnsureGrowthPipelines(*impl_)) {
    EnsureGrowthBuffersAndUploadSoA(*impl_, instance_id, soa);
  }
}

void LSystemGPUEngine::UploadCurveAtlas(uint32_t instance_id,
                                        const CurveAtlas& atlas) {
  impl_->curve_shadows[instance_id] = atlas;
  // Mirror the SoA upload behavior: push to GPU whenever the buffers
  // exist. If UploadTasselGrowth hasn't been called yet, the GPU upload
  // is deferred until then — the atlas blob will land on the next
  // EnsureGrowthBuffersAndUploadSoA via the dispatch path's first call.
  if (impl_->growth_gpu_resources.count(instance_id) != 0u) {
    UploadCurveAtlasBytes(*impl_, instance_id, atlas);
  }
}

bool LSystemGPUEngine::DispatchGrowAndPropagateCpuShadow(
    uint32_t instance_id,
    float gdd_step,
    float maturity_gdd,
    bool advance_age) {
  auto soa_it = impl_->growth_shadows.find(instance_id);
  if (soa_it == impl_->growth_shadows.end()) return false;
  auto atlas_it = impl_->curve_shadows.find(instance_id);
  if (atlas_it == impl_->curve_shadows.end()) return false;

  RunGrowCpu(soa_it->second, atlas_it->second, gdd_step, maturity_gdd, advance_age);
  RunPropagateCpu(soa_it->second);

  GpuFrameProfile& prof = impl_->last_profiles[instance_id];
  prof.module_count = soa_it->second.header.node_count;
  prof.grow_ms      = 0.0;  // CPU shadow doesn't time the work.
  prof.propagate_ms = 0.0;
  return true;
}

const TasselGrowthSoA* LSystemGPUEngine::PeekTasselGrowthShadow(uint32_t instance_id) const {
  const auto it = impl_->growth_shadows.find(instance_id);
  return it == impl_->growth_shadows.end() ? nullptr : &it->second;
}

bool LSystemGPUEngine::DispatchDeriveCpuShadow(
    uint32_t instance_id,
    const DeriveAndRepackFn& derive_and_repack) {
  if (!derive_and_repack) return false;
  // Lazily allocate a shadow if none exists yet (first derive of a fresh
  // instance happens before any UploadTasselGrowth).
  auto& shadow = impl_->growth_shadows[instance_id];
  const bool ok = derive_and_repack(instance_id, shadow);
  if (ok) {
    GpuFrameProfile& prof = impl_->last_profiles[instance_id];
    prof.module_count = shadow.header.node_count;
  }
  return ok;
}

void LSystemGPUEngine::TickFrame(double /*delta_time_seconds*/) {
  // Phases 1–3 fill this in. Order: upload → curve bake → derive → compact →
  // grow → propagate → record indirect mesh-shader draws.
}

GpuFrameProfile LSystemGPUEngine::LastFrameProfile(uint32_t instance_id) const {
  const auto it = impl_->last_profiles.find(instance_id);
  return it == impl_->last_profiles.end() ? GpuFrameProfile{} : it->second;
}

// ---------------------------------------------------------------------------
//  Phase 2b.1: GPU grow + propagate.
// ---------------------------------------------------------------------------

namespace {

// Lazy compile + link the two compute pipelines + shared descriptor layout.
// Returns false if anything fails — caller is expected to fall back to the
// CPU shadow path. Idempotent; first failure latches via init_attempted.
bool EnsureGrowthPipelines(LSystemGPUEngine::Impl& impl) {
  if (impl.grow_pipeline && impl.propagate_pipeline && impl.growth_layout) return true;
  if (impl.growth_pipelines_init_attempted) {
    return impl.grow_pipeline && impl.propagate_pipeline && impl.growth_layout;
  }
  impl.growth_pipelines_init_attempted = true;

  if (!evo_engine::Platform::Constants::support_mesh_shader) {
    // No mesh shader == almost certainly no compute either on this device
    // tier. Skip safely; the CPU path remains correct.
    return false;
  }

  auto layout = std::make_shared<evo_engine::DescriptorSetLayout>();
  // 12 SSBO bindings, all visible to the compute stage. Indices match the
  // GLSL declarations in tassel_grow.comp / tassel_propagate.comp exactly.
  for (uint32_t binding = 0; binding < 12; ++binding) {
    layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                  VK_SHADER_STAGE_COMPUTE_BIT, 0);
  }
  layout->Initialize();
  impl.growth_layout = layout;

  auto build_pipeline = [&](const char* leaf, size_t pc_size)
      -> std::shared_ptr<evo_engine::ComputePipeline> {
    auto shader = std::make_shared<evo_engine::Shader>();
    shader->TryCompile(evo_engine::ShaderType::Compute,
                       evo_engine::Platform::GetShaderGlobalDefines(),
                       TasselComputeShader(leaf));
    auto pipeline = std::make_shared<evo_engine::ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts.emplace_back(impl.growth_layout);
    auto& pcr = pipeline->push_constant_ranges.emplace_back();
    pcr.size = static_cast<uint32_t>(pc_size);
    pcr.offset = 0;
    pcr.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pipeline->Initialize();
    return pipeline;
  };

  impl.grow_pipeline      = build_pipeline("tassel_grow.comp",      sizeof(GrowPushConstant));
  impl.propagate_pipeline = build_pipeline("tassel_propagate.comp", sizeof(PropagatePushConstant));
  return impl.grow_pipeline && impl.grow_pipeline->Initialized()
      && impl.propagate_pipeline && impl.propagate_pipeline->Initialized();
}

// Lazy-build the pack pipeline. Requires both the growth descriptor layout
// (set=0) and the tassel_internode_layout (set=1) to be installed first.
bool EnsurePackPipeline(LSystemGPUEngine::Impl& impl) {
  if (impl.pack_internodes_pipeline) return true;
  if (impl.pack_pipeline_init_attempted) return false;
  impl.pack_pipeline_init_attempted = true;

  if (!impl.growth_layout || !impl.tassel_internode_layout) return false;
  if (!evo_engine::Platform::Constants::support_mesh_shader) return false;

  auto shader = std::make_shared<evo_engine::Shader>();
  shader->TryCompile(evo_engine::ShaderType::Compute,
                     evo_engine::Platform::GetShaderGlobalDefines(),
                     TasselComputeShader("tassel_pack_internodes.comp"));
  auto pipeline = std::make_shared<evo_engine::ComputePipeline>();
  pipeline->compute_shader = shader;
  pipeline->descriptor_set_layouts.emplace_back(impl.growth_layout);            // set=0
  pipeline->descriptor_set_layouts.emplace_back(impl.tassel_internode_layout);  // set=1
  auto& pcr = pipeline->push_constant_ranges.emplace_back();
  pcr.size = static_cast<uint32_t>(sizeof(PackPushConstant));
  pcr.offset = 0;
  pcr.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  pipeline->Initialize();
  if (!pipeline->Initialized()) return false;
  impl.pack_internodes_pipeline = pipeline;
  return true;
}

// Allocate / reallocate per-instance SSBOs for the SoA channels (no atlas).
// Uploads every per-node channel from the supplied SoA. The atlas slot is
// (re)bound to a 16-byte stub buffer when no atlas has been pushed yet so
// the descriptor set remains valid even before any UploadCurveAtlasToGpu
// call. Phase 2b.3 contract: callers may upload the SoA before they have a
// curve atlas (the pack-only path skips grow/propagate entirely).
void EnsureGrowthBuffersAndUploadSoA(LSystemGPUEngine::Impl& impl,
                                     uint32_t instance_id,
                                     const TasselGrowthSoA& soa) {
  auto& res = impl.growth_gpu_resources[instance_id];
  const uint32_t n = soa.header.node_count;
  const bool needs_realloc = !res.set || res.node_count != n;

  if (needs_realloc) {
    res.node_count = n;
    res.header             = MakeStorageBuffer(sizeof(TasselGrowthHeader),                          0, false);
    res.parent_index       = MakeStorageBuffer(static_cast<VkDeviceSize>(n) * sizeof(int32_t),      0, false);
    res.depth              = MakeStorageBuffer(static_cast<VkDeviceSize>(n) * sizeof(uint32_t),     0, false);
    res.type_tag           = MakeStorageBuffer(static_cast<VkDeviceSize>(n) * sizeof(uint32_t),     0, false);
    res.local_rotation     = MakeStorageBuffer(static_cast<VkDeviceSize>(n) * sizeof(glm::vec4),    0, false);
    res.growth_state       = MakeStorageBuffer(static_cast<VkDeviceSize>(n) * sizeof(glm::vec4),    0, true);
    res.node_random_misc   = MakeStorageBuffer(static_cast<VkDeviceSize>(n) * sizeof(glm::vec4),    0, false);
    res.length_thickness   = MakeStorageBuffer(static_cast<VkDeviceSize>(n) * sizeof(glm::vec2),    0, true);
    res.global_position    = MakeStorageBuffer(static_cast<VkDeviceSize>(n) * sizeof(glm::vec4),    0, true);
    res.global_rotation    = MakeStorageBuffer(static_cast<VkDeviceSize>(n) * sizeof(glm::vec4),    0, true);
    res.depth_sorted_index = MakeStorageBuffer(static_cast<VkDeviceSize>(n) * sizeof(uint32_t),     0, false);
    // Stub atlas: 16-byte header so the descriptor binding is valid even
    // when no curve atlas has been uploaded for this instance yet. Real
    // atlas allocations replace this in UploadCurveAtlasBytes().
    if (!res.atlas) {
      res.atlas = MakeStorageBuffer(16, 0, false);
      res.atlas_taps_count = 0;
    }
    res.set = std::make_shared<evo_engine::DescriptorSet>(impl.growth_layout);
    res.set->UpdateBufferDescriptorBinding(0,  res.header);
    res.set->UpdateBufferDescriptorBinding(1,  res.parent_index);
    res.set->UpdateBufferDescriptorBinding(2,  res.depth);
    res.set->UpdateBufferDescriptorBinding(3,  res.type_tag);
    res.set->UpdateBufferDescriptorBinding(4,  res.local_rotation);
    res.set->UpdateBufferDescriptorBinding(5,  res.growth_state);
    res.set->UpdateBufferDescriptorBinding(6,  res.node_random_misc);
    res.set->UpdateBufferDescriptorBinding(7,  res.length_thickness);
    res.set->UpdateBufferDescriptorBinding(8,  res.global_position);
    res.set->UpdateBufferDescriptorBinding(9,  res.global_rotation);
    res.set->UpdateBufferDescriptorBinding(10, res.depth_sorted_index);
    res.set->UpdateBufferDescriptorBinding(11, res.atlas);
  }

  // Push host bytes into every per-node channel + the header.
  res.header->Upload(soa.header);
  res.parent_index      ->UploadVector(soa.parent_index);
  res.depth             ->UploadVector(soa.depth);
  res.type_tag          ->UploadVector(soa.type_tag);
  res.local_rotation    ->UploadVector(soa.local_rotation);
  res.growth_state      ->UploadVector(soa.growth_state);
  res.node_random_misc  ->UploadVector(soa.node_random_misc);
  res.length_thickness  ->UploadVector(soa.length_thickness);
  res.global_position   ->UploadVector(soa.global_position);
  res.global_rotation   ->UploadVector(soa.global_rotation);
  res.depth_sorted_index->UploadVector(soa.depth_sorted_index);
}

// (Re)allocate + upload the curve atlas SSBO for an existing instance.
// Caller must have invoked EnsureGrowthBuffersAndUploadSoA() first so the
// descriptor set exists; this updates binding 11 to point at the freshly
// sized atlas buffer.
void UploadCurveAtlasBytes(LSystemGPUEngine::Impl& impl,
                           uint32_t instance_id,
                           const CurveAtlas& atlas) {
  auto res_it = impl.growth_gpu_resources.find(instance_id);
  if (res_it == impl.growth_gpu_resources.end()) return;
  auto& res = res_it->second;
  const uint32_t taps = static_cast<uint32_t>(atlas.taps.size());
  if (res.atlas_taps_count != taps || !res.atlas) {
    res.atlas = MakeStorageBuffer(16 + static_cast<VkDeviceSize>(taps) * sizeof(float),
                                  0, false);
    res.atlas_taps_count = taps;
    res.set->UpdateBufferDescriptorBinding(11, res.atlas);
  }
  // Serialize header + tap floats into a single std430-clean blob.
  std::vector<uint32_t> blob(4 + taps, 0);
  blob[0] = atlas.num_curves;
  blob[1] = atlas.samples_per_curve;
  blob[2] = atlas.stride;
  blob[3] = atlas._pad;
  if (taps > 0) {
    std::memcpy(blob.data() + 4, atlas.taps.data(), taps * sizeof(float));
  }
  res.atlas->UploadVector(blob);
}

// Allocate / reallocate per-instance SSBOs sized for the supplied SoA, write
// host-side bytes into each, then refresh the descriptor set bindings. The
// node_count and atlas tap counts are used as the sole reallocation
// trigger — channel byte sizes derive from those two scalars.
//
// Phase 2b.1 entry point used by the parity self-test; production code
// goes through UploadTasselGrowth + UploadCurveAtlas (which set up the
// same buffers incrementally).
void UploadGrowthInstance(LSystemGPUEngine::Impl& impl,
                          uint32_t instance_id,
                          const TasselGrowthSoA& soa,
                          const CurveAtlas& atlas) {
  EnsureGrowthBuffersAndUploadSoA(impl, instance_id, soa);
  UploadCurveAtlasBytes(impl, instance_id, atlas);
}

// Read GPU outputs back into the supplied SoA. Only the channels mutated by
// grow/propagate are downloaded; topology channels are left intact on the
// host side (they were never written GPU-side).
void DownloadGrowthOutputs(LSystemGPUEngine::Impl::GrowthGpuResources& res,
                           TasselGrowthSoA& soa) {
  if (res.node_count == 0) return;
  res.length_thickness->DownloadVector(soa.length_thickness, res.node_count);
  res.global_position ->DownloadVector(soa.global_position,  res.node_count);
  res.global_rotation ->DownloadVector(soa.global_rotation,  res.node_count);
  // grow.comp also mutates growth_state.x when advance_age is set.
  res.growth_state    ->DownloadVector(soa.growth_state,     res.node_count);
}

// CPU mirror of tassel_pack_internodes.comp. Used by the parity self-test.
// Mirrors the GPU pack exactly, including the divergence from the original
// CPU pack (HashToColor input is the SoA slot index `i`, not node.GetIndex()).
glm::vec3 PackHashToColor(uint32_t id) {
  const float hue = static_cast<float>((id * 2654435761u) & 1023u) / 1024.0f;
  const float s = 0.72f;
  const float v = 0.92f;
  const float h6 = hue * 6.0f;
  const int sector = static_cast<int>(std::floor(h6));
  const float f = h6 - static_cast<float>(sector);
  const float p = v * (1.0f - s);
  const float q = v * (1.0f - s * f);
  const float t = v * (1.0f - s * (1.0f - f));
  int idx = sector - 6 * (sector / 6);
  if (idx < 0) idx += 6;
  switch (idx) {
    case 0: return {v, t, p};
    case 1: return {q, v, p};
    case 2: return {p, v, t};
    case 3: return {p, q, v};
    case 4: return {t, p, v};
    default: return {v, p, q};
  }
}

void RunPackInternodesCpu(const TasselGrowthSoA& soa,
                          uint32_t color_mode,
                          const float instance_color[4],
                          std::vector<TasselInternodeInstance>& out) {
  const uint32_t n = soa.header.node_count;
  out.assign(n, TasselInternodeInstance{});
  // Cylinder axis fix in (x, y, z, w) layout. Matches CPU pack +
  // tassel_pack_internodes.comp constant.
  const glm::quat fix = glm::angleAxis(-glm::half_pi<float>(),
                                       glm::vec3(1.0f, 0.0f, 0.0f));
  for (uint32_t i = 0; i < n; ++i) {
    const uint32_t tag = soa.type_tag[i];
    const float len   = soa.length_thickness[i].x;
    const float halft = soa.length_thickness[i].y;
    const bool alive  = (tag == static_cast<uint32_t>(TasselTypeTag::Internode))
                     && (len > 0.0f) && (halft > 0.0f);
    if (!alive) {
      out[i] = TasselInternodeInstance{};
      out[i].rot = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);  // matches GPU sentinel
      continue;
    }
    const glm::vec4 gpos = soa.global_position[i];
    const glm::vec4 grot = soa.global_rotation[i];
    const glm::quat gq(grot.w, grot.x, grot.y, grot.z);
    glm::quat fixed = gq * fix;
    fixed = glm::normalize(fixed);

    glm::vec3 color;
    switch (color_mode) {
      case 3: color = PackHashToColor(i); break;                      // ByNode
      case 2: color = glm::vec3(instance_color[0], instance_color[1],
                                instance_color[2]); break;            // ByInstance
      case 1: color = glm::vec3(0.22f, 0.82f, 0.33f); break;          // ByType
      default: color = glm::vec3(0.45f, 0.55f, 0.20f); break;         // Shaded
    }
    out[i].pos_length  = glm::vec4(glm::vec3(gpos), len);
    out[i].rot         = glm::vec4(fixed.x, fixed.y, fixed.z, fixed.w);
    out[i].color_thick = glm::vec4(color, halft);
  }
}

}  // namespace

bool LSystemGPUEngine::DispatchGrowAndPropagateGpu(uint32_t instance_id,
                                                   float gdd_step,
                                                   float maturity_gdd,
                                                   bool advance_age) {
  auto soa_it = impl_->growth_shadows.find(instance_id);
  if (soa_it == impl_->growth_shadows.end()) return false;
  auto atlas_it = impl_->curve_shadows.find(instance_id);
  if (atlas_it == impl_->curve_shadows.end()) return false;

  if (!EnsureGrowthPipelines(*impl_)) return false;

  TasselGrowthSoA& soa = soa_it->second;
  const CurveAtlas& atlas = atlas_it->second;
  if (soa.header.node_count == 0) return true;  // nothing to do.

  UploadGrowthInstance(*impl_, instance_id, soa, atlas);
  auto& res = impl_->growth_gpu_resources[instance_id];

  GrowPushConstant grow_pc{};
  grow_pc.node_count   = soa.header.node_count;
  grow_pc.gdd_step     = gdd_step;
  grow_pc.maturity_gdd = maturity_gdd;
  grow_pc.advance_age  = advance_age ? 1u : 0u;

  const uint32_t wg = 64u;  // matches local_size_x in both .comp shaders.

  evo_engine::Platform::ImmediateSubmit([&](VkCommandBuffer cmd) {
    // ---- grow ----
    impl_->grow_pipeline->Bind(cmd);
    impl_->grow_pipeline->BindDescriptorSet(cmd, 0, res.set->GetVkDescriptorSet());
    impl_->grow_pipeline->PushConstant(cmd, 0, grow_pc);
    vkCmdDispatch(cmd, evo_engine::Platform::DivUp(soa.header.node_count, wg), 1, 1);
    evo_engine::Platform::EverythingBarrier(cmd);

    // ---- propagate (host loop over depth bands) ----
    impl_->propagate_pipeline->Bind(cmd);
    impl_->propagate_pipeline->BindDescriptorSet(cmd, 0, res.set->GetVkDescriptorSet());
    const uint32_t band_count = soa.header.depth_band_count;
    for (uint32_t b = 0; b < band_count; ++b) {
      const uint32_t s_begin = soa.header.depth_band_offsets[b];
      const uint32_t s_end   = soa.header.depth_band_offsets[b + 1];
      if (s_end <= s_begin) continue;
      PropagatePushConstant pc{};
      pc.band_begin = s_begin;
      pc.band_end   = s_end;
      impl_->propagate_pipeline->PushConstant(cmd, 0, pc);
      vkCmdDispatch(cmd, evo_engine::Platform::DivUp(s_end - s_begin, wg), 1, 1);
      evo_engine::Platform::EverythingBarrier(cmd);
    }
  });

  DownloadGrowthOutputs(res, soa);

  GpuFrameProfile& prof = impl_->last_profiles[instance_id];
  prof.module_count = soa.header.node_count;
  prof.grow_ms      = 0.0;  // not timed in 2b.1; Phase 2b.2 adds query pool.
  prof.propagate_ms = 0.0;
  return true;
}

// ---------------------------------------------------------------------------
//  Phase 2b.2: GPU pack_internodes.
// ---------------------------------------------------------------------------

namespace {

// Allocate / grow the per-instance TasselInternodeInstance SSBO to hold
// at least `count` entries. Mirrors the Phase 1b allocation in
// UploadTasselInternodes but does NOT upload any bytes — the GPU pack
// kernel writes them directly. Returns false if the descriptor layout
// hasn't been installed yet (i.e. headless test runs).
bool EnsureTasselInternodeBufferCapacity(LSystemGPUEngine::Impl& impl,
                                         uint32_t instance_id,
                                         uint32_t count) {
  if (!impl.tassel_internode_layout) return false;
  const uint32_t needed = std::max<uint32_t>(count, 1u);
  uint32_t& cap = impl.tassel_internode_buffer_capacities[instance_id];
  std::shared_ptr<evo_engine::Buffer>& buffer = impl.tassel_internode_buffers[instance_id];
  bool rebuilt = false;
  if (!buffer || cap < needed) {
    cap = NextPow2Capacity(needed);
    VkBufferCreateInfo bci{};
    bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bci.size  = static_cast<VkDeviceSize>(cap) * sizeof(TasselInternodeInstance);
    bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
              | VK_BUFFER_USAGE_TRANSFER_DST_BIT
              | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;  // SRC for parity readback
    bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo aci{};
    aci.usage = VMA_MEMORY_USAGE_AUTO;
    // RANDOM access required so the parity self-test can DownloadVector.
    // Phase 2b.3 production path could downgrade this to SEQUENTIAL_WRITE
    // if the readback cost becomes a measurable hit on real frames.
    aci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
    buffer = std::make_shared<evo_engine::Buffer>(bci, aci);
    rebuilt = true;
  }

  std::shared_ptr<evo_engine::DescriptorSet>& dset =
      impl.tassel_internode_descriptor_sets[instance_id];
  const bool fresh = !dset;
  if (fresh) dset = std::make_shared<evo_engine::DescriptorSet>(impl.tassel_internode_layout);
  if (fresh || rebuilt) dset->UpdateBufferDescriptorBinding(0, buffer);

  // Resize the CPU shadow so GetTasselInternodeCount returns the right
  // value. Bytes are unused on the CPU side once GPU pack runs.
  impl.tassel_internode_shadows[instance_id].assign(count, TasselInternodeInstance{});
  return true;
}

}  // namespace

bool LSystemGPUEngine::DispatchPackInternodesGpu(uint32_t instance_id,
                                                 uint32_t color_mode,
                                                 const float instance_color[4]) {
  auto soa_it = impl_->growth_shadows.find(instance_id);
  if (soa_it == impl_->growth_shadows.end()) return false;
  const auto res_it = impl_->growth_gpu_resources.find(instance_id);
  if (res_it == impl_->growth_gpu_resources.end()) return false;  // need prior grow upload
  if (!EnsurePackPipeline(*impl_)) return false;

  const TasselGrowthSoA& soa = soa_it->second;
  const uint32_t n = soa.header.node_count;
  if (n == 0) return true;

  if (!EnsureTasselInternodeBufferCapacity(*impl_, instance_id, n)) return false;

  PackPushConstant pc{};
  pc.node_count = n;
  pc.color_mode = color_mode;
  if (instance_color) {
    pc.instance_color[0] = instance_color[0];
    pc.instance_color[1] = instance_color[1];
    pc.instance_color[2] = instance_color[2];
    pc.instance_color[3] = instance_color[3];
  }

  auto& growth_res = res_it->second;
  auto& dset = impl_->tassel_internode_descriptor_sets[instance_id];
  const uint32_t wg = 64u;

  evo_engine::Platform::ImmediateSubmit([&](VkCommandBuffer cmd) {
    impl_->pack_internodes_pipeline->Bind(cmd);
    impl_->pack_internodes_pipeline->BindDescriptorSet(cmd, 0, growth_res.set->GetVkDescriptorSet());
    impl_->pack_internodes_pipeline->BindDescriptorSet(cmd, 1, dset->GetVkDescriptorSet());
    impl_->pack_internodes_pipeline->PushConstant(cmd, 0, pc);
    vkCmdDispatch(cmd, evo_engine::Platform::DivUp(n, wg), 1, 1);
    evo_engine::Platform::EverythingBarrier(cmd);
  });
  return true;
}

float LSystemGPUEngine::RunGrowPropagateSelfTest(std::string* failure_reason) {
  // --------------------------------------------------------------------
  // Build a deterministic synthetic SoA: linear chain of 8 internodes
  // with 2 lateral spurs at depth 3 and 5. Total 10 nodes. Curve atlas
  // exposes two simple curves (linear ramp + sin half-cycle) at the same
  // bake density used by CurveBaker.
  // --------------------------------------------------------------------
  const uint32_t kChain = 8;
  const uint32_t kLateral1Parent = 3;
  const uint32_t kLateral2Parent = 5;
  const uint32_t n = kChain + 2;

  TasselGrowthSoA soa;
  soa.Resize(n);
  // Topology: chain 0->1->2->...->7 then 8 child of 3, 9 child of 5.
  soa.parent_index[0] = -1;
  for (uint32_t i = 1; i < kChain; ++i) soa.parent_index[i] = static_cast<int32_t>(i - 1);
  soa.parent_index[8] = static_cast<int32_t>(kLateral1Parent);
  soa.parent_index[9] = static_cast<int32_t>(kLateral2Parent);

  // Depth = parent depth + 1, root = 0. depth_sorted_index = identity here
  // because the chain is already topologically ordered and laterals come
  // last (their depth is 4 and 6 respectively, larger than chain depth 3/5
  // so identity is NOT a valid depth-sort). Build a real one.
  for (uint32_t i = 0; i < n; ++i) {
    const int32_t p = soa.parent_index[i];
    soa.depth[i] = (p < 0) ? 0u : (soa.depth[p] + 1u);
    soa.type_tag[i] = static_cast<uint32_t>(TasselTypeTag::Internode);
    soa.local_rotation[i] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);  // identity
    // growth_state: (age_gdd, target_length, target_thickness, flags)
    soa.growth_state[i]   = glm::vec4(50.0f, 0.5f + 0.05f * static_cast<float>(i),
                                       0.04f, 0.0f);
  }

  // Build depth_sorted_index by stable bucket sort on depth.
  std::vector<std::vector<uint32_t>> by_depth;
  uint32_t max_depth = 0;
  for (uint32_t i = 0; i < n; ++i) max_depth = std::max(max_depth, soa.depth[i]);
  by_depth.resize(static_cast<size_t>(max_depth) + 1);
  for (uint32_t i = 0; i < n; ++i) by_depth[soa.depth[i]].push_back(i);
  uint32_t cursor = 0;
  soa.header.depth_band_offsets[0] = 0;
  for (uint32_t d = 0; d <= max_depth; ++d) {
    for (const uint32_t idx : by_depth[d]) soa.depth_sorted_index[cursor++] = idx;
    soa.header.depth_band_offsets[d + 1] = cursor;
  }
  soa.header.depth_band_count = max_depth + 1u;
  soa.header.instance_id = 0xFEEDu;

  // Seed root pose so propagate has something meaningful.
  soa.global_position[0] = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
  // Root rotation: small tilt so quat math exercises non-identity paths.
  const glm::quat q_seed = glm::angleAxis(glm::radians(15.0f),
                                          glm::normalize(glm::vec3(1.0f, 0.0f, 0.0f)));
  soa.global_rotation[0] = glm::vec4(q_seed.x, q_seed.y, q_seed.z, q_seed.w);

  // Per-node local quaternion: small twist around parent +Y so propagate
  // composes a non-trivial Hamilton chain.
  for (uint32_t i = 1; i < n; ++i) {
    const float angle = glm::radians(8.0f);
    const glm::quat lq = glm::angleAxis(angle, glm::vec3(0.0f, 1.0f, 0.0f));
    soa.local_rotation[i] = glm::vec4(lq.x, lq.y, lq.z, lq.w);
  }

  // --- Curve atlas: linear ramp (length factor) + half-sine (thickness). --
  CurveAtlas atlas;
  atlas.num_curves = 2;
  atlas.samples_per_curve = 64;
  atlas.stride = atlas.samples_per_curve * 2u;
  atlas._pad = 0;
  atlas.taps.assign(static_cast<size_t>(atlas.num_curves) * atlas.stride, 0.0f);
  for (uint32_t s = 0; s < atlas.samples_per_curve; ++s) {
    const float t = static_cast<float>(s) / static_cast<float>(atlas.samples_per_curve - 1u);
    atlas.taps[0 * atlas.stride + s * 2u + 0u] = t;                          // ramp
    atlas.taps[1 * atlas.stride + s * 2u + 0u] = std::sin(t * 3.14159265f);  // half-sine
  }

  // Stash a temporary instance.
  const uint32_t test_instance = 0xFFFFFF01u;
  impl_->growth_shadows[test_instance] = soa;
  impl_->curve_shadows[test_instance]  = atlas;

  // CPU baseline on a copy.
  TasselGrowthSoA cpu_copy = soa;
  RunGrowCpu(cpu_copy, atlas, /*gdd_step=*/3.0f, /*maturity_gdd=*/100.0f, /*advance_age=*/true);
  RunPropagateCpu(cpu_copy);

  // GPU run mutates the stashed shadow in place.
  const bool ok = DispatchGrowAndPropagateGpu(test_instance, 3.0f, 100.0f, true);
  if (!ok) {
    if (failure_reason) *failure_reason = "compute pipelines unavailable";
    impl_->growth_shadows.erase(test_instance);
    impl_->curve_shadows.erase(test_instance);
    impl_->growth_gpu_resources.erase(test_instance);
    return -1.0f;
  }

  const TasselGrowthSoA& gpu_out = impl_->growth_shadows[test_instance];

  float max_err = 0.0f;
  for (uint32_t i = 0; i < n; ++i) {
    max_err = std::max(max_err, std::fabs(cpu_copy.length_thickness[i].x - gpu_out.length_thickness[i].x));
    max_err = std::max(max_err, std::fabs(cpu_copy.length_thickness[i].y - gpu_out.length_thickness[i].y));
    for (int k = 0; k < 4; ++k) {
      max_err = std::max(max_err, std::fabs(cpu_copy.global_position[i][k] - gpu_out.global_position[i][k]));
      max_err = std::max(max_err, std::fabs(cpu_copy.global_rotation[i][k] - gpu_out.global_rotation[i][k]));
    }
    max_err = std::max(max_err, std::fabs(cpu_copy.growth_state[i].x - gpu_out.growth_state[i].x));
  }

  if (failure_reason && max_err > 1e-4f) {
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "GPU/CPU divergence: max_abs_err=%.6f over %u nodes",
                  static_cast<double>(max_err), n);
    *failure_reason = buf;
  }

  // -------------------------------------------------------------------------
  // Phase 2b.2: pack_internodes parity. Reuses the post-grow/propagate
  // SoA already on the GPU. Runs both the CPU mirror and the GPU pack
  // for each color mode (Shaded + ByNode cover the deterministic + hash
  // branches; ByType is constant; ByInstance is just a passthrough).
  // -------------------------------------------------------------------------
  if (max_err <= 1e-4f && impl_->tassel_internode_layout) {
    auto check_mode = [&](uint32_t color_mode, const float ic[4], const char* label) -> float {
      std::vector<TasselInternodeInstance> cpu_expected;
      RunPackInternodesCpu(gpu_out, color_mode, ic, cpu_expected);

      const bool ok_pack = DispatchPackInternodesGpu(test_instance, color_mode, ic);
      if (!ok_pack) return -1.0f;

      auto buf_it = impl_->tassel_internode_buffers.find(test_instance);
      if (buf_it == impl_->tassel_internode_buffers.end()) return -1.0f;
      std::vector<TasselInternodeInstance> gpu_actual;
      buf_it->second->DownloadVector(gpu_actual, n);

      float local_err = 0.0f;
      for (uint32_t i = 0; i < n; ++i) {
        for (int k = 0; k < 4; ++k) {
          local_err = std::max(local_err, std::fabs(cpu_expected[i].pos_length[k]  - gpu_actual[i].pos_length[k]));
          local_err = std::max(local_err, std::fabs(cpu_expected[i].rot[k]         - gpu_actual[i].rot[k]));
          local_err = std::max(local_err, std::fabs(cpu_expected[i].color_thick[k] - gpu_actual[i].color_thick[k]));
        }
      }
      if (failure_reason && local_err > 1e-4f) {
        char buf[200];
        std::snprintf(buf, sizeof(buf),
                      "pack divergence (mode=%s): max_abs_err=%.6f", label,
                      static_cast<double>(local_err));
        *failure_reason = buf;
      }
      return local_err;
    };

    const float shaded_ic[4]  = {0.0f, 0.0f, 0.0f, 1.0f};
    const float instance_ic[4] = {0.31f, 0.42f, 0.53f, 1.0f};
    const float pack_err_shaded = check_mode(0u, shaded_ic,   "Shaded");
    const float pack_err_byinst = check_mode(2u, instance_ic, "ByInstance");
    const float pack_err_bynode = check_mode(3u, shaded_ic,   "ByNode");
    if (pack_err_shaded < 0.0f || pack_err_byinst < 0.0f || pack_err_bynode < 0.0f) {
      if (failure_reason) *failure_reason = "pack pipeline unavailable";
      max_err = std::max(max_err, 1.0f);  // signal failure to the caller
    } else {
      max_err = std::max(max_err, pack_err_shaded);
      max_err = std::max(max_err, pack_err_byinst);
      max_err = std::max(max_err, pack_err_bynode);
    }
  }

  // Clean up the synthetic instance so it doesn't leak through DestroyInstance.
  impl_->growth_shadows.erase(test_instance);
  impl_->curve_shadows.erase(test_instance);
  impl_->growth_gpu_resources.erase(test_instance);
  impl_->last_profiles.erase(test_instance);
  impl_->tassel_internode_descriptor_sets.erase(test_instance);
  impl_->tassel_internode_buffers.erase(test_instance);
  impl_->tassel_internode_buffer_capacities.erase(test_instance);
  impl_->tassel_internode_shadows.erase(test_instance);
  return max_err;
}

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
