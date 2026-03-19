#pragma once
#include "ProceduralStrandModelData.hpp"
#include "StrandModelParameters.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @brief GPU compute pipeline for 2D profile packing simulation.
 *
 * Ports the CPU-side StrandModelProfile particle physics (center attraction force,
 * spatial-hash grid collision, Verlet integration) to five Vulkan compute shader passes.
 * All active profile particles are flattened into a single contiguous SSBO so that one
 * dispatch covers all profiles simultaneously. Inactive (frozen) profiles are skipped
 * via a per-profile `active` flag.
 */
class GpuProfileSimulator {
 public:
  /// GPU-side particle (matches GLSL ProfileParticle, 48 bytes).
  struct GpuProfileParticle {
    glm::vec2 position{};
    glm::vec2 last_position{};
    glm::vec2 acceleration{};
    glm::vec2 delta_position{};
    uint32_t enable = 0;
    int32_t strand_handle = -1;
    int32_t strand_segment_handle = -1;
    int32_t node_handle = -1;
  };

  /// GPU-side profile info (matches GLSL ProfileInfo, 48 bytes).
  struct GpuProfileInfo {
    uint32_t particle_offset = 0;
    uint32_t particle_count = 0;
    uint32_t grid_offset = 0;
    uint32_t is_active = 0;

    glm::vec2 grid_min_bound{};
    float grid_cell_size = 2.0f;
    int32_t grid_resolution_x = 1;

    int32_t grid_resolution_y = 1;
    float particle_softness = 0.1f;
    float damping = 0.02f;
    float max_speed = 60.0f;
  };

  /// GPU-side grid cell (matches GLSL GridCell, 32 bytes).
  struct GpuGridCell {
    uint32_t count = 0;
    int32_t handles[4]{};
    int32_t padding0 = 0;
    int32_t padding1 = 0;
    int32_t padding2 = 0;
  };

  /**
   * @brief One-time initialization of compute pipelines, descriptor set layout, and GPU buffers.
   */
  void Init();

  /**
   * @brief Uploads profile particle data from the skeleton to the GPU.
   *
   * Flattens all node profiles into a contiguous particle buffer, builds per-profile
   * metadata (offsets, grid params), and marks selected nodes as active.
   * @param skeleton The procedural strand model skeleton containing profiles.
   * @param active_nodes Node handles whose profiles should simulate (others are frozen).
   */
  /**
   * @brief Uploads profile particle data from the skeleton to the GPU.
   *
   * Flattens all node profiles into a contiguous particle buffer, builds per-profile
   * metadata (offsets, grid params), and marks selected nodes as active.
   * @param skeleton The procedural strand model skeleton containing profiles.
   * @param active_nodes Node handles whose profiles should simulate (others are frozen).
   * @param freeze_interior When true, non-boundary particles on active profiles are uploaded
   *        with enable=2 (frozen), making them immovable collision obstacles. Boundary
   *        particles remain fully active. When false, all particles on active profiles are active.
   */
  void UploadFromSkeleton(ProceduralStrandModelSkeleton& skeleton,
                          const std::vector<SkeletonNodeHandle>& active_nodes,
                          bool freeze_interior = false);

  /**
   * @brief Runs N iterations of the five-pass profile packing simulation on GPU.
   * @param iterations Number of simulation iterations.
   * @param params Strand model parameters (center attraction strength, physics settings).
   */
  void Simulate(uint32_t iterations, const StrandModelParameters& params);

  /**
   * @brief Downloads GPU particle positions back into the CPU skeleton profiles.
   * @param skeleton The skeleton whose profiles will be updated with GPU results.
   */
  void DownloadToSkeleton(ProceduralStrandModelSkeleton& skeleton);

 private:
  /// Shared descriptor set layout for the three SSBOs (particles, profiles, grid).
  inline static std::shared_ptr<DescriptorSetLayout> profile_packing_layout_;

  /// Compute pipelines — one per simulation pass.
  inline static std::shared_ptr<ComputePipeline> apply_forces_pipeline_;
  inline static std::shared_ptr<ComputePipeline> clear_grid_pipeline_;
  inline static std::shared_ptr<ComputePipeline> build_grid_pipeline_;
  inline static std::shared_ptr<ComputePipeline> solve_collisions_pipeline_;
  inline static std::shared_ptr<ComputePipeline> verlet_update_pipeline_;

  /// GPU buffers.
  std::shared_ptr<Buffer> particles_buffer_;
  std::shared_ptr<Buffer> profiles_buffer_;
  std::shared_ptr<Buffer> grid_buffer_;

  /// Descriptor sets (one per frame in flight).
  std::vector<std::shared_ptr<DescriptorSet>> descriptor_sets_;

  /// CPU staging data (kept across Upload/Download for indexing).
  std::vector<GpuProfileParticle> cpu_particles_;
  std::vector<GpuProfileInfo> cpu_profiles_;
  /// Maps from profile index → skeleton node handle (for download).
  std::vector<SkeletonNodeHandle> profile_to_node_;
  uint32_t total_grid_cells_ = 0;
  uint32_t total_particles_ = 0;
  uint32_t total_profiles_ = 0;

  bool initialized_ = false;

  /// Push constant structs matching the GLSL push constants.
  struct ApplyForcesPushConstant {
    uint32_t total_particles;
    uint32_t total_profiles;
    float center_attraction_strength;
  };
  struct ClearGridPushConstant {
    uint32_t total_grid_cells;
  };
  struct BuildGridPushConstant {
    uint32_t total_particles;
    uint32_t total_profiles;
  };
  struct SolveCollisionsPushConstant {
    uint32_t total_particles;
    uint32_t total_profiles;
  };
  struct VerletUpdatePushConstant {
    uint32_t total_particles;
    uint32_t total_profiles;
    float dt;
    float damping;
    float max_speed;
  };
};

}  // namespace eco_sys_lab_plugin
