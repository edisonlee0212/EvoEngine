#pragma once
#include "DevelopmentalStrandModelData.hpp"
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
  /// GPU-side skeleton node for surface mesh generation (matches GLSL SkeletonNode, 80 bytes).
  struct GpuSkeletonNode {
    glm::vec3 global_position{};       ///< Start position of the internode.
    float length = 0.0f;               ///< Internode length.

    glm::vec4 regulated_global_rotation{0, 0, 0, 1};  ///< Quaternion (x,y,z,w).

    glm::vec3 global_end_position{};   ///< End position (= global_position + dir * length).
    float strand_radius = 0.002f;      ///< 3D strand radius for this node.

    float root_distance = 0.0f;        ///< Distance from root along the skeleton.
    float max_root_distance = 1.0f;    ///< Maximum root distance across the tree (for normalisation).
    int32_t parent_handle = -1;        ///< Parent skeleton node handle (-1 for root).
    uint32_t is_end_node = 0;          ///< 1 if leaf/tip node, 0 otherwise.

    uint32_t boundary_particle_count = 0; ///< Number of boundary particles at this profile.
    uint32_t total_particle_count = 0;    ///< Total number of particles at this profile.
    uint32_t profile_index = 0xFFFFFFFF;  ///< Index into cpu_profiles_ / contour_infos_ (0xFFFFFFFF if none).
    float _pad0 = 0.0f;
  };
  static_assert(sizeof(GpuSkeletonNode) == 80, "GpuSkeletonNode must be 80 bytes for GPU alignment");

  /// GPU-side particle (matches GLSL ProfileParticle, 56 bytes).
  struct GpuProfileParticle {
    glm::vec2 position{};
    glm::vec2 last_position{};
    glm::vec2 acceleration{};
    glm::vec2 delta_position{};
    uint32_t enable = 0;
    int32_t strand_handle = -1;
    int32_t strand_segment_handle = -1;
    int32_t node_handle = -1;
    uint32_t birth_step = 0;
    uint32_t _pad1 = 0;
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

  /// GPU-side contour info per profile (matches GLSL ContourInfo, 16 bytes).
  struct GpuContourInfo {
    uint32_t vertex_offset = 0;   ///< Start index in vertex_buffer_ for this contour's vertices.
    uint32_t vertex_count = 0;    ///< Number of boundary vertices in sorted angular order.
    int32_t node_handle = -1;     ///< Skeleton node handle for this profile.
    uint32_t _pad = 0;
  };
  static_assert(sizeof(GpuContourInfo) == 16, "GpuContourInfo must be 16 bytes");

  /// Atomic counters for surface mesh generation (matches GLSL Counters, 8 bytes).
  struct GpuMeshCounters {
    uint32_t total_vertices = 0;
    uint32_t total_triangles = 0;
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
  void UploadFromSkeleton(DevelopmentalStrandModelSkeleton& skeleton,
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
  void DownloadToSkeleton(DevelopmentalStrandModelSkeleton& skeleton);

  /**
   * @brief Uploads skeleton node transforms and metadata to the GPU.
   *
   * Builds a GpuSkeletonNode array from the skeleton's sorted node list and
   * uploads it to skeleton_nodes_buffer_. This buffer is consumed by GPU-side
   * surface mesh generation passes (Phase 2) that replace CPU ApplyProfiles().
   * @param skeleton The skeleton with up-to-date node info.
   * @param params Strand model parameters (for strand_radius_distribution).
   */
  void UploadSkeleton(const DevelopmentalStrandModelSkeleton& skeleton, const StrandModelParameters& params);

  /**
   * @brief Generates a triangle surface mesh from profile boundary contours on the GPU.
   *
   * Two compute passes: (1) extract boundary particles per profile, sort by angle,
   * project 2D→3D via skeleton transforms, write Vertex buffer; (2) stitch adjacent
   * contours into triangle strips, write index buffer.
   * Must be called after UploadSkeleton() and after particle packing (Simulate).
   * Results are left in vertex_buffer_ and index_buffer_ for later BLAS building.
   */
  void GenerateSurfaceMesh();

  /// Number of vertices produced by the last GenerateSurfaceMesh() call.
  uint32_t GetSurfaceVertexCount() const { return surface_vertex_count_; }
  /// Number of triangles produced by the last GenerateSurfaceMesh() call.
  uint32_t GetSurfaceTriangleCount() const { return surface_triangle_count_; }

  /// GPU vertex buffer (engine Vertex layout, 80 bytes each) from surface mesh generation.
  [[nodiscard]] const std::shared_ptr<Buffer>& GetSurfaceVertexBuffer() const { return vertex_buffer_; }
  /// GPU index buffer (uvec3 triangles) from surface mesh generation.
  [[nodiscard]] const std::shared_ptr<Buffer>& GetSurfaceIndexBuffer() const { return index_buffer_; }

  /**
   * @brief Downloads the GPU-generated surface mesh to CPU vectors.
   *
   * After GenerateSurfaceMesh(), call this to retrieve the mesh data for
   * creating a Mesh asset (the engine only supports CPU→GPU SetVertices).
   * @param[out] vertices Downloaded vertex data.
   * @param[out] triangles Downloaded triangle index data.
   * @return True if data was downloaded, false if no surface mesh exists.
   */
  bool DownloadSurfaceMesh(std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles) const;

 private:
  /// Shared descriptor set layout for the three SSBOs (particles, profiles, grid).
  inline static std::shared_ptr<DescriptorSetLayout> profile_packing_layout_;

  /// Descriptor set layout for surface mesh generation (7 SSBOs).
  inline static std::shared_ptr<DescriptorSetLayout> surface_mesh_layout_;

  /// Compute pipelines — one per simulation pass.
  inline static std::shared_ptr<ComputePipeline> apply_forces_pipeline_;
  inline static std::shared_ptr<ComputePipeline> clear_grid_pipeline_;
  inline static std::shared_ptr<ComputePipeline> build_grid_pipeline_;
  inline static std::shared_ptr<ComputePipeline> solve_collisions_pipeline_;
  inline static std::shared_ptr<ComputePipeline> verlet_update_pipeline_;

  /// Compute pipelines for surface mesh generation.
  inline static std::shared_ptr<ComputePipeline> extract_project_pipeline_;
  inline static std::shared_ptr<ComputePipeline> stitch_contours_pipeline_;

  /// GPU buffers.
  std::shared_ptr<Buffer> particles_buffer_;
  std::shared_ptr<Buffer> profiles_buffer_;
  std::shared_ptr<Buffer> grid_buffer_;
  std::shared_ptr<Buffer> skeleton_nodes_buffer_;

  /// Surface mesh generation GPU buffers.
  std::shared_ptr<Buffer> contour_info_buffer_;
  std::shared_ptr<Buffer> vertex_buffer_;
  std::shared_ptr<Buffer> index_buffer_;
  std::shared_ptr<Buffer> counters_buffer_;

  /// CPU staging for skeleton node data (kept for debugging/inspection).
  std::vector<GpuSkeletonNode> cpu_skeleton_nodes_;
  uint32_t total_skeleton_nodes_ = 0;

  /// Descriptor sets (one per frame in flight).
  std::vector<std::shared_ptr<DescriptorSet>> descriptor_sets_;

  /// Descriptor sets for surface mesh generation.
  std::vector<std::shared_ptr<DescriptorSet>> surface_mesh_descriptor_sets_;

  /// Surface mesh output counts (read back after GenerateSurfaceMesh).
  uint32_t surface_vertex_count_ = 0;
  uint32_t surface_triangle_count_ = 0;

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

  /// Push constants for surface mesh generation passes.
  struct ExtractProjectPushConstant {
    uint32_t total_profiles;
    uint32_t max_vertices;
  };
  struct StitchContoursPushConstant {
    uint32_t total_profiles;
    uint32_t max_triangles;
  };

  bool surface_mesh_initialized_ = false;
  void InitSurfaceMesh();
};

}  // namespace eco_sys_lab_plugin
