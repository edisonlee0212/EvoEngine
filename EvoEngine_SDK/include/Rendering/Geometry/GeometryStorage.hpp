#pragma once
#include "GraphicsResources.hpp"
#include "ISingleton.hpp"
#include "Platform.hpp"
#include "Vertex.hpp"

namespace evo_engine {

/**
 * @brief Represents a chunk of vertex data used in meshlets.
 */
struct VertexDataChunk {
  /**
   * @brief An array of vertex data.
   */
  Vertex vertex_data[Platform::Constants::meshlet_max_vertices_size] = {};
};

/**
 * @brief Represents a meshlet structure with triangle indices and associated metadata.
 */
struct Meshlet {
  /**
   * @brief Stores the triangle indices of the meshlet, supporting up to 126 triangles.
   */
  glm::u8vec3 triangles[Platform::Constants::meshlet_max_triangles_size] = {};

  /**
   * @brief Number of vertices used in the meshlet.
   */
  uint32_t vertices_size = 0;

  /**
   * @brief Number of triangles used in the meshlet.
   */
  uint32_t triangle_size = 0;

  /**
   * @brief Index pointing to the corresponding vertex data chunk.
   */
  uint32_t vertex_chunk_index = 0;
};

/**
 * @brief Represents a chunk of skinned vertex data used in skinned meshlets.
 */
struct SkinnedVertexDataChunk {
  /**
   * @brief An array of skinned vertex data.
   */
  SkinnedVertex skinned_vertex_data[Platform::Constants::meshlet_max_vertices_size] = {};
};

/**
 * @brief Represents a skinned meshlet structure with triangle indices and associated metadata.
 */
struct SkinnedMeshlet {
  /**
   * @brief Stores the triangle indices of the skinned meshlet, supporting up to 126 triangles.
   */
  glm::u8vec3 skinned_triangles[Platform::Constants::meshlet_max_triangles_size] = {};

  /**
   * @brief Number of skinned vertices used in the skinned meshlet.
   */
  uint32_t skinned_vertices_size = 0;

  /**
   * @brief Number of skinned triangles used in the skinned meshlet.
   */
  uint32_t skinned_triangle_size = 0;

  /**
   * @brief Index pointing to the corresponding skinned vertex data chunk.
   */
  uint32_t skinned_vertex_chunk_index = 0;
};

/**
 * @brief Represents a chunk of strand point data used in strand meshlets.
 */
struct StrandPointDataChunk {
  /**
   * @brief An array of strand point data.
   */
  StrandPoint strand_point_data[Platform::Constants::meshlet_max_vertices_size] = {};
};

/**
 * @brief Represents a strand meshlet structure with segment information and associated metadata.
 */
struct StrandMeshlet {
  /**
   * @brief Stores the segment indices of the strand meshlet, supporting up to 126 triangles.
   */
  glm::u8vec4 segments[Platform::Constants::meshlet_max_triangles_size] = {};

  /**
   * @brief Number of strand points used in the strand meshlet.
   */
  uint32_t strand_points_size = 0;

  /**
   * @brief Number of segments used in the strand meshlet.
   */
  uint32_t segment_size = 0;

  /**
   * @brief Index pointing to the corresponding strand point data chunk.
   */
  uint32_t strand_point_chunk_index = 0;
};

/**
 * @brief Holds range information for geometry storage.
 */
class RangeDescriptor {
  friend class GeometryStorage;

  /**
   * @brief Internal handle associated with the range descriptor.
   */
  Handle handle_;

 public:
  /**
   * @brief Offset value of the range descriptor.
   */
  uint32_t offset;

  /**
   * @brief Range value for the descriptor.
   *
   * - When used for meshlets: Represents the count of meshlets for this geometry.
   * - When used for triangles: Represents the count of triangles, including space for empty fillers.
   */
  uint32_t range;

  /**
   * @brief Offset for the previous frame's data.
   */
  uint32_t prev_frame_offset;

  /**
   * @brief Number of indices in the current frame.
   */
  uint32_t index_count;

  /**
   * @brief Number of indices in the previous frame.
   */
  uint32_t prev_frame_index_count;
};

/**
 * @brief Represents data related to a particle's transformation and coloring.
 */
struct ParticleInfo {
  /**
   * @brief Instance matrix representing the particle's transformation.
   */
  Transform instance_matrix = {};

  /**
   * @brief Instance color of the particle.
   */
  glm::vec4 instance_color = glm::vec4(1.0f);
};

/**
 * @brief Enumerations representing the status of particle information list data.
 */
enum class ParticleInfoListDataStatus {
  Updated,        ///< Data has been updated.
  UpdatePending,  ///< Data update is pending.
  Removed         ///< Data has been removed.
};

/**
 * @brief Encapsulates the data for a list of particle information.
 */
struct ParticleInfoListData {
  /**
   * @brief Buffer associated with the particle info list data.
   */
  std::shared_ptr<Buffer> buffer;

  /**
   * @brief Descriptor set for the particle info list data.
   */
  std::shared_ptr<DescriptorSet> descriptor_set;

  /**
   * @brief List of particle information.
   */
  std::vector<ParticleInfo> particle_info_list;

  /**
   * @brief Status of the particle info list data.
   */
  ParticleInfoListDataStatus status = ParticleInfoListDataStatus::Updated;

  /**
   * @brief Range descriptor for the particle info list data.
   */
  std::shared_ptr<RangeDescriptor> range_descriptor;
};

class GeometryStorage final {
  EVOENGINE_SINGLETON_INSTANCE(GeometryStorage)
  std::vector<VertexDataChunk> vertex_data_chunks_ = {};
  std::vector<Meshlet> meshlets_ = {};
  std::vector<std::shared_ptr<RangeDescriptor>> meshlet_range_descriptor_;
  std::vector<glm::uvec3> triangles_;
  std::vector<std::shared_ptr<RangeDescriptor>> triangle_range_descriptor_;

  std::shared_ptr<Buffer> vertex_buffer_ = {};
  std::shared_ptr<Buffer> meshlet_buffer_ = {};
  std::shared_ptr<Buffer> triangle_buffer_ = {};
  bool require_mesh_data_device_update_ = {};

  std::vector<SkinnedVertexDataChunk> skinned_vertex_data_chunks_ = {};
  std::vector<SkinnedMeshlet> skinned_meshlets_ = {};
  std::vector<std::shared_ptr<RangeDescriptor>> skinned_meshlet_range_descriptor_;
  std::vector<glm::uvec3> skinned_triangles_;
  std::vector<std::shared_ptr<RangeDescriptor>> skinned_triangle_range_descriptor_;

  std::shared_ptr<Buffer> skinned_vertex_buffer_ = {};
  std::shared_ptr<Buffer> skinned_meshlet_buffer_ = {};
  std::shared_ptr<Buffer> skinned_triangle_buffer_ = {};
  bool require_skinned_mesh_data_device_update_ = {};

  std::vector<StrandPointDataChunk> strand_point_data_chunks_ = {};
  std::vector<StrandMeshlet> strand_meshlets_ = {};
  std::vector<std::shared_ptr<RangeDescriptor>> strand_meshlet_range_descriptor_;
  std::vector<glm::uvec4> segments_;
  std::vector<std::shared_ptr<RangeDescriptor>> segment_range_descriptor_;

  std::shared_ptr<Buffer> strand_point_buffer_ = {};
  std::shared_ptr<Buffer> strand_meshlet_buffer_ = {};
  std::shared_ptr<Buffer> segment_buffer_ = {};
  bool require_strand_mesh_data_device_update_ = {};

  void UploadData();
  friend class RenderLayer;
  friend class Resources;
  friend class Platform;
  static void DeviceSync();
  static void Initialize();

  std::vector<ParticleInfoListData> particle_info_list_data_list_;
  uint32_t version_ = 0;
  bool initialized_ = false;

 public:
  [[nodiscard]] static uint32_t GetVersion();
  static const std::shared_ptr<Buffer>& GetTriangleBuffer();
  static const std::shared_ptr<Buffer>& GetVertexBuffer();
  static const std::shared_ptr<Buffer>& GetMeshletBuffer();

  static const std::shared_ptr<Buffer>& GetSkinnedVertexBuffer();
  static const std::shared_ptr<Buffer>& GetSkinnedMeshletBuffer();

  static const std::shared_ptr<Buffer>& GetStrandPointBuffer();
  static const std::shared_ptr<Buffer>& GetStrandMeshletBuffer();

  static void BindVertices(VkCommandBuffer vk_command_buffer);
  static void BindSkinnedVertices(VkCommandBuffer vk_command_buffer);
  static void BindStrandPoints(VkCommandBuffer vk_command_buffer);

  [[nodiscard]] static const Vertex& PeekVertex(size_t vertex_index);
  [[nodiscard]] static const SkinnedVertex& PeekSkinnedVertex(size_t skinned_vertex_index);
  [[nodiscard]] static const StrandPoint& PeekStrandPoint(size_t strand_point_index);

  static void AllocateMesh(const Handle& handle, std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles,
                           const std::shared_ptr<RangeDescriptor>& target_meshlet_range,
                           const std::shared_ptr<RangeDescriptor>& target_triangle_range);
  static void AllocateSkinnedMesh(const Handle& handle, const std::vector<SkinnedVertex>& skinned_vertices,
                                  const std::vector<glm::uvec3>& skinned_triangles,
                                  const std::shared_ptr<RangeDescriptor>& target_skinned_meshlet_range,
                                  const std::shared_ptr<RangeDescriptor>& target_skinned_triangle_range);
  static void AllocateStrands(const Handle& handle, const std::vector<StrandPoint>& strand_points,
                              const std::vector<glm::uvec4>& segments,
                              const std::shared_ptr<RangeDescriptor>& target_strand_meshlet_range,
                              const std::shared_ptr<RangeDescriptor>& target_segment_range);

  static void FreeMesh(const Handle& handle);
  static void FreeSkinnedMesh(const Handle& handle);
  static void FreeStrands(const Handle& handle);

  static void AllocateParticleInfo(const Handle& handle, const std::shared_ptr<RangeDescriptor>& range_descriptor);
  static void UpdateParticleInfo(const std::shared_ptr<RangeDescriptor>& range_descriptor,
                                 const std::vector<ParticleInfo>& particle_infos);
  static void FreeParticleInfo(const std::shared_ptr<RangeDescriptor>& range_descriptor);
  [[nodiscard]] static const std::vector<ParticleInfo>& PeekParticleInfoList(
      const std::shared_ptr<RangeDescriptor>& range_descriptor);
  [[nodiscard]] static const std::shared_ptr<DescriptorSet>& PeekDescriptorSet(
      const std::shared_ptr<RangeDescriptor>& range_descriptor);

  [[nodiscard]] static const Meshlet& PeekMeshlet(uint32_t meshlet_index);
  [[nodiscard]] static const SkinnedMeshlet& PeekSkinnedMeshlet(uint32_t skinned_meshlet_index);
  [[nodiscard]] static const StrandMeshlet& PeekStrandMeshlet(uint32_t strand_meshlet_index);

  static void OnDestroy();
};
}  // namespace evo_engine
