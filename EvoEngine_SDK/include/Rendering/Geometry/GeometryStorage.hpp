#pragma once
#include "Platform.hpp"
#include "GraphicsResources.hpp"
#include "ISingleton.hpp"
#include "Vertex.hpp"

namespace evo_engine {
struct VertexDataChunk {
  Vertex vertex_data[Platform::Constants::meshlet_max_vertices_size] = {};
};

struct Meshlet {
  glm::u8vec3 triangles[Platform::Constants::meshlet_max_triangles_size] = {};  // up to 126 triangles
  uint32_t vertices_size = 0;
  uint32_t triangle_size = 0;
  uint32_t vertex_chunk_index = 0;
};

struct SkinnedVertexDataChunk {
  SkinnedVertex skinned_vertex_data[Platform::Constants::meshlet_max_vertices_size] = {};
};
struct SkinnedMeshlet {
  glm::u8vec3 skinned_triangles[Platform::Constants::meshlet_max_triangles_size] = {};  // up to 126 triangles
  uint32_t skinned_vertices_size = 0;
  uint32_t skinned_triangle_size = 0;
  uint32_t skinned_vertex_chunk_index = 0;
};

struct StrandPointDataChunk {
  StrandPoint strand_point_data[Platform::Constants::meshlet_max_vertices_size] = {};
};
struct StrandMeshlet {
  glm::u8vec4 segments[Platform::Constants::meshlet_max_triangles_size] = {};  // up to 126 triangles
  uint32_t strand_points_size = 0;
  uint32_t segment_size = 0;
  uint32_t strand_point_chunk_index = 0;
};
class RangeDescriptor {
  friend class GeometryStorage;
  Handle handle_;

 public:
  uint32_t offset;
  /**
   * \brief When used to record meshlet range. This records the newest number of meshlet for this geometry.
   * When used to record triangles, this records the newest number of triangles, including the size of the empty
   * fillers.
   */
  uint32_t range;

  uint32_t prev_frame_offset;
  uint32_t index_count;
  uint32_t prev_frame_index_count;
};

struct ParticleInfo {
  Transform instance_matrix = {};
  glm::vec4 instance_color = glm::vec4(1.0f);
};

enum class ParticleInfoListDataStatus { Updated, UpdatePending, Removed };
struct ParticleInfoListData {
  std::shared_ptr<Buffer> m_buffer;
  std::shared_ptr<DescriptorSet> descriptor_set;
  std::vector<ParticleInfo> particle_info_list;
  ParticleInfoListDataStatus m_status = ParticleInfoListDataStatus::Updated;
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

 public:
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

  static void AllocateMesh(const Handle& handle, const std::vector<Vertex>& vertices,
                           const std::vector<glm::uvec3>& triangles,
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
};
}  // namespace evo_engine
