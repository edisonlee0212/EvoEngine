#pragma once
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "Vertex.hpp"

#include <initializer_list>

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
  uint32_t offset = 0;

  /**
   * @brief Range value for the descriptor.
   *
   * - When used for meshlets: Represents the count of meshlets for this geometry.
   * - When used for triangles: Represents the count of triangles, including space for empty fillers.
   */
  uint32_t range = 0;

  /**
   * @brief Offset for the previous frame's data.
   */
  uint32_t prev_frame_offset = 0;

  /**
   * @brief Number of indices in the current frame.
   */
  uint32_t index_count = 0;

  /**
   * @brief Number of indices in the previous frame.
   */
  uint32_t prev_frame_index_count = 0;

  /**
   * @brief Committed range count that is safe for rendering.
   */
  uint32_t prev_frame_range = 0;
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
 public:
  static GeometryStorage& GetInstance();

 private:
  struct RangeCommit {
    std::shared_ptr<RangeDescriptor> descriptor;
    uint32_t offset = 0;
    uint32_t range = 0;
    uint32_t index_count = 0;
  };

  struct PendingGeometryUpload {
    bool active = false;
    bool scheduling = false;
    std::vector<GpuWorkHandle> handles;
    std::vector<RangeCommit> meshlet_commits;
    std::vector<RangeCommit> index_commits;
  };

  struct DirtyRange {
    bool dirty = false;
    size_t begin = 0;
    size_t end = 0;

    void Mark(size_t range_begin, size_t count);
    void MarkTail(size_t range_begin, size_t range_end);
    void Clear();
    [[nodiscard]] bool Empty() const;
  };

  struct DirtyBufferUpload {
    const std::shared_ptr<Buffer>* buffer = nullptr;
    const void* data = nullptr;
    size_t element_count = 0;
    size_t element_size = 0;
    DirtyRange* dirty_range = nullptr;
  };

  std::vector<VertexDataChunk> vertex_data_chunks_ = {};
  std::vector<Meshlet> meshlets_ = {};
  std::vector<std::shared_ptr<RangeDescriptor>> meshlet_range_descriptor_;
  std::vector<glm::uvec3> triangles_;
  std::vector<std::shared_ptr<RangeDescriptor>> triangle_range_descriptor_;

  std::shared_ptr<Buffer> vertex_buffer_ = {};
  std::shared_ptr<Buffer> meshlet_buffer_ = {};
  std::shared_ptr<Buffer> triangle_buffer_ = {};
  bool require_mesh_data_device_update_ = {};
  PendingGeometryUpload pending_mesh_upload_;
  DirtyRange mesh_vertex_dirty_range_;
  DirtyRange meshlet_dirty_range_;
  DirtyRange triangle_dirty_range_;

  std::vector<SkinnedVertexDataChunk> skinned_vertex_data_chunks_ = {};
  std::vector<SkinnedMeshlet> skinned_meshlets_ = {};
  std::vector<std::shared_ptr<RangeDescriptor>> skinned_meshlet_range_descriptor_;
  std::vector<glm::uvec3> skinned_triangles_;
  std::vector<std::shared_ptr<RangeDescriptor>> skinned_triangle_range_descriptor_;

  std::shared_ptr<Buffer> skinned_vertex_buffer_ = {};
  std::shared_ptr<Buffer> skinned_meshlet_buffer_ = {};
  std::shared_ptr<Buffer> skinned_triangle_buffer_ = {};
  bool require_skinned_mesh_data_device_update_ = {};
  PendingGeometryUpload pending_skinned_mesh_upload_;
  DirtyRange skinned_vertex_dirty_range_;
  DirtyRange skinned_meshlet_dirty_range_;
  DirtyRange skinned_triangle_dirty_range_;

  std::vector<StrandPointDataChunk> strand_point_data_chunks_ = {};
  std::vector<StrandMeshlet> strand_meshlets_ = {};
  std::vector<std::shared_ptr<RangeDescriptor>> strand_meshlet_range_descriptor_;
  std::vector<glm::uvec4> segments_;
  std::vector<std::shared_ptr<RangeDescriptor>> segment_range_descriptor_;

  std::shared_ptr<Buffer> strand_point_buffer_ = {};
  std::shared_ptr<Buffer> strand_meshlet_buffer_ = {};
  std::shared_ptr<Buffer> segment_buffer_ = {};
  bool require_strand_mesh_data_device_update_ = {};
  PendingGeometryUpload pending_strand_upload_;
  DirtyRange strand_point_dirty_range_;
  DirtyRange strand_meshlet_dirty_range_;
  DirtyRange segment_dirty_range_;

  void UploadData();
  void ClearMeshDirtyRanges();
  void ClearSkinnedMeshDirtyRanges();
  void ClearStrandDirtyRanges();
  static GpuWorkHandle ScheduleDirtyBufferUpload(const DirtyBufferUpload& upload);
  static void CaptureRangeCommits(const std::vector<std::shared_ptr<RangeDescriptor>>& descriptors,
                                  std::vector<RangeCommit>& commits);
  static void ApplyRangeCommits(const std::vector<RangeCommit>& commits);
  static bool HasValidUploadHandle(const PendingGeometryUpload& upload);
  static bool IsPendingUploadCompleted(const PendingGeometryUpload& upload);
  static void WaitPendingUpload(PendingGeometryUpload& upload);
  static void ClearPendingUpload(PendingGeometryUpload& upload);
  bool CompletePendingUpload(PendingGeometryUpload& upload);
  void CompletePendingUploads();
  void ScheduleUploadGroup(bool& dirty, PendingGeometryUpload& pending_upload,
                           std::initializer_list<DirtyBufferUpload> uploads,
                           const std::vector<std::shared_ptr<RangeDescriptor>>& meshlet_descriptors,
                           const std::vector<std::shared_ptr<RangeDescriptor>>& index_descriptors);
  void SchedulePendingUploads();
  friend class RenderLayer;
  friend class Resources;
  friend class Platform;
  static void DeviceSync();
  static void Initialize();

  std::vector<ParticleInfoListData> particle_info_list_data_list_;
  uint32_t version_ = 0;
  bool upload_data_in_progress_ = false;
  bool initialized_ = false;

 public:
  [[nodiscard]] static uint32_t GetVersion();
  [[nodiscard]] static bool HasPendingUploads();
  static void WaitForPendingUploads();
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
