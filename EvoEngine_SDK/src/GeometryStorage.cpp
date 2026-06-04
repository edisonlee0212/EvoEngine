#include "GeometryStorage.hpp"
#include "Application.hpp"
#include "Jobs.hpp"
#include "RenderLayer.hpp"
#include "meshoptimizer.h"
using namespace evo_engine;

namespace {
template <typename T>
GpuWorkHandle UploadVectorAsync(const std::shared_ptr<Buffer>& buffer, const std::vector<T>& data) {
  if (!buffer || data.empty()) {
    return {};
  }
  return buffer->UploadDataAsync(data.size() * sizeof(T), data.data());
}
}  // namespace

bool GeometryStorage::GeometryUploadSnapshot::Active() const {
  return mesh_dirty || mesh_upload_active || skinned_mesh_dirty || skinned_mesh_upload_active || strand_dirty ||
         strand_upload_active;
}

void GeometryStorage::CaptureRangeCommits(const std::vector<std::shared_ptr<RangeDescriptor>>& descriptors,
                                          std::vector<RangeCommit>& commits) {
  commits.clear();
  commits.reserve(descriptors.size());
  for (const auto& descriptor : descriptors) {
    if (!descriptor) {
      continue;
    }
    auto& commit = commits.emplace_back();
    commit.descriptor = descriptor;
    commit.offset = descriptor->offset;
    commit.range = descriptor->range;
    commit.index_count = descriptor->index_count;
  }
}

void GeometryStorage::ApplyRangeCommits(const std::vector<RangeCommit>& commits) {
  for (const auto& commit : commits) {
    if (!commit.descriptor) {
      continue;
    }
    commit.descriptor->prev_frame_offset = commit.offset;
    commit.descriptor->prev_frame_range = commit.range;
    commit.descriptor->prev_frame_index_count = commit.index_count;
  }
}

bool GeometryStorage::HasValidUploadHandle(const PendingGeometryUpload& upload) {
  for (const auto& handle : upload.handles) {
    if (handle.Valid()) {
      return true;
    }
  }
  return false;
}

bool GeometryStorage::IsPendingUploadCompleted(const PendingGeometryUpload& upload) {
  for (const auto& handle : upload.handles) {
    if (handle.Valid() && !Jobs::IsCompleted(handle)) {
      return false;
    }
  }
  return true;
}

void GeometryStorage::WaitPendingUpload(PendingGeometryUpload& upload) {
  auto* gpu_service = Platform::TryGetGpuService();
  for (const auto& handle : upload.handles) {
    if (!handle.Valid()) {
      continue;
    }
    if (gpu_service) {
      gpu_service->Wait(handle);
    } else {
      Jobs::Wait(handle);
    }
  }
}

void GeometryStorage::ClearPendingUpload(PendingGeometryUpload& upload) {
  upload.active = false;
  upload.handles.clear();
  upload.meshlet_commits.clear();
  upload.index_commits.clear();
}

bool GeometryStorage::CompletePendingUpload(PendingGeometryUpload& upload) {
  if (!upload.active || !IsPendingUploadCompleted(upload)) {
    return false;
  }
  WaitPendingUpload(upload);
  ApplyRangeCommits(upload.meshlet_commits);
  ApplyRangeCommits(upload.index_commits);
  ClearPendingUpload(upload);
  version_++;
  return true;
}

void GeometryStorage::CompletePendingUploads() {
  CompletePendingUpload(pending_mesh_upload_);
  CompletePendingUpload(pending_skinned_mesh_upload_);
  CompletePendingUpload(pending_strand_upload_);
}

void GeometryStorage::SchedulePendingUploads() {
  if (require_mesh_data_device_update_ && !pending_mesh_upload_.active) {
    pending_mesh_upload_.handles = {
        UploadVectorAsync(vertex_buffer_, vertex_data_chunks_),
        UploadVectorAsync(meshlet_buffer_, meshlets_),
        UploadVectorAsync(triangle_buffer_, triangles_),
    };
    CaptureRangeCommits(meshlet_range_descriptor_, pending_mesh_upload_.meshlet_commits);
    CaptureRangeCommits(triangle_range_descriptor_, pending_mesh_upload_.index_commits);
    require_mesh_data_device_update_ = false;
    if (HasValidUploadHandle(pending_mesh_upload_)) {
      pending_mesh_upload_.active = true;
    } else {
      ApplyRangeCommits(pending_mesh_upload_.meshlet_commits);
      ApplyRangeCommits(pending_mesh_upload_.index_commits);
      ClearPendingUpload(pending_mesh_upload_);
      version_++;
    }
  }

  if (require_skinned_mesh_data_device_update_ && !pending_skinned_mesh_upload_.active) {
    pending_skinned_mesh_upload_.handles = {
        UploadVectorAsync(skinned_vertex_buffer_, skinned_vertex_data_chunks_),
        UploadVectorAsync(skinned_meshlet_buffer_, skinned_meshlets_),
        UploadVectorAsync(skinned_triangle_buffer_, skinned_triangles_),
    };
    CaptureRangeCommits(skinned_meshlet_range_descriptor_, pending_skinned_mesh_upload_.meshlet_commits);
    CaptureRangeCommits(skinned_triangle_range_descriptor_, pending_skinned_mesh_upload_.index_commits);
    require_skinned_mesh_data_device_update_ = false;
    if (HasValidUploadHandle(pending_skinned_mesh_upload_)) {
      pending_skinned_mesh_upload_.active = true;
    } else {
      ApplyRangeCommits(pending_skinned_mesh_upload_.meshlet_commits);
      ApplyRangeCommits(pending_skinned_mesh_upload_.index_commits);
      ClearPendingUpload(pending_skinned_mesh_upload_);
      version_++;
    }
  }

  if (require_strand_mesh_data_device_update_ && !pending_strand_upload_.active) {
    pending_strand_upload_.handles = {
        UploadVectorAsync(strand_point_buffer_, strand_point_data_chunks_),
        UploadVectorAsync(strand_meshlet_buffer_, strand_meshlets_),
        UploadVectorAsync(segment_buffer_, segments_),
    };
    CaptureRangeCommits(strand_meshlet_range_descriptor_, pending_strand_upload_.meshlet_commits);
    CaptureRangeCommits(segment_range_descriptor_, pending_strand_upload_.index_commits);
    require_strand_mesh_data_device_update_ = false;
    if (HasValidUploadHandle(pending_strand_upload_)) {
      pending_strand_upload_.active = true;
    } else {
      ApplyRangeCommits(pending_strand_upload_.meshlet_commits);
      ApplyRangeCommits(pending_strand_upload_.index_commits);
      ClearPendingUpload(pending_strand_upload_);
      version_++;
    }
  }
}

void GeometryStorage::UploadData() {
  CompletePendingUploads();
  SchedulePendingUploads();
  CompletePendingUploads();

  for (int index = 0; index < particle_info_list_data_list_.size(); index++) {
    if (auto& particle_info_list_data = particle_info_list_data_list_.at(index);
        particle_info_list_data.status == ParticleInfoListDataStatus::Removed) {
      particle_info_list_data_list_.at(index) = particle_info_list_data_list_.back();
      particle_info_list_data_list_.pop_back();
      index--;
    } else if (particle_info_list_data.status == ParticleInfoListDataStatus::UpdatePending) {
      particle_info_list_data.buffer->UploadVector(particle_info_list_data.particle_info_list);

      version_++;

      VkDescriptorBufferInfo buffer_info{};
      buffer_info.offset = 0;
      buffer_info.range = VK_WHOLE_SIZE;
      buffer_info.buffer = particle_info_list_data.buffer->GetVkBuffer();
      particle_info_list_data.descriptor_set->UpdateBufferDescriptorBinding(0, buffer_info, 0);

      particle_info_list_data.status = ParticleInfoListDataStatus::Updated;
    }
  }
  for (int index = 0; index < particle_info_list_data_list_.size(); index++) {
    const auto& particle_info_list_data = particle_info_list_data_list_.at(index);
    particle_info_list_data.range_descriptor->offset = index;
  }
}

void GeometryStorage::DeviceSync() {
  if (!Platform::Initialized())
    return;
  auto& storage = GetInstance();
  storage.UploadData();
}

void GeometryStorage::Initialize() {
  auto& storage = GetInstance();
  VkBufferCreateInfo storage_buffer_create_info{};
  storage_buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  storage_buffer_create_info.size = 1;

  storage_buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo vertices_vma_allocation_create_info{};
  vertices_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  storage_buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
  storage.vertex_buffer_ = std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);
  storage_buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  storage.meshlet_buffer_ = std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);

  storage_buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
  storage.triangle_buffer_ = std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);

  storage.require_mesh_data_device_update_ = false;

  storage_buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
  storage.skinned_vertex_buffer_ =
      std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);
  storage_buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  storage.skinned_meshlet_buffer_ =
      std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);

  storage_buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
  storage.skinned_triangle_buffer_ =
      std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);

  storage.require_skinned_mesh_data_device_update_ = false;

  storage_buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
  storage.strand_point_buffer_ =
      std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);
  storage_buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  storage.strand_meshlet_buffer_ =
      std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);

  storage_buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
  storage.segment_buffer_ = std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);

  storage.require_strand_mesh_data_device_update_ = false;
  storage.initialized_ = true;
}

uint32_t GeometryStorage::GetVersion() {
  return GetInstance().version_;
}

GeometryStorage::GeometryUploadSnapshot GeometryStorage::GetUploadSnapshot() {
  const auto& storage = GetInstance();
  GeometryUploadSnapshot snapshot;
  snapshot.mesh_dirty = storage.require_mesh_data_device_update_;
  snapshot.mesh_upload_active = storage.pending_mesh_upload_.active;
  snapshot.mesh_upload_completed = IsPendingUploadCompleted(storage.pending_mesh_upload_);
  snapshot.mesh_upload_handles = storage.pending_mesh_upload_.handles.size();

  snapshot.skinned_mesh_dirty = storage.require_skinned_mesh_data_device_update_;
  snapshot.skinned_mesh_upload_active = storage.pending_skinned_mesh_upload_.active;
  snapshot.skinned_mesh_upload_completed = IsPendingUploadCompleted(storage.pending_skinned_mesh_upload_);
  snapshot.skinned_mesh_upload_handles = storage.pending_skinned_mesh_upload_.handles.size();

  snapshot.strand_dirty = storage.require_strand_mesh_data_device_update_;
  snapshot.strand_upload_active = storage.pending_strand_upload_.active;
  snapshot.strand_upload_completed = IsPendingUploadCompleted(storage.pending_strand_upload_);
  snapshot.strand_upload_handles = storage.pending_strand_upload_.handles.size();
  return snapshot;
}

bool GeometryStorage::HasPendingUploads() {
  return GetUploadSnapshot().Active();
}

void GeometryStorage::WaitForPendingUploads() {
  if (!Platform::Initialized()) {
    return;
  }
  auto& storage = GetInstance();
  storage.CompletePendingUploads();
  storage.SchedulePendingUploads();
  WaitPendingUpload(storage.pending_mesh_upload_);
  WaitPendingUpload(storage.pending_skinned_mesh_upload_);
  WaitPendingUpload(storage.pending_strand_upload_);
  storage.CompletePendingUploads();
}

const std::shared_ptr<Buffer>& GeometryStorage::GetTriangleBuffer() {
  const auto& storage = GetInstance();
  return storage.triangle_buffer_;
}

const std::shared_ptr<Buffer>& GeometryStorage::GetVertexBuffer() {
  const auto& storage = GetInstance();
  return storage.vertex_buffer_;
}

const std::shared_ptr<Buffer>& GeometryStorage::GetMeshletBuffer() {
  const auto& storage = GetInstance();
  return storage.meshlet_buffer_;
}

const std::shared_ptr<Buffer>& GeometryStorage::GetSkinnedVertexBuffer() {
  const auto& storage = GetInstance();
  return storage.skinned_vertex_buffer_;
}

const std::shared_ptr<Buffer>& GeometryStorage::GetSkinnedMeshletBuffer() {
  const auto& storage = GetInstance();
  return storage.skinned_meshlet_buffer_;
}

const std::shared_ptr<Buffer>& GeometryStorage::GetStrandPointBuffer() {
  const auto& storage = GetInstance();
  return storage.strand_point_buffer_;
}

const std::shared_ptr<Buffer>& GeometryStorage::GetStrandMeshletBuffer() {
  const auto& storage = GetInstance();
  return storage.strand_meshlet_buffer_;
}

void GeometryStorage::BindVertices(const VkCommandBuffer vk_command_buffer) {
  const auto& storage = GetInstance();
  storage.vertex_buffer_->BindVertex(vk_command_buffer);
  storage.triangle_buffer_->BindIndex(vk_command_buffer);
}

void GeometryStorage::BindSkinnedVertices(const VkCommandBuffer vk_command_buffer) {
  const auto& storage = GetInstance();
  storage.skinned_vertex_buffer_->BindVertex(vk_command_buffer);
  storage.skinned_triangle_buffer_->BindIndex(vk_command_buffer);
}

void GeometryStorage::BindStrandPoints(const VkCommandBuffer vk_command_buffer) {
  const auto& storage = GetInstance();
  storage.strand_point_buffer_->BindVertex(vk_command_buffer);
  storage.segment_buffer_->BindIndex(vk_command_buffer);
}

const Vertex& GeometryStorage::PeekVertex(const size_t vertex_index) {
  const auto& storage = GetInstance();
  return storage.vertex_data_chunks_[vertex_index / Platform::Constants::meshlet_max_vertices_size]
      .vertex_data[vertex_index % Platform::Constants::meshlet_max_vertices_size];
}

const SkinnedVertex& GeometryStorage::PeekSkinnedVertex(const size_t skinned_vertex_index) {
  const auto& storage = GetInstance();
  return storage.skinned_vertex_data_chunks_[skinned_vertex_index / Platform::Constants::meshlet_max_vertices_size]
      .skinned_vertex_data[skinned_vertex_index % Platform::Constants::meshlet_max_vertices_size];
}

const StrandPoint& GeometryStorage::PeekStrandPoint(const size_t strand_point_index) {
  const auto& storage = GetInstance();
  return storage.strand_point_data_chunks_[strand_point_index / Platform::Constants::meshlet_max_vertices_size]
      .strand_point_data[strand_point_index % Platform::Constants::meshlet_max_vertices_size];
}

void GeometryStorage::AllocateMesh(const Handle& handle, std::vector<Vertex>& vertices,
                                   std::vector<glm::uvec3>& triangles,
                                   const std::shared_ptr<RangeDescriptor>& target_meshlet_range,
                                   const std::shared_ptr<RangeDescriptor>& target_triangle_range) {
  if (vertices.empty() || triangles.empty()) {
    throw std::runtime_error("Empty vertices or triangles!");
  }
  auto& storage = GetInstance();
  WaitPendingUpload(storage.pending_mesh_upload_);
  storage.CompletePendingUpload(storage.pending_mesh_upload_);

  // const auto meshletRange = std::make_shared<RangeDescriptor>();
  target_meshlet_range->handle_ = handle;
  target_meshlet_range->offset = storage.meshlets_.size();
  target_meshlet_range->range = 0;

  // const auto triangleRange = std::make_shared<RangeDescriptor>();
  target_triangle_range->handle_ = handle;
  target_triangle_range->offset = storage.triangles_.size();
  target_triangle_range->range = 0;
  target_triangle_range->index_count = triangles.size();

  std::vector<meshopt_Meshlet> meshlets_results;
  std::vector<uint32_t> meshlet_result_vertices;
  std::vector<uint8_t> meshlet_result_triangles;
  const auto max_meshlets =
      meshopt_buildMeshletsBound(triangles.size() * 3, Platform::Constants::meshlet_max_vertices_size,
                                 Platform::Constants::meshlet_max_triangles_size);
  meshlets_results.resize(max_meshlets);
  meshlet_result_vertices.resize(max_meshlets * Platform::Constants::meshlet_max_vertices_size);
  meshlet_result_triangles.resize(max_meshlets * Platform::Constants::meshlet_max_triangles_size * 3);
  const auto meshlet_size = meshopt_buildMeshlets(
      meshlets_results.data(), meshlet_result_vertices.data(), meshlet_result_triangles.data(), &triangles.at(0).x,
      triangles.size() * 3, &vertices.at(0).position.x, vertices.size(), sizeof(Vertex),
      Platform::Constants::meshlet_max_vertices_size, Platform::Constants::meshlet_max_triangles_size, 0);
  std::vector<Vertex> replacement_vertices;
  std::vector<glm::uvec3> replacement_triangles;

  target_meshlet_range->range = meshlet_size;
  for (size_t meshlet_index = 0; meshlet_index < meshlet_size; meshlet_index++) {
    const uint32_t current_meshlet_index = storage.meshlets_.size();
    storage.meshlets_.emplace_back();
    auto& current_meshlet = storage.meshlets_[current_meshlet_index];

    current_meshlet.vertex_chunk_index = storage.vertex_data_chunks_.size();
    storage.vertex_data_chunks_.emplace_back();
    auto& current_chunk = storage.vertex_data_chunks_[current_meshlet.vertex_chunk_index];

    const auto& meshlet_result = meshlets_results.at(meshlet_index);

    const uint32_t replacement_vertex_offset = replacement_vertices.size();

    for (uint32_t vi = 0; vi < meshlet_result.vertex_count; vi++) {
      current_chunk.vertex_data[vi] = vertices[meshlet_result_vertices.at(meshlet_result.vertex_offset + vi)];
      replacement_vertices.emplace_back(current_chunk.vertex_data[vi]);
    }
    current_meshlet.vertices_size = meshlet_result.vertex_count;
    current_meshlet.triangle_size = meshlet_result.triangle_count;
    for (uint32_t ti = 0; ti < meshlet_result.triangle_count; ti++) {
      auto& current_meshlet_triangle = current_meshlet.triangles[ti];
      current_meshlet_triangle = glm::u8vec3(meshlet_result_triangles[ti * 3 + meshlet_result.triangle_offset],
                                             meshlet_result_triangles[ti * 3 + meshlet_result.triangle_offset + 1],
                                             meshlet_result_triangles[ti * 3 + meshlet_result.triangle_offset + 2]);

      auto& global_triangle = storage.triangles_.emplace_back();
      global_triangle.x = current_meshlet_triangle.x +
                          current_meshlet.vertex_chunk_index * Platform::Constants::meshlet_max_vertices_size;
      global_triangle.y = current_meshlet_triangle.y +
                          current_meshlet.vertex_chunk_index * Platform::Constants::meshlet_max_vertices_size;
      global_triangle.z = current_meshlet_triangle.z +
                          current_meshlet.vertex_chunk_index * Platform::Constants::meshlet_max_vertices_size;

      replacement_triangles.emplace_back(current_meshlet_triangle.x + replacement_vertex_offset,
                                         current_meshlet_triangle.y + replacement_vertex_offset,
                                         current_meshlet_triangle.z + replacement_vertex_offset);
    }
    target_triangle_range->range += current_meshlet.triangle_size;
  }
  vertices = replacement_vertices;
  triangles = replacement_triangles;

  storage.meshlet_range_descriptor_.push_back(target_meshlet_range);
  storage.triangle_range_descriptor_.push_back(target_triangle_range);
  storage.require_mesh_data_device_update_ = true;
}

void GeometryStorage::AllocateSkinnedMesh(const Handle& handle, const std::vector<SkinnedVertex>& skinned_vertices,
                                          const std::vector<glm::uvec3>& skinned_triangles,
                                          const std::shared_ptr<RangeDescriptor>& target_skinned_meshlet_range,
                                          const std::shared_ptr<RangeDescriptor>& target_skinned_triangle_range) {
  if (skinned_vertices.empty() || skinned_triangles.empty()) {
    throw std::runtime_error("Empty skinned vertices or skinned_triangles!");
  }
  auto& storage = GetInstance();
  WaitPendingUpload(storage.pending_skinned_mesh_upload_);
  storage.CompletePendingUpload(storage.pending_skinned_mesh_upload_);

  target_skinned_meshlet_range->handle_ = handle;
  target_skinned_meshlet_range->offset = storage.skinned_meshlets_.size();
  target_skinned_meshlet_range->range = 0;

  target_skinned_triangle_range->handle_ = handle;
  target_skinned_triangle_range->offset = storage.skinned_triangles_.size();
  target_skinned_triangle_range->range = 0;
  target_skinned_triangle_range->index_count = skinned_triangles.size();
  std::vector<meshopt_Meshlet> skinned_meshlets_results{};
  std::vector<uint32_t> skinned_meshlet_result_vertices{};
  std::vector<uint8_t> skinned_meshlet_result_triangles{};
  const auto max_meshlets =
      meshopt_buildMeshletsBound(skinned_triangles.size() * 3, Platform::Constants::meshlet_max_vertices_size,
                                 Platform::Constants::meshlet_max_triangles_size);
  skinned_meshlets_results.resize(max_meshlets);
  skinned_meshlet_result_vertices.resize(max_meshlets * Platform::Constants::meshlet_max_vertices_size);
  skinned_meshlet_result_triangles.resize(max_meshlets * Platform::Constants::meshlet_max_triangles_size * 3);
  const auto skinned_meshlet_size = meshopt_buildMeshlets(
      skinned_meshlets_results.data(), skinned_meshlet_result_vertices.data(), skinned_meshlet_result_triangles.data(),
      &skinned_triangles.at(0).x, skinned_triangles.size() * 3, &skinned_vertices.at(0).position.x,
      skinned_vertices.size(), sizeof(SkinnedVertex), Platform::Constants::meshlet_max_vertices_size,
      Platform::Constants::meshlet_max_triangles_size, 0);

  target_skinned_meshlet_range->range = skinned_meshlet_size;
  for (size_t skinned_meshlet_index = 0; skinned_meshlet_index < skinned_meshlet_size; skinned_meshlet_index++) {
    storage.skinned_meshlets_.emplace_back();
    auto& current_skinned_meshlet = storage.skinned_meshlets_.back();

    current_skinned_meshlet.skinned_vertex_chunk_index = storage.skinned_vertex_data_chunks_.size();
    storage.skinned_vertex_data_chunks_.emplace_back();
    auto& current_skinned_chunk = storage.skinned_vertex_data_chunks_.back();

    const auto& skinned_meshlet_result = skinned_meshlets_results.at(skinned_meshlet_index);
    for (uint32_t vi = 0; vi < skinned_meshlet_result.vertex_count; vi++) {
      current_skinned_chunk.skinned_vertex_data[vi] =
          skinned_vertices[skinned_meshlet_result_vertices.at(skinned_meshlet_result.vertex_offset + vi)];
    }
    current_skinned_meshlet.skinned_vertices_size = skinned_meshlet_result.vertex_count;
    current_skinned_meshlet.skinned_triangle_size = skinned_meshlet_result.triangle_count;
    for (uint32_t ti = 0; ti < skinned_meshlet_result.triangle_count; ti++) {
      auto& current_meshlet_triangle = current_skinned_meshlet.skinned_triangles[ti];
      current_meshlet_triangle =
          glm::u8vec3(skinned_meshlet_result_triangles[ti * 3 + skinned_meshlet_result.triangle_offset],
                      skinned_meshlet_result_triangles[ti * 3 + skinned_meshlet_result.triangle_offset + 1],
                      skinned_meshlet_result_triangles[ti * 3 + skinned_meshlet_result.triangle_offset + 2]);

      storage.skinned_triangles_.emplace_back();
      auto& global_triangle = storage.skinned_triangles_.back();
      global_triangle.x = current_meshlet_triangle.x + current_skinned_meshlet.skinned_vertex_chunk_index *
                                                           Platform::Constants::meshlet_max_vertices_size;
      global_triangle.y = current_meshlet_triangle.y + current_skinned_meshlet.skinned_vertex_chunk_index *
                                                           Platform::Constants::meshlet_max_vertices_size;
      global_triangle.z = current_meshlet_triangle.z + current_skinned_meshlet.skinned_vertex_chunk_index *
                                                           Platform::Constants::meshlet_max_vertices_size;
    }
    target_skinned_triangle_range->range += current_skinned_meshlet.skinned_triangle_size;
  }
  storage.skinned_meshlet_range_descriptor_.push_back(target_skinned_meshlet_range);
  storage.skinned_triangle_range_descriptor_.push_back(target_skinned_triangle_range);
  storage.require_skinned_mesh_data_device_update_ = true;
}

void GeometryStorage::AllocateStrands(const Handle& handle, const std::vector<StrandPoint>& strand_points,
                                      const std::vector<glm::uvec4>& segments,
                                      const std::shared_ptr<RangeDescriptor>& target_strand_meshlet_range,
                                      const std::shared_ptr<RangeDescriptor>& target_segment_range) {
  if (strand_points.empty() || segments.empty()) {
    throw std::runtime_error("Empty strand points or strand segments!");
  }
  auto& storage = GetInstance();
  WaitPendingUpload(storage.pending_strand_upload_);
  storage.CompletePendingUpload(storage.pending_strand_upload_);

  uint32_t current_segment_index = 0;
  target_strand_meshlet_range->handle_ = handle;
  target_strand_meshlet_range->offset = storage.strand_meshlets_.size();
  target_strand_meshlet_range->range = 0;

  target_segment_range->handle_ = handle;
  target_segment_range->offset = storage.segments_.size();
  target_segment_range->range = 0;
  target_segment_range->index_count = segments.size();

  while (current_segment_index < segments.size()) {
    target_strand_meshlet_range->range++;
    const uint32_t current_strand_meshlet_index = storage.strand_meshlets_.size();
    storage.strand_meshlets_.emplace_back();
    auto& current_strand_meshlet = storage.strand_meshlets_[current_strand_meshlet_index];

    current_strand_meshlet.strand_point_chunk_index = storage.strand_point_data_chunks_.size();
    storage.strand_point_data_chunks_.emplace_back();
    auto& current_chunk = storage.strand_point_data_chunks_[current_strand_meshlet.strand_point_chunk_index];

    current_strand_meshlet.strand_points_size = current_strand_meshlet.segment_size = 0;

    std::unordered_map<uint32_t, uint32_t> assigned_strand_points{};
    while (current_strand_meshlet.segment_size < Platform::Constants::meshlet_max_triangles_size &&
           current_segment_index < segments.size()) {
      const auto& current_segment = segments[current_segment_index];
      uint32_t new_strand_points_amount = 0;
      auto search_x = assigned_strand_points.find(current_segment.x);
      if (search_x == assigned_strand_points.end())
        new_strand_points_amount++;

      auto search_y = assigned_strand_points.find(current_segment.y);
      if (search_y == assigned_strand_points.end())
        new_strand_points_amount++;

      auto search_z = assigned_strand_points.find(current_segment.z);
      if (search_z == assigned_strand_points.end())
        new_strand_points_amount++;

      auto search_w = assigned_strand_points.find(current_segment.w);
      if (search_w == assigned_strand_points.end())
        new_strand_points_amount++;

      if (current_strand_meshlet.strand_points_size + new_strand_points_amount >
          Platform::Constants::meshlet_max_vertices_size) {
        break;
      }
      auto& current_strand_meshlet_segment = current_strand_meshlet.segments[current_strand_meshlet.segment_size];

      if (search_x != assigned_strand_points.end()) {
        current_strand_meshlet_segment.x = search_x->second;
      } else {
        // Add current strandPoint index into the map.
        assigned_strand_points[current_segment.x] = current_strand_meshlet.strand_points_size;

        // Assign new strandPoint in strandMeshlet, and retrieve actual strandPoint index in strandPoint data chunks.
        current_chunk.strand_point_data[current_strand_meshlet.strand_points_size] = strand_points[current_segment.x];
        current_strand_meshlet_segment.x = current_strand_meshlet.strand_points_size;
        current_strand_meshlet.strand_points_size++;
      }

      search_y = assigned_strand_points.find(current_segment.y);
      if (search_y != assigned_strand_points.end()) {
        current_strand_meshlet_segment.y = search_y->second;
      } else {
        // Add current strandPoint index into the map.
        assigned_strand_points[current_segment.y] = current_strand_meshlet.strand_points_size;

        // Assign new strandPoint in strandMeshlet, and retrieve actual strandPoint index in strandPoint data chunks.
        current_chunk.strand_point_data[current_strand_meshlet.strand_points_size] = strand_points[current_segment.y];
        current_strand_meshlet_segment.y = current_strand_meshlet.strand_points_size;
        current_strand_meshlet.strand_points_size++;
      }

      search_z = assigned_strand_points.find(current_segment.z);
      if (search_z != assigned_strand_points.end()) {
        current_strand_meshlet_segment.z = search_z->second;
      } else {
        // Add current strandPoint index into the map.
        assigned_strand_points[current_segment.z] = current_strand_meshlet.strand_points_size;

        // Assign new strandPoint in strandMeshlet, and retrieve actual strandPoint index in strandPoint data chunks.
        current_chunk.strand_point_data[current_strand_meshlet.strand_points_size] = strand_points[current_segment.z];
        current_strand_meshlet_segment.z = current_strand_meshlet.strand_points_size;
        current_strand_meshlet.strand_points_size++;
      }

      search_w = assigned_strand_points.find(current_segment.w);
      if (search_w != assigned_strand_points.end()) {
        current_strand_meshlet_segment.w = search_w->second;
      } else {
        // Add current strandPoint index into the map.
        assigned_strand_points[current_segment.w] = current_strand_meshlet.strand_points_size;

        // Assign new strandPoint in strandMeshlet, and retrieve actual strandPoint index in strandPoint data chunks.
        current_chunk.strand_point_data[current_strand_meshlet.strand_points_size] = strand_points[current_segment.w];
        current_strand_meshlet_segment.w = current_strand_meshlet.strand_points_size;
        current_strand_meshlet.strand_points_size++;
      }
      current_strand_meshlet.segment_size++;
      current_segment_index++;

      auto& global_segment = storage.segments_.emplace_back();
      global_segment.x = current_strand_meshlet_segment.x + current_strand_meshlet.strand_point_chunk_index *
                                                                Platform::Constants::meshlet_max_vertices_size;
      global_segment.y = current_strand_meshlet_segment.y + current_strand_meshlet.strand_point_chunk_index *
                                                                Platform::Constants::meshlet_max_vertices_size;
      global_segment.z = current_strand_meshlet_segment.z + current_strand_meshlet.strand_point_chunk_index *
                                                                Platform::Constants::meshlet_max_vertices_size;
      global_segment.w = current_strand_meshlet_segment.w + current_strand_meshlet.strand_point_chunk_index *
                                                                Platform::Constants::meshlet_max_vertices_size;
      target_segment_range->range++;
    }
  }

  storage.strand_meshlet_range_descriptor_.push_back(target_strand_meshlet_range);
  storage.segment_range_descriptor_.push_back(target_segment_range);
  storage.require_strand_mesh_data_device_update_ = true;
}

void GeometryStorage::FreeMesh(const Handle& handle) {
  auto& storage = GetInstance();
  if (!storage.initialized_) {
    return;
  }
  WaitPendingUpload(storage.pending_mesh_upload_);
  storage.CompletePendingUpload(storage.pending_mesh_upload_);
  uint32_t meshlet_range_descriptor_index = UINT_MAX;
  for (int i = 0; i < storage.meshlet_range_descriptor_.size(); i++) {
    if (storage.meshlet_range_descriptor_[i]->handle_ == handle) {
      meshlet_range_descriptor_index = i;
      break;
    }
  }
  if (meshlet_range_descriptor_index == UINT_MAX) {
    return;
  }
  const auto& meshlet_range_descriptor = storage.meshlet_range_descriptor_[meshlet_range_descriptor_index];
  const uint32_t remove_chunk_size = meshlet_range_descriptor->range;
  storage.meshlets_.erase(storage.meshlets_.begin() + meshlet_range_descriptor->offset,
                          storage.meshlets_.begin() + meshlet_range_descriptor->offset + remove_chunk_size);
  storage.vertex_data_chunks_.erase(
      storage.vertex_data_chunks_.begin() + meshlet_range_descriptor->offset,
      storage.vertex_data_chunks_.begin() + meshlet_range_descriptor->offset + remove_chunk_size);
  for (uint32_t i = meshlet_range_descriptor_index; i < storage.meshlets_.size(); i++) {
    storage.meshlets_[i].vertex_chunk_index = i;
  }
  for (uint32_t i = meshlet_range_descriptor_index + 1; i < storage.meshlet_range_descriptor_.size(); i++) {
    assert(storage.meshlet_range_descriptor_[i]->offset >= meshlet_range_descriptor->range);
    storage.meshlet_range_descriptor_[i]->offset -= meshlet_range_descriptor->range;
  }
  storage.meshlet_range_descriptor_.erase(storage.meshlet_range_descriptor_.begin() + meshlet_range_descriptor_index);

  uint32_t triangle_range_descriptor_index = UINT_MAX;
  for (uint32_t i = 0; i < storage.triangle_range_descriptor_.size(); i++) {
    if (storage.triangle_range_descriptor_[i]->handle_ == handle) {
      triangle_range_descriptor_index = i;
      break;
    }
  }
  if (triangle_range_descriptor_index == UINT_MAX) {
    return;
  }
  const auto& triangle_range_descriptor = storage.triangle_range_descriptor_[triangle_range_descriptor_index];
  storage.triangles_.erase(
      storage.triangles_.begin() + triangle_range_descriptor->offset,
      storage.triangles_.begin() + triangle_range_descriptor->offset + triangle_range_descriptor->range);
  for (uint32_t i = triangle_range_descriptor_index + 1; i < storage.triangle_range_descriptor_.size(); i++) {
    assert(storage.triangle_range_descriptor_[i]->offset >= triangle_range_descriptor->range);
    storage.triangle_range_descriptor_[i]->offset -= triangle_range_descriptor->range;
  }

  for (uint32_t i = triangle_range_descriptor->offset; i < storage.triangles_.size(); i++) {
    storage.triangles_[i].x -= remove_chunk_size * Platform::Constants::meshlet_max_vertices_size;
    storage.triangles_[i].y -= remove_chunk_size * Platform::Constants::meshlet_max_vertices_size;
    storage.triangles_[i].z -= remove_chunk_size * Platform::Constants::meshlet_max_vertices_size;
  }
  storage.triangle_range_descriptor_.erase(storage.triangle_range_descriptor_.begin() +
                                           triangle_range_descriptor_index);
  storage.require_mesh_data_device_update_ = true;
}

void GeometryStorage::FreeSkinnedMesh(const Handle& handle) {
  auto& storage = GetInstance();
  if (!storage.initialized_) {
    return;
  }
  WaitPendingUpload(storage.pending_skinned_mesh_upload_);
  storage.CompletePendingUpload(storage.pending_skinned_mesh_upload_);
  uint32_t skinned_meshlet_range_descriptor_index = UINT_MAX;
  for (int i = 0; i < storage.skinned_meshlet_range_descriptor_.size(); i++) {
    if (storage.skinned_meshlet_range_descriptor_[i]->handle_ == handle) {
      skinned_meshlet_range_descriptor_index = i;
      break;
    }
  }
  if (skinned_meshlet_range_descriptor_index == UINT_MAX) {
    return;
  };
  const auto& skinned_meshlet_range_descriptor =
      storage.skinned_meshlet_range_descriptor_[skinned_meshlet_range_descriptor_index];
  const uint32_t remove_chunk_size = skinned_meshlet_range_descriptor->range;
  storage.skinned_meshlets_.erase(
      storage.skinned_meshlets_.begin() + skinned_meshlet_range_descriptor->offset,
      storage.skinned_meshlets_.begin() + skinned_meshlet_range_descriptor->offset + remove_chunk_size);
  storage.skinned_vertex_data_chunks_.erase(
      storage.skinned_vertex_data_chunks_.begin() + skinned_meshlet_range_descriptor->offset,
      storage.skinned_vertex_data_chunks_.begin() + skinned_meshlet_range_descriptor->offset + remove_chunk_size);
  for (uint32_t i = skinned_meshlet_range_descriptor_index; i < storage.skinned_meshlets_.size(); i++) {
    storage.skinned_meshlets_[i].skinned_vertex_chunk_index = i;
  }
  for (uint32_t i = skinned_meshlet_range_descriptor_index + 1; i < storage.skinned_meshlet_range_descriptor_.size();
       i++) {
    assert(storage.skinned_meshlet_range_descriptor_[i]->offset >= skinned_meshlet_range_descriptor->range);
    storage.skinned_meshlet_range_descriptor_[i]->offset -= skinned_meshlet_range_descriptor->range;
  }
  storage.skinned_meshlet_range_descriptor_.erase(storage.skinned_meshlet_range_descriptor_.begin() +
                                                  skinned_meshlet_range_descriptor_index);

  uint32_t skinned_triangle_range_descriptor_index = UINT_MAX;
  for (uint32_t i = 0; i < storage.skinned_triangle_range_descriptor_.size(); i++) {
    if (storage.skinned_triangle_range_descriptor_[i]->handle_ == handle) {
      skinned_triangle_range_descriptor_index = i;
      break;
    }
  }
  if (skinned_triangle_range_descriptor_index == UINT_MAX) {
    return;
  }
  const auto& skinned_triangle_range_descriptor =
      storage.skinned_triangle_range_descriptor_[skinned_triangle_range_descriptor_index];
  storage.skinned_triangles_.erase(storage.skinned_triangles_.begin() + skinned_triangle_range_descriptor->offset,
                                   storage.skinned_triangles_.begin() + skinned_triangle_range_descriptor->offset +
                                       skinned_triangle_range_descriptor->range);
  for (uint32_t i = skinned_triangle_range_descriptor_index + 1; i < storage.skinned_triangle_range_descriptor_.size();
       i++) {
    assert(storage.skinned_triangle_range_descriptor_[i]->offset >= skinned_triangle_range_descriptor->range);
    storage.skinned_triangle_range_descriptor_[i]->offset -= skinned_triangle_range_descriptor->range;
  }

  for (uint32_t i = skinned_triangle_range_descriptor->offset; i < storage.skinned_triangles_.size(); i++) {
    storage.skinned_triangles_[i].x -= remove_chunk_size * Platform::Constants::meshlet_max_vertices_size;
    storage.skinned_triangles_[i].y -= remove_chunk_size * Platform::Constants::meshlet_max_vertices_size;
    storage.skinned_triangles_[i].z -= remove_chunk_size * Platform::Constants::meshlet_max_vertices_size;
  }

  storage.skinned_triangle_range_descriptor_.erase(storage.skinned_triangle_range_descriptor_.begin() +
                                                   skinned_triangle_range_descriptor_index);
  storage.require_skinned_mesh_data_device_update_ = true;
}

void GeometryStorage::FreeStrands(const Handle& handle) {
  auto& storage = GetInstance();
  if (!storage.initialized_) {
    return;
  }
  WaitPendingUpload(storage.pending_strand_upload_);
  storage.CompletePendingUpload(storage.pending_strand_upload_);
  uint32_t strand_meshlet_range_descriptor_index = UINT_MAX;
  for (int i = 0; i < storage.strand_meshlet_range_descriptor_.size(); i++) {
    if (storage.strand_meshlet_range_descriptor_[i]->handle_ == handle) {
      strand_meshlet_range_descriptor_index = i;
      break;
    }
  }
  if (strand_meshlet_range_descriptor_index == UINT_MAX) {
    return;
  }
  const auto& strand_meshlet_range_descriptor =
      storage.strand_meshlet_range_descriptor_[strand_meshlet_range_descriptor_index];
  const uint32_t remove_chunk_size = strand_meshlet_range_descriptor->range;
  storage.strand_meshlets_.erase(
      storage.strand_meshlets_.begin() + strand_meshlet_range_descriptor->offset,
      storage.strand_meshlets_.begin() + strand_meshlet_range_descriptor->offset + remove_chunk_size);
  storage.strand_point_data_chunks_.erase(
      storage.strand_point_data_chunks_.begin() + strand_meshlet_range_descriptor->offset,
      storage.strand_point_data_chunks_.begin() + strand_meshlet_range_descriptor->offset + remove_chunk_size);
  for (uint32_t i = strand_meshlet_range_descriptor_index; i < storage.strand_meshlets_.size(); i++) {
    storage.strand_meshlets_[i].strand_point_chunk_index = i;
  }
  for (uint32_t i = strand_meshlet_range_descriptor_index + 1; i < storage.strand_meshlet_range_descriptor_.size();
       i++) {
    assert(storage.strand_meshlet_range_descriptor_[i]->offset >= strand_meshlet_range_descriptor->range);
    storage.strand_meshlet_range_descriptor_[i]->offset -= strand_meshlet_range_descriptor->range;
  }
  storage.strand_meshlet_range_descriptor_.erase(storage.strand_meshlet_range_descriptor_.begin() +
                                                 strand_meshlet_range_descriptor_index);

  uint32_t segment_range_descriptor_index = UINT_MAX;
  for (uint32_t i = 0; i < storage.segment_range_descriptor_.size(); i++) {
    if (storage.segment_range_descriptor_[i]->handle_ == handle) {
      segment_range_descriptor_index = i;
      break;
    }
  }
  if (segment_range_descriptor_index == UINT_MAX) {
    return;
  }
  const auto& segment_range_descriptor = storage.segment_range_descriptor_[segment_range_descriptor_index];
  storage.segments_.erase(
      storage.segments_.begin() + segment_range_descriptor->offset,
      storage.segments_.begin() + segment_range_descriptor->offset + segment_range_descriptor->range);
  for (uint32_t i = segment_range_descriptor_index + 1; i < storage.segment_range_descriptor_.size(); i++) {
    assert(storage.segment_range_descriptor_[i]->offset >= segment_range_descriptor->range);
    storage.segment_range_descriptor_[i]->offset -= segment_range_descriptor->range;
  }

  for (uint32_t i = segment_range_descriptor->offset; i < storage.segments_.size(); i++) {
    storage.segments_[i].x -= remove_chunk_size * Platform::Constants::meshlet_max_vertices_size;
    storage.segments_[i].y -= remove_chunk_size * Platform::Constants::meshlet_max_vertices_size;
    storage.segments_[i].z -= remove_chunk_size * Platform::Constants::meshlet_max_vertices_size;
    storage.segments_[i].w -= remove_chunk_size * Platform::Constants::meshlet_max_vertices_size;
  }
  storage.segment_range_descriptor_.erase(storage.segment_range_descriptor_.begin() + segment_range_descriptor_index);

  storage.require_strand_mesh_data_device_update_ = true;
}

void GeometryStorage::AllocateParticleInfo(const Handle& handle,
                                           const std::shared_ptr<RangeDescriptor>& range_descriptor) {
  auto& storage = GetInstance();
  storage.particle_info_list_data_list_.emplace_back();
  auto& info_data = storage.particle_info_list_data_list_.back();
  info_data.range_descriptor = range_descriptor;
  info_data.range_descriptor->offset = storage.particle_info_list_data_list_.size() - 1;
  info_data.range_descriptor->handle_ = handle;
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = sizeof(ParticleInfo);
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  info_data.buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  info_data.descriptor_set = std::make_shared<DescriptorSet>(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetParticleInstancedDataDescriptorSetLayout());
  info_data.status = ParticleInfoListDataStatus::UpdatePending;
}

void GeometryStorage::UpdateParticleInfo(const std::shared_ptr<RangeDescriptor>& range_descriptor,
                                         const std::vector<ParticleInfo>& particle_infos) {
  auto& storage = GetInstance();
  assert(range_descriptor->offset < storage.particle_info_list_data_list_.size());
  auto& info_data = storage.particle_info_list_data_list_.at(range_descriptor->offset);
  assert(info_data.status != ParticleInfoListDataStatus::Removed);
  if (particle_infos.empty() && info_data.particle_info_list.empty()) {
    return;
  }
  info_data.particle_info_list = particle_infos;
  info_data.status = ParticleInfoListDataStatus::UpdatePending;
}

void GeometryStorage::FreeParticleInfo(const std::shared_ptr<RangeDescriptor>& range_descriptor) {
  auto& storage = GetInstance();
  if (!storage.initialized_) {
    return;
  }
  assert(range_descriptor->offset < storage.particle_info_list_data_list_.size());
  auto& info_data = storage.particle_info_list_data_list_.at(range_descriptor->offset);
  assert(info_data.status != ParticleInfoListDataStatus::Removed);
  info_data.status = ParticleInfoListDataStatus::Removed;
}

const std::vector<ParticleInfo>& GeometryStorage::PeekParticleInfoList(
    const std::shared_ptr<RangeDescriptor>& range_descriptor) {
  const auto& storage = GetInstance();
  return storage.particle_info_list_data_list_[range_descriptor->offset].particle_info_list;
}

const std::shared_ptr<DescriptorSet>& GeometryStorage::PeekDescriptorSet(
    const std::shared_ptr<RangeDescriptor>& range_descriptor) {
  const auto& storage = GetInstance();
  return storage.particle_info_list_data_list_[range_descriptor->offset].descriptor_set;
}

const Meshlet& GeometryStorage::PeekMeshlet(const uint32_t meshlet_index) {
  const auto& storage = GetInstance();
  return storage.meshlets_[meshlet_index];
}

const SkinnedMeshlet& GeometryStorage::PeekSkinnedMeshlet(const uint32_t skinned_meshlet_index) {
  const auto& storage = GetInstance();
  return storage.skinned_meshlets_[skinned_meshlet_index];
}

const StrandMeshlet& GeometryStorage::PeekStrandMeshlet(const uint32_t strand_meshlet_index) {
  const auto& storage = GetInstance();
  return storage.strand_meshlets_[strand_meshlet_index];
}

void GeometryStorage::OnDestroy() {
  auto& storage = GetInstance();
  WaitPendingUpload(storage.pending_mesh_upload_);
  storage.CompletePendingUpload(storage.pending_mesh_upload_);
  WaitPendingUpload(storage.pending_skinned_mesh_upload_);
  storage.CompletePendingUpload(storage.pending_skinned_mesh_upload_);
  WaitPendingUpload(storage.pending_strand_upload_);
  storage.CompletePendingUpload(storage.pending_strand_upload_);
  ClearPendingUpload(storage.pending_mesh_upload_);
  ClearPendingUpload(storage.pending_skinned_mesh_upload_);
  ClearPendingUpload(storage.pending_strand_upload_);

  storage.vertex_data_chunks_.clear();
  storage.meshlets_.clear();
  storage.meshlet_range_descriptor_.clear();
  storage.triangles_.clear();
  storage.triangle_range_descriptor_.clear();

  storage.vertex_buffer_.reset();
  storage.meshlet_buffer_.reset();
  storage.triangle_buffer_.reset();

  storage.skinned_vertex_data_chunks_.clear();
  storage.skinned_meshlets_.clear();
  storage.skinned_meshlet_range_descriptor_.clear();
  storage.skinned_triangles_.clear();
  storage.skinned_triangle_range_descriptor_.clear();

  storage.skinned_vertex_buffer_.reset();
  storage.skinned_meshlet_buffer_.reset();
  storage.skinned_triangle_buffer_.reset();

  storage.strand_point_data_chunks_.clear();
  storage.strand_meshlets_.clear();
  storage.strand_meshlet_range_descriptor_.clear();
  storage.segments_.clear();
  storage.segment_range_descriptor_.clear();

  storage.strand_point_buffer_.reset();
  storage.strand_meshlet_buffer_.reset();
  storage.segment_buffer_.reset();

  storage.particle_info_list_data_list_.clear();
  storage.initialized_ = false;
}
