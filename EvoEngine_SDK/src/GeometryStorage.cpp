#include "GeometryStorage.hpp"
#include "Application.hpp"
#include "Jobs.hpp"
#include "RenderLayer.hpp"
#include "meshoptimizer.h"

#include <algorithm>
#include <cstddef>
#include <type_traits>

using namespace evo_engine;

namespace {
class GeometryUploadScope {
  bool& in_progress_;
  bool active_ = false;

 public:
  explicit GeometryUploadScope(bool& in_progress) : in_progress_(in_progress) {
    if (!in_progress_) {
      in_progress_ = true;
      active_ = true;
    }
  }

  ~GeometryUploadScope() {
    if (active_) {
      in_progress_ = false;
    }
  }

  [[nodiscard]] bool Active() const {
    return active_;
  }
};
}  // namespace

void GeometryStorage::DirtyRange::Mark(const size_t range_begin, const size_t count) {
  if (count == 0) {
    return;
  }
  const auto range_end = range_begin + count;
  if (!dirty) {
    dirty = true;
    begin = range_begin;
    end = range_end;
    return;
  }
  begin = std::min(begin, range_begin);
  end = std::max(end, range_end);
}

void GeometryStorage::DirtyRange::MarkTail(const size_t range_begin, const size_t range_end) {
  if (range_begin >= range_end) {
    return;
  }
  Mark(range_begin, range_end - range_begin);
}

void GeometryStorage::DirtyRange::Clear() {
  *this = {};
}

bool GeometryStorage::DirtyRange::Empty() const {
  return !dirty || begin >= end;
}

void GeometryStorage::ClearMeshDirtyRanges() {
  mesh_vertex_dirty_range_.Clear();
  meshlet_dirty_range_.Clear();
  triangle_dirty_range_.Clear();
}

void GeometryStorage::ClearSkinnedMeshDirtyRanges() {
  skinned_vertex_dirty_range_.Clear();
  skinned_meshlet_dirty_range_.Clear();
  skinned_triangle_dirty_range_.Clear();
}

void GeometryStorage::ClearStrandDirtyRanges() {
  strand_point_dirty_range_.Clear();
  strand_meshlet_dirty_range_.Clear();
  segment_dirty_range_.Clear();
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
  upload.scheduling = false;
  upload.handles.clear();
  upload.meshlet_commits.clear();
  upload.index_commits.clear();
}

bool GeometryStorage::CompletePendingUpload(PendingGeometryUpload& upload) {
  if (upload.scheduling || !upload.active || !IsPendingUploadCompleted(upload)) {
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

GpuWorkHandle GeometryStorage::ScheduleDirtyBufferUpload(const DirtyBufferUpload& upload) {
  if (!upload.buffer || !*upload.buffer || !upload.dirty_range || upload.dirty_range->Empty() ||
      upload.data == nullptr || upload.element_count == 0 || upload.element_size == 0) {
    return {};
  }

  const auto begin = std::min(upload.dirty_range->begin, upload.element_count);
  const auto end = std::min(upload.dirty_range->end, upload.element_count);
  if (begin >= end) {
    return {};
  }

  const auto& buffer = *upload.buffer;
  const auto full_upload_size = upload.element_count * upload.element_size;
  const auto range_offset = begin * upload.element_size;
  const auto range_size = (end - begin) * upload.element_size;
  if (end * upload.element_size > buffer->GetSize()) {
    return buffer->UploadDataAsync(full_upload_size, upload.data);
  }

  const auto* bytes = static_cast<const std::byte*>(upload.data);
  return buffer->UploadSubDataAsync(range_size, bytes + range_offset, range_offset);
}

void GeometryStorage::ScheduleUploadGroup(bool& dirty, PendingGeometryUpload& pending_upload,
                                          std::initializer_list<DirtyBufferUpload> uploads,
                                          const std::vector<std::shared_ptr<RangeDescriptor>>& meshlet_descriptors,
                                          const std::vector<std::shared_ptr<RangeDescriptor>>& index_descriptors) {
  if (!dirty || pending_upload.active || pending_upload.scheduling) {
    return;
  }

  pending_upload.scheduling = true;
  pending_upload.handles.clear();
  pending_upload.handles.reserve(uploads.size());
  try {
    for (const auto& upload : uploads) {
      pending_upload.handles.emplace_back(ScheduleDirtyBufferUpload(upload));
      if (upload.dirty_range) {
        upload.dirty_range->Clear();
      }
    }
    CaptureRangeCommits(meshlet_descriptors, pending_upload.meshlet_commits);
    CaptureRangeCommits(index_descriptors, pending_upload.index_commits);
    dirty = false;

    if (HasValidUploadHandle(pending_upload)) {
      pending_upload.active = true;
      pending_upload.scheduling = false;
      return;
    }

    ApplyRangeCommits(pending_upload.meshlet_commits);
    ApplyRangeCommits(pending_upload.index_commits);
    ClearPendingUpload(pending_upload);
    version_++;
  } catch (...) {
    pending_upload.scheduling = false;
    throw;
  }
}

void GeometryStorage::SchedulePendingUploads() {
  const auto make_upload = [](const std::shared_ptr<Buffer>& buffer, const auto& data, DirtyRange& dirty_range) {
    return DirtyBufferUpload{&buffer, data.empty() ? nullptr : static_cast<const void*>(data.data()), data.size(),
                             sizeof(typename std::decay_t<decltype(data)>::value_type), &dirty_range};
  };

  ScheduleUploadGroup(require_mesh_data_device_update_, pending_mesh_upload_,
                      {make_upload(vertex_buffer_, vertex_data_chunks_, mesh_vertex_dirty_range_),
                       make_upload(meshlet_buffer_, meshlets_, meshlet_dirty_range_),
                       make_upload(triangle_buffer_, triangles_, triangle_dirty_range_)},
                      meshlet_range_descriptor_, triangle_range_descriptor_);

  ScheduleUploadGroup(require_skinned_mesh_data_device_update_, pending_skinned_mesh_upload_,
                      {make_upload(skinned_vertex_buffer_, skinned_vertex_data_chunks_, skinned_vertex_dirty_range_),
                       make_upload(skinned_meshlet_buffer_, skinned_meshlets_, skinned_meshlet_dirty_range_),
                       make_upload(skinned_triangle_buffer_, skinned_triangles_, skinned_triangle_dirty_range_)},
                      skinned_meshlet_range_descriptor_, skinned_triangle_range_descriptor_);

  ScheduleUploadGroup(require_strand_mesh_data_device_update_, pending_strand_upload_,
                      {make_upload(strand_point_buffer_, strand_point_data_chunks_, strand_point_dirty_range_),
                       make_upload(strand_meshlet_buffer_, strand_meshlets_, strand_meshlet_dirty_range_),
                       make_upload(segment_buffer_, segments_, segment_dirty_range_)},
                      strand_meshlet_range_descriptor_, segment_range_descriptor_);
}

void GeometryStorage::UploadData() {
  GeometryUploadScope upload_scope(upload_data_in_progress_);
  if (!upload_scope.Active()) {
    return;
  }
  CompletePendingUploads();
  BottomLevelAccelerationStructure::ProcessStaticBuilds();
  if (!BottomLevelAccelerationStructure::StaticBuildInProgress()) {
    SchedulePendingUploads();
  }
  CompletePendingUploads();
  BottomLevelAccelerationStructure::ProcessStaticBuilds();

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
  if (Platform::RayAccelerationStructureEnabled()) {
    storage_buffer_create_info.usage |= VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                                        VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
  }
  storage.vertex_buffer_ = std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);
  storage_buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  storage.meshlet_buffer_ = std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);

  storage_buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
  if (Platform::RayAccelerationStructureEnabled()) {
    storage_buffer_create_info.usage |= VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                                        VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
  }
  storage.triangle_buffer_ = std::make_shared<Buffer>(storage_buffer_create_info, vertices_vma_allocation_create_info);

  storage.require_mesh_data_device_update_ = false;
  storage.ClearMeshDirtyRanges();

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
  storage.ClearSkinnedMeshDirtyRanges();

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
  storage.ClearStrandDirtyRanges();
  storage.initialized_ = true;
}

uint32_t GeometryStorage::GetVersion() {
  return GetInstance().version_;
}

bool GeometryStorage::HasPendingUploads() {
  const auto& storage = GetInstance();
  return storage.require_mesh_data_device_update_ || storage.pending_mesh_upload_.active ||
         storage.require_skinned_mesh_data_device_update_ || storage.pending_skinned_mesh_upload_.active ||
         storage.require_strand_mesh_data_device_update_ || storage.pending_strand_upload_.active;
}

void GeometryStorage::WaitForPendingUploads() {
  if (!Platform::Initialized()) {
    return;
  }
  auto& storage = GetInstance();
  GeometryUploadScope upload_scope(storage.upload_data_in_progress_);
  if (!upload_scope.Active()) {
    return;
  }
  storage.CompletePendingUploads();
  BottomLevelAccelerationStructure::ProcessStaticBuilds();
  if (BottomLevelAccelerationStructure::StaticBuildInProgress() && HasPendingUploads()) {
    BottomLevelAccelerationStructure::WaitForActiveStaticBuild();
  }
  if (!BottomLevelAccelerationStructure::StaticBuildInProgress()) {
    storage.SchedulePendingUploads();
  }
  WaitPendingUpload(storage.pending_mesh_upload_);
  WaitPendingUpload(storage.pending_skinned_mesh_upload_);
  WaitPendingUpload(storage.pending_strand_upload_);
  storage.CompletePendingUploads();
  BottomLevelAccelerationStructure::ProcessStaticBuilds();
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

const glm::uvec3& GeometryStorage::PeekTriangle(const size_t triangle_index) {
  return GetInstance().triangles_[triangle_index];
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
                                   const std::shared_ptr<RangeDescriptor>& target_triangle_range,
                                   std::vector<uint32_t>* packed_source_vertex_indices) {
  if (vertices.empty() || triangles.empty()) {
    throw std::runtime_error("Empty vertices or triangles!");
  }
  auto& storage = GetInstance();
  WaitPendingUpload(storage.pending_mesh_upload_);
  storage.CompletePendingUpload(storage.pending_mesh_upload_);
  const auto meshlet_begin = storage.meshlets_.size();
  const auto triangle_begin = storage.triangles_.size();

  // const auto meshletRange = std::make_shared<RangeDescriptor>();
  target_meshlet_range->handle_ = handle;
  target_meshlet_range->offset = meshlet_begin;
  target_meshlet_range->range = 0;

  // const auto triangleRange = std::make_shared<RangeDescriptor>();
  target_triangle_range->handle_ = handle;
  target_triangle_range->offset = triangle_begin;
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
  if (packed_source_vertex_indices) {
    packed_source_vertex_indices->clear();
    packed_source_vertex_indices->reserve(vertices.size());
  }

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
      const auto source_vertex_index = meshlet_result_vertices.at(meshlet_result.vertex_offset + vi);
      current_chunk.vertex_data[vi] = vertices[source_vertex_index];
      replacement_vertices.emplace_back(current_chunk.vertex_data[vi]);
      if (packed_source_vertex_indices) {
        packed_source_vertex_indices->emplace_back(source_vertex_index);
      }
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
  storage.mesh_vertex_dirty_range_.Mark(meshlet_begin, target_meshlet_range->range);
  storage.meshlet_dirty_range_.Mark(meshlet_begin, target_meshlet_range->range);
  storage.triangle_dirty_range_.Mark(triangle_begin, target_triangle_range->range);
  storage.require_mesh_data_device_update_ = true;
}

void GeometryStorage::UpdateMeshVertices(const std::shared_ptr<RangeDescriptor>& meshlet_range,
                                         const std::vector<Vertex>& packed_vertices) {
  if (!meshlet_range) {
    throw std::invalid_argument("Meshlet range is null.");
  }
  auto& storage = GetInstance();
  WaitPendingUpload(storage.pending_mesh_upload_);
  storage.CompletePendingUpload(storage.pending_mesh_upload_);
  if (meshlet_range->offset + meshlet_range->range > storage.meshlets_.size()) {
    throw std::runtime_error("Meshlet range is outside geometry storage.");
  }

  size_t packed_vertex_index = 0;
  for (uint32_t meshlet_index = 0; meshlet_index < meshlet_range->range; meshlet_index++) {
    const auto& meshlet = storage.meshlets_[meshlet_range->offset + meshlet_index];
    if (packed_vertex_index + meshlet.vertices_size > packed_vertices.size() ||
        meshlet.vertex_chunk_index >= storage.vertex_data_chunks_.size()) {
      throw std::runtime_error("Packed mesh vertices do not match the existing meshlet topology.");
    }
    auto& chunk = storage.vertex_data_chunks_[meshlet.vertex_chunk_index];
    std::copy_n(packed_vertices.begin() + packed_vertex_index, meshlet.vertices_size, chunk.vertex_data);
    packed_vertex_index += meshlet.vertices_size;
  }
  if (packed_vertex_index != packed_vertices.size()) {
    throw std::runtime_error("Packed mesh vertex count does not match the existing meshlet topology.");
  }
  storage.mesh_vertex_dirty_range_.Mark(meshlet_range->offset, meshlet_range->range);
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
  const auto skinned_meshlet_begin = storage.skinned_meshlets_.size();
  const auto skinned_triangle_begin = storage.skinned_triangles_.size();

  target_skinned_meshlet_range->handle_ = handle;
  target_skinned_meshlet_range->offset = skinned_meshlet_begin;
  target_skinned_meshlet_range->range = 0;

  target_skinned_triangle_range->handle_ = handle;
  target_skinned_triangle_range->offset = skinned_triangle_begin;
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
  storage.skinned_vertex_dirty_range_.Mark(skinned_meshlet_begin, target_skinned_meshlet_range->range);
  storage.skinned_meshlet_dirty_range_.Mark(skinned_meshlet_begin, target_skinned_meshlet_range->range);
  storage.skinned_triangle_dirty_range_.Mark(skinned_triangle_begin, target_skinned_triangle_range->range);
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
  const auto strand_meshlet_begin = storage.strand_meshlets_.size();
  const auto segment_begin = storage.segments_.size();

  uint32_t current_segment_index = 0;
  target_strand_meshlet_range->handle_ = handle;
  target_strand_meshlet_range->offset = strand_meshlet_begin;
  target_strand_meshlet_range->range = 0;

  target_segment_range->handle_ = handle;
  target_segment_range->offset = segment_begin;
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
  storage.strand_point_dirty_range_.Mark(strand_meshlet_begin, target_strand_meshlet_range->range);
  storage.strand_meshlet_dirty_range_.Mark(strand_meshlet_begin, target_strand_meshlet_range->range);
  storage.segment_dirty_range_.Mark(segment_begin, target_segment_range->range);
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
  const auto meshlet_remove_offset = meshlet_range_descriptor->offset;
  const uint32_t remove_chunk_size = meshlet_range_descriptor->range;
  storage.meshlets_.erase(storage.meshlets_.begin() + meshlet_range_descriptor->offset,
                          storage.meshlets_.begin() + meshlet_range_descriptor->offset + remove_chunk_size);
  storage.vertex_data_chunks_.erase(
      storage.vertex_data_chunks_.begin() + meshlet_range_descriptor->offset,
      storage.vertex_data_chunks_.begin() + meshlet_range_descriptor->offset + remove_chunk_size);
  for (uint32_t i = meshlet_remove_offset; i < storage.meshlets_.size(); i++) {
    storage.meshlets_[i].vertex_chunk_index = i;
  }
  for (uint32_t i = meshlet_range_descriptor_index + 1; i < storage.meshlet_range_descriptor_.size(); i++) {
    assert(storage.meshlet_range_descriptor_[i]->offset >= meshlet_range_descriptor->range);
    storage.meshlet_range_descriptor_[i]->offset -= meshlet_range_descriptor->range;
  }
  storage.meshlet_range_descriptor_.erase(storage.meshlet_range_descriptor_.begin() + meshlet_range_descriptor_index);
  storage.mesh_vertex_dirty_range_.MarkTail(meshlet_remove_offset, storage.vertex_data_chunks_.size());
  storage.meshlet_dirty_range_.MarkTail(meshlet_remove_offset, storage.meshlets_.size());

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
  const auto triangle_remove_offset = triangle_range_descriptor->offset;
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
  storage.triangle_dirty_range_.MarkTail(triangle_remove_offset, storage.triangles_.size());
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
  const auto skinned_meshlet_remove_offset = skinned_meshlet_range_descriptor->offset;
  const uint32_t remove_chunk_size = skinned_meshlet_range_descriptor->range;
  storage.skinned_meshlets_.erase(
      storage.skinned_meshlets_.begin() + skinned_meshlet_range_descriptor->offset,
      storage.skinned_meshlets_.begin() + skinned_meshlet_range_descriptor->offset + remove_chunk_size);
  storage.skinned_vertex_data_chunks_.erase(
      storage.skinned_vertex_data_chunks_.begin() + skinned_meshlet_range_descriptor->offset,
      storage.skinned_vertex_data_chunks_.begin() + skinned_meshlet_range_descriptor->offset + remove_chunk_size);
  for (uint32_t i = skinned_meshlet_remove_offset; i < storage.skinned_meshlets_.size(); i++) {
    storage.skinned_meshlets_[i].skinned_vertex_chunk_index = i;
  }
  for (uint32_t i = skinned_meshlet_range_descriptor_index + 1; i < storage.skinned_meshlet_range_descriptor_.size();
       i++) {
    assert(storage.skinned_meshlet_range_descriptor_[i]->offset >= skinned_meshlet_range_descriptor->range);
    storage.skinned_meshlet_range_descriptor_[i]->offset -= skinned_meshlet_range_descriptor->range;
  }
  storage.skinned_meshlet_range_descriptor_.erase(storage.skinned_meshlet_range_descriptor_.begin() +
                                                  skinned_meshlet_range_descriptor_index);
  storage.skinned_vertex_dirty_range_.MarkTail(skinned_meshlet_remove_offset,
                                               storage.skinned_vertex_data_chunks_.size());
  storage.skinned_meshlet_dirty_range_.MarkTail(skinned_meshlet_remove_offset, storage.skinned_meshlets_.size());

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
  const auto skinned_triangle_remove_offset = skinned_triangle_range_descriptor->offset;
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
  storage.skinned_triangle_dirty_range_.MarkTail(skinned_triangle_remove_offset, storage.skinned_triangles_.size());
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
  const auto strand_meshlet_remove_offset = strand_meshlet_range_descriptor->offset;
  const uint32_t remove_chunk_size = strand_meshlet_range_descriptor->range;
  storage.strand_meshlets_.erase(
      storage.strand_meshlets_.begin() + strand_meshlet_range_descriptor->offset,
      storage.strand_meshlets_.begin() + strand_meshlet_range_descriptor->offset + remove_chunk_size);
  storage.strand_point_data_chunks_.erase(
      storage.strand_point_data_chunks_.begin() + strand_meshlet_range_descriptor->offset,
      storage.strand_point_data_chunks_.begin() + strand_meshlet_range_descriptor->offset + remove_chunk_size);
  for (uint32_t i = strand_meshlet_remove_offset; i < storage.strand_meshlets_.size(); i++) {
    storage.strand_meshlets_[i].strand_point_chunk_index = i;
  }
  for (uint32_t i = strand_meshlet_range_descriptor_index + 1; i < storage.strand_meshlet_range_descriptor_.size();
       i++) {
    assert(storage.strand_meshlet_range_descriptor_[i]->offset >= strand_meshlet_range_descriptor->range);
    storage.strand_meshlet_range_descriptor_[i]->offset -= strand_meshlet_range_descriptor->range;
  }
  storage.strand_meshlet_range_descriptor_.erase(storage.strand_meshlet_range_descriptor_.begin() +
                                                 strand_meshlet_range_descriptor_index);
  storage.strand_point_dirty_range_.MarkTail(strand_meshlet_remove_offset, storage.strand_point_data_chunks_.size());
  storage.strand_meshlet_dirty_range_.MarkTail(strand_meshlet_remove_offset, storage.strand_meshlets_.size());

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
  const auto segment_remove_offset = segment_range_descriptor->offset;
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
  storage.segment_dirty_range_.MarkTail(segment_remove_offset, storage.segments_.size());

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
  BottomLevelAccelerationStructure::WaitForStaticBuilds();
  ClearPendingUpload(storage.pending_mesh_upload_);
  ClearPendingUpload(storage.pending_skinned_mesh_upload_);
  ClearPendingUpload(storage.pending_strand_upload_);
  storage.ClearMeshDirtyRanges();
  storage.ClearSkinnedMeshDirtyRanges();
  storage.ClearStrandDirtyRanges();

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
