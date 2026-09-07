#include "GraphicsResources.hpp"

#include "Application.hpp"
#include "Console.hpp"
#include "GeometryStorage.hpp"
#include "Jobs.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "SkinnedMesh.hpp"
#include "Utilities.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <thread>
#include <unordered_set>

using namespace evo_engine;

namespace {
constexpr uint32_t kDdgiRayMaskGeometry = 0x01u;
constexpr uint32_t kDdgiRayMaskShadow = 0x02u;
constexpr uint32_t kCameraLssRayMaskGeometry = 0x04u;
constexpr uint32_t kCameraLssRayMaskShadow = 0x08u;
constexpr VkBuildAccelerationStructureFlagsKHR kTlasBuildFlags =
    VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
constexpr VkBuildAccelerationStructureFlagsKHR kBlasBuildFlags =
    VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
static_assert(sizeof(VkAccelerationStructureInstanceKHR) == 64);

VkPipelineStageFlags2 RayTraversalStageMask() {
  VkPipelineStageFlags2 stages = 0;
  if (Platform::RayTracingEnabled()) {
    stages |= VK_PIPELINE_STAGE_2_RAY_TRACING_SHADER_BIT_KHR;
  }
  if (Platform::RayQueryEnabled()) {
    stages |= VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
  }
  return stages;
}

VkGeometryInstanceFlagsKHR BuildGltfRayTracingInstanceFlags(
    const RenderInstanceStorage::IRenderInstance& render_instance,
    const std::vector<GltfShadeMaterial>& gltf_shade_materials) {
  VkGeometryInstanceFlagsKHR flags{};
  const auto material_index = render_instance.material_index;
  if (material_index < 0 || static_cast<size_t>(material_index) >= gltf_shade_materials.size()) {
    flags |= VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
  } else {
    const auto& material = gltf_shade_materials[material_index];
    const bool opaque = material.transmission_factor == 0.0f &&
                        material.alpha_mode == static_cast<int32_t>(GltfAlphaMode::Opaque) &&
                        material.diffuse_transmission_factor == 0.0f;
    if (opaque) {
      flags |= VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR;
    }
    if (material.double_sided != 0 || material.thickness_factor > 0.0f || material.transmission_factor > 0.0f) {
      flags |= VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
    }
  }
  return flags;
}

bool IsValidAccelerationStructureTransform(const glm::mat4& model) {
  for (glm::length_t column = 0; column < 4; ++column) {
    for (glm::length_t row = 0; row < 4; ++row) {
      if (!std::isfinite(model[column][row])) {
        return false;
      }
    }
  }
  const auto determinant = glm::determinant(glm::mat3(model));
  return std::isfinite(determinant) && determinant != 0.0f;
}

void SetAccelerationStructureTransform(VkAccelerationStructureInstanceKHR& instance, const glm::mat4& model) {
  const auto transposed = glm::transpose(model);
  memcpy(&instance.transform.matrix[0][0], glm::value_ptr(transposed), sizeof(VkTransformMatrixKHR));
}

bool AccelerationStructureInstancesEqual(const VkAccelerationStructureInstanceKHR& lhs,
                                         const VkAccelerationStructureInstanceKHR& rhs) {
  return memcmp(&lhs.transform, &rhs.transform, sizeof(VkTransformMatrixKHR)) == 0 &&
         lhs.instanceCustomIndex == rhs.instanceCustomIndex && lhs.mask == rhs.mask &&
         lhs.instanceShaderBindingTableRecordOffset == rhs.instanceShaderBindingTableRecordOffset &&
         lhs.flags == rhs.flags && lhs.accelerationStructureReference == rhs.accelerationStructureReference;
}

VkAccelerationStructureGeometryKHR CreateBlasGeometry(const Buffer& vertex_buffer, const Buffer& index_buffer,
                                                      const Buffer& transform_buffer, const uint32_t vertex_count) {
  VkAccelerationStructureGeometryKHR geometry{};
  geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
  geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
  geometry.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
  geometry.geometry.triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
  geometry.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
  geometry.geometry.triangles.maxVertex = vertex_count - 1;
  geometry.geometry.triangles.vertexStride = sizeof(Vertex);
  geometry.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;
  geometry.geometry.triangles.vertexData.deviceAddress = vertex_buffer.GetDeviceAddress();
  geometry.geometry.triangles.indexData.deviceAddress = index_buffer.GetDeviceAddress();
  geometry.geometry.triangles.transformData.deviceAddress = transform_buffer.GetDeviceAddress();
  return geometry;
}

bool HasFiniteBlasPositions(const std::vector<Vertex>& vertices) {
  return std::all_of(vertices.begin(), vertices.end(), [](const Vertex& vertex) {
    return std::isfinite(vertex.position.x) && std::isfinite(vertex.position.y) && std::isfinite(vertex.position.z);
  });
}

bool HasValidLinearSweptSphereInput(const std::vector<StrandPoint>& points, const std::vector<uint32_t>& indices) {
  return std::all_of(points.begin(), points.end(),
                     [](const StrandPoint& point) {
                       return std::isfinite(point.position.x) && std::isfinite(point.position.y) &&
                              std::isfinite(point.position.z) && std::isfinite(point.thickness) &&
                              point.thickness >= 0.0f;
                     }) &&
         std::all_of(indices.begin(), indices.end(), [&](const uint32_t index) {
           return index < points.size() - 1u;
         });
}

VkDeviceSize SaturatingAdd(const VkDeviceSize left, const VkDeviceSize right) {
  return right > (std::numeric_limits<VkDeviceSize>::max)() - left ? (std::numeric_limits<VkDeviceSize>::max)()
                                                                   : left + right;
}

VkDeviceSize AlignUp(const VkDeviceSize value, const VkDeviceSize alignment) {
  const auto resolved_alignment = std::max<VkDeviceSize>(1, alignment);
  const auto remainder = value % resolved_alignment;
  return remainder == 0 ? value : SaturatingAdd(value, resolved_alignment - remainder);
}

void RequireVkSuccess(const VkResult result, const char* operation) {
  if (Platform::CheckVk(result) != VK_SUCCESS) {
    throw std::runtime_error(std::string(operation) + " failed.");
  }
}

struct PendingStaticBlasBuild {
  std::weak_ptr<BottomLevelAccelerationStructure> target;
  std::shared_ptr<RangeDescriptor> meshlet_range;
  std::shared_ptr<RangeDescriptor> triangle_range;
  uint32_t meshlet_offset = 0;
  uint32_t meshlet_count = 0;
  uint32_t triangle_offset = 0;
  uint32_t triangle_count = 0;
};

struct StaticBlasBuildResult {
  std::weak_ptr<BottomLevelAccelerationStructure> target;
  VkAccelerationStructureKHR acceleration_structure = VK_NULL_HANDLE;
  std::shared_ptr<Buffer> buffer;
  VkDeviceAddress device_address = 0;
  VkDeviceSize uncompacted_size = 0;
  VkDeviceSize compacted_size = 0;
};

struct StaticBlasBuildJob {
  std::vector<PendingStaticBlasBuild> requests;
  std::vector<PendingStaticBlasBuild> deferred_requests;
  std::vector<StaticBlasBuildResult> results;
  StaticBlasBuildTelemetry telemetry;
  uint64_t retained_compacted_bytes = 0;
};

struct StaticBlasBuilderState {
  std::mutex mutex;
  std::vector<PendingStaticBlasBuild> pending;
  std::shared_ptr<StaticBlasBuildJob> active_job;
  GpuWorkHandle active_handle;
  uint64_t active_pending_count = 0;
  StaticBlasBuildTelemetry telemetry;
};

StaticBlasBuilderState& GetStaticBlasBuilderState() {
  static StaticBlasBuilderState state;
  return state;
}

std::shared_ptr<Buffer> CreateAccelerationStructureBuffer(const VkDeviceSize size) {
  VkBufferCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  create_info.size = size;
  create_info.usage =
      VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_info{};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  return std::make_shared<Buffer>(create_info, allocation_info);
}

VkAccelerationStructureKHR CreateBottomLevelAccelerationStructure(const std::shared_ptr<Buffer>& buffer,
                                                                  const VkDeviceSize size) {
  VkAccelerationStructureCreateInfoKHR create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
  create_info.buffer = buffer->GetVkBuffer();
  create_info.size = size;
  create_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  VkAccelerationStructureKHR result = VK_NULL_HANDLE;
  RequireVkSuccess(vkCreateAccelerationStructureKHR(Platform::GetVkDevice(), &create_info, nullptr, &result),
                   "Bottom-level acceleration-structure creation");
  if (result == VK_NULL_HANDLE) {
    throw std::runtime_error("Bottom-level acceleration-structure creation returned a null handle.");
  }
  return result;
}

void DestroyAccelerationStructure(VkAccelerationStructureKHR& acceleration_structure) {
  if (acceleration_structure != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyAccelerationStructureKHR(Platform::GetVkDevice(), acceleration_structure, nullptr);
    acceleration_structure = VK_NULL_HANDLE;
  }
}
}  // namespace

std::vector<StaticBlasBuildPassPlan> evo_engine::PlanStaticBlasBuildPasses(
    const std::vector<StaticBlasBuildSize>& build_sizes, const VkDeviceSize scratch_alignment,
    const VkDeviceSize budget) {
  if (budget == 0) {
    throw std::invalid_argument("Static BLAS build budget must be nonzero.");
  }
  std::vector<StaticBlasBuildPassPlan> result;
  for (size_t begin = 0; begin < build_sizes.size();) {
    StaticBlasBuildPassPlan pass;
    pass.begin = begin;
    while (begin + pass.count < build_sizes.size()) {
      const auto& candidate = build_sizes[begin + pass.count];
      const auto candidate_scratch = AlignUp(candidate.scratch_size, scratch_alignment);
      if (pass.count != 0 &&
          (candidate.destination_size > budget || pass.destination_size > budget - candidate.destination_size ||
           candidate_scratch > budget)) {
        break;
      }
      pass.destination_size = SaturatingAdd(pass.destination_size, candidate.destination_size);
      ++pass.count;
      if (pass.destination_size >= budget || candidate_scratch > budget) {
        break;
      }
    }
    VkDeviceSize scratch_sum = 0;
    VkDeviceSize largest_scratch = 0;
    for (size_t index = 0; index < pass.count; ++index) {
      const auto aligned = AlignUp(build_sizes[begin + index].scratch_size, scratch_alignment);
      scratch_sum = SaturatingAdd(scratch_sum, aligned);
      largest_scratch = std::max(largest_scratch, aligned);
    }
    pass.scratch_size = scratch_sum <= budget ? scratch_sum : std::max(largest_scratch, budget);
    pass.oversized_singleton = pass.count == 1 && (pass.destination_size > budget || pass.scratch_size > budget);

    VkDeviceSize wave_size = 0;
    for (size_t index = 0; index < pass.count; ++index) {
      const auto aligned = AlignUp(build_sizes[begin + index].scratch_size, scratch_alignment);
      if (aligned == 0) {
        continue;
      }
      if (wave_size == 0 || aligned > pass.scratch_size - wave_size) {
        ++pass.scratch_wave_count;
        wave_size = aligned;
      } else {
        wave_size += aligned;
      }
    }
    result.emplace_back(pass);
    begin += pass.count;
  }
  return result;
}

namespace {
struct StaticBlasBuildRecord {
  PendingStaticBlasBuild request;
  VkAccelerationStructureGeometryKHR geometry{};
  VkAccelerationStructureBuildSizesInfoKHR sizes{};
  std::shared_ptr<Buffer> original_buffer;
  VkAccelerationStructureKHR original = VK_NULL_HANDLE;
  std::shared_ptr<Buffer> compact_buffer;
  VkAccelerationStructureKHR compact = VK_NULL_HANDLE;
};

void RecordAccelerationStructureBarrier(const VkCommandBuffer command_buffer) {
  VkMemoryBarrier2 barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
  barrier.srcStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
  barrier.srcAccessMask = VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
  barrier.dstStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
  barrier.dstAccessMask =
      VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR | VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
  VkDependencyInfo dependency{};
  dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependency.memoryBarrierCount = 1;
  dependency.pMemoryBarriers = &barrier;
  vkCmdPipelineBarrier2(command_buffer, &dependency);
}

void ExecuteStaticBlasBuildJob(const std::shared_ptr<StaticBlasBuildJob>& job,
                               const std::shared_ptr<Buffer>& vertex_buffer,
                               const std::shared_ptr<Buffer>& index_buffer) {
  const auto wall_start = std::chrono::steady_clock::now();
  std::vector<StaticBlasBuildRecord> records;
  try {
    const auto vertex_capacity = vertex_buffer->GetSize() / sizeof(Vertex);
    if (vertex_capacity == 0 || vertex_capacity > (std::numeric_limits<uint32_t>::max)()) {
      throw std::runtime_error("Geometry storage vertex capacity is invalid for static BLAS construction.");
    }
    const auto vertex_address = vertex_buffer->GetDeviceAddress();
    const auto index_address = index_buffer->GetDeviceAddress();
    records.reserve(job->requests.size());
    std::vector<StaticBlasBuildSize> build_sizes;
    build_sizes.reserve(job->requests.size());
    for (const auto& request : job->requests) {
      auto& record = records.emplace_back();
      record.request = request;
      record.geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
      record.geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
      record.geometry.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      auto& triangles = record.geometry.geometry.triangles;
      triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
      triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
      triangles.vertexData.deviceAddress = vertex_address;
      triangles.vertexStride = sizeof(Vertex);
      triangles.maxVertex = static_cast<uint32_t>(vertex_capacity - 1);
      triangles.indexType = VK_INDEX_TYPE_UINT32;
      triangles.indexData.deviceAddress = index_address;

      VkAccelerationStructureBuildGeometryInfoKHR size_info{};
      size_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
      size_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      size_info.flags = kBlasBuildFlags | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
      size_info.geometryCount = 1;
      size_info.pGeometries = &record.geometry;
      record.sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
      vkGetAccelerationStructureBuildSizesKHR(Platform::GetVkDevice(), VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                              &size_info, &request.triangle_count, &record.sizes);
      build_sizes.emplace_back(
          StaticBlasBuildSize{record.sizes.accelerationStructureSize, record.sizes.buildScratchSize});
    }

    const auto scratch_alignment = std::max<VkDeviceSize>(
        1, Platform::GetSelectedPhysicalDevice()
               ->acceleration_structure_properties_khr.minAccelerationStructureScratchOffsetAlignment);
    auto passes = PlanStaticBlasBuildPasses(build_sizes, scratch_alignment);
    if (passes.size() > 1) {
      const auto first_count = passes.front().count;
      job->deferred_requests.assign(std::make_move_iterator(job->requests.begin() + first_count),
                                    std::make_move_iterator(job->requests.end()));
      job->requests.resize(first_count);
      records.resize(first_count);
      build_sizes.resize(first_count);
      passes.resize(1);
    }
    job->telemetry.passes = passes;
    for (const auto& pass : passes) {
      job->telemetry.scratch_peak_bytes = std::max<uint64_t>(job->telemetry.scratch_peak_bytes, pass.scratch_size);
      job->telemetry.scratch_wave_count += pass.scratch_wave_count;
    }
    job->telemetry.pass_count = passes.size();

    VkBufferCreateInfo scratch_create_info{};
    scratch_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    scratch_create_info.size = std::max<uint64_t>(1, job->telemetry.scratch_peak_bytes) + scratch_alignment - 1;
    scratch_create_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    scratch_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VmaAllocationCreateInfo scratch_allocation_info{};
    scratch_allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    auto scratch_buffer = std::make_shared<Buffer>(scratch_create_info, scratch_allocation_info);
    scratch_buffer->SetDebugName("Static BLAS Shared Scratch");
    const auto scratch_address = AlignUp(scratch_buffer->GetDeviceAddress(), scratch_alignment);

    uint64_t retained_compacted_bytes = job->retained_compacted_bytes;
    for (const auto& pass : passes) {
      VkQueryPool query_pool = VK_NULL_HANDLE;
      try {
        uint64_t original_bytes = 0;
        std::vector<VkAccelerationStructureKHR> originals;
        originals.reserve(pass.count);
        for (size_t local_index = 0; local_index < pass.count; ++local_index) {
          auto& record = records[pass.begin + local_index];
          record.original_buffer = CreateAccelerationStructureBuffer(record.sizes.accelerationStructureSize);
          record.original =
              CreateBottomLevelAccelerationStructure(record.original_buffer, record.sizes.accelerationStructureSize);
          originals.emplace_back(record.original);
          original_bytes = SaturatingAdd(original_bytes, record.sizes.accelerationStructureSize);
        }
        job->telemetry.eligible_static_uncompacted_bytes =
            SaturatingAdd(job->telemetry.eligible_static_uncompacted_bytes, original_bytes);
        job->telemetry.transient_peak_bytes = std::max(
            job->telemetry.transient_peak_bytes,
            SaturatingAdd(job->telemetry.scratch_peak_bytes, SaturatingAdd(retained_compacted_bytes, original_bytes)));

        VkQueryPoolCreateInfo query_pool_info{};
        query_pool_info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
        query_pool_info.queryType = VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR;
        query_pool_info.queryCount = static_cast<uint32_t>(pass.count);
        RequireVkSuccess(vkCreateQueryPool(Platform::GetVkDevice(), &query_pool_info, nullptr, &query_pool),
                         "Static BLAS compact-size query-pool creation");

        Platform::ImmediateSubmitWithGpuTimestamp(
            {"BlasBuild", "BLAS Build", "Ray Tracing", GpuTimestampQueue::Immediate, 0, 0},
            [&](const VkCommandBuffer command_buffer) {
              vkCmdResetQueryPool(command_buffer, query_pool, 0, static_cast<uint32_t>(pass.count));
              std::vector<VkAccelerationStructureBuildGeometryInfoKHR> wave_builds;
              std::vector<VkAccelerationStructureBuildRangeInfoKHR> wave_ranges;
              std::vector<const VkAccelerationStructureBuildRangeInfoKHR*> wave_range_pointers;
              wave_builds.reserve(pass.count);
              wave_ranges.reserve(pass.count);
              wave_range_pointers.reserve(pass.count);
              VkDeviceSize wave_scratch_size = 0;
              const auto flush_wave = [&]() {
                if (wave_builds.empty()) {
                  return;
                }
                wave_range_pointers.clear();
                for (const auto& range : wave_ranges) {
                  wave_range_pointers.emplace_back(&range);
                }
                vkCmdBuildAccelerationStructuresKHR(command_buffer, static_cast<uint32_t>(wave_builds.size()),
                                                    wave_builds.data(), wave_range_pointers.data());
                RecordAccelerationStructureBarrier(command_buffer);
                wave_builds.clear();
                wave_ranges.clear();
                wave_scratch_size = 0;
              };

              for (size_t local_index = 0; local_index < pass.count; ++local_index) {
                auto& record = records[pass.begin + local_index];
                const auto aligned_scratch = AlignUp(record.sizes.buildScratchSize, scratch_alignment);
                if (wave_scratch_size != 0 && aligned_scratch > pass.scratch_size - wave_scratch_size) {
                  flush_wave();
                }
                auto& build = wave_builds.emplace_back();
                build.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
                build.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
                build.flags = kBlasBuildFlags | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
                build.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
                build.dstAccelerationStructure = record.original;
                build.geometryCount = 1;
                build.pGeometries = &record.geometry;
                build.scratchData.deviceAddress = scratch_address + wave_scratch_size;
                auto& range = wave_ranges.emplace_back();
                range.primitiveCount = record.request.triangle_count;
                range.primitiveOffset = record.request.triangle_offset * sizeof(glm::uvec3);
                wave_scratch_size += aligned_scratch;
              }
              flush_wave();
              vkCmdWriteAccelerationStructuresPropertiesKHR(
                  command_buffer, static_cast<uint32_t>(originals.size()), originals.data(),
                  VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR, query_pool, 0);
            });

        std::vector<VkDeviceSize> compact_sizes(pass.count);
        RequireVkSuccess(
            vkGetQueryPoolResults(Platform::GetVkDevice(), query_pool, 0, static_cast<uint32_t>(pass.count),
                                  compact_sizes.size() * sizeof(VkDeviceSize), compact_sizes.data(),
                                  sizeof(VkDeviceSize), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT),
            "Static BLAS compact-size query readback");

        uint64_t compact_bytes = 0;
        for (size_t local_index = 0; local_index < pass.count; ++local_index) {
          if (compact_sizes[local_index] == 0) {
            throw std::runtime_error("Static BLAS compaction returned an empty destination size.");
          }
          auto& record = records[pass.begin + local_index];
          record.compact_buffer = CreateAccelerationStructureBuffer(compact_sizes[local_index]);
          record.compact = CreateBottomLevelAccelerationStructure(record.compact_buffer, compact_sizes[local_index]);
          compact_bytes = SaturatingAdd(compact_bytes, compact_sizes[local_index]);
        }
        job->telemetry.eligible_static_compacted_bytes =
            SaturatingAdd(job->telemetry.eligible_static_compacted_bytes, compact_bytes);
        job->telemetry.transient_peak_bytes = std::max(
            job->telemetry.transient_peak_bytes,
            SaturatingAdd(job->telemetry.scratch_peak_bytes,
                          SaturatingAdd(retained_compacted_bytes, SaturatingAdd(original_bytes, compact_bytes))));

        Platform::ImmediateSubmitWithGpuTimestamp(
            {"BlasCompact", "BLAS Compact", "Ray Tracing", GpuTimestampQueue::Immediate, 0, 0},
            [&](const VkCommandBuffer command_buffer) {
              for (size_t local_index = 0; local_index < pass.count; ++local_index) {
                const auto& record = records[pass.begin + local_index];
                VkCopyAccelerationStructureInfoKHR copy_info{};
                copy_info.sType = VK_STRUCTURE_TYPE_COPY_ACCELERATION_STRUCTURE_INFO_KHR;
                copy_info.src = record.original;
                copy_info.dst = record.compact;
                copy_info.mode = VK_COPY_ACCELERATION_STRUCTURE_MODE_COMPACT_KHR;
                vkCmdCopyAccelerationStructureKHR(command_buffer, &copy_info);
              }
            });

        for (size_t local_index = 0; local_index < pass.count; ++local_index) {
          auto& record = records[pass.begin + local_index];
          DestroyAccelerationStructure(record.original);
          record.original_buffer.reset();
          VkAccelerationStructureDeviceAddressInfoKHR address_info{};
          address_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
          address_info.accelerationStructure = record.compact;
          const auto device_address =
              vkGetAccelerationStructureDeviceAddressKHR(Platform::GetVkDevice(), &address_info);
          if (device_address == 0) {
            throw std::runtime_error("Compacted static BLAS returned a zero device address.");
          }
          auto& result = job->results.emplace_back();
          result.target = record.request.target;
          result.acceleration_structure = record.compact;
          result.buffer = std::move(record.compact_buffer);
          result.device_address = device_address;
          result.uncompacted_size = record.sizes.accelerationStructureSize;
          result.compacted_size = compact_sizes[local_index];
          record.compact = VK_NULL_HANDLE;
        }
        retained_compacted_bytes = SaturatingAdd(retained_compacted_bytes, compact_bytes);
        vkDestroyQueryPool(Platform::GetVkDevice(), query_pool, nullptr);
        query_pool = VK_NULL_HANDLE;
      } catch (...) {
        if (query_pool != VK_NULL_HANDLE) {
          vkDestroyQueryPool(Platform::GetVkDevice(), query_pool, nullptr);
        }
        throw;
      }
    }
    job->telemetry.static_eligible_count = job->results.size();
    job->telemetry.shared_input_count = job->results.size();
    job->telemetry.wall_milliseconds =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_start).count();
  } catch (...) {
    for (auto& record : records) {
      DestroyAccelerationStructure(record.original);
      DestroyAccelerationStructure(record.compact);
    }
    for (auto& result : job->results) {
      DestroyAccelerationStructure(result.acceleration_structure);
    }
    job->results.clear();
    throw;
  }
}
}  // namespace

Fence::Fence(const VkFenceCreateInfo& vk_fence_create_info) {
  if (!Platform::Initialized())
    return;
  Platform::CheckVk(vkCreateFence(Platform::GetVkDevice(), &vk_fence_create_info, nullptr, &vk_fence_));
  flags_ = vk_fence_create_info.flags;
}

Fence::~Fence() {
  if (!Platform::Initialized())
    return;
  if (vk_fence_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyFence(Platform::GetVkDevice(), vk_fence_, nullptr);
    vk_fence_ = nullptr;
  }
}

const VkFence& Fence::GetVkFence() const {
  return vk_fence_;
}

Semaphore::Semaphore(const VkSemaphoreCreateInfo& semaphore_create_info) {
  if (!Platform::Initialized())
    return;
  Platform::CheckVk(vkCreateSemaphore(Platform::GetVkDevice(), &semaphore_create_info, nullptr, &vk_semaphore_));
  flags_ = semaphore_create_info.flags;
}

Semaphore::~Semaphore() {
  if (!Platform::Initialized())
    return;
  if (vk_semaphore_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroySemaphore(Platform::GetVkDevice(), vk_semaphore_, nullptr);
    vk_semaphore_ = VK_NULL_HANDLE;
  }
}

const VkSemaphore& Semaphore::GetVkSemaphore() const {
  return vk_semaphore_;
}
Swapchain::Swapchain(const VkSwapchainCreateInfoKHR& swap_chain_create_info) {
  if (!Platform::Initialized())
    return;
  const auto& device = Platform::GetVkDevice();
  Platform::CheckVk(vkCreateSwapchainKHR(Platform::GetVkDevice(), &swap_chain_create_info, nullptr, &vk_swapchain_));
  uint32_t image_count = 0;
  Platform::CheckVk(vkGetSwapchainImagesKHR(device, vk_swapchain_, &image_count, nullptr));
  vk_images_.resize(image_count);
  Platform::CheckVk(vkGetSwapchainImagesKHR(device, vk_swapchain_, &image_count, vk_images_.data()));
  flags_ = swap_chain_create_info.flags;
  surface_ = swap_chain_create_info.surface;
  min_image_count_ = swap_chain_create_info.minImageCount;
  image_format_ = swap_chain_create_info.imageFormat;
  image_extent_ = swap_chain_create_info.imageExtent;
  image_array_layers_ = swap_chain_create_info.imageArrayLayers;
  image_usage_ = swap_chain_create_info.imageUsage;
  image_sharing_mode_ = swap_chain_create_info.imageSharingMode;
  ApplyVector(queue_family_indices_, swap_chain_create_info.queueFamilyIndexCount,
              swap_chain_create_info.pQueueFamilyIndices);
  pre_transform_ = swap_chain_create_info.preTransform;
  composite_alpha_ = swap_chain_create_info.compositeAlpha;
  present_mode_ = swap_chain_create_info.presentMode;
  clipped_ = swap_chain_create_info.clipped;

  vk_image_views_.clear();
  for (size_t i = 0; i < vk_images_.size(); i++) {
    VkImageViewCreateInfo image_view_create_info{};
    image_view_create_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    image_view_create_info.image = vk_images_[i];
    image_view_create_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    image_view_create_info.format = image_format_;
    image_view_create_info.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    image_view_create_info.subresourceRange.baseMipLevel = 0;
    image_view_create_info.subresourceRange.levelCount = 1;
    image_view_create_info.subresourceRange.baseArrayLayer = 0;
    image_view_create_info.subresourceRange.layerCount = 1;
    auto image_view = std::make_shared<ImageView>(image_view_create_info);
    vk_image_views_.emplace_back(image_view);
  }
}

Swapchain::~Swapchain() {
  if (!Platform::Initialized())
    return;
  vk_image_views_.clear();
  if (vk_swapchain_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroySwapchainKHR(Platform::GetVkDevice(), vk_swapchain_, nullptr);
    vk_swapchain_ = VK_NULL_HANDLE;
  }
}

VkSwapchainKHR Swapchain::GetVkSwapchain() const {
  return vk_swapchain_;
}

const std::vector<VkImage>& Swapchain::GetAllVkImages() const {
  return vk_images_;
}

const VkImage& Swapchain::GetVkImage() const {
  return vk_images_[Platform::GetNextImageIndex()];
}

const VkImageView& Swapchain::GetVkImageView() const {
  return vk_image_views_[Platform::GetNextImageIndex()]->vk_image_view_;
}

const std::vector<std::shared_ptr<ImageView>>& Swapchain::GetAllImageViews() const {
  return vk_image_views_;
}

VkFormat Swapchain::GetImageFormat() const {
  return image_format_;
}

VkExtent2D Swapchain::GetImageExtent() const {
  return image_extent_;
}

ImageView::ImageView(const VkImageViewCreateInfo& image_view_create_info) {
  if (!Platform::Initialized())
    return;
  Platform::CheckVk(vkCreateImageView(Platform::GetVkDevice(), &image_view_create_info, nullptr, &vk_image_view_));
  image_ = nullptr;
  flags_ = image_view_create_info.flags;
  view_type_ = image_view_create_info.viewType;
  format_ = image_view_create_info.format;
  components_ = image_view_create_info.components;
  subresource_range_ = image_view_create_info.subresourceRange;
}

ImageView::ImageView(const VkImageViewCreateInfo& image_view_create_info, const std::shared_ptr<Image>& image) {
  if (!Platform::Initialized())
    return;
  Platform::CheckVk(vkCreateImageView(Platform::GetVkDevice(), &image_view_create_info, nullptr, &vk_image_view_));
  image_ = image;
  flags_ = image_view_create_info.flags;
  view_type_ = image_view_create_info.viewType;
  format_ = image->GetFormat();
  components_ = image_view_create_info.components;
  subresource_range_ = image_view_create_info.subresourceRange;
}

ImageView::~ImageView() {
  if (!Platform::Initialized())
    return;
  if (vk_image_view_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyImageView(Platform::GetVkDevice(), vk_image_view_, nullptr);
    vk_image_view_ = VK_NULL_HANDLE;
  }
}

VkImageView ImageView::GetVkImageView() const {
  return vk_image_view_;
}

const std::shared_ptr<Image>& ImageView::GetImage() const {
  return image_;
}

ShaderModule::~ShaderModule() {
  if (!Platform::Initialized())
    return;
  if (vk_shader_module_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyShaderModule(Platform::GetVkDevice(), vk_shader_module_, nullptr);
    vk_shader_module_ = VK_NULL_HANDLE;
  }
}

ShaderModule::ShaderModule(const VkShaderModuleCreateInfo& create_info) {
  if (!Platform::Initialized())
    return;
  Platform::CheckVk(vkCreateShaderModule(Platform::GetVkDevice(), &create_info, nullptr, &vk_shader_module_));
}

VkShaderModule ShaderModule::GetVkShaderModule() const {
  return vk_shader_module_;
}

PipelineLayout::PipelineLayout(const VkPipelineLayoutCreateInfo& pipeline_layout_create_info) {
  if (!Platform::Initialized())
    return;
  Platform::CheckVk(
      vkCreatePipelineLayout(Platform::GetVkDevice(), &pipeline_layout_create_info, nullptr, &vk_pipeline_layout_));

  flags_ = pipeline_layout_create_info.flags;
  ApplyVector(set_layouts_, pipeline_layout_create_info.setLayoutCount, pipeline_layout_create_info.pSetLayouts);
  ApplyVector(push_constant_ranges_, pipeline_layout_create_info.pushConstantRangeCount,
              pipeline_layout_create_info.pPushConstantRanges);
}

PipelineLayout::~PipelineLayout() {
  if (!Platform::Initialized())
    return;
  if (vk_pipeline_layout_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyPipelineLayout(Platform::GetVkDevice(), vk_pipeline_layout_, nullptr);
    vk_pipeline_layout_ = VK_NULL_HANDLE;
  }
}

VkPipelineLayout PipelineLayout::GetVkPipelineLayout() const {
  return vk_pipeline_layout_;
}

CommandPool::CommandPool(const VkCommandPoolCreateInfo& command_pool_create_info) {
  if (!Platform::Initialized())
    return;
  Platform::CheckVk(
      vkCreateCommandPool(Platform::GetVkDevice(), &command_pool_create_info, nullptr, &vk_command_pool_));
}

CommandPool::~CommandPool() {
  if (!Platform::Initialized())
    return;
  if (vk_command_pool_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyCommandPool(Platform::GetVkDevice(), vk_command_pool_, nullptr);
    vk_command_pool_ = VK_NULL_HANDLE;
  }
}

VkCommandPool CommandPool::GetVkCommandPool() const {
  return vk_command_pool_;
}

uint32_t Image::GetMipLevels() const {
  return mip_levels_;
}

Image::Image(VkImageCreateInfo image_create_info) {
  if (!Platform::Initialized())
    return;
  VmaAllocationCreateInfo alloc_info = {};
  alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
  if (Platform::CheckVk(vmaCreateImage(Platform::GetVmaAllocator(), &image_create_info, &alloc_info, &vk_image_,
                                       &vma_allocation_, &vma_allocation_info_))) {
    throw std::runtime_error("Failed to create image!");
  }
  flags_ = image_create_info.flags;
  image_type_ = image_create_info.imageType;
  format_ = image_create_info.format;
  extent_ = image_create_info.extent;
  mip_levels_ = image_create_info.mipLevels;
  array_layers_ = image_create_info.arrayLayers;
  samples_ = image_create_info.samples;
  tiling_ = image_create_info.tiling;
  usage_ = image_create_info.usage;
  sharing_mode_ = image_create_info.sharingMode;
  ApplyVector(queue_family_indices_, image_create_info.queueFamilyIndexCount, image_create_info.pQueueFamilyIndices);

  layout_ = initial_layout_ = image_create_info.initialLayout;
}

Image::Image(VkImageCreateInfo image_create_info, const VmaAllocationCreateInfo& vma_allocation_create_info) {
  if (!Platform::Initialized())
    return;
  if (Platform::CheckVk(vmaCreateImage(Platform::GetVmaAllocator(), &image_create_info, &vma_allocation_create_info,
                                       &vk_image_, &vma_allocation_, &vma_allocation_info_))) {
    throw std::runtime_error("Failed to create image!");
  }
  flags_ = image_create_info.flags;
  image_type_ = image_create_info.imageType;
  format_ = image_create_info.format;
  extent_ = image_create_info.extent;
  mip_levels_ = image_create_info.mipLevels;
  array_layers_ = image_create_info.arrayLayers;
  samples_ = image_create_info.samples;
  tiling_ = image_create_info.tiling;
  usage_ = image_create_info.usage;
  sharing_mode_ = image_create_info.sharingMode;
  ApplyVector(queue_family_indices_, image_create_info.queueFamilyIndexCount, image_create_info.pQueueFamilyIndices);

  layout_ = initial_layout_ = image_create_info.initialLayout;
}

bool Image::HasStencilComponent() const {
  return format_ == VK_FORMAT_D32_SFLOAT_S8_UINT || format_ == VK_FORMAT_D24_UNORM_S8_UINT;
}

void Image::CopyFromBuffer(const VkCommandBuffer vk_command_buffer, const VkBuffer& src_buffer,
                           VkDeviceSize src_offset) const {
  VkBufferImageCopy region{};
  region.bufferOffset = src_offset;
  region.bufferRowLength = 0;
  region.bufferImageHeight = 0;
  region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  region.imageSubresource.mipLevel = 0;
  region.imageSubresource.baseArrayLayer = 0;
  region.imageSubresource.layerCount = 1;
  region.imageOffset = {0, 0, 0};
  region.imageExtent = extent_;
  vkCmdCopyBufferToImage(vk_command_buffer, src_buffer, vk_image_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
}

void Image::CopyFromBuffer(const VkCommandBuffer vk_command_buffer, const VkBuffer& src_buffer,
                           const std::vector<VkBufferImageCopy>& regions) const {
  if (regions.empty()) {
    return;
  }
  vkCmdCopyBufferToImage(vk_command_buffer, src_buffer, vk_image_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                         static_cast<uint32_t>(regions.size()), regions.data());
}

void Image::GenerateMipmaps(const VkCommandBuffer vk_command_buffer) {
  if (mip_levels_ > 1) {
    VkFormatProperties format_properties{};
    vkGetPhysicalDeviceFormatProperties(Platform::GetSelectedPhysicalDevice()->vk_physical_device, format_,
                                        &format_properties);
    constexpr VkFormatFeatureFlags required_features = VK_FORMAT_FEATURE_BLIT_SRC_BIT | VK_FORMAT_FEATURE_BLIT_DST_BIT |
                                                       VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT;
    if ((format_properties.optimalTilingFeatures & required_features) != required_features) {
      throw std::runtime_error("Image format does not support linear mipmap blits.");
    }
  }
  VkImageMemoryBarrier barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
  barrier.image = vk_image_;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  barrier.subresourceRange.baseArrayLayer = 0;
  barrier.subresourceRange.layerCount = array_layers_;
  barrier.subresourceRange.levelCount = 1;

  int32_t mip_width = extent_.width;
  int32_t mip_height = extent_.height;

  for (uint32_t i = 1; i < mip_levels_; i++) {
    barrier.subresourceRange.baseMipLevel = i - 1;
    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

    vkCmdPipelineBarrier(vk_command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0,
                         nullptr, 0, nullptr, 1, &barrier);

    VkImageBlit blit{};
    blit.srcOffsets[0] = {0, 0, 0};
    blit.srcOffsets[1] = {mip_width, mip_height, 1};
    blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    blit.srcSubresource.mipLevel = i - 1;
    blit.srcSubresource.baseArrayLayer = 0;
    blit.srcSubresource.layerCount = array_layers_;
    blit.dstOffsets[0] = {0, 0, 0};
    blit.dstOffsets[1] = {mip_width > 1 ? mip_width / 2 : 1, mip_height > 1 ? mip_height / 2 : 1, 1};
    blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    blit.dstSubresource.mipLevel = i;
    blit.dstSubresource.baseArrayLayer = 0;
    blit.dstSubresource.layerCount = array_layers_;

    vkCmdBlitImage(vk_command_buffer, vk_image_, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, vk_image_,
                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);

    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    vkCmdPipelineBarrier(vk_command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                         nullptr, 0, nullptr, 1, &barrier);

    if (mip_width > 1)
      mip_width /= 2;
    if (mip_height > 1)
      mip_height /= 2;
  }
  barrier.subresourceRange.baseMipLevel = mip_levels_ - 1;
  barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
  barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
  barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

  vkCmdPipelineBarrier(vk_command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                       nullptr, 0, nullptr, 1, &barrier);
  layout_ = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
}

VkImage Image::GetVkImage() const {
  return vk_image_;
}

VkFormat Image::GetFormat() const {
  return format_;
}

VmaAllocation Image::GetVmaAllocation() const {
  return vma_allocation_;
}

VkExtent3D Image::GetExtent() const {
  return extent_;
}

VkImageLayout Image::GetLayout() const {
  return layout_;
}

Image::~Image() {
  if (!Platform::Initialized())
    return;
  if ((vk_image_ != VK_NULL_HANDLE || vma_allocation_ != VK_NULL_HANDLE) &&
      Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vmaDestroyImage(Platform::GetVmaAllocator(), vk_image_, vma_allocation_);
    vk_image_ = VK_NULL_HANDLE;
    vma_allocation_ = VK_NULL_HANDLE;
    vma_allocation_info_ = {};
  }
}

void Image::TransitImageLayout(VkCommandBuffer vk_command_buffer, const VkImageLayout new_layout) {
  // if (newLayout == layout_) return;
  Platform::TransitImageLayout(vk_command_buffer, vk_image_, format_, array_layers_, layout_, new_layout, mip_levels_);
  layout_ = new_layout;
}

void Image::TransitImageLayout(const VkCommandBuffer vk_command_buffer, const VkImageLayout old_layout,
                               const VkImageLayout new_layout, const uint32_t src_queue_family_index,
                               const uint32_t dst_queue_family_index, const bool update_tracked_layout) {
  Platform::TransitImageLayout(vk_command_buffer, vk_image_, format_, array_layers_, old_layout, new_layout,
                               mip_levels_, src_queue_family_index, dst_queue_family_index, !update_tracked_layout);
  if (update_tracked_layout) {
    layout_ = new_layout;
  }
}

const VmaAllocationInfo& Image::GetVmaAllocationInfo() const {
  return vma_allocation_info_;
}

VkMemoryRequirements Image::GetMemoryRequirements() const {
  VkMemoryRequirements memory_requirements{};
  vkGetImageMemoryRequirements(Platform::GetVkDevice(), vk_image_, &memory_requirements);
  return memory_requirements;
}

Sampler::Sampler(const VkSamplerCreateInfo& sampler_create_info) {
  if (!Platform::Initialized())
    return;
  Platform::CheckVk(vkCreateSampler(Platform::GetVkDevice(), &sampler_create_info, nullptr, &vk_sampler_));
}

Sampler::~Sampler() {
  if (!Platform::Initialized())
    return;
  if (vk_sampler_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroySampler(Platform::GetVkDevice(), vk_sampler_, nullptr);
    vk_sampler_ = VK_NULL_HANDLE;
  }
}

VkSampler Sampler::GetVkSampler() const {
  return vk_sampler_;
}

struct Buffer::GpuState {
  VkBuffer vk_buffer = VK_NULL_HANDLE;
  VmaAllocation vma_allocation = VK_NULL_HANDLE;
  VmaAllocationInfo vma_allocation_info = {};

  VkBufferCreateFlags flags = {};
  VkDeviceSize size = {};
  VkBufferUsageFlags usage = {};
  VkSharingMode sharing_mode = {};
  std::vector<uint32_t> queue_family_indices = {};
  VmaAllocationCreateInfo vma_allocation_create_info = {};
  mutable std::mutex pending_gpu_work_mutex;
  mutable std::vector<GpuWorkHandle> pending_gpu_work;
};

void Buffer::UploadDataOnGpuThread(const std::shared_ptr<GpuState>& state, const size_t size, const void* src,
                                   const VkDeviceSize dst_offset) {
  const auto required_size = dst_offset + size;
  if (required_size > state->size && dst_offset != 0) {
    throw std::runtime_error("Subrange buffer upload cannot grow the destination buffer.");
  }
  if (required_size > state->size) {
    ResizeOnGpuThread(state, required_size);
  }
  if (state->vma_allocation_create_info.flags & VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT ||
      state->vma_allocation_create_info.flags & VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT) {
    void* mapping;
    Platform::CheckVk(vmaMapMemory(Platform::GetVmaAllocator(), state->vma_allocation, &mapping));
    memcpy(static_cast<char*>(mapping) + dst_offset, src, size);
    vmaUnmapMemory(Platform::GetVmaAllocator(), state->vma_allocation);
  } else {
    auto& gpu_service = Platform::GetGpuService();
    auto staging_buffer = gpu_service.AcquireStagingBuffer(size, false);
    try {
      const bool timing_enabled = Platform::GpuTimestampCaptureEnabled();
      const auto staging_copy_started = std::chrono::steady_clock::now();
      void* mapping;
      Platform::CheckVk(vmaMapMemory(Platform::GetVmaAllocator(), staging_buffer.vma_allocation, &mapping));
      memcpy(mapping, src, size);
      vmaUnmapMemory(Platform::GetVmaAllocator(), staging_buffer.vma_allocation);
      if (timing_enabled) {
        Platform::RecordCpuTimingSample(
            "Buffer Upload / Staging Copy",
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - staging_copy_started).count());
      }
      CopyFromBufferOnGpuThread(state, staging_buffer.vk_buffer, size, 0, dst_offset);
      gpu_service.ReleaseStagingBuffer(staging_buffer);
    } catch (...) {
      gpu_service.ReleaseStagingBuffer(staging_buffer);
      throw;
    }
  }
}

void Buffer::UploadData(const size_t size, const void* src) {
  const bool timing_enabled = Platform::GpuTimestampCaptureEnabled();
  const auto started = std::chrono::steady_clock::now();
  const auto handle = UploadDataAsync(size, src);
  Platform::GetGpuService().Wait(handle);
  if (timing_enabled && size != 0) {
    Platform::RecordCpuTimingSample(
        "Buffer Upload / Synchronous Operation",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count());
  }
}

void Buffer::UploadSubData(const size_t size, const void* src, const VkDeviceSize dst_offset) {
  const bool timing_enabled = Platform::GpuTimestampCaptureEnabled();
  const auto started = std::chrono::steady_clock::now();
  const auto handle = UploadSubDataAsync(size, src, dst_offset);
  Platform::GetGpuService().Wait(handle);
  if (timing_enabled && size != 0) {
    Platform::RecordCpuTimingSample(
        "Buffer Upload / Synchronous Operation",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count());
  }
}

GpuWorkHandle Buffer::UploadDataAsync(const size_t size, const void* src) {
  return UploadSubDataAsync(size, src, 0);
}

GpuWorkHandle Buffer::UploadSubDataAsync(const size_t size, const void* src, const VkDeviceSize dst_offset) {
  if (size == 0) {
    return {};
  }
  if (src == nullptr) {
    throw std::invalid_argument("Buffer upload source cannot be null.");
  }
  auto owned_data = std::make_shared<std::vector<std::byte>>(size);
  const bool timing_enabled = Platform::GpuTimestampCaptureEnabled();
  const auto source_copy_started = std::chrono::steady_clock::now();
  memcpy(owned_data->data(), src, size);
  if (timing_enabled) {
    Platform::RecordCpuTimingSample(
        "Buffer Upload / Source Copy",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - source_copy_started).count());
  }

  GpuWorkOptions options;
  options.debug_name = dst_offset == 0 ? "Buffer::UploadDataAsync" : "Buffer::UploadSubDataAsync";
  const auto state = gpu_state_;
  auto& gpu_service = Platform::GetGpuService();
  const auto handle = gpu_service.EnqueueStaging(size, options, [state, size, owned_data, dst_offset]() {
    UploadDataOnGpuThread(state, size, owned_data->data(), dst_offset);
  });
  TrackPendingGpuWork(handle);
  return handle;
}

void Buffer::DownloadDataOnGpuThread(const std::shared_ptr<GpuState>& state, const size_t size, void* dst) {
  if (size > state->size)
    ResizeOnGpuThread(state, size);
  if (state->vma_allocation_create_info.flags & VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT ||
      state->vma_allocation_create_info.flags & VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT) {
    void* mapping;
    Platform::CheckVk(vmaMapMemory(Platform::GetVmaAllocator(), state->vma_allocation, &mapping));
    memcpy(dst, mapping, size);
    vmaUnmapMemory(Platform::GetVmaAllocator(), state->vma_allocation);
  } else {
    auto& gpu_service = Platform::GetGpuService();
    auto staging_buffer = gpu_service.AcquireStagingBuffer(size, true);
    try {
      gpu_service.SubmitImmediate([&](const VkCommandBuffer vk_command_buffer) {
        VkBufferCopy copy_region{};
        copy_region.size = size;
        copy_region.srcOffset = 0;
        copy_region.dstOffset = 0;
        vkCmdCopyBuffer(vk_command_buffer, state->vk_buffer, staging_buffer.vk_buffer, 1, &copy_region);
      });
      void* mapping;
      Platform::CheckVk(vmaMapMemory(Platform::GetVmaAllocator(), staging_buffer.vma_allocation, &mapping));
      memcpy(dst, mapping, size);
      vmaUnmapMemory(Platform::GetVmaAllocator(), staging_buffer.vma_allocation);
      gpu_service.ReleaseStagingBuffer(staging_buffer);
    } catch (...) {
      gpu_service.ReleaseStagingBuffer(staging_buffer);
      throw;
    }
  }
}

void Buffer::DownloadData(const size_t size, void* dst) {
  if (size == 0) {
    return;
  }
  if (dst == nullptr) {
    throw std::invalid_argument("Buffer download destination cannot be null.");
  }
  const auto future = DownloadDataAsync(size);
  const auto bytes = future.get();
  memcpy(dst, bytes.data(), bytes.size());
}

std::shared_future<std::vector<std::byte>> Buffer::DownloadDataAsync(const size_t size) {
  auto promise = std::make_shared<std::promise<std::vector<std::byte>>>();
  auto future = promise->get_future().share();
  if (size == 0) {
    promise->set_value({});
    return future;
  }

  GpuWorkOptions options;
  options.debug_name = "Buffer::DownloadDataAsync";
  const auto state = gpu_state_;
  auto& gpu_service = Platform::GetGpuService();
  const auto handle = gpu_service.EnqueueStaging(size, options, [state, size, promise]() {
    try {
      std::vector<std::byte> bytes(size);
      DownloadDataOnGpuThread(state, size, bytes.data());
      promise->set_value(std::move(bytes));
    } catch (...) {
      promise->set_exception(std::current_exception());
      throw;
    }
  });
  TrackPendingGpuWork(handle);
  return future;
}

void Buffer::Allocate(VkBufferCreateInfo buffer_create_info,
                      const VmaAllocationCreateInfo& vma_allocation_create_info) {
  std::vector<uint32_t> queue_family_indices;
  if (buffer_create_info.queueFamilyIndexCount > 0 && buffer_create_info.pQueueFamilyIndices != nullptr) {
    queue_family_indices.assign(buffer_create_info.pQueueFamilyIndices,
                                buffer_create_info.pQueueFamilyIndices + buffer_create_info.queueFamilyIndexCount);
  }

  const auto state = gpu_state_;
  const auto allocate = [state, buffer_create_info, vma_allocation_create_info, queue_family_indices]() {
    auto resolved_buffer_create_info = buffer_create_info;
    resolved_buffer_create_info.pQueueFamilyIndices =
        queue_family_indices.empty() ? nullptr : queue_family_indices.data();
    AllocateOnGpuThread(state, resolved_buffer_create_info, vma_allocation_create_info);
  };

  if (const auto gpu_service = Platform::TryGetGpuService();
      gpu_service && gpu_service->Initialized() && !gpu_service->IsGpuThread()) {
    GpuWorkOptions options;
    options.debug_name = "Buffer::Allocate";
    const auto handle = gpu_service->Enqueue(options, allocate);
    gpu_service->Wait(handle);
  } else {
    allocate();
  }
}

void Buffer::AllocateOnGpuThread(const std::shared_ptr<GpuState>& state, VkBufferCreateInfo buffer_create_info,
                                 const VmaAllocationCreateInfo& vma_allocation_create_info) {
  if (Platform::CheckVk(vmaCreateBuffer(Platform::GetVmaAllocator(), &buffer_create_info, &vma_allocation_create_info,
                                        &state->vk_buffer, &state->vma_allocation, &state->vma_allocation_info))) {
    throw std::runtime_error("Failed to create buffer!");
  }
  assert(buffer_create_info.usage != 0);
  state->flags = buffer_create_info.flags;
  state->size = buffer_create_info.size;
  state->usage = buffer_create_info.usage;
  state->sharing_mode = buffer_create_info.sharingMode;
  ApplyVector(state->queue_family_indices, buffer_create_info.queueFamilyIndexCount,
              buffer_create_info.pQueueFamilyIndices);
  state->vma_allocation_create_info = vma_allocation_create_info;
}

Buffer::Buffer(const size_t staging_buffer_size, bool random_access) : gpu_state_(std::make_shared<GpuState>()) {
  if (!Platform::Initialized())
    return;
  VkBufferCreateInfo staging_buffer_create_info{};
  staging_buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  staging_buffer_create_info.size = staging_buffer_size;
  staging_buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  staging_buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo staging_buffer_vma_allocation_create_info{};
  staging_buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO;
  staging_buffer_vma_allocation_create_info.flags = random_access
                                                        ? VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT
                                                        : VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
  Allocate(staging_buffer_create_info, staging_buffer_vma_allocation_create_info);
}

Buffer::Buffer(const VkBufferCreateInfo& buffer_create_info) : gpu_state_(std::make_shared<GpuState>()) {
  if (!Platform::Initialized())
    return;
  VmaAllocationCreateInfo alloc_info = {};
  alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
  Allocate(buffer_create_info, alloc_info);
}

Buffer::Buffer(const VkBufferCreateInfo& buffer_create_info, const VmaAllocationCreateInfo& vma_allocation_create_info)
    : gpu_state_(std::make_shared<GpuState>()) {
  if (!Platform::Initialized())
    return;
  Allocate(buffer_create_info, vma_allocation_create_info);
}

void Buffer::ResizeOnGpuThread(const std::shared_ptr<GpuState>& state, const VkDeviceSize new_size) {
  if (new_size == state->size)
    return;
  if ((state->vk_buffer != VK_NULL_HANDLE || state->vma_allocation != VK_NULL_HANDLE) &&
      Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vmaDestroyBuffer(Platform::GetVmaAllocator(), state->vk_buffer, state->vma_allocation);
    state->vk_buffer = VK_NULL_HANDLE;
    state->vma_allocation = VK_NULL_HANDLE;
    state->vma_allocation_info = {};
  }
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.flags = state->flags;
  buffer_create_info.size = new_size;
  buffer_create_info.usage = state->usage;
  buffer_create_info.sharingMode = state->sharing_mode;
  buffer_create_info.queueFamilyIndexCount = state->queue_family_indices.size();
  buffer_create_info.pQueueFamilyIndices = state->queue_family_indices.data();
  if (Platform::CheckVk(vmaCreateBuffer(Platform::GetVmaAllocator(), &buffer_create_info,
                                        &state->vma_allocation_create_info, &state->vk_buffer, &state->vma_allocation,
                                        &state->vma_allocation_info))) {
    throw std::runtime_error("Failed to create buffer!");
  }
  state->size = new_size;
}

void Buffer::Resize(const VkDeviceSize new_size) {
  WaitForPendingGpuWork();
  const auto state = gpu_state_;
  if (const auto gpu_service = Platform::TryGetGpuService();
      gpu_service && gpu_service->Initialized() && !gpu_service->IsGpuThread()) {
    GpuWorkOptions options;
    options.debug_name = "Buffer::Resize";
    const auto handle = gpu_service->Enqueue(options, [state, new_size]() {
      ResizeOnGpuThread(state, new_size);
    });
    gpu_service->Wait(handle);
  } else {
    ResizeOnGpuThread(state, new_size);
  }
}

void Buffer::DestroyOnGpuThread(const std::shared_ptr<GpuState>& state) {
  if (!Platform::Initialized())
    return;
  if ((state->vk_buffer != VK_NULL_HANDLE || state->vma_allocation != VK_NULL_HANDLE) &&
      Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vmaDestroyBuffer(Platform::GetVmaAllocator(), state->vk_buffer, state->vma_allocation);
    state->vk_buffer = VK_NULL_HANDLE;
    state->vma_allocation = VK_NULL_HANDLE;
    state->vma_allocation_info = {};
  }
}

Buffer::~Buffer() {
  try {
    WaitForPendingGpuWork();
    const auto state = gpu_state_;
    if (const auto gpu_service = Platform::TryGetGpuService();
        gpu_service && gpu_service->Initialized() && !gpu_service->IsGpuThread()) {
      GpuWorkOptions options;
      options.debug_name = "Buffer::~Buffer";
      const auto handle = gpu_service->Enqueue(options, [state]() {
        DestroyOnGpuThread(state);
      });
      gpu_service->Wait(handle);
    } else {
      DestroyOnGpuThread(state);
    }
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to destroy GPU buffer: " + std::string(e.what()))
  } catch (...) {
    EVOENGINE_ERROR("Failed to destroy GPU buffer.")
  }
}

void Buffer::CopyFromBufferOnGpuThread(const std::shared_ptr<GpuState>& state,
                                       const std::shared_ptr<GpuState>& src_state, const VkDeviceSize size,
                                       const VkDeviceSize src_offset, const VkDeviceSize dst_offset) {
  CopyFromBufferOnGpuThread(state, src_state->vk_buffer, size, src_offset, dst_offset);
}

void Buffer::CopyFromBufferOnGpuThread(const std::shared_ptr<GpuState>& state, const VkBuffer src_buffer,
                                       const VkDeviceSize size, const VkDeviceSize src_offset,
                                       const VkDeviceSize dst_offset) {
  const auto required_size = dst_offset + size;
  if (required_size > state->size && dst_offset != 0) {
    throw std::runtime_error("Subrange buffer copy cannot grow the destination buffer.");
  }
  if (required_size > state->size) {
    ResizeOnGpuThread(state, required_size);
  }
  Platform::GetGpuService().SubmitImmediate([&](const VkCommandBuffer vk_command_buffer) {
    VkBufferCopy copy_region{};
    copy_region.size = size;
    copy_region.srcOffset = src_offset;
    copy_region.dstOffset = dst_offset;
    vkCmdCopyBuffer(vk_command_buffer, src_buffer, state->vk_buffer, 1, &copy_region);

    VkBufferMemoryBarrier2 upload_barrier{};
    upload_barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
    upload_barrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    upload_barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    upload_barrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    upload_barrier.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
    upload_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    upload_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    upload_barrier.buffer = state->vk_buffer;
    upload_barrier.offset = dst_offset;
    upload_barrier.size = size;

    VkDependencyInfo dependency_info{};
    dependency_info.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    dependency_info.bufferMemoryBarrierCount = 1;
    dependency_info.pBufferMemoryBarriers = &upload_barrier;
    vkCmdPipelineBarrier2(vk_command_buffer, &dependency_info);
  });
}

void Buffer::CopyFromBuffer(const Buffer& src_buffer, const VkDeviceSize size, const VkDeviceSize src_offset,
                            const VkDeviceSize dst_offset) {
  WaitForPendingGpuWork();
  src_buffer.WaitForPendingGpuWork();
  const auto state = gpu_state_;
  const auto src_state = src_buffer.gpu_state_;
  GpuWorkOptions options;
  options.debug_name = "Buffer::CopyFromBuffer";
  auto& gpu_service = Platform::GetGpuService();
  const auto handle = gpu_service.Enqueue(options, [state, src_state, size, src_offset, dst_offset]() {
    CopyFromBufferOnGpuThread(state, src_state, size, src_offset, dst_offset);
  });
  TrackPendingGpuWork(handle);
  gpu_service.Wait(handle);
}

void Buffer::CopyFromImageOnGpuThread(const std::shared_ptr<GpuState>& state, Image& src_image,
                                      const VkBufferImageCopy& image_copy_info) {
  Platform::GetGpuService().SubmitImmediate([&](const VkCommandBuffer vk_command_buffer) {
    const auto tracked_layout = src_image.GetLayout();
    const auto prev_layout = tracked_layout == VK_IMAGE_LAYOUT_UNDEFINED ? VK_IMAGE_LAYOUT_GENERAL : tracked_layout;
    src_image.TransitImageLayout(vk_command_buffer, prev_layout, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                 VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, true);
    vkCmdCopyImageToBuffer(vk_command_buffer, src_image.GetVkImage(), src_image.GetLayout(), state->vk_buffer, 1,
                           &image_copy_info);
    src_image.TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, prev_layout,
                                 VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, true);
  });
}

void Buffer::CopyFromImage(Image& src_image, const VkBufferImageCopy& image_copy_info) const {
  WaitForPendingGpuWork();
  const auto state = gpu_state_;
  GpuWorkOptions options;
  options.debug_name = "Buffer::CopyFromImage";
  auto& gpu_service = Platform::GetGpuService();
  const auto handle = gpu_service.Enqueue(options, [state, &src_image, image_copy_info]() {
    CopyFromImageOnGpuThread(state, src_image, image_copy_info);
  });
  TrackPendingGpuWork(handle);
  gpu_service.Wait(handle);
}

void Buffer::CopyFromImage(Image& src_image, const VkDeviceSize pixel_size) {
  Resize(src_image.GetExtent().width * src_image.GetExtent().height * pixel_size);
  VkBufferImageCopy image_copy_info{};
  image_copy_info.bufferOffset = 0;
  image_copy_info.bufferRowLength = 0;
  image_copy_info.bufferImageHeight = 0;
  image_copy_info.imageSubresource.layerCount = 1;
  image_copy_info.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  image_copy_info.imageSubresource.baseArrayLayer = 0;
  image_copy_info.imageSubresource.mipLevel = 0;

  image_copy_info.imageExtent = src_image.GetExtent();
  image_copy_info.imageOffset.x = 0;
  image_copy_info.imageOffset.y = 0;
  image_copy_info.imageOffset.z = 0;
  CopyFromImage(src_image, image_copy_info);
}

void Buffer::CopyFromDepth(Image& src_image, const VkDeviceSize pixel_size) {
  Resize(src_image.GetExtent().width * src_image.GetExtent().height * pixel_size);
  VkBufferImageCopy image_copy_info{};
  image_copy_info.bufferOffset = 0;
  image_copy_info.bufferRowLength = 0;
  image_copy_info.bufferImageHeight = 0;
  image_copy_info.imageSubresource.layerCount = 1;
  image_copy_info.imageSubresource.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
  image_copy_info.imageSubresource.baseArrayLayer = 0;
  image_copy_info.imageSubresource.mipLevel = 0;

  image_copy_info.imageExtent = src_image.GetExtent();
  image_copy_info.imageOffset.x = 0;
  image_copy_info.imageOffset.y = 0;
  image_copy_info.imageOffset.z = 0;
  CopyFromImage(src_image, image_copy_info);
}

void Buffer::TrackPendingGpuWork(const GpuWorkHandle& handle) const {
  if (!handle.Valid()) {
    return;
  }
  std::lock_guard lock(gpu_state_->pending_gpu_work_mutex);
  gpu_state_->pending_gpu_work.emplace_back(handle);
}

void Buffer::WaitForPendingGpuWork() const {
  std::vector<GpuWorkHandle> pending_work;
  {
    std::lock_guard lock(gpu_state_->pending_gpu_work_mutex);
    pending_work.swap(gpu_state_->pending_gpu_work);
  }
  auto* gpu_service = Platform::TryGetGpuService();
  if (!Platform::Initialized() || !gpu_service) {
    return;
  }
  const auto lifecycle_state = gpu_service->GetLifecycleState();
  if (lifecycle_state == GpuService::LifecycleState::Uninitialized ||
      lifecycle_state == GpuService::LifecycleState::Stopped) {
    return;
  }

  const bool on_gpu_thread = gpu_service && gpu_service->IsGpuThread();
  std::vector<GpuWorkHandle> deferred_work;
  for (const auto& handle : pending_work) {
    if (!handle.Valid() || Jobs::IsCompleted(handle)) {
      continue;
    }
    if (on_gpu_thread) {
      deferred_work.emplace_back(handle);
      continue;
    }
    gpu_service->Wait(handle);
  }

  if (!deferred_work.empty()) {
    deferred_work.erase(std::remove_if(deferred_work.begin(), deferred_work.end(),
                                       [](const GpuWorkHandle& handle) {
                                         return !handle.Valid() || Jobs::IsCompleted(handle);
                                       }),
                        deferred_work.end());
    if (!deferred_work.empty()) {
      std::lock_guard lock(gpu_state_->pending_gpu_work_mutex);
      gpu_state_->pending_gpu_work.insert(gpu_state_->pending_gpu_work.end(), deferred_work.begin(),
                                          deferred_work.end());
    }
  }
}

void Buffer::Fill(const VkCommandBuffer vk_command_buffer, const VkDeviceSize offset, const VkDeviceSize size,
                  const uint32_t data) const {
  vkCmdFillBuffer(vk_command_buffer, gpu_state_->vk_buffer, offset, size, data);
}

void Buffer::BindVertex(const VkCommandBuffer vk_command_buffer, const uint32_t first_binding,
                        const VkDeviceSize offset) const {
  vkCmdBindVertexBuffers(vk_command_buffer, first_binding, 1, &gpu_state_->vk_buffer, &offset);
}

void Buffer::BindIndex(const VkCommandBuffer vk_command_buffer, const VkDeviceSize offset,
                       const VkIndexType index_type) const {
  vkCmdBindIndexBuffer(vk_command_buffer, gpu_state_->vk_buffer, offset, index_type);
}

const VkBuffer& Buffer::GetVkBuffer() const {
  return gpu_state_->vk_buffer;
}

VkDeviceSize Buffer::GetSize() const {
  return gpu_state_->size;
}

struct BufferUploadArena::State {
  struct Block {
    std::shared_ptr<Buffer> buffer;
    void* mapping = nullptr;
    VkDeviceSize capacity = 0;
    VkDeviceSize cursor = 0;
  };

  VkDeviceSize initial_block_size = 0;
  std::vector<Block> blocks;
  std::shared_ptr<FrameSubmissionState> submission;
};

BufferUploadArena::BufferUploadArena(const VkDeviceSize initial_block_size) : state_(std::make_unique<State>()) {
  if (initial_block_size == 0) {
    throw std::invalid_argument("Buffer upload arena block size cannot be zero.");
  }
  state_->initial_block_size = (initial_block_size + 3) & ~VkDeviceSize{3};
}

BufferUploadArena::~BufferUploadArena() {
  if (Platform::Initialized()) {
    for (auto& block : state_->blocks) {
      vmaUnmapMemory(Platform::GetVmaAllocator(), block.buffer->GetVmaAllocation());
    }
  }
}

VkDeviceSize BufferUploadArena::GetAllocationBytes() const {
  VkDeviceSize result = 0;
  for (const auto& block : state_->blocks)
    result += block.buffer->GetVmaAllocationInfo().size;
  return result;
}

void BufferUploadBatch::Add(const std::shared_ptr<Buffer>& destination, const void* source, const size_t size,
                            const VkDeviceSize destination_offset, const BufferUploadOptions& options) {
  if (size == 0) {
    return;
  }
  if (!destination) {
    throw std::invalid_argument("Buffer upload batch destination cannot be null.");
  }
  if (!source) {
    throw std::invalid_argument("Buffer upload batch source cannot be null.");
  }
  if ((size & 3u) != 0 || (destination_offset & 3u) != 0) {
    throw std::invalid_argument("Buffer upload batch sizes and offsets must be four-byte aligned.");
  }
  if (size > std::numeric_limits<VkDeviceSize>::max() - destination_offset) {
    throw std::overflow_error("Buffer upload batch destination range overflows.");
  }
  const auto required_size = destination_offset + size;
  const auto& state = destination->gpu_state_;
  if ((state->usage & VK_BUFFER_USAGE_TRANSFER_DST_BIT) == 0) {
    throw std::invalid_argument("Buffer upload batch destination must support transfer writes.");
  }
  if (options.capacity_policy == BufferUploadCapacityPolicy::RequireCapacity && required_size > state->size) {
    throw std::out_of_range("Buffer upload batch destination capacity is insufficient.");
  }
  const bool has_stage_override = options.destination_stage_mask != 0;
  const bool has_access_override = options.destination_access_mask != 0;
  if ((!has_stage_override && has_access_override) ||
      (options.usage == BufferUploadUsage::Custom && !has_stage_override)) {
    throw std::invalid_argument("Buffer upload batch custom synchronization requires a stage mask.");
  }
  for (const auto& entry : entries_) {
    if (entry.destination.get() != destination.get()) {
      continue;
    }
    const auto entry_end = entry.destination_offset + entry.size;
    if (destination_offset < entry_end && entry.destination_offset < required_size) {
      throw std::invalid_argument("Buffer upload batch destination ranges cannot overlap.");
    }
  }
  if (size > std::numeric_limits<VkDeviceSize>::max() - staging_size_) {
    throw std::overflow_error("Buffer upload batch staging size overflows.");
  }
  entries_.push_back({destination, source, size, destination_offset, staging_size_, options});
  staging_size_ += size;
}

size_t BufferUploadBatch::GetEntryCount() const {
  return entries_.size();
}

VkDeviceSize BufferUploadBatch::GetStagingSize() const {
  return staging_size_;
}

void BufferUploadBatch::RecordCopies(const VkCommandBuffer command_buffer, const VkBuffer staging_buffer,
                                     const VkDeviceSize staging_base, const std::vector<Entry>& entries) {
  std::vector<VkBufferMemoryBarrier2> barriers;
  barriers.reserve(entries.size());
  for (const auto& entry : entries) {
    const VkBufferCopy copy{staging_base + entry.staging_offset, entry.destination_offset, entry.size};
    vkCmdCopyBuffer(command_buffer, staging_buffer, entry.destination->gpu_state_->vk_buffer, 1, &copy);

    VkPipelineStageFlags2 destination_stage = entry.options.destination_stage_mask;
    VkAccessFlags2 destination_access = entry.options.destination_access_mask;
    if (destination_stage == 0) {
      switch (entry.options.usage) {
        case BufferUploadUsage::Uniform:
          destination_stage = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
          destination_access = VK_ACCESS_2_UNIFORM_READ_BIT;
          break;
        case BufferUploadUsage::StorageRead:
          destination_stage = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
          destination_access = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
          break;
        case BufferUploadUsage::StorageReadWrite:
          destination_stage = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
          destination_access = VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
          break;
        case BufferUploadUsage::Vertex:
          destination_stage = VK_PIPELINE_STAGE_2_VERTEX_INPUT_BIT;
          destination_access = VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT;
          break;
        case BufferUploadUsage::Index:
          destination_stage = VK_PIPELINE_STAGE_2_INDEX_INPUT_BIT;
          destination_access = VK_ACCESS_2_INDEX_READ_BIT;
          break;
        case BufferUploadUsage::Indirect:
          destination_stage = VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT;
          destination_access = VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT;
          break;
        case BufferUploadUsage::Custom:
          break;
      }
    }
    VkBufferMemoryBarrier2 barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    barrier.dstStageMask = destination_stage;
    barrier.dstAccessMask = destination_access;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.buffer = entry.destination->gpu_state_->vk_buffer;
    barrier.offset = entry.destination_offset;
    barrier.size = entry.size;
    barriers.push_back(barrier);
  }
  VkDependencyInfo dependency_info{};
  dependency_info.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependency_info.bufferMemoryBarrierCount = static_cast<uint32_t>(barriers.size());
  dependency_info.pBufferMemoryBarriers = barriers.data();
  vkCmdPipelineBarrier2(command_buffer, &dependency_info);
}

void BufferUploadBatch::SubmitImmediate() {
  if (entries_.empty()) {
    return;
  }
  const bool timing_enabled = Platform::GpuTimestampCaptureEnabled();
  const auto started = std::chrono::steady_clock::now();
  const auto entries = entries_;
  const auto staging_size = staging_size_;
  GpuWorkOptions work_options;
  work_options.debug_name = "BufferUploadBatch::SubmitImmediate";
  auto& gpu_service = Platform::GetGpuService();
  const auto handle = gpu_service.EnqueueStaging(staging_size, work_options, [entries, staging_size]() {
    for (const auto& entry : entries) {
      const auto required_size = entry.destination_offset + entry.size;
      if (required_size > entry.destination->gpu_state_->size) {
        Buffer::ResizeOnGpuThread(entry.destination->gpu_state_, required_size);
      }
    }

    auto& service = Platform::GetGpuService();
    auto staging_buffer = service.AcquireStagingBuffer(staging_size, false);
    try {
      const auto staging_copy_started = std::chrono::steady_clock::now();
      void* mapping = nullptr;
      Platform::CheckVk(vmaMapMemory(Platform::GetVmaAllocator(), staging_buffer.vma_allocation, &mapping));
      for (const auto& entry : entries) {
        memcpy(static_cast<std::byte*>(mapping) + entry.staging_offset, entry.source, entry.size);
      }
      vmaUnmapMemory(Platform::GetVmaAllocator(), staging_buffer.vma_allocation);
      if (Platform::GpuTimestampCaptureEnabled()) {
        Platform::RecordCpuTimingSample(
            "Buffer Upload Batch / Staging Copy",
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - staging_copy_started).count());
      }

      service.SubmitImmediate([&](const VkCommandBuffer command_buffer) {
        RecordCopies(command_buffer, staging_buffer.vk_buffer, 0, entries);
      });
      service.ReleaseStagingBuffer(staging_buffer);
    } catch (...) {
      service.ReleaseStagingBuffer(staging_buffer);
      throw;
    }
  });
  gpu_service.Wait(handle);
  entries_.clear();
  staging_size_ = 0;
  if (timing_enabled) {
    Platform::RecordCpuTimingSample(
        "Buffer Upload Batch / Synchronous Operation",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count());
  }
}

void BufferUploadBatch::Record(BufferUploadArena& arena) {
  if (entries_.empty()) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  for (const auto& entry : entries_) {
    const auto required_size = entry.destination_offset + entry.size;
    if (required_size > entry.destination->GetSize()) {
      entry.destination->Resize(required_size);
    }
  }

  auto& arena_state = *arena.state_;
  if (!arena_state.submission || arena_state.submission->status != FrameSubmissionState::Status::Pending) {
    for (auto& block : arena_state.blocks) {
      block.cursor = 0;
    }
    arena_state.submission = Platform::TrackCurrentFrameSubmission();
  }
  BufferUploadArena::State::Block* selected_block = nullptr;
  for (auto& block : arena_state.blocks) {
    if (staging_size_ <= block.capacity - block.cursor) {
      selected_block = &block;
      break;
    }
  }
  if (!selected_block) {
    VkDeviceSize capacity = arena_state.initial_block_size;
    while (capacity < staging_size_) {
      capacity *= 2;
    }
    auto buffer = std::make_shared<Buffer>(capacity, false);
    buffer->SetDebugName("Frame Buffer Upload Arena");
    void* mapping = nullptr;
    Platform::CheckVk(vmaMapMemory(Platform::GetVmaAllocator(), buffer->GetVmaAllocation(), &mapping));
    arena_state.blocks.push_back({std::move(buffer), mapping, capacity, 0});
    selected_block = &arena_state.blocks.back();
  }
  const auto staging_base = selected_block->cursor;
  const auto staging_copy_started = std::chrono::steady_clock::now();
  for (const auto& entry : entries_) {
    memcpy(static_cast<std::byte*>(selected_block->mapping) + staging_base + entry.staging_offset, entry.source,
           entry.size);
  }
  Platform::CheckVk(vmaFlushAllocation(Platform::GetVmaAllocator(), selected_block->buffer->GetVmaAllocation(),
                                       staging_base, staging_size_));
  selected_block->cursor += staging_size_;
  if (Platform::GpuTimestampCaptureEnabled()) {
    Platform::RecordCpuTimingSample(
        "Buffer Upload Batch / Frame Staging Copy",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - staging_copy_started).count());
  }
  const auto entries = entries_;
  const auto staging_buffer = selected_block->buffer;
  Platform::RecordCommandsMainQueue([entries, staging_buffer, staging_base](const VkCommandBuffer command_buffer) {
    RecordCopies(command_buffer, staging_buffer->GetVkBuffer(), staging_base, entries);
  });
  entries_.clear();
  staging_size_ = 0;
  if (Platform::GpuTimestampCaptureEnabled()) {
    Platform::RecordCpuTimingSample(
        "Buffer Upload Batch / Frame Record",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count());
  }
}

VmaAllocation Buffer::GetVmaAllocation() const {
  return gpu_state_->vma_allocation;
}

VkDeviceAddress Buffer::GetDeviceAddress() const {
  VkBufferDeviceAddressInfo buffer_device_address_info{};
  buffer_device_address_info.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
  buffer_device_address_info.buffer = gpu_state_->vk_buffer;
  const VkDeviceAddress address = vkGetBufferDeviceAddress(Platform::GetVkDevice(), &buffer_device_address_info);
  assert(address != 0);
  return address;
}

const VmaAllocationInfo& Buffer::GetVmaAllocationInfo() const {
  return gpu_state_->vma_allocation_info;
}

void Buffer::SetDebugName(const std::string& name) const {
  vmaSetAllocationName(Platform::GetVmaAllocator(), gpu_state_->vma_allocation, name.c_str());
  // debug shader function
  if (vkSetDebugUtilsObjectNameEXT) {
    VkDebugUtilsObjectNameInfoEXT nameInfo{};
    nameInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT;
    nameInfo.objectType = VK_OBJECT_TYPE_BUFFER;
    nameInfo.objectHandle = reinterpret_cast<uint64_t>(gpu_state_->vk_buffer);
    nameInfo.pObjectName = name.c_str();
    vkSetDebugUtilsObjectNameEXT(Platform::GetVkDevice(), &nameInfo);
  }
}

DescriptorSetLayout::~DescriptorSetLayout() {
  if (!Platform::Initialized())
    return;
  if (vk_descriptor_set_layout_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyDescriptorSetLayout(Platform::GetVkDevice(), vk_descriptor_set_layout_, nullptr);
    vk_descriptor_set_layout_ = VK_NULL_HANDLE;
  }
}

void DescriptorSetLayout::PushDescriptorBinding(uint32_t binding_index, VkDescriptorType type,
                                                VkShaderStageFlags stage_flags, VkDescriptorBindingFlags binding_flags,
                                                const uint32_t descriptor_count) {
  DescriptorBinding binding;
  VkDescriptorSetLayoutBinding binding_info{};
  binding_info.binding = binding_index;
  binding_info.descriptorCount = descriptor_count;
  binding_info.descriptorType = type;
  binding_info.pImmutableSamplers = nullptr;
  binding_info.stageFlags = stage_flags;
  binding.binding = binding_info;
  binding.binding_flags = binding_flags;
  descriptor_set_layout_bindings_[binding_index] = binding;
}

const std::unordered_map<uint32_t, DescriptorBinding>& DescriptorSetLayout::GetDescriptorBindings() const {
  return descriptor_set_layout_bindings_;
}

void DescriptorSetLayout::Initialize() {
  if (!Platform::Initialized())
    return;
  if (vk_descriptor_set_layout_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyDescriptorSetLayout(Platform::GetVkDevice(), vk_descriptor_set_layout_, nullptr);
    vk_descriptor_set_layout_ = VK_NULL_HANDLE;
  }

  std::vector<VkDescriptorSetLayoutBinding> list_of_bindings;
  std::vector<VkDescriptorBindingFlags> list_of_binding_flags;
  for (const auto& binding : descriptor_set_layout_bindings_) {
    list_of_bindings.emplace_back(binding.second.binding);
    list_of_binding_flags.emplace_back(binding.second.binding_flags);
  }

  VkDescriptorSetLayoutBindingFlagsCreateInfoEXT extended_info{
      VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_BINDING_FLAGS_CREATE_INFO_EXT, nullptr};
  extended_info.bindingCount = static_cast<uint32_t>(list_of_binding_flags.size());
  extended_info.pBindingFlags = list_of_binding_flags.data();

  VkDescriptorSetLayoutCreateInfo descriptor_set_layout_create_info{};
  descriptor_set_layout_create_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptor_set_layout_create_info.bindingCount = static_cast<uint32_t>(list_of_bindings.size());
  descriptor_set_layout_create_info.pBindings = list_of_bindings.data();
  descriptor_set_layout_create_info.pNext = &extended_info;
  Platform::CheckVk(vkCreateDescriptorSetLayout(Platform::GetVkDevice(), &descriptor_set_layout_create_info, nullptr,
                                                &vk_descriptor_set_layout_));
}

const VkDescriptorSet& DescriptorSet::GetVkDescriptorSet() const {
  return descriptor_set_;
}

DescriptorSet::~DescriptorSet() {
  if (descriptor_set_ == VK_NULL_HANDLE)
    return;
  if (Platform::Initialized() && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    Platform::CheckVk(vkFreeDescriptorSets(Platform::GetVkDevice(),
                                           Platform::GetDescriptorPool()->GetVkDescriptorPool(), 1, &descriptor_set_));
  }
  descriptor_set_ = VK_NULL_HANDLE;
}

DescriptorSet::DescriptorSet(const std::shared_ptr<DescriptorSetLayout>& target_layout) {
  if (!Platform::Initialized())
    return;
  VkDescriptorSetAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  alloc_info.descriptorPool = Platform::GetDescriptorPool()->GetVkDescriptorPool();
  alloc_info.descriptorSetCount = 1;
  alloc_info.pSetLayouts = &target_layout->GetVkDescriptorSetLayout();

  if (Platform::CheckVk(vkAllocateDescriptorSets(Platform::GetVkDevice(), &alloc_info, &descriptor_set_)) !=
      VK_SUCCESS) {
    throw std::runtime_error("failed to allocate descriptor sets!");
  }
  descriptor_set_layout_ = target_layout;
}

void DescriptorSet::UpdateImageDescriptorBinding(const uint32_t binding_index, const VkDescriptorImageInfo& image_info,
                                                 uint32_t array_element) const {
  const auto& descriptor_binding = descriptor_set_layout_->descriptor_set_layout_bindings_[binding_index];
  if ((descriptor_binding.binding.descriptorType == VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER ||
       descriptor_binding.binding.descriptorType == VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE ||
       descriptor_binding.binding.descriptorType == VK_DESCRIPTOR_TYPE_STORAGE_IMAGE) &&
      image_info.imageView == VK_NULL_HANDLE) {
    throw std::runtime_error("Attempted to write a null image view to descriptor binding " +
                             std::to_string(binding_index) + ", array element " + std::to_string(array_element) + ".");
  }
  VkWriteDescriptorSet write_info{};
  write_info.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  write_info.dstSet = descriptor_set_;
  write_info.dstBinding = binding_index;
  write_info.dstArrayElement = array_element;
  write_info.descriptorType = descriptor_binding.binding.descriptorType;
  write_info.descriptorCount = 1;
  write_info.pImageInfo = &image_info;
  vkUpdateDescriptorSets(Platform::GetVkDevice(), 1, &write_info, 0, nullptr);
}

void DescriptorSet::UpdateAccelerationStructureDescriptorBinding(
    const uint32_t binding_index, const VkAccelerationStructureKHR& acceleration_structure) const {
  VkWriteDescriptorSetAccelerationStructureKHR descriptor_acceleration_structure_info{};
  descriptor_acceleration_structure_info.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR;
  descriptor_acceleration_structure_info.accelerationStructureCount = 1;
  descriptor_acceleration_structure_info.pAccelerationStructures = &acceleration_structure;

  VkWriteDescriptorSet acceleration_structure_write{};
  acceleration_structure_write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  acceleration_structure_write.dstSet = descriptor_set_;
  acceleration_structure_write.dstBinding = binding_index;
  acceleration_structure_write.descriptorCount = 1;
  acceleration_structure_write.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
  acceleration_structure_write.pNext = &descriptor_acceleration_structure_info;

  vkUpdateDescriptorSets(Platform::GetVkDevice(), 1, &acceleration_structure_write, 0, nullptr);
}

void DescriptorSet::UpdateAccelerationStructureDescriptorBinding(
    const uint32_t binding_index, const std::shared_ptr<TopLevelAccelerationStructure>& acceleration_structure) const {
  const auto as = acceleration_structure->GetVkAccelerationStructure();
  UpdateAccelerationStructureDescriptorBinding(binding_index, as);
}

void DescriptorSet::UpdateBufferDescriptorBinding(const uint32_t binding_index,
                                                  const VkDescriptorBufferInfo& buffer_info,
                                                  uint32_t array_element) const {
  const auto& descriptor_binding = descriptor_set_layout_->descriptor_set_layout_bindings_[binding_index];
  VkWriteDescriptorSet write_info{};
  write_info.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  write_info.dstSet = descriptor_set_;
  write_info.dstBinding = binding_index;
  write_info.dstArrayElement = array_element;
  write_info.descriptorType = descriptor_binding.binding.descriptorType;
  write_info.descriptorCount = 1;
  write_info.pBufferInfo = &buffer_info;
  vkUpdateDescriptorSets(Platform::GetVkDevice(), 1, &write_info, 0, nullptr);
}

void DescriptorSet::UpdateBufferDescriptorBinding(const uint32_t binding_index, const std::shared_ptr<Buffer>& buffer,
                                                  const uint32_t array_element) const {
  VkDescriptorBufferInfo buffer_info;
  buffer_info.offset = 0;
  buffer_info.range = VK_WHOLE_SIZE;
  buffer_info.buffer = buffer->GetVkBuffer();
  UpdateBufferDescriptorBinding(binding_index, buffer_info, array_element);
}

const VkDescriptorSetLayout& DescriptorSetLayout::GetVkDescriptorSetLayout() const {
  return vk_descriptor_set_layout_;
}

DescriptorPool::DescriptorPool(const VkDescriptorPoolCreateInfo& descriptor_pool_create_info) {
  if (!Platform::Initialized())
    return;
  Platform::CheckVk(
      vkCreateDescriptorPool(Platform::GetVkDevice(), &descriptor_pool_create_info, nullptr, &vk_descriptor_pool_));
}

DescriptorPool::~DescriptorPool() {
  if (!Platform::Initialized())
    return;
  if (vk_descriptor_pool_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyDescriptorPool(Platform::GetVkDevice(), vk_descriptor_pool_, nullptr);
    vk_descriptor_pool_ = VK_NULL_HANDLE;
  }
}

VkDescriptorPool DescriptorPool::GetVkDescriptorPool() const {
  return vk_descriptor_pool_;
}

ShaderExt::ShaderExt(const VkShaderCreateInfoEXT& shader_create_info_ext) {
  if (!Platform::Initialized())
    return;
  Platform::CheckVk(vkCreateShadersEXT(Platform::GetVkDevice(), 1, &shader_create_info_ext, nullptr, &shader_ext_));
  flags_ = shader_create_info_ext.flags;
  stage_ = shader_create_info_ext.stage;
  next_stage_ = shader_create_info_ext.nextStage;
  code_type_ = shader_create_info_ext.codeType;
  name_ = shader_create_info_ext.pName;
  ApplyVector(set_layouts_, shader_create_info_ext.setLayoutCount, shader_create_info_ext.pSetLayouts);
  ApplyVector(push_constant_ranges_, shader_create_info_ext.pushConstantRangeCount,
              shader_create_info_ext.pPushConstantRanges);
  if (shader_create_info_ext.pSpecializationInfo)
    specialization_info_ = *shader_create_info_ext.pSpecializationInfo;
}

ShaderExt::~ShaderExt() {
  if (!Platform::Initialized())
    return;
  if (shader_ext_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkDestroyShaderEXT(Platform::GetVkDevice(), shader_ext_, nullptr);
    shader_ext_ = VK_NULL_HANDLE;
  }
}

const VkShaderEXT& ShaderExt::GetVkShaderExt() const {
  return shader_ext_;
}

CommandBufferStatus CommandBuffer::GetStatus() const {
  return status_;
}

CommandBuffer::CommandBuffer(const VkCommandBufferLevel& buffer_level)
    : CommandBuffer(Platform::Initialized() ? Platform::GetVkCommandPool() : VK_NULL_HANDLE, buffer_level) {
}

CommandBuffer::CommandBuffer(const VkCommandPool command_pool, const VkCommandBufferLevel& buffer_level) {
  if (!Platform::Initialized() || command_pool == VK_NULL_HANDLE)
    return;
  vk_command_pool_ = command_pool;
  VkCommandBufferAllocateInfo command_buffer_allocate_info = {};
  command_buffer_allocate_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  command_buffer_allocate_info.commandPool = vk_command_pool_;
  command_buffer_allocate_info.level = buffer_level;
  command_buffer_allocate_info.commandBufferCount = 1;
  Platform::CheckVk(
      vkAllocateCommandBuffers(Platform::GetVkDevice(), &command_buffer_allocate_info, &vk_command_buffer_));
  status_ = CommandBufferStatus::Ready;
}

CommandBuffer::~CommandBuffer() {
  if (!Platform::Initialized())
    return;
  if (vk_command_buffer_ != VK_NULL_HANDLE && Platform::GetVkInstance() != VK_NULL_HANDLE) {
    vkFreeCommandBuffers(Platform::GetVkDevice(), vk_command_pool_, 1, &vk_command_buffer_);
    vk_command_buffer_ = VK_NULL_HANDLE;
  }
  status_ = CommandBufferStatus::Invalid;
}

const VkCommandBuffer& CommandBuffer::GetVkCommandBuffer() const {
  return vk_command_buffer_;
}

void CommandBuffer::Begin(const VkCommandBufferUsageFlags& usage) {
  if (status_ == CommandBufferStatus::Invalid) {
    EVOENGINE_ERROR("Command buffer invalid!")
    return;
  }
  if (status_ != CommandBufferStatus::Ready) {
    EVOENGINE_ERROR("Command buffer not ready!")
    return;
  }
  VkCommandBufferBeginInfo begin_info = {};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  begin_info.flags = usage;
  Platform::CheckVk(vkBeginCommandBuffer(vk_command_buffer_, &begin_info));
  status_ = CommandBufferStatus::Recording;
}

void CommandBuffer::End() {
  if (status_ == CommandBufferStatus::Invalid) {
    EVOENGINE_ERROR("Command buffer invalid!")
    return;
  }
  if (status_ != CommandBufferStatus::Recording) {
    EVOENGINE_ERROR("Command buffer not recording!")
    return;
  }
  Platform::CheckVk(vkEndCommandBuffer(vk_command_buffer_));
  status_ = CommandBufferStatus::Recorded;
}

bool CommandBuffer::Record(const std::function<void(VkCommandBuffer vk_command_buffer)>& commands) {
  Begin();
  if (status_ != CommandBufferStatus::Recording) {
    return false;
  }
  commands(vk_command_buffer_);
  End();
  return status_ == CommandBufferStatus::Recorded;
}

void CommandBuffer::Reset() {
  if (status_ == CommandBufferStatus::Invalid) {
    EVOENGINE_ERROR("Command buffer invalid!");
    return;
  }
  Platform::CheckVk(vkResetCommandBuffer(vk_command_buffer_, 0));
  status_ = CommandBufferStatus::Ready;
}

void CommandQueue::Submit(
    const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers, uint32_t offset, uint32_t buffer_count,
    const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
    const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores, const std::shared_ptr<Fence>& fence) const {
  VkSubmitInfo submit_info{};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  //===========
  submit_info.waitSemaphoreCount = wait_semaphores.size();
  std::vector<VkSemaphore> wait_vk_semaphores(wait_semaphores.size());
  std::vector<VkPipelineStageFlags> wait_vk_wait_stage_flags(wait_semaphores.size());
  for (uint32_t i = 0; i < wait_vk_semaphores.size(); i++) {
    wait_vk_semaphores[i] = wait_semaphores[i].first->GetVkSemaphore();
    wait_vk_wait_stage_flags[i] = wait_semaphores[i].second;
  }
  submit_info.pWaitSemaphores = wait_vk_semaphores.data();
  submit_info.pWaitDstStageMask = wait_vk_wait_stage_flags.data();

  //===========
  submit_info.signalSemaphoreCount = signal_semaphores.size();
  std::vector<VkSemaphore> signal_vk_semaphores(signal_semaphores.size());
  for (uint32_t i = 0; i < signal_vk_semaphores.size(); i++) {
    signal_vk_semaphores[i] = signal_semaphores[i]->GetVkSemaphore();
  }
  submit_info.pSignalSemaphores = signal_vk_semaphores.data();

  //===========
  submit_info.commandBufferCount = buffer_count;
  std::vector<VkCommandBuffer> vk_command_buffers(buffer_count);

  for (uint32_t i = 0; i < buffer_count; ++i) {
    vk_command_buffers[i] = command_buffers[i + offset]->GetVkCommandBuffer();
  }

  submit_info.pCommandBuffers = vk_command_buffers.data();

  const std::lock_guard queue_lock(Platform::GetQueueHostMutex());
  if (Platform::CheckVk(vkQueueSubmit(vk_queue_, 1, &submit_info, fence->GetVkFence())) != VK_SUCCESS) {
    throw std::runtime_error("Failed to submit command buffer!");
  }
}

void CommandQueue::Submit(
    const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
    const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
    const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores, const std::shared_ptr<Fence>& fence) const {
  VkSubmitInfo submit_info{};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  //===========
  submit_info.waitSemaphoreCount = wait_semaphores.size();
  std::vector<VkSemaphore> wait_vk_semaphores(wait_semaphores.size());
  std::vector<VkPipelineStageFlags> wait_vk_wait_stage_flags(wait_semaphores.size());
  for (uint32_t i = 0; i < wait_vk_semaphores.size(); i++) {
    wait_vk_semaphores[i] = wait_semaphores[i].first->GetVkSemaphore();
    wait_vk_wait_stage_flags[i] = wait_semaphores[i].second;
  }
  submit_info.pWaitSemaphores = wait_vk_semaphores.data();
  submit_info.pWaitDstStageMask = wait_vk_wait_stage_flags.data();

  //===========
  submit_info.signalSemaphoreCount = signal_semaphores.size();
  std::vector<VkSemaphore> signal_vk_semaphores(signal_semaphores.size());
  for (uint32_t i = 0; i < signal_vk_semaphores.size(); i++) {
    signal_vk_semaphores[i] = signal_semaphores[i]->GetVkSemaphore();
  }
  submit_info.pSignalSemaphores = signal_vk_semaphores.data();

  //===========
  submit_info.commandBufferCount = command_buffers.size();
  std::vector<VkCommandBuffer> vk_command_buffers(command_buffers.size());
  for (size_t i = 0; i < command_buffers.size(); ++i) {
    vk_command_buffers[i] = command_buffers[i]->GetVkCommandBuffer();
  }
  submit_info.pCommandBuffers = vk_command_buffers.data();

  const std::lock_guard queue_lock(Platform::GetQueueHostMutex());
  if (Platform::CheckVk(vkQueueSubmit(vk_queue_, 1, &submit_info, fence->GetVkFence())) != VK_SUCCESS) {
    throw std::runtime_error("Failed to submit command buffer!");
  }
}

void CommandQueue::Submit(
    const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
    const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
    const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores) const {
  VkSubmitInfo submit_info{};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  //===========
  submit_info.waitSemaphoreCount = wait_semaphores.size();
  std::vector<VkSemaphore> wait_vk_semaphores(wait_semaphores.size());
  std::vector<VkPipelineStageFlags> wait_vk_wait_stage_flags(wait_semaphores.size());
  for (uint32_t i = 0; i < wait_vk_semaphores.size(); i++) {
    wait_vk_semaphores[i] = wait_semaphores[i].first->GetVkSemaphore();
    wait_vk_wait_stage_flags[i] = wait_semaphores[i].second;
  }
  submit_info.pWaitSemaphores = wait_vk_semaphores.data();
  submit_info.pWaitDstStageMask = wait_vk_wait_stage_flags.data();

  //===========
  submit_info.signalSemaphoreCount = signal_semaphores.size();
  std::vector<VkSemaphore> signal_vk_semaphores(signal_semaphores.size());
  for (uint32_t i = 0; i < signal_vk_semaphores.size(); i++) {
    signal_vk_semaphores[i] = signal_semaphores[i]->GetVkSemaphore();
  }
  submit_info.pSignalSemaphores = signal_vk_semaphores.data();

  //===========
  submit_info.commandBufferCount = command_buffers.size();
  std::vector<VkCommandBuffer> vk_command_buffers(command_buffers.size());
  for (uint32_t i = 0; i < command_buffers.size(); i++) {
    vk_command_buffers[i] = command_buffers[i]->GetVkCommandBuffer();
  }
  submit_info.pCommandBuffers = vk_command_buffers.data();

  const std::lock_guard queue_lock(Platform::GetQueueHostMutex());
  if (Platform::CheckVk(vkQueueSubmit(vk_queue_, 1, &submit_info, VK_NULL_HANDLE)) != VK_SUCCESS) {
    throw std::runtime_error("Failed to submit command buffer!");
  }
}

void CommandQueue::ImmediateSubmit(
    const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
    const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
    const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores) const {
  Submit(command_buffers, wait_semaphores, signal_semaphores);
  WaitIdle();
}

void CommandQueue::Present(const std::vector<std::shared_ptr<Semaphore>>& wait_semaphores,
                           const std::vector<std::pair<std::shared_ptr<Swapchain>, uint32_t>>& targets) const {
  VkPresentInfoKHR present_info{};
  present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
  std::vector<VkSemaphore> wait_vk_semaphores(wait_semaphores.size());
  for (uint32_t i = 0; i < wait_vk_semaphores.size(); i++) {
    wait_vk_semaphores[i] = wait_semaphores[i]->GetVkSemaphore();
  }
  present_info.waitSemaphoreCount = wait_vk_semaphores.size();
  present_info.pWaitSemaphores = wait_vk_semaphores.data();

  //===========
  present_info.swapchainCount = targets.size();
  std::vector<VkSwapchainKHR> vk_swapchain_khrs(targets.size());
  std::vector<uint32_t> image_indices(targets.size());
  for (uint32_t i = 0; i < targets.size(); i++) {
    vk_swapchain_khrs[i] = targets[i].first->GetVkSwapchain();
    image_indices[i] = targets[i].second;
  }
  present_info.pSwapchains = vk_swapchain_khrs.data();
  present_info.pImageIndices = image_indices.data();

  const std::lock_guard queue_lock(Platform::GetQueueHostMutex());
  vkQueuePresentKHR(vk_queue_, &present_info);
}

void CommandQueue::WaitIdle() const {
  const std::lock_guard queue_lock(Platform::GetQueueHostMutex());
  Platform::CheckVk(vkQueueWaitIdle(vk_queue_));
}

VkQueue CommandQueue::GetVkQueue() const {
  return vk_queue_;
}

BottomLevelAccelerationStructure::BottomLevelAccelerationStructure(const uint32_t vertex_count,
                                                                   const uint32_t primitive_count)
    : vertex_count_(vertex_count), primitive_count_(primitive_count) {
}

std::shared_ptr<BottomLevelAccelerationStructure> BottomLevelAccelerationStructure::CreateStatic(
    const std::shared_ptr<RangeDescriptor>& meshlet_range, const std::shared_ptr<RangeDescriptor>& triangle_range,
    const std::vector<Vertex>& vertices) {
  if (!meshlet_range || !triangle_range) {
    throw std::invalid_argument("Static BLAS geometry ranges cannot be null.");
  }
  if (!vertices.empty() && !HasFiniteBlasPositions(vertices)) {
    throw std::runtime_error("BLAS vertices contain a non-finite position.");
  }
  auto result = std::shared_ptr<BottomLevelAccelerationStructure>(
      new BottomLevelAccelerationStructure(static_cast<uint32_t>(vertices.size()), triangle_range->range));
  if (!Platform::Initialized() || triangle_range->range == 0) {
    return result;
  }
  auto& state = GetStaticBlasBuilderState();
  std::lock_guard lock(state.mutex);
  state.pending.emplace_back(PendingStaticBlasBuild{result, meshlet_range, triangle_range, meshlet_range->offset,
                                                    meshlet_range->range, triangle_range->offset,
                                                    triangle_range->range});
  return result;
}

std::shared_ptr<BottomLevelAccelerationStructure> BottomLevelAccelerationStructure::CreateLinearSweptSpheres(
    const std::vector<StrandPoint>& points, const std::vector<uint32_t>& indices) {
  return std::shared_ptr<BottomLevelAccelerationStructure>(new BottomLevelAccelerationStructure(points, indices));
}

BottomLevelAccelerationStructure::BottomLevelAccelerationStructure(const std::vector<StrandPoint>& points,
                                                                   const std::vector<uint32_t>& indices)
    : vertex_count_(static_cast<uint32_t>(points.size())), primitive_count_(static_cast<uint32_t>(indices.size())) {
#ifdef VK_NV_ray_tracing_linear_swept_spheres
  if (!Platform::Initialized() || !Platform::RayTracingLinearSweptSpheresEnabled() || points.empty() ||
      indices.empty()) {
    return;
  }
  if (points.size() > (std::numeric_limits<uint32_t>::max)() ||
      indices.size() > (std::numeric_limits<uint32_t>::max)() || !HasValidLinearSweptSphereInput(points, indices)) {
    throw std::runtime_error("Linear swept sphere BLAS input is invalid.");
  }

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                             VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  buffer_create_info.size = points.size() * sizeof(StrandPoint);
  auto point_buffer = std::make_shared<Buffer>(buffer_create_info, allocation_create_info);
  point_buffer->UploadVector(points);
  point_buffer->SetDebugName("LSS BLAS Point Input");
  buffer_create_info.size = indices.size() * sizeof(uint32_t);
  auto lss_index_buffer = std::make_shared<Buffer>(buffer_create_info, allocation_create_info);
  lss_index_buffer->UploadVector(indices);
  lss_index_buffer->SetDebugName("LSS BLAS Index Input");

  VkAccelerationStructureGeometryLinearSweptSpheresDataNV lss_data{};
  lss_data.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_LINEAR_SWEPT_SPHERES_DATA_NV;
  lss_data.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
  lss_data.vertexData.deviceAddress = point_buffer->GetDeviceAddress() + offsetof(StrandPoint, position);
  lss_data.vertexStride = sizeof(StrandPoint);
  lss_data.radiusFormat = VK_FORMAT_R32_SFLOAT;
  lss_data.radiusData.deviceAddress = point_buffer->GetDeviceAddress() + offsetof(StrandPoint, thickness);
  lss_data.radiusStride = sizeof(StrandPoint);
  lss_data.indexType = VK_INDEX_TYPE_UINT32;
  lss_data.indexData.deviceAddress = lss_index_buffer->GetDeviceAddress();
  lss_data.indexStride = sizeof(uint32_t);
  lss_data.indexingMode = VK_RAY_TRACING_LSS_INDEXING_MODE_SUCCESSIVE_NV;
  lss_data.endCapsMode = VK_RAY_TRACING_LSS_PRIMITIVE_END_CAPS_MODE_CHAINED_NV;

  VkAccelerationStructureGeometryKHR geometry{};
  geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
  geometry.pNext = &lss_data;
  geometry.geometryType = VK_GEOMETRY_TYPE_LINEAR_SWEPT_SPHERES_NV;
  geometry.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;

  VkAccelerationStructureBuildGeometryInfoKHR size_info{};
  size_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
  size_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  size_info.flags = kBlasBuildFlags;
  size_info.geometryCount = 1;
  size_info.pGeometries = &geometry;
  VkAccelerationStructureBuildSizesInfoKHR build_sizes{};
  build_sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
  vkGetAccelerationStructureBuildSizesKHR(Platform::GetVkDevice(), VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                          &size_info, &primitive_count_, &build_sizes);

  acceleration_structure_buffer_ = CreateAccelerationStructureBuffer(build_sizes.accelerationStructureSize);
  vk_acceleration_structure_khr_ =
      CreateBottomLevelAccelerationStructure(acceleration_structure_buffer_, build_sizes.accelerationStructureSize);
  const auto scratch_alignment =
      std::max(1u, Platform::GetSelectedPhysicalDevice()
                       ->acceleration_structure_properties_khr.minAccelerationStructureScratchOffsetAlignment);
  buffer_create_info.size = build_sizes.buildScratchSize + static_cast<VkDeviceSize>(scratch_alignment) - 1;
  buffer_create_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  auto scratch_buffer = std::make_shared<Buffer>(buffer_create_info, allocation_create_info);

  VkAccelerationStructureBuildGeometryInfoKHR build_info = size_info;
  build_info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
  build_info.dstAccelerationStructure = vk_acceleration_structure_khr_;
  build_info.scratchData.deviceAddress = AlignUp(scratch_buffer->GetDeviceAddress(), scratch_alignment);
  VkAccelerationStructureBuildRangeInfoKHR range_info{};
  range_info.primitiveCount = primitive_count_;
  const VkAccelerationStructureBuildRangeInfoKHR* range_infos[] = {&range_info};
  Platform::ImmediateSubmitWithGpuTimestamp(
      {"LssBlasBuild", "LSS BLAS Build", "Ray Tracing", GpuTimestampQueue::Immediate, 0, 0},
      [&](const VkCommandBuffer command_buffer) {
        vkCmdBuildAccelerationStructuresKHR(command_buffer, 1, &build_info, range_infos);
      });

  VkAccelerationStructureDeviceAddressInfoKHR address_info{};
  address_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
  address_info.accelerationStructure = vk_acceleration_structure_khr_;
  device_address_ = vkGetAccelerationStructureDeviceAddressKHR(Platform::GetVkDevice(), &address_info);
  if (device_address_ == 0) {
    throw std::runtime_error("Linear swept sphere BLAS returned a zero device address.");
  }
  acceleration_structure_buffer_->SetDebugName("LSS BLAS Storage");
  content_version_ = 1;
  auto& builder_state = GetStaticBlasBuilderState();
  std::lock_guard lock(builder_state.mutex);
  ++builder_state.telemetry.total_blas_count;
  telemetry_registered_ = true;
#endif
}

void BottomLevelAccelerationStructure::ProcessStaticBuilds() {
  if (!Platform::Initialized()) {
    return;
  }
  auto& state = GetStaticBlasBuilderState();
  std::shared_ptr<StaticBlasBuildJob> completed_job;
  GpuWorkHandle completed_handle;
  {
    std::lock_guard lock(state.mutex);
    if (state.active_job && state.active_handle.Valid() && Jobs::IsCompleted(state.active_handle)) {
      completed_job = state.active_job;
      completed_handle = state.active_handle;
    }
  }
  if (completed_job) {
    try {
      Platform::GetGpuService().Wait(completed_handle);
    } catch (...) {
      std::lock_guard lock(state.mutex);
      if (state.active_job == completed_job) {
        state.pending.insert(state.pending.begin(), std::make_move_iterator(completed_job->deferred_requests.begin()),
                             std::make_move_iterator(completed_job->deferred_requests.end()));
        state.pending.insert(state.pending.begin(), std::make_move_iterator(completed_job->requests.begin()),
                             std::make_move_iterator(completed_job->requests.end()));
        state.active_job.reset();
        state.active_handle = {};
        state.active_pending_count = 0;
      }
      throw;
    }
    std::vector<std::shared_ptr<BottomLevelAccelerationStructure>> retained_targets;
    std::lock_guard lock(state.mutex);
    if (state.active_job == completed_job) {
      const auto pass_begin = state.telemetry.passes.empty()
                                  ? 0
                                  : state.telemetry.passes.back().begin + state.telemetry.passes.back().count;
      uint64_t published_count = 0;
      uint64_t published_uncompacted_bytes = 0;
      uint64_t published_compacted_bytes = 0;
      state.telemetry.cumulative_built_static_count =
          SaturatingAdd(state.telemetry.cumulative_built_static_count, completed_job->telemetry.static_eligible_count);
      state.telemetry.cumulative_uncompacted_bytes = SaturatingAdd(
          state.telemetry.cumulative_uncompacted_bytes, completed_job->telemetry.eligible_static_uncompacted_bytes);
      state.telemetry.cumulative_compacted_bytes = SaturatingAdd(
          state.telemetry.cumulative_compacted_bytes, completed_job->telemetry.eligible_static_compacted_bytes);
      for (auto& result : completed_job->results) {
        if (const auto target = result.target.lock(); target && !target->IsReady()) {
          retained_targets.emplace_back(target);
          target->vk_acceleration_structure_khr_ = result.acceleration_structure;
          target->acceleration_structure_buffer_ = std::move(result.buffer);
          target->device_address_ = result.device_address;
          target->content_version_ = 1;
          target->telemetry_registered_ = true;
          target->telemetry_uncompacted_bytes_ = result.uncompacted_size;
          target->telemetry_compacted_bytes_ = result.compacted_size;
          target->acceleration_structure_buffer_->SetDebugName("Compacted Static BLAS Storage");
          result.acceleration_structure = VK_NULL_HANDLE;
          ++published_count;
          published_uncompacted_bytes = SaturatingAdd(published_uncompacted_bytes, result.uncompacted_size);
          published_compacted_bytes = SaturatingAdd(published_compacted_bytes, result.compacted_size);
        } else {
          DestroyAccelerationStructure(result.acceleration_structure);
        }
      }
      state.telemetry.total_blas_count += published_count;
      state.telemetry.static_eligible_count += published_count;
      state.telemetry.shared_input_count += published_count;
      state.telemetry.pass_count += completed_job->telemetry.pass_count;
      state.telemetry.scratch_wave_count += completed_job->telemetry.scratch_wave_count;
      state.telemetry.scratch_peak_bytes =
          std::max(state.telemetry.scratch_peak_bytes, completed_job->telemetry.scratch_peak_bytes);
      state.telemetry.eligible_static_uncompacted_bytes =
          SaturatingAdd(state.telemetry.eligible_static_uncompacted_bytes, published_uncompacted_bytes);
      state.telemetry.eligible_static_compacted_bytes =
          SaturatingAdd(state.telemetry.eligible_static_compacted_bytes, published_compacted_bytes);
      state.telemetry.final_compacted_storage_bytes =
          SaturatingAdd(state.telemetry.final_compacted_storage_bytes, published_compacted_bytes);
      state.telemetry.transient_peak_bytes =
          std::max(state.telemetry.transient_peak_bytes, completed_job->telemetry.transient_peak_bytes);
      state.telemetry.wall_milliseconds += completed_job->telemetry.wall_milliseconds;
      for (auto pass : completed_job->telemetry.passes) {
        pass.begin += pass_begin;
        state.telemetry.passes.emplace_back(pass);
      }
      state.pending.insert(state.pending.begin(), std::make_move_iterator(completed_job->deferred_requests.begin()),
                           std::make_move_iterator(completed_job->deferred_requests.end()));
      state.active_job.reset();
      state.active_handle = {};
      state.active_pending_count = 0;
    }
  }

  if (GeometryStorage::HasPendingMeshUploads()) {
    return;
  }
  auto job = std::make_shared<StaticBlasBuildJob>();
  {
    std::lock_guard lock(state.mutex);
    if (state.active_job) {
      return;
    }
    for (auto iterator = state.pending.begin(); iterator != state.pending.end();) {
      if (iterator->target.expired()) {
        iterator = state.pending.erase(iterator);
        continue;
      }
      if (iterator->meshlet_range->range != iterator->meshlet_count ||
          iterator->triangle_range->range != iterator->triangle_count) {
        iterator = state.pending.erase(iterator);
        continue;
      }
      iterator->meshlet_offset = iterator->meshlet_range->offset;
      iterator->triangle_offset = iterator->triangle_range->offset;
      const bool committed = iterator->meshlet_range->prev_frame_offset == iterator->meshlet_offset &&
                             iterator->meshlet_range->prev_frame_range == iterator->meshlet_count &&
                             iterator->triangle_range->prev_frame_offset == iterator->triangle_offset &&
                             iterator->triangle_range->prev_frame_range == iterator->triangle_count;
      if (!committed) {
        ++iterator;
        continue;
      }
      job->requests.emplace_back(std::move(*iterator));
      iterator = state.pending.erase(iterator);
    }
    if (job->requests.empty()) {
      return;
    }
    job->retained_compacted_bytes = state.telemetry.final_compacted_storage_bytes;
    state.active_job = job;
    state.active_pending_count = job->requests.size();
  }

  const auto vertex_buffer = GeometryStorage::GetVertexBuffer();
  const auto index_buffer = GeometryStorage::GetTriangleBuffer();
  GpuWorkOptions options;
  options.priority = TaskPriority::Low;
  options.debug_name = "Static BLAS build and compaction";
  try {
    const auto handle = Platform::GetGpuService().Enqueue(options, [job, vertex_buffer, index_buffer]() {
      ExecuteStaticBlasBuildJob(job, vertex_buffer, index_buffer);
    });
    std::lock_guard lock(state.mutex);
    state.active_handle = handle;
  } catch (...) {
    std::lock_guard lock(state.mutex);
    state.pending.insert(state.pending.begin(), std::make_move_iterator(job->requests.begin()),
                         std::make_move_iterator(job->requests.end()));
    state.active_job.reset();
    state.active_handle = {};
    state.active_pending_count = 0;
    throw;
  }
}

void BottomLevelAccelerationStructure::WaitForActiveStaticBuild() {
  while (true) {
    GpuWorkHandle active_handle;
    {
      auto& state = GetStaticBlasBuilderState();
      std::lock_guard lock(state.mutex);
      if (!state.active_job) {
        return;
      }
      active_handle = state.active_handle;
    }
    if (!active_handle.Valid()) {
      std::this_thread::yield();
      continue;
    }
    try {
      Platform::GetGpuService().Wait(active_handle);
    } catch (...) {
      ProcessStaticBuilds();
      throw;
    }
    ProcessStaticBuilds();
    return;
  }
}

void BottomLevelAccelerationStructure::WaitForStaticBuilds() {
  while (HasPendingStaticBuilds()) {
    GeometryStorage::WaitForPendingUploads();
    ProcessStaticBuilds();
    GpuWorkHandle active_handle;
    {
      auto& state = GetStaticBlasBuilderState();
      std::lock_guard lock(state.mutex);
      active_handle = state.active_handle;
    }
    if (active_handle.Valid()) {
      try {
        Platform::GetGpuService().Wait(active_handle);
      } catch (...) {
        ProcessStaticBuilds();
        throw;
      }
      ProcessStaticBuilds();
    }
  }
}

bool BottomLevelAccelerationStructure::HasPendingStaticBuilds() {
  auto& state = GetStaticBlasBuilderState();
  std::lock_guard lock(state.mutex);
  return state.active_job || !state.pending.empty();
}

bool BottomLevelAccelerationStructure::StaticBuildInProgress() {
  auto& state = GetStaticBlasBuilderState();
  std::lock_guard lock(state.mutex);
  return state.active_job != nullptr;
}

StaticBlasBuildTelemetry BottomLevelAccelerationStructure::GetStaticBuildTelemetry() {
  auto& state = GetStaticBlasBuilderState();
  std::lock_guard lock(state.mutex);
  auto result = state.telemetry;
  result.pending_count = state.pending.size() + state.active_pending_count;
  result.complete = result.pending_count == 0;
  return result;
}

BottomLevelAccelerationStructure::BottomLevelAccelerationStructure(const std::vector<Vertex>& vertices,
                                                                   const std::vector<glm::uvec3>& triangles,
                                                                   const bool allow_update)
    : vertex_count_(static_cast<uint32_t>(vertices.size())),
      primitive_count_(static_cast<uint32_t>(triangles.size())),
      allow_update_(allow_update) {
  if (!allow_update_) {
    throw std::invalid_argument("Private BLAS construction is reserved for updateable geometry.");
  }
  if (!Platform::Initialized() || vertices.empty() || triangles.empty())
    return;
  if (!HasFiniteBlasPositions(vertices)) {
    throw std::runtime_error("BLAS vertices contain a non-finite position.");
  }
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                             VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo device_allocation_create_info{};
  device_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  buffer_create_info.size = vertices.size() * sizeof(Vertex);
  vertex_buffer = std::make_shared<Buffer>(buffer_create_info, device_allocation_create_info);
  vertex_buffer->UploadVector(vertices);
  buffer_create_info.size = triangles.size() * sizeof(glm::uvec3);
  index_buffer = std::make_shared<Buffer>(buffer_create_info, device_allocation_create_info);
  index_buffer->UploadVector(triangles);
  buffer_create_info.size = sizeof(VkTransformMatrixKHR);
  transform_buffer = std::make_shared<Buffer>(buffer_create_info, device_allocation_create_info);
  const VkTransformMatrixKHR transform_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                                                 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
  transform_buffer->Upload(transform_matrix);

  auto geometry = CreateBlasGeometry(*vertex_buffer, *index_buffer, *transform_buffer, vertex_count_);
  VkAccelerationStructureBuildGeometryInfoKHR size_info{};
  size_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
  size_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  size_info.flags = kBlasBuildFlags | (allow_update_ ? VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR : 0);
  size_info.geometryCount = 1;
  size_info.pGeometries = &geometry;
  VkAccelerationStructureBuildSizesInfoKHR build_sizes{};
  build_sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
  vkGetAccelerationStructureBuildSizesKHR(Platform::GetVkDevice(), VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                          &size_info, &primitive_count_, &build_sizes);

  buffer_create_info.size = build_sizes.accelerationStructureSize;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  acceleration_structure_buffer_ = std::make_shared<Buffer>(buffer_create_info, device_allocation_create_info);
  const auto scratch_alignment =
      std::max(1u, Platform::GetSelectedPhysicalDevice()
                       ->acceleration_structure_properties_khr.minAccelerationStructureScratchOffsetAlignment);
  buffer_create_info.size = (allow_update_ ? std::max(build_sizes.buildScratchSize, build_sizes.updateScratchSize)
                                           : build_sizes.buildScratchSize) +
                            static_cast<VkDeviceSize>(scratch_alignment) - 1;
  buffer_create_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  auto scratch_buffer = std::make_shared<Buffer>(buffer_create_info, device_allocation_create_info);
  if (allow_update_) {
    scratch_buffer_ = scratch_buffer;
    const auto frame_slots = std::max(1u, static_cast<uint32_t>(Platform::GetMaxFramesInFlight()));
    vertex_staging_buffers_.reserve(frame_slots);
    for (uint32_t frame_slot = 0; frame_slot < frame_slots; frame_slot++) {
      vertex_staging_buffers_.emplace_back(std::make_shared<Buffer>(vertices.size() * sizeof(Vertex)));
    }
  }

  VkAccelerationStructureCreateInfoKHR create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
  create_info.buffer = acceleration_structure_buffer_->GetVkBuffer();
  create_info.size = build_sizes.accelerationStructureSize;
  create_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  Platform::CheckVk(vkCreateAccelerationStructureKHR(Platform::GetVkDevice(), &create_info, nullptr,
                                                     &vk_acceleration_structure_khr_));

  VkAccelerationStructureBuildGeometryInfoKHR build_info{};
  build_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
  build_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  build_info.flags = size_info.flags;
  build_info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
  build_info.dstAccelerationStructure = vk_acceleration_structure_khr_;
  build_info.geometryCount = 1;
  build_info.pGeometries = &geometry;
  build_info.scratchData.deviceAddress =
      (scratch_buffer->GetDeviceAddress() + scratch_alignment - 1) / scratch_alignment * scratch_alignment;
  VkAccelerationStructureBuildRangeInfoKHR range_info{};
  range_info.primitiveCount = primitive_count_;
  const VkAccelerationStructureBuildRangeInfoKHR* range_infos[] = {&range_info};
  Platform::ImmediateSubmitWithGpuTimestamp(
      {"BlasBuild", "BLAS Build", "Ray Tracing", GpuTimestampQueue::Immediate, 0, 0},
      [&](const VkCommandBuffer vk_command_buffer) {
        vkCmdBuildAccelerationStructuresKHR(vk_command_buffer, 1, &build_info, range_infos);
      });

  VkAccelerationStructureDeviceAddressInfoKHR address_info{};
  address_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
  address_info.accelerationStructure = vk_acceleration_structure_khr_;
  device_address_ = vkGetAccelerationStructureDeviceAddressKHR(Platform::GetVkDevice(), &address_info);
  content_version_ = 1;
  if (allow_update_) {
    vertex_buffer->SetDebugName("Dynamic BLAS Vertex Input");
    index_buffer->SetDebugName("Dynamic BLAS Index Input");
    acceleration_structure_buffer_->SetDebugName("Dynamic BLAS Storage");
    scratch_buffer_->SetDebugName("Dynamic BLAS Scratch");
  }
  auto& builder_state = GetStaticBlasBuilderState();
  std::lock_guard lock(builder_state.mutex);
  ++builder_state.telemetry.total_blas_count;
  ++builder_state.telemetry.private_input_count;
  if (allow_update_) {
    ++builder_state.telemetry.updateable_count;
  }
  builder_state.telemetry.private_input_bytes = SaturatingAdd(
      builder_state.telemetry.private_input_bytes,
      SaturatingAdd(vertex_buffer->GetSize(), SaturatingAdd(index_buffer->GetSize(), transform_buffer->GetSize())));
  telemetry_registered_ = true;
  telemetry_updateable_ = allow_update_;
  telemetry_private_input_bytes_ =
      SaturatingAdd(vertex_buffer->GetSize(), SaturatingAdd(index_buffer->GetSize(), transform_buffer->GetSize()));
}

void BottomLevelAccelerationStructure::ResolvePendingUpdate() {
  if (!pending_update_ || !pending_submission_state_ ||
      pending_submission_state_->status == FrameSubmissionState::Status::Pending) {
    return;
  }
  if (pending_submission_state_->status == FrameSubmissionState::Status::Submitted) {
    content_version_ = pending_content_version_;
  }
  pending_update_ = false;
  pending_content_version_ = 0;
  pending_submission_state_.reset();
}

std::shared_ptr<FrameSubmissionState> BottomLevelAccelerationStructure::UpdateVertices(
    const std::vector<Vertex>& vertices) {
  if (!allow_update_ || !scratch_buffer_) {
    throw std::runtime_error("Bottom-level acceleration structure was not created for updates.");
  }
  if (vertices.size() != vertex_count_) {
    throw std::runtime_error("Dynamic BLAS vertex count changed; a full rebuild is required.");
  }
  if (!HasFiniteBlasPositions(vertices)) {
    throw std::runtime_error("Dynamic BLAS vertices contain a non-finite position.");
  }
  ResolvePendingUpdate();
  if (pending_update_) {
    return pending_submission_state_;
  }

  const auto byte_size = static_cast<VkDeviceSize>(vertices.size() * sizeof(Vertex));
  const auto staging_buffer = vertex_staging_buffers_.at(Platform::GetCurrentFrameIndex());
  void* mapped_data = nullptr;
  Platform::CheckVk(vmaMapMemory(Platform::GetVmaAllocator(), staging_buffer->GetVmaAllocation(), &mapped_data));
  memcpy(mapped_data, vertices.data(), byte_size);
  Platform::CheckVk(vmaFlushAllocation(Platform::GetVmaAllocator(), staging_buffer->GetVmaAllocation(), 0, byte_size));
  vmaUnmapMemory(Platform::GetVmaAllocator(), staging_buffer->GetVmaAllocation());

  Platform::RecordCommandsMainQueue([this, staging_buffer, byte_size](const VkCommandBuffer vk_command_buffer) {
    VkBufferMemoryBarrier2 reuse_vertex_barrier{};
    reuse_vertex_barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
    reuse_vertex_barrier.srcStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
    reuse_vertex_barrier.srcAccessMask = VK_ACCESS_2_SHADER_READ_BIT;
    reuse_vertex_barrier.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    reuse_vertex_barrier.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    reuse_vertex_barrier.buffer = vertex_buffer->GetVkBuffer();
    reuse_vertex_barrier.offset = 0;
    reuse_vertex_barrier.size = byte_size;
    VkDependencyInfo reuse_vertex_dependency{};
    reuse_vertex_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    reuse_vertex_dependency.bufferMemoryBarrierCount = 1;
    reuse_vertex_dependency.pBufferMemoryBarriers = &reuse_vertex_barrier;
    vkCmdPipelineBarrier2(vk_command_buffer, &reuse_vertex_dependency);

    VkBufferCopy copy_region{};
    copy_region.size = byte_size;
    vkCmdCopyBuffer(vk_command_buffer, staging_buffer->GetVkBuffer(), vertex_buffer->GetVkBuffer(), 1, &copy_region);

    VkBufferMemoryBarrier2 vertex_barrier{};
    vertex_barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
    vertex_barrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    vertex_barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    vertex_barrier.dstStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
    vertex_barrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT;
    vertex_barrier.buffer = vertex_buffer->GetVkBuffer();
    vertex_barrier.offset = 0;
    vertex_barrier.size = byte_size;

    VkMemoryBarrier2 reuse_as_barrier{};
    reuse_as_barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    reuse_as_barrier.srcStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR | RayTraversalStageMask();
    reuse_as_barrier.srcAccessMask =
        VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR | VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
    reuse_as_barrier.dstStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
    reuse_as_barrier.dstAccessMask =
        VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR | VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
    VkDependencyInfo pre_update_dependency{};
    pre_update_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    pre_update_dependency.memoryBarrierCount = 1;
    pre_update_dependency.pMemoryBarriers = &reuse_as_barrier;
    pre_update_dependency.bufferMemoryBarrierCount = 1;
    pre_update_dependency.pBufferMemoryBarriers = &vertex_barrier;
    vkCmdPipelineBarrier2(vk_command_buffer, &pre_update_dependency);

    auto geometry = CreateBlasGeometry(*vertex_buffer, *index_buffer, *transform_buffer, vertex_count_);
    VkAccelerationStructureBuildGeometryInfoKHR build_info{};
    build_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    build_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    build_info.flags = kBlasBuildFlags | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    build_info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR;
    build_info.srcAccelerationStructure = vk_acceleration_structure_khr_;
    build_info.dstAccelerationStructure = vk_acceleration_structure_khr_;
    build_info.geometryCount = 1;
    build_info.pGeometries = &geometry;
    const auto scratch_alignment =
        std::max(1u, Platform::GetSelectedPhysicalDevice()
                         ->acceleration_structure_properties_khr.minAccelerationStructureScratchOffsetAlignment);
    build_info.scratchData.deviceAddress =
        (scratch_buffer_->GetDeviceAddress() + scratch_alignment - 1) / scratch_alignment * scratch_alignment;
    VkAccelerationStructureBuildRangeInfoKHR range_info{};
    range_info.primitiveCount = primitive_count_;
    const VkAccelerationStructureBuildRangeInfoKHR* range_infos[] = {&range_info};
    const auto timestamp = Platform::BeginGpuTimestampScope(
        vk_command_buffer, {"BlasUpdate", "BLAS Update", "Ray Tracing", GpuTimestampQueue::Graphics, 0, 0});
    vkCmdBuildAccelerationStructuresKHR(vk_command_buffer, 1, &build_info, range_infos);
    Platform::EndGpuTimestampScope(vk_command_buffer, timestamp);

    VkMemoryBarrier2 completion_barrier{};
    completion_barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    completion_barrier.srcStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
    completion_barrier.srcAccessMask = VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
    completion_barrier.dstStageMask =
        VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR | RayTraversalStageMask();
    completion_barrier.dstAccessMask =
        VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR | VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
    VkDependencyInfo completion_dependency{};
    completion_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    completion_dependency.memoryBarrierCount = 1;
    completion_dependency.pMemoryBarriers = &completion_barrier;
    vkCmdPipelineBarrier2(vk_command_buffer, &completion_dependency);
  });

  pending_update_ = true;
  pending_content_version_ = content_version_ + 1;
  pending_submission_state_ = Platform::TrackCurrentFrameSubmission();
  return pending_submission_state_;
}

uint32_t BottomLevelAccelerationStructure::GetContentVersion() {
  ResolvePendingUpdate();
  return pending_update_ ? pending_content_version_ : content_version_;
}

BottomLevelAccelerationStructure::~BottomLevelAccelerationStructure() {
  if (telemetry_registered_) {
    auto& state = GetStaticBlasBuilderState();
    std::lock_guard lock(state.mutex);
    state.telemetry.total_blas_count = state.telemetry.total_blas_count > 0 ? state.telemetry.total_blas_count - 1 : 0;
    if (telemetry_uncompacted_bytes_ != 0) {
      state.telemetry.static_eligible_count =
          state.telemetry.static_eligible_count > 0 ? state.telemetry.static_eligible_count - 1 : 0;
      state.telemetry.shared_input_count =
          state.telemetry.shared_input_count > 0 ? state.telemetry.shared_input_count - 1 : 0;
      state.telemetry.eligible_static_uncompacted_bytes -= std::min(
          state.telemetry.eligible_static_uncompacted_bytes, static_cast<uint64_t>(telemetry_uncompacted_bytes_));
      state.telemetry.eligible_static_compacted_bytes -=
          std::min(state.telemetry.eligible_static_compacted_bytes, static_cast<uint64_t>(telemetry_compacted_bytes_));
      state.telemetry.final_compacted_storage_bytes -=
          std::min(state.telemetry.final_compacted_storage_bytes, static_cast<uint64_t>(telemetry_compacted_bytes_));
    } else if (telemetry_private_input_bytes_ != 0) {
      if (telemetry_updateable_) {
        state.telemetry.updateable_count =
            state.telemetry.updateable_count > 0 ? state.telemetry.updateable_count - 1 : 0;
      }
      state.telemetry.private_input_count =
          state.telemetry.private_input_count > 0 ? state.telemetry.private_input_count - 1 : 0;
      state.telemetry.private_input_bytes -=
          std::min(state.telemetry.private_input_bytes, static_cast<uint64_t>(telemetry_private_input_bytes_));
    }
  }
  if (!Platform::Initialized())
    return;
  if (vk_acceleration_structure_khr_ != VK_NULL_HANDLE)
    vkDestroyAccelerationStructureKHR(Platform::GetVkDevice(), vk_acceleration_structure_khr_, nullptr);
}

VkDeviceAddress BottomLevelAccelerationStructure::GetDeviceAddress() const {
  return device_address_;
}

bool BottomLevelAccelerationStructure::IsReady() const {
  return vk_acceleration_structure_khr_ != VK_NULL_HANDLE && device_address_ != 0;
}

TopLevelAccelerationStructure::UpdateMode TopLevelAccelerationStructure::ClassifyUpdateMode(
    const bool built, const std::vector<VkAccelerationStructureInstanceKHR>& previous_instances,
    const std::vector<VkAccelerationStructureInstanceKHR>& current_instances) {
  return ClassifyUpdateMode(built, previous_instances, current_instances, {}, {});
}

TopLevelAccelerationStructure::UpdateMode TopLevelAccelerationStructure::ClassifyUpdateMode(
    const bool built, const std::vector<VkAccelerationStructureInstanceKHR>& previous_instances,
    const std::vector<VkAccelerationStructureInstanceKHR>& current_instances,
    const std::vector<uint32_t>& previous_blas_content_versions,
    const std::vector<uint32_t>& current_blas_content_versions) {
  if (!built || previous_instances.size() != current_instances.size()) {
    return UpdateMode::Build;
  }
  bool changed = false;
  for (size_t index = 0; index < current_instances.size(); ++index) {
    const auto& previous = previous_instances[index];
    const auto& current = current_instances[index];
    if ((previous.accelerationStructureReference == 0) != (current.accelerationStructureReference == 0)) {
      return UpdateMode::Build;
    }
    changed |= !AccelerationStructureInstancesEqual(previous, current);
  }
  return changed || previous_blas_content_versions != current_blas_content_versions ? UpdateMode::Update
                                                                                    : UpdateMode::NoOp;
}

std::vector<TopLevelAccelerationStructure::InstanceUploadRange> TopLevelAccelerationStructure::PlanInstanceUploadRanges(
    const bool full_upload, const std::vector<VkAccelerationStructureInstanceKHR>& previous_instances,
    const std::vector<VkAccelerationStructureInstanceKHR>& current_instances) {
  if (current_instances.empty()) {
    return {};
  }
  if (full_upload) {
    return {{0, static_cast<uint32_t>(current_instances.size())}};
  }
  std::vector<InstanceUploadRange> ranges;
  for (uint32_t index = 0; index < current_instances.size(); ++index) {
    if (index < previous_instances.size() &&
        AccelerationStructureInstancesEqual(previous_instances[index], current_instances[index])) {
      continue;
    }
    if (!ranges.empty() && ranges.back().first_instance + ranges.back().instance_count == index) {
      ranges.back().instance_count++;
    } else {
      ranges.push_back({index, 1});
    }
  }
  return ranges;
}

void TopLevelAccelerationStructure::Destroy() {
  if (Platform::Initialized() && vk_acceleration_structure_khr_ != VK_NULL_HANDLE) {
    vkDestroyAccelerationStructureKHR(Platform::GetVkDevice(), vk_acceleration_structure_khr_, nullptr);
  }
  vk_acceleration_structure_khr_ = VK_NULL_HANDLE;
  acceleration_structure_buffer_.reset();
  instance_staging_buffer_.reset();
  instances_data_buffer_.reset();
  scratch_buffer_.reset();
  device_address_ = 0;
  instance_capacity_ = 0;
  built_ = false;
  previous_instances_.clear();
  previous_blas_content_versions_.clear();
  committed_blas_references_.clear();
  pending_ = false;
  pending_submission_state_.reset();
  pending_instances_.clear();
  pending_blas_content_versions_.clear();
  pending_final_blas_references_.clear();
  pending_retained_blas_references_.clear();
  pending_extra_staging_buffers_.clear();
}

void TopLevelAccelerationStructure::Allocate(const uint32_t instance_capacity) {
  Destroy();
  instance_capacity_ = instance_capacity;
  const auto instance_buffer_size =
      static_cast<VkDeviceSize>(instance_capacity_) * sizeof(VkAccelerationStructureInstanceKHR);
  instance_staging_buffer_ = std::make_shared<Buffer>(instance_buffer_size);

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.size = instance_buffer_size;
  buffer_create_info.usage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                             VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo device_allocation_create_info{};
  device_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  instances_data_buffer_ = std::make_shared<Buffer>(buffer_create_info, device_allocation_create_info);

  VkAccelerationStructureGeometryKHR geometry{};
  geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
  geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
  geometry.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
  geometry.geometry.instances.data.deviceAddress = instances_data_buffer_->GetDeviceAddress();

  VkAccelerationStructureBuildGeometryInfoKHR build_geometry_info{};
  build_geometry_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
  build_geometry_info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
  build_geometry_info.flags = kTlasBuildFlags;
  build_geometry_info.geometryCount = 1;
  build_geometry_info.pGeometries = &geometry;
  VkAccelerationStructureBuildSizesInfoKHR build_sizes{};
  build_sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
  vkGetAccelerationStructureBuildSizesKHR(Platform::GetVkDevice(), VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                          &build_geometry_info, &instance_capacity_, &build_sizes);

  buffer_create_info.size = build_sizes.accelerationStructureSize;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  acceleration_structure_buffer_ = std::make_shared<Buffer>(buffer_create_info, device_allocation_create_info);

  const auto scratch_alignment =
      std::max(1u, Platform::GetSelectedPhysicalDevice()
                       ->acceleration_structure_properties_khr.minAccelerationStructureScratchOffsetAlignment);
  buffer_create_info.size = std::max(build_sizes.buildScratchSize, build_sizes.updateScratchSize) +
                            static_cast<VkDeviceSize>(scratch_alignment) - 1;
  buffer_create_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  scratch_buffer_ = std::make_shared<Buffer>(buffer_create_info, device_allocation_create_info);

  VkAccelerationStructureCreateInfoKHR create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
  create_info.buffer = acceleration_structure_buffer_->GetVkBuffer();
  create_info.size = build_sizes.accelerationStructureSize;
  create_info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
  Platform::CheckVk(vkCreateAccelerationStructureKHR(Platform::GetVkDevice(), &create_info, nullptr,
                                                     &vk_acceleration_structure_khr_));

  VkAccelerationStructureDeviceAddressInfoKHR address_info{};
  address_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
  address_info.accelerationStructure = vk_acceleration_structure_khr_;
  device_address_ = vkGetAccelerationStructureDeviceAddressKHR(Platform::GetVkDevice(), &address_info);
  instance_staging_buffer_->SetDebugName("TLAS Instance Staging");
  instances_data_buffer_->SetDebugName("TLAS Instance Input");
  acceleration_structure_buffer_->SetDebugName("TLAS Storage");
  scratch_buffer_->SetDebugName("TLAS Scratch");
}

void TopLevelAccelerationStructure::ResolvePendingUpdate() {
  if (!pending_) {
    return;
  }
  if (!pending_submission_state_ || pending_submission_state_->status == FrameSubmissionState::Status::Pending) {
    return;
  }
  if (pending_submission_state_->status == FrameSubmissionState::Status::Submitted) {
    built_ = true;
    previous_instances_ = std::move(pending_instances_);
    previous_blas_content_versions_ = std::move(pending_blas_content_versions_);
    committed_blas_references_ = std::move(pending_final_blas_references_);
  }
  pending_ = false;
  pending_submission_state_.reset();
  pending_instances_.clear();
  pending_blas_content_versions_.clear();
  pending_final_blas_references_.clear();
  pending_retained_blas_references_.clear();
  pending_extra_staging_buffers_.clear();
}

TopLevelAccelerationStructure::UpdateMode TopLevelAccelerationStructure::Update(
    RenderInstanceStorage& render_instance_storage) {
  if (!Platform::Initialized()) {
    return UpdateMode::NoOp;
  }
  ResolvePendingUpdate();

  std::vector<VkAccelerationStructureInstanceKHR> instances;
  std::vector<uint32_t> blas_content_versions;
  std::vector<std::shared_ptr<BottomLevelAccelerationStructure>> blas_references;
  std::unordered_set<const BottomLevelAccelerationStructure*> unique_blas_references;
  const auto append_blas_reference = [&](const std::shared_ptr<BottomLevelAccelerationStructure>& blas) {
    if (unique_blas_references.emplace(blas.get()).second) {
      blas_references.emplace_back(blas);
    }
  };
  const auto& gltf_shade_materials = render_instance_storage.GetGltfShadeMaterials();
  const auto register_instance = [&](const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance,
                                     const std::shared_ptr<BottomLevelAccelerationStructure>& blas,
                                     const glm::mat4& model, const uint32_t custom_index,
                                     const bool linear_swept_spheres = false) {
    if (!render_instance || !blas || !blas->IsReady()) {
      return;
    }
    if (custom_index > 0x00ffffffu) {
      throw std::runtime_error("Ray tracing instance custom index exceeds 24 bits.");
    }
    auto& instance = instances.emplace_back();
    blas_content_versions.emplace_back(blas->GetContentVersion());
    instance.instanceCustomIndex = custom_index;
    instance.instanceShaderBindingTableRecordOffset = linear_swept_spheres ? 1u : 0u;
    if (!IsValidAccelerationStructureTransform(model)) {
      SetAccelerationStructureTransform(instance, glm::mat4(1.0f));
      instance.mask = 0;
      instance.accelerationStructureReference = 0;
      return;
    }
    SetAccelerationStructureTransform(instance, model);
    instance.mask = linear_swept_spheres
                        ? kCameraLssRayMaskGeometry | (render_instance->cast_shadow ? kCameraLssRayMaskShadow : 0u)
                        : kDdgiRayMaskGeometry | (render_instance->cast_shadow ? kDdgiRayMaskShadow : 0u);
    instance.flags = BuildGltfRayTracingInstanceFlags(*render_instance, gltf_shade_materials);
    if (linear_swept_spheres) {
      instance.flags |= VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
    }
    instance.accelerationStructureReference = blas->GetDeviceAddress();
    append_blas_reference(blas);
  };
  instances.reserve(render_instance_storage.top_level_acceleration_structure_inputs_.size());
  blas_content_versions.reserve(render_instance_storage.top_level_acceleration_structure_inputs_.size());
  unique_blas_references.reserve(render_instance_storage.top_level_acceleration_structure_inputs_.size());
  for (const auto& input : render_instance_storage.top_level_acceleration_structure_inputs_) {
    register_instance(input.render_instance, input.bottom_level_acceleration_structure, input.model, input.custom_index,
                      input.linear_swept_spheres);
  }

  if (instances.empty()) {
    auto& dummy = instances.emplace_back();
    blas_content_versions.emplace_back(0);
    SetAccelerationStructureTransform(dummy, glm::mat4(1.0f));
    dummy.mask = 0;
    dummy.accelerationStructureReference = 0;
  }
  const auto max_instance_count =
      Platform::GetSelectedPhysicalDevice()->acceleration_structure_properties_khr.maxInstanceCount;
  if (instances.size() > max_instance_count || instances.size() > (std::numeric_limits<uint32_t>::max)()) {
    throw std::runtime_error("Top-level acceleration structure instance count exceeds the device limit.");
  }

  const auto frame_index = Platform::GetCurrentFrameIndex();
  const auto frame_count = Platform::GetFrameCount();
  const bool same_pending_frame =
      pending_ && pending_frame_index_ == frame_index && pending_frame_count_ == frame_count;
  const bool base_built = same_pending_frame || built_;
  const auto& base_instances = same_pending_frame ? pending_instances_ : previous_instances_;
  const auto& base_blas_content_versions =
      same_pending_frame ? pending_blas_content_versions_ : previous_blas_content_versions_;
  auto mode =
      ClassifyUpdateMode(base_built, base_instances, instances, base_blas_content_versions, blas_content_versions);
  if (mode == UpdateMode::NoOp) {
    return mode;
  }

  const auto instance_count = static_cast<uint32_t>(instances.size());
  bool full_upload = !base_built;
  if (vk_acceleration_structure_khr_ == VK_NULL_HANDLE || instance_count > instance_capacity_) {
    if (same_pending_frame) {
      throw std::runtime_error("TLAS capacity cannot grow after recording an update in the same frame.");
    }
    const auto doubled_capacity = instance_capacity_ > (std::numeric_limits<uint32_t>::max)() / 2
                                      ? (std::numeric_limits<uint32_t>::max)()
                                      : instance_capacity_ * 2;
    Allocate(std::max(instance_count, std::max(1u, doubled_capacity)));
    mode = UpdateMode::Build;
    full_upload = true;
  }

  const auto byte_size = static_cast<VkDeviceSize>(instance_count) * sizeof(VkAccelerationStructureInstanceKHR);
  const auto upload_ranges = PlanInstanceUploadRanges(full_upload, base_instances, instances);
  VkDeviceSize upload_byte_size = 0;
  for (const auto& range : upload_ranges) {
    upload_byte_size += static_cast<VkDeviceSize>(range.instance_count) * sizeof(VkAccelerationStructureInstanceKHR);
  }
  std::shared_ptr<Buffer> staging_buffer;
  if (upload_byte_size != 0) {
    staging_buffer = instance_staging_buffer_;
    if (same_pending_frame) {
      staging_buffer = std::make_shared<Buffer>(upload_byte_size);
      pending_extra_staging_buffers_.emplace_back(staging_buffer);
    }
    void* mapped_data = nullptr;
    Platform::CheckVk(vmaMapMemory(Platform::GetVmaAllocator(), staging_buffer->GetVmaAllocation(), &mapped_data));
    VkDeviceSize staging_offset = 0;
    for (const auto& range : upload_ranges) {
      const auto range_byte_size =
          static_cast<VkDeviceSize>(range.instance_count) * sizeof(VkAccelerationStructureInstanceKHR);
      memcpy(static_cast<std::byte*>(mapped_data) + staging_offset, instances.data() + range.first_instance,
             range_byte_size);
      staging_offset += range_byte_size;
    }
    Platform::CheckVk(
        vmaFlushAllocation(Platform::GetVmaAllocator(), staging_buffer->GetVmaAllocation(), 0, upload_byte_size));
    vmaUnmapMemory(Platform::GetVmaAllocator(), staging_buffer->GetVmaAllocation());
  }

  const auto existing_build = base_built;
  Platform::RecordCommandsMainQueue([this, staging_buffer, upload_ranges, instance_count, mode, existing_build,
                                     same_pending_frame](const VkCommandBuffer vk_command_buffer) {
    std::vector<VkBufferMemoryBarrier2> instance_barriers;
    std::vector<VkBufferMemoryBarrier2> reuse_instance_buffer_barriers;
    std::vector<VkBufferCopy> copy_regions;
    VkDeviceSize staging_offset = 0;
    for (const auto& range : upload_ranges) {
      const auto destination_offset =
          static_cast<VkDeviceSize>(range.first_instance) * sizeof(VkAccelerationStructureInstanceKHR);
      const auto range_byte_size =
          static_cast<VkDeviceSize>(range.instance_count) * sizeof(VkAccelerationStructureInstanceKHR);
      copy_regions.push_back({staging_offset, destination_offset, range_byte_size});

      auto& instance_barrier = instance_barriers.emplace_back();
      instance_barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
      instance_barrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
      instance_barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
      instance_barrier.dstStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
      instance_barrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT;
      instance_barrier.buffer = instances_data_buffer_->GetVkBuffer();
      instance_barrier.offset = destination_offset;
      instance_barrier.size = range_byte_size;

      if (same_pending_frame) {
        auto& reuse_barrier = reuse_instance_buffer_barriers.emplace_back();
        reuse_barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
        reuse_barrier.srcStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
        reuse_barrier.srcAccessMask = VK_ACCESS_2_SHADER_READ_BIT;
        reuse_barrier.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
        reuse_barrier.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
        reuse_barrier.buffer = instances_data_buffer_->GetVkBuffer();
        reuse_barrier.offset = destination_offset;
        reuse_barrier.size = range_byte_size;
      }
      staging_offset += range_byte_size;
    }
    if (!reuse_instance_buffer_barriers.empty()) {
      VkDependencyInfo reuse_instance_buffer_dependency{};
      reuse_instance_buffer_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
      reuse_instance_buffer_dependency.bufferMemoryBarrierCount =
          static_cast<uint32_t>(reuse_instance_buffer_barriers.size());
      reuse_instance_buffer_dependency.pBufferMemoryBarriers = reuse_instance_buffer_barriers.data();
      vkCmdPipelineBarrier2(vk_command_buffer, &reuse_instance_buffer_dependency);
    }
    if (!copy_regions.empty()) {
      vkCmdCopyBuffer(vk_command_buffer, staging_buffer->GetVkBuffer(), instances_data_buffer_->GetVkBuffer(),
                      static_cast<uint32_t>(copy_regions.size()), copy_regions.data());
    }

    VkMemoryBarrier2 reuse_barrier{};
    reuse_barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    reuse_barrier.srcStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR | RayTraversalStageMask();
    reuse_barrier.srcAccessMask =
        VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR | VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR;
    reuse_barrier.dstStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
    reuse_barrier.dstAccessMask =
        VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR | VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;

    VkDependencyInfo pre_build_dependency{};
    pre_build_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    pre_build_dependency.memoryBarrierCount = existing_build ? 1u : 0u;
    pre_build_dependency.pMemoryBarriers = existing_build ? &reuse_barrier : nullptr;
    pre_build_dependency.bufferMemoryBarrierCount = static_cast<uint32_t>(instance_barriers.size());
    pre_build_dependency.pBufferMemoryBarriers = instance_barriers.data();
    vkCmdPipelineBarrier2(vk_command_buffer, &pre_build_dependency);

    VkAccelerationStructureGeometryKHR geometry{};
    geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
    geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    geometry.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
    geometry.geometry.instances.data.deviceAddress = instances_data_buffer_->GetDeviceAddress();
    VkAccelerationStructureBuildGeometryInfoKHR build_info{};
    build_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    build_info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    build_info.flags = kTlasBuildFlags;
    build_info.mode = mode == UpdateMode::Update ? VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR
                                                 : VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    build_info.srcAccelerationStructure = mode == UpdateMode::Update ? vk_acceleration_structure_khr_ : VK_NULL_HANDLE;
    build_info.dstAccelerationStructure = vk_acceleration_structure_khr_;
    build_info.geometryCount = 1;
    build_info.pGeometries = &geometry;
    const auto scratch_alignment =
        std::max(1u, Platform::GetSelectedPhysicalDevice()
                         ->acceleration_structure_properties_khr.minAccelerationStructureScratchOffsetAlignment);
    build_info.scratchData.deviceAddress =
        (scratch_buffer_->GetDeviceAddress() + scratch_alignment - 1) / scratch_alignment * scratch_alignment;
    VkAccelerationStructureBuildRangeInfoKHR range_info{};
    range_info.primitiveCount = instance_count;
    const VkAccelerationStructureBuildRangeInfoKHR* range_infos[] = {&range_info};

    const auto timestamp =
        Platform::BeginGpuTimestampScope(vk_command_buffer, {mode == UpdateMode::Update ? "TlasUpdate" : "TlasBuild",
                                                             mode == UpdateMode::Update ? "TLAS Update" : "TLAS Build",
                                                             "Ray Tracing", GpuTimestampQueue::Graphics, 0, 0});
    vkCmdBuildAccelerationStructuresKHR(vk_command_buffer, 1, &build_info, range_infos);
    Platform::EndGpuTimestampScope(vk_command_buffer, timestamp);

    VkMemoryBarrier2 traversal_barrier{};
    traversal_barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    traversal_barrier.srcStageMask = VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
    traversal_barrier.srcAccessMask = VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
    traversal_barrier.dstStageMask = RayTraversalStageMask();
    traversal_barrier.dstAccessMask = VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR;
    VkDependencyInfo post_build_dependency{};
    post_build_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    post_build_dependency.memoryBarrierCount = 1;
    post_build_dependency.pMemoryBarriers = &traversal_barrier;
    vkCmdPipelineBarrier2(vk_command_buffer, &post_build_dependency);
  });

  if (!same_pending_frame) {
    pending_retained_blas_references_ = committed_blas_references_;
    pending_extra_staging_buffers_.clear();
  }
  std::unordered_set<const BottomLevelAccelerationStructure*> retained_blas_references;
  retained_blas_references.reserve(pending_retained_blas_references_.size() + blas_references.size());
  for (const auto& blas : pending_retained_blas_references_) {
    retained_blas_references.emplace(blas.get());
  }
  for (const auto& blas : blas_references) {
    if (retained_blas_references.emplace(blas.get()).second) {
      pending_retained_blas_references_.emplace_back(blas);
    }
  }
  pending_ = true;
  pending_frame_index_ = frame_index;
  pending_frame_count_ = frame_count;
  if (!same_pending_frame) {
    pending_submission_state_ = Platform::TrackCurrentFrameSubmission();
  }
  pending_instances_ = std::move(instances);
  pending_blas_content_versions_ = std::move(blas_content_versions);
  pending_final_blas_references_ = std::move(blas_references);
  return mode;
}

TopLevelAccelerationStructure::~TopLevelAccelerationStructure() {
  Destroy();
}

VkAccelerationStructureKHR TopLevelAccelerationStructure::GetVkAccelerationStructure() const {
  return vk_acceleration_structure_khr_;
}

VkDeviceAddress TopLevelAccelerationStructure::GetDeviceAddress() const {
  return device_address_;
}
