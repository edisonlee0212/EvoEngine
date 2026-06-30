#include "RenderPasses/GaussianSplatPass.hpp"

#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "GaussianSplat.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <algorithm>
#include <cstddef>

using namespace evo_engine;

namespace {
constexpr uint32_t kGaussianSplatSortedIndicesFlag = 1u;
constexpr uint32_t kGaussianSplatCullWorkGroupSize = 256u;
constexpr uint32_t kGaussianSplatRadix = 256u;
constexpr uint32_t kGaussianSplatRadixPassCount = 4u;
constexpr uint32_t kGaussianSplatRadixPartitionSize = 4096u;
constexpr uint32_t kGaussianSplatIndirectDrawFlag = 2u;

[[nodiscard]] bool IsValidStorageBuffer(const std::shared_ptr<Buffer>& buffer) {
  return buffer && buffer->GetVkBuffer() != VK_NULL_HANDLE && buffer->GetSize() != 0;
}

[[nodiscard]] bool IsValidGpuPrepassCache(const GaussianSplatGpuPrepassCache& cache, const uint32_t splat_count) {
  return cache.valid && cache.capacity == splat_count && IsValidStorageBuffer(cache.visible_index_buffer) &&
         IsValidStorageBuffer(cache.depth_key_buffer) && IsValidStorageBuffer(cache.indirect_draw_buffer) &&
         IsValidStorageBuffer(cache.mesh_task_indirect_draw_buffer);
}

[[nodiscard]] bool IsValidGpuRadixSortCache(const GaussianSplatGpuPrepassCache& cache, const uint32_t splat_count) {
  const auto partition_count = Platform::DivUp(splat_count, kGaussianSplatRadixPartitionSize);
  return IsValidGpuPrepassCache(cache, splat_count) && cache.radix_partition_capacity >= partition_count &&
         IsValidStorageBuffer(cache.radix_scratch_index_buffer) &&
         IsValidStorageBuffer(cache.radix_scratch_key_buffer) &&
         IsValidStorageBuffer(cache.radix_global_histogram_buffer) &&
         IsValidStorageBuffer(cache.radix_partition_histogram_buffer);
}

[[nodiscard]] bool GpuRadixSortSupported() {
  return Platform::Initialized() && Platform::GetInstance().GetCapabilities().subgroup_size >= 32u;
}

void ConfigurePremultipliedAlphaBlend(GraphicsPipeline& pipeline) {
  if (pipeline.states.color_blend_attachment_states.empty()) {
    return;
  }
  auto& blend = pipeline.states.color_blend_attachment_states[0];
  blend.blendEnable = VK_TRUE;
  blend.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
  blend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
  blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
}

void ResetIndirectDrawBuffers(const VkCommandBuffer vk_command_buffer, const Buffer& indirect_draw_buffer,
                              const Buffer& mesh_task_indirect_draw_buffer) {
  indirect_draw_buffer.Fill(vk_command_buffer, offsetof(VkDrawIndirectCommand, vertexCount), sizeof(uint32_t), 6u);
  indirect_draw_buffer.Fill(vk_command_buffer, offsetof(VkDrawIndirectCommand, instanceCount), sizeof(uint32_t), 0u);
  indirect_draw_buffer.Fill(vk_command_buffer, offsetof(VkDrawIndirectCommand, firstVertex), sizeof(uint32_t), 0u);
  indirect_draw_buffer.Fill(vk_command_buffer, offsetof(VkDrawIndirectCommand, firstInstance), sizeof(uint32_t), 0u);
  mesh_task_indirect_draw_buffer.Fill(vk_command_buffer, offsetof(VkDrawMeshTasksIndirectCommandEXT, groupCountX),
                                      sizeof(uint32_t), 0u);
  mesh_task_indirect_draw_buffer.Fill(vk_command_buffer, offsetof(VkDrawMeshTasksIndirectCommandEXT, groupCountY),
                                      sizeof(uint32_t), 1u);
  mesh_task_indirect_draw_buffer.Fill(vk_command_buffer, offsetof(VkDrawMeshTasksIndirectCommandEXT, groupCountZ),
                                      sizeof(uint32_t), 1u);
  Platform::BufferMemoryBarrier(vk_command_buffer, indirect_draw_buffer);
  Platform::BufferMemoryBarrier(vk_command_buffer, mesh_task_indirect_draw_buffer);
}

void UpdateRadixSortDescriptorSet(
    const std::shared_ptr<DescriptorSet>& descriptor_set, const std::shared_ptr<Buffer>& indirect_draw_buffer,
    const std::shared_ptr<Buffer>& global_histogram_buffer, const std::shared_ptr<Buffer>& partition_histogram_buffer,
    const std::shared_ptr<Buffer>& key_input_buffer, const std::shared_ptr<Buffer>& key_output_buffer,
    const std::shared_ptr<Buffer>& value_input_buffer, const std::shared_ptr<Buffer>& value_output_buffer) {
  descriptor_set->UpdateBufferDescriptorBinding(0, indirect_draw_buffer);
  descriptor_set->UpdateBufferDescriptorBinding(1, global_histogram_buffer);
  descriptor_set->UpdateBufferDescriptorBinding(2, partition_histogram_buffer);
  descriptor_set->UpdateBufferDescriptorBinding(3, key_input_buffer);
  descriptor_set->UpdateBufferDescriptorBinding(4, key_output_buffer);
  descriptor_set->UpdateBufferDescriptorBinding(5, value_input_buffer);
  descriptor_set->UpdateBufferDescriptorBinding(6, value_output_buffer);
}

void RecordGaussianSplatCull(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                             const GaussianSplatCullPass::Parameters& parameters,
                             const std::shared_ptr<RenderInstanceStorage::GaussianSplatRenderInstanceCollection>&
                                 gaussian_splat_render_instances,
                             const uint32_t total_gaussian_splats) {
  if (!parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.per_frame_descriptor_set ||
      !parameters.descriptor_set_layout || !parameters.transient_resources || !parameters.camera ||
      total_gaussian_splats == 0u || !gaussian_splat_render_instances || gaussian_splat_render_instances->Empty()) {
    return;
  }

  ApplyGraphResourceBarriers(vk_command_buffer, context);
  parameters.pipeline->Bind(vk_command_buffer);
  parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         parameters.per_frame_descriptor_set->GetVkDescriptorSet());

  gaussian_splat_render_instances->ForEachRenderInstance(
      [&](const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance) {
        const auto gaussian_instance =
            std::dynamic_pointer_cast<RenderInstanceStorage::GaussianSplatRenderInstance>(render_instance);
        if (!gaussian_instance || !gaussian_instance->gaussian_splat) {
          return;
        }
        const auto splat_count = static_cast<uint32_t>(gaussian_instance->gaussian_splat->GetSplatCount());
        const auto splat_buffer = gaussian_instance->gaussian_splat->GetGpuDataBuffer();
        if (splat_count == 0u || !IsValidStorageBuffer(splat_buffer)) {
          return;
        }

        const auto* gpu_prepass = gaussian_instance->gaussian_splat->FindGpuPrepassCache(
            parameters.camera->GetHandle(), gaussian_instance->renderer_handle);
        if (!gpu_prepass || !IsValidGpuPrepassCache(*gpu_prepass, splat_count)) {
          return;
        }

        const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
        descriptor_set->UpdateBufferDescriptorBinding(0, splat_buffer);
        descriptor_set->UpdateBufferDescriptorBinding(1, gpu_prepass->visible_index_buffer);
        descriptor_set->UpdateBufferDescriptorBinding(2, gpu_prepass->depth_key_buffer);
        descriptor_set->UpdateBufferDescriptorBinding(3, gpu_prepass->indirect_draw_buffer);
        descriptor_set->UpdateBufferDescriptorBinding(4, gpu_prepass->mesh_task_indirect_draw_buffer);
        parameters.transient_resources->RetainDescriptorSet(descriptor_set);

        ResetIndirectDrawBuffers(vk_command_buffer, *gpu_prepass->indirect_draw_buffer,
                                 *gpu_prepass->mesh_task_indirect_draw_buffer);
        parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());

        GaussianSplatCullPushConstant push_constant{};
        push_constant.camera_instance_count_flags = glm::uvec4(
            parameters.camera_index, static_cast<uint32_t>(gaussian_instance->instance_index), splat_count, 0u);
        const float opacity_scale = gaussian_instance->opacity_scale > 0.0f ? gaussian_instance->opacity_scale : 0.0f;
        push_constant.opacity_extent_min_max = glm::vec4(opacity_scale, 2.8284271f, 1.0f, 192.0f);
        parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        parameters.pipeline->Dispatch(vk_command_buffer, Platform::DivUp(splat_count, kGaussianSplatCullWorkGroupSize));
        Platform::BufferMemoryBarrier(vk_command_buffer, *gpu_prepass->visible_index_buffer);
        Platform::BufferMemoryBarrier(vk_command_buffer, *gpu_prepass->depth_key_buffer);
        Platform::BufferMemoryBarrier(vk_command_buffer, *gpu_prepass->indirect_draw_buffer);
        Platform::BufferMemoryBarrier(vk_command_buffer, *gpu_prepass->mesh_task_indirect_draw_buffer);
      });
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}

void RecordGaussianSplatRadixSort(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                                  const GaussianSplatSortPass::Parameters& parameters,
                                  const std::shared_ptr<RenderInstanceStorage::GaussianSplatRenderInstanceCollection>&
                                      gaussian_splat_render_instances,
                                  const uint32_t total_gaussian_splats) {
  if (!GpuRadixSortSupported() || !parameters.upsweep_pipeline || !parameters.upsweep_pipeline->Initialized() ||
      !parameters.spine_pipeline || !parameters.spine_pipeline->Initialized() || !parameters.downsweep_pipeline ||
      !parameters.downsweep_pipeline->Initialized() || !parameters.descriptor_set_layout ||
      !parameters.transient_resources || !parameters.camera || total_gaussian_splats == 0u ||
      !gaussian_splat_render_instances || gaussian_splat_render_instances->Empty()) {
    return;
  }

  ApplyGraphResourceBarriers(vk_command_buffer, context);
  gaussian_splat_render_instances->ForEachRenderInstance(
      [&](const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance) {
        const auto gaussian_instance =
            std::dynamic_pointer_cast<RenderInstanceStorage::GaussianSplatRenderInstance>(render_instance);
        if (!gaussian_instance || !gaussian_instance->gaussian_splat ||
            gaussian_instance->sort_mode != GaussianSplatSortMode::GpuRadix) {
          return;
        }
        const auto splat_count = static_cast<uint32_t>(gaussian_instance->gaussian_splat->GetSplatCount());
        if (splat_count == 0u) {
          return;
        }
        const auto* gpu_prepass = gaussian_instance->gaussian_splat->FindGpuPrepassCache(
            parameters.camera->GetHandle(), gaussian_instance->renderer_handle);
        if (!gpu_prepass || !IsValidGpuRadixSortCache(*gpu_prepass, splat_count)) {
          return;
        }

        const auto partition_count = Platform::DivUp(splat_count, kGaussianSplatRadixPartitionSize);
        const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
        parameters.transient_resources->RetainDescriptorSet(descriptor_set);

        GaussianSplatRadixSortPushConstant push_constant{};
        push_constant.pass_count_partition_reserved.y = splat_count;
        push_constant.pass_count_partition_reserved.z = partition_count;
        for (uint32_t pass = 0u; pass < kGaussianSplatRadixPassCount; ++pass) {
          const bool odd_pass = (pass & 1u) != 0u;
          const auto key_input = odd_pass ? gpu_prepass->radix_scratch_key_buffer : gpu_prepass->depth_key_buffer;
          const auto key_output = odd_pass ? gpu_prepass->depth_key_buffer : gpu_prepass->radix_scratch_key_buffer;
          const auto value_input =
              odd_pass ? gpu_prepass->radix_scratch_index_buffer : gpu_prepass->visible_index_buffer;
          const auto value_output =
              odd_pass ? gpu_prepass->visible_index_buffer : gpu_prepass->radix_scratch_index_buffer;

          UpdateRadixSortDescriptorSet(
              descriptor_set, gpu_prepass->indirect_draw_buffer, gpu_prepass->radix_global_histogram_buffer,
              gpu_prepass->radix_partition_histogram_buffer, key_input, key_output, value_input, value_output);
          gpu_prepass->radix_global_histogram_buffer->Fill(vk_command_buffer, 0, kGaussianSplatRadix * sizeof(uint32_t),
                                                           0u);
          Platform::BufferMemoryBarrier(vk_command_buffer, *gpu_prepass->radix_global_histogram_buffer);

          push_constant.pass_count_partition_reserved.x = pass;
          parameters.upsweep_pipeline->Bind(vk_command_buffer);
          parameters.upsweep_pipeline->BindDescriptorSet(vk_command_buffer, 0, descriptor_set->GetVkDescriptorSet());
          parameters.upsweep_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          parameters.upsweep_pipeline->Dispatch(vk_command_buffer, partition_count);
          Platform::BufferMemoryBarrier(vk_command_buffer, *gpu_prepass->radix_global_histogram_buffer);
          Platform::BufferMemoryBarrier(vk_command_buffer, *gpu_prepass->radix_partition_histogram_buffer);

          parameters.spine_pipeline->Bind(vk_command_buffer);
          parameters.spine_pipeline->BindDescriptorSet(vk_command_buffer, 0, descriptor_set->GetVkDescriptorSet());
          parameters.spine_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          parameters.spine_pipeline->Dispatch(vk_command_buffer, kGaussianSplatRadix);
          Platform::BufferMemoryBarrier(vk_command_buffer, *gpu_prepass->radix_global_histogram_buffer);
          Platform::BufferMemoryBarrier(vk_command_buffer, *gpu_prepass->radix_partition_histogram_buffer);

          parameters.downsweep_pipeline->Bind(vk_command_buffer);
          parameters.downsweep_pipeline->BindDescriptorSet(vk_command_buffer, 0, descriptor_set->GetVkDescriptorSet());
          parameters.downsweep_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          parameters.downsweep_pipeline->Dispatch(vk_command_buffer, partition_count);
          Platform::BufferMemoryBarrier(vk_command_buffer, *key_output);
          Platform::BufferMemoryBarrier(vk_command_buffer, *value_output);
        }

        Platform::BufferMemoryBarrier(vk_command_buffer, *gpu_prepass->depth_key_buffer);
        Platform::BufferMemoryBarrier(vk_command_buffer, *gpu_prepass->visible_index_buffer);
      });
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}

void RecordGaussianSplats(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                          const GaussianSplatPass::Parameters& parameters,
                          const std::shared_ptr<RenderInstanceStorage::GaussianSplatRenderInstanceCollection>&
                              gaussian_splat_render_instances,
                          const uint32_t total_gaussian_splats) {
  const bool mesh_pipeline_available =
      parameters.use_mesh_shader && parameters.mesh_pipeline && parameters.mesh_pipeline->Initialized();
  if (!parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.per_frame_descriptor_set ||
      !parameters.descriptor_set_layout || !parameters.transient_resources || !parameters.camera ||
      !parameters.camera->GetRenderTexture() || total_gaussian_splats == 0u || !gaussian_splat_render_instances ||
      gaussian_splat_render_instances->Empty()) {
    return;
  }

  ApplyGraphResourceBarriers(vk_command_buffer, context);
  std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
  parameters.camera->GetRenderTexture()->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                    VK_ATTACHMENT_STORE_OP_STORE);
  auto depth_attachment = parameters.camera->GetRenderTexture()->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                                        VK_ATTACHMENT_STORE_OP_STORE);

  VkRect2D render_area{};
  render_area.offset = {0, 0};
  render_area.extent.width = parameters.camera->GetSize().x;
  render_area.extent.height = parameters.camera->GetSize().y;
  VkRenderingInfo render_info{};
  render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  render_info.renderArea = render_area;
  render_info.layerCount = 1;
  render_info.colorAttachmentCount = static_cast<uint32_t>(color_attachment_infos.size());
  render_info.pColorAttachments = color_attachment_infos.data();
  render_info.pDepthAttachment = parameters.use_scene_depth ? &depth_attachment : nullptr;

  Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
    const glm::ivec4 viewport{0, 0, static_cast<int>(parameters.camera->GetSize().x),
                              static_cast<int>(parameters.camera->GetSize().y)};
    std::shared_ptr<GraphicsPipeline> bound_pipeline;
    const auto bind_pipeline = [&](const std::shared_ptr<GraphicsPipeline>& pipeline) {
      if (bound_pipeline == pipeline) {
        return;
      }
      pipeline->states.ResetAllStates(color_attachment_infos.size());
      pipeline->states.SetViewportScissor(viewport);
      pipeline->states.depth_write = false;
      pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
      pipeline->states.cull_mode = VK_CULL_MODE_NONE;
      ConfigurePremultipliedAlphaBlend(*pipeline);
      pipeline->Bind(vk_command_buffer);
      pipeline->BindDescriptorSet(vk_command_buffer, 0, parameters.per_frame_descriptor_set->GetVkDescriptorSet());
      bound_pipeline = pipeline;
    };

    gaussian_splat_render_instances->ForEachRenderInstance(
        [&](const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance) {
          const auto gaussian_instance =
              std::dynamic_pointer_cast<RenderInstanceStorage::GaussianSplatRenderInstance>(render_instance);
          if (!gaussian_instance || !gaussian_instance->gaussian_splat) {
            return;
          }
          const auto splat_count = static_cast<uint32_t>(gaussian_instance->gaussian_splat->GetSplatCount());
          const auto splat_buffer = gaussian_instance->gaussian_splat->GetGpuDataBuffer();
          if (splat_count == 0u || !IsValidStorageBuffer(splat_buffer)) {
            return;
          }

          bool use_sorted_indices = false;
          bool use_indirect_draw = false;
          auto index_buffer = splat_buffer;
          const bool use_cpu_sort =
              gaussian_instance->sort_mode == GaussianSplatSortMode::CpuDepth ||
              (gaussian_instance->sort_mode == GaussianSplatSortMode::GpuRadix && !GpuRadixSortSupported());
          if (use_cpu_sort) {
            const auto* sort_cache = gaussian_instance->gaussian_splat->FindSortCache(
                parameters.camera->GetHandle(), gaussian_instance->renderer_handle);
            if (sort_cache && sort_cache->valid && sort_cache->indices.size() == splat_count &&
                IsValidStorageBuffer(sort_cache->index_buffer)) {
              use_sorted_indices = true;
              index_buffer = sort_cache->index_buffer;
            }
          } else if (gaussian_instance->sort_mode == GaussianSplatSortMode::None ||
                     gaussian_instance->sort_mode == GaussianSplatSortMode::GpuRadix) {
            const auto* gpu_prepass = gaussian_instance->gaussian_splat->FindGpuPrepassCache(
                parameters.camera->GetHandle(), gaussian_instance->renderer_handle);
            if (!gpu_prepass || !IsValidGpuPrepassCache(*gpu_prepass, splat_count)) {
              return;
            }
            use_sorted_indices = true;
            use_indirect_draw = true;
            index_buffer = gpu_prepass->visible_index_buffer;
          }

          const bool use_mesh_draw = mesh_pipeline_available && use_indirect_draw &&
                                     gaussian_instance->raster_mode != GaussianSplatRasterMode::Vertex;
          const auto active_pipeline = use_mesh_draw ? parameters.mesh_pipeline : parameters.pipeline;
          if (!active_pipeline || !active_pipeline->Initialized()) {
            return;
          }
          bind_pipeline(active_pipeline);

          const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
          descriptor_set->UpdateBufferDescriptorBinding(0, splat_buffer);
          descriptor_set->UpdateBufferDescriptorBinding(1, index_buffer);
          const auto requested_sh_degree = static_cast<uint32_t>(std::clamp(gaussian_instance->sh_degree, 0, 3));
          const auto available_sh_degree = gaussian_instance->gaussian_splat->GetSphericalHarmonicsDegree();
          auto effective_sh_degree = std::min(requested_sh_degree, available_sh_degree);
          auto rest_float_count = gaussian_instance->gaussian_splat->GetSphericalHarmonicsRestFloatCount();
          auto rest_buffer = splat_buffer;
          if (effective_sh_degree > 0u) {
            if (const auto candidate = gaussian_instance->gaussian_splat->GetSphericalHarmonicsRestBuffer();
                IsValidStorageBuffer(candidate)) {
              rest_buffer = candidate;
            } else {
              effective_sh_degree = 0u;
              rest_float_count = 0u;
            }
          }
          descriptor_set->UpdateBufferDescriptorBinding(2, rest_buffer);
          if (const auto* gpu_prepass = gaussian_instance->gaussian_splat->FindGpuPrepassCache(
                  parameters.camera->GetHandle(), gaussian_instance->renderer_handle);
              gpu_prepass && IsValidStorageBuffer(gpu_prepass->indirect_draw_buffer)) {
            descriptor_set->UpdateBufferDescriptorBinding(3, gpu_prepass->indirect_draw_buffer);
          } else {
            descriptor_set->UpdateBufferDescriptorBinding(3, splat_buffer);
          }
          parameters.transient_resources->RetainDescriptorSet(descriptor_set);
          active_pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());

          active_pipeline->states.depth_test =
              parameters.use_scene_depth && gaussian_instance->depth_mode == GaussianSplatDepthMode::SceneDepth;
          active_pipeline->states.ApplyAllStates(vk_command_buffer);

          GaussianSplatPushConstant push_constant{};
          uint32_t flags = use_sorted_indices ? kGaussianSplatSortedIndicesFlag : 0u;
          if (use_indirect_draw) {
            flags |= kGaussianSplatIndirectDrawFlag;
          }
          push_constant.camera_instance_count_flags = glm::uvec4(
              parameters.camera_index, static_cast<uint32_t>(gaussian_instance->instance_index), splat_count, flags);
          push_constant.sh_degree_rest_count_reserved = glm::uvec4(effective_sh_degree, rest_float_count, 0u, 0u);
          const float opacity_scale = gaussian_instance->opacity_scale > 0.0f ? gaussian_instance->opacity_scale : 0.0f;
          push_constant.opacity_extent_min_max = glm::vec4(opacity_scale, 2.8284271f, 1.0f, 192.0f);
          active_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          if (use_indirect_draw) {
            const auto* gpu_prepass = gaussian_instance->gaussian_splat->FindGpuPrepassCache(
                parameters.camera->GetHandle(), gaussian_instance->renderer_handle);
            if (use_mesh_draw) {
              Platform::DrawMeshTasksIndirect(vk_command_buffer, *gpu_prepass->mesh_task_indirect_draw_buffer, 0, 1,
                                              sizeof(VkDrawMeshTasksIndirectCommandEXT));
            } else {
              vkCmdDrawIndirect(vk_command_buffer, gpu_prepass->indirect_draw_buffer->GetVkBuffer(), 0, 1,
                                sizeof(VkDrawIndirectCommand));
            }
          } else {
            vkCmdDraw(vk_command_buffer, 6u, splat_count, 0u, 0u);
          }
        });
  });
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}
}  // namespace

RenderPassDescriptor GaussianSplatCullPass::CreateDescriptor(const char* dependency) {
  return {
      RenderPassNames::gaussian_splat_cull,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_render_instances, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_gaussian_splat_prepass, RenderResourceUsage::Write,
        RenderResourceState::StorageReadWrite}},
      {dependency ? dependency : RenderPassNames::deferred_camera}};
}

void GaussianSplatCullPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  const auto gaussian_splat_render_instances =
      parameters.render_instances ? parameters.render_instances->gaussian_splat_render_instances : nullptr;
  const auto total_gaussian_splats =
      parameters.render_instances ? parameters.render_instances->total_gaussian_splats : 0u;
  if (!parameters.record_commands || !parameters.camera || !parameters.render_instances ||
      total_gaussian_splats == 0u || !gaussian_splat_render_instances || gaussian_splat_render_instances->Empty()) {
    return;
  }

  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    RecordGaussianSplatCull(vk_command_buffer, context, parameters, gaussian_splat_render_instances,
                            total_gaussian_splats);
  });
}

RenderPassDescriptor GaussianSplatSortPass::CreateDescriptor(const char* dependency) {
  return {RenderPassNames::gaussian_splat_sort,
          RenderPassQueue::Graphics,
          RenderPassScope::Camera,
          {{RenderResourceNames::frame_render_instances, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
           {RenderResourceNames::camera_gaussian_splat_prepass, RenderResourceUsage::ReadWrite,
            RenderResourceState::StorageReadWrite}},
          {dependency ? dependency : RenderPassNames::gaussian_splat_cull}};
}

void GaussianSplatSortPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  const auto gaussian_splat_render_instances =
      parameters.render_instances ? parameters.render_instances->gaussian_splat_render_instances : nullptr;
  const auto total_gaussian_splats =
      parameters.render_instances ? parameters.render_instances->total_gaussian_splats : 0u;
  if (!parameters.record_commands || !parameters.camera || !parameters.render_instances ||
      total_gaussian_splats == 0u || !gaussian_splat_render_instances || gaussian_splat_render_instances->Empty()) {
    return;
  }

  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    RecordGaussianSplatRadixSort(vk_command_buffer, context, parameters, gaussian_splat_render_instances,
                                 total_gaussian_splats);
  });
}

RenderPassDescriptor GaussianSplatPass::CreateDescriptor(const char* dependency) {
  return {
      RenderPassNames::gaussian_splat,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_render_instances, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_gaussian_splat_prepass, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::DepthAttachment},
       {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::ColorAttachment}},
      {dependency ? dependency : RenderPassNames::deferred_camera}};
}

RenderPassDescriptor GaussianSplatPass::CreateOverlayDescriptor(const char* dependency) {
  return {
      RenderPassNames::gaussian_splat,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_render_instances, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_gaussian_splat_prepass, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::ColorAttachment}},
      {dependency ? dependency : RenderPassNames::ray_tracing_camera}};
}

void GaussianSplatPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  const auto gaussian_splat_render_instances =
      parameters.render_instances ? parameters.render_instances->gaussian_splat_render_instances : nullptr;
  const auto total_gaussian_splats =
      parameters.render_instances ? parameters.render_instances->total_gaussian_splats : 0u;
  if (!parameters.record_commands || !parameters.camera || !parameters.camera->GetRenderTexture() ||
      !parameters.render_instances || total_gaussian_splats == 0u || !gaussian_splat_render_instances ||
      gaussian_splat_render_instances->Empty()) {
    return;
  }

  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    RecordGaussianSplats(vk_command_buffer, context, parameters, gaussian_splat_render_instances,
                         total_gaussian_splats);
  });
}
