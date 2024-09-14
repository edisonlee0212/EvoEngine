#include "GpuRayTracerCamera.hpp"

#include "ClassRegistry.hpp"
#include "EditorLayer.hpp"
#include "ProjectManager.hpp"
#include "RayTracer.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"
using namespace evo_engine;

float GpuRayTracerCamera::GetSizeRatio() const {
  if (resolution.x == 0 || resolution.y == 0)
    return 0;
  return static_cast<float>(resolution.x) / static_cast<float>(resolution.y);
}

void GpuRayTracerCamera::UpdateCameraInfoBlock(CameraInfoBlock& camera_info_block,
                                               const GlobalTransform& global_transform) const {
  const auto rotation = global_transform.GetRotation();
  const auto position = global_transform.GetPosition();
  const glm::vec3 front = rotation * glm::vec3(0, 0, -1);
  const glm::vec3 up = rotation * glm::vec3(0, 1, 0);
  const auto ratio = GetSizeRatio();
  camera_info_block.projection = glm::perspective(glm::radians(fov * 0.5f), ratio, near_distance, far_distance);
  camera_info_block.view = glm::lookAt(position, position + front, up);
  camera_info_block.projection_view = camera_info_block.projection * camera_info_block.view;
  camera_info_block.inverse_projection = glm::inverse(camera_info_block.projection);
  camera_info_block.inverse_view = glm::inverse(camera_info_block.view);
  camera_info_block.inverse_projection_view = glm::inverse(camera_info_block.projection * camera_info_block.view);
  camera_info_block.reserved_parameters1 =
      glm::vec4(near_distance, far_distance, glm::tan(glm::radians(fov * 0.5f)), glm::tan(glm::radians(fov * 0.25f)));
}

std::shared_ptr<DescriptorSetLayout> ray_tracer_camera_descriptor_set_layout;
std::shared_ptr<Shader> ray_tracer_camera_shader{};
std::shared_ptr<ComputePipeline> ray_tracer_camera_pipeline{};

struct CameraConfig {
  glm::vec4 resolution_xy_sample_bounce;
  glm::mat4 inverse_projection_view;
};
void GpuRayTracerCamera::OnCreate() {
  texture_ref = ProjectManager::CreateTemporaryAsset<Texture2D>();
  texture_ref.Get<Texture2D>()->SetResolution(resolution, false);
}

void GpuRayTracerCamera::OnDestroy() {
  texture_ref.Clear();
}

void GpuRayTracerCamera::LateUpdate() {
  if (const auto texture = texture_ref.Get<Texture2D>(); texture && per_frame_capture) {
    Capture();
  }
}

void GpuRayTracerCamera::Capture(const CaptureParameters& parameters, std::vector<glm::vec4>& pixels) const {
  std::shared_ptr<RenderInstances> render_instances;
  if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
    render_instances = render_layer->render_instances;
  } else {
    render_instances = std::make_shared<RenderInstances>(1);
  }
  pixels.resize(resolution.x * resolution.y);
  CameraInfoBlock camera_info_block;
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(owner);
  UpdateCameraInfoBlock(camera_info_block, global_transform);
  RayTracer cpu_ray_tracer;
  cpu_ray_tracer.Initialize(
      render_instances,
      [&](uint32_t, const std::shared_ptr<Mesh>&) {

      },
      [&](const uint32_t node_index, const Entity& entity) {

      });
  auto aggregate_scene = cpu_ray_tracer.Aggregate();

  std::vector<glm::vec4> colors(65536);
  for (auto& color : colors) {
    color = glm::vec4(glm::abs(glm::sphericalRand(1.f)), 1.f);
  }
  std::vector<RayTracer::RayDescriptor> ray_descriptors(resolution.x * resolution.y * parameters.sampled_count);
  Jobs::RunParallelFor(resolution.x * resolution.y, [&](const size_t pixel_index) {
    const float x_coordinate = pixel_index % resolution.y;
    const float y_coordinate = pixel_index / resolution.y;
    const float half_x = resolution.x * .5f;
    const float half_y = resolution.y * .5f;
    RandomSampler random_sampler;
    random_sampler.SetSeed(pixel_index);

    for (uint32_t sample_index = 0; sample_index < parameters.sampled_count; sample_index++) {
      const auto screen = glm::vec2((x_coordinate + random_sampler.Get1D() - half_x) / half_x,
                                    (y_coordinate + random_sampler.Get1D() - half_y) / half_y);
      auto start = camera_info_block.inverse_projection_view * glm::vec4(screen.x, screen.y, 0.0f, 1.0f);
      auto end = camera_info_block.inverse_projection_view * glm::vec4(screen.x, screen.y, 1.0f, 1.0f);
      start /= start.w;
      end /= end.w;
      auto& current_ray_descriptor = ray_descriptors[pixel_index * parameters.sampled_count + sample_index];

      current_ray_descriptor.origin = start;
      current_ray_descriptor.direction = glm::normalize(end - start);
    }
  });

  std::vector<RayTracer::HitInfo> hit_infos;
  aggregate_scene.TraceGpu(ray_descriptors, hit_infos, RayTracer::TraceFlags::None);
  Jobs::RunParallelFor(resolution.x * resolution.y, [&](const size_t pixel_index) {
    float hit_count = 0;
    float temp = 0;
    for (uint32_t sample_index = 0; sample_index < parameters.sampled_count; sample_index++) {
      const auto& hit_info = hit_infos[pixel_index * parameters.sampled_count + sample_index];
      if (hit_info.has_hit) {
        hit_count += 1;
        temp += static_cast<float>(cpu_ray_tracer.GetEntity(hit_info.node_index).GetIndex());
      }
    }
    if (hit_count > 0.f) {
      const size_t entity_index = temp / hit_count;
      pixels[pixel_index] = colors[entity_index];
    } else {
      pixels[pixel_index] = glm::vec4(0, 0, 0, 1);
    }
  });
}

void GpuRayTracerCamera::Capture() {
  std::shared_ptr<RenderInstances> render_instances;

  if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
    render_instances = render_layer->render_instances;
  } else {
    render_instances = std::make_shared<RenderInstances>(1);
  }
  CameraInfoBlock camera_info_block;
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(owner);
  UpdateCameraInfoBlock(camera_info_block, global_transform);
  RayTracer cpu_ray_tracer;
  cpu_ray_tracer.Initialize(
      render_instances,
      [&](uint32_t, const std::shared_ptr<Mesh>&) {

      },
      [&](const uint32_t node_index, const Entity& entity) {

      });
  auto aggregate_scene = cpu_ray_tracer.Aggregate();

  aggregate_scene.InitializeBuffers();

  if (!ray_tracer_camera_descriptor_set_layout) {
    ray_tracer_camera_descriptor_set_layout = std::make_shared<DescriptorSetLayout>();
    ray_tracer_camera_descriptor_set_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                   VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_camera_descriptor_set_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                   VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_camera_descriptor_set_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                   VK_SHADER_STAGE_COMPUTE_BIT, 0);

    ray_tracer_camera_descriptor_set_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                   VK_SHADER_STAGE_COMPUTE_BIT, 0);
    ray_tracer_camera_descriptor_set_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                   VK_SHADER_STAGE_COMPUTE_BIT, 0);
    /*
    ray_tracer_camera_descriptor_set_layout->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                   VK_SHADER_STAGE_COMPUTE_BIT, 0);
    */

    ray_tracer_camera_descriptor_set_layout->PushDescriptorBinding(
        6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
        VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT, Platform::Settings::max_texture_2d_resource_size);
    ray_tracer_camera_descriptor_set_layout->PushDescriptorBinding(
        7, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
        VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT, Platform::Settings::max_cubemap_resource_size);

    ray_tracer_camera_descriptor_set_layout->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                                   VK_SHADER_STAGE_COMPUTE_BIT, 0);

    ray_tracer_camera_descriptor_set_layout->Initialize();
  }
  if (!ray_tracer_camera_shader) {
    ray_tracer_camera_shader = ProjectManager::CreateTemporaryAsset<Shader>();
    ray_tracer_camera_shader->Set(ShaderType::Compute,
                                  std::filesystem::path("./DefaultResources") / "Shaders/Compute/RayTracerCamera.comp");
  }

  if (!ray_tracer_camera_pipeline) {
    ray_tracer_camera_pipeline = std::make_shared<ComputePipeline>();
    ray_tracer_camera_pipeline->descriptor_set_layouts.emplace_back(ray_tracer_camera_descriptor_set_layout);
    auto& push_constant_range = ray_tracer_camera_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(CameraConfig);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    ray_tracer_camera_pipeline->compute_shader = ray_tracer_camera_shader;
    ray_tracer_camera_pipeline->Initialize();
  }

  const auto ray_tracer_descriptor_set = std::make_shared<DescriptorSet>(ray_tracer_camera_descriptor_set_layout);

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto material_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info);
  const auto instance_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info);
  material_info_descriptor_buffer->UploadVector(render_instances->GetMaterialInfoBlocks());
  instance_info_descriptor_buffer->UploadVector(render_instances->GetInstanceInfoBlocks());

  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(0, aggregate_scene.aggregate_scene_graph_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(1, aggregate_scene.aggregate_scene_geometry_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(2, aggregate_scene.aggregate_scene_info_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(3, material_info_descriptor_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(4, instance_info_descriptor_buffer);

  TextureStorage::DeviceSync();
  TextureStorage::BindTexture2DToDescriptorSet(ray_tracer_descriptor_set, 6);
  TextureStorage::BindCubemapToDescriptorSet(ray_tracer_descriptor_set, 7);
  /*
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.size = sizeof(glm::vec4) * pixels.size();
  const auto pixel_buffer = std::make_shared<Buffer>(buffer_create_info);

  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(5, pixel_buffer);
  */

  const auto texture2d = texture_ref.Get<Texture2D>();
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    texture2d->GetImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
  });
  VkDescriptorImageInfo image_info;
  image_info.imageLayout = texture2d->GetImage()->GetLayout();
  image_info.imageView = texture2d->GetVkImageView();
  image_info.sampler = texture2d->GetVkSampler();
  ray_tracer_descriptor_set->UpdateImageDescriptorBinding(5, image_info);
  CameraConfig camera_config;
  camera_config.resolution_xy_sample_bounce = glm::vec4(
      glm::uintBitsToFloat(resolution.x), glm::uintBitsToFloat(resolution.y),
      glm::uintBitsToFloat(capture_parameters.sampled_count), glm::uintBitsToFloat(capture_parameters.bounce));
  camera_config.inverse_projection_view = camera_info_block.inverse_projection_view;
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    ray_tracer_camera_pipeline->Bind(vk_command_buffer);
    ray_tracer_camera_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                  ray_tracer_descriptor_set->GetVkDescriptorSet());
    ray_tracer_camera_pipeline->PushConstant(vk_command_buffer, 0, camera_config);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(resolution.x * resolution.y, 256), 1, 1);
  });
  Platform::WaitForDeviceIdle();
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    texture2d->GetImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });

  capture_parameters.sampled_count++;
}

bool GpuRayTracerCamera::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (EditorLayer::DragAndDropButton<Texture2D>(texture_ref, "Target texture"))
    changed = true;

  static glm::ivec2 new_resolution = {512, 512};
  if (ImGui::DragInt2("Resolution", &new_resolution.x, 1, 1, 4096)) {
    capture_parameters.sampled_count = 0;
    resolution = new_resolution;
    texture_ref.Get<Texture2D>()->SetResolution(resolution, false);
  }
  ImGui::Checkbox("Capture", &per_frame_capture);
  ImGui::Text((std::string("Accumulation count: ") + std::to_string(capture_parameters.sampled_count)).c_str());
  if (ImGui::Button("Restart accumulation")) {
    capture_parameters.sampled_count = 0;
  }
  if (const auto texture2d = texture_ref.Get<Texture2D>(); texture2d) {
    const auto texture_storage = texture2d->PeekTexture2DStorage();
    if (texture_storage.im_texture_id) {
      static float debug_scale = 0.25f;
      ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 10.0f);
      debug_scale = glm::clamp(debug_scale, 0.1f, 10.0f);
      ImGui::Image(texture_storage.im_texture_id,
                   ImVec2(texture_storage.image->GetExtent().width * debug_scale,
                          texture_storage.image->GetExtent().height * debug_scale),
                   ImVec2(0, 1), ImVec2(1, 0));
    }
  }
  return changed;
}
