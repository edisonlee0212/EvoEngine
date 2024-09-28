#include "CpuRayTracerCamera.hpp"

#include "ClassRegistry.hpp"
#include "CpuRayTracer.hpp"

#include "EditorLayer.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
using namespace evo_engine;

float CpuRayTracerCamera::GetSizeRatio() const {
  if (resolution.x == 0 || resolution.y == 0)
    return 0;
  return static_cast<float>(resolution.x) / static_cast<float>(resolution.y);
}

void CpuRayTracerCamera::UpdateCameraInfoBlock(CameraInfoBlock& camera_info_block,
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

void CpuRayTracerCamera::OnCreate() {
}

void CpuRayTracerCamera::Capture(const CaptureParameters& parameters,
                                 const std::shared_ptr<Texture2D>& target_texture) const {
  std::shared_ptr<RenderInstances> render_instances;

  if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
    render_instances = render_layer->render_instances_list[Platform::GetCurrentFrameIndex()];
  } else {
    render_instances = std::make_shared<RenderInstances>();
  }
  std::vector<glm::vec4> pixels{resolution.x * resolution.y};
  CameraInfoBlock camera_info_block;
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(owner);
  UpdateCameraInfoBlock(camera_info_block, global_transform);
  CpuRayTracer cpu_ray_tracer;
  cpu_ray_tracer.Initialize(
      render_instances,
      [&](uint32_t, const std::shared_ptr<Mesh>&) {
      },
      [&](const uint32_t node_index, const Entity& entity) {
      });
  std::vector<glm::vec4> colors(65536);
  for (auto& color : colors) {
    color = glm::vec4(glm::abs(glm::sphericalRand(1.f)), 1.f);
  }
  const auto aggregated_scene = cpu_ray_tracer.Aggregate();
  Jobs::RunParallelFor(resolution.x * resolution.y, [&](const size_t pixel_index) {
    const float x_coordinate = pixel_index % resolution.y;
    const float y_coordinate = pixel_index / resolution.y;
    const float half_x = resolution.x * .5f;
    const float half_y = resolution.y * .5f;
    RandomSampler random_sampler;
    random_sampler.SetSeed(pixel_index);
    CpuRayTracer::RayDescriptor current_ray_descriptor{};
    float hit_count = 0;
    float temp = 0;
    for (uint32_t sample_index = 0; sample_index < parameters.sample; sample_index++) {
      const auto screen = glm::vec2((x_coordinate + random_sampler.Get1D() - half_x) / half_x,
                                    (y_coordinate + random_sampler.Get1D() - half_y) / half_y);
      auto start = camera_info_block.inverse_projection_view * glm::vec4(screen.x, screen.y, 0.0f, 1.0f);
      auto end = camera_info_block.inverse_projection_view * glm::vec4(screen.x, screen.y, 1.0f, 1.0f);
      start /= start.w;
      end /= end.w;
      current_ray_descriptor.origin = start;
      current_ray_descriptor.direction = glm::normalize(end - start);

      aggregated_scene.Trace(
          current_ray_descriptor,
          [&](const CpuRayTracer::HitInfo& hit_info) {
            hit_count += 1;
            temp += static_cast<float>(cpu_ray_tracer.GetEntity(hit_info.node_index).GetIndex());
          },
          [&]() {
          },
          [](const CpuRayTracer::HitInfo& hit_info) {
          });
    }
    if (hit_count > 0.f) {
      size_t entity_index = temp / hit_count;
      pixels[pixel_index] = colors[entity_index];
      // pixels[pixel_index] = glm::vec4(1);
    } else {
      pixels[pixel_index] = glm::vec4(0, 0, 0, 1);
    }
  });

  target_texture->SetRgbaChannelData(pixels, resolution, true);
}

bool CpuRayTracerCamera::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (!texture_ref.Get<Texture2D>()) {
    auto new_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
    new_texture->SetRgbaChannelData({glm::vec4(1.f)}, {1, 1});
    texture_ref = new_texture;
  }
  if (EditorLayer::DragAndDropButton<Texture2D>(texture_ref, "Target texture"))
    changed = true;

  static glm::ivec2 new_resolution = {64, 64};
  ImGui::DragInt2("Resolution", &new_resolution.x, 1, 1, 2048);

  if (const auto texture = texture_ref.Get<Texture2D>(); texture && ImGui::Button("Capture")) {
    resolution = new_resolution;
    Capture(capture_parameters, texture);
  }

  return changed;
}