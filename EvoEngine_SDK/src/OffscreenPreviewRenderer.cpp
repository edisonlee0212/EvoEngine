#include "OffscreenPreviewRenderer.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "GeometryStorage.hpp"
#include "Lights.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "RenderTexture.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "Texture2D.hpp"

#include <algorithm>
#include <cmath>
#include <glm/gtx/quaternion.hpp>
#include <vector>

using namespace evo_engine;

namespace {
constexpr uint32_t kMinPreviewResolution = 16;
constexpr uint32_t kMaxPreviewResolution = 512;
constexpr float kPreviewCameraFov = 120.0f;
constexpr float kMinCameraDistance = 0.05f;
constexpr float kDegenerateBoundHalfExtentRatio = 0.01f;
constexpr float kMinDegenerateBoundHalfExtent = 0.001f;

glm::uvec2 ClampResolution(const glm::uvec2& resolution) {
  return {std::clamp(resolution.x, kMinPreviewResolution, kMaxPreviewResolution),
          std::clamp(resolution.y, kMinPreviewResolution, kMaxPreviewResolution)};
}

bool IsFinite(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

glm::vec3 SafeNormalize(const glm::vec3& value, const glm::vec3& fallback) {
  const float length = glm::length(value);
  if (!std::isfinite(length) || length <= 0.0f) {
    return fallback;
  }
  return value / length;
}

Bound NormalizeBound(const Bound& bound) {
  Bound normalized_bound;
  if (IsFinite(bound.min) && IsFinite(bound.max) && bound.min.x <= bound.max.x && bound.min.y <= bound.max.y &&
      bound.min.z <= bound.max.z) {
    normalized_bound = bound;
  } else {
    normalized_bound.min = glm::vec3(-0.5f);
    normalized_bound.max = glm::vec3(0.5f);
  }

  const auto center = normalized_bound.Center();
  auto half_extents = normalized_bound.Size();
  const float reference_half_extent = std::max({half_extents.x, half_extents.y, half_extents.z, 0.5f});
  const float min_half_extent =
      std::max(reference_half_extent * kDegenerateBoundHalfExtentRatio, kMinDegenerateBoundHalfExtent);
  half_extents.x = std::max(half_extents.x, min_half_extent);
  half_extents.y = std::max(half_extents.y, min_half_extent);
  half_extents.z = std::max(half_extents.z, min_half_extent);
  normalized_bound.min = center - half_extents;
  normalized_bound.max = center + half_extents;
  return normalized_bound;
}

void SetEntityTransform(const std::shared_ptr<Scene>& scene, const Entity& entity, const GlobalTransform& transform) {
  Transform local_transform;
  local_transform.value = transform.value;
  scene->SetDataComponent(entity, local_transform);
  scene->SetDataComponent(entity, transform);
}

std::shared_ptr<Material> CreateDefaultMaterial() {
  const auto material = AssetManager::CreateTemporaryAsset<Material>();
  if (!material) {
    return {};
  }
  material->material_data.shade_material.pbr_base_color_factor = glm::vec4(glm::vec3(0.72f), 1.0f);
  material->material_data.shade_material.pbr_metallic_factor = 0.0f;
  material->material_data.shade_material.pbr_roughness_factor = 0.45f;
  material->MarkDirty();
  return material;
}

void ConfigurePreviewLighting(const std::shared_ptr<Scene>& scene) {
  const auto* light_owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>();
  if (!light_owners) {
    return;
  }

  GlobalTransform light_transform;
  light_transform.SetValue(glm::vec3(0.0f), glm::radians(glm::vec3(125.0f, -35.0f, 0.0f)), glm::vec3(1.0f));
  for (const auto& light_owner : *light_owners) {
    const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(light_owner).lock();
    if (!light) {
      continue;
    }
    light->cast_shadow = false;
    light->diffuse = glm::vec3(1.0f);
    light->diffuse_brightness = 2.4f;
    SetEntityTransform(scene, light_owner, light_transform);
  }
}

void UploadPreviewResources(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material) {
  if (!mesh || !material) {
    return;
  }

  GeometryStorage::WaitForPendingUploads();

  auto texture_refs = material->PeekTextureRefs();
  for (auto& texture_ref : texture_refs) {
    if (const auto texture = texture_ref.Get<Texture2D>()) {
      texture->UnsafeUploadDataImmediately();
    }
  }
}

float CalculateFitDistance(const Bound& bound, const glm::vec3& front, const glm::vec3& right, const glm::vec3& up,
                           const OffscreenPreviewSettings& settings) {
  const auto resolution = ClampResolution(settings.resolution);
  const float aspect_ratio = static_cast<float>(resolution.x) / static_cast<float>(resolution.y);
  const float vertical_tangent = std::tan(glm::radians(kPreviewCameraFov * 0.25f));
  const float horizontal_tangent = vertical_tangent * aspect_ratio;
  const float padding = std::max(settings.camera_distance_multiplier, 1.0f);
  const auto center = bound.Center();

  std::vector<glm::vec3> corners;
  bound.PopulateCorners(corners);

  float distance = kMinCameraDistance;
  for (const auto& corner : corners) {
    const auto offset = corner - center;
    const float projected_depth = glm::dot(offset, front);
    distance = std::max(distance, glm::abs(glm::dot(offset, right)) * padding / horizontal_tangent - projected_depth);
    distance = std::max(distance, glm::abs(glm::dot(offset, up)) * padding / vertical_tangent - projected_depth);
    distance = std::max(distance, kMinCameraDistance - projected_depth);
  }
  return distance / std::max(settings.camera_zoom, 0.01f);
}

glm::quat CreateSubjectRotation(const OffscreenPreviewSettings& settings) {
  return glm::angleAxis(settings.subject_rotation.y, glm::vec3(1.0f, 0.0f, 0.0f)) *
         glm::angleAxis(settings.subject_rotation.x, glm::vec3(0.0f, 1.0f, 0.0f));
}
}  // namespace

std::shared_ptr<Texture2D> OffscreenPreviewRenderer::RenderMaterial(const std::shared_ptr<Material>& material,
                                                                    const OffscreenPreviewSettings& settings) {
  if (!material) {
    return {};
  }

  const auto mesh = Resources::GetInstance().GetPrimitives().sphere;
  if (!mesh) {
    return {};
  }
  return RenderMeshWithMaterial(mesh, material, mesh->GetBound(), settings);
}

std::shared_ptr<Texture2D> OffscreenPreviewRenderer::RenderMesh(const std::shared_ptr<Mesh>& mesh,
                                                                const std::shared_ptr<Material>& material,
                                                                const OffscreenPreviewSettings& settings) {
  if (!mesh) {
    return {};
  }
  const auto preview_material = material ? material : CreateDefaultMaterial();
  if (!preview_material) {
    return {};
  }
  return RenderMeshWithMaterial(mesh, preview_material, mesh->GetBound(), settings);
}

std::shared_ptr<Texture2D> OffscreenPreviewRenderer::RenderMeshWithMaterial(const std::shared_ptr<Mesh>& mesh,
                                                                            const std::shared_ptr<Material>& material,
                                                                            const Bound& focus_bound,
                                                                            const OffscreenPreviewSettings& settings) {
  const auto application = ApplicationContext::TryGet();
  if (!application || !Platform::Initialized() || !Platform::TryGetGpuService() || !mesh || !material) {
    return {};
  }

  const auto render_layer = application->GetLayer<RenderLayer>();
  if (!render_layer) {
    return {};
  }

  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  if (!scene) {
    return {};
  }
  scene->environment.environment_type = Scene::EnvironmentType::Color;
  scene->environment.background_color = glm::vec3(settings.clear_color);
  scene->environment.background_intensity = settings.clear_color.a;
  scene->environment.ambient_light_intensity = 1.1f;
  scene->environment.volumetric_cloud_settings.enabled = false;
  scene->environment.ddgi_settings.runtime.enabled = false;
  ConfigurePreviewLighting(scene);

  const auto subject = scene->CreateEntity("Preview Subject");
  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(subject).lock();
  if (!mesh_renderer) {
    return {};
  }
  mesh_renderer->mesh.Set(mesh);
  mesh_renderer->material.Set(material);
  mesh_renderer->cast_shadow = false;
  UploadPreviewResources(mesh, material);

  const auto subject_rotation = CreateSubjectRotation(settings);
  GlobalTransform subject_transform;
  subject_transform.SetValue(glm::vec3(0.0f), subject_rotation, glm::vec3(1.0f));
  SetEntityTransform(scene, subject, subject_transform);

  const auto camera = scene->main_camera.Get<Camera>();
  if (!camera || !scene->IsEntityValid(camera->GetOwner())) {
    return {};
  }
  camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  camera->camera_settings.use_clear_color = true;
  camera->camera_settings.clear_color = settings.clear_color;
  camera->camera_settings.background_intensity = settings.clear_color.a;
  camera->camera_settings.fov = kPreviewCameraFov;
  camera->camera_settings.near_distance = 0.01f;
  camera->camera_settings.far_distance = 1000.0f;
  camera->Resize(ClampResolution(settings.resolution));

  auto preview_bound = NormalizeBound(focus_bound);
  preview_bound.ApplyTransform(glm::mat4_cast(subject_rotation));
  preview_bound = NormalizeBound(preview_bound);
  scene->SetBound(preview_bound);
  const auto camera_transform = CreateCameraTransform(preview_bound, settings);
  SetEntityTransform(scene, camera->GetOwner(), camera_transform);
  camera->SetRequireRendering(true);

  render_layer->RenderSceneToCameraImmediately(scene, camera_transform, camera);
  return ReadColorTexture(camera, settings);
}

std::shared_ptr<Texture2D> OffscreenPreviewRenderer::ReadColorTexture(const std::shared_ptr<Camera>& camera,
                                                                      const OffscreenPreviewSettings& settings) {
  if (!camera || !camera->GetRenderTexture() || !camera->GetRenderTexture()->GetColorImage()) {
    return {};
  }

  const auto resolution = ClampResolution(settings.resolution);
  std::vector<glm::vec4> pixels;
  Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
  image_buffer.CopyFromImage(*camera->GetRenderTexture()->GetColorImage());
  image_buffer.DownloadVector(pixels, resolution.x * resolution.y);

  const auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  if (!texture) {
    return {};
  }
  for (auto& pixel : pixels) {
    pixel = glm::clamp(pixel, glm::vec4(0.0f), glm::vec4(1.0f));
    pixel.a = 1.0f;
  }
  texture->SetRgbaChannelData(pixels, resolution, false);
  return texture;
}

GlobalTransform OffscreenPreviewRenderer::CreateCameraTransform(const Bound& focus_bound,
                                                                const OffscreenPreviewSettings& settings) {
  const auto bound = NormalizeBound(focus_bound);
  const auto center = bound.Center();
  const auto direction = SafeNormalize(glm::vec3(0.25f, 0.2f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f));
  const auto front = -direction;
  const auto right = SafeNormalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)), glm::vec3(1.0f, 0.0f, 0.0f));
  const auto up = SafeNormalize(glm::cross(right, front), glm::vec3(0.0f, 1.0f, 0.0f));
  const auto distance = CalculateFitDistance(bound, front, right, up, settings);
  const auto position = center - front * distance;

  GlobalTransform camera_transform;
  camera_transform.SetValue(position, glm::quatLookAt(front, up), glm::vec3(1.0f));
  return camera_transform;
}
