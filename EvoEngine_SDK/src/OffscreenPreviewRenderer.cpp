#include "OffscreenPreviewRenderer.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "EnvironmentalLighting.hpp"
#include "EnvironmentalMap.hpp"
#include "GeometryStorage.hpp"
#include "GlobalReflectionProbe.hpp"
#include "Lights.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "RenderTexture.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
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
constexpr float kMaterialPresentationZoom = 1.32f;
constexpr float kMaterialBloomThreshold = 1.25f;
constexpr float kMaterialBloomIntensity = 0.5f;
constexpr float kMinCameraDistance = 0.05f;
constexpr float kDegenerateBoundHalfExtentRatio = 0.01f;
constexpr float kMinDegenerateBoundHalfExtent = 0.001f;

struct PreviewContext {
  std::filesystem::path project_path;
  std::shared_ptr<Scene> scene;
  std::shared_ptr<EnvironmentalLighting> lighting;
  std::shared_ptr<Texture2D> studio_texture;
  std::shared_ptr<EnvironmentalMap> studio_environment;
  std::shared_ptr<GlobalReflectionProbe> studio_reflection_probe;
  Entity subject;
  std::shared_ptr<MeshRenderer> mesh_renderer;
  std::shared_ptr<Camera> camera;
  std::shared_ptr<PostProcessingStack> post_processing_stack;
};

std::unique_ptr<PreviewContext> preview_context;

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
  GlobalTransform light_transform;
  light_transform.SetValue(glm::vec3(0.0f), glm::radians(glm::vec3(125.0f, -35.0f, 0.0f)), glm::vec3(1.0f));

  const auto configure_light = [&](const Entity& light_owner) {
    const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(light_owner).lock();
    if (!light) {
      return;
    }
    light->cast_shadow = false;
    light->diffuse = glm::vec3(1.0f);
    light->diffuse_brightness = 2.4f;
    SetEntityTransform(scene, light_owner, light_transform);
  };

  const auto* light_owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>();
  if (!light_owners || light_owners->empty()) {
    configure_light(scene->CreateEntity("Preview Directional Light"));
    return;
  }

  for (const auto& light_owner : *light_owners) {
    configure_light(light_owner);
  }
}

PreviewContext* GetPreviewContext() {
  const auto project_path = ProjectManager::GetProjectPath();
  if (preview_context && preview_context->project_path == project_path) {
    return preview_context.get();
  }

  auto context = std::make_unique<PreviewContext>();
  context->project_path = project_path;
  context->scene = AssetManager::CreateTemporaryAsset<Scene>();
  context->lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  if (!context->scene || !context->lighting) {
    return nullptr;
  }

  context->studio_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  const auto studio_path = Resources::GetDefaultResourcePath("Textures/MaterialPreview/neutral_studio.hdr");
  if (context->studio_texture && Serialization::LoadAsset(*context->studio_texture, studio_path)) {
    context->studio_environment = AssetManager::CreateTemporaryAsset<EnvironmentalMap>();
    context->studio_environment->ConstructFromTexture2D(context->studio_texture);
    context->lighting->indirect_environment_source.kind =
        EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap;
    context->lighting->indirect_environment_source.environmental_map = context->studio_environment;
    context->lighting->indirect_environment_source.rotation = 0.0f;
    if (auto cubemap_ref = context->studio_environment->environment_cubemap;
        const auto cubemap = cubemap_ref.Get<Cubemap>()) {
      context->studio_reflection_probe = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
      if (context->studio_reflection_probe->ConstructFromCubemap(cubemap)) {
        context->scene->global_reflection_probe_fallback = context->studio_reflection_probe;
      }
    }
  } else {
    context->studio_texture.reset();
    context->lighting->indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::Color;
  }
  context->lighting->environment_lighting_intensity = 1.0f;
  context->lighting->diffuse_fallback_intensity = 1.0f;
  context->lighting->ddgi_settings.runtime.enabled = false;
  context->scene->environmental_lighting = context->lighting;
  ConfigurePreviewLighting(context->scene);

  context->subject = context->scene->CreateEntity("Preview Subject");
  context->mesh_renderer = context->scene->GetOrSetPrivateComponent<MeshRenderer>(context->subject).lock();
  context->camera = context->scene->main_camera.Get<Camera>();
  if (!context->mesh_renderer || !context->camera || !context->scene->IsEntityValid(context->camera->GetOwner())) {
    return nullptr;
  }
  context->post_processing_stack = context->camera->post_processing_stack_ref.Get<PostProcessingStack>();
  if (context->post_processing_stack && context->post_processing_stack->bloom) {
    context->post_processing_stack->bloom->threshold = kMaterialBloomThreshold;
    context->post_processing_stack->bloom->intensity = kMaterialBloomIntensity;
  }
  if (context->post_processing_stack && context->post_processing_stack->tone_mapping) {
    auto& tone_mapping = *context->post_processing_stack->tone_mapping;
    tone_mapping.method = ToneMapping::ToneMapMethod::Aces;
    tone_mapping.exposure = 1.0f;
    tone_mapping.auto_exposure = false;
    tone_mapping.dither = false;
  }
  context->mesh_renderer->cast_shadow = false;
  preview_context = std::move(context);
  return preview_context.get();
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

void OffscreenPreviewRenderer::Reset() {
  preview_context.reset();
}

std::shared_ptr<Texture2D> OffscreenPreviewRenderer::RenderMaterial(const std::shared_ptr<Material>& material,
                                                                    const OffscreenPreviewSettings& settings) {
  if (!material) {
    return {};
  }

  const auto mesh = Resources::GetInstance().GetPrimitives().sphere;
  if (!mesh) {
    return {};
  }
  auto material_settings = settings;
  material_settings.camera_zoom *= kMaterialPresentationZoom;
  return RenderMeshWithMaterial(mesh, material, mesh->GetBound(), material_settings, true);
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
  return RenderMeshWithMaterial(mesh, preview_material, mesh->GetBound(), settings, false);
}

std::shared_ptr<Texture2D> OffscreenPreviewRenderer::RenderMeshWithMaterial(const std::shared_ptr<Mesh>& mesh,
                                                                            const std::shared_ptr<Material>& material,
                                                                            const Bound& focus_bound,
                                                                            const OffscreenPreviewSettings& settings,
                                                                            const bool material_presentation) {
  const auto application = ApplicationContext::TryGet();
  if (!application || !Platform::Initialized() || !Platform::TryGetGpuService() || !mesh || !material) {
    return {};
  }

  const auto render_layer = application->GetLayer<RenderLayer>();
  if (!render_layer) {
    return {};
  }

  const auto context = GetPreviewContext();
  if (!context) {
    return {};
  }
  const auto& scene = context->scene;
  const auto& mesh_renderer = context->mesh_renderer;
  const auto& camera = context->camera;
  if (!context->studio_environment) {
    context->lighting->indirect_environment_source.color = glm::vec3(settings.clear_color);
  }
  mesh_renderer->mesh.Set(mesh);
  mesh_renderer->material.Set(material);
  UploadPreviewResources(mesh, material);

  const auto subject_rotation = CreateSubjectRotation(settings);
  GlobalTransform subject_transform;
  subject_transform.SetValue(glm::vec3(0.0f), subject_rotation, glm::vec3(1.0f));
  SetEntityTransform(scene, context->subject, subject_transform);
  camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  camera->camera_settings.background_source = context->studio_environment
                                                  ? Camera::BackgroundSource::InheritEnvironmentalLighting
                                                  : Camera::BackgroundSource::ClearColor;
  camera->camera_settings.clear_color = settings.clear_color;
  camera->camera_settings.background_intensity = context->studio_environment ? 0.45f : settings.clear_color.a;
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
  if (material_presentation && context->post_processing_stack) {
    context->post_processing_stack->ProcessBloomAndToneMappingImmediately(camera);
  }
  return CopyColorTexture(camera, settings);
}

std::shared_ptr<Texture2D> OffscreenPreviewRenderer::CopyColorTexture(const std::shared_ptr<Camera>& camera,
                                                                      const OffscreenPreviewSettings& settings) {
  if (!camera || !camera->GetRenderTexture() || !camera->GetRenderTexture()->GetColorImage()) {
    return {};
  }

  const auto resolution = ClampResolution(settings.resolution);
  const auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  if (!texture) {
    return {};
  }

  const auto source = camera->GetRenderTexture()->GetColorImage();
  auto& target_storage = texture->RefTexture2DStorage();
  target_storage.Initialize(resolution, source->GetFormat(), true, 1);
  const auto target = target_storage.GetImage();
  if (!target) {
    return {};
  }

  Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
    const auto source_layout = source->GetLayout();
    source->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    target->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    VkImageCopy copy_region{};
    copy_region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copy_region.srcSubresource.layerCount = 1;
    copy_region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copy_region.dstSubresource.layerCount = 1;
    copy_region.extent = {resolution.x, resolution.y, 1};
    vkCmdCopyImage(command_buffer, source->GetVkImage(), VK_IMAGE_LAYOUT_GENERAL, target->GetVkImage(),
                   VK_IMAGE_LAYOUT_GENERAL, 1, &copy_region);
    source->TransitImageLayout(command_buffer, source_layout);
    target->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_GENERAL);
  });
  texture->red_channel = true;
  texture->green_channel = true;
  texture->blue_channel = true;
  texture->alpha_channel = true;
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
