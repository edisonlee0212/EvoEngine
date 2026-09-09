#include "SDKInspectionAdapters.hpp"

#include <algorithm>

#include "Animation.hpp"
#include "AnimationPlayer.hpp"
#include "Animator.hpp"
#include "ApplicationContext.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "Cubemap.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalLighting.hpp"
#include "EnvironmentalMap.hpp"
#include "GaussianSplat.hpp"
#include "GaussianSplatRenderer.hpp"
#include "HddagiResources.hpp"
#include "InspectorRegistry.hpp"
#include "Jobs.hpp"
#include "LightProbe.hpp"
#include "Lights.hpp"
#include "LodGroup.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "Platform.hpp"
#include "PlayerController.hpp"
#include "PointCloud.hpp"
#include "PointCloudScanner.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProceduralNoise.hpp"
#include "ProceduralNoiseGenerators.hpp"
#include "ProceduralNoiseOperators.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "SdfgiDebug.hpp"
#include "SdfgiRuntime.hpp"
#include "Serialization.hpp"
#include "Shader.hpp"
#include "SkinnedMesh.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "SkyIllumination.hpp"
#include "Strands.hpp"
#include "StrandsRenderer.hpp"
#include "Texture2D.hpp"
#include "TextureStorage.hpp"
#include "Transform.hpp"
#include "UnknownPrivateComponent.hpp"
#include "Utilities.hpp"
#include "WayPoints.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <unordered_map>

using namespace evo_engine;

namespace {
namespace pn = evo_engine::procedural_noise;
using NoiseGraph = NodeGraph<pn::InputPinData, pn::OutputPinData, pn::NodeData, int>;

bool InspectCameraBackground(const std::shared_ptr<EditorLayer>& editor_layer, CameraSettings::BackgroundSource& source,
                             float* intensity, glm::vec4& clear_color, AssetRef& cubemap, AssetRef& environmental_map) {
  if (!editor_layer || !ImGui::TreeNodeEx("Background", ImGuiTreeNodeFlags_DefaultOpen)) {
    return false;
  }
  bool changed = false;
  uint32_t source_index = static_cast<uint32_t>(source);
  if (ImGui::Combo("Source", Camera::GetBackgroundSourceNames(), source_index)) {
    source = Camera::NormalizeBackgroundSource(source_index);
    changed = true;
  }
  if (intensity) {
    changed = ImGui::DragFloat("Intensity", intensity, 0.01f, 0.0f, 10.0f) || changed;
  }
  switch (source) {
    case CameraSettings::BackgroundSource::ClearColor:
      changed = ImGui::ColorEdit4("Clear Color", reinterpret_cast<float*>(&clear_color)) || changed;
      break;
    case CameraSettings::BackgroundSource::Cubemap:
      changed = editor_layer->DragAndDropButton<Cubemap>(cubemap, "Skybox") || changed;
      break;
    case CameraSettings::BackgroundSource::EnvironmentalMap:
      changed = editor_layer->DragAndDropButton<EnvironmentalMap>(environmental_map, "Environmental Map") || changed;
      break;
    case CameraSettings::BackgroundSource::InheritEnvironmentalLighting:
    case CameraSettings::BackgroundSource::EngineDefaultSkybox:
      break;
  }
  ImGui::TreePop();
  return changed;
}

const char* polygon_mode_string[]{"Point", "Line", "Fill"};
const char* culling_mode_string[]{"Front", "Back", "FrontAndBack", "None"};
const char* blending_factor_string[]{"Zero",
                                     "One",
                                     "SrcColor",
                                     "OneMinusSrcColor",
                                     "DstColor",
                                     "OneMinusDstColor",
                                     "SrcAlpha",
                                     "OneMinusSrcAlpha",
                                     "DstAlpha",
                                     "OneMinusDstAlpha",
                                     "ConstantColor",
                                     "OneMinusConstantColor",
                                     "ConstantAlpha",
                                     "OneMinusConstantAlpha",
                                     "SrcAlphaSaturate",
                                     "Src1Color",
                                     "OneMinusSrc1Color",
                                     "Src1Alpha",
                                     "OneMinusSrc1Alpha"};

std::string FormatRenderCounter(const size_t value) {
  if (value < 999) {
    return std::to_string(value);
  }
  if (value < 999999) {
    return std::to_string(static_cast<int>(value / 1000)) + "K";
  }
  return std::to_string(static_cast<int>(value / 1000000)) + "M";
}

bool HasDrawStats(const RenderPassDrawStats& stats) {
  return stats.TotalDrawCalls() != 0 || stats.indirect_draw_commands != 0 || stats.prim_count != 0;
}

size_t SaturatingSubtract(const size_t value, const size_t subtraction) {
  return value > subtraction ? value - subtraction : 0;
}

void SubtractDrawStats(RenderPassDrawStats& stats, const RenderPassDrawStats& subtraction) {
  stats.direct_draw_calls = SaturatingSubtract(stats.direct_draw_calls, subtraction.direct_draw_calls);
  stats.indirect_draw_calls = SaturatingSubtract(stats.indirect_draw_calls, subtraction.indirect_draw_calls);
  stats.indirect_draw_commands = SaturatingSubtract(stats.indirect_draw_commands, subtraction.indirect_draw_commands);
  stats.prim_count = SaturatingSubtract(stats.prim_count, subtraction.prim_count);
}

RenderPassDrawStats TotalDrawStats(
    const std::array<RenderPassDrawStats, Platform::kRenderPassDrawBucketCount>& pass_stats) {
  RenderPassDrawStats total;
  for (const auto& stats : pass_stats) {
    total.direct_draw_calls += stats.direct_draw_calls;
    total.indirect_draw_calls += stats.indirect_draw_calls;
    total.indirect_draw_commands += stats.indirect_draw_commands;
    total.prim_count += stats.prim_count;
  }
  return total;
}

std::string BuildCameraDrawStatsLabel(const RenderCameraDrawStats& stats, const std::shared_ptr<Scene>& scene) {
  if (stats.scene_camera) {
    return "Scene";
  }
  if (stats.entity_index != 0 && scene) {
    const Scene& scene_ref = *scene;
    const auto entity = scene_ref.GetEntity(static_cast<size_t>(stats.entity_index));
    if (scene->IsEntityValid(entity)) {
      return std::to_string(entity.GetIndex()) + ": " + scene->GetEntityName(entity);
    }
    return std::to_string(stats.entity_index) + ": <deleted camera>";
  }
  return "Camera " + std::to_string(stats.camera_handle);
}

void DrawRenderPassStatsRows(const std::array<RenderPassDrawStats, Platform::kRenderPassDrawBucketCount>& pass_stats) {
  for (size_t bucket_index = 0; bucket_index < Platform::kRenderPassDrawBucketCount; bucket_index++) {
    const auto& stats = pass_stats[bucket_index];
    if (!HasDrawStats(stats)) {
      continue;
    }
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(Platform::GetRenderPassDrawBucketName(static_cast<RenderPassDrawBucket>(bucket_index)));
    ImGui::TableNextColumn();
    ImGui::Text("%llu", static_cast<unsigned long long>(stats.direct_draw_calls));
    ImGui::TableNextColumn();
    ImGui::Text("%llu", static_cast<unsigned long long>(stats.indirect_draw_calls));
    ImGui::TableNextColumn();
    ImGui::Text("%llu", static_cast<unsigned long long>(stats.indirect_draw_commands));
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(FormatRenderCounter(stats.prim_count).c_str());
  }
}

void DrawRenderPassStatsTable(const char* id,
                              const std::array<RenderPassDrawStats, Platform::kRenderPassDrawBucketCount>& pass_stats) {
  if (!ImGui::BeginTable(id, 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable)) {
    return;
  }
  ImGui::TableSetupColumn("Pass");
  ImGui::TableSetupColumn("Direct");
  ImGui::TableSetupColumn("Indirect");
  ImGui::TableSetupColumn("Records");
  ImGui::TableSetupColumn("Prims");
  ImGui::TableHeadersRow();
  DrawRenderPassStatsRows(pass_stats);
  ImGui::EndTable();
}

int ShaderStringResizeCallback(ImGuiInputTextCallbackData* data) {
  if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
    const auto shader_code = static_cast<std::string*>(data->UserData);
    IM_ASSERT(shader_code->data() == data->Buf);
    shader_code->resize(data->BufSize);
    data->Buf = shader_code->data();
  }
  return 0;
}

bool InspectBone(Bone& bone) {
  bool changed = false;
  if (ImGui::TreeNode((bone.name + "##" + std::to_string(bone.index)).c_str())) {
    ImGui::Text("Controller: ");
    ImGui::SameLine();
    for (auto& child : bone.children) {
      if (InspectBone(*child))
        changed = true;
    }
    ImGui::TreePop();
  }
  return changed;
}

bool InspectAnimation(InspectorContext&, Animation& animation) {
  if (!animation.root_bone) {
    return false;
  }
  bool changed = false;
  ImGui::Text(("Bone size: " + std::to_string(animation.bone_size)).c_str());
  if (InspectBone(*animation.root_bone))
    changed = true;
  return changed;
}

bool InspectAnimator(InspectorContext& context, Animator& animator) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  AssetRef animation_ref(animator.GetAnimation());
  const auto previous_handle = animation_ref.GetAssetHandle();
  if (editor_layer->DragAndDropButton<Animation>(animation_ref, "Animation") &&
      animation_ref.GetAssetHandle() != previous_handle) {
    if (const auto animation = animation_ref.Get<Animation>()) {
      animator.Setup(animation);
    } else {
      animator.ClearAnimation();
    }
    changed = true;
  }

  const auto animation = animator.GetAnimation();
  if (!animation || animation->IsEmpty() || animator.GetBoneSize() == 0) {
    return changed;
  }

  auto active_animation_name = animator.GetCurrentAnimationName();
  if (!animation->HasAnimation(active_animation_name)) {
    active_animation_name = animation->GetFirstAvailableAnimationName();
    animator.Animate(active_animation_name, 0.0f);
    changed = true;
  }

  if (ImGui::BeginCombo("Animations##Animator", active_animation_name.c_str())) {
    for (const auto& i : animation->UnsafeGetAnimationLengths()) {
      const bool selected = active_animation_name == i.first;
      if (ImGui::Selectable(i.first.c_str(), selected)) {
        animator.Animate(i.first, 0.0f);
        active_animation_name = i.first;
        changed = true;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }

  float animation_time = animator.GetCurrentAnimationTimePoint();
  if (ImGui::SliderFloat("Animation time", &animation_time, 0.0f,
                         animation->GetAnimationLength(animator.GetCurrentAnimationName()))) {
    animator.Animate(animation_time);
    changed = true;
  }
  return changed;
}

bool InspectAnimationPlayer(InspectorContext&, AnimationPlayer& animation_player) {
  bool changed = false;
  if (ImGui::Checkbox("AutoPlay", &animation_player.auto_play)) {
    changed = true;
  }
  if (animation_player.auto_play && ImGui::DragFloat("AutoPlay Speed", &animation_player.auto_play_speed, 1.0f)) {
    changed = true;
  }
  return changed;
}

bool InspectUnknownPrivateComponent(InspectorContext&, UnknownPrivateComponent& component) {
  ImGui::Text("Missing private component type: %s", component.GetOriginalTypeName().c_str());
  return false;
}

bool InspectUnknownAsset(InspectorContext&, UnknownAsset& asset) {
  ImGui::Text("Missing asset type: %s", asset.GetOriginalTypeName().c_str());
  return false;
}

bool InspectUnknownSystem(InspectorContext&, UnknownSystem& system) {
  ImGui::Text("Missing system type: %s", system.GetOriginalTypeName().c_str());
  return false;
}

bool InspectUnknownLayer(InspectorContext&, UnknownLayer& layer) {
  const auto window_title = layer.GetLayerName();
  bool open = layer.enable_inspection;
  if (!ImGui::Begin(window_title.c_str(), &open)) {
    ImGui::End();
    layer.enable_inspection = open;
    return false;
  }
  ImGui::Text("Missing layer type: %s", layer.GetOriginalTypeName().c_str());
  ImGui::End();
  layer.enable_inspection = open;
  return false;
}

bool InspectEditorLayer(InspectorContext& context, EditorLayer& layer) {
  layer.DrawLayerSettingsWindow(context.editor_layer);
  return false;
}

bool InspectPlayerController(InspectorContext&, PlayerController& controller) {
  bool changed = false;

  if (ImGui::DragFloat("Velocity", &controller.velocity, 0.01f))
    changed = true;
  if (ImGui::DragFloat("Mouse sensitivity", &controller.sensitivity, 0.01f))
    changed = true;
  if (ImGui::DragFloat("Yaw angle", &controller.RefSceneCameraYawAngle(), 0.01f))
    changed = true;
  if (ImGui::DragFloat("Pitch angle", &controller.RefSceneCameraPitchAngle(), 0.01f))
    changed = true;

  return changed;
}

bool InspectPointCloudScanner(InspectorContext& context, PointCloudScanner& scanner) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  if (ImGui::DragFloat("Angle", &scanner.rotate_angle, 0.1f, -90.0f, 90.0f))
    changed = true;
  if (ImGui::DragFloat2("Size", &scanner.size.x, 0.1f))
    changed = true;
  if (ImGui::DragFloat2("Distance", &scanner.distance.x, 0.001f, 1.0f, 0.001f))
    changed = true;

  const auto scene = scanner.GetScene();
  static glm::vec4 color = glm::vec4(0, 1, 0, 0.5);
  if (ImGui::ColorEdit4("Color", &color.x))
    changed = true;
  static bool render_plane = true;
  ImGui::Checkbox("Render plane", &render_plane);
  const auto gt = scene->GetDataComponent<GlobalTransform>(scanner.GetOwner());
  const auto front = glm::normalize(gt.GetRotation() * glm::vec3(0, 0, -1));
  const auto up = glm::normalize(gt.GetRotation() * glm::vec3(0, 1, 0));
  const glm::vec3 actual_vector = glm::rotate(front, glm::radians(scanner.rotate_angle), up);
  if (render_plane) {
    editor_layer->DrawGizmoMesh(Resources::GetInstance().GetPrimitives().quad, glm::vec4(1, 0, 0, 0.5),
                                glm::translate(gt.GetPosition() + front * 0.5f) *
                                    glm::mat4_cast(glm::quatLookAt(up, glm::normalize(actual_vector))) *
                                    glm::scale(glm::vec3(0.1, 0.5, 0.1f)),
                                1.0f);
    editor_layer->DrawGizmoMesh(Resources::GetInstance().GetPrimitives().quad, color,
                                glm::translate(gt.GetPosition()) * glm::mat4_cast(glm::quatLookAt(up, front)) *
                                    glm::scale(glm::vec3(scanner.size.x / 2.0f, 1.0, scanner.size.y / 2.0f)),
                                1.0f);
  }
  if (ImGui::Button("Scan")) {
    scanner.Scan();
    changed = true;
  }

  ImGui::Text("Sample amount: %zu", scanner.points.size());
  if (!scanner.points.empty()) {
    if (ImGui::Button("Clear")) {
      scanner.points.clear();
      scanner.point_colors.clear();
    }
    ImGui::Text("Construct PointCloud");
    ImGui::SameLine();
    if (editor_layer->DragAndDropButton<PointCloud>(scanner.point_cloud_drop_ref, "Here", false)) {
      if (const auto ptr = scanner.point_cloud_drop_ref.Get<PointCloud>()) {
        scanner.ConstructPointCloud(ptr);
      }
      scanner.point_cloud_drop_ref.Clear();
    }
  }
  return changed;
}

bool InspectPrefabComponents(const Prefab& prefab) {
  bool changed = false;
  if (ImGui::TreeNode("Data Components")) {
    for (const auto& i : prefab.data_components) {
      ImGui::Text(("Type: " + i.data_component_type.type_name).c_str());
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Private Components")) {
    for (const auto& i : prefab.private_components) {
      ImGui::Text(("Type: " + i.private_component->GetTypeName()).c_str());
    }
    ImGui::TreePop();
  }
  return changed;
}

bool InspectPrefabWalker(const std::shared_ptr<Prefab>& walker) {
  bool changed = false;
  ImGui::Text(("Name: " + walker->instance_name).c_str());

  if (InspectPrefabComponents(*walker))
    changed = true;

  if (!walker->child_prefabs.empty()) {
    if (ImGui::TreeNode("Children")) {
      for (const auto& child : walker->child_prefabs) {
        if (InspectPrefabWalker(child))
          changed = true;
      }
      ImGui::TreePop();
    }
  }
  return changed;
}

bool InspectPrefab(InspectorContext& context, Prefab& prefab) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  if (ImGui::Button("Instantiate")) {
    prefab.ToEntity(ApplicationContext::Get().GetActiveScene());
  }
  if (prefab.collected_assets.empty() && ImGui::Button("Collect assets"))
    prefab.GatherAssets();
  if (!prefab.collected_assets.empty()) {
    if (ImGui::TreeNode("Assets")) {
      for (auto& i : prefab.collected_assets) {
        const auto ptr = i.second.Get<IAsset>();
        if (!ptr) {
          continue;
        }
        const std::string tag = "##" + ptr->GetTypeName() + std::to_string(ptr->GetHandle());
        ImGui::Button((ptr->GetTitle() + tag).c_str());
        EditorLayer::DraggableAsset(ptr);
        EditorLayer::Rename(i.second);
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
          editor_layer->OpenAssetInspector(ptr);
        }
      }
      ImGui::TreePop();
    }
  }

  if (ImGui::TreeNode("Prefab Hierarchy")) {
    ImGui::Text((prefab.instance_name + " (root)").c_str());
    if (InspectPrefabComponents(prefab))
      changed = true;

    if (!prefab.child_prefabs.empty()) {
      if (ImGui::TreeNode("Children")) {
        for (const auto& child : prefab.child_prefabs) {
          if (InspectPrefabWalker(child))
            changed = true;
        }
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }
  return changed;
}

bool InspectCamera(InspectorContext& context, Camera& camera) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  uint32_t mode = static_cast<uint32_t>(camera.camera_render_mode);
  if (ImGui::Combo("Render Mode", Camera::GetCameraRenderModeNames(), mode)) {
    camera.camera_render_mode = Camera::NormalizeCameraRenderMode(mode);
    camera.ResetFrameCount();
    changed = true;
  }
  if (ImGui::DragFloat("Fade ratio", &camera.camera_settings.fade_ratio, 0.01f, 0.01f, 1.0f)) {
    changed = true;
  }
  if (camera.camera_settings.fade_ratio != 0.f &&
      ImGui::DragFloat("Fade factor", &camera.camera_settings.fade_factor, 0.01f, 0.01f, 1.0f)) {
    changed = true;
  }
  if (Camera::IsRayCameraRenderMode(camera.camera_render_mode)) {
    uint32_t debug_view = static_cast<uint32_t>(camera.camera_settings.ray_debug_view);
    if (ImGui::Combo("Ray Debug View", Camera::GetRayDebugViewNames(), debug_view)) {
      camera.camera_settings.ray_debug_view = Camera::NormalizeRayDebugView(debug_view);
      camera.ResetFrameCount();
      changed = true;
    }
    if (ImGui::DragFloat("Gamma", &camera.camera_settings.gamma, 0.01f, 0.01f, 10.0f)) {
      changed = true;
    }
    const char* sample_label = camera.camera_settings.auto_spp_enabled ? "Samples/frame" : "Samples";
    if (ImGui::SliderInt(sample_label, &camera.camera_settings.sample_size, 1, 32)) {
      changed = true;
    }
    if (ImGui::SliderInt("Bounce", &camera.camera_settings.bounce, 1, 8)) {
      changed = true;
    }
  }
  if (Camera::IsRayCameraRenderMode(camera.camera_render_mode)) {
    if (ImGui::DragFloat("Firefly threshold", &camera.camera_settings.firefly_clamp_threshold, 0.1f, 0.0f, 1000.0f,
                         "%.2f")) {
      camera.camera_settings.firefly_clamp_threshold = glm::max(camera.camera_settings.firefly_clamp_threshold, 0.0f);
      camera.ResetFrameCount();
      changed = true;
    }
    if (ImGui::Checkbox("Auto SPP", &camera.camera_settings.auto_spp_enabled)) {
      camera.ResetFrameCount();
      changed = true;
    }
    if (camera.camera_settings.auto_spp_enabled) {
      if (ImGui::DragInt("Auto min SPP", &camera.camera_settings.auto_spp_min_samples, 1, 1, 4096)) {
        camera.camera_settings.auto_spp_min_samples = glm::max(camera.camera_settings.auto_spp_min_samples, 1);
        camera.camera_settings.auto_spp_max_samples =
            glm::max(camera.camera_settings.auto_spp_max_samples, camera.camera_settings.auto_spp_min_samples);
        camera.ResetFrameCount();
        changed = true;
      }
      if (ImGui::DragInt("Auto max SPP", &camera.camera_settings.auto_spp_max_samples, 1,
                         camera.camera_settings.auto_spp_min_samples, 8192)) {
        camera.camera_settings.auto_spp_max_samples =
            glm::max(camera.camera_settings.auto_spp_max_samples, camera.camera_settings.auto_spp_min_samples);
        camera.ResetFrameCount();
        changed = true;
      }
      if (ImGui::DragFloat("Auto threshold", &camera.camera_settings.auto_spp_convergence_threshold, 0.001f, 0.0f, 1.0f,
                           "%.4f")) {
        camera.camera_settings.auto_spp_convergence_threshold =
            glm::max(camera.camera_settings.auto_spp_convergence_threshold, 0.0f);
        camera.ResetFrameCount();
        changed = true;
      }
    }
  }
  if (camera.camera_render_mode == Camera::CameraRenderMode::RayTracing) {
    uint32_t ser_mode = static_cast<uint32_t>(camera.camera_settings.shader_execution_reordering_mode);
    if (ImGui::Combo("Shader Execution Reordering", Camera::GetShaderExecutionReorderingModeNames(), ser_mode)) {
      camera.camera_settings.shader_execution_reordering_mode =
          Camera::NormalizeShaderExecutionReorderingMode(ser_mode);
      camera.ResetFrameCount();
      changed = true;
    }
    if (!Platform::ShaderExecutionReorderingEnabled() && camera.camera_settings.shader_execution_reordering_mode !=
                                                             CameraSettings::ShaderExecutionReorderingMode::Disabled) {
      ImGui::TextUnformatted("SER unavailable; using standard ray tracing scheduling.");
    }
  }
  if (ImGui::TreeNode("Debug")) {
    camera.SetRequireRendering(true);
    static bool external_window = false;
    ImGui::Checkbox("Display in external window", &external_window);

    static float debug_scale = 0.25f;
    if (camera.Rendered()) {
      if (external_window) {
        if (ImGui::Begin("Camera Debug")) {
          ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
          debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
          DrawCameraDebugViews(camera, debug_scale);
        }
        ImGui::End();
      } else {
        ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
        debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
        DrawCameraDebugViews(camera, debug_scale);
      }
    }
    ImGui::TreePop();
  }

  changed = InspectCameraBackground(editor_layer, camera.camera_settings.background_source,
                                    &camera.camera_settings.background_intensity, camera.camera_settings.clear_color,
                                    camera.skybox, camera.background_environment) ||
            changed;

  if (const auto scene = camera.GetScene()) {
    const bool saved_state = (scene->main_camera.Get<Camera>().get() == &camera);
    bool is_main_camera = saved_state;
    ImGui::Checkbox("Main Camera", &is_main_camera);
    if (saved_state != is_main_camera) {
      changed = true;
      if (is_main_camera) {
        scene->main_camera = scene->GetOrSetPrivateComponent<Camera>(camera.GetOwner()).lock();
      } else {
        ApplicationContext::Get().GetActiveScene()->main_camera.Clear();
      }
    }
    if (!is_main_camera || !ApplicationContext::Get().GetLayer<EditorLayer>()->main_camera_allow_auto_resize) {
      glm::ivec2 resolution = camera.GetSize();
      if (ImGui::DragInt2("Resolution", &resolution.x, 1, 1, 4096)) {
        camera.Resize({resolution.x, resolution.y});
      }
    }
  }
  if (editor_layer->DragAndDropButton<PostProcessingStack>(camera.post_processing_stack_ref, "PostProcessingStack")) {
    changed = true;
  }
  if (ImGui::TreeNode("Intrinsic Settings")) {
    if (ImGui::DragFloat("Near", &camera.camera_settings.near_distance, camera.camera_settings.near_distance / 10.0f, 0,
                         camera.camera_settings.far_distance)) {
      changed = true;
    }
    if (ImGui::DragFloat("Far", &camera.camera_settings.far_distance, camera.camera_settings.far_distance / 10.0f,
                         camera.camera_settings.near_distance)) {
      changed = true;
    }
    if (ImGui::DragFloat("FOV", &camera.camera_settings.fov, 1.0f, 1, 359)) {
      changed = true;
    }
    ImGui::TreePop();
  }
  FileUtils::SaveFile(
      "ScreenShot", "Image", {".png", ".jpg", ".hdr"},
      [&camera](const std::filesystem::path& file_path) {
        camera.GetRenderTexture()->Save(file_path);
      },
      false);

  return changed;
}

bool InspectAmbientOcclusion(AmbientOcclusion& ambient_occlusion) {
  bool changed = false;
  if (ImGui::Button("Apply default settings##AmbientOcclusion")) {
    ambient_occlusion = {};
    changed = true;
  }
  if (ImGui::DragFloat("Radius", &ambient_occlusion.radius, 0.001f, 0.0f, 10.f))
    changed = true;
  if (ImGui::DragFloat("Thickness", &ambient_occlusion.thickness, 0.01f, 0.001f, 10.f))
    changed = true;
  if (ImGui::DragInt("Slice count", &ambient_occlusion.slice_count, 1, 1, 16))
    changed = true;
  if (ImGui::DragInt("Steps per slice", &ambient_occlusion.steps_per_slice, 1, 1, 16))
    changed = true;
  if (ImGui::DragFloat("Intensity", &ambient_occlusion.intensity, 0.01f, 0.0f, 5.f))
    changed = true;
  if (ImGui::DragFloat("Denoise radius", &ambient_occlusion.denoise_radius, 0.01f, 0.0f, 100.f))
    changed = true;
  if (ImGui::DragFloat("Bias", &ambient_occlusion.bias, 0.001f, 0.0f, 1.f))
    changed = true;
  return changed;
}

bool InspectAntiAliasing(AntiAliasing& anti_aliasing) {
  bool changed = false;
  if (ImGui::Button("Apply default settings##AntiAliasing")) {
    anti_aliasing = {};
    changed = true;
  }
  int preset = static_cast<int>(anti_aliasing.preset);
  const char* presets[] = {"Low", "Medium", "High", "Ultra"};
  if (ImGui::Combo("Preset", &preset, presets, IM_ARRAYSIZE(presets))) {
    anti_aliasing.preset = static_cast<AntiAliasing::Preset>(preset);
    changed = true;
  }
  int debug_mode = static_cast<int>(anti_aliasing.debug_mode);
  const char* debug_modes[] = {"None", "Edges", "Blend weights"};
  if (ImGui::Combo("Debug view", &debug_mode, debug_modes, IM_ARRAYSIZE(debug_modes))) {
    anti_aliasing.debug_mode = static_cast<AntiAliasing::DebugMode>(debug_mode);
    changed = true;
  }
  return changed;
}

bool InspectBloom(Bloom& bloom) {
  bool changed = false;
  if (ImGui::Button("Apply default settings##Bloom")) {
    bloom = {};
    changed = true;
  }
  if (ImGui::DragFloat("Filter radius", &bloom.filter_radius, 0.05f, 0.1f, 4.0f))
    changed = true;
  if (ImGui::DragFloat("Threshold", &bloom.threshold, 0.01f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Knee", &bloom.knee, 0.01f, 0.0f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Intensity", &bloom.intensity, 0.01f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Source ceiling", &bloom.source_ceiling, 0.05f, 0.0f, 1000.0f, "%.3f",
                       ImGuiSliderFlags_AlwaysClamp)) {
    bloom.compression_start = glm::clamp(bloom.compression_start, 0.0f, bloom.source_ceiling);
    changed = true;
  }
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip(
        "Maximum extracted bloom-source brightness (linear HDR). Does not clamp scene color. Zero disables bloom.");
  if (ImGui::DragFloat("Compression start", &bloom.compression_start, 0.05f, 0.0f, bloom.source_ceiling, "%.3f",
                       ImGuiSliderFlags_AlwaysClamp)) {
    bloom.compression_start = glm::clamp(bloom.compression_start, 0.0f, bloom.source_ceiling);
    changed = true;
  }
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip(
        "Bloom is unchanged below this brightness and smoothly approaches the ceiling above it. Equal values give a "
        "hard cap.");
  return changed;
}

bool InspectScreenSpaceReflection(ScreenSpaceReflection& ssr) {
  bool changed = false;
  if (ImGui::Button("Apply default settings##ScreenSpaceReflection")) {
    ssr = {};
    changed = true;
  }
  if (ImGui::DragFloat("Max march distance", &ssr.max_distance, 0.01f, 0.01f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Distance confidence", &ssr.distance_confidence, 0.1f, 0.0f, 128.0f))
    changed = true;
  if (ImGui::DragInt("Max iteration count", &ssr.max_iteration_count, 1, 1, 256))
    changed = true;
  if (ImGui::DragInt("Binary search iterations", &ssr.binary_search_iteration_count, 1, 0, 64))
    changed = true;
  if (ImGui::DragFloat("Thickness", &ssr.thickness, 0.01f, 0.0f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Start bias", &ssr.start_bias, 0.001f, 0.0f, 10.0f))
    changed = true;
  if (ImGui::Checkbox("Edge-aware spatial resolve", &ssr.blur))
    changed = true;
  if (ImGui::Checkbox("Temporal stabilization", &ssr.temporal_stabilization))
    changed = true;
  const char* debug_modes[] = {"None", "Hit UV", "Ray distance", "Rejection reason", "Confidence"};
  int debug_mode = static_cast<int>(ssr.debug_mode);
  if (ImGui::Combo("Debug view", &debug_mode, debug_modes, IM_ARRAYSIZE(debug_modes))) {
    ssr.debug_mode = static_cast<ScreenSpaceReflection::DebugMode>(debug_mode);
    changed = true;
  }
  return changed;
}

bool InspectToneMapping(ToneMapping& tone_mapping) {
  bool changed = false;
  if (ImGui::Button("Apply default settings##ToneMapping")) {
    tone_mapping = {};
    changed = true;
  }
  int method = static_cast<int>(tone_mapping.method);
  const char* methods[] = {"Filmic", "Uncharted 2", "Clip", "ACES", "AgX", "Khronos PBR", "EvoEngine Exponential"};
  if (ImGui::Combo("Method", &method, methods, IM_ARRAYSIZE(methods))) {
    tone_mapping.method = static_cast<ToneMapping::ToneMapMethod>(method);
    changed = true;
  }
  if (ImGui::DragFloat("Exposure", &tone_mapping.exposure, 0.01f, 0.01f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Brightness", &tone_mapping.brightness, 0.01f, 0.01f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Contrast", &tone_mapping.contrast, 0.01f, 0.0f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Saturation", &tone_mapping.saturation, 0.01f, 0.0f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Vignette", &tone_mapping.vignette, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::Checkbox("Auto exposure", &tone_mapping.auto_exposure))
    changed = true;
  if (tone_mapping.auto_exposure) {
    if (ImGui::DragFloat("Adaptation speed", &tone_mapping.auto_exposure_speed, 0.01f, 0.0f, 100.0f))
      changed = true;
    if (ImGui::DragFloat("Min EV100", &tone_mapping.ev_min_value, 0.01f, -24.0f, 24.0f))
      changed = true;
    if (ImGui::DragFloat("Max EV100", &tone_mapping.ev_max_value, 0.01f, -24.0f, 24.0f))
      changed = true;
    if (ImGui::Checkbox("Center metering", &tone_mapping.enable_center_metering))
      changed = true;
    if (tone_mapping.enable_center_metering &&
        ImGui::DragFloat("Center metering size", &tone_mapping.center_metering_size, 0.01f, 0.01f, 1.0f))
      changed = true;
    if (ImGui::DragInt("Average mode", &tone_mapping.average_mode, 1, 0, 1))
      changed = true;
  }
  if (ImGui::Checkbox("Dither", &tone_mapping.dither))
    changed = true;
  return changed;
}

bool InspectPostProcessingStack(InspectorContext&, PostProcessingStack& stack) {
  bool changed = false;

  if (ImGui::Checkbox("AO##0", &stack.enable_ambient_occlusion))
    changed = true;
  if (stack.enable_ambient_occlusion && ImGui::TreeNodeEx("AO##1", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (InspectAmbientOcclusion(*stack.ambient_occlusion))
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::Checkbox("SSR##0", &stack.enable_screen_space_reflection))
    changed = true;
  if (stack.enable_screen_space_reflection && ImGui::TreeNodeEx("SSR##1", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (InspectScreenSpaceReflection(*stack.screen_space_reflection))
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::Checkbox("AA##0", &stack.enable_anti_aliasing))
    changed = true;
  if (stack.enable_anti_aliasing && ImGui::TreeNodeEx("AA##1", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (InspectAntiAliasing(*stack.anti_aliasing))
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::Checkbox("Bloom##0", &stack.enable_bloom))
    changed = true;
  if (stack.enable_bloom && ImGui::TreeNodeEx("Bloom##1", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (InspectBloom(*stack.bloom))
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::Checkbox("Tone Mapping##0", &stack.enable_tone_mapping))
    changed = true;
  if (stack.enable_tone_mapping && ImGui::TreeNodeEx("Tone Mapping##1", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (InspectToneMapping(*stack.tone_mapping))
      changed = true;
    ImGui::TreePop();
  }

  return changed;
}

const char* GetProceduralNodeName(const pn::NodeType type) {
  switch (type) {
    case pn::NodeType::Unknown:
      return "Unknown";
    case pn::NodeType::Input:
      return "Input";
    case pn::NodeType::Output:
      return "Output";
    case pn::NodeType::Constant:
      return "Constant";
    case pn::NodeType::Sine:
      return "Sine";
    case pn::NodeType::Tangent:
      return "Tangent";
    case pn::NodeType::Cosine:
      return "Cosine";
    case pn::NodeType::Simplex2D:
      return "Simplex2D";
    case pn::NodeType::Simplex3D:
      return "Simplex3D";
    case pn::NodeType::Simplex4D:
      return "Simplex4D";
    case pn::NodeType::Perlin2D:
      return "Perlin2D";
    case pn::NodeType::Perlin3D:
      return "Perlin3D";
    case pn::NodeType::Perlin4D:
      return "Perlin4D";
    case pn::NodeType::Add:
      return "Add";
    case pn::NodeType::Subtract:
      return "Subtract";
    case pn::NodeType::Multiply:
      return "Multiply";
    case pn::NodeType::Divide:
      return "Divide";
    case pn::NodeType::Power:
      return "Pow";
    case pn::NodeType::Min:
      return "Min";
    case pn::NodeType::Max:
      return "Max";
    case pn::NodeType::Abs:
      return "Abs";
    case pn::NodeType::Clamp:
      return "Clamp";
    case pn::NodeType::Exponent:
      return "Exponent";
    case pn::NodeType::Negate:
      return "Negate";
    case pn::NodeType::FlipUp:
      return "FlipUp";
    case pn::NodeType::FlipDown:
      return "FlipDown";
    case pn::NodeType::Sigmoid:
      return "Sigmoid";
    case pn::NodeType::SoftSign:
      return "SoftSign";
    case pn::NodeType::Tanh:
      return "Tanh";
  }
  return "Unknown";
}

bool InspectProceduralFrequency(float& frequency) {
  bool changed = false;
  ImGui::PushItemWidth(50);
  if (ImGui::DragFloat("Frequency", &frequency, 0.1f))
    changed = true;
  ImGui::PopItemWidth();
  return changed;
}

bool InspectProceduralNode(pn::INode& node) {
  if (auto* constant = dynamic_cast<pn::ConstantNode*>(&node)) {
    bool changed = false;
    ImGui::PushItemWidth(50);
    if (ImGui::DragFloat("Value", &constant->value, 0.1f))
      changed = true;
    ImGui::PopItemWidth();
    return changed;
  }
  if (auto* perlin = dynamic_cast<pn::Perlin2DNode*>(&node))
    return InspectProceduralFrequency(perlin->frequency);
  if (auto* perlin = dynamic_cast<pn::Perlin3DNode*>(&node))
    return InspectProceduralFrequency(perlin->frequency);
  if (auto* perlin = dynamic_cast<pn::Perlin4DNode*>(&node))
    return InspectProceduralFrequency(perlin->frequency);
  if (auto* simplex = dynamic_cast<pn::Simplex2DNode*>(&node))
    return InspectProceduralFrequency(simplex->frequency);
  if (auto* simplex = dynamic_cast<pn::Simplex3DNode*>(&node))
    return InspectProceduralFrequency(simplex->frequency);
  if (auto* simplex = dynamic_cast<pn::Simplex4DNode*>(&node))
    return InspectProceduralFrequency(simplex->frequency);
  if (auto* sine = dynamic_cast<pn::SineNode*>(&node))
    return InspectProceduralFrequency(sine->frequency);
  if (auto* cosine = dynamic_cast<pn::CosineNode*>(&node))
    return InspectProceduralFrequency(cosine->frequency);
  if (auto* tangent = dynamic_cast<pn::TangentNode*>(&node))
    return InspectProceduralFrequency(tangent->frequency);
  return false;
}

bool InspectProceduralNodeData(const pn::NodeData& data) {
  return data.node_impl && InspectProceduralNode(*data.node_impl);
}

template <typename Node>
bool AddProceduralNodeMenuItem(NoiseGraph& graph, const char* label, const pn::NodeType type,
                               const std::initializer_list<const char*> input_names, const ImVec2 click_pos) {
  if (!ImGui::MenuItem(label))
    return false;

  const auto new_node_handle = graph.AllocateNode(static_cast<int>(input_names.size()), 1);
  auto& node = graph.RefNode(new_node_handle);
  node.data.type = type;
  node.data.node_impl = std::make_shared<Node>();
  size_t index = 0;
  for (const auto* input_name : input_names) {
    graph.RefInputPin(node.GetInputPinHandles()[index++]).data.name = input_name;
  }
  ImNodes::SetNodeScreenSpacePos(new_node_handle, click_pos);
  return true;
}

bool DrawProceduralNodeCreationMenu(NoiseGraph& graph, const ImVec2 click_pos) {
  bool changed = false;
  if (ImGui::BeginMenu("Generators")) {
    changed = AddProceduralNodeMenuItem<pn::ConstantNode>(graph, "Constant", pn::NodeType::Constant, {}, click_pos) ||
              changed;
    changed = AddProceduralNodeMenuItem<pn::SineNode>(graph, "Sine", pn::NodeType::Sine, {"x"}, click_pos) || changed;
    changed =
        AddProceduralNodeMenuItem<pn::CosineNode>(graph, "Cosine", pn::NodeType::Cosine, {"x"}, click_pos) || changed;
    changed = AddProceduralNodeMenuItem<pn::TangentNode>(graph, "Tangent", pn::NodeType::Tangent, {"x"}, click_pos) ||
              changed;
    changed =
        AddProceduralNodeMenuItem<pn::Perlin2DNode>(graph, "Perlin2D", pn::NodeType::Perlin2D, {"x", "y"}, click_pos) ||
        changed;
    changed = AddProceduralNodeMenuItem<pn::Perlin3DNode>(graph, "Perlin3D", pn::NodeType::Perlin3D, {"x", "y", "z"},
                                                          click_pos) ||
              changed;
    changed = AddProceduralNodeMenuItem<pn::Perlin4DNode>(graph, "Perlin4D", pn::NodeType::Perlin4D,
                                                          {"x", "y", "z", "w"}, click_pos) ||
              changed;
    changed = AddProceduralNodeMenuItem<pn::Simplex2DNode>(graph, "Simplex2D", pn::NodeType::Simplex2D, {"x", "y"},
                                                           click_pos) ||
              changed;
    changed = AddProceduralNodeMenuItem<pn::Simplex3DNode>(graph, "Simplex3D", pn::NodeType::Simplex3D, {"x", "y", "z"},
                                                           click_pos) ||
              changed;
    changed = AddProceduralNodeMenuItem<pn::Simplex4DNode>(graph, "Simplex4D", pn::NodeType::Simplex4D,
                                                           {"x", "y", "z", "w"}, click_pos) ||
              changed;
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Combiners")) {
    changed = AddProceduralNodeMenuItem<pn::AddNode>(graph, "Add", pn::NodeType::Add, {"a", "b"}, click_pos) || changed;
    changed =
        AddProceduralNodeMenuItem<pn::SubtractNode>(graph, "Subtract", pn::NodeType::Subtract, {"a", "b"}, click_pos) ||
        changed;
    changed =
        AddProceduralNodeMenuItem<pn::MultiplyNode>(graph, "Multiply", pn::NodeType::Multiply, {"a", "b"}, click_pos) ||
        changed;
    changed = AddProceduralNodeMenuItem<pn::DivideNode>(graph, "Divide", pn::NodeType::Divide, {"a", "b"}, click_pos) ||
              changed;
    changed =
        AddProceduralNodeMenuItem<pn::PowerNode>(graph, "Power", pn::NodeType::Power, {"x", "power"}, click_pos) ||
        changed;
    changed = AddProceduralNodeMenuItem<pn::MinNode>(graph, "Min", pn::NodeType::Min, {"a", "b"}, click_pos) || changed;
    changed = AddProceduralNodeMenuItem<pn::MaxNode>(graph, "Max", pn::NodeType::Max, {"a", "b"}, click_pos) || changed;
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Modifiers")) {
    changed = AddProceduralNodeMenuItem<pn::AbsNode>(graph, "Abs", pn::NodeType::Abs, {"x"}, click_pos) || changed;
    changed =
        AddProceduralNodeMenuItem<pn::NegateNode>(graph, "Negate", pn::NodeType::Negate, {"x"}, click_pos) || changed;
    changed =
        AddProceduralNodeMenuItem<pn::ExponentNode>(graph, "Exponent", pn::NodeType::Exponent, {"x"}, click_pos) ||
        changed;
    changed = AddProceduralNodeMenuItem<pn::ClampNode>(graph, "Clamp", pn::NodeType::Clamp, {"x", "lower", "upper"},
                                                       click_pos) ||
              changed;
    changed =
        AddProceduralNodeMenuItem<pn::FlipUpNode>(graph, "FlipUp", pn::NodeType::FlipUp, {"a", "base"}, click_pos) ||
        changed;
    changed = AddProceduralNodeMenuItem<pn::FlipDownNode>(graph, "FlipDown", pn::NodeType::FlipDown, {"a", "base"},
                                                          click_pos) ||
              changed;
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Activation")) {
    changed = AddProceduralNodeMenuItem<pn::SigmoidNode>(graph, "Sigmoid", pn::NodeType::Sigmoid,
                                                         {"a", "b", "speed", "x"}, click_pos) ||
              changed;
    changed = AddProceduralNodeMenuItem<pn::SoftSignNode>(graph, "SoftSign", pn::NodeType::SoftSign,
                                                          {"a", "b", "speed", "x"}, click_pos) ||
              changed;
    changed = AddProceduralNodeMenuItem<pn::TanhNode>(graph, "Tanh", pn::NodeType::Tanh, {"a", "b", "speed", "x"},
                                                      click_pos) ||
              changed;
    ImGui::EndMenu();
  }
  return changed;
}

bool InspectProceduralNoiseGraph(pn::IProceduralNoise& noise, const std::string& window_title,
                                 const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Begin(window_title.c_str())) {
    const auto id = ImGui::GetID(window_title.c_str());
    static NodeGraphNodeHandle hovered_node_handle = -1;
    static NodeGraphLinkHandle hovered_link_handle = -1;
    auto& graph = noise.node_graph;

    graph.Draw(
        id, editor_layer,
        [&](const NodeGraphNodeHandle node_handle) {
          ImGui::TextUnformatted(GetProceduralNodeName(graph.PeekNode(node_handle).data.type));
        },
        [&](const NodeGraphNodeHandle node_handle) {
          if (InspectProceduralNodeData(graph.RefNode(node_handle).data))
            changed = true;
        },
        [&](const NodeGraphInputPinHandle input_pin_handle) {
          ImGui::TextUnformatted(graph.RefInputPin(input_pin_handle).data.name.c_str());
        },
        [&](const NodeGraphOutputPinHandle output_pin_handle) {
          ImGui::TextUnformatted(graph.RefOutputPin(output_pin_handle).data.name.c_str());
        },
        [&](const NodeGraphNodeHandle node_handle, const NodeGraphLinkHandle link_handle,
            const NodeGraphInputPinHandle input_pin_handle, const NodeGraphOutputPinHandle output_pin_handle) {
          hovered_node_handle = node_handle;
          hovered_link_handle = link_handle;
        },
        [&](const std::vector<NodeGraphNodeHandle>& selected_node_handles,
            const std::vector<NodeGraphLinkHandle>& selected_link_handles) {
        },
        [&](const ImVec2 click_pos) {
          if (hovered_node_handle > 1) {
            if (ImGui::MenuItem("Delete node")) {
              graph.RecycleNode(hovered_node_handle);
              changed = true;
            }
          } else if (hovered_link_handle != -1) {
            if (ImGui::MenuItem("Delete link")) {
              graph.RecycleLink(hovered_link_handle);
              changed = true;
            }
          } else if (ImGui::BeginMenu("New node...")) {
            if (DrawProceduralNodeCreationMenu(graph, click_pos))
              changed = true;
            ImGui::EndMenu();
          }
        },
        [&](const NodeGraphOutputPinHandle start_handle, const NodeGraphInputPinHandle end_handle) {
          if (graph.PeekInputPin(end_handle).GetLinkHandle() == -1) {
            graph.AllocateLink(start_handle, end_handle);
            changed = true;
          }
        },
        [&](const NodeGraphLinkHandle link_handle) {
          graph.RecycleLink(link_handle);
          changed = true;
        });
  }
  ImGui::End();
  return changed;
}

std::shared_ptr<Texture2D>& RefProceduralPreviewTexture(const Handle handle) {
  static std::map<uint64_t, std::shared_ptr<Texture2D>> preview_textures;
  auto& texture = preview_textures[handle.GetValue()];
  if (!texture) {
    texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  }
  return texture;
}

void DrawProceduralPreviewTexture(const std::shared_ptr<Texture2D>& texture) {
  const auto texture_storage = texture->PeekTexture2DStorage();
  if (!texture_storage.im_texture_id) {
    return;
  }
  static float debug_scale = 1.f;
  ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 10.0f);
  debug_scale = glm::clamp(debug_scale, 0.1f, 10.0f);
  ImGui::Image(texture_storage.im_texture_id,
               ImVec2(texture_storage.image->GetExtent().width * debug_scale,
                      texture_storage.image->GetExtent().height * debug_scale),
               ImVec2(0, 1), ImVec2(1, 0));
}

bool InspectProceduralNoise2D(InspectorContext& context, pn::ProceduralNoise2D& noise) {
  bool changed = false;
  static glm::vec2 temp_input{};
  static float temp_output = 0.0f;
  if (ImGui::DragFloat2("Test Input", &temp_input.x))
    temp_output = noise.GetValue(temp_input);
  if (ImGui::Button("Calculate"))
    temp_output = noise.GetValue(temp_input);
  ImGui::Text("Test Output: %.3f", temp_output);

  static bool show_node_graph = true;
  ImGui::Checkbox("Show node graph", &show_node_graph);
  if (show_node_graph)
    changed = InspectProceduralNoiseGraph(noise, "Procedural Noise 2D", context.editor_layer) || changed;

  if (ImGui::TreeNode("Show sample texture")) {
    static int resolution = 64;
    static float position_scale = 1.f;
    static glm::vec2 position_offset = glm::vec2(0.f);
    bool resolution_changed = false;
    if (ImGui::DragInt("Resolution", &resolution, 1, 16, 1024)) {
      resolution = glm::clamp(resolution, 16, 1024);
      resolution_changed = true;
    }
    if (ImGui::DragFloat("Position scale", &position_scale, 0.1f))
      resolution_changed = true;
    if (ImGui::DragFloat2("Position offset", &position_offset.x, 0.1f))
      resolution_changed = true;
    auto& texture = RefProceduralPreviewTexture(noise.GetHandle());
    if (changed || resolution_changed || !texture->PeekTexture2DStorage().image) {
      std::vector<glm::vec4> color(resolution * resolution);
      Jobs::RunParallelFor(resolution * resolution, [&](size_t i) {
        float x = static_cast<float>(i / resolution);
        float y = static_cast<float>(i % resolution);
        x /= resolution;
        y /= resolution;
        color[i] = glm::vec4(glm::vec3(noise.GetValue(glm::vec2(x, y) * position_scale + position_offset)), 1.0f);
      });
      texture->SetRgbaChannelData(color, glm::uvec2(resolution));
    }
    DrawProceduralPreviewTexture(texture);
    ImGui::TreePop();
  }
  return changed;
}

bool InspectProceduralNoise3D(InspectorContext& context, pn::ProceduralNoise3D& noise) {
  bool changed = false;
  static glm::vec3 temp_input{};
  static float temp_output = 0.0f;
  if (ImGui::DragFloat3("Test Input", &temp_input.x))
    temp_output = noise.GetValue(temp_input);
  if (ImGui::Button("Calculate"))
    temp_output = noise.GetValue(temp_input);
  ImGui::Text("Test Output: %.3f", temp_output);

  static bool show_node_graph = true;
  ImGui::Checkbox("Show node graph", &show_node_graph);
  if (show_node_graph)
    changed = InspectProceduralNoiseGraph(noise, "Procedural Noise 3D", context.editor_layer) || changed;

  if (ImGui::TreeNode("Show sample texture")) {
    static int resolution = 64;
    static float position_scale = 1.f;
    static glm::vec3 position_offset = glm::vec3(0.f);
    static float z = 0.0f;
    bool resolution_changed = false;
    if (ImGui::DragInt("Resolution", &resolution, 1, 16, 1024)) {
      resolution = glm::clamp(resolution, 16, 1024);
      resolution_changed = true;
    }
    if (ImGui::DragFloat("Position scale", &position_scale, 0.1f))
      resolution_changed = true;
    if (ImGui::DragFloat2("Position offset", &position_offset.x, 0.1f))
      resolution_changed = true;
    if (ImGui::SliderFloat("z", &z, 0, 1))
      resolution_changed = true;
    auto& texture = RefProceduralPreviewTexture(noise.GetHandle());
    if (changed || resolution_changed || !texture->PeekTexture2DStorage().image) {
      std::vector<glm::vec4> color(resolution * resolution);
      Jobs::RunParallelFor(resolution * resolution, [&](size_t i) {
        float x = static_cast<float>(i / resolution);
        float y = static_cast<float>(i % resolution);
        x /= resolution;
        y /= resolution;
        color[i] = glm::vec4(glm::vec3(noise.GetValue(glm::vec3(x, y, z) * position_scale + position_offset)), 1.0f);
      });
      texture->SetRgbaChannelData(color, glm::uvec2(resolution));
    }
    DrawProceduralPreviewTexture(texture);
    ImGui::TreePop();
  }
  return changed;
}

bool InspectProceduralNoise4D(InspectorContext& context, pn::ProceduralNoise4D& noise) {
  bool changed = false;
  static glm::vec4 temp_input{};
  static float temp_output = 0.0f;
  if (ImGui::DragFloat4("Test Input", &temp_input.x))
    temp_output = noise.GetValue(temp_input);
  if (ImGui::Button("Calculate"))
    temp_output = noise.GetValue(temp_input);
  ImGui::Text("Test Output: %.3f", temp_output);

  static bool show_node_graph = true;
  ImGui::Checkbox("Show node graph", &show_node_graph);
  if (show_node_graph)
    changed = InspectProceduralNoiseGraph(noise, "Procedural Noise 4D", context.editor_layer) || changed;

  if (ImGui::TreeNode("Show sample texture")) {
    static int resolution = 64;
    static float position_scale = 1.f;
    static glm::vec4 position_offset = glm::vec4(0.f);
    static float z = 0.0f;
    static float w = 0.0f;
    bool resolution_changed = false;
    if (ImGui::DragInt("Resolution", &resolution, 1, 16, 1024)) {
      resolution = glm::clamp(resolution, 16, 1024);
      resolution_changed = true;
    }
    if (ImGui::DragFloat("Position scale", &position_scale, 0.1f))
      resolution_changed = true;
    if (ImGui::DragFloat4("Position offset", &position_offset.x, 0.1f))
      resolution_changed = true;
    if (ImGui::SliderFloat("z", &z, 0, 1))
      resolution_changed = true;
    if (ImGui::SliderFloat("w", &w, 0, 1))
      resolution_changed = true;
    auto& texture = RefProceduralPreviewTexture(noise.GetHandle());
    if (changed || resolution_changed || !texture->PeekTexture2DStorage().image) {
      std::vector<glm::vec4> color(resolution * resolution);
      Jobs::RunParallelFor(resolution * resolution, [&](size_t i) {
        float x = static_cast<float>(i / resolution);
        float y = static_cast<float>(i % resolution);
        x /= resolution;
        y /= resolution;
        color[i] = glm::vec4(glm::vec3(noise.GetValue(glm::vec4(x, y, z, w) * position_scale + position_offset)), 1.0f);
      });
      texture->SetRgbaChannelData(color, glm::uvec2(resolution));
    }
    DrawProceduralPreviewTexture(texture);
    ImGui::TreePop();
  }
  return changed;
}

bool InspectDrawSettings(DrawSettings& draw_settings, const bool inspect_material_render_state = true) {
  bool changed = false;
  int polygon_mode_tmp = 0;
  switch (draw_settings.polygon_mode) {
    case VK_POLYGON_MODE_POINT:
      polygon_mode_tmp = 0;
      break;
    case VK_POLYGON_MODE_LINE:
      polygon_mode_tmp = 1;
      break;
    case VK_POLYGON_MODE_FILL:
      polygon_mode_tmp = 2;
      break;
  }
  if (ImGui::Combo("Polygon Mode", &polygon_mode_tmp, polygon_mode_string, IM_ARRAYSIZE(polygon_mode_string))) {
    changed = true;
    switch (polygon_mode_tmp) {
      case 0:
        draw_settings.polygon_mode = VK_POLYGON_MODE_POINT;
        break;
      case 1:
        draw_settings.polygon_mode = VK_POLYGON_MODE_LINE;
        break;
      case 2:
        draw_settings.polygon_mode = VK_POLYGON_MODE_FILL;
        break;
    }
  }
  if (draw_settings.polygon_mode == VK_POLYGON_MODE_LINE) {
    ImGui::DragFloat("Line width", &draw_settings.line_width, 0.1f, 0.0f, 100.0f);
  }
  if (inspect_material_render_state) {
    int cull_face_mode_tmp = 0;
    switch (draw_settings.cull_mode) {
      case VK_CULL_MODE_FRONT_BIT:
        cull_face_mode_tmp = 0;
        break;
      case VK_CULL_MODE_BACK_BIT:
        cull_face_mode_tmp = 1;
        break;
      case VK_CULL_MODE_FRONT_AND_BACK:
        cull_face_mode_tmp = 2;
        break;
      case VK_CULL_MODE_NONE:
        cull_face_mode_tmp = 3;
        break;
    }
    if (ImGui::Combo("Cull Face Mode", &cull_face_mode_tmp, culling_mode_string, IM_ARRAYSIZE(culling_mode_string))) {
      changed = true;
      switch (cull_face_mode_tmp) {
        case 0:
          draw_settings.cull_mode = VK_CULL_MODE_FRONT_BIT;
          break;
        case 1:
          draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;
          break;
        case 2:
          draw_settings.cull_mode = VK_CULL_MODE_FRONT_AND_BACK;
          break;
        case 3:
          draw_settings.cull_mode = VK_CULL_MODE_NONE;
          break;
      }
    }

    if (ImGui::Checkbox("Blending", &draw_settings.blending))
      changed = true;

    if (false && draw_settings.blending) {
      if (ImGui::Combo("Blending Source Factor", reinterpret_cast<int*>(&draw_settings.blending_src_factor),
                       blending_factor_string, IM_ARRAYSIZE(blending_factor_string))) {
        changed = true;
      }
      if (ImGui::Combo("Blending Destination Factor", reinterpret_cast<int*>(&draw_settings.blending_dst_factor),
                       blending_factor_string, IM_ARRAYSIZE(blending_factor_string))) {
        changed = true;
      }
    }
  }
  return changed;
}

bool InspectShader(InspectorContext&, Shader& shader) {
  bool changed = false;
  ImGui::Text((std::string("Current Status: ") + std::string(shader.Compiled() ? "Compiled" : "Not compiled")).c_str());
  if (ImGui::Button("TryCompile")) {
    shader.TryCompile();
  }
  if (ImGui::Combo(
          "Type",
          {"Vertex", "Tessellation Control", "Tessellation Evaluation", "Geometry", "Task", "Mesh", "Fragment",
           "Compute", "Ray Generation", "Closest Hit", "Miss", "Any Hit", "Intersection", "Callable", "Unknown"},
          shader.RefShaderType())) {
    changed = true;
  }
  auto& shader_code = shader.RefShaderCode();
  if (ImGui::InputTextMultiline("Code", shader_code.data(), shader_code.size(), ImGui::GetContentRegionAvail(),
                                ImGuiInputTextFlags_CallbackResize, ShaderStringResizeCallback, &shader_code)) {
    changed = true;
  }
  return changed;
}

void InspectRenderLayerGeneralSettings(RenderLayer& render_layer) {
  ImGui::Checkbox("Count shadows drawcalls", &render_layer.count_shadow_rendering_draw_calls);
  ImGui::Checkbox("Wireframe", &render_layer.wire_frame);
  if (Platform::MeshShaderEnabled()) {
    ImGui::Checkbox("Meshlet", &render_layer.enable_meshlet);
  }
  ImGui::Checkbox("Indirect Rendering", &render_layer.enable_indirect_rendering);
  ImGui::Checkbox("Show entities", &render_layer.render_settings.enable_debug_visualization);
  const char* indirect_lighting_debug_views[] = {"Beauty",
                                                 "Diffuse Indirect",
                                                 "Unoccluded Probe Specular",
                                                 "Specular Visibility",
                                                 "Occluded Probe Specular",
                                                 "DDGI Probe Blend Loss"};
  auto indirect_lighting_debug_view = static_cast<int>(render_layer.render_settings.indirect_lighting_debug_view);
  if (ImGui::Combo("Indirect lighting debug", &indirect_lighting_debug_view, indirect_lighting_debug_views,
                   IM_ARRAYSIZE(indirect_lighting_debug_views))) {
    render_layer.render_settings.indirect_lighting_debug_view = static_cast<RenderSettings::IndirectLightingDebugView>(
        glm::clamp(indirect_lighting_debug_view, 0, static_cast<int>(IM_ARRAYSIZE(indirect_lighting_debug_views)) - 1));
  }
  ImGui::Checkbox("Full camera-ray shaders", &render_layer.force_full_ray_camera_shader_variant);
  auto capture_gpu_timing = Platform::GpuTimestampCaptureEnabled();
  if (ImGui::Checkbox("Capture live GPU timing", &capture_gpu_timing)) {
    Platform::SetGpuTimestampCaptureEnabled(capture_gpu_timing);
  }
  if (capture_gpu_timing && ImGui::Button("Reset live timing")) {
    Platform::ResetGpuTimestampStats();
  }
  const auto draw_variant = [&](const char* label, const RayCameraShaderTechnique technique) {
    const auto stats = render_layer.GetRayCameraShaderVariantStats(technique);
    ImGui::Text("%s: %s -> %s (%s%s)", label, stats.requested_key.c_str(), stats.active_key.c_str(),
                stats.cache_source.c_str(), stats.pending ? ", pending" : "");
    ImGui::Text("  variants %u/%u, pending %u, failed %u, retained %u, evictions %llu", stats.resident_variant_count,
                stats.variant_capacity, stats.pending_build_count, stats.failed_entry_count,
                stats.retained_submission_count, static_cast<unsigned long long>(stats.eviction_count));
    ImGui::Text("  pipeline %.2f ms%s%s", stats.pipeline_creation.wall_milliseconds,
                stats.pipeline_creation.feedback_valid
                    ? (stats.pipeline_creation.application_cache_hit ? ", cache hit" : ", cache miss")
                    : ", cache status unknown",
                stats.pipeline_creation.deferred_used ? ", deferred" : "");
  };
  if (Platform::RayTracingEnabled())
    draw_variant("RTX variant", RayCameraShaderTechnique::RayTracing);
  if (Platform::RayQueryEnabled())
    draw_variant("Ray Query variant", RayCameraShaderTechnique::RayQuery);
}

void InspectRenderLayerStats(RenderLayer& render_layer) {
  const auto& graphics = Platform::GetInstance();
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto prim_count =
      current_frame_index < graphics.prim_count.size() ? graphics.prim_count[current_frame_index] : 0u;
  const auto draw_call_count =
      current_frame_index < graphics.draw_call.size() ? graphics.draw_call[current_frame_index] : 0u;
  ImGui::Text("Frame: %u", current_frame_index);
  ImGui::Text("%s prims", FormatRenderCounter(prim_count).c_str());
  ImGui::Text("%llu draw submissions", static_cast<unsigned long long>(draw_call_count));
  ImGui::Separator();

  if (ImGui::TreeNodeEx("Ray camera frame path", ImGuiTreeNodeFlags_DefaultOpen)) {
    const auto frame_path = render_layer.GetRayCameraFramePathStats();
    const auto history = render_layer.GetRayCameraHistoryStats();
    const auto& cache = frame_path.render_graph_plan_cache;
    ImGui::Text("Graph plans %llu hits / %llu misses, %llu compiles (%.2f ms)",
                static_cast<unsigned long long>(cache.hit_count), static_cast<unsigned long long>(cache.miss_count),
                static_cast<unsigned long long>(cache.compilation_count), cache.compilation_milliseconds);
    ImGui::Text("Graph cache %zu/%zu, %llu evictions", cache.entry_count, cache.capacity,
                static_cast<unsigned long long>(cache.eviction_count));
    ImGui::Text("Output descriptors %llu live / %llu peak, %llu creates / %llu reuses",
                static_cast<unsigned long long>(frame_path.live_output_descriptor_count),
                static_cast<unsigned long long>(frame_path.peak_live_output_descriptor_count),
                static_cast<unsigned long long>(frame_path.output_descriptor_creation_count),
                static_cast<unsigned long long>(frame_path.output_descriptor_reuse_count));
    ImGui::Text("Frame slots %u retained / %u pending", frame_path.retained_frame_slot_count,
                Platform::GetPendingFrameSubmissionCount());

    std::shared_ptr<Camera> ray_camera;
    render_layer.ForEachCollectedCamera([&](const std::shared_ptr<Camera>& camera) {
      if (!ray_camera && camera && Camera::IsRayCameraRenderMode(camera->camera_render_mode)) {
        ray_camera = camera;
      }
    });
    if (ray_camera) {
      const auto samples_per_frame = static_cast<uint64_t>(std::max(ray_camera->camera_settings.sample_size, 1));
      const auto accumulated_samples = static_cast<uint64_t>(ray_camera->GetFrameCount()) * samples_per_frame;
      ImGui::Text("Camera frames %u, %llu accumulated spp (%llu spp/frame)", ray_camera->GetFrameCount(),
                  static_cast<unsigned long long>(accumulated_samples),
                  static_cast<unsigned long long>(samples_per_frame));
      const auto render_texture = ray_camera->GetRenderTexture();
      for (const auto& timing : Platform::GetGpuTimestampStats()) {
        if (timing.name != "Path Trace (RTX)" && timing.name != "Path Trace (RQ)") {
          continue;
        }
        const auto extent = render_texture ? render_texture->GetExtent() : VkExtent3D{};
        const auto throughput = timing.last_milliseconds > 0.0
                                    ? static_cast<double>(extent.width) * extent.height * samples_per_frame /
                                          timing.last_milliseconds / 1000.0
                                    : 0.0;
        ImGui::Text("%s %.3f ms last / %.3f ms median, %.2f Msample/s", timing.name.c_str(), timing.last_milliseconds,
                    timing.MedianMilliseconds(), throughput);
      }
    } else {
      ImGui::TextUnformatted("No collected ray camera.");
    }

    const auto memory = Platform::GetGpuMemorySnapshot();
    uint64_t device_local_bytes = 0;
    uint64_t host_bytes = 0;
    for (const auto& heap : memory.heaps) {
      (heap.device_local ? device_local_bytes : host_bytes) += heap.allocation_bytes;
    }
    const auto blas = BottomLevelAccelerationStructure::GetStaticBuildTelemetry();
    ImGui::Text("VRAM device-local %.2f MiB, host %.2f MiB, %llu allocations",
                static_cast<double>(device_local_bytes) / (1024.0 * 1024.0),
                static_cast<double>(host_bytes) / (1024.0 * 1024.0),
                static_cast<unsigned long long>(memory.allocation_count));
    ImGui::Text("Ray history %.2f MiB, compacted static BLAS %.2f MiB",
                static_cast<double>(history.live_byte_size) / (1024.0 * 1024.0),
                static_cast<double>(blas.final_compacted_storage_bytes) / (1024.0 * 1024.0));
    for (const auto& timing : Platform::GetCpuTimingStats()) {
      if (timing.name.find("Wait") != std::string::npos) {
        ImGui::Text("%s: %llu waits, %.3f ms median", timing.name.c_str(),
                    static_cast<unsigned long long>(timing.sample_count), timing.MedianMilliseconds());
      }
    }
    ImGui::TreePop();
  }
  ImGui::Separator();

  std::array<RenderPassDrawStats, Platform::kRenderPassDrawBucketCount> frame_pass_stats{};
  if (current_frame_index < graphics.render_pass_draw_stats.size()) {
    frame_pass_stats = graphics.render_pass_draw_stats[current_frame_index];
  }

  const auto scene = ApplicationContext::Get().GetActiveScene();
  bool drew_camera_stats = false;
  if (current_frame_index < graphics.render_camera_draw_stats.size()) {
    const auto& camera_stats_list = graphics.render_camera_draw_stats[current_frame_index];
    for (size_t camera_stats_index = 0; camera_stats_index < camera_stats_list.size(); camera_stats_index++) {
      const auto& camera_stats = camera_stats_list[camera_stats_index];
      for (size_t bucket_index = 0; bucket_index < Platform::kRenderPassDrawBucketCount; bucket_index++) {
        SubtractDrawStats(frame_pass_stats[bucket_index], camera_stats.pass_stats[bucket_index]);
      }
      const auto total = camera_stats.Total();
      if (!HasDrawStats(total)) {
        continue;
      }
      drew_camera_stats = true;
      const auto label = BuildCameraDrawStatsLabel(camera_stats, scene);
      const auto tree_id = label + "##RenderCameraDrawStats" + std::to_string(camera_stats_index) + "_" +
                           std::to_string(camera_stats.camera_handle);
      if (ImGui::TreeNodeEx(tree_id.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("%s prims, %llu draw submissions", FormatRenderCounter(total.prim_count).c_str(),
                    static_cast<unsigned long long>(total.TotalDrawCalls()));
        const auto table_id = "RenderCameraDrawStatsTable" + std::to_string(camera_stats_index);
        DrawRenderPassStatsTable(table_id.c_str(), camera_stats.pass_stats);
        ImGui::TreePop();
      }
    }
  }
  if (!drew_camera_stats) {
    ImGui::TextUnformatted("No camera draw stats recorded for the current frame.");
  }

  const auto frame_total = TotalDrawStats(frame_pass_stats);
  if (HasDrawStats(frame_total)) {
    if (ImGui::TreeNodeEx("Frame##RenderFrameDrawStats", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Text("%s prims, %llu draw submissions", FormatRenderCounter(frame_total.prim_count).c_str(),
                  static_cast<unsigned long long>(frame_total.TotalDrawCalls()));
      DrawRenderPassStatsTable("RenderFrameDrawStatsTable", frame_pass_stats);
      ImGui::TreePop();
    }
  }
}

void InspectShadowSettings(RenderSettings& render_settings) {
  const char* shadow_debug_modes[] = {"Off", "Cascade Index", "Light UV", "Light Depth", "Atlas UV", "Texel Density"};
  const char* fit_modes[] = {"Stable Sphere", "Tight Light-Space AABB"};
  auto fit_mode = static_cast<int>(render_settings.shadow_cascade_fit_mode);
  if (ImGui::Combo("Fit policy", &fit_mode, fit_modes, IM_ARRAYSIZE(fit_modes))) {
    render_settings.shadow_cascade_fit_mode = static_cast<RenderSettings::ShadowCascadeFitMode>(
        glm::clamp(fit_mode, 0, static_cast<int>(IM_ARRAYSIZE(fit_modes)) - 1));
  }
  ImGui::TextUnformatted("Stable Sphere is quantized and snapped; Tight AABB is intentionally unsnapped.");
  ImGui::TextUnformatted("Split policy: Practical Log/Uniform");
  if (ImGui::TreeNode("Distance")) {
    if (ImGui::DragFloat("Max shadow distance", &render_settings.max_shadow_distance, 1.0f, 10.f, 1000.f)) {
      render_settings.max_shadow_distance = glm::clamp(render_settings.max_shadow_distance, 10.f, 1000.f);
    }
    if (ImGui::DragFloat("Cascade transition width", &render_settings.shadow_cascade_transition_width, 0.1f, 0.0f,
                         100.0f)) {
      render_settings.shadow_cascade_transition_width = glm::max(render_settings.shadow_cascade_transition_width, 0.0f);
    }
    if (ImGui::DragFloat("Distance fade", &render_settings.shadow_distance_fade, 0.5f, 0.0f,
                         render_settings.max_shadow_distance)) {
      render_settings.shadow_distance_fade =
          glm::clamp(render_settings.shadow_distance_fade, 0.0f, render_settings.max_shadow_distance);
    }
    if (ImGui::DragFloat("Split lambda", &render_settings.shadow_cascade_split_lambda, 0.01f, 0.0f, 1.0f)) {
      render_settings.shadow_cascade_split_lambda = glm::clamp(render_settings.shadow_cascade_split_lambda, 0.0f, 1.0f);
    }
    const auto split_0 = render_settings.GetShadowCascadeSplit(0);
    const auto split_1 = render_settings.GetShadowCascadeSplit(1);
    const auto split_2 = render_settings.GetShadowCascadeSplit(2);
    ImGui::Text("Effective splits: %.3f, %.3f, %.3f, 1.000", split_0, split_1, split_2);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Sampling")) {
    ImGui::TextUnformatted("Shadow filtering: PCF");
    ImGui::TextUnformatted("Samples per filtered shadow: 8 (shader constant)");
    ImGui::TextUnformatted("Directional PCF radius: light size in world units.");
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Diagnostics")) {
    const auto& graphics_settings = ApplicationContext::Get().GetApplicationInfo().graphics_settings;
    const auto resolution = graphics_settings.directional_light_shadow_map_resolution;
    ImGui::Text(
        "Shadow map resolution: %s (%u x %u)",
        GraphicsInitializationSettings::ShadowMapResolutionQualityName(graphics_settings.shadow_map_resolution_quality),
        resolution, resolution);
    ImGui::Text("Directional shadow layer: %u x %u", resolution, resolution);
    ImGui::Text("Single shadow light viewport: %u x %u", resolution, resolution);
    ImGui::Text("2-4 shadow light viewports: %u x %u", resolution / 2, resolution / 2);
    if (ImGui::Combo("Mode", &render_settings.shadow_debug_mode, shadow_debug_modes,
                     IM_ARRAYSIZE(shadow_debug_modes))) {
      render_settings.shadow_debug_mode =
          glm::clamp(render_settings.shadow_debug_mode, 0, static_cast<int>(IM_ARRAYSIZE(shadow_debug_modes)) - 1);
    }
    if (ImGui::DragInt("Directional light", &render_settings.shadow_debug_selected_light, 1, 0, 31)) {
      render_settings.shadow_debug_selected_light = glm::max(render_settings.shadow_debug_selected_light, 0);
    }
    if (ImGui::DragInt("Cascade", &render_settings.shadow_debug_selected_cascade, 1, 0, 3)) {
      render_settings.shadow_debug_selected_cascade = glm::clamp(render_settings.shadow_debug_selected_cascade, 0, 3);
    }
    ImGui::TextUnformatted("Light UV, light depth, atlas UV, and texel density use the selected cascade.");
    ImGui::TreePop();
  }
}

void InspectStrandsSettings(RenderSettings& render_settings) {
  ImGui::DragFloat("Curve subdivision factor", &render_settings.strands_subdivision_x_factor, 1.0f, 1.0f, 1000.0f);
  ImGui::DragFloat("Ring subdivision factor", &render_settings.strands_subdivision_y_factor, 1.0f, 1.0f, 1000.0f);
  ImGui::DragInt("Max curve subdivision", &render_settings.strands_subdivision_max_x, 1, 1, 15);
  ImGui::DragInt("Max ring subdivision", &render_settings.strands_subdivision_max_y, 1, 1, 15);
}

std::string FormatDdgiBytes(const uint64_t bytes) {
  constexpr std::array<const char*, 4> units{"B", "KiB", "MiB", "GiB"};
  double value = static_cast<double>(bytes);
  size_t unit = 0;
  while (value >= 1024.0 && unit + 1 < units.size()) {
    value /= 1024.0;
    ++unit;
  }
  if (unit == 0)
    return std::to_string(static_cast<uint64_t>(value)) + " " + units[unit];
  auto text = std::to_string(value);
  text.resize(text.find('.') + 2);
  return text + " " + units[unit];
}

const char* GetDdgiSceneStatus(const RenderLayer::DdgiInspectorSnapshot& snapshot,
                               const RenderLayer::DdgiSessionState& session) {
  if (!snapshot.enabled)
    return "Disabled";
  if (!snapshot.validation_error.empty())
    return "Invalid";
  if (session.reset_history_requested)
    return "Reset pending";
  if (session.pause_updates)
    return "Paused";
  if (std::any_of(snapshot.cascades.begin(), snapshot.cascades.end(), [](const auto& volume) {
        return volume.warmup_active;
      }))
    return "Relocation warmup";
  if (snapshot.cascades.empty() ||
      std::any_of(snapshot.cascades.begin(), snapshot.cascades.end(), [](const auto& volume) {
        return !volume.sampling_complete;
      }))
    return "Updating";
  return "Updating / history window filled";
}

const char* GetDdgiCascadeStatus(const DdgiCascadeRuntimeStats& volume, const RenderLayer::DdgiSessionState& session) {
  if (!volume.resources_ready)
    return "Resources unavailable";
  if (session.reset_history_requested)
    return "Reset pending";
  if (session.pause_updates)
    return volume.pending_scene_changes ? "Paused / stale frozen" : "Paused / frozen";
  if (volume.warmup_active)
    return "Relocation warmup";
  return volume.has_valid_probe_history ? "Updating" : "Initializing";
}

void DrawDdgiSelectedProbeStateReadoutCompact(const DdgiProbeDebugDataView& debug_data, const uint32_t logical_probe,
                                              const uint32_t physical_probe) {
  const auto metadata_offset = static_cast<size_t>(physical_probe) * 3ull;
  if (!debug_data.metadata || debug_data.metadata->size() < metadata_offset + 3ull) {
    ImGui::TextDisabled("Probe state: pending or unavailable");
    return;
  }
  const auto irradiance = (*debug_data.metadata)[metadata_offset];
  const auto visibility = (*debug_data.metadata)[metadata_offset + 1ull];
  const auto state = (*debug_data.metadata)[metadata_offset + 2ull];
  ImGui::Text("Logical / physical: %u / %u", logical_probe, physical_probe);
  ImGui::Text("State: %s", state.w >= 0.5f ? "active" : "inactive");
  ImGui::Text("Irradiance: %.3f, %.3f, %.3f", irradiance.x, irradiance.y, irradiance.z);
  ImGui::Text("Hit / backface: %.1f%% / %.1f%%", glm::clamp(irradiance.w, 0.0f, 1.0f) * 100.0f,
              glm::clamp(visibility.y, 0.0f, 1.0f) * 100.0f);
  ImGui::Text("Visibility distance: %.3f", visibility.x);
  ImGui::Text("Relocation: %.3f, %.3f, %.3f", state.x, state.y, state.z);
}

void DrawDdgiSelectedRayReadoutCompact(const DdgiProbeDebugDataView& debug_data) {
  if (!debug_data.selected_ray_samples_available || !debug_data.selected_ray_samples) {
    ImGui::TextDisabled("Selected rays: pending or unavailable");
    return;
  }
  const auto sample_count =
      glm::min(debug_data.selected_ray_sample_count, static_cast<uint32_t>(debug_data.selected_ray_samples->size()));
  uint32_t frontface = 0;
  uint32_t backface = 0;
  uint32_t miss = 0;
  uint32_t inactive = 0;
  for (uint32_t index = 0; index < sample_count; ++index) {
    const auto& sample = (*debug_data.selected_ray_samples)[index];
    if (sample.hit_count == 0u) {
      sample.hit_info.color.a <= -1.5f ? ++inactive : ++miss;
    } else {
      sample.hit_info.color.a < 0.0f ? ++backface : ++frontface;
    }
  }
  ImGui::Text("Selected rays: %u", sample_count);
  ImGui::Text("Front / back / miss / inactive: %u / %u / %u / %u", frontface, backface, miss, inactive);
}

GizmoSettings MakeBoundingVolumeGizmoSettings();

void InspectDdgiRuntime(InspectorContext& context, RenderLayer& render_layer) {
  auto& session = render_layer.GetDdgiSessionState();
  const auto snapshot = render_layer.GetDdgiInspectorSnapshot();
  const auto scene = context.scene ? context.scene : ApplicationContext::Get().GetActiveScene();
  if (session.show_probes && context.editor_layer) {
    const auto settings = MakeBoundingVolumeGizmoSettings();
    const auto lighting = scene ? scene->environmental_lighting.Get<EnvironmentalLighting>() : nullptr;
    for (const auto& volume : snapshot.cascades) {
      const auto extent_x = volume.probe_step_x * static_cast<float>(glm::max(volume.probe_counts.x - 1, 1));
      const auto extent_y = volume.probe_step_y * static_cast<float>(glm::max(volume.probe_counts.y - 1, 1));
      const auto extent_z = volume.probe_step_z * static_cast<float>(glm::max(volume.probe_counts.z - 1, 1));
      glm::mat4 transform(1.0f);
      transform[0] = glm::vec4(extent_x, 0.0f);
      transform[1] = glm::vec4(extent_y, 0.0f);
      transform[2] = glm::vec4(extent_z, 0.0f);
      transform[3] = glm::vec4(volume.first_probe + (extent_x + extent_y + extent_z) * 0.5f, 1.0f);
      context.editor_layer->DrawGizmoCube(glm::vec4(1.0f, 0.45f, 0.05f, 0.35f), transform, 1.0f, settings);
    }
  }

  if (ImGui::TreeNodeEx("Overview", ImGuiTreeNodeFlags_DefaultOpen)) {
    const auto resident_probes = std::accumulate(snapshot.cascades.begin(), snapshot.cascades.end(), 0ull,
                                                 [](const uint64_t total, const auto& volume) {
                                                   return total + volume.probe_count;
                                                 });
    const auto resident_bytes = std::accumulate(snapshot.cascades.begin(), snapshot.cascades.end(), 0ull,
                                                [](const uint64_t total, const auto& volume) {
                                                  return total + volume.resident_byte_size;
                                                });
    ImGui::Text("Status: %s", GetDdgiSceneStatus(snapshot, session));
    ImGui::Text("Cascades: %zu   Probes: %llu   Memory: %s", snapshot.cascades.size(),
                static_cast<unsigned long long>(resident_probes), FormatDdgiBytes(resident_bytes).c_str());
    if (!snapshot.validation_error.empty()) {
      ImGui::TextColored({1.0f, 0.35f, 0.25f, 1.0f}, "%s", snapshot.validation_error.c_str());
    }
    ImGui::Checkbox("Pause updates", &session.pause_updates);
    if (!session.pause_updates) {
      ImGui::SameLine();
      if (ImGui::Button("Reset history"))
        render_layer.RequestDdgiHistoryReset();
    }

    if (ImGui::BeginTable("DdgiRuntimeCascades", 6,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable)) {
      ImGui::TableSetupColumn("Cascade");
      ImGui::TableSetupColumn("Grid");
      ImGui::TableSetupColumn("Probes");
      ImGui::TableSetupColumn("Memory");
      ImGui::TableSetupColumn("State");
      ImGui::TableSetupColumn("Target");
      ImGui::TableHeadersRow();
      for (const auto& volume : snapshot.cascades) {
        ImGui::PushID(static_cast<int>(volume.stable_entity_id));
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(volume.name.empty() ? "DDGI Cascade" : volume.name.c_str());
        ImGui::TextDisabled("%llu", static_cast<unsigned long long>(volume.stable_entity_id));
        ImGui::TableNextColumn();
        ImGui::Text("%d x %d x %d", volume.probe_counts.x, volume.probe_counts.y, volume.probe_counts.z);
        ImGui::TableNextColumn();
        ImGui::Text("%u", volume.probe_count);
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(FormatDdgiBytes(volume.resident_byte_size).c_str());
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(GetDdgiCascadeStatus(volume, session));
        ImGui::TextDisabled("%s", DdgiRuntime::FormatUpdateReasons(volume.last_probe_update_reasons).c_str());
        ImGui::TableNextColumn();
        if (session.selected_cascade_id == volume.stable_entity_id) {
          ImGui::TextUnformatted("Selected");
        } else if (ImGui::SmallButton("Inspect")) {
          session.selected_cascade_id = volume.stable_entity_id;
          session.selected_probe_grid = glm::ivec3(0);
        }
        ImGui::PopID();
      }
      ImGui::EndTable();
    }
    ImGui::TreePop();
  }

  const auto selected = std::find_if(snapshot.cascades.begin(), snapshot.cascades.end(), [&](const auto& volume) {
    return volume.stable_entity_id == session.selected_cascade_id;
  });
  if (selected == snapshot.cascades.end() && session.selected_cascade_id != 0u) {
    session.selected_cascade_id = 0u;
    session.selected_probe_grid = glm::ivec3(0);
  }

  if (ImGui::TreeNode("Visualization")) {
    ImGui::Checkbox("Show probes", &session.show_probes);
    const auto scene_camera = context.editor_layer ? context.editor_layer->GetSceneCamera() : nullptr;
    if (snapshot.cascades.empty()) {
      ImGui::TextDisabled("Unavailable: no DDGI cascade with runtime resources.");
    } else if (!scene_camera) {
      ImGui::TextDisabled("Unavailable: the editor scene viewport has no camera.");
    } else if (scene_camera->camera_render_mode != Camera::CameraRenderMode::Rasterization) {
      ImGui::TextDisabled("Unavailable: visualization supports the raster editor scene viewport only.");
    } else {
      ImGui::TextDisabled("Overlay target: raster editor scene viewport (the game main camera remains clean).");
    }
    const char* color_modes[]{"Atlas irradiance", "Probe metadata", "Visibility", "Hit ratio"};
    ImGui::Combo("Probe color", &session.probe_visualization_mode, color_modes, IM_ARRAYSIZE(color_modes));
    const char* depth_modes[]{"Depth tested", "X-ray"};
    ImGui::Combo("Depth mode", &session.probe_visualization_depth_mode, depth_modes, IM_ARRAYSIZE(depth_modes));
    ImGui::DragFloat("Radius / minimum spacing", &session.probe_visualization_radius_fraction, 0.005f, 0.001f, 1.0f,
                     "%.3f");
    ImGui::DragFloat("Probe intensity", &session.probe_visualization_intensity, 0.01f, 0.0f, 1000.0f);
    ImGui::SliderFloat("Probe opacity", &session.probe_visualization_alpha, 0.0f, 1.0f);
    if (selected != snapshot.cascades.end()) {
      ImGui::SeparatorText(selected->name.empty() ? "Diagnostic target" : selected->name.c_str());
      ImGui::Checkbox("Show selected marker", &session.show_selected_probe);
      ImGui::DragFloat("Selected marker scale", &session.selected_probe_visualization_scale, 0.05f, 1.0f, 20.0f);
      if (ImGui::DragInt3("Probe grid", &session.selected_probe_grid.x, 1.0f)) {
        session.selected_probe_grid =
            glm::clamp(session.selected_probe_grid, glm::ivec3(0), glm::max(selected->probe_counts - 1, glm::ivec3(0)));
      }
      ImGui::Checkbox("Show rays", &session.show_rays);
      ImGui::SliderFloat("Ray opacity", &session.ray_visualization_alpha, 0.0f, 1.0f);
    } else {
      ImGui::TextDisabled("Choose Inspect on a runtime volume to select a probe or show rays.");
    }
    ImGui::TreePop();
  }

  session.selected_probe_readback_requested = false;
  if (ImGui::TreeNode("Diagnostics")) {
    if (selected != snapshot.cascades.end()) {
      ImGui::Checkbox("Read selected probe state", &session.show_selected_probe_state);
      session.selected_probe_readback_requested = session.show_selected_probe_state;
    }
    const auto debug_data = render_layer.RefreshDdgiProbeDebugData();
    if (selected != snapshot.cascades.end() && session.show_selected_probe_state) {
      const auto logical =
          DdgiRuntime::GetProbeCount(selected->probe_counts) == 0u
              ? 0u
              : static_cast<uint32_t>(
                    session.selected_probe_grid.x + session.selected_probe_grid.y * selected->probe_counts.x +
                    session.selected_probe_grid.z * selected->probe_counts.x * selected->probe_counts.y);
      const auto physical_grid = session.selected_probe_grid + selected->probe_scroll_offset;
      const auto wrapped = (physical_grid % glm::max(selected->probe_counts, glm::ivec3(1)) +
                            glm::max(selected->probe_counts, glm::ivec3(1))) %
                           glm::max(selected->probe_counts, glm::ivec3(1));
      const auto physical = static_cast<uint32_t>(wrapped.x + wrapped.y * selected->probe_counts.x +
                                                  wrapped.z * selected->probe_counts.x * selected->probe_counts.y);
      DrawDdgiSelectedProbeStateReadoutCompact(debug_data, logical, physical);
    }
    if (selected != snapshot.cascades.end() && session.show_rays) {
      DrawDdgiSelectedRayReadoutCompact(debug_data);
    }
    ImGui::SeparatorText("Emissive inventory");
    ImGui::Text("Instances / groups / fallback: %u / %u / %u", snapshot.aggregate.emissive_eligible_instance_count,
                snapshot.aggregate.emissive_distribution_count,
                snapshot.aggregate.emissive_fallback_distribution_count);
    ImGui::Text("Logical / stored triangles: %llu / %llu",
                static_cast<unsigned long long>(snapshot.aggregate.emissive_logical_triangle_count),
                static_cast<unsigned long long>(snapshot.aggregate.emissive_stored_triangle_count));
    ImGui::Text("Build / upload: %.3f / %.3f ms", snapshot.aggregate.emissive_distribution_build_ms,
                snapshot.aggregate.emissive_distribution_upload_ms);
    ImGui::Text("Estimated power: %.3f", snapshot.aggregate.emissive_estimated_power);
    ImGui::Text("Sampling enabled cascades / candidate rays: %u / %llu",
                snapshot.aggregate.emissive_sampling_enabled_volume_count,
                static_cast<unsigned long long>(snapshot.aggregate.emissive_sampling_candidate_ray_count));
    if (snapshot.aggregate.emissive_excluded_instance_count != 0u) {
      ImGui::TextColored({1.0f, 0.65f, 0.2f, 1.0f}, "Some positive-emission instances are excluded from sampling.");
    }
    if (snapshot.aggregate.emissive_fallback_distribution_count != 0u) {
      ImGui::TextColored({1.0f, 0.65f, 0.2f, 1.0f},
                         "%u non-uniform or deforming emitters use exact per-instance distributions.",
                         snapshot.aggregate.emissive_fallback_distribution_count);
    }
    if (snapshot.aggregate.emissive_unrepresentable_probability_count != 0u) {
      ImGui::TextColored({1.0f, 0.35f, 0.25f, 1.0f},
                         "%u positive-power entries have no representable sampling probability.",
                         snapshot.aggregate.emissive_unrepresentable_probability_count);
    }
    ImGui::TreePop();
  }
}

void RenderEnvironmentalLightingProbeBounds(const std::shared_ptr<EditorLayer>& editor_layer,
                                            const EnvironmentalLighting& lighting, const glm::vec4& color);

void RenderReflectionProbeBounds(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Scene>& scene,
                                 const glm::vec4& color);

void InspectRenderLayerDebugRendering(InspectorContext& context) {
  if (!ImGui::TreeNodeEx("Debug rendering##RenderLayer")) {
    return;
  }
  static bool display_reflection_probe_bounds = false;
  static auto reflection_probe_bounds_color = glm::vec4(1.0f, 0.78f, 0.1f, 0.35f);
  ImGui::Checkbox("Display all reflection probe bounds##RenderLayer", &display_reflection_probe_bounds);
  ImGui::ColorEdit4("Reflection probe bounds color##RenderLayer",
                    static_cast<float*>(static_cast<void*>(&reflection_probe_bounds_color)));
  if (display_reflection_probe_bounds) {
    const auto scene = context.scene ? context.scene : ApplicationContext::Get().GetActiveScene();
    const auto editor_layer =
        context.editor_layer ? context.editor_layer : ApplicationContext::Get().GetLayer<EditorLayer>();
    RenderReflectionProbeBounds(editor_layer, scene, reflection_probe_bounds_color);
  }
  ImGui::TreePop();
}

void InspectSdfgiRuntime(InspectorContext& context) {
  const auto scene = context.scene ? context.scene : ApplicationContext::Get().GetActiveScene();
  const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
  if (!runtime) {
    ImGui::TextUnformatted("No Automatic SDFGI runtime. Select Automatic SDFGI in Environmental Lighting.");
    return;
  }
  auto& debug = *runtime->debug;
  ImGui::Text("Provider: Automatic SDFGI -> %s", runtime->published ? "Automatic SDFGI" : "Environment");
  if (!runtime->published)
    ImGui::TextWrapped("%s", runtime->fallback_reason.c_str());
  ImGui::Checkbox("Enable selected-camera diagnostics", &debug.enabled);
  ImGui::SetItemTooltip("Session-only visualization. The selected camera does not become the GI anchor.");
  ImGui::Checkbox("Freeze field", &debug.frozen);
  ImGui::SameLine();
  if (ImGui::Button("Single step")) {
    debug.frozen = true;
    debug.single_step = true;
  }
  ImGui::SetItemTooltip(
      "Advance one GI update while frozen. Scene and settings changes wait until stepping or resuming.");
  if (ImGui::BeginCombo("Debug camera",
                        debug.camera_id ? std::to_string(debug.camera_id).c_str() : "Editor Scene camera")) {
    if (ImGui::Selectable("Editor Scene camera", debug.camera_id == 0))
      debug.camera_id = 0;
    if (scene)
      if (const auto owners = scene->UnsafeGetPrivateComponentOwnersList<Camera>())
        for (const auto entity : *owners) {
          const auto camera = scene->GetOrSetPrivateComponent<Camera>(entity).lock();
          if (camera &&
              ImGui::Selectable(
                  (scene->GetEntityName(entity) + "##" + std::to_string(camera->GetHandle().GetValue())).c_str(),
                  debug.camera_id == camera->GetHandle().GetValue()))
            debug.camera_id = camera->GetHandle().GetValue();
        }
    ImGui::EndCombo();
  }
  if (ImGui::BeginCombo("View", GetSdfgiDebugViewName(debug.view))) {
    for (uint32_t i = 0; i <= static_cast<uint32_t>(SdfgiDebugView::Contributors); ++i)
      if (ImGui::Selectable(GetSdfgiDebugViewName(static_cast<SdfgiDebugView>(i)),
                            i == static_cast<uint32_t>(debug.view)))
        debug.view = static_cast<SdfgiDebugView>(i);
    ImGui::EndCombo();
  }
  ImGui::SetItemTooltip(
      "Visibility: red hidden, white visible. Fallback: red Environment, green SDFGI. "
      "Contributors: green included, cyan receiver-only, red excluded. Diffuse/specular isolate SDFGI lighting.");
  const auto slider = [](const char* label, uint32_t& value, int maximum) {
    int input = std::min(static_cast<int>(value), maximum);
    if (ImGui::SliderInt(label, &input, 0, maximum))
      value = static_cast<uint32_t>(input);
  };
  slider("Cascade", debug.cascade, std::max(0, static_cast<int>(runtime->cascades.size()) - 1));
  const auto probes = runtime->settings.ProbeSize();
  slider("Probe (X, Z, Y order)", debug.probe, probes.x * probes.y * probes.z - 1);
  slider("Distance slice Z", debug.slice, runtime->settings.GridSize().z - 1);
  ImGui::Checkbox("Depth-test overlays", &debug.depth_test);
  ImGui::SetItemTooltip("Disable to inspect probes and visibility through geometry.");
  if (debug.cascade < runtime->cascades.size()) {
    const auto& cascade = runtime->cascades[debug.cascade];
    const glm::vec3 cell(debug.probe % probes.x, debug.probe / (probes.x * probes.z),
                         (debug.probe / probes.x) % probes.z);
    const auto position = (glm::vec3(cascade.position - cascade.size / 2) +
                           cell * static_cast<float>(runtime->settings.probe_spacing_cells)) *
                          cascade.cell_size / glm::vec3(1, SdfgiYMultiplier(runtime->settings.vertical_scale), 1);
    ImGui::Text("Selected probe world: %.3f, %.3f, %.3f", position.x, position.y, position.z);
  }
  if (!debug.failure.empty())
    ImGui::TextWrapped("Visualization unavailable: %s", debug.failure.c_str());
}

bool InspectRenderLayer(InspectorContext& context, RenderLayer& render_layer) {
  bool open = render_layer.enable_inspection;
  if (!ImGui::Begin(render_layer.GetLayerName().c_str(), &open)) {
    ImGui::End();
    render_layer.enable_inspection = open;
    return false;
  }
  const auto draw_ddgi_tab = [&] {
    if (ImGui::BeginTabItem("DDGI")) {
      InspectDdgiRuntime(context, render_layer);
      ImGui::EndTabItem();
    }
  };

  if (ImGui::BeginTabBar("RenderLayerInspectionTabs")) {
    if (ImGui::BeginTabItem("General")) {
      InspectRenderLayerGeneralSettings(render_layer);
      InspectRenderLayerDebugRendering(context);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Stats")) {
      InspectRenderLayerStats(render_layer);
      ImGui::EndTabItem();
    }
    draw_ddgi_tab();
    if (ImGui::BeginTabItem("Automatic SDFGI")) {
      InspectSdfgiRuntime(context);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Shadow")) {
      InspectShadowSettings(render_layer.render_settings);
      ImGui::EndTabItem();
    }
    if (Platform::MeshShaderEnabled() && ImGui::BeginTabItem("Strands")) {
      InspectStrandsSettings(render_layer.render_settings);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  ImGui::End();
  render_layer.enable_inspection = open;
  return false;
}

std::shared_ptr<Scene> ResolveInspectorScene(InspectorContext& context) {
  return context.scene ? context.scene : ApplicationContext::Get().GetActiveScene();
}

bool InspectScene(InspectorContext& context, Scene& scene) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool modified = false;
  if (&scene == ApplicationContext::Get().GetActiveScene().get()) {
    if (editor_layer->DragAndDropButton<Camera>(scene.main_camera, "Main Camera", true))
      modified = true;
  }
  if (ImGui::TreeNodeEx("Environment Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (editor_layer->DragAndDropButton<GlobalReflectionProbe>(scene.global_reflection_probe_fallback,
                                                               "Global Reflection Probe Fallback"))
      modified = true;
    if (editor_layer->DragAndDropButton<EnvironmentalLighting>(scene.environmental_lighting, "Environmental Lighting"))
      modified = true;
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Systems")) {
    if (ImGui::BeginPopupContextWindow("SystemInspectorPopup")) {
      ImGui::Text("Add system: ");
      ImGui::Separator();
      static float rank = 0.0f;
      ImGui::DragFloat("Rank", &rank, 1.0f, 0.0f, 999.0f);
      for (const auto& i : Serialization::GetRegisteredSystemTypes()) {
        const auto id = i.second;
        const auto name = i.first;
        if (!scene.HasSystemType(id) && ImGui::Button(name.c_str())) {
          scene.CreateSystemByTypeId(id, rank);
        }
      }
      ImGui::Separator();
      ImGui::EndPopup();
    }
    for (const auto& i : scene.PeekSystems()) {
      if (ImGui::CollapsingHeader(i.second->GetTypeName().c_str())) {
        bool enabled = i.second->Enabled();
        if (ImGui::Checkbox("Enabled", &enabled)) {
          if (i.second->Enabled() != enabled) {
            if (enabled) {
              i.second->Enable();
              modified = true;
            } else {
              i.second->Disable();
              modified = true;
            }
          }
        }
        InspectorContext system_context;
        system_context.editor_layer = editor_layer;
        system_context.scene = scene.GetSelfScene();
        if (InspectorRegistry::GetInstance().Inspect(system_context, *i.second)) {
          modified = true;
        }
      }
    }
    ImGui::TreePop();
  }
  return modified;
}

bool InspectSpotLight(InspectorContext&, SpotLight& light) {
  bool changed = false;
  if (ImGui::Checkbox("Cast Shadow", &light.cast_shadow))
    changed = false;
  if (light.cast_shadow && ImGui::DragFloat("Shadow distance", &light.shadow_distance, 0.001f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::ColorEdit3("Color", &light.diffuse[0]))
    changed = false;
  if (ImGui::DragFloat("Intensity", &light.diffuse_brightness, 0.01f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Range", &light.range, 0.01f, 0.0f, 9999.0f))
    changed = false;
  if (ImGui::DragFloat("Bias", &light.bias, 0.001f, 0.0f, 999.0f))
    changed = false;

  if (ImGui::DragFloat("Constant", &light.constant, 0.01f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Linear", &light.linear, 0.001f, 0, 1, "%.3f"))
    changed = false;
  if (ImGui::DragFloat("Quadratic", &light.quadratic, 0.001f, 0, 10, "%.4f"))
    changed = false;

  if (ImGui::DragFloat("Inner Degrees", &light.inner_degrees, 0.1f, 0.0f, light.outer_degrees))
    changed = false;
  if (ImGui::DragFloat("Outer Degrees", &light.outer_degrees, 0.1f, light.inner_degrees, 180.0f))
    changed = false;
  if (ImGui::DragFloat("Light Size", &light.light_size, 0.001f, 0.0f, 999.0f))
    changed = false;

  return changed;
}

bool InspectPointLight(InspectorContext&, PointLight& light) {
  bool changed = false;
  if (ImGui::Checkbox("Cast Shadow", &light.cast_shadow))
    changed = false;
  if (light.cast_shadow && ImGui::DragFloat("Shadow distance", &light.shadow_distance, 0.001f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::ColorEdit3("Color", &light.diffuse[0]))
    changed = false;
  if (ImGui::DragFloat("Intensity", &light.diffuse_brightness, 0.01f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Range", &light.range, 0.01f, 0.0f, 9999.0f))
    changed = false;
  if (ImGui::DragFloat("Bias", &light.bias, 0.001f, 0.0f, 999.0f))
    changed = false;

  if (ImGui::DragFloat("Constant", &light.constant, 0.01f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Linear", &light.linear, 0.0001f, 0, 1, "%.4f"))
    changed = false;
  if (ImGui::DragFloat("Quadratic", &light.quadratic, 0.00001f, 0, 10, "%.5f"))
    changed = false;

  if (ImGui::DragFloat("Light Size", &light.light_size, 0.001f, 0.0f, 999.0f))
    changed = false;

  return changed;
}

bool InspectDirectionalLight(InspectorContext&, DirectionalLight& light) {
  bool changed = false;
  if (ImGui::Checkbox("Cast Shadow", &light.cast_shadow))
    changed = false;
  if (ImGui::ColorEdit3("Color", &light.diffuse[0]))
    changed = false;
  if (ImGui::DragFloat("Intensity", &light.diffuse_brightness, 0.01f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Bias (texels)", &light.bias, 1.f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Slope Bias (texels)", &light.slope_bias, 1.f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Normal Offset (texels)", &light.normal_offset, 1.f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Light Size", &light.light_size, 0.001f, 0.0f, 999.0f))
    changed = false;
  return changed;
}

bool InspectSkyIllumination(InspectorContext&, SkyIllumination& sky_illumination) {
  bool changed = false;
  auto& atmosphere = sky_illumination.atmosphere;
  if (ImGui::TreeNodeEx("Atmosphere Settings")) {
    if (ImGui::DragFloat("Earth Radius (km)", &atmosphere.earth_radius, 1.0f, 0.0f,
                         atmosphere.atmosphere_radius - 1.0f)) {
      atmosphere.earth_radius = glm::clamp(atmosphere.earth_radius, 1.0f, atmosphere.atmosphere_radius - 1.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Atmosphere Radius (km)", &atmosphere.atmosphere_radius, 1.0f, atmosphere.earth_radius + 1.0f,
                         100000.0f)) {
      atmosphere.atmosphere_radius =
          glm::clamp(atmosphere.atmosphere_radius, atmosphere.earth_radius + 1.0f, 100000.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Rayleigh scale height (m)", &atmosphere.hr, 1.0f, 0.0f, 100000.0f)) {
      atmosphere.hr = glm::clamp(atmosphere.hr, 0.0f, 10000.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Mie scale height (m)", &atmosphere.hm, 1.0f, 0.0f, 100000.0f)) {
      atmosphere.hm = glm::clamp(atmosphere.hm, 0.0f, 10000.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Mie scattering mean cosine", &atmosphere.g, 0.001f, 0.0f, 0.999f, "%.4f")) {
      atmosphere.g = glm::clamp(atmosphere.g, 0.0f, 0.999f);
      changed = true;
    }
    if (ImGui::DragInt("Samples", &atmosphere.num_samples, 1, 128)) {
      atmosphere.num_samples = glm::clamp(atmosphere.num_samples, 1, 128);
      changed = true;
    }
    if (ImGui::DragInt("Samples light", &atmosphere.num_samples_light, 1, 128)) {
      atmosphere.num_samples_light = glm::clamp(atmosphere.num_samples_light, 1, 128);
      changed = true;
    }

    if (ImGui::DragFloat("Intensity", &atmosphere.intensity, 0.1f, 0.0f, 10.f)) {
      changed = true;
    }
    ImGui::TreePop();
  }
  if (ImGui::DragFloat("Gamma", &sky_illumination.gamma, 0.01f, 0.0f, 5.f)) {
    changed = true;
  }
  static glm::vec3 angles = glm::vec3(90, 0, 0);
  if (ImGui::DragFloat3("Sun angle", &angles.x, 1.0f)) {
    sky_illumination.sun_direction = glm::quat(glm::radians(angles)) * glm::vec3(0, 0, -1);
    changed = true;
  }

  if (ImGui::ColorEdit3("Ground color", &sky_illumination.ground_color.x)) {
    changed = true;
  }

  if (ImGui::DragFloat("Ground transmittance", &sky_illumination.ground_transmittance, 0.01f, 0.0f, 1.0f)) {
    sky_illumination.ground_transmittance = glm::clamp(sky_illumination.ground_transmittance, 0.0f, 1.f);
    changed = true;
  }
  return changed;
}

bool InspectMesh(InspectorContext&, Mesh& mesh) {
  ImGui::Text(("Vertices size: " + std::to_string(mesh.GetVerticesAmount())).c_str());
  ImGui::Text(("Triangle amount: " + std::to_string(mesh.GetTriangleAmount())).c_str());
  if (mesh.GetVerticesAmount() != 0) {
    FileUtils::SaveFile(
        "Export as OBJ", "Mesh", {".obj"},
        [&](const std::filesystem::path& path) {
          mesh.Export(path);
        },
        false);
  }
  return false;
}

bool InspectCubemap(InspectorContext& context, Cubemap& cubemap) {
  bool changed = false;
  if (ImGui::TreeNode("Sky illumination")) {
    static bool auto_rebuild = true;
    ImGui::Checkbox("Auto refresh", &auto_rebuild);
    static SkyIllumination sky_illumination{};
    const bool rebuild = InspectSkyIllumination(context, sky_illumination);
    if (ImGui::Button("Build") || (auto_rebuild && rebuild)) {
      cubemap.BuildSkyIllumination(sky_illumination);
      changed = true;
    }
    ImGui::TreePop();
  }

  if (const auto& storage = cubemap.RefStorage(); !storage.im_texture_ids.empty()) {
    static float debug_scale = 0.25f;
    ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
    debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
    for (int i = 0; i < 6; i++) {
      ImGui::Image(
          storage.im_texture_ids[i],
          ImVec2(storage.image->GetExtent().width * debug_scale, storage.image->GetExtent().height * debug_scale),
          ImVec2(0, 1), ImVec2(1, 0));
    }
  }

  return changed;
}

bool InspectCubemapPreviewFaces(Cubemap& cubemap) {
  const auto& storage = cubemap.RefStorage();
  if (storage.im_texture_ids.empty()) {
    return false;
  }
  static float debug_scale = 0.25f;
  ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
  debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
  for (int i = 0; i < 6; i++) {
    ImGui::Image(
        storage.im_texture_ids[i],
        ImVec2(storage.image->GetExtent().width * debug_scale, storage.image->GetExtent().height * debug_scale),
        ImVec2(0, 1), ImVec2(1, 0));
  }
  return false;
}

bool InspectLightProbe(InspectorContext&, LightProbe& light_probe) {
  const auto cubemap = light_probe.GetCubemap();
  return cubemap ? InspectCubemapPreviewFaces(*cubemap) : false;
}

bool InspectGlobalReflectionProbe(InspectorContext& context, GlobalReflectionProbe& reflection_probe) {
  bool changed = false;
  if (context.editor_layer) {
    AssetRef source;
    if (context.editor_layer->DragAndDropButton<Cubemap>(source, "Import Cubemap")) {
      changed |= reflection_probe.ConstructFromCubemap(source.Get<Cubemap>());
    }
    if (context.editor_layer->DragAndDropButton<Texture2D>(source, "Import Equirectangular HDR")) {
      const auto cubemap = AssetManager::CreateTemporaryAsset<Cubemap>();
      cubemap->ConvertFromEquirectangularTexture(source.Get<Texture2D>());
      changed |= reflection_probe.ConstructFromCubemap(cubemap);
    }
  }
  ImGui::Text("Canonical: 256x256, 9 mips, RGBA16F");
  if (reflection_probe.GetCanonicalPayloadByteSize() == 0 && reflection_probe.IsRuntimeReady()) {
    ImGui::Text("Payload: GPU resident; downloaded on save");
  } else {
    ImGui::Text("Payload: %zu bytes", reflection_probe.GetCanonicalPayloadByteSize());
  }
  ImGui::Text("Source: %s", reflection_probe.GetSourceKind() == GlobalReflectionProbe::SourceKind::Baked ? "baked"
                            : reflection_probe.GetSourceKind() == GlobalReflectionProbe::SourceKind::Imported
                                ? "imported"
                                : "empty");

  if (ImGui::TreeNode("Packed runtime quality")) {
    struct PackedMetrics {
      uint64_t payload_hash = 0;
      float normalized_rms_error = 0.0f;
      float relative_peak_error = 0.0f;
      bool valid = false;
    };
    static std::unordered_map<uint64_t, PackedMetrics> packed_metrics;
    auto& metrics = packed_metrics[reflection_probe.GetHandle().GetValue()];
    if (metrics.payload_hash != reflection_probe.GetPayloadHash()) {
      metrics.payload_hash = reflection_probe.GetPayloadHash();
      metrics.valid = GlobalReflectionProbe::EvaluatePackedRuntimeQuality(
          reflection_probe.GetCanonicalPayload(), metrics.normalized_rms_error, metrics.relative_peak_error);
    }
    if (metrics.valid) {
      ImGui::Text("B10G11R11 exact-usage capability: %s",
                  reflection_probe.PackedRuntimeFormatSupported() ? "yes" : "no");
      ImGui::Text("Packed NRMSE / peak: %.6f / %.6f", metrics.normalized_rms_error, metrics.relative_peak_error);
      ImGui::Text("Active runtime format: RGBA16F (packed path lacks required 5%% GPU-time evidence)");
    } else {
      ImGui::Text("Packed quality unavailable.");
    }
    ImGui::TreePop();
  }
  const auto cubemap = reflection_probe.GetCubemap();
  if (cubemap && ImGui::TreeNode("Cubemap face preview")) {
    InspectCubemapPreviewFaces(*cubemap);
    ImGui::TreePop();
  }
  return changed;
}

GizmoSettings MakeBoundingVolumeGizmoSettings() {
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_FILL;
  gizmo_settings.depth_test = true;
  gizmo_settings.depth_write = false;
  return gizmo_settings;
}

void RenderEnvironmentalLightingProbeBound(const std::shared_ptr<EditorLayer>& editor_layer,
                                           const EnvironmentalLighting::LocalReflectionProbe& probe,
                                           const glm::vec4& color, const bool include_disabled = false) {
  if (!editor_layer || (!include_disabled && !probe.enabled)) {
    return;
  }
  const auto gizmo_settings = MakeBoundingVolumeGizmoSettings();
  if (probe.shape == static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Sphere)) {
    editor_layer->DrawGizmoSphere(color, probe.transform, glm::max(probe.sphere_radius, 0.001f), gizmo_settings);
    return;
  }
  editor_layer->DrawGizmoCube(color, probe.transform, 1.0f, gizmo_settings);
}

void RenderEnvironmentalLightingProbeBounds(const std::shared_ptr<EditorLayer>& editor_layer,
                                            const EnvironmentalLighting& lighting, const glm::vec4& color) {
  if (!editor_layer) {
    return;
  }
  const auto pack = lighting.GetReflectionProbePack();
  if (!pack)
    return;
  for (size_t index = 0; index < pack->probes.size(); ++index) {
    const auto& probe = pack->probes[index];
    if (editor_layer->IsEnvironmentalLightingGizmoTarget(
            lighting, EnvironmentalLightingGizmoTargetType::LocalReflectionProbe, index, probe.stable_id)) {
      continue;
    }
    RenderEnvironmentalLightingProbeBound(editor_layer, probe, color);
  }
}

void RenderEnvironmentalLightingDebugProbeBounds(const std::shared_ptr<EditorLayer>& editor_layer,
                                                 const EnvironmentalLighting& lighting, const glm::vec4& color) {
  if (!editor_layer) {
    return;
  }
  const auto pack = lighting.GetReflectionProbePack();
  if (!pack)
    return;
  for (size_t index = 0; index < pack->probes.size(); ++index) {
    const auto& probe = pack->probes[index];
    if (probe.debug_draw_bounds &&
        !editor_layer->IsEnvironmentalLightingGizmoTarget(
            lighting, EnvironmentalLightingGizmoTargetType::LocalReflectionProbe, index, probe.stable_id)) {
      RenderEnvironmentalLightingProbeBound(editor_layer, probe, color);
    }
  }
}

void RenderActiveEnvironmentalLightingGizmoBound(const std::shared_ptr<EditorLayer>& editor_layer,
                                                 const EnvironmentalLighting& lighting) {
  if (!editor_layer) {
    return;
  }
  const auto reflection_pack = lighting.GetReflectionProbePack();
  for (size_t index = 0; reflection_pack && index < reflection_pack->probes.size(); ++index) {
    const auto& probe = reflection_pack->probes[index];
    if (editor_layer->IsEnvironmentalLightingGizmoTarget(
            lighting, EnvironmentalLightingGizmoTargetType::LocalReflectionProbe, index, probe.stable_id)) {
      RenderEnvironmentalLightingProbeBound(editor_layer, probe, glm::vec4(0.1f, 0.8f, 1.0f, 0.55f), true);
      return;
    }
  }
}

void RenderReflectionProbeBounds(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Scene>& scene,
                                 const glm::vec4& color) {
  if (!editor_layer || !scene) {
    return;
  }
  if (const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>()) {
    RenderEnvironmentalLightingProbeBounds(editor_layer, *lighting, color);
  }
}

bool InspectSkinnedMesh(InspectorContext&, SkinnedMesh& mesh) {
  ImGui::Text(("Vertices size: " + std::to_string(mesh.GetSkinnedVerticesAmount())).c_str());
  ImGui::Text(("Triangle amount: " + std::to_string(mesh.GetTriangleAmount())).c_str());
  if (mesh.GetSkinnedVerticesAmount() != 0) {
    FileUtils::SaveFile(
        "Export as OBJ", "Mesh", {".obj"},
        [&](const std::filesystem::path& path) {
          mesh.Export(path);
        },
        false);
  }
  return false;
}

bool AuthoringTransformsEqual(const glm::mat4& lhs, const glm::mat4& rhs) {
  for (glm::length_t column = 0; column < 4; ++column) {
    for (glm::length_t row = 0; row < 4; ++row) {
      if (glm::abs(lhs[column][row] - rhs[column][row]) > 1.0e-4f) {
        return false;
      }
    }
  }
  return true;
}

bool InspectEnvironmentalLightingEntryName(std::string& name) {
  std::array<char, 256> buffer{};
  const auto size = std::min(name.size(), buffer.size() - 1u);
  std::copy_n(name.data(), size, buffer.data());
  if (!ImGui::InputText("Name", buffer.data(), buffer.size()))
    return false;
  name = buffer.data();
  return true;
}

bool InspectAuthoringTransform(const std::shared_ptr<EditorLayer>& editor_layer, const char* label,
                               glm::mat4& transform) {
  glm::vec3 position(0.0f);
  glm::vec3 rotation(0.0f);
  glm::vec3 scale(1.0f);
  glm::mat4 normalized(1.0f);
  const bool valid = EditorLayer::TryNormalizeAuthoringTransform(transform, normalized, position, rotation, scale);
  bool changed = !valid;
  if (valid) {
    if (!AuthoringTransformsEqual(transform, normalized)) {
      transform = normalized;
      changed = true;
    }
  } else {
    transform = glm::mat4(1.0f);
  }
  if (ImGui::TreeNode(label)) {
    bool edited = ImGui::DragFloat3("##AuthoringPosition", &position.x, 0.01f);
    ImGui::SameLine();
    if (ImGui::Selectable("Position##Authoring", editor_layer->LocalPositionSelected())) {
      editor_layer->SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Translate);
    }
    edited = ImGui::DragFloat3("##AuthoringRotation", &rotation.x, 0.1f) || edited;
    ImGui::SameLine();
    if (ImGui::Selectable("Rotation##Authoring", editor_layer->LocalRotationSelected())) {
      editor_layer->SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Rotate);
    }
    edited = ImGui::DragFloat3("##AuthoringScale", &scale.x, 0.01f) || edited;
    ImGui::SameLine();
    if (ImGui::Selectable("Scale##Authoring", editor_layer->LocalScaleSelected())) {
      editor_layer->SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Scale);
    }
    if (edited) {
      transform = EditorLayer::ComposeAuthoringTransform(position, rotation, scale);
      changed = true;
    }
    ImGui::TreePop();
  }
  return changed;
}

bool InspectReflectionProbePack(InspectorContext& context, ReflectionProbePack& pack) {
  if (!context.editor_layer)
    return false;
  bool changed = pack.RepairStableIds();
  ImGui::Text("Local probes: %zu", pack.probes.size());
  if (ImGui::Button("Add Local Reflection Probe")) {
    pack.probes.emplace_back();
    (void)pack.RepairStableIds();
    changed = true;
  }
  for (size_t index = 0; index < pack.probes.size(); ++index) {
    auto& probe = pack.probes[index];
    ImGui::PushID(static_cast<int>(index));
    if (ImGui::TreeNode((probe.name + "##ReflectionProbePackEntry").c_str())) {
      changed = InspectEnvironmentalLightingEntryName(probe.name) || changed;
      changed = ImGui::Checkbox("Enabled", &probe.enabled) || changed;
      changed = ImGui::Checkbox("Debug draw bounds", &probe.debug_draw_bounds) || changed;
      changed = ImGui::InputScalar("Stable id", ImGuiDataType_U64, &probe.stable_id) || changed;
      changed = InspectAuthoringTransform(context.editor_layer, "Transform", probe.transform) || changed;
      const char* shapes[]{"Box", "Sphere"};
      changed = ImGui::Combo("Shape", &probe.shape, shapes, IM_ARRAYSIZE(shapes)) || changed;
      changed = ImGui::DragInt("Artist priority", &probe.artist_priority) || changed;
      changed = ImGui::DragFloat("Sphere radius", &probe.sphere_radius, 0.05f, 0.001f, 10000.0f) || changed;
      changed = ImGui::DragFloat("Blend distance", &probe.blend_distance, 0.01f, 0.0f, 10000.0f) || changed;
      changed = ImGui::DragFloat("Reflection intensity", &probe.reflection_intensity, 0.01f, 0.0f, 10000.0f) || changed;
      changed = ImGui::Checkbox("Box projection", &probe.box_projection) || changed;
      changed =
          ImGui::DragFloat3("Projection half extents", &probe.box_projection_extents.x, 0.05f, 0.001f, 10000.0f) ||
          changed;
      ImGui::Text("Payload: %s", probe.HasValidPayload() ? "valid" : "unbaked");
      if (ImGui::Button("Remove")) {
        pack.probes.erase(pack.probes.begin() + static_cast<std::ptrdiff_t>(index));
        changed = true;
        ImGui::TreePop();
        ImGui::PopID();
        break;
      }
      ImGui::TreePop();
    }
    ImGui::PopID();
  }
  if (changed) {
    (void)pack.RepairStableIds();
    pack.SetUnsaved();
  }
  return changed;
}

const char* GetGlobalReflectionProbeSourceKindName(const GlobalReflectionProbe::SourceKind source_kind) {
  switch (source_kind) {
    case GlobalReflectionProbe::SourceKind::Imported:
      return "imported";
    case GlobalReflectionProbe::SourceKind::Baked:
      return "baked";
    case GlobalReflectionProbe::SourceKind::Empty:
    default:
      return "empty";
  }
}

bool QueueEnvironmentalLightingLocalProbeBake(InspectorContext& context,
                                              const std::shared_ptr<ReflectionProbePack>& pack,
                                              EnvironmentalLighting::LocalReflectionProbe& probe) {
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_ERROR("Environmental lighting reflection probe bake was not queued: the render layer is unavailable.")
    return false;
  }
  const auto payload = probe.GetOrCreatePayload();
  return render_layer->QueueGlobalReflectionProbeBakeBatch(
             ResolveInspectorScene(context), {{glm::vec3(probe.transform[3]), payload, pack, probe.stable_id}}) == 1u;
}

uint32_t QueueEnvironmentalLightingLocalProbeBakes(InspectorContext& context, const EnvironmentalLighting& lighting) {
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_ERROR("Environmental lighting reflection probe bakes were not queued: the render layer is unavailable.")
    return 0;
  }
  std::vector<RenderLayer::ReflectionProbeBakeRequest> requests;
  const auto pack = lighting.GetReflectionProbePack();
  if (!pack)
    return 0u;
  requests.reserve(pack->probes.size());
  for (auto& probe : pack->probes) {
    requests.emplace_back(RenderLayer::ReflectionProbeBakeRequest{glm::vec3(probe.transform[3]),
                                                                  probe.GetOrCreatePayload(), pack, probe.stable_id});
  }
  return render_layer->QueueGlobalReflectionProbeBakeBatch(ResolveInspectorScene(context), requests);
}

void InspectEnvironmentalLightingLocalProbePayload(InspectorContext& context,
                                                   const std::shared_ptr<ReflectionProbePack>& pack,
                                                   EnvironmentalLighting::LocalReflectionProbe& probe,
                                                   const bool bake_available) {
  const auto& payload = probe.payload;
  if (!payload || payload->GetSourceKind() == GlobalReflectionProbe::SourceKind::Empty) {
    ImGui::TextColored({1.0f, 0.78f, 0.1f, 1.0f}, "Payload: unbaked (global fallback active)");
  } else {
    ImGui::Text("Payload: %s (%s)", payload->IsRuntimeReady() ? "runtime ready" : "not ready",
                GetGlobalReflectionProbeSourceKindName(payload->GetSourceKind()));
    if (pack && !pack->Saved() && payload->GetSourceKind() == GlobalReflectionProbe::SourceKind::Baked) {
      ImGui::TextColored({1.0f, 0.78f, 0.1f, 1.0f}, "Persistence: unsaved pack payload");
    }
  }

  ImGui::BeginDisabled(!bake_available);
  if (ImGui::Button("Bake Local Probe Payload")) {
    if (QueueEnvironmentalLightingLocalProbeBake(context, pack, probe)) {
      EVOENGINE_LOG("Queued environmental lighting reflection probe bake.")
    }
  }
  ImGui::EndDisabled();
  if (!bake_available && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip("Disable dynamic local probe updates before baking a persistent payload.");
  }
}

bool InspectEnvironmentalLightingSource(InspectorContext& context,
                                        EnvironmentalLighting::IndirectEnvironmentSource& source) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }
  bool changed = false;
  const char* source_types[]{"Engine Default", "Color", "Environmental Map"};
  auto type = static_cast<int>(source.kind);
  if (ImGui::Combo("Indirect source", &type, source_types, IM_ARRAYSIZE(source_types))) {
    source.kind = static_cast<EnvironmentalLighting::IndirectEnvironmentSourceKind>(type);
    changed = true;
  }
  switch (source.kind) {
    case EnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault:
      break;
    case EnvironmentalLighting::IndirectEnvironmentSourceKind::Color:
      changed = ImGui::ColorEdit3("Indirect color", &source.color.x) || changed;
      changed = ImGui::DragFloat("Indirect gamma", &source.gamma, 0.01f, 0.0f, 10.0f) || changed;
      break;
    case EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap:
      changed =
          editor_layer->DragAndDropButton<EnvironmentalMap>(source.environmental_map, "Indirect Environmental Map") ||
          changed;
      changed = ImGui::DragFloat("Indirect gamma", &source.gamma, 0.01f, 0.0f, 10.0f) || changed;
      changed = ImGui::SliderAngle("Indirect rotation", &source.rotation, -360.0f, 360.0f) || changed;
      break;
  }
  return changed;
}

bool InspectEnvironmentalLightingSceneGizmoToggle(const std::shared_ptr<EditorLayer>& editor_layer,
                                                  const std::shared_ptr<EnvironmentalLighting>& lighting,
                                                  const EnvironmentalLightingGizmoTargetType type, const size_t index,
                                                  const uint64_t stable_id, const bool available) {
  bool active = lighting && editor_layer->IsEnvironmentalLightingGizmoTarget(*lighting, type, index, stable_id);
  if (active && !available) {
    editor_layer->ClearEnvironmentalLightingGizmoTarget();
    active = false;
  }
  ImGui::BeginDisabled(!available);
  if (ImGui::Checkbox("Edit in Scene", &active)) {
    if (active) {
      editor_layer->SetEnvironmentalLightingGizmoTarget(lighting, type, index, stable_id);
    } else {
      editor_layer->ClearEnvironmentalLightingGizmoTarget();
    }
  }
  ImGui::EndDisabled();
  if (!available && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::BeginTooltip();
    ImGui::TextUnformatted("Assign this Environmental Lighting asset to the active scene to edit it in the viewport.");
    ImGui::EndTooltip();
  }
  return active;
}

bool InspectEnvironmentalLighting(InspectorContext& context, EnvironmentalLighting& lighting) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }
  const auto active_scene = ApplicationContext::Get().GetActiveScene();
  const auto active_lighting = active_scene ? active_scene->environmental_lighting.Get<EnvironmentalLighting>()
                                            : std::shared_ptr<EnvironmentalLighting>{};
  const auto lighting_asset = active_lighting.get() == &lighting ? active_lighting : nullptr;
  const bool scene_gizmo_available = lighting_asset != nullptr;
  auto reflection_pack = lighting.GetReflectionProbePack();
  bool changed = false;
  if (reflection_pack && reflection_pack->RepairStableIds())
    reflection_pack->SetUnsaved();
  if (ImGui::BeginTabBar("EnvironmentalLightingInspectionTabs")) {
    if (ImGui::BeginTabItem("General")) {
      changed = InspectEnvironmentalLightingSource(context, lighting.indirect_environment_source) || changed;
      changed = ImGui::DragFloat("Environment lighting intensity", &lighting.environment_lighting_intensity, 0.01f,
                                 0.0f, 10000.0f) ||
                changed;
      changed =
          ImGui::DragFloat("Diffuse fallback intensity", &lighting.diffuse_fallback_intensity, 0.01f, 0.0f, 10000.0f) ||
          changed;
      changed = ImGui::DragFloat("Specular fallback intensity", &lighting.specular_fallback_intensity, 0.01f, 0.0f,
                                 10000.0f) ||
                changed;
      ImGui::EndTabItem();
    }

    if (ImGui::BeginTabItem("GI")) {
      auto candidate = lighting.GetGiSettings();
      auto& shared = candidate.gi_probe_settings;
      auto& settings = candidate.sdfgi_settings;
      bool gi_changed = false;
      static std::string gi_error;
      ImGui::SeparatorText("Probe settings");
      const auto probe_count = [&](const char* label, uint32_t& value, bool horizontal) {
        if (ImGui::BeginCombo(label, std::to_string(value).c_str())) {
          for (uint32_t count = 3; count <= 257; count += 2) {
            auto choice = candidate;
            (horizontal ? choice.gi_probe_settings.probe_count_x : choice.gi_probe_settings.probe_count_y) = count;
            if (!choice.Validate().empty())
              continue;
            if (ImGui::Selectable(std::to_string(count).c_str(), value == count)) {
              value = count;
              gi_changed = true;
            }
          }
          ImGui::EndCombo();
        }
        ImGui::SetItemTooltip(
            "Odd probe counts per cascade. Unavailable layouts are omitted. Changes recreate resources and restart "
            "lighting.");
      };
      probe_count("Probe count X/Z", shared.probe_count_x, true);
      probe_count("Probe count Y", shared.probe_count_y, false);
      int cascades = static_cast<int>(shared.cascade_count);
      if (ImGui::SliderInt("Cascades", &cascades, 1, 8)) {
        shared.cascade_count = static_cast<uint32_t>(cascades);
        gi_changed = true;
      }
      gi_changed =
          ImGui::DragFloat("Base probe distance", &shared.base_probe_distance, 0.01f, 0.001f, 512.0f) || gi_changed;
      ImGui::SetItemTooltip(
          "Horizontal distance between neighboring probes in cascade 0. Each successive cascade doubles it. "
          "Editing recreates resources and restarts convergence.");
      int vertical_scale = static_cast<int>(shared.vertical_scale);
      const char* vertical_scales[]{"50%", "75%", "100%"};
      if (ImGui::Combo("Y Scale", &vertical_scale, vertical_scales, IM_ARRAYSIZE(vertical_scales))) {
        shared.vertical_scale = static_cast<GiProbeSettings::VerticalScale>(vertical_scale);
        gi_changed = true;
      }
      ImGui::SetItemTooltip(
          "Compresses vertical cell/probe spacing and coverage without changing probe count. "
          "Godot modes: 100% = full spacing, 75% = spacing / 1.5, 50% = spacing / 2. "
          "Compact modes can improve detail and reduce leaks in low spaces. Recreates the field and restarts "
          "convergence.");
      PrivateComponentRef anchor;
      if (lighting_asset) {
        if (const auto entity = active_scene->GetEntity(Handle(shared.anchor_camera_entity));
            active_scene->IsEntityValid(entity) && active_scene->HasPrivateComponent<Camera>(entity))
          anchor = active_scene->GetOrSetPrivateComponent<Camera>(entity).lock();
      }
      ImGui::BeginDisabled(!lighting_asset);
      if (editor_layer->DragAndDropButton<Camera>(anchor, "Optional anchor camera")) {
        shared.anchor_camera_entity = anchor.GetEntityHandle().GetValue();
        gi_changed = true;
      }
      if (shared.anchor_camera_entity != 0 && ImGui::Button("Use automatic anchor")) {
        shared.anchor_camera_entity = 0;
        gi_changed = true;
      }
      ImGui::EndDisabled();

      ImGui::SeparatorText("Indirect GI provider");
      int provider = static_cast<int>(candidate.indirect_gi_provider);
      if (ImGui::Combo("Provider", &provider,
                       "Environment\0Automatic DDGI (RT)\0Automatic SDFGI\0Automatic HDDAGI\0")) {
        candidate.indirect_gi_provider = static_cast<IndirectGiProvider>(provider);
        gi_changed = true;
      }
      auto effective_provider = IndirectGiProvider::Environment;
      if (lighting_asset) {
        if (lighting.indirect_gi_provider == IndirectGiProvider::AutomaticSdfgi) {
          if (const auto runtime = active_scene->GetSdfgiRuntime()) {
            if (runtime->published)
              effective_provider = IndirectGiProvider::AutomaticSdfgi;
            else
              ImGui::TextWrapped("Fallback: %s", runtime->fallback_reason.c_str());
            if (runtime->anchor.override_fell_back)
              ImGui::TextDisabled("Explicit anchor unavailable; automatic selection used.");
          }
        } else if (lighting.indirect_gi_provider == IndirectGiProvider::AutomaticHddagi) {
          if (const auto runtime = active_scene->GetHddagiRuntime()) {
            if (const auto field = runtime->resources)
              ImGui::Text("Completed transport: %u; ready: %s; failure flags: %u", field->transport_generation,
                          field->transport_ready ? "yes" : "no", field->transport_failure_flags);
            ImGui::Text("Live allocation: %.1f MiB; retiring: %.1f MiB",
                        runtime->resources ? runtime->resources->AllocationBytes() / 1048576.0 : 0.0,
                        runtime->retiring_bytes / 1048576.0);
            if (runtime->published)
              effective_provider = IndirectGiProvider::AutomaticHddagi;
            else
              ImGui::TextWrapped("Fallback: %s", runtime->fallback_reason.c_str());
          } else {
            ImGui::TextWrapped("Fallback: HDDAGI runtime has not been prepared.");
          }
        } else if (lighting.indirect_gi_provider == IndirectGiProvider::AutomaticDdgi) {
          if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
            const auto snapshot = render_layer->GetDdgiInspectorSnapshot();
            if (snapshot.enabled && snapshot.aggregate.active_probe_count &&
                snapshot.aggregate.lighting_descriptors_bound)
              effective_provider = IndirectGiProvider::AutomaticDdgi;
          }
        }
      }
      ImGui::Text("Effective GI provider: %s", GetIndirectGiProviderName(effective_provider));

      if (candidate.indirect_gi_provider == IndirectGiProvider::AutomaticSdfgi) {
        ImGui::SeparatorText("Automatic SDFGI settings");
        if (ImGui::BeginCombo("Probe spacing", std::to_string(settings.probe_spacing_cells).c_str())) {
          for (const uint32_t spacing : {4u, 8u}) {
            auto choice = candidate;
            choice.sdfgi_settings.probe_spacing_cells = spacing;
            if (choice.Validate().empty() &&
                ImGui::Selectable(std::to_string(spacing).c_str(), settings.probe_spacing_cells == spacing)) {
              settings.probe_spacing_cells = spacing;
              gi_changed = true;
            }
          }
          ImGui::EndCombo();
        }
        ImGui::SetItemTooltip(
            "Voxels per probe interval. Changes voxel resolution, not nominal probe positions or coverage. Recreates "
            "the field.");
        const auto derived = DeriveSdfgiSettings(shared, settings);
        const auto grid = derived.GridSize();
        ImGui::TextDisabled("%dx%dx%d voxels; %ux%ux%u probes per cascade.", grid.x, grid.y, grid.z,
                            shared.probe_count_x, shared.probe_count_y, shared.probe_count_x);
        const auto inspect_choice = [&](const char* label, uint32_t& value,
                                        const std::initializer_list<uint32_t> choices) {
          if (ImGui::BeginCombo(label, std::to_string(value).c_str())) {
            for (const auto choice : choices) {
              if (ImGui::Selectable(std::to_string(choice).c_str(), value == choice)) {
                value = choice;
                gi_changed = true;
              }
            }
            ImGui::EndCombo();
          }
        };
        inspect_choice("Rays per probe", settings.ray_count, {4, 8, 16, 32, 64, 96, 128});
        inspect_choice("History frames", settings.history_size, {5, 10, 15, 20, 25, 30});
        inspect_choice("Dynamic-light update frames", settings.light_update_frames, {1, 2, 4, 8, 16});
        int contributors = settings.static_entities_only ? 1 : 0;
        if (ImGui::Combo("Geometry contributors", &contributors, "All supported entities\0Static entities only\0")) {
          settings.static_entities_only = contributors == 1;
          gi_changed = true;
        }
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip(
              "All includes non-static regular meshes; skinned, morphing and other unsupported geometry stay "
              "excluded.\n"
              "Moving geometry can trigger expensive cascade rebuilds. Changes need time to reconverge.\n"
              "Entity Static flags are not modified.");
        gi_changed = ImGui::Checkbox("Use Occlusion", &settings.use_occlusion) || gi_changed;
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip(
              "Weights GI probes by visibility to reduce light leaks; may produce dark patches.\n"
              "Changing this recreates the field and restarts convergence.");
        gi_changed = ImGui::Checkbox("Probe relocation", &settings.probe_relocation) || gi_changed;
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip(
              "Avoids nearby surfaces using the unsigned SDF; cannot reliably escape closed interiors. "
              "Uses bounded segment visibility from relocated probes. Changing this recreates the field "
              "and restarts convergence.");
        gi_changed = ImGui::Checkbox("Read sky light", &settings.read_sky_light) || gi_changed;
        gi_changed = ImGui::DragFloat("Bounce feedback", &settings.bounce_feedback, 0.01f, 0.0f, 1.99f) || gi_changed;
        gi_changed = ImGui::DragFloat("Energy", &settings.energy, 0.01f, 0.0f, 64.0f) || gi_changed;
        gi_changed = ImGui::DragFloat("Normal bias", &settings.normal_bias, 0.01f, 0.0f, 4.0f) || gi_changed;
        gi_changed = ImGui::DragFloat("Probe bias", &settings.probe_bias, 0.01f, 0.0f, 8.0f) || gi_changed;

        int light_cascades = static_cast<int>(settings.positional_light_cascade_count);
        if (ImGui::SliderInt("Positional light cascades", &light_cascades, 1, 8)) {
          settings.positional_light_cascade_count = static_cast<uint32_t>(light_cascades);
          gi_changed = true;
        }
        ImGui::SetItemTooltip(
            "Point/spot lights inject into the first N active cascades. 8 covers all cascades; 3 matches Godot's "
            "default. Directional lights are unaffected. Updates use normal light cadence and probe convergence; "
            "no geometry-field rebuild is needed.");
      } else if (candidate.indirect_gi_provider == IndirectGiProvider::AutomaticHddagi) {
        ImGui::SeparatorText("Automatic HDDAGI settings");
        auto& hddagi = candidate.hddagi_settings;
        if (ImGui::BeginCombo("History updates", std::to_string(hddagi.history_size).c_str())) {
          for (const uint32_t count : {6u, 12u, 18u, 24u, 32u})
            if (ImGui::Selectable(std::to_string(count).c_str(), count == hddagi.history_size)) {
              hddagi.history_size = count;
              gi_changed = true;
            }
          ImGui::EndCombo();
        }
        if (ImGui::BeginCombo("Light update frames", std::to_string(hddagi.light_update_frames).c_str())) {
          for (const uint32_t count : {1u, 2u, 4u, 8u, 16u})
            if (ImGui::Selectable(std::to_string(count).c_str(), hddagi.light_update_frames == count)) {
              hddagi.light_update_frames = count;
              gi_changed = true;
            }
          ImGui::EndCombo();
        }
        gi_changed |= ImGui::DragFloat("Bounce feedback", &hddagi.bounce_feedback, 0.01f, 0.0f, 16.0f);
        gi_changed |= ImGui::DragFloat("Energy", &hddagi.energy, 0.01f, 0.0f, 16.0f);
        gi_changed |= ImGui::DragFloat("Normal bias", &hddagi.normal_bias, 0.01f, 0.0f, 16.0f);
        gi_changed |= ImGui::DragFloat("Probe bias", &hddagi.probe_bias, 0.01f, 0.0f, 16.0f);
        gi_changed |= ImGui::DragFloat("Reflection bias", &hddagi.reflection_bias, 0.01f, 0.0f, 16.0f);
        gi_changed |= ImGui::DragFloat("Occlusion bias", &hddagi.occlusion_bias, 0.01f, 0.0f, 16.0f);
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip(
              "Minimum probe visibility weight. Lower values reject occluded probes more strongly, "
              "but can darken corners.");
        gi_changed |= ImGui::Checkbox("Filter probes", &hddagi.filter_probes);
        gi_changed |= ImGui::Checkbox("Filter ambient", &hddagi.filter_ambient);
        gi_changed |= ImGui::Checkbox("Filter reflections", &hddagi.filter_reflections);
        gi_changed |= ImGui::Checkbox("Static entities only", &hddagi.static_entities_only);
        gi_changed |= ImGui::Checkbox("Read sky light", &hddagi.read_sky_light);
        ImGui::TextWrapped(
            "Eight cells per probe. Diffuse GI and sharp reflections always use full resolution. Reflection "
            "captures use diffuse GI only.");
      } else if (candidate.indirect_gi_provider == IndirectGiProvider::AutomaticDdgi) {
        ImGui::SeparatorText("Automatic DDGI settings");
        auto& runtime = candidate.ddgi_settings.runtime;
        if (ImGui::BeginCombo("History count", std::to_string(runtime.history_count).c_str())) {
          for (int count = 5; count <= 30; count += 5)
            if (ImGui::Selectable(std::to_string(count).c_str(), runtime.history_count == count)) {
              runtime.history_count = count;
              gi_changed = true;
            }
          ImGui::EndCombo();
        }
        ImGui::SetItemTooltip(
            "Shared irradiance and visibility rolling window in completed updates. Zero-filled on reset; changes "
            "restart lighting.");
        if (ImGui::TreeNodeEx("Tracing and blending", ImGuiTreeNodeFlags_DefaultOpen)) {
          gi_changed = ImGui::Checkbox("Emissive mesh sampling", &runtime.enable_emissive_mesh_sampling) || gi_changed;
          gi_changed = ImGui::DragInt("Uniform ray count", &runtime.ray_count, 1.0f, 1, 4096) || gi_changed;
          gi_changed = ImGui::DragInt("Emissive ray count", &runtime.emissive_ray_count, 1.0f, 0, 4096) || gi_changed;
          gi_changed = ImGui::DragInt("Relocation warmup frames", &runtime.warmup_frames, 1.0f, 0, 4096) || gi_changed;
          gi_changed = ImGui::DragFloat("Normal bias", &runtime.normal_bias, 0.001f, 0.0f, 10.0f, "%.3f") || gi_changed;
          gi_changed = ImGui::DragFloat("View bias", &runtime.view_bias, 0.001f, 0.0f, 10.0f, "%.3f") || gi_changed;
          gi_changed =
              ImGui::DragFloat("Max ray distance", &runtime.max_ray_distance, 0.1f, 0.05f, 1e27f) || gi_changed;
          gi_changed =
              ImGui::DragFloat("Distance exponent", &runtime.distance_exponent, 0.1f, 0.0f, 256.0f) || gi_changed;
          gi_changed =
              ImGui::DragFloat("Irradiance gamma", &runtime.irradiance_gamma, 0.01f, 0.1f, 16.0f) || gi_changed;
          gi_changed =
              ImGui::DragFloat("Visibility moment bias", &runtime.visibility_moment_bias, 0.001f, 0.0f, 10.0f) ||
              gi_changed;
          gi_changed = ImGui::Checkbox("Deterministic ray seed", &runtime.deterministic_ray_seed_enabled) || gi_changed;
          if (runtime.deterministic_ray_seed_enabled)
            gi_changed =
                ImGui::InputScalar("Ray seed", ImGuiDataType_U32, &runtime.deterministic_ray_seed) || gi_changed;
          ImGui::TreePop();
        }

        gi_changed =
            ImGui::SliderFloat("Random ray backface threshold", &runtime.random_ray_backface_threshold, 0.0f, 1.0f) ||
            gi_changed;
        gi_changed =
            ImGui::SliderFloat("Fixed ray backface threshold", &runtime.fixed_ray_backface_threshold, 0.0f, 1.0f) ||
            gi_changed;
        gi_changed = ImGui::Checkbox("Probe relocation", &runtime.enable_probe_relocation) || gi_changed;
        gi_changed = ImGui::Checkbox("Probe classification", &runtime.enable_probe_classification) || gi_changed;
        gi_changed =
            ImGui::DragFloat("Relocation distance", &runtime.relocation_distance, 0.01f, 0.0f, 10000.0f) || gi_changed;
      }
      if (gi_changed && lighting.TrySetGiSettings(candidate, gi_error))
        changed = true;
      if (!gi_error.empty())
        ImGui::TextWrapped("Edit rejected; previous settings retained: %s", gi_error.c_str());
      ImGui::EndTabItem();
    }

    if (ImGui::BeginTabItem("Reflection Probes")) {
      if (editor_layer->DragAndDropButton<ReflectionProbePack>(lighting.reflection_probe_pack,
                                                               "Reflection Probe Pack")) {
        reflection_pack = lighting.GetReflectionProbePack();
        changed = true;
      }
      changed = ImGui::Checkbox("Enable local probe reflections", &lighting.local_reflection_probes_enabled) || changed;
      auto& dynamic_settings = lighting.dynamic_reflection_probe_settings;
      const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
      const bool explicit_bake_pending = render_layer && render_layer->HasPendingGlobalReflectionProbeBake();
      ImGui::BeginDisabled(!dynamic_settings.enabled && explicit_bake_pending);
      changed = ImGui::Checkbox("Enable dynamic local probe updates", &dynamic_settings.enabled) || changed;
      ImGui::EndDisabled();
      if (!dynamic_settings.enabled && explicit_bake_pending &&
          ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Wait for the explicit reflection-probe bake to finish.");
      }
      ImGui::BeginDisabled(!dynamic_settings.enabled);
      changed = ImGui::SliderInt("Faces per frame", &dynamic_settings.faces_per_frame, 1, 6) || changed;
      if (render_layer) {
        const auto stats = render_layer->GetDynamicReflectionProbeStats();
        ImGui::Text("Dynamic status: %s (continuous), queued %u, in progress %u", stats.active ? "active" : "idle",
                    stats.queued_probe_count, stats.in_progress_probe_count);
        if (stats.in_progress_probe_count != 0u) {
          ImGui::Text("Current probe: %llu (%u/6 faces)",
                      static_cast<unsigned long long>(stats.current_probe_stable_id), stats.completed_face_count);
        } else {
          ImGui::TextUnformatted("Current probe: none (0/6 faces)");
        }
        if (stats.filtering_probe_count != 0u) {
          ImGui::Text("Current GGX filter: %llu (%u/6 faces)",
                      static_cast<unsigned long long>(stats.current_filter_probe_stable_id),
                      stats.completed_filter_face_count);
        } else {
          ImGui::TextUnformatted("Current GGX filter: none (0/6 faces)");
        }
        ImGui::Text("Published generations: %llu, transient GPU: %.2f MiB",
                    static_cast<unsigned long long>(stats.published_generation_count),
                    static_cast<double>(stats.transient_gpu_bytes) / (1024.0 * 1024.0));
        ImGui::Text("Published A/B: %u/%u, transitioning: %u (%.3f-%.3f)", stats.generation_a_probe_count,
                    stats.generation_b_probe_count, stats.transitioning_probe_count, stats.minimum_transition_weight,
                    stats.maximum_transition_weight);
        ImGui::Text("Last GPU: total %.3f ms, capture %.3f ms, prefilter %.3f ms", stats.last_update_gpu_ms,
                    stats.last_capture_gpu_ms, stats.last_prefilter_gpu_ms);
        if (dynamic_settings.enabled && ImGui::Button("Reset Dynamic Probe History")) {
          render_layer->ResetDynamicReflectionProbeHistory();
        }
      }
      ImGui::EndDisabled();
      dynamic_settings.Clamp();
      auto& background = lighting.reflection_probe_bake_background;
      changed = InspectCameraBackground(editor_layer, background.source, nullptr, background.clear_color,
                                        background.cubemap, background.environmental_map) ||
                changed;
      const bool lighting_changed_before_probes = changed;
      changed = false;
      ImGui::BeginDisabled(!reflection_pack);
      if (ImGui::Button("Add Local Reflection Probe")) {
        reflection_pack->probes.emplace_back();
        (void)reflection_pack->RepairStableIds();
        changed = true;
      }
      ImGui::EndDisabled();
      ImGui::BeginDisabled(dynamic_settings.enabled);
      if (ImGui::Button("Bake All Local Probe Payloads")) {
        const auto queued_count = QueueEnvironmentalLightingLocalProbeBakes(context, lighting);
        EVOENGINE_LOG("Queued " + std::to_string(queued_count) + "/" +
                      std::to_string(reflection_pack ? reflection_pack->probes.size() : 0u) +
                      " environmental lighting reflection probe bakes.")
      }
      ImGui::EndDisabled();
      if (dynamic_settings.enabled && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Disable dynamic local probe updates before baking persistent payloads.");
      }
      for (size_t index = 0; reflection_pack && index < reflection_pack->probes.size(); ++index) {
        auto& probe = reflection_pack->probes[index];
        ImGui::PushID(static_cast<int>(index));
        const auto label = probe.name + "##EnvironmentalLightingLocalProbe";
        if (ImGui::TreeNode(label.c_str())) {
          changed = InspectEnvironmentalLightingEntryName(probe.name) || changed;
          changed = ImGui::Checkbox("Enabled", &probe.enabled) || changed;
          changed = ImGui::Checkbox("Debug draw bounds", &probe.debug_draw_bounds) || changed;
          const bool editing_in_scene = InspectEnvironmentalLightingSceneGizmoToggle(
              editor_layer, lighting_asset, EnvironmentalLightingGizmoTargetType::LocalReflectionProbe, index,
              probe.stable_id, scene_gizmo_available);
          const bool stable_id_changed = ImGui::InputScalar("Stable id", ImGuiDataType_U64, &probe.stable_id);
          changed = stable_id_changed || changed;
          if (stable_id_changed) {
            (void)reflection_pack->RepairStableIds();
            if (editing_in_scene) {
              editor_layer->SetEnvironmentalLightingGizmoTarget(
                  lighting_asset, EnvironmentalLightingGizmoTargetType::LocalReflectionProbe, index, probe.stable_id);
            }
          }
          if (ImGui::TreeNode("Payload status")) {
            InspectEnvironmentalLightingLocalProbePayload(context, reflection_pack, probe, !dynamic_settings.enabled);
            ImGui::TreePop();
          }
          changed = InspectAuthoringTransform(editor_layer, "Transform", probe.transform) || changed;
          const char* shapes[]{"Box", "Sphere"};
          changed = ImGui::Combo("Shape", &probe.shape, shapes, IM_ARRAYSIZE(shapes)) || changed;
          changed = ImGui::DragInt("Artist priority", &probe.artist_priority) || changed;
          changed = ImGui::DragFloat("Sphere radius", &probe.sphere_radius, 0.05f, 0.001f, 10000.0f) || changed;
          changed = ImGui::DragFloat("Blend distance", &probe.blend_distance, 0.01f, 0.0f, 10000.0f) || changed;
          changed =
              ImGui::DragFloat("Reflection intensity", &probe.reflection_intensity, 0.01f, 0.0f, 10000.0f) || changed;
          changed = ImGui::Checkbox("Box projection", &probe.box_projection) || changed;
          changed =
              ImGui::DragFloat3("Projection half extents", &probe.box_projection_extents.x, 0.05f, 0.001f, 10000.0f) ||
              changed;
          if (ImGui::Button("Remove")) {
            editor_layer->ClearEnvironmentalLightingGizmoTarget(lighting.GetHandle());
            reflection_pack->probes.erase(reflection_pack->probes.begin() + static_cast<std::ptrdiff_t>(index));
            changed = true;
            ImGui::TreePop();
            ImGui::PopID();
            break;
          }
          ImGui::TreePop();
        }
        ImGui::PopID();
      }
      if (changed && reflection_pack)
        reflection_pack->SetUnsaved();
      changed = lighting_changed_before_probes || changed;
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  RenderEnvironmentalLightingDebugProbeBounds(editor_layer, lighting, glm::vec4(0.1f, 0.8f, 1.0f, 0.55f));
  RenderActiveEnvironmentalLightingGizmoBound(editor_layer, lighting);

  return changed;
}

bool InspectEnvironmentalMap(InspectorContext& context, EnvironmentalMap& environmental_map) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  AssetRef inspection_target_texture;
  if (editor_layer->DragAndDropButton<Cubemap>(inspection_target_texture, "Convert from Skybox")) {
    if (const auto texture = inspection_target_texture.Get<Cubemap>()) {
      environmental_map.ConstructFromCubemap(texture);
      changed = true;
    }
  }

  if (ImGui::TreeNode("Sky illumination")) {
    static bool auto_rebuild = true;
    ImGui::Checkbox("Auto refresh", &auto_rebuild);
    static SkyIllumination sky_illumination{};
    const bool rebuild = InspectSkyIllumination(context, sky_illumination);
    if (ImGui::Button("Build") || (auto_rebuild && rebuild)) {
      environmental_map.BuildSkyIllumination(sky_illumination);
      changed = true;
    }
    ImGui::TreePop();
  }

  if (editor_layer->DragAndDropButton<LightProbe>(environmental_map.light_probe, "LightProbe"))
    changed = true;

  return changed;
}

bool InspectStrands(InspectorContext&, Strands& strands) {
  ImGui::Text(("Point size: " + std::to_string(strands.GetStrandPointAmount())).c_str());
  return false;
}

bool InspectPointCloud(InspectorContext&, PointCloud& point_cloud) {
  bool changed = false;
  ImGui::Text("Has Colors: %s", (!point_cloud.colors.empty() ? "True" : "False"));
  ImGui::Text("Has Positions: %s", (!point_cloud.positions.empty() ? "True" : "False"));
  ImGui::Text("Has Normals: %s", (!point_cloud.normals.empty() ? "True" : "False"));
  if (ImGui::DragScalarN("Offset", ImGuiDataType_Double, &point_cloud.offset.x, 3))
    changed = true;
  ImGui::Text(("Original amount: " + std::to_string(point_cloud.positions.size())).c_str());
  if (ImGui::DragFloat("Point size", &point_cloud.point_size, 0.01f, 0.01f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Compress factor", &point_cloud.compress_factor, 0.001f, 0.0001f, 10.0f))
    changed = true;

  if (ImGui::Button("Apply compressed")) {
    point_cloud.ApplyCompressed();
  }

  if (ImGui::Button("Apply original")) {
    point_cloud.ApplyOriginal();
  }

  FileUtils::OpenFile(
      "Load PLY file##Particles", "PointCloud", {".ply"},
      [&](const std::filesystem::path& file_path) {
        try {
          point_cloud.LoadPly({}, file_path);
          EVOENGINE_LOG("Loaded from " + file_path.string());
        } catch (std::exception&) {
          EVOENGINE_ERROR("Failed to load from " + file_path.string());
        }
      },
      false);
  FileUtils::SaveFile(
      "Save to PLY##Particles", "PointCloud", {".ply"},
      [&](const std::filesystem::path& file_path) {
        try {
          point_cloud.SavePly({}, file_path);
          EVOENGINE_LOG("Saved to " + file_path.string());
        } catch (std::exception&) {
          EVOENGINE_ERROR("Failed to save to " + file_path.string());
        }
      },
      false);
  if (ImGui::Button("Clear all positions")) {
    point_cloud.positions.clear();
    changed = true;
  }

  return changed;
}

bool InspectMaterialTextureSlot(const std::shared_ptr<EditorLayer>& editor_layer, Material& material,
                                uint16_t GltfShadeMaterial::* texture_slot, const char* label) {
  const auto current_texture = material.GetTexture(texture_slot);
  AssetRef texture_ref(current_texture);
  if (editor_layer->DragAndDropButton<Texture2D>(texture_ref, label)) {
    material.SetTexture(texture_slot, texture_ref.Get<Texture2D>());
    return true;
  }
  return false;
}

bool InspectGaussianSplat(InspectorContext&, GaussianSplat& gaussian_splat) {
  const auto min_bound = gaussian_splat.GetMinBound();
  const auto max_bound = gaussian_splat.GetMaxBound();
  ImGui::Text("Splats: %zu", gaussian_splat.GetSplatCount());
  ImGui::Text("Bounds min: %.3f, %.3f, %.3f", min_bound.x, min_bound.y, min_bound.z);
  ImGui::Text("Bounds max: %.3f, %.3f, %.3f", max_bound.x, max_bound.y, max_bound.z);
  ImGui::Text("SH rest floats per splat: %u", gaussian_splat.spherical_harmonics_rest_float_count);
  return false;
}

bool InspectMaterial(InspectorContext& context, Material& material) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  if (ImGui::Checkbox("Vertex color only", &material.vertex_color_only)) {
    changed = true;
  }

  ImGui::Separator();
  if (ImGui::TreeNodeEx("PBR##Material", ImGuiTreeNodeFlags_DefaultOpen)) {
    auto& shade_material = material.material_data.shade_material;
    if (ImGui::ColorEdit4("Base Color##Material", &shade_material.pbr_base_color_factor.x)) {
      changed = true;
    }
    if (ImGui::DragFloat("Metallic##Material", &shade_material.pbr_metallic_factor, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Roughness##Material", &shade_material.pbr_roughness_factor, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Specular##Material", &shade_material.specular_factor, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::ColorEdit3("Specular Color##Material", &shade_material.specular_color_factor.x,
                          ImGuiColorEditFlags_HDR | ImGuiColorEditFlags_Float)) {
      changed = true;
    }
    if (ImGui::ColorEdit3("Emissive##Material", &shade_material.emissive_factor.x,
                          ImGuiColorEditFlags_HDR | ImGuiColorEditFlags_Float)) {
      changed = true;
    }
    if (ImGui::DragFloat("Normal Scale##Material", &shade_material.normal_texture_scale, 0.01f, 0.0f, 10.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Occlusion Strength##Material", &shade_material.occlusion_strength, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    int alpha_mode = glm::clamp(shade_material.alpha_mode, 0, 2);
    constexpr const char* alpha_modes[] = {"Opaque", "Mask", "Blend"};
    if (ImGui::Combo("Alpha Mode##Material", &alpha_mode, alpha_modes, IM_ARRAYSIZE(alpha_modes))) {
      shade_material.alpha_mode = alpha_mode;
      changed = true;
    }
    if (ImGui::DragFloat("Alpha Cutoff##Material", &shade_material.alpha_cutoff, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    bool double_sided = shade_material.double_sided != 0;
    if (ImGui::Checkbox("Double Sided##Material", &double_sided)) {
      shade_material.double_sided = double_sided ? 1 : 0;
      changed = true;
    }
    if (ImGui::DragFloat("IOR##Material", &shade_material.ior, 0.01f, 0.0f, 5.0f)) {
      changed = true;
    }
    int nested_priority = static_cast<int>(std::min(shade_material.nested_priority, 15u));
    if (ImGui::SliderInt("Nested Priority##Material", &nested_priority, 0, 15)) {
      shade_material.nested_priority = static_cast<uint32_t>(nested_priority);
      changed = true;
    }
    if (ImGui::DragFloat("Transmission##Material", &shade_material.transmission_factor, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Clearcoat##Material", &shade_material.clearcoat_factor, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Clearcoat Roughness##Material", &shade_material.clearcoat_roughness, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Clearcoat Normal Scale##Material", &shade_material.clearcoat_normal_texture_scale, 0.01f,
                         -2.0f, 2.0f)) {
      changed = true;
    }
    if (ImGui::ColorEdit3("Sheen Color##Material", &shade_material.sheen_color_factor.x)) {
      changed = true;
    }
    if (ImGui::DragFloat("Sheen Roughness##Material", &shade_material.sheen_roughness_factor, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Iridescence##Material", &shade_material.iridescence_factor, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Iridescence IOR##Material", &shade_material.iridescence_ior, 0.01f, 1.0f, 5.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Iridescence Min Thickness (nm)##Material", &shade_material.iridescence_thickness_minimum,
                         1.0f, 0.0f, 10000.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Iridescence Max Thickness (nm)##Material", &shade_material.iridescence_thickness_maximum,
                         1.0f, 0.0f, 10000.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Anisotropy##Material", &shade_material.anisotropy_strength, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    float anisotropy_rotation = std::atan2(shade_material.anisotropy_rotation.y, shade_material.anisotropy_rotation.x);
    if (ImGui::DragFloat("Anisotropy Rotation (rad)##Material", &anisotropy_rotation, 0.01f)) {
      shade_material.anisotropy_rotation = glm::vec2(std::cos(anisotropy_rotation), std::sin(anisotropy_rotation));
      changed = true;
    }
    if (ImGui::DragFloat("Dispersion##Material", &shade_material.dispersion, 0.01f, 0.0f, 10.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Reference Retroreflection##Material", &shade_material.retroreflection_factor, 0.01f, 0.0f,
                         1.0f)) {
      changed = true;
    }

    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Others##Material")) {
    if (InspectDrawSettings(material.draw_settings, false)) {
      changed = true;
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Textures##Material", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::pbr_base_color_texture,
                                         "Base Color Tex") ||
              changed;
    changed =
        InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::normal_texture, "Normal Tex") || changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::pbr_metallic_roughness_texture,
                                         "Metallic Roughness Tex") ||
              changed;
    changed =
        InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::emissive_texture, "Emissive Tex") ||
        changed;
    changed =
        InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::occlusion_texture, "Occlusion Tex") ||
        changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::transmission_texture,
                                         "Transmission Tex") ||
              changed;
    changed =
        InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::clearcoat_texture, "Clearcoat Tex") ||
        changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::clearcoat_roughness_texture,
                                         "Clearcoat Roughness Tex") ||
              changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::clearcoat_normal_texture,
                                         "Clearcoat Normal Tex") ||
              changed;
    changed =
        InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::specular_texture, "Specular Tex") ||
        changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::specular_color_texture,
                                         "Specular Color Tex") ||
              changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::iridescence_texture,
                                         "Iridescence Tex") ||
              changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::iridescence_thickness_texture,
                                         "Iridescence Thickness Tex") ||
              changed;
    changed =
        InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::anisotropy_texture, "Anisotropy Tex") ||
        changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::retroreflection_texture,
                                         "Reference Retroreflection Tex") ||
              changed;

    AssetRef rma_texture_ref;
    if (editor_layer->DragAndDropButton<Texture2D>(rma_texture_ref, "Apply RMA Texture")) {
      if (const auto rma_texture = rma_texture_ref.Get<Texture2D>()) {
        std::vector<glm::vec3> rma_data;
        rma_texture->GetRgbChannelData(rma_data);
        const auto rma_resolution = rma_texture->GetResolution();
        std::vector<glm::vec3> temp_data(rma_data.size());
        Jobs::RunParallelFor(temp_data.size(), [&](const size_t pixel_index) {
          temp_data[pixel_index] = glm::vec3(1.0f, rma_data[pixel_index].x, rma_data[pixel_index].y);
        });
        const auto metallic_roughness_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
        metallic_roughness_texture->SetRgbChannelData(temp_data, rma_resolution);

        Jobs::RunParallelFor(temp_data.size(), [&](const size_t pixel_index) {
          temp_data[pixel_index] = glm::vec3(rma_data[pixel_index].z);
        });
        const auto ao_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
        ao_texture->SetRgbChannelData(temp_data, rma_resolution);

        material.SetTexture(&GltfShadeMaterial::pbr_metallic_roughness_texture, metallic_roughness_texture);
        material.SetTexture(&GltfShadeMaterial::occlusion_texture, ao_texture);
        changed = true;
      }
    }
    ImGui::TreePop();
  }
  if (changed) {
    material.MarkDirty();
  }
  return changed;
}

bool InspectTexture2D(InspectorContext& context, Texture2D& texture) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  ImGui::Text((std::string("Red Channel: ") + (texture.red_channel ? "True" : "False")).c_str());
  ImGui::Text((std::string("Green Channel: ") + (texture.green_channel ? "True" : "False")).c_str());
  ImGui::Text((std::string("Blue Channel: ") + (texture.blue_channel ? "True" : "False")).c_str());
  ImGui::Text((std::string("Alpha Channel: ") + (texture.alpha_channel ? "True" : "False")).c_str());

  const auto texture_storage = texture.PeekTexture2DStorage();
  AssetRef opacity_texture_drop_ref;
  if (editor_layer->DragAndDropButton<Texture2D>(opacity_texture_drop_ref, "Apply Opacity...")) {
    changed = true;
    if (const auto tex = opacity_texture_drop_ref.Get<Texture2D>()) {
      texture.ApplyOpacityMap(tex);
    }
  }
  if (texture_storage.im_texture_id && texture_storage.image) {
    static float debug_scale = 0.25f;
    ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 10.0f);
    debug_scale = glm::clamp(debug_scale, 0.1f, 10.0f);
    ImGui::Image(texture_storage.im_texture_id,
                 ImVec2(texture_storage.image->GetExtent().width * debug_scale,
                        texture_storage.image->GetExtent().height * debug_scale),
                 ImVec2(0, 1), ImVec2(1, 0));
  }
  return changed;
}

bool InspectWayPoints(InspectorContext& context, WayPoints& way_points) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  const auto scene = way_points.GetScene();
  bool changed = false;
  if (EntityRef temp_entity_holder;
      editor_layer->DragAndDropButton(temp_entity_holder, "Drop new SoilLayerDescriptor here...")) {
    if (auto entity = temp_entity_holder.Get(); scene->IsEntityValid(entity)) {
      way_points.entities.emplace_back(entity);
      changed = true;
    }
    temp_entity_holder.Clear();
  }
  for (int i = 0; i < way_points.entities.size(); i++) {
    auto entity = way_points.entities[i].Get();
    if (scene->IsEntityValid(entity)) {
      if (ImGui::TreeNodeEx(("No." + std::to_string(i + 1)).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text(("Name: " + scene->GetEntityName(entity)).c_str());

        if (ImGui::Button("Remove")) {
          way_points.entities.erase(way_points.entities.begin() + i);
          changed = true;
          ImGui::TreePop();
          continue;
        }
        if (i < way_points.entities.size() - 1) {
          ImGui::SameLine();
          if (ImGui::Button("Move down")) {
            changed = true;
            std::swap(way_points.entities[i + 1], way_points.entities[i]);
          }
        }
        if (i > 0) {
          ImGui::SameLine();
          if (ImGui::Button("Move up")) {
            changed = true;
            std::swap(way_points.entities[i - 1], way_points.entities[i]);
          }
        }
        ImGui::TreePop();
      }
    } else {
      way_points.entities.erase(way_points.entities.begin() + i);
      i--;
    }
  }
  return changed;
}

void RenderMeshRendererBound(const std::shared_ptr<EditorLayer>& editor_layer, MeshRenderer& renderer,
                             const glm::vec4& color) {
  const auto mesh = renderer.mesh.Get<Mesh>();
  if (!mesh) {
    return;
  }
  const auto scene = renderer.GetScene();
  const auto transform = scene->GetDataComponent<GlobalTransform>(renderer.GetOwner()).value;
  glm::vec3 size = mesh->GetBound().Size() * 2.0f;
  if (size.x < 0.001f)
    size.x = 0.001f;
  if (size.z < 0.001f)
    size.z = 0.001f;
  if (size.y < 0.001f)
    size.y = 0.001f;
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_LINE;
  gizmo_settings.draw_settings.line_width = 5.0f;
  editor_layer->DrawGizmoMesh(Resources::GetInstance().GetPrimitives().cube, color,
                              transform * (glm::translate(mesh->GetBound().Center()) * glm::scale(size)), 1,
                              gizmo_settings);
}

struct EmissiveMeshAuthoringEstimate {
  double world_area = 0.0;
  double emitted_power = 0.0;
  uint32_t positive_area_triangles = 0;
  bool eligible = false;
  const char* reason = "No positive emissive radiance";
};

template <typename VertexType>
EmissiveMeshAuthoringEstimate EstimateEmissiveMeshAuthoring(const std::vector<VertexType>& vertices,
                                                            const std::vector<glm::uvec3>& triangles,
                                                            const glm::mat4& model, const Material& material) {
  EmissiveMeshAuthoringEstimate estimate;
  const auto& shade_material = material.material_data.shade_material;
  const glm::vec3 radiance = glm::max(shade_material.emissive_factor, glm::vec3(0.0f));
  const double luminance = glm::dot(glm::dvec3(radiance), glm::dvec3(0.2126, 0.7152, 0.0722));
  if (!std::isfinite(luminance) || luminance <= 0.0) {
    return estimate;
  }
  if (triangles.empty()) {
    estimate.reason = "Mesh has no triangles";
    return estimate;
  }
  if (material.draw_settings.polygon_mode != VK_POLYGON_MODE_FILL) {
    estimate.reason = "DDGI sampling requires filled triangles";
    return estimate;
  }
  if (shade_material.unlit != 0) {
    estimate.reason = "Unlit materials are excluded from DDGI sampling";
    return estimate;
  }
  const float determinant = glm::determinant(glm::mat3(model));
  if (!std::isfinite(determinant) || determinant == 0.0f) {
    estimate.reason = "World transform is singular";
    return estimate;
  }
  for (const auto& triangle : triangles) {
    if (triangle.x >= vertices.size() || triangle.y >= vertices.size() || triangle.z >= vertices.size()) {
      continue;
    }
    const glm::dvec3 p0 = glm::dvec3(model * glm::vec4(vertices[triangle.x].position, 1.0f));
    const glm::dvec3 p1 = glm::dvec3(model * glm::vec4(vertices[triangle.y].position, 1.0f));
    const glm::dvec3 p2 = glm::dvec3(model * glm::vec4(vertices[triangle.z].position, 1.0f));
    const double area = 0.5 * glm::length(glm::cross(p1 - p0, p2 - p0));
    if (std::isfinite(area) && area > 0.0) {
      estimate.world_area += area;
      ++estimate.positive_area_triangles;
    }
  }
  if (estimate.positive_area_triangles == 0) {
    estimate.reason = "Mesh has no positive-area world-space triangles";
    return estimate;
  }
  estimate.emitted_power =
      glm::pi<double>() * estimate.world_area * luminance * (shade_material.double_sided != 0 ? 2.0 : 1.0);
  estimate.eligible = Platform::RayAccelerationStructureEnabled();
  estimate.reason = estimate.eligible ? "Eligible; runtime inventory confirms uploaded geometry"
                                      : "Ray tracing is unavailable on the current device";
  return estimate;
}

template <typename VertexType>
bool InspectEmissiveMeshAuthoring(const std::shared_ptr<EditorLayer>& editor_layer, Material& material,
                                  const std::vector<VertexType>& vertices, const std::vector<glm::uvec3>& triangles,
                                  const glm::mat4& model, const bool bind_pose_area, const void* id) {
  bool changed = false;
  ImGui::PushID(id);
  if (ImGui::TreeNodeEx("DDGI emissive authoring", ImGuiTreeNodeFlags_DefaultOpen)) {
    auto& shade_material = material.material_data.shade_material;
    changed = ImGui::ColorEdit3("Radiance", &shade_material.emissive_factor.x,
                                ImGuiColorEditFlags_HDR | ImGuiColorEditFlags_Float) ||
              changed;
    bool double_sided = shade_material.double_sided != 0;
    if (ImGui::Checkbox("Double sided", &double_sided)) {
      shade_material.double_sided = double_sided ? 1 : 0;
      changed = true;
    }
    int alpha_mode = glm::clamp(shade_material.alpha_mode, 0, 2);
    constexpr const char* alpha_modes[] = {"Opaque", "Mask", "Blend"};
    if (ImGui::Combo("Alpha mode", &alpha_mode, alpha_modes, IM_ARRAYSIZE(alpha_modes))) {
      shade_material.alpha_mode = alpha_mode;
      changed = true;
    }
    changed =
        InspectMaterialTextureSlot(editor_layer, material, &GltfShadeMaterial::emissive_texture, "Emissive texture") ||
        changed;

    const auto estimate = EstimateEmissiveMeshAuthoring(vertices, triangles, model, material);
    const glm::vec3 radiance = glm::max(shade_material.emissive_factor, glm::vec3(0.0f));
    const double luminance = glm::dot(glm::dvec3(radiance), glm::dvec3(0.2126, 0.7152, 0.0722));
    ImGui::SeparatorText("Power preview");
    ImGui::Text("Radiance RGB: %.6g, %.6g, %.6g", radiance.x, radiance.y, radiance.z);
    ImGui::Text("Radiance luminance: %.6g", luminance);
    ImGui::Text("Estimated world area: %.6g%s", estimate.world_area, bind_pose_area ? " (bind pose)" : "");
    ImGui::Text("Estimated emitted power: %.6g", estimate.emitted_power);
    ImGui::Text("DDGI triangle candidates: %u / %zu", estimate.positive_area_triangles, triangles.size());
    ImGui::Text("DDGI sampling: %s", estimate.reason);
    if (material.GetTexture(&GltfShadeMaterial::emissive_texture)) {
      ImGui::TextDisabled("Factor-only preview; the runtime inventory includes emissive texture energy.");
    }
    if (shade_material.alpha_mode == static_cast<int32_t>(GltfAlphaMode::Mask)) {
      ImGui::TextDisabled("Alpha masking adds opacity sampling and can reject emitter samples.");
    }
    if (shade_material.double_sided != 0) {
      ImGui::TextDisabled("Double-sided emission doubles factor-only power and emits from both faces.");
    }
    if (luminance >= 1.0 && estimate.emitted_power < 0.01) {
      ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f),
                         "Bright radiance but negligible total power; increase emitting area or radiance.");
    }

    static std::unordered_map<const void*, double> target_power_by_renderer;
    double& target_power = target_power_by_renderer[id];
    if (target_power <= 0.0 && estimate.emitted_power > 0.0) {
      target_power = estimate.emitted_power;
    }
    ImGui::InputDouble("Target emitted power", &target_power, 0.0, 0.0, "%.6g");
    const bool can_apply_target = target_power > 0.0 && estimate.emitted_power > 0.0 && std::isfinite(target_power);
    if (!can_apply_target) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Button("Apply target power to radiance")) {
      shade_material.emissive_factor *= static_cast<float>(target_power / estimate.emitted_power);
      changed = true;
    }
    if (!can_apply_target) {
      ImGui::EndDisabled();
    }
    ImGui::TextDisabled("Explicit action scales emissive radiance; it never creates or changes an analytic light.");
    ImGui::TextDisabled("The Render Layer DDGI inspector is authoritative for runtime inventory power and exclusions.");
    ImGui::TreePop();
  }
  ImGui::PopID();
  if (changed) {
    material.MarkDirty();
  }
  return changed;
}

bool InspectMeshRenderer(InspectorContext& context, MeshRenderer& renderer) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  if (ImGui::Checkbox("Cast shadow##MeshRenderer", &renderer.cast_shadow))
    changed = true;
  if (editor_layer->DragAndDropButton<Material>(renderer.material, "Material"))
    changed = true;
  if (editor_layer->DragAndDropButton<Mesh>(renderer.mesh, "Mesh"))
    changed = true;
  if (const auto mesh = renderer.mesh.Get<Mesh>()) {
    if (ImGui::TreeNodeEx("Mesh##MeshRenderer", ImGuiTreeNodeFlags_DefaultOpen)) {
      static bool display_bound = false;
      ImGui::Checkbox("Display bounds##MeshRenderer", &display_bound);
      if (display_bound) {
        static auto display_bound_color = glm::vec4(0.0f, 1.0f, 0.0f, 0.1f);
        ImGui::ColorEdit4("Color:##MeshRenderer", static_cast<float*>(static_cast<void*>(&display_bound_color)));
        RenderMeshRendererBound(editor_layer, renderer, display_bound_color);
      }
      ImGui::TreePop();
    }
    if (const auto material = renderer.material.Get<Material>()) {
      const auto model = renderer.GetScene()->GetDataComponent<GlobalTransform>(renderer.GetOwner()).value;
      changed = InspectEmissiveMeshAuthoring(editor_layer, *material, mesh->PeekVertices(), mesh->PeekTriangles(),
                                             model, false, &renderer) ||
                changed;
    }
  }
  return changed;
}

bool InspectParticles(InspectorContext& context, Particles& particles) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  if (ImGui::Checkbox("Cast shadow##Particles", &particles.cast_shadow))
    changed = true;
  if (editor_layer->DragAndDropButton<Material>(particles.material, "Material"))
    changed = true;
  if (editor_layer->DragAndDropButton<Mesh>(particles.mesh, "Mesh"))
    changed = true;
  if (editor_layer->DragAndDropButton<ParticleInfoList>(particles.particle_info_list, "ParticleInfoList"))
    changed = true;

  if (const auto pil = particles.particle_info_list.Get<ParticleInfoList>()) {
    ImGui::Text(("Instance count##Particles" + std::to_string(pil->PeekParticleInfoList().size())).c_str());
    if (ImGui::Button("Calculate bounds##Particles")) {
      particles.RecalculateBoundingBox();
    }
    static bool display_bound;
    ImGui::Checkbox("Display bounds##Particles", &display_bound);
    if (display_bound) {
      static auto display_bound_color = glm::vec4(0.0f, 1.0f, 0.0f, 0.2f);
      ImGui::ColorEdit4("Color:##Particles", static_cast<float*>(static_cast<void*>(&display_bound_color)));
      const auto transform = particles.GetScene()->GetDataComponent<GlobalTransform>(particles.GetOwner()).value;

      GizmoSettings gizmo_settings;
      gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
      gizmo_settings.draw_settings.blending = true;
      gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_LINE;
      gizmo_settings.draw_settings.line_width = 3.0f;

      editor_layer->DrawGizmoCube(
          display_bound_color,
          transform * glm::translate(particles.bounding_box.Center()) * glm::scale(particles.bounding_box.Size()), 1,
          gizmo_settings);
    }
  }
  return changed;
}

void RenderStrandsRendererBound(const std::shared_ptr<EditorLayer>& editor_layer, StrandsRenderer& renderer,
                                glm::vec4& color) {
  const auto strands = renderer.strands.Get<Strands>();
  if (!strands) {
    return;
  }
  const auto transform = renderer.GetScene()->GetDataComponent<GlobalTransform>(renderer.GetOwner()).value;
  const auto bound = strands->GetBound();
  glm::vec3 size = bound.Size();
  if (size.x < 0.01f)
    size.x = 0.01f;
  if (size.z < 0.01f)
    size.z = 0.01f;
  if (size.y < 0.01f)
    size.y = 0.01f;
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_LINE;
  gizmo_settings.draw_settings.line_width = 3.0f;
  editor_layer->DrawGizmoMesh(Resources::GetInstance().GetPrimitives().cube, color,
                              transform * (glm::translate(bound.Center()) * glm::scale(size)), 1, gizmo_settings);
}

bool InspectStrandsRenderer(InspectorContext& context, StrandsRenderer& renderer) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  if (ImGui::Checkbox("Cast shadow##StrandsRenderer:", &renderer.cast_shadow))
    changed = true;
  if (editor_layer->DragAndDropButton<Material>(renderer.material, "Material"))
    changed = true;
  if (editor_layer->DragAndDropButton<Strands>(renderer.strands, "Strands"))
    changed = true;
  if (renderer.strands.Get<Strands>()) {
    if (ImGui::TreeNode("Strands##StrandsRenderer")) {
      static bool display_bound = true;
      ImGui::Checkbox("Display bounds##StrandsRenderer", &display_bound);
      if (display_bound) {
        static auto display_bound_color = glm::vec4(0.0f, 1.0f, 0.0f, 0.2f);
        ImGui::ColorEdit4("Color:##StrandsRenderer", static_cast<float*>(static_cast<void*>(&display_bound_color)));
        RenderStrandsRendererBound(editor_layer, renderer, display_bound_color);
      }
      ImGui::TreePop();
    }
  }
  return changed;
}

void RenderGaussianSplatRendererBound(const std::shared_ptr<EditorLayer>& editor_layer, GaussianSplatRenderer& renderer,
                                      glm::vec4& color) {
  const auto gaussian_splat = renderer.gaussian_splat.Get<GaussianSplat>();
  if (!gaussian_splat) {
    return;
  }
  const auto transform = renderer.GetScene()->GetDataComponent<GlobalTransform>(renderer.GetOwner()).value;
  Bound bound;
  bound.min = gaussian_splat->GetMinBound();
  bound.max = gaussian_splat->GetMaxBound();
  glm::vec3 size = bound.Size();
  if (size.x < 0.01f)
    size.x = 0.01f;
  if (size.z < 0.01f)
    size.z = 0.01f;
  if (size.y < 0.01f)
    size.y = 0.01f;
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_LINE;
  gizmo_settings.draw_settings.line_width = 3.0f;
  editor_layer->DrawGizmoMesh(Resources::GetInstance().GetPrimitives().cube, color,
                              transform * (glm::translate(bound.Center()) * glm::scale(size)), 1, gizmo_settings);
}

bool InspectGaussianSplatRenderer(InspectorContext& context, GaussianSplatRenderer& renderer) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  if (editor_layer->DragAndDropButton<GaussianSplat>(renderer.gaussian_splat, "Gaussian Splat"))
    changed = true;
  if (ImGui::DragFloat("Opacity scale##GaussianSplatRenderer", &renderer.opacity_scale, 0.01f, 0.0f, 10.0f))
    changed = true;
  if (ImGui::SliderInt("SH degree##GaussianSplatRenderer", &renderer.sh_degree, 0, 3))
    changed = true;

  int sort_mode = static_cast<int>(renderer.sort_mode);
  const char* sort_modes[] = {"None", "CPU depth", "GPU radix"};
  if (ImGui::Combo("Sort mode##GaussianSplatRenderer", &sort_mode, sort_modes, IM_ARRAYSIZE(sort_modes))) {
    renderer.sort_mode = static_cast<GaussianSplatSortMode>(sort_mode);
    changed = true;
  }

  int depth_mode = static_cast<int>(renderer.depth_mode);
  const char* depth_modes[] = {"Always", "Scene depth"};
  if (ImGui::Combo("Depth mode##GaussianSplatRenderer", &depth_mode, depth_modes, IM_ARRAYSIZE(depth_modes))) {
    renderer.depth_mode = static_cast<GaussianSplatDepthMode>(depth_mode);
    changed = true;
  }

  int raster_mode = static_cast<int>(renderer.raster_mode);
  const char* raster_modes[] = {"Auto", "Vertex", "Mesh shader"};
  if (ImGui::Combo("Raster mode##GaussianSplatRenderer", &raster_mode, raster_modes, IM_ARRAYSIZE(raster_modes))) {
    renderer.raster_mode = static_cast<GaussianSplatRasterMode>(raster_mode);
    changed = true;
  }

  if (const auto gaussian_splat = renderer.gaussian_splat.Get<GaussianSplat>()) {
    ImGui::Text("Splats: %zu", gaussian_splat->GetSplatCount());
    if (ImGui::TreeNode("Gaussian Splat##GaussianSplatRenderer")) {
      static bool display_bound = true;
      ImGui::Checkbox("Display bounds##GaussianSplatRenderer", &display_bound);
      if (display_bound) {
        static auto display_bound_color = glm::vec4(0.0f, 0.7f, 1.0f, 0.2f);
        ImGui::ColorEdit4("Color:##GaussianSplatRenderer",
                          static_cast<float*>(static_cast<void*>(&display_bound_color)));
        RenderGaussianSplatRendererBound(editor_layer, renderer, display_bound_color);
      }
      ImGui::TreePop();
    }
  }
  return changed;
}

void RenderSkinnedMeshRendererBound(const std::shared_ptr<EditorLayer>& editor_layer, SkinnedMeshRenderer& renderer,
                                    glm::vec4& color) {
  const auto skinned_mesh = renderer.skinned_mesh.Get<SkinnedMesh>();
  if (!skinned_mesh) {
    return;
  }
  const auto scene = renderer.GetScene();
  const auto transform = scene->GetDataComponent<GlobalTransform>(renderer.GetOwner()).value;
  const auto bound = skinned_mesh->GetBound();
  glm::vec3 size = bound.Size() * 2.0f;
  if (size.x < 0.01f)
    size.x = 0.01f;
  if (size.z < 0.01f)
    size.z = 0.01f;
  if (size.y < 0.01f)
    size.y = 0.01f;
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.polygon_mode = VK_POLYGON_MODE_LINE;
  gizmo_settings.draw_settings.line_width = 5.0f;
  editor_layer->DrawGizmoMesh(Resources::GetInstance().GetPrimitives().cube, color,
                              transform * (glm::translate(bound.Center()) * glm::scale(size)), 1, gizmo_settings);
}

void DrawSkinnedMeshRendererBones(const std::shared_ptr<EditorLayer>& editor_layer, SkinnedMeshRenderer& renderer,
                                  const std::shared_ptr<Animator>& animator, const glm::vec4& color, const float size) {
  static std::shared_ptr<ParticleInfoList> debug_bone_particle_info_list;
  const auto scene = renderer.GetScene();
  const auto owner = renderer.GetOwner();
  const auto self_scale = scene->GetDataComponent<GlobalTransform>(owner).GetScale();
  const auto& offset_matrices = animator->PeekOffsetMatrices();
  const auto& matrices = renderer.RagDoll() ? renderer.PeekRagDollTransformChain() : animator->PeekTransformChain();
  std::vector<ParticleInfo> debug_rendering_matrices;
  GlobalTransform ltw;

  if (!debug_bone_particle_info_list) {
    debug_bone_particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  }
  debug_rendering_matrices.resize(matrices.size());
  Jobs::RunParallelFor(matrices.size(), [&](size_t i) {
    debug_rendering_matrices.at(i).instance_matrix.value = matrices.at(i);
    debug_rendering_matrices.at(i).instance_color = color;
  });
  if (!renderer.RagDoll()) {
    ltw = scene->GetDataComponent<GlobalTransform>(owner);
  }
  for (int index = 0; index < debug_rendering_matrices.size() && index < offset_matrices.size(); index++) {
    debug_rendering_matrices[index].instance_matrix.value = debug_rendering_matrices[index].instance_matrix.value *
                                                            glm::inverse(offset_matrices[index]) *
                                                            glm::inverse(glm::scale(self_scale));
  }
  debug_bone_particle_info_list->SetParticleInfos(debug_rendering_matrices);
  editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().sphere,
                                              debug_bone_particle_info_list, ltw.value, size);
}

bool InspectSkinnedMeshRenderer(InspectorContext& context, SkinnedMeshRenderer& renderer) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  bool changed = false;
  if (editor_layer->DragAndDropButton<Animator>(renderer.animator, "Animator"))
    changed = true;
  if (ImGui::Checkbox("Cast shadow##SkinnedMeshRenderer", &renderer.cast_shadow))
    changed = true;
  if (editor_layer->DragAndDropButton<Material>(renderer.material, "Material"))
    changed = true;
  if (editor_layer->DragAndDropButton<SkinnedMesh>(renderer.skinned_mesh, "Skinned Mesh"))
    changed = true;
  if (const auto skinned_mesh = renderer.skinned_mesh.Get<SkinnedMesh>()) {
    if (ImGui::TreeNode("Skinned Mesh:##SkinnedMeshRenderer")) {
      static bool display_bound = true;
      ImGui::Checkbox("Display bounds##SkinnedMeshRenderer", &display_bound);
      if (display_bound) {
        static auto display_bound_color = glm::vec4(0.0f, 1.0f, 0.0f, 0.2f);
        ImGui::ColorEdit4("Color:##SkinnedMeshRenderer", static_cast<float*>(static_cast<void*>(&display_bound_color)));
        RenderSkinnedMeshRendererBound(editor_layer, renderer, display_bound_color);
      }
      ImGui::TreePop();
    }
    if (const auto material = renderer.material.Get<Material>()) {
      const auto model = renderer.GetScene()->GetDataComponent<GlobalTransform>(renderer.GetOwner()).value;
      changed = InspectEmissiveMeshAuthoring(editor_layer, *material, skinned_mesh->PeekSkinnedVertices(),
                                             skinned_mesh->PeekTriangles(), model, true, &renderer) ||
                changed;
    }
  }
  if (const auto animator = renderer.animator.Get<Animator>()) {
    static bool debug_render_bones = true;
    static float debug_render_bones_size = 0.01f;
    static glm::vec4 debug_render_bones_color = glm::vec4(1, 0, 0, 0.5);
    ImGui::Checkbox("Display bones", &debug_render_bones);
    if (debug_render_bones) {
      ImGui::DragFloat("Size", &debug_render_bones_size, 0.01f, 0.01f, 3.0f);
      ImGui::ColorEdit4("Color", &debug_render_bones_color.x);
      DrawSkinnedMeshRendererBones(editor_layer, renderer, animator, debug_render_bones_color, debug_render_bones_size);
    }

    bool rag_doll = renderer.RagDoll();
    if (ImGui::Checkbox("RagDoll", &rag_doll)) {
      renderer.SetRagDoll(rag_doll);
      changed = true;
    }
    if (renderer.RagDoll()) {
      ImGui::Checkbox("Freeze", &renderer.rag_doll_freeze);

      if (ImGui::TreeNode("RagDoll")) {
        const auto& bone_names = animator->PeekBoneNames();
        for (int i = 0; i < renderer.GetRagDollBoneSize(); i++) {
          EntityRef bound_entity(renderer.GetRagDollBoundEntity(i));
          const auto label =
              "Bone: " + (i < bone_names.size() ? bone_names[i] : std::string("No.") + std::to_string(i + 1));
          if (editor_layer->DragAndDropButton(bound_entity, label)) {
            if (const auto entity = bound_entity.Get(); renderer.GetScene()->IsEntityValid(entity)) {
              renderer.SetRagDollBoundEntity(i, entity);
            } else {
              renderer.ClearRagDollBoundEntity(i);
            }
            changed = true;
          }
        }
        ImGui::TreePop();
      }
    }
  }
  return changed;
}

bool PrivateComponentRefExistsInLod(const Lod& lod, const Handle handle) {
  for (const auto& renderer : lod.renderers) {
    if (renderer.GetHandle() == handle) {
      return true;
    }
  }
  return false;
}

template <typename T>
void AddLodRendererIfPresent(const std::shared_ptr<Scene>& scene, const Entity& entity, Lod& lod) {
  if (!scene->HasPrivateComponent<T>(entity)) {
    return;
  }
  const auto component = scene->GetOrSetPrivateComponent<T>(entity).lock();
  if (!component || PrivateComponentRefExistsInLod(lod, component->GetHandle())) {
    return;
  }
  lod.renderers.emplace_back();
  lod.renderers.back().Set(component);
}

bool InspectLod(Lod& lod) {
  bool changed = false;
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.f, 0.3f, 0, 1));
  ImGui::Button("Drop to Add...");
  ImGui::PopStyleColor(1);
  EntityRef temp{};
  if (EditorLayer::Droppable(temp)) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto entity = temp.Get();
    AddLodRendererIfPresent<MeshRenderer>(scene, entity, lod);
    AddLodRendererIfPresent<SkinnedMeshRenderer>(scene, entity, lod);
    AddLodRendererIfPresent<Particles>(scene, entity, lod);
    AddLodRendererIfPresent<StrandsRenderer>(scene, entity, lod);
  }
  for (auto it = lod.renderers.begin(); it != lod.renderers.end(); ++it) {
    if (const auto ptr = it->Get<IPrivateComponent>()) {
      const auto scene = ApplicationContext::Get().GetActiveScene();
      if (!scene->IsEntityValid(ptr->GetOwner())) {
        it->Clear();
        ImGui::Button("none");
        return true;
      }
      ImGui::Button((scene->GetEntityName(ptr->GetOwner()) + "(" + ptr->GetTypeName() + ")").c_str());
      EditorLayer::Draggable(*it);
      if (EditorLayer::Remove(*it)) {
        if (!it->Get<IPrivateComponent>()) {
          lod.renderers.erase(it);
          break;
        }
      }
    }
  }
  return changed;
}

bool InspectLodGroup(InspectorContext&, LodGroup& lod_group) {
  bool changed = false;
  if (ImGui::Checkbox("Override LOD Factor", &lod_group.override_lod_factor))
    changed = true;
  if (lod_group.override_lod_factor) {
    if (ImGui::SliderFloat("Current LOD Factor", &lod_group.lod_factor, 0.f, 1.f))
      changed = true;
  } else {
    ImGui::Text((std::string("LOD Factor: ") + std::to_string(lod_group.lod_factor)).c_str());
  }
  if (ImGui::TreeNodeEx("LODs", ImGuiTreeNodeFlags_DefaultOpen)) {
    int lod_index = 0;
    for (auto it = lod_group.lods.begin(); it != lod_group.lods.end(); ++it) {
      if (ImGui::TreeNode((std::string("LOD ") + std::to_string(lod_index)).c_str())) {
        float min = 0.f;
        float max = 1.f;
        if (it != lod_group.lods.begin()) {
          min = (it - 1)->lod_offset;
        }
        if (it + 1 != lod_group.lods.end()) {
          max = (it + 1)->lod_offset;
        }
        if (ImGui::SliderFloat("LOD Offset", &it->lod_offset, min, max))
          changed = true;
        if (InspectLod(*it))
          changed = true;
        ImGui::TreePop();
      }
      lod_index++;
    }
    if (!lod_group.lods.empty() && lod_group.lods.back().lod_offset != 1.f) {
      ImGui::Text((std::string("Culled: [") + std::to_string(lod_group.lods.back().lod_offset) + " -> 1.0]").c_str());
    }
    if (ImGui::Button("Push LOD")) {
      lod_group.lods.emplace_back();
      lod_group.lods.back().lod_offset = 1.f;
      lod_group.lods.back().index = lod_group.lods.size() - 1;
      if (lod_group.lods.size() > 1) {
        float prev_val = 0.f;
        if (lod_group.lods.size() > 2) {
          prev_val = lod_group.lods.at(lod_group.lods.size() - 3).lod_offset;
        }
        lod_group.lods.at(lod_group.lods.size() - 2).lod_offset = (1.f + prev_val) * .5f;
      }
      changed = true;
    }
    if (!lod_group.lods.empty()) {
      ImGui::SameLine();
      if (ImGui::Button("Pop LOD")) {
        const float last_offset = lod_group.lods.back().lod_offset;
        lod_group.lods.pop_back();
        if (!lod_group.lods.empty())
          lod_group.lods.back().lod_offset = last_offset;
        changed = true;
      }
    }
    ImGui::TreePop();
  }
  return changed;
}
}  // namespace

void evo_engine::DrawCameraDebugViews(const Camera& camera, const float debug_scale) {
  const auto size = camera.GetSize();
  const ImVec2 image_size(size.x * debug_scale, size.y * debug_scale);
  if (ImGui::TreeNodeEx("Base color / AO", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Image(camera.GetGBufferBaseColorAoImTextureId(), image_size, ImVec2(0, 1), ImVec2(1, 0));
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Normal / Roughness", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Image(camera.GetGBufferNormalRoughnessImTextureId(), image_size, ImVec2(0, 1), ImVec2(1, 0));
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("PBR / Flags")) {
    ImGui::Image(camera.GetGBufferPbrFlagsImTextureId(), image_size, ImVec2(0, 1), ImVec2(1, 0));
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Emissive")) {
    ImGui::Image(camera.GetGBufferEmissiveImTextureId(), image_size, ImVec2(0, 1), ImVec2(1, 0));
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Metadata")) {
    ImGui::TextWrapped(
        "Integer instance, material, info, flags, and packed vertex-color data. Use the scene debug "
        "views or viewport picking to inspect decoded values.");
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Depth")) {
    ImGui::Image(camera.GetRenderTexture()->GetDepthImTextureId(), image_size, ImVec2(0, 1), ImVec2(1, 0));
    ImGui::TreePop();
  }
}

bool evo_engine::DrawProceduralNoiseGraph(procedural_noise::IProceduralNoise& noise, const std::string& window_title,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  return InspectProceduralNoiseGraph(noise, window_title, editor_layer);
}

bool evo_engine::DrawSettingsGui(DrawSettings& draw_settings) {
  return InspectDrawSettings(draw_settings);
}

void evo_engine::RegisterWayPointsHandlers() {
  InspectorRegistry::GetInstance().RegisterInspector<WayPoints>(InspectWayPoints, {}, "WayPoints");
}

void evo_engine::RegisterSdkInspectionAdapters() {
  InspectorRegistry::GetInstance().RegisterInspector<Animation>(InspectAnimation, {}, "Animation");
  InspectorRegistry::GetInstance().RegisterInspector<AnimationPlayer>(InspectAnimationPlayer, {}, "AnimationPlayer");
  InspectorRegistry::GetInstance().RegisterInspector<Animator>(InspectAnimator, {}, "Animator");
  InspectorRegistry::GetInstance().RegisterInspector<Camera>(InspectCamera, {}, "Camera");
  InspectorRegistry::GetInstance().RegisterInspector<Cubemap>(InspectCubemap, {}, "Cubemap");
  InspectorRegistry::GetInstance().RegisterInspector<DirectionalLight>(InspectDirectionalLight, {}, "DirectionalLight");
  InspectorRegistry::GetInstance().RegisterInspector<EnvironmentalLighting>(InspectEnvironmentalLighting, {},
                                                                            "EnvironmentalLighting");
  InspectorRegistry::GetInstance().RegisterInspector<ReflectionProbePack>(InspectReflectionProbePack, {},
                                                                          "ReflectionProbePack");
  InspectorRegistry::GetInstance().RegisterInspector<EnvironmentalMap>(InspectEnvironmentalMap, {}, "EnvironmentalMap");
  InspectorRegistry::GetInstance().RegisterInspector<EditorLayer>(InspectEditorLayer, {}, "EditorLayer");
  InspectorRegistry::GetInstance().RegisterInspector<LightProbe>(InspectLightProbe, {}, "LightProbe");
  InspectorRegistry::GetInstance().RegisterInspector<Mesh>(InspectMesh, {}, "Mesh");
  InspectorRegistry::GetInstance().RegisterInspector<MeshRenderer>(InspectMeshRenderer, {}, "MeshRenderer");
  InspectorRegistry::GetInstance().RegisterInspector<GaussianSplatRenderer>(InspectGaussianSplatRenderer, {},
                                                                            "GaussianSplatRenderer");
  InspectorRegistry::GetInstance().RegisterInspector<Particles>(InspectParticles, {}, "Particles");
  InspectorRegistry::GetInstance().RegisterInspector<PointCloud>(InspectPointCloud, {}, "PointCloud");
  InspectorRegistry::GetInstance().RegisterInspector<GaussianSplat>(InspectGaussianSplat, {}, "GaussianSplat");
  InspectorRegistry::GetInstance().RegisterInspector<GlobalReflectionProbe>(InspectGlobalReflectionProbe, {},
                                                                            "GlobalReflectionProbe");
  InspectorRegistry::GetInstance().RegisterInspector<PointCloudScanner>(InspectPointCloudScanner, {},
                                                                        "PointCloudScanner");
  InspectorRegistry::GetInstance().RegisterInspector<Prefab>(InspectPrefab, {}, "Prefab");
  InspectorRegistry::GetInstance().RegisterInspector<PostProcessingStack>(InspectPostProcessingStack, {},
                                                                          "PostProcessingStack");
  InspectorRegistry::GetInstance().RegisterInspector<pn::ProceduralNoise2D>(InspectProceduralNoise2D, {},
                                                                            "ProceduralNoise2D");
  InspectorRegistry::GetInstance().RegisterInspector<pn::ProceduralNoise3D>(InspectProceduralNoise3D, {},
                                                                            "ProceduralNoise3D");
  InspectorRegistry::GetInstance().RegisterInspector<pn::ProceduralNoise4D>(InspectProceduralNoise4D, {},
                                                                            "ProceduralNoise4D");
  InspectorRegistry::GetInstance().RegisterInspector<PointLight>(InspectPointLight, {}, "PointLight");
  InspectorRegistry::GetInstance().RegisterInspector<PlayerController>(InspectPlayerController, {}, "PlayerController");
  InspectorRegistry::GetInstance().RegisterInspector<RenderLayer>(InspectRenderLayer, {}, "RenderLayer");
  InspectorRegistry::GetInstance().RegisterInspector<Scene>(InspectScene, {}, "Scene");
  InspectorRegistry::GetInstance().RegisterInspector<Shader>(InspectShader, {}, "Shader");
  InspectorRegistry::GetInstance().RegisterInspector<SkinnedMesh>(InspectSkinnedMesh, {}, "SkinnedMesh");
  InspectorRegistry::GetInstance().RegisterInspector<SkinnedMeshRenderer>(InspectSkinnedMeshRenderer, {},
                                                                          "SkinnedMeshRenderer");
  InspectorRegistry::GetInstance().RegisterInspector<SkyIllumination>(InspectSkyIllumination, {}, "SkyIllumination");
  InspectorRegistry::GetInstance().RegisterInspector<SpotLight>(InspectSpotLight, {}, "SpotLight");
  InspectorRegistry::GetInstance().RegisterInspector<Strands>(InspectStrands, {}, "Strands");
  InspectorRegistry::GetInstance().RegisterInspector<StrandsRenderer>(InspectStrandsRenderer, {}, "StrandsRenderer");
  InspectorRegistry::GetInstance().RegisterInspector<LodGroup>(InspectLodGroup, {}, "LodGroup");
  InspectorRegistry::GetInstance().RegisterInspector<Material>(InspectMaterial, {}, "Material");
  InspectorRegistry::GetInstance().RegisterInspector<Texture2D>(InspectTexture2D, {}, "Texture2D");
  InspectorRegistry::GetInstance().RegisterInspector<UnknownAsset>(InspectUnknownAsset, {}, "UnknownAsset");
  InspectorRegistry::GetInstance().RegisterInspector<UnknownLayer>(InspectUnknownLayer, {}, "UnknownLayer");
  InspectorRegistry::GetInstance().RegisterInspector<UnknownPrivateComponent>(InspectUnknownPrivateComponent, {},
                                                                              "UnknownPrivateComponent");
  InspectorRegistry::GetInstance().RegisterInspector<UnknownSystem>(InspectUnknownSystem, {}, "UnknownSystem");
}
