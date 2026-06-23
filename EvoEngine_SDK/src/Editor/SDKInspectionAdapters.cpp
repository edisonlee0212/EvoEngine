#include "SDKInspectionAdapters.hpp"

#include "Animation.hpp"
#include "AnimationPlayer.hpp"
#include "Animator.hpp"
#include "ApplicationContext.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "Cubemap.hpp"
#include "DdgiVolume.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalMap.hpp"
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

#include <cmath>
#include <functional>
#include <limits>
#include <map>

using namespace evo_engine;

namespace {
using MaterialTextureSetter = void (Material::*)(const std::shared_ptr<Texture2D>&);
namespace pn = evo_engine::procedural_noise;
using NoiseGraph = NodeGraph<pn::InputPinData, pn::OutputPinData, pn::NodeData, int>;

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
  if (ImGui::Combo("Render Mode", {"Rasterization", "Ray Tracing"}, mode)) {
    camera.camera_render_mode = static_cast<Camera::CameraRenderMode>(mode);
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
  if (camera.camera_render_mode == Camera::CameraRenderMode::RayTracing) {
    if (ImGui::DragFloat("Gamma", &camera.camera_settings.gamma, 0.01f, 0.01f, 10.0f)) {
      changed = true;
    }
    if (ImGui::SliderInt("Samples", &camera.camera_settings.sample_size, 1, 32)) {
      changed = true;
    }
    if (ImGui::SliderInt("Bounce", &camera.camera_settings.bounce, 1, 8)) {
      changed = true;
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

  if (ImGui::TreeNodeEx("Background", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::DragFloat("Intensity", &camera.camera_settings.background_intensity, 0.01f, 0.0f, 10.f)) {
      changed = true;
    }
    if (ImGui::Checkbox("Use clear color", &camera.camera_settings.use_clear_color)) {
      changed = true;
    }
    if (camera.camera_settings.use_clear_color) {
      if (ImGui::ColorEdit4("Clear Color", reinterpret_cast<float*>(&camera.camera_settings.clear_color))) {
        changed = true;
      }
    } else if (editor_layer->DragAndDropButton<Cubemap>(camera.skybox, "Skybox")) {
      changed = true;
    }
    ImGui::TreePop();
  }

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

bool InspectScreenSpaceAmbientOcclusion(ScreenSpaceAmbientOcclusion& ssao) {
  bool changed = false;
  if (ImGui::DragInt("Kernel size", &ssao.kernel_size, 1, 1, 64))
    changed = true;
  if (ImGui::DragFloat("Disk radius", &ssao.radius, 0.001f, 0.0f, 10.f))
    changed = true;
  if (ImGui::DragFloat("Bias", &ssao.bias, 0.001f, 0.0f, 1.f))
    changed = true;
  if (ImGui::DragFloat("Factor", &ssao.factor, 0.01f, 0.0f, 5.f))
    changed = true;
  if (ImGui::DragFloat("Intensity", &ssao.intensity, 0.01f, 0.0f, 5.f))
    changed = true;
  if (ImGui::DragFloat("Avoid distance", &ssao.avoid_distance, 0.1f, 0.0f, 100.f))
    changed = true;
  if (ImGui::Button("Rebuild pipelines")) {
    ssao.BuildPipelines();
  }
  return changed;
}

bool InspectBloom(Bloom& bloom) {
  bool changed = false;
  if (ImGui::DragFloat("Filter radius", &bloom.filter_radius, 0.0001f, 0.0001f, .1f))
    changed = true;
  if (ImGui::DragInt("Chain length", &bloom.bloom_chain_length, 1, 0, 10))
    changed = true;
  if (ImGui::Button("Rebuild pipelines")) {
    bloom.BuildPipelines();
  }
  return changed;
}

bool InspectScreenSpaceReflection(ScreenSpaceReflection& ssr) {
  bool changed = false;
  if (ImGui::DragFloat("Max march distance", &ssr.max_distance, 0.01f, 0.01f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Distance confidence", &ssr.distance_confidence, 0.1f, 0.0f, 128.0f))
    changed = true;
  if (ImGui::DragInt("Max iteration count", &ssr.max_iteration_count, 1, 1, 256))
    changed = true;
  if (ImGui::DragInt("Steps", &ssr.initial_steps, 1, 1, 64))
    changed = true;
  if (ImGui::DragFloat("Thickness", &ssr.thickness, 0.01f, 0.0f, 10.0f))
    changed = true;
  if (ImGui::Checkbox("Blur", &ssr.blur))
    changed = true;
  if (ImGui::Button("Rebuild pipelines")) {
    ssr.BuildPipelines();
  }
  return changed;
}

bool InspectToneMapping(ToneMapping& tone_mapping) {
  bool changed = false;
  if (ImGui::DragFloat("Exposure", &tone_mapping.exposure, 0.01f, 0.01f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Gamma", &tone_mapping.gamma, 0.01f, 0.01f, 10.0f))
    changed = true;
  return changed;
}

bool InspectPostProcessingStack(InspectorContext&, PostProcessingStack& stack) {
  bool changed = false;

  if (ImGui::Checkbox("SSAO", &stack.enable_screen_space_ambient_occlusion))
    changed = true;
  if (stack.enable_screen_space_ambient_occlusion && ImGui::TreeNodeEx("SSAO", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (InspectScreenSpaceAmbientOcclusion(*stack.screen_space_ambient_occlusion))
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::Checkbox("Bloom", &stack.enable_bloom))
    changed = true;
  if (stack.enable_bloom && ImGui::TreeNodeEx("Bloom", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (InspectBloom(*stack.bloom))
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::Checkbox("SSR", &stack.enable_screen_space_reflection))
    changed = true;
  if (stack.enable_screen_space_reflection && ImGui::TreeNodeEx("SSR", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (InspectScreenSpaceReflection(*stack.screen_space_reflection))
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::Checkbox("Tone Mapping", &stack.enable_tone_mapping))
    changed = true;
  if (stack.enable_tone_mapping && ImGui::TreeNodeEx("Tong Mapping", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (InspectToneMapping(*stack.tone_mapping))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Debug")) {
    static float debug_scale = 0.25f;
    ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
    debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
    auto initial_size = ImVec2(stack.source_color_texture->GetExtent().width * debug_scale,
                               stack.source_color_texture->GetExtent().height * debug_scale);
    if (ImGui::TreeNode("Source")) {
      ImGui::Image(stack.source_color_texture->GetColorImTextureId(), initial_size, ImVec2(0, 1), ImVec2(1, 0));
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Result")) {
      ImGui::Image(stack.result_texture->GetColorImTextureId(),
                   ImVec2(stack.result_texture->GetExtent().width * debug_scale,
                          stack.result_texture->GetExtent().height * debug_scale),
                   ImVec2(0, 1), ImVec2(1, 0));
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Mipmaps")) {
      const auto mip_levels = stack.result_texture->GetMipLevels();
      for (uint32_t mip_level = 1; mip_level < mip_levels; mip_level++) {
        initial_size /= 2.f;
        ImGui::Image(stack.result_texture->GetColorImTextureId(mip_level), initial_size, ImVec2(0, 1), ImVec2(1, 0));
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Swap")) {
      ImGui::Image(stack.swap_texture->GetColorImTextureId(),
                   ImVec2(stack.swap_texture->GetExtent().width * debug_scale,
                          stack.swap_texture->GetExtent().height * debug_scale),
                   ImVec2(0, 1), ImVec2(1, 0));
      ImGui::TreePop();
    }
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

bool InspectDrawSettings(DrawSettings& draw_settings) {
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
}

void InspectShadowSettings(RenderSettings& render_settings) {
  if (ImGui::TreeNode("Distance")) {
    if (ImGui::DragFloat("Max shadow distance", &render_settings.max_shadow_distance, 1.0f, 10.f, 1000.f)) {
      render_settings.max_shadow_distance = glm::clamp(render_settings.max_shadow_distance, 10.f, 1000.f);
    }
    if (ImGui::DragFloat("Split 1", &render_settings.shadow_cascade_split[0], 0.01f, 0.0f,
                         render_settings.shadow_cascade_split[1])) {
      render_settings.shadow_cascade_split[0] =
          glm::clamp(render_settings.shadow_cascade_split[0], 0.f, render_settings.shadow_cascade_split[1]);
    }
    if (ImGui::DragFloat("Split 2", &render_settings.shadow_cascade_split[1], 0.01f,
                         render_settings.shadow_cascade_split[0], render_settings.shadow_cascade_split[2])) {
      render_settings.shadow_cascade_split[1] =
          glm::clamp(render_settings.shadow_cascade_split[1], render_settings.shadow_cascade_split[0],
                     render_settings.shadow_cascade_split[2]);
    }
    if (ImGui::DragFloat("Split 3", &render_settings.shadow_cascade_split[2], 0.01f,
                         render_settings.shadow_cascade_split[1], render_settings.shadow_cascade_split[3])) {
      render_settings.shadow_cascade_split[2] =
          glm::clamp(render_settings.shadow_cascade_split[2], render_settings.shadow_cascade_split[1],
                     render_settings.shadow_cascade_split[3]);
    }
    if (ImGui::DragFloat("Split 4", &render_settings.shadow_cascade_split[3], 0.01f,
                         render_settings.shadow_cascade_split[2], 1.0f)) {
      render_settings.shadow_cascade_split[3] =
          glm::clamp(render_settings.shadow_cascade_split[3], render_settings.shadow_cascade_split[2], 1.f);
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("PCSS")) {
    ImGui::DragInt("PCF Sample Size", &render_settings.pcf_sample_amount, 1, 1, 64);
    ImGui::TreePop();
  }
  ImGui::DragFloat("Seam fix ratio", &render_settings.seam_fix_ratio, 0.001f, 0.0f, 0.1f);
  ImGui::Checkbox("Stable fit", &render_settings.stable_fit);
}

void InspectStrandsSettings(RenderSettings& render_settings) {
#ifdef EVOENGINE_WINDOWS
  ImGui::DragFloat("Curve subdivision factor", &render_settings.strands_subdivision_x_factor, 1.0f, 1.0f, 1000.0f);
  ImGui::DragFloat("Ring subdivision factor", &render_settings.strands_subdivision_y_factor, 1.0f, 1.0f, 1000.0f);
  ImGui::DragInt("Max curve subdivision", &render_settings.strands_subdivision_max_x, 1, 1, 15);
  ImGui::DragInt("Max ring subdivision", &render_settings.strands_subdivision_max_y, 1, 1, 15);
#else
  ImGui::TextUnformatted("Strands settings are only available on Windows.");
#endif
}

bool InspectVolumetricCloudSettings(VolumetricCloudSettings& settings) {
  settings.ClampSettings();
  bool modified = false;
  if (ImGui::Checkbox("Enable volumetric clouds", &settings.enabled))
    modified = true;
  if (ImGui::DragFloat("Coverage", &settings.coverage, 0.001f, 0.0f, 1.0f, "%.3f"))
    modified = true;
  if (ImGui::DragFloat("Density", &settings.density, 0.001f, 0.0f, 10.0f, "%.3f"))
    modified = true;
  if (ImGui::DragFloat("Bottom altitude", &settings.bottom_altitude, 10.0f, 0.0f, 100000.0f, "%.1f"))
    modified = true;
  if (ImGui::DragFloat("Top altitude", &settings.top_altitude, 10.0f, 1.0f, 100000.0f, "%.1f"))
    modified = true;
  if (ImGui::DragFloat2("Wind direction", &settings.wind_direction.x, 0.001f, -1.0f, 1.0f, "%.3f"))
    modified = true;
  if (ImGui::DragFloat("Wind speed", &settings.wind_speed, 0.1f, 0.0f, 10000.0f, "%.1f"))
    modified = true;
  if (ImGui::DragInt("Primary steps", &settings.primary_step_count, 1.0f, 1, 512))
    modified = true;
  if (ImGui::DragInt("Light steps", &settings.light_step_count, 1.0f, 1, 128))
    modified = true;
  int resolution_mode = settings.resolution_divisor == 4 ? 2 : (settings.resolution_divisor == 2 ? 1 : 0);
  const char* resolution_modes[] = {"Full", "Half", "Quarter"};
  if (ImGui::Combo("Cloud resolution", &resolution_mode, resolution_modes, IM_ARRAYSIZE(resolution_modes))) {
    settings.resolution_divisor = resolution_mode == 2 ? 4 : (resolution_mode == 1 ? 2 : 1);
    modified = true;
  }
  if (ImGui::DragFloat("Lighting intensity", &settings.lighting_intensity, 0.01f, 0.0f, 100.0f, "%.3f"))
    modified = true;
  if (ImGui::DragFloat("Ambient lighting", &settings.ambient_lighting_strength, 0.01f, 0.0f, 10.0f, "%.3f"))
    modified = true;
  if (ImGui::DragFloat("Phase anisotropy", &settings.phase_anisotropy, 0.001f, -0.99f, 0.99f, "%.3f"))
    modified = true;
  if (ImGui::Checkbox("Cloud debug", &settings.debug_visualization))
    modified = true;
  const char* debug_modes[] = {"Final", "Density", "Transmittance", "March depth"};
  if (ImGui::Combo("Cloud debug mode", &settings.debug_mode, debug_modes, IM_ARRAYSIZE(debug_modes)))
    modified = true;
  if (modified) {
    settings.ClampSettings();
  }
  return modified;
}

void ClampDdgiSettings(RenderLayer::DdgiSettings& settings) {
  auto& runtime = settings.runtime;
  runtime.ray_count = glm::clamp(runtime.ray_count, 1, 4096);
  runtime.warmup_frames = glm::clamp(runtime.warmup_frames, 0, 4096);
  runtime.hysteresis = glm::clamp(runtime.hysteresis, 0.0f, 1.0f);
  runtime.normal_bias = glm::clamp(runtime.normal_bias, 0.0f, 10.0f);
  runtime.view_bias = glm::clamp(runtime.view_bias, 0.0f, 10.0f);
  runtime.max_ray_distance = glm::clamp(runtime.max_ray_distance, 0.05f, 1e27f);
  runtime.distance_exponent = glm::clamp(runtime.distance_exponent, 0.0f, 256.0f);
  runtime.irradiance_gamma = glm::clamp(runtime.irradiance_gamma, 0.1f, 16.0f);
  runtime.visibility_moment_bias = glm::clamp(runtime.visibility_moment_bias, 0.0f, 10.0f);
  runtime.indirect_intensity = glm::clamp(runtime.indirect_intensity, 0.0f, 10.0f);
  runtime.irradiance_threshold = glm::clamp(runtime.irradiance_threshold, 0.0f, 1.0f);
  runtime.brightness_threshold = glm::clamp(runtime.brightness_threshold, 0.0f, 1.0f);

  auto& volume = settings.volume_defaults;
  volume.probe_counts.x = glm::clamp(volume.probe_counts.x, 1, 256);
  volume.probe_counts.y = glm::clamp(volume.probe_counts.y, 1, 256);
  volume.probe_counts.z = glm::clamp(volume.probe_counts.z, 1, 256);
  volume.probe_spacing = glm::clamp(volume.probe_spacing, glm::vec3(0.05f), glm::vec3(10000.0f));
  volume.movement_type = glm::clamp(volume.movement_type, static_cast<int>(DdgiVolumeMovementType::Default),
                                    static_cast<int>(DdgiVolumeMovementType::Scrolling));
  volume.relocation_distance = glm::clamp(volume.relocation_distance, 0.0f, 10000.0f);
  volume.random_ray_backface_threshold = glm::clamp(volume.random_ray_backface_threshold, 0.0f, 1.0f);
  volume.fixed_ray_backface_threshold = glm::clamp(volume.fixed_ray_backface_threshold, 0.0f, 1.0f);
  volume.probe_variability_threshold = glm::clamp(volume.probe_variability_threshold, 0.0f, 10.0f);
  volume.probe_variability_min_samples = glm::clamp(volume.probe_variability_min_samples, 0, 4096);

  auto& storage = settings.storage;
  storage.max_probe_count = glm::clamp(storage.max_probe_count, 1, 16777216);
  storage.irradiance_tile_resolution = glm::clamp(storage.irradiance_tile_resolution, 1, 128);
  storage.visibility_tile_resolution = glm::clamp(storage.visibility_tile_resolution, 1, 128);
  storage.atlas_probe_columns = glm::clamp(storage.atlas_probe_columns, 1, 4096);

  auto& debug = settings.debug;
  const auto probe_count = RenderLayer::GetDdgiAllocatedProbeCount(settings);
  debug.selected_probe_index = glm::clamp(debug.selected_probe_index, 0, static_cast<int>(probe_count - 1u));
  debug.atlas_layer = glm::clamp(debug.atlas_layer, 0, 4096);
  debug.visualization_scale = glm::clamp(debug.visualization_scale, 0.01f, 1000.0f);
  debug.probe_visualization_mode = glm::clamp(debug.probe_visualization_mode, 0, 3);
  debug.probe_visualization_depth_mode = glm::clamp(debug.probe_visualization_depth_mode, 0, 1);
  debug.probe_visualization_radius = glm::clamp(debug.probe_visualization_radius, 0.001f, 1000.0f);
  debug.probe_visualization_intensity = glm::clamp(debug.probe_visualization_intensity, 0.0f, 1000.0f);
  debug.probe_visualization_alpha = glm::clamp(debug.probe_visualization_alpha, 0.0f, 1.0f);
  debug.selected_probe_visualization_scale = glm::clamp(debug.selected_probe_visualization_scale, 1.0f, 1000.0f);
}

bool DrawDdgiVolumeTriggerConditionCheckbox(const char* label, int& trigger_conditions, const int condition) {
  bool enabled = (trigger_conditions & condition) != 0;
  if (ImGui::Checkbox(label, &enabled)) {
    if (enabled) {
      trigger_conditions |= condition;
    } else {
      trigger_conditions &= ~condition;
    }
    trigger_conditions &= DdgiVolumeTriggerConditionAll;
    return true;
  }
  return false;
}

void DrawDdgiAtlasReadout(const char* label, const RenderLayer::DdgiProbeDebugCoordinates& coordinates) {
  ImGui::Text("%s atlas: %u x %u", label, coordinates.atlas_layout.resolution.x, coordinates.atlas_layout.resolution.y);
  ImGui::Text("%s tile: %u, %u", label, coordinates.atlas_tile_offset.x, coordinates.atlas_tile_offset.y);
  const auto tile_stride = coordinates.atlas_layout.tile_resolution + 2u;
  ImGui::Text("%s tile interior: %u x %u", label, coordinates.atlas_layout.tile_resolution,
              coordinates.atlas_layout.tile_resolution);
  ImGui::Text("%s tile stride: %u", label, tile_stride);
  ImGui::Text("%s wrapped border: x %u-%u, y %u-%u", label, coordinates.atlas_tile_offset.x,
              coordinates.atlas_tile_offset.x + tile_stride - 1u, coordinates.atlas_tile_offset.y,
              coordinates.atlas_tile_offset.y + tile_stride - 1u);
}

glm::ivec3 WrapDdgiProbeGrid(const glm::ivec3& probe_grid, const glm::ivec3& probe_counts) {
  const auto safe_counts = glm::max(probe_counts, glm::ivec3(1));
  return (probe_grid % safe_counts + safe_counts) % safe_counts;
}

uint32_t GetDdgiProbeIndexFromGrid(const glm::ivec3& probe_grid, const glm::ivec3& probe_counts) {
  const auto safe_counts = glm::max(probe_counts, glm::ivec3(1));
  const auto wrapped_grid = WrapDdgiProbeGrid(probe_grid, safe_counts);
  return static_cast<uint32_t>(wrapped_grid.x + wrapped_grid.y * safe_counts.x +
                               wrapped_grid.z * safe_counts.x * safe_counts.y);
}

uint32_t GetScrolledDdgiProbeIndex(const glm::uvec3& logical_probe_grid, const glm::ivec3& probe_scroll_offset,
                                   const glm::ivec3& probe_counts) {
  return GetDdgiProbeIndexFromGrid(glm::ivec3(logical_probe_grid) + probe_scroll_offset, probe_counts);
}

const RenderLayer::DdgiVolumeRuntimeInfo* GetSelectedDdgiVolumeInfo(
    const std::vector<RenderLayer::DdgiVolumeRuntimeInfo>& volume_infos) {
  if (volume_infos.empty()) {
    return nullptr;
  }
  return &volume_infos.front();
}

RenderLayer::DdgiProbeDebugCoordinates CalculateActiveDdgiProbeDebugCoordinates(
    const RenderLayer::DdgiSettings& settings, const glm::ivec3& active_probe_counts, const uint32_t tile_resolution) {
  RenderLayer::DdgiProbeDebugCoordinates coordinates;
  const auto active_probe_count = RenderLayer::GetDdgiProbeCount(active_probe_counts);
  const auto allocated_probe_count = RenderLayer::GetDdgiAllocatedProbeCount(settings, active_probe_count);
  coordinates.probe_index =
      glm::min(static_cast<uint32_t>(glm::max(settings.debug.selected_probe_index, 0)), active_probe_count - 1u);
  coordinates.grid_index = RenderLayer::GetDdgiProbeGridIndex(active_probe_counts, coordinates.probe_index);
  coordinates.atlas_layout = RenderLayer::CalculateDdgiAtlasLayout(
      allocated_probe_count, tile_resolution, static_cast<uint32_t>(glm::max(settings.storage.atlas_probe_columns, 1)));
  const auto tile_stride = coordinates.atlas_layout.tile_resolution + 2u;
  coordinates.atlas_tile_offset = {(coordinates.probe_index % coordinates.atlas_layout.columns) * tile_stride,
                                   (coordinates.probe_index / coordinates.atlas_layout.columns) * tile_stride};
  return coordinates;
}

void DrawDdgiSelectedProbeStateReadout(const RenderLayer::DdgiProbeDebugDataView& debug_data,
                                       const uint32_t selected_logical_probe, const uint32_t selected_physical_probe) {
  if (selected_physical_probe != selected_logical_probe) {
    ImGui::Text("Selected physical probe: %u", selected_physical_probe);
  }
  const auto metadata_offset = static_cast<size_t>(selected_physical_probe) * 3ull;
  if (debug_data.probe_count == 0 || selected_physical_probe >= debug_data.probe_count || !debug_data.metadata ||
      debug_data.metadata->size() < metadata_offset + 3ull) {
    ImGui::Text("Selected probe state: unavailable");
    return;
  }

  const auto irradiance_sample = (*debug_data.metadata)[metadata_offset];
  const auto visibility_sample = (*debug_data.metadata)[metadata_offset + 1ull];
  const auto state_sample = (*debug_data.metadata)[metadata_offset + 2ull];
  ImGui::Text("Selected probe state: %s", state_sample.w >= 0.5f ? "active" : "inactive");
  ImGui::Text("Selected irradiance: %.3f, %.3f, %.3f", irradiance_sample.x, irradiance_sample.y, irradiance_sample.z);
  ImGui::Text("Selected hit ratio: %.3f", irradiance_sample.w);
  ImGui::Text("Selected miss ratio: %.3f", 1.0f - glm::clamp(irradiance_sample.w, 0.0f, 1.0f));
  ImGui::Text("Selected backface ratio: %.3f", visibility_sample.y);
  ImGui::Text("Selected visibility distance: %.3f", visibility_sample.x);
  ImGui::Text("Selected relocation: %.3f, %.3f, %.3f", state_sample.x, state_sample.y, state_sample.z);
  ImGui::Text("Selected relocation amount: %.3f", visibility_sample.z);
  if (debug_data.update_ages && selected_physical_probe < debug_data.update_ages->size()) {
    ImGui::Text("Selected update age: %.0f frames", (*debug_data.update_ages)[selected_physical_probe]);
  }
}

enum class DdgiRaySampleType { Frontface, Backface, Miss, Inactive };

DdgiRaySampleType GetDdgiRaySampleType(const PointCloudSample& sample) {
  if (sample.hit_count == 0u && sample.hit_info.color.a <= -1.5f) {
    return DdgiRaySampleType::Inactive;
  }
  if (sample.hit_count == 0u) {
    return DdgiRaySampleType::Miss;
  }
  return sample.hit_info.color.a < 0.0f ? DdgiRaySampleType::Backface : DdgiRaySampleType::Frontface;
}

const char* FormatDdgiRaySampleType(const DdgiRaySampleType sample_type) {
  switch (sample_type) {
    case DdgiRaySampleType::Backface:
      return "backface";
    case DdgiRaySampleType::Miss:
      return "miss";
    case DdgiRaySampleType::Inactive:
      return "inactive";
    case DdgiRaySampleType::Frontface:
    default:
      return "frontface";
  }
}

void DrawDdgiSelectedProbeRayReadout(const RenderLayer::DdgiProbeDebugDataView& debug_data, const bool show_rays) {
  if (!show_rays) {
    return;
  }
  if (!debug_data.selected_ray_samples_available || !debug_data.selected_ray_samples ||
      debug_data.selected_ray_samples->empty() || debug_data.selected_ray_sample_count == 0u) {
    ImGui::Text("Selected ray samples: unavailable this frame");
    return;
  }

  const auto sample_count =
      glm::min(debug_data.selected_ray_sample_count, static_cast<uint32_t>(debug_data.selected_ray_samples->size()));
  uint32_t fixed_ray_count = 0;
  uint32_t frontface_count = 0;
  uint32_t backface_count = 0;
  uint32_t miss_count = 0;
  uint32_t inactive_count = 0;
  float hit_distance_sum = 0.0f;
  for (uint32_t sample_index = 0; sample_index < sample_count; ++sample_index) {
    const auto& sample = (*debug_data.selected_ray_samples)[sample_index];
    fixed_ray_count += sample.padding0 > 0.5f ? 1u : 0u;
    const auto sample_type = GetDdgiRaySampleType(sample);
    switch (sample_type) {
      case DdgiRaySampleType::Inactive:
        inactive_count++;
        break;
      case DdgiRaySampleType::Miss:
        miss_count++;
        break;
      case DdgiRaySampleType::Backface:
        backface_count++;
        hit_distance_sum += glm::length(sample.hit_info.position - sample.start);
        break;
      case DdgiRaySampleType::Frontface:
        frontface_count++;
        hit_distance_sum += glm::length(sample.hit_info.position - sample.start);
        break;
    }
  }
  const auto hit_count = frontface_count + backface_count;
  ImGui::Text("Selected ray probe: %u", debug_data.selected_ray_probe_index);
  if (debug_data.selected_ray_physical_probe_index != debug_data.selected_ray_probe_index) {
    ImGui::Text("Selected ray physical probe: %u", debug_data.selected_ray_physical_probe_index);
  }
  ImGui::Text("Selected ray samples: %u", sample_count);
  ImGui::Text("Selected ray front/back/miss/inactive: %u / %u / %u / %u", frontface_count, backface_count, miss_count,
              inactive_count);
  ImGui::Text("Selected fixed rays: %u", fixed_ray_count);
  if (hit_count != 0u) {
    ImGui::Text("Selected average hit distance: %.3f", hit_distance_sum / static_cast<float>(hit_count));
  }
  if (ImGui::TreeNode("Selected ray sample preview")) {
    const auto preview_count = glm::min(sample_count, 8u);
    for (uint32_t sample_index = 0; sample_index < preview_count; ++sample_index) {
      const auto& sample = (*debug_data.selected_ray_samples)[sample_index];
      ImGui::Text("#%u %s dir %.2f %.2f %.2f color %.2f %.2f %.2f %.2f", sample_index,
                  FormatDdgiRaySampleType(GetDdgiRaySampleType(sample)), sample.direction.x, sample.direction.y,
                  sample.direction.z, sample.hit_info.color.x, sample.hit_info.color.y, sample.hit_info.color.z,
                  sample.hit_info.color.w);
    }
    ImGui::TreePop();
  }
}

void InspectDdgiSettings(RenderLayer::DdgiSettings& settings, const glm::ivec3& probe_scroll_offset,
                         const glm::ivec3& last_probe_scroll_delta, const uint32_t pending_probe_update_count,
                         const std::string& last_probe_update_reasons,
                         const RenderLayer::DdgiProbeUpdateStats& last_probe_update_stats,
                         const RenderLayer::DdgiPerformanceStats& performance_stats,
                         const RenderLayer::DdgiProbeDebugDataView& probe_debug_data, const bool show_header = true) {
  if (show_header && !ImGui::CollapsingHeader("DDGI", ImGuiTreeNodeFlags_DefaultOpen)) {
    ClampDdgiSettings(settings);
    return;
  }
  ClampDdgiSettings(settings);

  auto& runtime = settings.runtime;
  auto& volume = settings.volume_defaults;
  auto& storage = settings.storage;
  auto& debug = settings.debug;
  const auto volume_infos =
      RenderLayer::CollectDdgiVolumeRuntimeInfos(ApplicationContext::Get().GetActiveScene(), settings);
  const auto* selected_volume_info = GetSelectedDdgiVolumeInfo(volume_infos);
  const auto active_probe_counts = selected_volume_info ? selected_volume_info->probe_counts : volume.probe_counts;
  const auto active_probe_count = RenderLayer::GetDdgiProbeCount(active_probe_counts);
  auto probe_count = RenderLayer::GetDdgiAllocatedProbeCount(settings, active_probe_count);
  auto irradiance_coordinates = CalculateActiveDdgiProbeDebugCoordinates(
      settings, active_probe_counts, static_cast<uint32_t>(storage.irradiance_tile_resolution));
  auto visibility_coordinates = CalculateActiveDdgiProbeDebugCoordinates(
      settings, active_probe_counts, static_cast<uint32_t>(storage.visibility_tile_resolution));
  auto frame_layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, active_probe_count);

  ImGui::Checkbox("Enable DDGI", &runtime.enabled);
  ImGui::SameLine();
  ImGui::Checkbox("Pause updates", &runtime.pause_updates);
  ImGui::SameLine();
  if (ImGui::Button("Reset history")) {
    runtime.reset_probe_history = true;
  }
  ImGui::SameLine();
  ImGui::Checkbox("Reset pending", &runtime.reset_probe_history);

  if (ImGui::TreeNodeEx("Runtime", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Active probes: %u", active_probe_count);
    ImGui::Text("Storage probes: %u", probe_count);
    ImGui::Text("Selected probe: %u", irradiance_coordinates.probe_index);
    ImGui::Text("Last probe update: %s", last_probe_update_reasons.c_str());
    ImGui::Text("Last update probes: %u", last_probe_update_stats.probe_count);
    if (last_probe_update_stats.probe_count != 0u) {
      ImGui::Text("Last update index range: %u - %u", last_probe_update_stats.first_probe_index,
                  last_probe_update_stats.last_probe_index);
    }
    ImGui::Text("DDGI scroll offset: %d, %d, %d", probe_scroll_offset.x, probe_scroll_offset.y, probe_scroll_offset.z);
    ImGui::Text("DDGI scroll delta: %d, %d, %d", last_probe_scroll_delta.x, last_probe_scroll_delta.y,
                last_probe_scroll_delta.z);
    ImGui::Text("DDGI pending probes: %u", pending_probe_update_count);
    ImGui::Text("DDGI frame CPU: %.3f ms", performance_stats.frame_graph_execute_ms);
    ImGui::Text("DDGI record CPU atlas/rays/update/relocate/classify: %.3f / %.3f / %.3f / %.3f / %.3f ms",
                performance_stats.atlas_prepare_record_ms, performance_stats.ray_diagnostics_record_ms,
                performance_stats.probe_update_record_ms, performance_stats.probe_relocation_record_ms,
                performance_stats.probe_classification_record_ms);
    ImGui::Text("DDGI variability CPU: %.3f ms", performance_stats.probe_variability_record_ms);
    ImGui::Text("DDGI variability: %.4f, samples %u, stable %u/%u, %s", performance_stats.probe_variability_average,
                performance_stats.probe_variability_sample_count,
                performance_stats.probe_variability_stable_sample_count,
                performance_stats.probe_variability_required_stable_sample_count,
                performance_stats.probe_variability_converged ? "converged" : "updating");
    ImGui::Text("DDGI warm up: %u/%u, hysteresis %.3f, %s", performance_stats.probe_warmup_frame_index,
                performance_stats.probe_warmup_frame_count, performance_stats.probe_update_hysteresis,
                performance_stats.probe_warmup_active ? "active" : "complete");
    ImGui::Text("DDGI debug CPU probes/rays: %.3f / %.3f ms", performance_stats.probe_visualization_record_ms,
                performance_stats.probe_ray_visualization_record_ms);
    ImGui::Text("DDGI ray samples: %u (%u rays/probe)", performance_stats.ray_sample_count,
                performance_stats.ray_count);
    ImGui::Text("DDGI visualized probes/rays: %u / %u", performance_stats.visualized_probe_count,
                performance_stats.selected_ray_sample_count);
    ImGui::Text("DDGI buffer bytes metadata/state/rays: %llu / %llu / %llu",
                static_cast<unsigned long long>(performance_stats.probe_metadata_byte_size),
                static_cast<unsigned long long>(performance_stats.probe_state_byte_size),
                static_cast<unsigned long long>(performance_stats.ray_output_byte_size));
    ImGui::Text("DDGI atlas sizes: %u x %u / %u x %u", performance_stats.irradiance_atlas_extent.x,
                performance_stats.irradiance_atlas_extent.y, performance_stats.visibility_atlas_extent.x,
                performance_stats.visibility_atlas_extent.y);
    ImGui::Text("DDGI variability atlas/reduction: %u x %u / %u x %u", performance_stats.variability_atlas_extent.x,
                performance_stats.variability_atlas_extent.y, performance_stats.variability_reduction_extent.x,
                performance_stats.variability_reduction_extent.y);
    ImGui::Text("Enabled DDGI volumes: %llu", static_cast<unsigned long long>(volume_infos.size()));
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Probe blending", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::DragFloat("Hysteresis", &runtime.hysteresis, 0.001f, 0.0f, 1.0f, "%.3f");
    ImGui::DragInt("Warm up frames", &runtime.warmup_frames, 1.0f, 0, 4096);
    ImGui::DragFloat("Distance exponent", &runtime.distance_exponent, 0.1f, 0.0f, 256.0f);
    ImGui::DragFloat("Irradiance gamma", &runtime.irradiance_gamma, 0.01f, 0.1f, 16.0f);
    ImGui::DragFloat("Visibility moment bias", &runtime.visibility_moment_bias, 0.001f, 0.0f, 10.0f, "%.3f");
    ImGui::DragFloat("Irradiance threshold", &runtime.irradiance_threshold, 0.001f, 0.0f, 1.0f, "%.3f");
    ImGui::DragFloat("Brightness threshold", &runtime.brightness_threshold, 0.001f, 0.0f, 1.0f, "%.3f");
    ImGui::DragFloat("Indirect intensity", &runtime.indirect_intensity, 0.01f, 0.0f, 10.0f, "%.3f");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Ray tracing", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::DragInt("Ray count", &runtime.ray_count, 1.0f, 1, 4096);
    ImGui::DragFloat("Max ray distance", &runtime.max_ray_distance, 0.1f, 0.05f, 1e27f);
    ImGui::DragFloat("Normal bias", &runtime.normal_bias, 0.001f, 0.0f, 10.0f, "%.3f");
    ImGui::DragFloat("View bias", &runtime.view_bias, 0.001f, 0.0f, 10.0f, "%.3f");
    ClampDdgiSettings(settings);
    frame_layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, active_probe_count);
    ImGui::Text("Ray samples: %llu",
                static_cast<unsigned long long>(frame_layout.probe_count * static_cast<uint32_t>(runtime.ray_count)));
    ImGui::Text("Ray output bytes: %llu", static_cast<unsigned long long>(frame_layout.ray_output_byte_size));
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Atlas storage", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::DragInt("Max probe count", &storage.max_probe_count, 1.0f, 1, 16777216);
    ImGui::DragInt("Atlas probe columns", &storage.atlas_probe_columns, 1.0f, 1, 4096);
    ImGui::DragInt("Irradiance tile resolution", &storage.irradiance_tile_resolution, 1.0f, 1, 128);
    ImGui::DragInt("Visibility tile resolution", &storage.visibility_tile_resolution, 1.0f, 1, 128);
    ImGui::DragInt("Atlas layer", &debug.atlas_layer, 1.0f, 0, 4096);
    ClampDdgiSettings(settings);
    probe_count = RenderLayer::GetDdgiAllocatedProbeCount(settings, active_probe_count);
    irradiance_coordinates = CalculateActiveDdgiProbeDebugCoordinates(
        settings, active_probe_counts, static_cast<uint32_t>(storage.irradiance_tile_resolution));
    visibility_coordinates = CalculateActiveDdgiProbeDebugCoordinates(
        settings, active_probe_counts, static_cast<uint32_t>(storage.visibility_tile_resolution));
    DrawDdgiAtlasReadout("Irradiance", irradiance_coordinates);
    DrawDdgiAtlasReadout("Visibility", visibility_coordinates);
    ImGui::Text("Variability atlas: %u x %u", frame_layout.variability_atlas.resolution.x,
                frame_layout.variability_atlas.resolution.y);
    ImGui::Text("Variability reduction: %u x %u", frame_layout.variability_reduction_extent.x,
                frame_layout.variability_reduction_extent.y);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Debug visualization", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("Enable debug visualization", &debug.enabled);
    ImGui::Checkbox("Volume bounds", &debug.visualize_volume_bounds);
    ImGui::Checkbox("Probe positions", &debug.visualize_probe_positions);
    ImGui::Checkbox("Selected probe", &debug.visualize_selected_probe);
    ImGui::Checkbox("Probe state", &debug.visualize_probe_state);
    ImGui::Checkbox("Probe illumination", &debug.visualize_probe_illumination);
    ImGui::Checkbox("Atlas preview", &debug.show_atlas_preview);
    ImGui::Checkbox("Update age", &debug.show_update_age);
    ImGui::Checkbox("Rays", &debug.show_rays);
    ImGui::Checkbox("Irradiance", &debug.show_irradiance);
    ImGui::Checkbox("Visibility", &debug.show_visibility);
    ImGui::Checkbox("Sampling weights", &debug.show_sampling_weights);
    ImGui::DragInt("Selected probe index", &debug.selected_probe_index, 1.0f, 0, static_cast<int>(probe_count - 1u));
    ImGui::DragFloat("Visualization scale", &debug.visualization_scale, 0.01f, 0.01f, 1000.0f);
    const char* probe_visualization_modes[] = {"Atlas irradiance", "Probe metadata", "Visibility", "Hit ratio"};
    ImGui::Combo("Probe color mode", &debug.probe_visualization_mode, probe_visualization_modes,
                 IM_ARRAYSIZE(probe_visualization_modes));
    const char* probe_visualization_depth_modes[] = {"Depth tested", "X-ray"};
    ImGui::Combo("Probe depth mode", &debug.probe_visualization_depth_mode, probe_visualization_depth_modes,
                 IM_ARRAYSIZE(probe_visualization_depth_modes));
    ImGui::DragFloat("Probe radius", &debug.probe_visualization_radius, 0.001f, 0.001f, 1000.0f);
    ImGui::DragFloat("Probe intensity", &debug.probe_visualization_intensity, 0.01f, 0.0f, 1000.0f);
    ImGui::DragFloat("Ray alpha", &debug.probe_visualization_alpha, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Selected probe scale", &debug.selected_probe_visualization_scale, 0.01f, 1.0f, 1000.0f);
    ClampDdgiSettings(settings);
    irradiance_coordinates = CalculateActiveDdgiProbeDebugCoordinates(
        settings, active_probe_counts, static_cast<uint32_t>(storage.irradiance_tile_resolution));
    ImGui::Text("Selected grid: %u, %u, %u", irradiance_coordinates.grid_index.x, irradiance_coordinates.grid_index.y,
                irradiance_coordinates.grid_index.z);
    ImGui::Text("Selected atlas column: %u", irradiance_coordinates.atlas_layout.columns);
    const auto selected_logical_probe =
        GetDdgiProbeIndexFromGrid(glm::ivec3(irradiance_coordinates.grid_index), volume.probe_counts);
    const auto selected_physical_probe =
        GetScrolledDdgiProbeIndex(irradiance_coordinates.grid_index, probe_scroll_offset, volume.probe_counts);
    DrawDdgiSelectedProbeStateReadout(probe_debug_data, selected_logical_probe, selected_physical_probe);
    DrawDdgiSelectedProbeRayReadout(probe_debug_data, debug.show_rays);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Volume selection")) {
    ImGui::Text("Active policy: single selected volume");
    ImGui::Text("Enabled volumes: %llu", static_cast<unsigned long long>(volume_infos.size()));
    if (selected_volume_info) {
      ImGui::Text("Active owner index: %u", selected_volume_info->owner_index);
      ImGui::Text("Active probes: %u (%d, %d, %d)", selected_volume_info->probe_count,
                  selected_volume_info->probe_counts.x, selected_volume_info->probe_counts.y,
                  selected_volume_info->probe_counts.z);
    }
    ImGui::TreePop();
  }

  ClampDdgiSettings(settings);
}

bool InspectRenderLayer(InspectorContext&, RenderLayer& render_layer) {
  const auto window_title =
      render_layer.force_ddgi_inspection_layout ? "RenderLayer DDGI Atlas" : render_layer.GetLayerName();
  bool open = render_layer.enable_inspection;
  if (render_layer.force_ddgi_inspection_layout) {
    ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
    ImGui::SetNextWindowDockID(0, ImGuiCond_Always);
    ImGui::SetNextWindowPos(
        {render_layer.forced_inspection_window_position.x, render_layer.forced_inspection_window_position.y},
        ImGuiCond_Always);
    ImGui::SetNextWindowSize(
        {render_layer.forced_inspection_window_size.x, render_layer.forced_inspection_window_size.y}, ImGuiCond_Always);
    ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
  }
  const auto window_flags = render_layer.force_ddgi_inspection_layout
                                ? ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings
                                : ImGuiWindowFlags_None;
  if (!ImGui::Begin(window_title.c_str(), &open, window_flags)) {
    ImGui::End();
    render_layer.enable_inspection = open;
    return false;
  }
  auto& ddgi_settings = render_layer.GetDdgiSettings();
  const auto probe_debug_data =
      render_layer.GetDdgiProbeDebugData(ddgi_settings.debug.enabled && ddgi_settings.debug.visualize_probe_state);

  const auto draw_ddgi_tab = [&](const ImGuiTabItemFlags flags = ImGuiTabItemFlags_None) {
    if (ImGui::BeginTabItem("DDGI", nullptr, flags)) {
      const auto use_forced_scroll = render_layer.force_ddgi_inspection_layout;
      if (use_forced_scroll) {
        ImGui::BeginChild("DDGIInspectionContent");
        ImGui::SetScrollY(520.0f);
      }
      InspectDdgiSettings(ddgi_settings, render_layer.GetDdgiProbeScrollOffset(),
                          render_layer.GetDdgiLastProbeScrollDelta(), render_layer.GetDdgiPendingProbeUpdateCount(),
                          render_layer.GetDdgiLastProbeUpdateReasonText(), render_layer.GetDdgiLastProbeUpdateStats(),
                          render_layer.GetDdgiLastPerformanceStats(), probe_debug_data, false);
      if (use_forced_scroll) {
        ImGui::EndChild();
      }
      ImGui::EndTabItem();
    }
  };

  if (ImGui::BeginTabBar("RenderLayerInspectionTabs")) {
    if (render_layer.force_ddgi_inspection_layout) {
      draw_ddgi_tab();
    }
    if (ImGui::BeginTabItem("General")) {
      InspectRenderLayerGeneralSettings(render_layer);
      ImGui::EndTabItem();
    }
    if (!render_layer.force_ddgi_inspection_layout) {
      draw_ddgi_tab();
    }
    if (ImGui::BeginTabItem("Shadow")) {
      InspectShadowSettings(render_layer.render_settings);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Strands")) {
      InspectStrandsSettings(render_layer.render_settings);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  ImGui::End();
  render_layer.enable_inspection = open;
  return false;
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
    const char* environment_types[]{"Environmental Map", "Color"};
    static int type = static_cast<int>(scene.environment.environment_type);
    if (ImGui::Combo("Environment type", &type, environment_types, IM_ARRAYSIZE(environment_types))) {
      scene.environment.environment_type = static_cast<Scene::EnvironmentType>(type);
      modified = true;
    }
    switch (scene.environment.environment_type) {
      case Scene::EnvironmentType::EnvironmentalMap: {
        if (editor_layer->DragAndDropButton<EnvironmentalMap>(scene.environment.environmental_map, "Environmental Map"))
          modified = true;
      } break;
      case Scene::EnvironmentType::Color: {
        if (ImGui::ColorEdit3("Background Color", &scene.environment.background_color.x))
          modified = true;
      } break;
    }
    if (ImGui::DragFloat("Environmental light intensity", &scene.environment.ambient_light_intensity, 0.01f, 0.0f,
                         10.0f))
      modified = true;
    if (ImGui::DragFloat("Environmental light gamma", &scene.environment.environment_gamma, 0.01f, 0.0f, 10.0f)) {
      modified = true;
    }
    if (ImGui::TreeNodeEx("Volumetric clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (InspectVolumetricCloudSettings(scene.environment.volumetric_cloud_settings))
        modified = true;
      ImGui::TreePop();
    }
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
  if (ImGui::DragFloat("Bias", &light.bias, 0.001f, 0.0f, 999.0f))
    changed = false;
  if (ImGui::DragFloat("Normal Offset", &light.normal_offset, 0.001f, 0.0f, 999.0f))
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

bool InspectReflectionProbe(InspectorContext&, ReflectionProbe& reflection_probe) {
  const auto cubemap = reflection_probe.GetCubemap();
  return cubemap ? InspectCubemapPreviewFaces(*cubemap) : false;
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
  if (editor_layer->DragAndDropButton<LightProbe>(environmental_map.reflection_probe, "ReflectionProbe"))
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

bool InspectDdgiVolume(InspectorContext& context, DdgiVolume& volume) {
  const auto& editor_layer = context.editor_layer;
  if (!editor_layer) {
    return false;
  }

  volume.ClampSettings();
  bool changed = false;
  if (ImGui::DragInt3("Probe counts##DdgiVolume", &volume.probe_counts.x, 1.0f, 1, 256))
    changed = true;
  if (ImGui::DragFloat3("Probe spacing##DdgiVolume", &volume.probe_spacing.x, 0.05f, 0.05f, 10000.0f))
    changed = true;
  if (ImGui::DragFloat3("Volume origin##DdgiVolume", &volume.volume_origin.x, 0.1f))
    changed = true;
  const char* movement_types[] = {"Default", "Scrolling"};
  if (ImGui::Combo("Movement type##DdgiVolume", &volume.movement_type, movement_types, IM_ARRAYSIZE(movement_types)))
    changed = true;

  if (ImGui::TreeNodeEx("Probe update##DdgiVolume", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Checkbox("Probe relocation##DdgiVolume", &volume.enable_probe_relocation))
      changed = true;
    if (ImGui::Checkbox("Probe classification##DdgiVolume", &volume.enable_probe_classification))
      changed = true;
    if (ImGui::Checkbox("Probe variability##DdgiVolume", &volume.enable_probe_variability))
      changed = true;
    if (ImGui::Checkbox("Variability update gating##DdgiVolume", &volume.enable_probe_variability_gating))
      changed = true;
    if (ImGui::DragFloat("Relocation distance##DdgiVolume", &volume.relocation_distance, 0.01f, 0.0f, 10000.0f))
      changed = true;
    if (ImGui::DragFloat("Random-ray backface threshold##DdgiVolume", &volume.random_ray_backface_threshold, 0.001f,
                         0.0f, 1.0f, "%.3f"))
      changed = true;
    if (ImGui::DragFloat("Fixed-ray backface threshold##DdgiVolume", &volume.fixed_ray_backface_threshold, 0.001f, 0.0f,
                         1.0f, "%.3f"))
      changed = true;
    if (ImGui::DragFloat("Variability threshold##DdgiVolume", &volume.probe_variability_threshold, 0.001f, 0.0f, 10.0f,
                         "%.3f"))
      changed = true;
    if (ImGui::DragInt("Variability min samples##DdgiVolume", &volume.probe_variability_min_samples, 1.0f, 0, 4096))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Update triggers##DdgiVolume", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextUnformatted("Warm-up triggers");
    if (DrawDdgiVolumeTriggerConditionCheckbox("Light enable/disable##DdgiVolumeWarmup",
                                               volume.warmup_trigger_conditions,
                                               DdgiVolumeTriggerConditionLightEnableChanged))
      changed = true;
    if (DrawDdgiVolumeTriggerConditionCheckbox("Lighting condition##DdgiVolumeWarmup", volume.warmup_trigger_conditions,
                                               DdgiVolumeTriggerConditionLightingConditionChanged))
      changed = true;
    if (DrawDdgiVolumeTriggerConditionCheckbox("Geometry##DdgiVolumeWarmup", volume.warmup_trigger_conditions,
                                               DdgiVolumeTriggerConditionGeometryChanged))
      changed = true;
    ImGui::TextUnformatted("Variability reset triggers");
    if (DrawDdgiVolumeTriggerConditionCheckbox("Light enable/disable##DdgiVolumeVariability",
                                               volume.variability_reset_trigger_conditions,
                                               DdgiVolumeTriggerConditionLightEnableChanged))
      changed = true;
    if (DrawDdgiVolumeTriggerConditionCheckbox("Lighting condition##DdgiVolumeVariability",
                                               volume.variability_reset_trigger_conditions,
                                               DdgiVolumeTriggerConditionLightingConditionChanged))
      changed = true;
    if (DrawDdgiVolumeTriggerConditionCheckbox("Geometry##DdgiVolumeVariability",
                                               volume.variability_reset_trigger_conditions,
                                               DdgiVolumeTriggerConditionGeometryChanged))
      changed = true;
    ImGui::TreePop();
  }

  ImGui::Text("Probe amount: %u", volume.GetProbeAmount());

  if (changed) {
    volume.ClampSettings();
  }
  return changed;
}

bool InspectMaterialTextureSlot(const std::shared_ptr<EditorLayer>& editor_layer, Material& material,
                                const std::shared_ptr<Texture2D>& current_texture, const char* label,
                                const MaterialTextureSetter setter) {
  AssetRef texture_ref(current_texture);
  if (editor_layer->DragAndDropButton<Texture2D>(texture_ref, label)) {
    (material.*setter)(texture_ref.Get<Texture2D>());
    return true;
  }
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
    if (ImGui::ColorEdit3("Albedo##Material", &material.material_properties.albedo_color.x)) {
      changed = true;
    }
    if (ImGui::DragFloat("Subsurface##Material", &material.material_properties.subsurface_factor, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (material.material_properties.subsurface_factor > 0.0f) {
      if (ImGui::DragFloat3("Subsurface Radius##Material", &material.material_properties.subsurface_radius.x, 0.01f,
                            0.0f, 999.0f)) {
        changed = true;
      }
      if (ImGui::ColorEdit3("Subsurface Color##Material", &material.material_properties.subsurface_color.x)) {
        changed = true;
      }
    }
    if (ImGui::DragFloat("Metallic##Material", &material.material_properties.metallic, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Specular##Material", &material.material_properties.specular, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Specular Tint##Material", &material.material_properties.specular_tint, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Roughness##Material", &material.material_properties.roughness, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Sheen##Material", &material.material_properties.sheen, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Sheen Tint##Material", &material.material_properties.sheen_tint, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Clear Coat##Material", &material.material_properties.clear_coat, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Clear Coat Roughness##Material", &material.material_properties.clear_coat_roughness, 0.01f,
                         0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("IOR##Material", &material.material_properties.ior, 0.01f, 0.0f, 5.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Transmission##Material", &material.material_properties.transmission, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Transmission Roughness##Material", &material.material_properties.transmission_roughness,
                         0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Emission##Material", &material.material_properties.emission, 0.01f, 0.0f, 10.0f)) {
      changed = true;
    }

    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Others##Material")) {
    if (InspectDrawSettings(material.draw_settings)) {
      changed = true;
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Textures##Material", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = InspectMaterialTextureSlot(editor_layer, material, material.GetAlbedoTexture(), "Albedo Tex",
                                         &Material::SetAlbedoTexture) ||
              changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, material.GetNormalTexture(), "Normal Tex",
                                         &Material::SetNormalTexture) ||
              changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, material.GetMetallicTexture(), "Metallic Tex",
                                         &Material::SetMetallicTexture) ||
              changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, material.GetRoughnessTexture(), "Roughness Tex",
                                         &Material::SetRoughnessTexture) ||
              changed;
    changed = InspectMaterialTextureSlot(editor_layer, material, material.GetAoTexture(), "AO Tex",
                                         &Material::SetAoTexture) ||
              changed;

    AssetRef rma_texture_ref;
    if (editor_layer->DragAndDropButton<Texture2D>(rma_texture_ref, "Apply RMA Texture")) {
      if (const auto rma_texture = rma_texture_ref.Get<Texture2D>()) {
        std::vector<glm::vec3> rma_data;
        rma_texture->GetRgbChannelData(rma_data);
        const auto rma_resolution = rma_texture->GetResolution();
        std::vector<glm::vec3> temp_data(rma_data.size());
        Jobs::RunParallelFor(temp_data.size(), [&](const size_t pixel_index) {
          temp_data[pixel_index] = glm::vec3(rma_data[pixel_index].x);
        });
        const auto roughness_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
        roughness_texture->SetRgbChannelData(temp_data, rma_resolution);

        Jobs::RunParallelFor(temp_data.size(), [&](const size_t pixel_index) {
          temp_data[pixel_index] = glm::vec3(rma_data[pixel_index].y);
        });
        const auto metallic_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
        metallic_texture->SetRgbChannelData(temp_data, rma_resolution);

        Jobs::RunParallelFor(temp_data.size(), [&](const size_t pixel_index) {
          temp_data[pixel_index] = glm::vec3(rma_data[pixel_index].z);
        });
        const auto ao_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
        ao_texture->SetRgbChannelData(temp_data, rma_resolution);

        material.SetRoughnessTexture(roughness_texture);
        material.SetMetallicTexture(metallic_texture);
        material.SetAoTexture(ao_texture);
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
  if (renderer.mesh.Get<Mesh>()) {
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
  if (renderer.skinned_mesh.Get<SkinnedMesh>()) {
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
  if (ImGui::TreeNodeEx("Normal", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Image(camera.GetGBufferNormalImTextureId(), image_size, ImVec2(0, 1), ImVec2(1, 0));
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("UV", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Image(camera.GetGBufferMaterialTexCoordImTextureId(), image_size, ImVec2(0, 1), ImVec2(1, 0));
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Instance/Material Index")) {
    ImGui::Image(camera.GetGBufferMaterialIndicesImTextureId(), image_size, ImVec2(0, 1), ImVec2(1, 0));
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
  InspectorRegistry::GetInstance().RegisterInspector<DdgiVolume>(InspectDdgiVolume, {}, "DdgiVolume");
  InspectorRegistry::GetInstance().RegisterInspector<DirectionalLight>(InspectDirectionalLight, {}, "DirectionalLight");
  InspectorRegistry::GetInstance().RegisterInspector<EnvironmentalMap>(InspectEnvironmentalMap, {}, "EnvironmentalMap");
  InspectorRegistry::GetInstance().RegisterInspector<EditorLayer>(InspectEditorLayer, {}, "EditorLayer");
  InspectorRegistry::GetInstance().RegisterInspector<LightProbe>(InspectLightProbe, {}, "LightProbe");
  InspectorRegistry::GetInstance().RegisterInspector<Mesh>(InspectMesh, {}, "Mesh");
  InspectorRegistry::GetInstance().RegisterInspector<MeshRenderer>(InspectMeshRenderer, {}, "MeshRenderer");
  InspectorRegistry::GetInstance().RegisterInspector<Particles>(InspectParticles, {}, "Particles");
  InspectorRegistry::GetInstance().RegisterInspector<PointCloud>(InspectPointCloud, {}, "PointCloud");
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
  InspectorRegistry::GetInstance().RegisterInspector<ReflectionProbe>(InspectReflectionProbe, {}, "ReflectionProbe");
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
