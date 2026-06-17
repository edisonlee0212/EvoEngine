#include "LeafIlluminationEstimator.hpp"

#ifdef CUDA_MODULE_SERVICE
#  include "BtfMeshRenderer.hpp"
#  include "EditorLayer.hpp"
#  include "MeshRenderer.hpp"
#  include "Platform.hpp"
#  include "Scene.hpp"
#  include "Sorghum.hpp"
#  include "SorghumLayer.hpp"
#  include "TriangleIlluminationEstimator.hpp"
using namespace digital_agriculture_package;
using namespace evo_engine;

namespace {
bool IsLeafMeshEntity(const std::shared_ptr<Scene>& scene, const Entity& entity) {
  return scene->GetEntityName(entity) == "Leaf Mesh";
}

glm::vec3 CalculateProbeCenter(const LightProbeGroup& light_probe_group) {
  if (light_probe_group.light_probes.empty())
    return glm::vec3(0.0f);
  glm::vec3 center_sum = glm::vec3(0.0f);
  float area_sum = 0.0f;
  for (const auto& probe : light_probe_group.light_probes) {
    const auto area = probe.GetArea();
    center_sum += probe.GetCenter() * area;
    area_sum += area;
  }
  if (area_sum == 0.0f)
    return light_probe_group.light_probes.front().GetCenter();
  return center_sum / area_sum;
}

void AppendLightProbes(LightProbeGroup& target, const LightProbeGroup& source) {
  target.light_probes.insert(target.light_probes.end(), source.light_probes.begin(), source.light_probes.end());
}

void RegenerateSeparatedLeafMeshes(const std::shared_ptr<Scene>& scene, const Entity& owner) {
  if (!scene->HasPrivateComponent<Sorghum>(owner))
    return;
  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  if (!sorghum_layer)
    return;
  auto settings = sorghum_layer->sorghum_mesh_generator_settings;
  settings.enable_leaves = true;
  settings.leaf_separated = true;
  settings.single_leaf_index = -1;
  sorghum_layer->sorghum_mesh_generator_settings.enable_leaves = true;
  sorghum_layer->sorghum_mesh_generator_settings.leaf_separated = true;
  sorghum_layer->sorghum_mesh_generator_settings.single_leaf_index = -1;
  scene->GetOrSetPrivateComponent<Sorghum>(owner).lock()->GenerateGeometryEntities(settings);
}
}  // namespace

bool LeafIlluminationEstimator::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  const auto owner = GetOwner();
  light_probe_group_.DrawGui();
  static int seed = 0;
  static float push_normal_distance = 0.001f;
  static RayProperties ray_properties;
  if (ImGui::DragInt("Seed", &seed))
    changed = true;
  if (ImGui::DragFloat("Normal Distance", &push_normal_distance, 0.0001f, -1.0f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Samples", &ray_properties.samples))
    changed = true;
  if (ImGui::DragInt("Bounces", &ray_properties.bounces))
    changed = true;
  if (ImGui::Button("Estimate")) {
    PrepareLightProbeGroup();
    SampleLightProbeGroup(ray_properties, seed, push_normal_distance);
    changed = true;
  }
  if (ImGui::TreeNode("Details")) {
    if (ImGui::Button("Prepare light probe group")) {
      PrepareLightProbeGroup();
    }
    if (ImGui::Button("Sample light probe group")) {
      SampleLightProbeGroup(ray_properties, seed, push_normal_distance);
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaves", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("%s", ("Leaf count: " + std::to_string(leaf_illumination_infos_.size())).c_str());
    ImGui::Text("%s", ("Probe count: " + std::to_string(light_probe_group_.light_probes.size())).c_str());
    for (size_t leaf_index = 0; leaf_index < leaf_illumination_infos_.size(); leaf_index++) {
      const auto& leaf_info = leaf_illumination_infos_[leaf_index];
      ImGui::Text("Leaf %zu: avg %.6f, total %.6f, area %.6f", leaf_index,
                  glm::length(leaf_info.average_illumination), glm::length(leaf_info.total_illumination),
                  leaf_info.area);
      const auto label = "Leaf " + std::to_string(leaf_index) + "##LeafIllumination" + std::to_string(leaf_index);
      if (ImGui::TreeNode(label.c_str())) {
        ImGui::Text("%s",
                    ("Entity: " + leaf_info.name + " (" + std::to_string(leaf_info.entity.GetIndex()) + ")").c_str());
        ImGui::Text("%s", ("Probes: " + std::to_string(leaf_info.probe_count)).c_str());
        ImGui::Text("%s", ("Area: " + std::to_string(leaf_info.area)).c_str());
        ImGui::Text("%s", ("Total illumination: " + std::to_string(glm::length(leaf_info.total_illumination))).c_str());
        ImGui::Text("%s",
                    ("Average illumination: " + std::to_string(glm::length(leaf_info.average_illumination))).c_str());
        ImGui::Text("Average RGB: %.6f, %.6f, %.6f", leaf_info.average_illumination.x,
                    leaf_info.average_illumination.y, leaf_info.average_illumination.z);
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }

  static bool show_leaf_gizmos = true;
  ImGui::Checkbox("Show leaf illumination", &show_leaf_gizmos);
  if (show_leaf_gizmos) {
    static float node_render_size = .02f;
    static float energy_scale_factor = 1.f;
    if (ImGui::TreeNode("Debug settings")) {
      ImGui::DragFloat("Leaf marker size", &node_render_size, 0.001f, 0.0f, 1.f);
      ImGui::DragFloat("Energy scale factor", &energy_scale_factor, 0.01f, 0.0f, 100.f);
      ImGui::TreePop();
    }
    static Entity previous_referenced_entity;
    static std::shared_ptr<ParticleInfoList> leaf_debug_info_list;
    if (!leaf_debug_info_list)
      leaf_debug_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
    if (ImGui::Button("Refresh leaf debug info") || previous_referenced_entity != owner) {
      previous_referenced_entity = owner;
      std::vector<ParticleInfo> particle_infos;
      particle_infos.resize(leaf_illumination_infos_.size());
      for (size_t i = 0; i < leaf_illumination_infos_.size(); i++) {
        const auto& leaf_info = leaf_illumination_infos_.at(i);
        auto& matrix = particle_infos[i].instance_matrix;
        matrix.value = glm::translate(leaf_info.center) * glm::scale(glm::vec3(node_render_size));
        particle_infos[i].instance_color = glm::vec4(leaf_info.average_illumination * energy_scale_factor, 1.0f);
      }
      leaf_debug_info_list->SetParticleInfos(particle_infos);
    }
    GizmoSettings gizmo_settings{};
    gizmo_settings.depth_test = false;
    editor_layer->DrawGizmoCubes(leaf_debug_info_list, glm::mat4(1), 1, gizmo_settings);
  }

  return changed;
}

const LightProbeGroup& LeafIlluminationEstimator::PeekProbes() const {
  return light_probe_group_;
}

const std::vector<LeafIlluminationEstimator::LeafIlluminationInfo>&
LeafIlluminationEstimator::PeekLeafIlluminationInfos() const {
  return leaf_illumination_infos_;
}

void LeafIlluminationEstimator::SampleLightProbeGroup(const RayProperties& ray_properties, int seed,
                                                      float push_normal_distance) {
  light_probe_group_.light_probes.clear();
  const auto scene = GetScene();
  for (auto& leaf_info : leaf_illumination_infos_) {
    if (!scene->HasPrivateComponent<TriangleIlluminationEstimator>(leaf_info.entity))
      continue;
    const auto triangle_estimator = scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(leaf_info.entity).lock();
    triangle_estimator->SampleLightProbeGroup(ray_properties, seed, push_normal_distance);
    leaf_info.probe_offset = light_probe_group_.light_probes.size();
    leaf_info.probe_count = triangle_estimator->PeekProbes().light_probes.size();
    leaf_info.area = triangle_estimator->total_area;
    leaf_info.total_illumination = triangle_estimator->total_flux;
    leaf_info.average_illumination = triangle_estimator->average_flux;
    leaf_info.center = CalculateProbeCenter(triangle_estimator->PeekProbes());
    AppendLightProbes(light_probe_group_, triangle_estimator->PeekProbes());
  }
}

void LeafIlluminationEstimator::PrepareLightProbeGroup() {
  light_probe_group_.light_probes.clear();
  leaf_illumination_infos_.clear();
  const auto scene = GetScene();
  RegenerateSeparatedLeafMeshes(scene, GetOwner());
  auto entities = scene->GetDescendants(GetOwner());
  entities.push_back(GetOwner());
  for (const auto& entity : entities) {
    if (!IsLeafMeshEntity(scene, entity))
      continue;
    if (!scene->HasPrivateComponent<MeshRenderer>(entity) && !scene->HasPrivateComponent<BtfMeshRenderer>(entity))
      continue;

    const auto triangle_estimator = scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(entity).lock();
    triangle_estimator->PrepareLightProbeGroup();
    const auto& triangle_probes = triangle_estimator->PeekProbes();
    if (triangle_probes.light_probes.empty())
      continue;

    LeafIlluminationInfo leaf_info;
    leaf_info.entity = entity;
    leaf_info.name = scene->GetEntityName(entity);
    leaf_info.probe_offset = light_probe_group_.light_probes.size();
    leaf_info.probe_count = triangle_probes.light_probes.size();
    leaf_info.area = triangle_estimator->total_area;
    leaf_info.total_illumination = triangle_estimator->total_flux;
    leaf_info.average_illumination = triangle_estimator->average_flux;
    leaf_info.center = CalculateProbeCenter(triangle_probes);
    AppendLightProbes(light_probe_group_, triangle_probes);
    leaf_illumination_infos_.emplace_back(leaf_info);
  }
}

#endif
