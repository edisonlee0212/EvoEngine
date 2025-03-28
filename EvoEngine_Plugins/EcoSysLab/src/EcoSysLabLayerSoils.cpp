//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"
#ifdef CUDA_MODULE_PLUGIN
#  include <RayTracerLayer.hpp>
#endif

#include "ClassRegistry.hpp"
#include "DynamicTreeStrands.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

AssetRegistration<HeightField> height_field_registry("HeightField", {".heightfield"});
AssetRegistration<SoilLayerDescriptor> soil_layer_d_registry("SoilLayerDescriptor", {".soillayer"});

PrivateComponentRegistration<Soil> soil_registry("Soil");

AssetRegistration<SoilDescriptor> soil_d_registry("SoilDescriptor", {".soil"});
bool EcoSysLabLayer::SoilVisualizationSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Checkbox("Enable", &enable);
  static bool force_update;
  ImGui::Checkbox("Force Update", &force_update);

  if (ImGui::Checkbox("Vector Visualization", &vector_enable)) {
    if (vector_enable)
      update_vector_matrices = true;
  }

  if (ImGui::Checkbox("Scalar Visualization", &scalar_enable)) {
    if (scalar_enable)
      update_scalar_matrices = true;
  }

  if (vector_enable) {
    update_vector_matrices = update_vector_matrices || force_update;
    if (ImGui::TreeNodeEx("Vector", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Reset")) {
        vector_multiplier = 50.0f;
        vector_base_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.8f);
        vector_soil_property = 4;
        vector_line_width_factor = 0.1f;
        vector_line_max_width = 0.1f;
        update_vector_matrices = true;
      }
      if (ImGui::ColorEdit4("Vector Base Color", &vector_base_color.x)) {
        update_vector_matrices = true;
      }
      if (ImGui::DragFloat("Multiplier", &vector_multiplier, 0.1f, 0.0f, 100.0f, "%.3f")) {
        update_vector_matrices = true;
      }
      if (ImGui::DragFloat("Line Width Factor", &vector_line_width_factor, 0.01f, 0.0f, 5.0f)) {
        update_vector_matrices = true;
      }
      if (ImGui::DragFloat("Max Line Width", &vector_line_max_width, 0.01f, 0.0f, 5.0f)) {
        update_vector_matrices = true;
      }
      if (ImGui::Combo("Vector Mode",
                       {"N/A", "N/A", "Water Density Gradient", "Flux", "Divergence", "N/A", "N/A", "N/A"},
                       vector_soil_property)) {
        update_vector_matrices = true;
      }
      ImGui::TreePop();
    }
  }
  if (scalar_enable) {
    update_scalar_matrices = update_scalar_matrices || force_update;

    if (scalar_enable && ImGui::TreeNodeEx("Scalar", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Reset")) {
        scalar_multiplier = 1.0f;
        scalar_box_size = 0.5f;
        scalar_min_alpha = 0.00f;
        scalar_base_color = glm::vec3(0.0f, 0.0f, 1.0f);
        scalar_soil_property = 1;
        update_scalar_matrices = true;
      }
      if (ImGui::SliderFloat("X Depth", &soil_cutout_x_depth, 0.0f, 1.0f)) {
        update_scalar_matrices = true;
      }
      if (ImGui::SliderFloat("Z Depth", &soil_cutout_z_depth, 0.0f, 1.0f)) {
        update_scalar_matrices = true;
      }

      if (ImGui::TreeNodeEx("Layer colors", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (int i = 0; i < 10; i++) {
          ImGui::ColorEdit4(("Layer " + std::to_string(i)).c_str(), &soil_layer_colors[i].x);
        }
        ImGui::TreePop();
      }

      if (ImGui::ColorEdit3("Scalar Base Color", &scalar_base_color.x)) {
        update_scalar_matrices = true;
      }
      if (ImGui::SliderFloat("Multiplier", &scalar_multiplier, 0.001, 10000, "%.4f", ImGuiSliderFlags_Logarithmic)) {
        update_scalar_matrices = true;
      }
      if (ImGui::DragFloat("Min alpha", &scalar_min_alpha, 0.001f, 0.0f, 1.0f)) {
        update_scalar_matrices = true;
      }
      if (ImGui::DragFloat("Box size", &scalar_box_size, 0.001f, 0.0f, 1.0f)) {
        update_scalar_matrices = true;
      }
      // disable less useful visualizations to avoid clutter in the gui
      if (ImGui::Combo(
              "Scalar Mode",
              {"Blank", "Water Density", "N/A", "N/A", "N/A", "Nutrient Density", "Soil Density", "Soil Layer"},
              scalar_soil_property)) {
        update_scalar_matrices = true;
      }
      ImGui::TreePop();
    }
  }
  return changed;
}

void EcoSysLabLayer::SoilVisualization() {
  std::shared_ptr<Soil> soil;
  if (const auto soil_candidate = FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();

  if (!soil)
    return;

  const auto& soil_model = soil->soil_model;
  if (soil_version_ != soil_model.m_version) {
    soil_visualization_settings_.update_vector_matrices = true;
    soil_visualization_settings_.update_scalar_matrices = true;
    soil_version_ = soil_model.m_version;
  }

  if (soil_visualization_settings_.vector_enable) {
    SoilVisualizationVector(soil_model);
  }
  if (soil_visualization_settings_.scalar_enable) {
    SoilVisualizationScalar(soil_model);
  }
}

void EcoSysLabLayer::SoilVisualizationScalar(const VoxelSoilModel& soil_model) {
  const auto num_voxels = soil_model.m_resolution.x * soil_model.m_resolution.y * soil_model.m_resolution.z;
  if (soil_visualization_settings_.update_scalar_matrices) {
    std::vector<ParticleInfo> particle_infos;
    particle_infos.resize(num_voxels);
    Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
      const auto coordinate = soil_model.GetCoordinateFromIndex(i);
      if (static_cast<float>(coordinate.x) / soil_model.m_resolution.x <
              soil_visualization_settings_.soil_cutout_x_depth ||
          static_cast<float>(coordinate.z) / soil_model.m_resolution.z >
              (1.0f - soil_visualization_settings_.soil_cutout_z_depth)) {
        particle_infos[i].instance_matrix.value = glm::mat4(0.0f);
      } else {
        particle_infos[i].instance_matrix.value =
            glm::translate(soil_model.GetPositionFromCoordinate(coordinate)) *
            glm::mat4_cast(glm::quat(glm::vec3(0.0f))) *
            glm::scale(glm::vec3(soil_model.GetVoxelSize() * soil_visualization_settings_.scalar_box_size));
      }
    });
    auto visualize_vec3 = [&](const Field& x, const Field& y, const Field& z) {
      Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
        const auto value = glm::vec3(x[i], y[i], z[i]);
        particle_infos[i].instance_color = {
            glm::normalize(value), glm::clamp(glm::length(value) * soil_visualization_settings_.scalar_multiplier,
                                              soil_visualization_settings_.scalar_min_alpha, 1.0f)};
      });
    };

    auto visualize_float = [&](const Field& v) {
      Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
        const auto value = glm::vec3(v[i]);
        particle_infos[i].instance_color = {
            soil_visualization_settings_.scalar_base_color,
            glm::clamp(glm::length(value) * soil_visualization_settings_.scalar_multiplier,
                       soil_visualization_settings_.scalar_min_alpha, 1.0f)};
      });
    };

    switch (static_cast<SoilProperty>(soil_visualization_settings_.scalar_soil_property)) {
      case SoilProperty::Blank: {
        Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
          particle_infos[i].instance_color = {soil_visualization_settings_.scalar_base_color, 0.01f};
        });
      } break;
      case SoilProperty::WaterDensity: {
        visualize_float(soil_model.m_w);
      } break;
      case SoilProperty::NutrientDensity: {
        visualize_float(soil_model.m_n);
      } break;
      case SoilProperty::SoilDensity: {
        visualize_float(soil_model.m_d);
      } break;
      case SoilProperty::SoilLayer: {
        Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
          const auto layerIndex = soil_model.m_material_id[i];
          if (layerIndex == 0)
            particle_infos[i].instance_color = glm::vec4(0.0f);
          else {
            particle_infos[i].instance_color = soil_visualization_settings_.soil_layer_colors[layerIndex - 1];
          }
        });
      } break;
        /*case SoilProperty::DiffusionDivergence:
        {
                visualize_vec3(soilModel.m_div_diff_x, soilModel.m_div_diff_y, soilModel.m_div_diff_z);
        }break;*/
      default: {
        Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
          particle_infos[i].instance_color = {soil_visualization_settings_.scalar_base_color, 0.01f};
        });
      } break;
    }
    ground_fruit_matrices_->SetParticleInfos(particle_infos);
  }
  soil_visualization_settings_.update_scalar_matrices = false;
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.blending_src_factor = VK_BLEND_FACTOR_SRC_ALPHA;
  gizmo_settings.draw_settings.blending_dst_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cube, scalar_matrices_, glm::mat4(1.0f), 1.0f,
                                              gizmo_settings);
}

void EcoSysLabLayer::SoilVisualizationVector(const VoxelSoilModel& soil_model) {
  const auto num_voxels = soil_model.m_resolution.x * soil_model.m_resolution.y * soil_model.m_resolution.z;

  if (soil_visualization_settings_.update_vector_matrices) {
    std::vector<ParticleInfo> particle_infos;
    particle_infos.resize(num_voxels);

    const auto actual_vector_multiplier = soil_visualization_settings_.vector_multiplier * soil_model.m_dx;
    switch (static_cast<SoilProperty>(soil_visualization_settings_.vector_soil_property)) {
        /*
        case SoilProperty::WaterDensityGradient:
        {
                Jobs::ParallelFor(numVoxels, [&](unsigned i)
                        {
                                const auto targetVector = glm::vec3(soilModel.m_w_grad_x[i], soilModel.m_w_grad_y[i],
        soilModel.m_w_grad_z[i]); const auto start =
        soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i)); const auto end = start + targetVector
        * actualVectorMultiplier; const auto direction = glm::normalize(end - start); glm::quat rotation =
        glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x)); rotation *=
        glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f)); const auto length = glm::distance(end, start) / 2.0f;
                                const auto width = glm::min(vector_line_max_width, length * vector_line_width_factor);
                                const auto model = glm::translate((start + end) / 2.0f) * glm::mat4_cast(rotation) *
                                        glm::scale(glm::vec3(width, length, width));
                                particleInfos[i] = model;
                        }, results);
        }break;*/
        /*
        case SoilProperty::Divergence:
        {
                Jobs::ParallelFor(numVoxels, [&](unsigned i)
                        {
                                const auto targetVector = glm::vec3(soilModel.m_div_diff_x[i],
        soilModel.m_div_diff_y[i], soilModel.m_div_diff_z[i]); const auto start =
        soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i)); const auto end = start + targetVector
        * actualVectorMultiplier; const auto direction = glm::normalize(end - start); glm::quat rotation =
        glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x)); rotation *=
        glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f)); const auto length = glm::distance(end, start) / 2.0f;
                                const auto width = glm::min(vector_line_max_width, length * vector_line_width_factor);
                                const auto model = glm::translate((start + end) / 2.0f) * glm::mat4_cast(rotation) *
                                        glm::scale(glm::vec3(width, length, width));
                                particleInfos[i] = model;
                        }, results);
        }break;
        */
      default: {
        Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
          particle_infos[i].instance_matrix.value =
              glm::translate(soil_model.GetPositionFromCoordinate(soil_model.GetCoordinateFromIndex(i))) *
              glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(0.0f));
        });
      } break;
    }
    Jobs::RunParallelFor(num_voxels, [&](unsigned i) {
      particle_infos[i].instance_color = soil_visualization_settings_.vector_base_color;
    });

    ground_fruit_matrices_->SetParticleInfos(particle_infos);
    soil_visualization_settings_.update_vector_matrices = false;
  }

  const auto editor_layer = Application::GetLayer<EditorLayer>();
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.blending = true;
  gizmo_settings.draw_settings.blending_src_factor = VK_BLEND_FACTOR_SRC_ALPHA;
  gizmo_settings.draw_settings.blending_dst_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  gizmo_settings.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;

  editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cylinder, vector_matrices_, glm::mat4(1.0f), 1.0f,
                                              gizmo_settings);
}

EcoSysLabLayer::SoilVisualizationSettings::SoilVisualizationSettings() {
  if (soil_layer_colors.empty()) {
    for (int i = 0; i < 10; i++) {
      glm::vec4 color = {glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)), 1.0f};
      soil_layer_colors.emplace_back(color);
    }
  }
}

std::weak_ptr<Soil> EcoSysLabLayer::FindSoil() {
  const auto scene = Application::GetActiveScene();
  const std::vector<Entity>* soil_entities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soil_entities && !soil_entities->empty()) {
    return scene->GetOrSetPrivateComponent<Soil>(soil_entities->at(0));
  }
  return {};
}