#include "BasicFoliageDescriptor.hpp"
#include "ShootModel.hpp"
using namespace eco_sys_lab_package;

void BasicFoliageDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "leaf_size" << YAML::Value << leaf_size;
  out << YAML::Key << "leaf_count" << YAML::Value << leaf_count;
  stem_length.Save("stem_length", out);
  branching_angle.Save("branching_angle", out);
  out << YAML::Key << "rotation_variance" << YAML::Value << rotation_variance;
  out << YAML::Key << "max_node_thickness" << YAML::Value << max_node_thickness;
  out << YAML::Key << "min_root_distance" << YAML::Value << min_root_distance;
  out << YAML::Key << "max_end_distance" << YAML::Value << max_end_distance;
  out << YAML::Key << "horizontal_tropism" << YAML::Value << horizontal_tropism;
  out << YAML::Key << "gravitropism" << YAML::Value << gravitropism;
  leaf_material_ref.Save("leaf_material_ref", out);
}

void BasicFoliageDescriptor::Deserialize(const YAML::Node& in) {
  if (in["leaf_size"])
    leaf_size = in["leaf_size"].as<glm::vec2>();
  if (in["leaf_count"])
    leaf_count = in["leaf_count"].as<int>();
  stem_length.Load("stem_length", in);
  if (in["rotation_variance"])
    rotation_variance = in["rotation_variance"].as<float>();
  branching_angle.Load("branching_angle", in);
  if (in["max_node_thickness"])
    max_node_thickness = in["max_node_thickness"].as<float>();
  if (in["min_root_distance"])
    min_root_distance = in["min_root_distance"].as<float>();
  if (in["max_end_distance"])
    max_end_distance = in["max_end_distance"].as<float>();
  if (in["horizontal_tropism"])
    horizontal_tropism = in["horizontal_tropism"].as<float>();
  if (in["gravitropism"])
    gravitropism = in["gravitropism"].as<float>();
  leaf_material_ref.Load("leaf_material_ref", in);
}

bool BasicFoliageDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  changed = activation_temperature.OnInspect("Activation temperature") | changed;
  changed = activation_light_intensity.OnInspect("Activation light intensity") | changed;
  changed = growth_rate.OnInspect("Growth rate") | changed;
  changed = damage_temperature.OnInspect("Damage temperature") | changed;
  changed = damage_rate.OnInspect("Damage rate") | changed;
  changed = hang_time.OnInspect("Hang time") | changed;

  if (ImGui::DragFloat2("Leaf size", &leaf_size.x, 0.001f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Leaf per node", &leaf_count, 1, 0, 50))
    changed = true;
  changed = stem_length.OnInspect("Stem length") | changed;
  if (ImGui::DragFloat("Rotation variance", &rotation_variance, 0.01f, 0.0f, 1.0f))
    changed = true;
  changed = branching_angle.OnInspect("Branching angle") | changed;
  if (ImGui::DragFloat("Max node thickness", &max_node_thickness, 0.001f, 0.0f, 5.0f))
    changed = true;
  if (ImGui::DragFloat("Min root distance", &min_root_distance, 0.01f, 0.0f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Max end distance", &max_end_distance, 0.01f, 0.0f, 10.0f))
    changed = true;

  changed = ImGui::DragFloat("Horizontal Tropism", &horizontal_tropism, 0.001f, 0.0f, 1.0f) || changed;
  changed = ImGui::DragFloat("Gravitropism", &gravitropism, 0.001f, 0.0f, 1.0f) || changed;
  if (editor_layer->DragAndDropButton<Material>(leaf_material_ref, "Leaf Material"))
    changed = true;
  return changed;
}

void BasicFoliageDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (leaf_material_ref.Get<Material>())
    list.push_back(leaf_material_ref);
}

void BasicFoliageDescriptor::PrepareController(FoliageController& foliage_controller) const {
  foliage_controller.leaf_count = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                      const SkeletonNode<InternodeGrowthData>& internode) {
    return leaf_count;
  };

  foliage_controller.leaf_formulation = [&](std::mt19937& random_engine, const glm::mat4& global_transform, Leaf& leaf,
                                            const ClimateModel& climate_model, const ShootSkeleton& shoot_skeleton,
                                            const SkeletonNode<InternodeGrowthData>& internode) {
    const bool activation = internode.info.end_distance < max_end_distance &&
                            internode.info.root_distance > min_root_distance &&
                            internode.info.thickness < max_node_thickness;
    if (activation) {
      leaf.activation_temperature = activation_temperature.GetValue();
      leaf.activation_light_intensity = activation_light_intensity.GetValue();
      leaf.hang_time = hang_time.GetValue();
      leaf.growth_rate = growth_rate.GetValue();

      leaf.damage_temperature = damage_temperature.GetValue();
      leaf.damage_rate = damage_rate.GetValue();

      leaf.rotation = internode.info.global_rotation *
                      glm::quat(glm::radians(glm::vec3(glm::gaussRand(0.0f, rotation_variance),
                                                       branching_angle.GetValue(), glm::linearRand(0.0f, 360.0f))));
      auto front = leaf.rotation * glm::vec3(0, 0, -1);
      auto up = leaf.rotation * glm::vec3(0, 1, 0);
      ShootModel::ApplyTropism(glm::vec3(0, -1, 0), gravitropism, front, up);
      if (const auto horizontal_direction = glm::vec3(front.x, 0.0f, front.z);
          glm::length(horizontal_direction) > glm::epsilon<float>()) {
        ShootModel::ApplyTropism(glm::normalize(horizontal_direction), horizontal_tropism, front, up);
      }
      leaf.rotation = glm::quatLookAt(front, up);

      leaf.position_offset =
          glm::mix(glm::vec3(0.f), internode.info.GetGlobalEndPosition() - internode.info.global_position,
                   glm::linearRand(0.f, 1.f));
      leaf.leaf_stem_length = glm::abs(stem_length.GetValue());
    }

    return activation;
  };
  foliage_controller.leaf_growth = [&](std::mt19937& random_engine, const glm::mat4& global_transform,
                                       const float delta_time, Leaf& leaf, const ClimateModel& climate_model,
                                       const ShootSkeleton& shoot_skeleton,
                                       const SkeletonNode<InternodeGrowthData>& internode) {
    bool status_changed = false;
    // If leaf is not active, try to activate in the first place.
    const glm::vec3 position = glm::vec3(global_transform[3]) + internode.info.global_position;
    const auto temperature = climate_model.GetHighTemp(position);
    const auto light_intensity = internode.data.light_intake;
    if (leaf.status == OrganStatus::Dormant) {
      if (temperature >= leaf.activation_temperature && light_intensity >= leaf.activation_light_intensity) {
        leaf.status = OrganStatus::Flushed;
        leaf.maturity = 0.0f;
        leaf.health = 1.f;

        status_changed = true;
      }
    } else if (leaf.status == OrganStatus::Flushed) {
      leaf.maturity = glm::clamp(leaf.growth_rate * delta_time + leaf.maturity, 0.0f, 1.0f);
      if (climate_model.GetTimeInYear() > 0.75f && temperature < leaf.damage_temperature) {
        leaf.health = glm::clamp(leaf.health - leaf.damage_rate * delta_time, 0.0f, 1.0f);
      }
    }

    const auto current_leaf_size = leaf_size * leaf.maturity;
    const auto front = leaf.rotation * glm::vec3(0, 0, -1);
    const auto up = leaf.rotation * glm::vec3(0, 1, 0);

    leaf.position = internode.info.global_position + leaf.position_offset +
                    front * current_leaf_size.y * (1.f + leaf.leaf_stem_length);

    leaf.scale = glm::vec3(current_leaf_size.x, 1.0f, current_leaf_size.y);
    if (glm::any(glm::isnan(leaf.position)) || glm::any(glm::isnan(front)) || glm::any(glm::isnan(up))) {
      leaf.position = glm::vec3(0.f);
      leaf.scale = glm::vec3(0.f);
    }
    return status_changed;
  };
}

void BasicFoliageDescriptor::GenerateFoliageMatrices(std::vector<glm::mat4>& matrices,
                                                     const SkeletonNodeInfo& internode_info,
                                                     const float tree_size) const {
  if (internode_info.thickness <= max_node_thickness && internode_info.root_distance >= min_root_distance &&
      internode_info.end_distance <= max_end_distance) {
    for (int i = 0; i < leaf_count * internode_info.leaves; i++) {
      const auto current_leaf_size = leaf_size * tree_size * 0.1f;
      glm::quat rotation =
          internode_info.global_rotation *
          glm::quat(glm::radians(glm::vec3(glm::gaussRand(0.0f, rotation_variance), branching_angle.GetValue(),
                                           glm::linearRand(0.0f, 360.0f))));
      auto front = rotation * glm::vec3(0, 0, -1);
      auto up = rotation * glm::vec3(0, 1, 0);
      ShootModel::ApplyTropism(glm::vec3(0, -1, 0), gravitropism, front, up);
      if (const auto horizontal_direction = glm::vec3(front.x, 0.0f, front.z);
          glm::length(horizontal_direction) > glm::epsilon<float>()) {
        ShootModel::ApplyTropism(glm::normalize(horizontal_direction), horizontal_tropism, front, up);
      }
      auto foliage_position =
          glm::mix(internode_info.global_position, internode_info.GetGlobalEndPosition(), glm::linearRand(0.f, 1.f)) +
          front * (current_leaf_size.y + stem_length.GetValue() * 0.1f);
      if (glm::any(glm::isnan(foliage_position)) || glm::any(glm::isnan(front)) || glm::any(glm::isnan(up)))
        continue;
      const auto leaf_transform = glm::translate(foliage_position) * glm::mat4_cast(glm::quatLookAt(front, up)) *
                                  glm::scale(glm::vec3(current_leaf_size.x, 1.0f, current_leaf_size.y));
      matrices.emplace_back(leaf_transform);
    }
  }
}