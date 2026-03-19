#include "BasicReproductionModuleDescriptor.hpp"
#include "ShootGrowthData.hpp"
#include "ShootModel.hpp"

#include <random>
using namespace eco_sys_lab_plugin;

void BasicReproductionModuleDescriptor::PrepareController(ShootReproductionController& reproduction_controller) const {
  reproduction_controller.module_count = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                             const SkeletonNode<InternodeGrowthData>& internode) {
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    if (dis(random_engine) < glm::clamp(module_spawn_chance, 0.0f, 1.0f)) {
      return count_per_internode;
    }
    return 0;
  };

  reproduction_controller.flower_formulation = [&](std::mt19937& random_engine, const glm::mat4& global_transform,
                                                   Flower& flower, const ClimateModel& climate_model,
                                                   const ShootSkeleton& shoot_skeleton,
                                                   const SkeletonNode<InternodeGrowthData>& internode) {
    const bool activation = internode.info.end_distance < max_end_distance &&
                            internode.info.root_distance > min_root_distance &&
                            internode.info.thickness < max_node_thickness;
    if (activation) {
      flower.activation_temperature = flower_activation_temperature.GetValue();
      flower.pollination_time = pollination_time.GetValue();
      flower.hang_time = flower_hang_time.GetValue();
      flower.growth_rate = flower_growth_rate.GetValue();

      flower.rotation = internode.info.global_rotation *
                        glm::quat(glm::radians(glm::vec3(glm::gaussRand(0.0f, rotation_variance),
                                                         branching_angle.GetValue(), glm::linearRand(0.0f, 360.0f))));
      auto front = flower.rotation * glm::vec3(0, 0, -1);
      auto up = flower.rotation * glm::vec3(0, 1, 0);
      ShootModel::ApplyTropism(glm::vec3(0, 1, 0), phototropism, front, up);

      flower.rotation = glm::quatLookAt(front, up);

      flower.position_offset =
          glm::mix(glm::vec3(0.f), internode.info.GetGlobalEndPosition() - internode.info.global_position,
                   glm::linearRand(0.f, 1.f));
      flower.stem_length = glm::abs(stem_length.GetValue());
    }

    return activation;
  };

  reproduction_controller.flower_growth = [&](std::mt19937& random_engine, const glm::mat4& global_transform,
                                              const float delta_time, Flower& flower, const ClimateModel& climate_model,
                                              const ShootSkeleton& shoot_skeleton,
                                              const SkeletonNode<InternodeGrowthData>& internode) {
    bool status_changed = false;
    // If flower is not active, try to activate in the first place.
    const glm::vec3 position = glm::vec3(global_transform[3]) + internode.info.global_position;
    const auto temperature = climate_model.GetHighTemp(position);
    if (flower.status == OrganStatus::Dormant) {
      if (temperature >= flower.activation_temperature) {
        flower.status = OrganStatus::Flushed;
        flower.maturity = 0.0f;
        flower.health = 1.f;

        status_changed = true;
      }
    } else if (flower.status == OrganStatus::Flushed) {
      flower.maturity = glm::clamp(flower.growth_rate * delta_time + flower.maturity, 0.0f, 1.0f);
    }

    const auto current_flower_size = flower_size * flower.maturity;
    const auto front = flower.rotation * glm::vec3(0, 0, -1);
    const auto up = flower.rotation * glm::vec3(0, 1, 0);

    flower.position = internode.info.global_position + flower.position_offset +
                      front * current_flower_size * (1.f + flower.stem_length);

    flower.scale = glm::vec3(current_flower_size);
    if (glm::any(glm::isnan(flower.position)) || glm::any(glm::isnan(front)) || glm::any(glm::isnan(up))) {
      flower.position = glm::vec3(0.f);
      flower.scale = glm::vec3(0.f);
    }
    return status_changed;
  };

  reproduction_controller.fruit_formulation = [&](std::mt19937& random_engine, const glm::mat4& global_transform,
                                                  Fruit& fruit, const ClimateModel& climate_model,
                                                  const ShootSkeleton& shoot_skeleton,
                                                  const SkeletonNode<InternodeGrowthData>& internode) {
    const bool activation = internode.info.end_distance < max_end_distance &&
                            internode.info.root_distance > min_root_distance &&
                            internode.info.thickness < max_node_thickness;
    if (activation) {
      fruit.activation_temperature = fruit_activation_temperature.GetValue();
      fruit.hang_time = fruit_hang_time.GetValue();
      fruit.growth_rate = fruit_growth_rate.GetValue();

      fruit.rotation = internode.info.global_rotation *
                       glm::quat(glm::radians(glm::vec3(glm::gaussRand(0.0f, rotation_variance),
                                                        branching_angle.GetValue(), glm::linearRand(0.0f, 360.0f))));
      auto front = fruit.rotation * glm::vec3(0, 0, -1);
      auto up = fruit.rotation * glm::vec3(0, 1, 0);
      ShootModel::ApplyTropism(glm::vec3(0, -1, 0), gravitropism, front, up);

      fruit.rotation = glm::quatLookAt(front, up);

      fruit.position_offset =
          glm::mix(glm::vec3(0.f), internode.info.GetGlobalEndPosition() - internode.info.global_position,
                   glm::linearRand(0.f, 1.f));
      fruit.carbohydrate_sink = fruit_sink_strength;
    }

    return activation;
  };

  reproduction_controller.fruit_growth = [&](std::mt19937& random_engine, const glm::mat4& global_transform,
                                             const float delta_time, Fruit& fruit, const ClimateModel& climate_model,
                                             const ShootSkeleton& shoot_skeleton,
                                             const SkeletonNode<InternodeGrowthData>& internode) {
    bool status_changed = false;
    // If fruit is not active, try to activate in the first place.
    const glm::vec3 position = glm::vec3(global_transform[3]) + internode.info.global_position;
    const auto temperature = climate_model.GetHighTemp(position);
    if (fruit.status == OrganStatus::Dormant) {
      if (temperature >= fruit.activation_temperature) {
        fruit.status = OrganStatus::Flushed;
        fruit.maturity = 0.0f;
        fruit.health = 1.f;

        status_changed = true;
      }
    } else if (fruit.status == OrganStatus::Flushed) {
      fruit.maturity = glm::clamp(fruit.growth_rate * delta_time + fruit.maturity, 0.0f, 1.0f);
    }

    const auto current_fruit_size = fruit_size * fruit.maturity;
    const auto front = fruit.rotation * glm::vec3(0, 0, -1);
    const auto up = fruit.rotation * glm::vec3(0, 1, 0);

    fruit.position =
        internode.info.global_position + fruit.position_offset + front * current_fruit_size * (1.f + fruit.stem_length);

    fruit.scale = glm::vec3(current_fruit_size);
    if (glm::any(glm::isnan(fruit.position)) || glm::any(glm::isnan(front)) || glm::any(glm::isnan(up))) {
      fruit.position = glm::vec3(0.f);
      fruit.scale = glm::vec3(0.f);
    }
    return status_changed;
  };

  reproduction_controller.calculate_flower_sink_strength =
      [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
          const SkeletonNode<InternodeGrowthData>& internode, Flower& flower) {
        flower.carbohydrate_sink = 0.0;
      };

  reproduction_controller.calculate_fruit_sink_strength =
      [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
          const SkeletonNode<InternodeGrowthData>& internode, Fruit& fruit) {
        fruit.carbohydrate_sink = fruit_sink_strength * fruit.maturity * fruit.health;
      };
}

void BasicReproductionModuleDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "flower_size" << YAML::Value << flower_size;
  out << YAML::Key << "fruit_size" << YAML::Value << fruit_size;
  out << YAML::Key << "count_per_internode" << YAML::Value << count_per_internode;
  out << YAML::Key << "module_spawn_chance" << YAML::Value << module_spawn_chance;
  stem_length.Save("stem_length", out);
  branching_angle.Save("branching_angle", out);
  out << YAML::Key << "rotation_variance" << YAML::Value << rotation_variance;
  out << YAML::Key << "max_node_thickness" << YAML::Value << max_node_thickness;
  out << YAML::Key << "min_root_distance" << YAML::Value << min_root_distance;
  out << YAML::Key << "max_end_distance" << YAML::Value << max_end_distance;
  out << YAML::Key << "phototropism" << YAML::Value << phototropism;
  out << YAML::Key << "gravitropism" << YAML::Value << gravitropism;

  flower_activation_temperature.Save("flower_activation_temperature", out);
  flower_growth_rate.Save("flower_growth_rate", out);
  flower_hang_time.Save("flower_hang_time", out);
  pollination_time.Save("pollination_time", out);
  fruit_activation_temperature.Save("fruit_activation_temperature", out);
  fruit_hang_time.Save("fruit_hang_time", out);
  fruit_growth_rate.Save("fruit_growth_rate", out);

  out << YAML::Key << "flower_sink_strength" << YAML::Value << flower_sink_strength;
  out << YAML::Key << "fruit_sink_strength" << YAML::Value << fruit_sink_strength;
}

void BasicReproductionModuleDescriptor::Deserialize(const YAML::Node& in) {
  if (in["flower_size"]) {
    flower_size = in["flower_size"].as<float>();
  }
  if (in["fruit_size"]) {
    fruit_size = in["fruit_size"].as<float>();
  }
  if (in["count_per_internode"]) {
    count_per_internode = in["count_per_internode"].as<int>();
  }
  if (in["module_spawn_chance"]) {
    module_spawn_chance = in["module_spawn_chance"].as<float>();
  }
  stem_length.Load("stem_length", in);
  if (in["rotation_variance"]) {
    rotation_variance = in["rotation_variance"].as<float>();
  }
  branching_angle.Load("branching_angle", in);
  if (in["max_node_thickness"]) {
    max_node_thickness = in["max_node_thickness"].as<float>();
  }
  if (in["min_root_distance"]) {
    min_root_distance = in["min_root_distance"].as<float>();
  }
  if (in["max_end_distance"]) {
    max_end_distance = in["max_end_distance"].as<float>();
  }
  if (in["phototropism"]) {
    phototropism = in["phototropism"].as<float>();
  }
  if (in["gravitropism"]) {
    gravitropism = in["gravitropism"].as<float>();
  }

  flower_activation_temperature.Load("flower_activation_temperature", in);
  flower_growth_rate.Load("flower_growth_rate", in);
  flower_hang_time.Load("flower_hang_time", in);
  pollination_time.Load("pollination_time", in);
  fruit_activation_temperature.Load("fruit_activation_temperature", in);
  fruit_hang_time.Load("fruit_hang_time", in);
  fruit_growth_rate.Load("fruit_growth_rate", in);

  if (in["flower_sink_strength"])
    flower_sink_strength = in["flower_sink_strength"].as<float>();
  if (in["fruit_sink_strength"])
    fruit_sink_strength = in["fruit_sink_strength"].as<float>();
}

bool BasicReproductionModuleDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::DragFloat("Flower size", &flower_size, 0.001f, 0.0f, 1.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Fruit size", &fruit_size, 0.001f, 0.0f, 1.0f)) {
    changed = true;
  }
  if (ImGui::DragInt("Count per internode (on spawn)", &count_per_internode, 1, 0, 50)) {
    changed = true;
  }
  if (ImGui::DragFloat("Flower/Fruit spawn chance", &module_spawn_chance, 0.001f, 0.0f, 1.0f)) {
    changed = true;
  }
  changed = stem_length.OnInspect("Stem length") || changed;
  if (ImGui::DragFloat("Rotation variance", &rotation_variance, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  changed = branching_angle.OnInspect("Branching angle") || changed;
  if (ImGui::DragFloat("Max node thickness", &max_node_thickness, 0.001f, 0.0f, 5.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Min root distance", &min_root_distance, 0.01f, 0.0f, 10.0f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Max end distance", &max_end_distance, 0.01f, 0.0f, 10.0f)) {
    changed = true;
  }
  if (ImGui::TreeNodeEx("Source-Sink", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragFloat("Flower Sink Strength", &flower_sink_strength, 1.0f, 0.0f, 1000.0f) || changed;
    changed = ImGui::DragFloat("Fruit Sink Strength", &fruit_sink_strength, 1.0f, 0.0f, 1000.0f) || changed;
    ImGui::TreePop();
  }

  changed = ImGui::DragFloat("Phototropism", &phototropism, 0.001f, 0.0f, 1.0f) || changed;
  changed = ImGui::DragFloat("Gravitropism", &gravitropism, 0.001f, 0.0f, 1.0f) || changed;

  return changed;
}

void BasicReproductionModuleDescriptor::GenerateFruitMatrices(std::vector<glm::mat4>& matrices,
                                                              const SkeletonNodeInfo& internode_info,
                                                              float tree_size) const {
}