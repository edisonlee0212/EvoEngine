#include "BasicReproductionModuleDescriptor.hpp"
#include "TreeModel.hpp"
using namespace eco_sys_lab_plugin;

void BasicReproductionModuleDescriptor::PrepareController(ReproductionController& reproduction_controller) const {
  reproduction_controller.module_count = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                             const SkeletonNode<InternodeGrowthData>& internode) {
    return count_per_internode;
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
      TreeModel::ApplyTropism(glm::vec3(0, 1, 0), phototropism, front, up);

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
      TreeModel::ApplyTropism(glm::vec3(0, -1, 0), gravitropism, front, up);

      fruit.rotation = glm::quatLookAt(front, up);

      fruit.position_offset =
          glm::mix(glm::vec3(0.f), internode.info.GetGlobalEndPosition() - internode.info.global_position,
                   glm::linearRand(0.f, 1.f));
      fruit.stem_length = glm::abs(stem_length.GetValue());
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
}
void BasicReproductionModuleDescriptor::Serialize(YAML::Emitter& out) const {
}
void BasicReproductionModuleDescriptor::Deserialize(const YAML::Node& in) {
}
bool BasicReproductionModuleDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  return changed;
}
void BasicReproductionModuleDescriptor::GenerateFruitMatrices(std::vector<glm::mat4>& matrices,
                                                              const SkeletonNodeInfo& internode_info,
                                                              float tree_size) const {
}