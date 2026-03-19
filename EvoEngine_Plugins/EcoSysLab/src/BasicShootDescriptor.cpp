#include "BasicShootDescriptor.hpp"

#include <cmath>
#include "ShootModel.hpp"

using namespace eco_sys_lab_plugin;

void BasicShootDescriptor::PrepareController(ShootGrowthController& shoot_growth_controller) const {
  shoot_growth_controller.base_internode_count = base_internode_count;
  shoot_growth_controller.base_node_initialization = [&](std::mt19937& random_engine,
                                                         const ShootGrowthData& shoot_growth_data,
                                                         SkeletonNode<InternodeGrowthData>& shoot_node) {
    auto& node_info = shoot_node.info;
    auto& node_data = shoot_node.data;

    node_data.internode_thickness = 1.f;
    node_info.thickness = shoot_growth_controller.base_thickness;
    node_data.internode_length = 0.0f;
    node_data.buds.emplace_back();
    auto& apical_bud = node_data.buds.back();
    apical_bud.type = BudType::Apical;
    apical_bud.status = OrganStatus::Flushed;

    apical_bud.local_rotation = glm::vec3(0, 0.0f, glm::radians(Random::Uniform(random_engine, 0.f, 360.f)));

    node_info.global_position = node_data.desired_global_position = glm::vec3(0.0f);
    node_data.desired_local_rotation = glm::vec3(0.0f);
    node_info.global_rotation = node_info.regulated_global_rotation = node_data.desired_global_rotation =
        glm::vec3(glm::radians(90.0f), 0.0f, 0.0f);
    node_info.GetGlobalDirection() = glm::normalize(node_info.global_rotation * glm::vec3(0, 0, -1));
  };
  shoot_growth_controller.sagging = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                        const SkeletonNode<InternodeGrowthData>& internode) {
    float strength = end_node_thickness * internode.data.sagging_force * gravity_bending_strength /
                     glm::pow(internode.info.thickness / end_node_thickness, gravity_bending_thickness_factor);
    strength = gravity_bending_max * (1.f - glm::exp(-glm::abs(strength)));
    return glm::max(internode.data.sagging, strength);
  };

  shoot_growth_controller.internode_growth_rate = growth_rate / internode_length;

  shoot_growth_controller.bud_rotation = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                             const SkeletonNode<InternodeGrowthData>& internode, const Bud& bud) {
    if (bud.type == BudType::Apical) {
      const float roll_angle =
          roll_angle_graph.GetValue(glm::vec4(internode.info.global_position, internode.info.root_distance));
      float apical_angle = 0.f;
      if (straight_trunk == 0.f || internode.info.order != 0 || internode.info.root_distance >= straight_trunk) {
        apical_angle =
            apical_angle_graph.GetValue(glm::vec4(internode.info.global_position, internode.info.root_distance));
      }
      return glm::vec3(glm::radians(apical_angle), 0.0f, glm::radians(roll_angle));
    }
    const float branching_angle =
        branching_angle_graph.GetValue(glm::vec4(internode.info.global_position, internode.info.root_distance));
    return glm::vec3(0.f, glm::radians(branching_angle), glm::radians((bud.index - 1) * 360.f / lateral_bud_count));
  };
  shoot_growth_controller.tropism = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                        const SkeletonNode<InternodeGrowthData>& old_internode,
                                        const SkeletonNode<InternodeGrowthData>& new_internode, glm::quat& rotation) {
    auto desired_global_front = rotation * glm::vec3(0, 0, -1);
    auto desired_global_up = rotation * glm::vec3(0, 1, 0);

    if (straight_trunk == 0.f || old_internode.info.order != 0 || old_internode.info.root_distance >= straight_trunk) {
      PlantModel::ApplyTropism(-shoot_growth_data.gravity_direction, gravitropism, desired_global_front,
                               desired_global_up);
      PlantModel::ApplyTropism(old_internode.data.light_direction, phototropism, desired_global_front,
                               desired_global_up);
      if (const auto horizontal_direction = glm::vec3(desired_global_front.x, 0.0f, desired_global_front.z);
          glm::length(horizontal_direction) > glm::epsilon<float>() && old_internode.info.order != 0) {
        PlantModel::ApplyTropism(glm::normalize(horizontal_direction), horizontal_tropism, desired_global_front,
                                 desired_global_up);
      }
    }
    rotation = glm::quatLookAt(desired_global_front, desired_global_up);
  };

  shoot_growth_controller.base_internode_length = internode_length;
  shoot_growth_controller.internode_length = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                                 const SkeletonNode<InternodeGrowthData>& internode) {
    return internode.data.internode_length * glm::pow(internode.info.thickness / shoot_growth_controller.base_thickness,
                                                      internode_length_thickness_factor);
  };

  shoot_growth_controller.base_thickness = end_node_thickness;

  shoot_growth_controller.thickness = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                          const SkeletonNode<InternodeGrowthData>& internode) {
    return thickness_age_factor * shoot_growth_controller.internode_growth_rate *
           (shoot_growth_data.age - internode.data.start_age);
  };

  shoot_growth_controller.thickness_accumulation_factor = [&](std::mt19937& random_engine,
                                                              const ShootGrowthData& shoot_growth_data,
                                                              const SkeletonNode<InternodeGrowthData>& internode) {
    return thickness_accumulation_factor;
  };

  shoot_growth_controller.shadow_size = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                            const SkeletonNode<InternodeGrowthData>& internode) {
    return internode_shadow_factor;
  };

  shoot_growth_controller.lateral_bud_count = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                                  const SkeletonNode<InternodeGrowthData>& internode) {
    if (max_order == -1 || internode.info.order < max_order) {
      return lateral_bud_count;
    }
    return 0;
  };
  shoot_growth_controller.bud_flushing_rate = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                                  const SkeletonNode<InternodeGrowthData>& internode) {
    if (internode.IsApical()) {
      if (internode.info.root_distance < 0.5f)
        return 1.f;
      return 1.f - apical_bud_extinction_rate;
    }
    float flushing_rate = lateral_bud_flushing_rate;
    if (internode.data.inhibitor_sink > 0.0f)
      flushing_rate *= glm::exp(-internode.data.inhibitor_sink);
    return flushing_rate;
  };

  shoot_growth_controller.growth_potential = [&](std::mt19937& random_engine, const ShootSkeleton& shoot_skeleton,
                                                 const SkeletonNode<InternodeGrowthData>& internode) {
    const float factor =
        (apical_control < 0.f ? static_cast<float>(internode.info.level + 1) / (shoot_skeleton.GetMaxLevel() + 1)
                              : static_cast<float>(internode.info.order + 1) / (shoot_skeleton.GetMaxOrder() + 1));
    /*const float local_apical_control =
        1.f / glm::pow(apical_control, shoot_growth_controller.use_level_for_apical_control ? internode.data.level
                                                                                            : internode.data.order);*/
    const float local_apical_control = 1.f - ActivationFunction::Sigmoid(0, 1, 0.5f, apical_control, factor);
    float local_root_distance_control;
    if (root_distance_control != 0.f) {
      float distance = internode.info.root_distance + internode.info.length;
      if (distance == 0.f)
        distance = 1.f;
      local_root_distance_control = glm::pow(1.f / distance, root_distance_control);
    } else {
      local_root_distance_control = 1.f;
    }
    float local_height_control;
    if (height_control != 0.f) {
      float y = internode.info.GetGlobalEndPosition().y;
      if (y == 0.f)
        y = 1.f;
      local_height_control = glm::pow(1.f / y, height_control);
    } else {
      local_height_control = 1.f;
    }

    return local_apical_control * local_root_distance_control * local_height_control;
  };

  shoot_growth_controller.growth_inhibitor = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                                 const SkeletonNode<InternodeGrowthData>& internode) {
    return apical_dominance * internode.data.light_intake;
  };

  shoot_growth_controller.growth_inhibitor_transport =
      [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data, float growth_inhibitor,
          const SkeletonNode<InternodeGrowthData>& internode) {
        // Example implementation: apply apical_dominance_loss to the inhibitor value
        return growth_inhibitor * (1.0f - apical_dominance_loss);
      };

  // Update the Source/Sink Logic
  shoot_growth_controller.update_carbohydrate_state = [&](std::mt19937& random_engine, ShootSkeleton& shoot_skeleton,
                                                          SkeletonNode<InternodeGrowthData>& internode,
                                                          const ClimateModel& climate_model, float delta_time) {
    auto& data = internode.data;
    auto& info = internode.info;

    // 1. Capacity
    data.max_carbohydrate_mass = glm::max(info.volume * 1e6f, 1e-6f);
    float saturation = data.carbohydrate_mass / data.max_carbohydrate_mass;
    saturation = glm::clamp(saturation, 0.0f, 1.0f);

    auto clamp_mass = [&](float m) -> float {
      if (!std::isfinite(m))
        return 0.0f;
      return glm::clamp(m, 0.0f, data.max_carbohydrate_mass);
    };

    // 2. Leaf Production (With Feedback Inhibition)
    // As saturation approaches 1.0, production drops to 0.
    float production_feedback = 1.0f - glm::pow(saturation, 2.0f);  // Non-linear dropoff
    for (const auto& leaf : data.leaves) {
      if (leaf.status == OrganStatus::Flushed) {
        const float production =
            (leaf.carbohydrate_source * delta_time) * data.max_carbohydrate_mass * production_feedback;
        data.carbohydrate_mass = clamp_mass(data.carbohydrate_mass + production);
      }
    }

    // 3. Fruit Consumption (Standard)
    for (const auto& fruit : data.fruits) {
      if (fruit.status == OrganStatus::Flushed) {
        const float consumption = (fruit.carbohydrate_sink * delta_time) * data.max_carbohydrate_mass;
        data.carbohydrate_mass = clamp_mass(data.carbohydrate_mass - consumption);
      }
    }

    if (!std::isfinite(data.carbohydrate_mass)) {
      data.carbohydrate_mass = 0.0f;
    }

    // 4. Respiration (Non-Linear "Luxury Consumption")
    // Burn more when full to push system towards equilibrium.
    // Rate scales from 0.2x at low saturation to 1.8x at high saturation.
    float metabolic_scaling = 0.2f + 1.8f * (saturation * saturation);
    data.carbohydrate_sink += data.max_carbohydrate_mass * respiration_rate * metabolic_scaling * delta_time;

    // Safety
    if (!std::isfinite(data.carbohydrate_sink) || data.carbohydrate_sink < 0.0f) {
      data.carbohydrate_sink = std::isfinite(data.carbohydrate_sink) ? glm::max(0.0f, data.carbohydrate_sink) : 0.0f;
    }

    // 5. Conductance (Flow)
    float base_turnover_rate = conductance_multiplier;

    if (conductance_model == 0) {
      if (info.length > 0.0f) {
        float area = glm::pi<float>() * info.thickness * info.thickness;
        data.conductance = (area / info.length) * 1e6f * conductance_multiplier;
      } else {
        data.conductance = 0.0f;
      }
    } else if (conductance_model == 1) {  // Pipe Model
      float thickness_ratio = 1.0f;
      if (internode.GetParentHandle() != -1) {
        const auto& parent_node = shoot_skeleton.RefNode(internode.GetParentHandle());
        const float parent_thickness = glm::max(parent_node.info.thickness, 1e-9f);
        thickness_ratio = glm::clamp(info.thickness / parent_thickness, 0.0f, 1.0f);
      }
      data.conductance =
          data.max_carbohydrate_mass * base_turnover_rate * glm::pow(thickness_ratio, pipe_model_exponent);
    } else {
      data.conductance = data.max_carbohydrate_mass * base_turnover_rate;
    }

    /*// Winter source: make every root node a source during winter (DecFeb)
    {
      // climate_model.time is in years; convert to day-of-year
      const float days = climate_model.time * 365.0f;
      const float day_in_year = glm::mod(days, 365.0f);
      const bool is_winter = (day_in_year < 59.0f) || (day_in_year >= 210.0f);  // JanFeb and Dec

      if (is_winter) {
        data.conductance = 0.0f;
      }
    }*/

    // Safety
    if (!std::isfinite(data.conductance) || data.conductance < 0.0f) {
      data.conductance = 0.0f;
    }
    data.net_flow_balance = 0.0f;
  };
}

void BasicShootDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "base_internode_count" << YAML::Value << base_internode_count;
  out << YAML::Key << "straight_trunk" << YAML::Value << straight_trunk;

  out << YAML::Key << "growth_rate" << YAML::Value << growth_rate;

  branching_angle_graph.Save("branching_angle_graph", out);
  roll_angle_graph.Save("roll_angle_graph", out);
  apical_angle_graph.Save("apical_angle_graph", out);

  out << YAML::Key << "gravitropism" << YAML::Value << gravitropism;
  out << YAML::Key << "phototropism" << YAML::Value << phototropism;
  out << YAML::Key << "horizontal_tropism" << YAML::Value << horizontal_tropism;
  out << YAML::Key << "gravity_bending_strength" << YAML::Value << gravity_bending_strength;
  out << YAML::Key << "gravity_bending_thickness_factor" << YAML::Value << gravity_bending_thickness_factor;
  out << YAML::Key << "gravity_bending_max" << YAML::Value << gravity_bending_max;

  out << YAML::Key << "internode_length" << YAML::Value << internode_length;
  out << YAML::Key << "internode_length_thickness_factor" << YAML::Value << internode_length_thickness_factor;
  out << YAML::Key << "end_node_thickness" << YAML::Value << end_node_thickness;
  out << YAML::Key << "thickness_accumulation_factor" << YAML::Value << thickness_accumulation_factor;
  out << YAML::Key << "thickness_age_factor" << YAML::Value << thickness_age_factor;
  out << YAML::Key << "internode_shadow_factor" << YAML::Value << internode_shadow_factor;

  out << YAML::Key << "lateral_bud_count" << YAML::Value << lateral_bud_count;
  out << YAML::Key << "max_order" << YAML::Value << max_order;
  out << YAML::Key << "apical_bud_extinction_rate" << YAML::Value << apical_bud_extinction_rate;
  out << YAML::Key << "lateral_bud_flushing_rate" << YAML::Value << lateral_bud_flushing_rate;
  out << YAML::Key << "apical_control" << YAML::Value << apical_control;
  out << YAML::Key << "height_control" << YAML::Value << height_control;
  out << YAML::Key << "root_distance_control" << YAML::Value << root_distance_control;

  out << YAML::Key << "apical_dominance" << YAML::Value << apical_dominance;
  out << YAML::Key << "apical_dominance_loss" << YAML::Value << apical_dominance_loss;

  out << YAML::Key << "conductance_model" << YAML::Value << conductance_model;
  out << YAML::Key << "pipe_model_exponent" << YAML::Value << pipe_model_exponent;
  out << YAML::Key << "respiration_rate" << YAML::Value << respiration_rate;
  out << YAML::Key << "conductance_multiplier" << YAML::Value << conductance_multiplier;

  out << YAML::Key << "pruning_force_sink_enabled" << YAML::Value << pruning_force_sink_enabled;
  out << YAML::Key << "pruning_force_sink_multiplier" << YAML::Value << pruning_force_sink_multiplier;
  out << YAML::Key << "pruning_force_sink_depth" << YAML::Value << pruning_force_sink_depth;
  out << YAML::Key << "pruning_force_sink_depth_decay" << YAML::Value << pruning_force_sink_depth_decay;
}

void BasicShootDescriptor::Deserialize(const YAML::Node& in) {
  if (in["base_internode_count"])
    base_internode_count = in["base_internode_count"].as<int>();
  if (in["straight_trunk"])
    straight_trunk = in["straight_trunk"].as<float>();

  if (in["growth_rate"])
    growth_rate = in["growth_rate"].as<float>();

  branching_angle_graph.Load("branching_angle_graph", in);
  roll_angle_graph.Load("roll_angle_graph", in);
  apical_angle_graph.Load("apical_angle_graph", in);

  if (in["gravitropism"])
    gravitropism = in["gravitropism"].as<float>();
  if (in["phototropism"])
    phototropism = in["phototropism"].as<float>();
  if (in["horizontal_tropism"])
    horizontal_tropism = in["horizontal_tropism"].as<float>();
  if (in["gravity_bending_strength"])
    gravity_bending_strength = in["gravity_bending_strength"].as<float>();
  if (in["gravity_bending_thickness_factor"])
    gravity_bending_thickness_factor = in["gravity_bending_thickness_factor"].as<float>();
  if (in["gravity_bending_max"])
    gravity_bending_max = in["gravity_bending_max"].as<float>();

  if (in["internode_length"])
    internode_length = in["internode_length"].as<float>();
  if (in["internode_length_thickness_factor"])
    internode_length_thickness_factor = in["internode_length_thickness_factor"].as<float>();
  if (in["end_node_thickness"])
    end_node_thickness = in["end_node_thickness"].as<float>();
  if (in["thickness_accumulation_factor"])
    thickness_accumulation_factor = in["thickness_accumulation_factor"].as<float>();
  if (in["thickness_age_factor"])
    thickness_age_factor = in["thickness_age_factor"].as<float>();
  if (in["internode_shadow_factor"])
    internode_shadow_factor = in["internode_shadow_factor"].as<float>();

  if (in["lateral_bud_count"])
    lateral_bud_count = in["lateral_bud_count"].as<int>();
  if (in["max_order"])
    max_order = in["max_order"].as<int>();
  if (in["apical_bud_extinction_rate"])
    apical_bud_extinction_rate = in["apical_bud_extinction_rate"].as<float>();
  if (in["lateral_bud_flushing_rate"])
    lateral_bud_flushing_rate = in["lateral_bud_flushing_rate"].as<float>();
  if (in["apical_control"])
    apical_control = in["apical_control"].as<float>();
  if (in["root_distance_control"])
    root_distance_control = in["root_distance_control"].as<float>();
  if (in["height_control"])
    height_control = in["height_control"].as<float>();

  if (in["apical_dominance"])
    apical_dominance = in["apical_dominance"].as<float>();
  if (in["apical_dominance_loss"])
    apical_dominance_loss = in["apical_dominance_loss"].as<float>();

  if (in["conductance_model"])
    conductance_model = in["conductance_model"].as<int>();
  if (in["pipe_model_exponent"])
    pipe_model_exponent = in["pipe_model_exponent"].as<float>();
  if (in["respiration_rate"])
    respiration_rate = in["respiration_rate"].as<float>();
  if (in["conductance_multiplier"])
    conductance_multiplier = in["conductance_multiplier"].as<float>();

  if (in["pruning_force_sink_enabled"])
    pruning_force_sink_enabled = in["pruning_force_sink_enabled"].as<bool>();
  if (in["pruning_force_sink_multiplier"])
    pruning_force_sink_multiplier = in["pruning_force_sink_multiplier"].as<float>();
  if (in["pruning_force_sink_depth"])
    pruning_force_sink_depth = in["pruning_force_sink_depth"].as<int>();
  if (in["pruning_force_sink_depth_decay"])
    pruning_force_sink_depth_decay = in["pruning_force_sink_depth_decay"].as<float>();
}

bool BasicShootDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  changed = ImGui::DragFloat("Growth rate", &growth_rate, 0.01f, 0.0f, 10.0f) || changed;
  changed = ImGui::DragFloat("Straight Trunk", &straight_trunk, 0.1f, 0.0f, 100.f) || changed;
  if (ImGui::TreeNodeEx("Internode", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragInt("Base node count", &base_internode_count, 1, 0, 3) || changed;
    changed = ImGui::DragInt("Lateral bud count", &lateral_bud_count, 1, 0, 3) || changed;
    changed = ImGui::DragInt("Max Order", &max_order, 1, -1, 100) || changed;

    static bool show_branching_angle_graph = false;
    static bool show_roll_angle_graph = false;
    static bool show_apical_angle_graph = false;
    ImGui::Checkbox("Show branching angle graph", &show_branching_angle_graph);
    ImGui::Checkbox("Show roll angle graph", &show_roll_angle_graph);
    ImGui::Checkbox("Show apical angle graph", &show_apical_angle_graph);
    if (show_branching_angle_graph) {
      changed = branching_angle_graph.ShowGraph("Branching Angle Graph", editor_layer) || changed;
    }
    if (show_roll_angle_graph) {
      changed = roll_angle_graph.ShowGraph("Roll Angle Graph", editor_layer) || changed;
    }
    if (show_apical_angle_graph) {
      changed = apical_angle_graph.ShowGraph("Apical Angle Graph", editor_layer) || changed;
    }

    changed = ImGui::DragFloat("Internode length", &internode_length, 0.001f) || changed;
    changed = ImGui::DragFloat("Internode length thickness factor", &internode_length_thickness_factor, 0.0001f, 0.0f,
                               1.0f) ||
              changed;
    changed =
        ImGui::DragFloat3("Thickness min/factor/age", &end_node_thickness, 0.0001f, 0.0f, 1.0f, "%.6f") || changed;

    changed = ImGui::DragFloat("Bending strength", &gravity_bending_strength, 0.01f, 0.0f, 1.0f, "%.3f") || changed;
    changed =
        ImGui::DragFloat("Bending thickness factor", &gravity_bending_thickness_factor, 0.1f, 0.0f, 10.f, "%.3f") ||
        changed;
    changed = ImGui::DragFloat("Bending angle factor", &gravity_bending_max, 0.01f, 0.0f, 1.0f, "%.3f") || changed;

    changed = ImGui::DragFloat("Internode shadow factor", &internode_shadow_factor, 0.001f, 0.0f, 1.0f) || changed;

    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Bud fate", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragFloat("Gravitropism", &gravitropism, 0.01f) || changed;
    changed = ImGui::DragFloat("Phototropism", &phototropism, 0.01f) || changed;
    changed = ImGui::DragFloat("Horizontal Tropism", &horizontal_tropism, 0.01f) || changed;

    changed = ImGui::DragFloat("Apical bud extinction rate", &apical_bud_extinction_rate, 0.01f, 0.0f, 1.0f, "%.5f") ||
              changed;
    changed =
        ImGui::DragFloat("Lateral bud flushing rate", &lateral_bud_flushing_rate, 0.01f, 0.0f, 1.0f, "%.5f") || changed;

    changed = ImGui::DragFloat2("Inhibitor val/loss", &apical_dominance, 0.01f) || changed;
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Tree Shape Control", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragFloat("Apical control", &apical_control, 0.01f) || changed;
    changed = ImGui::DragFloat("Root distance control", &root_distance_control, 0.01f) || changed;
    changed = ImGui::DragFloat("Height control", &height_control, 0.01f) || changed;

    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Source/Sink Solver", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragFloat("Respiration Rate (%)", &respiration_rate, 0.001f, 0.0f, 1.0f, "%.6f") || changed;
    changed = ImGui::DragFloat("Conductance Multiplier (Turnover/Day)", &conductance_multiplier, 0.1f, 0.0f, 100.0f,
                               "%.6f") ||
              changed;
    const char* items[] = {"Area / Length", "Pipe Model", "Multiplier Only"};
    changed = ImGui::Combo("Conductance Model", &conductance_model, items, IM_ARRAYSIZE(items)) || changed;
    if (conductance_model == 1) {
      changed = ImGui::DragFloat("Pipe Model Exponent", &pipe_model_exponent, 0.1f, 0.1f, 8.0f) || changed;
    }

    ImGui::Separator();
    changed = ImGui::Checkbox("Enable Pruning Force Sink", &pruning_force_sink_enabled) || changed;
    changed = ImGui::DragFloat("Pruning Sink Multiplier", &pruning_force_sink_multiplier, 1.0f, 1.0f, 5000.0f,
                               "%.3f") ||
              changed;
    changed = ImGui::DragInt("Pruning Sink Depth", &pruning_force_sink_depth, 1, 0, 32) || changed;
    changed = ImGui::DragFloat("Pruning Sink Depth Decay", &pruning_force_sink_depth_decay, 0.01f, 0.0f, 1.0f,
                               "%.3f") ||
              changed;

    ImGui::TreePop();
  }
  return changed;
}