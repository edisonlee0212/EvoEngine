#include "BasicRootDescriptor.hpp"

#include "ShootModel.hpp"

using namespace eco_sys_lab_plugin;

void BasicRootDescriptor::PrepareController(RootGrowthController& root_growth_controller) const {
  root_growth_controller.base_root_node_count = base_root_node_count;
  root_growth_controller.root_node_growth_rate = growth_rate / root_node_length;

  root_growth_controller.base_node_initialization = [&](std::mt19937& random_engine,
                                                        const RootGrowthData& root_growth_data,
                                                        SkeletonNode<RootNodeGrowthData>& root_node) {
    auto& node_info = root_node.info;
    auto& node_data = root_node.data;
    root_node.data.horizontal_tropism = 0.0f;
    root_node.data.horizontal_tropism = tropism_intensity;

    node_info.global_position = node_data.desired_global_position = glm::vec3(0.0f);
    node_data.desired_local_rotation = glm::vec3(0.0f);
    node_info.global_rotation = node_info.regulated_global_rotation = node_data.desired_global_rotation =
        glm::vec3(glm::radians(-90.0f), 0.0f, 0.0f);
    node_info.GetGlobalDirection() = glm::normalize(node_info.global_rotation * glm::vec3(0, 0, -1));
  };

  root_growth_controller.node_rotation = [&](std::mt19937& random_engine, const RootGrowthData& root_growth_data,
                                             const SkeletonNode<RootNodeGrowthData>& old_node,
                                             const SkeletonNode<RootNodeGrowthData>& new_node) {
    if (new_node.IsApical()) {
      const float roll_angle =
          roll_angle_graph.GetValue(glm::vec4(old_node.info.global_position, old_node.info.root_distance));
      float apical_angle = 0.f;
      if (straight_tap_root == 0.f || old_node.info.order != 0 || old_node.info.root_distance >= straight_tap_root) {
        apical_angle =
            apical_angle_graph.GetValue(glm::vec4(old_node.info.global_position, old_node.info.root_distance));
      }
      return glm::vec3(glm::radians(apical_angle), 0.0f, glm::radians(roll_angle));
    }
    const float branching_angle =
        branching_angle_graph.GetValue(glm::vec4(old_node.info.global_position, old_node.info.root_distance));
    std::uniform_real_distribution<> distribution(0.0, 360.0);
    return glm::vec3(0.f, glm::radians(branching_angle), glm::radians(distribution(random_engine)));
  };

  root_growth_controller.tropism = [&](std::mt19937& random_engine, const RootGrowthData& root_growth_data,
                                       const SkeletonNode<RootNodeGrowthData>& old_root_node,
                                       SkeletonNode<RootNodeGrowthData>& new_root_node, glm::quat& rotation) {
    if (!new_root_node.IsApical()) {
      const float probability = tropism_switch_probability *
                                glm::exp(-tropism_switch_base_distance_factor * old_root_node.info.root_distance);
      std::uniform_real_distribution<> distribution(0.0, 1.0);
      if (probability >= distribution(random_engine)) {
        new_root_node.data.horizontal_tropism = old_root_node.data.vertical_tropism;
        new_root_node.data.vertical_tropism = old_root_node.data.horizontal_tropism;
      }
    }

    auto desired_global_front = rotation * glm::vec3(0, 0, -1);
    auto desired_global_up = rotation * glm::vec3(0, 1, 0);

    if (straight_tap_root == 0.f || old_root_node.info.order != 0 ||
        old_root_node.info.root_distance >= straight_tap_root) {
      PlantModel::ApplyTropism(root_growth_data.gravity_direction, new_root_node.data.vertical_tropism,
                               desired_global_front, desired_global_up);
      auto horizontal_front = glm::vec3(desired_global_front.x, 0.0f, desired_global_front.z);
      if (glm::length(horizontal_front) < glm::epsilon<float>()) {
        horizontal_front = glm::vec3(0, 0, -1);
      }
      PlantModel::ApplyTropism(horizontal_front, new_root_node.data.horizontal_tropism, desired_global_front,
                               desired_global_up);
    }

    if (old_root_node.data.soil_density == 0.0f) {
      PlantModel::ApplyTropism(root_growth_data.gravity_direction, 0.5f, desired_global_front, desired_global_up);
    }
    rotation = glm::quatLookAt(desired_global_front, desired_global_up);
  };

  root_growth_controller.base_root_node_length = root_node_length;
  root_growth_controller.root_node_length = [&](std::mt19937& random_engine, const RootGrowthData& root_growth_data,
                                                const SkeletonNode<RootNodeGrowthData>& root_node) {
    return root_node.data.node_length * glm::pow(root_node.info.thickness / root_growth_controller.base_thickness,
                                                 root_node_length_thickness_factor);
  };

  root_growth_controller.base_thickness = end_node_thickness;

  root_growth_controller.thickness = [&](std::mt19937& random_engine, const RootGrowthData& root_growth_data,
                                         const SkeletonNode<RootNodeGrowthData>& root_node) {
    return thickness_age_factor * root_growth_controller.root_node_growth_rate *
           (root_growth_data.age - root_node.data.start_age);
  };

  root_growth_controller.thickness_accumulation_factor = [&](std::mt19937& random_engine,
                                                             const RootGrowthData& root_growth_data,
                                                             const SkeletonNode<RootNodeGrowthData>& root_node) {
    return thickness_accumulation_factor;
  };

  root_growth_controller.growth_potential = [&](std::mt19937& random_engine, const RootSkeleton& root_skeleton,
                                                const SkeletonNode<RootNodeGrowthData>& root_node) {
    const float factor =
        (apical_control < 0.f ? static_cast<float>(root_node.info.level + 1) / (root_skeleton.GetMaxLevel() + 1)
                              : static_cast<float>(root_node.info.order + 1) / (root_skeleton.GetMaxOrder() + 1));
    const float local_apical_control = 1.f - ActivationFunction::Sigmoid(0, 1, 0.5f, apical_control, factor);
    float local_root_distance_control;
    if (root_distance_control != 0.f) {
      float distance = root_node.info.root_distance + root_node.info.length;
      if (distance == 0.f)
        distance = 1.f;
      local_root_distance_control = glm::pow(1.f / distance, root_distance_control);
    } else {
      local_root_distance_control = 1.f;
    }
    const float soil_friction =
        1.0f -
        glm::pow(1.0f / glm::max(root_node.data.soil_density * soil_density_friction.x, 1.0f), soil_density_friction.y);
    return local_apical_control * local_root_distance_control * glm::clamp(1.f - soil_friction, 0.0f, 1.0f);
  };

  root_growth_controller.growth_inhibitor = [&](std::mt19937& random_engine, const RootGrowthData& shoot_growth_data,
                                                const SkeletonNode<RootNodeGrowthData>& internode) {
    return apical_dominance * internode.data.nutrient;
  };

  root_growth_controller.growth_inhibitor_transport =
      [&](std::mt19937& random_engine, const RootGrowthData& shoot_growth_data, const float growth_inhibitor,
          const SkeletonNode<RootNodeGrowthData>& internode) {
        return growth_inhibitor * glm::clamp(1.f - apical_dominance_loss, 0.0f, 1.f);
      };

  root_growth_controller.lateral_node_flushing_rate = [&](std::mt19937& random_engine,
                                                          const RootGrowthData& root_growth_data,
                                                          const SkeletonNode<RootNodeGrowthData>& root_node) {
    float flushing_rate = lateral_node_flushing_probability;
    if (root_node.data.inhibitor_sink > 0.0f)
      flushing_rate *= glm::exp(-root_node.data.inhibitor_sink);
    return flushing_rate;
  };

  root_growth_controller.calculate_root_node_sink_strength = [&](std::mt19937& random_engine,
                                                                 RootSkeleton& root_skeleton,
                                                                 SkeletonNode<RootNodeGrowthData>& node,
                                                                 const ClimateModel& climate_model, float delta_time) {
    auto& info = node.info;
    auto& data = node.data;

    data.max_carbohydrate_mass = glm::max(info.volume * 1e6f, 1e-6f);
    float saturation = data.carbohydrate_mass / data.max_carbohydrate_mass;
    saturation = glm::clamp(saturation, 0.0f, 1.0f);

    if (conductance_model == 0) {
      if (info.length > 0.0f) {
        const float area = glm::pi<float>() * info.thickness * info.thickness;
        data.conductance = (area / info.length) * 1e6f * conductance_multiplier;
      } else {
        data.conductance = 0.0f;
      }
    } else if (conductance_model == 1) {
      float ratio = 1.0f;
      if (node.GetParentHandle() != -1) {
        const auto& parent = root_skeleton.RefNode(node.GetParentHandle());
        const float parent_th = glm::max(parent.info.thickness, 1e-9f);
        ratio = glm::clamp(info.thickness / parent_th, 0.0f, 1.0f);
      }
      data.conductance = data.max_carbohydrate_mass * conductance_multiplier * glm::pow(ratio, pipe_model_exponent);
    } else {
      data.conductance = data.max_carbohydrate_mass * conductance_multiplier;
    }
    if (!std::isfinite(data.conductance) || data.conductance < 0.0f)
      data.conductance = 0.0f;

    float metabolic_scaling = 0.2f + 1.8f * (saturation * saturation);
    data.carbohydrate_sink += data.max_carbohydrate_mass * respiration_rate * metabolic_scaling * delta_time;

    if (!std::isfinite(data.carbohydrate_sink) || data.carbohydrate_sink < 0.0f) {
      data.carbohydrate_sink = std::isfinite(data.carbohydrate_sink) ? glm::max(0.0f, data.carbohydrate_sink) : 0.0f;
    }

    if (!std::isfinite(data.carbohydrate_source))
      data.carbohydrate_source = 0.0f;

    if (!std::isfinite(data.carbohydrate_mass))
      data.carbohydrate_mass = 0.0f;
    data.carbohydrate_mass = glm::clamp(data.carbohydrate_mass, 0.0f, data.max_carbohydrate_mass);

    data.net_flow_balance = 0.0f;
  };
}

void BasicRootDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "base_root_node_count" << YAML::Value << base_root_node_count;
  out << YAML::Key << "straight_tap_root" << YAML::Value << straight_tap_root;

  out << YAML::Key << "growth_rate" << YAML::Value << growth_rate;
  out << YAML::Key << "lateral_node_flushing_probability" << YAML::Value << lateral_node_flushing_probability;

  branching_angle_graph.Save("branching_angle_graph", out);
  roll_angle_graph.Save("roll_angle_graph", out);
  apical_angle_graph.Save("apical_angle_graph", out);

  out << YAML::Key << "root_node_length" << YAML::Value << root_node_length;
  out << YAML::Key << "root_node_length_thickness_factor" << YAML::Value << root_node_length_thickness_factor;
  out << YAML::Key << "end_node_thickness" << YAML::Value << end_node_thickness;
  out << YAML::Key << "thickness_accumulation_factor" << YAML::Value << thickness_accumulation_factor;
  out << YAML::Key << "thickness_age_factor" << YAML::Value << thickness_age_factor;

  out << YAML::Key << "apical_control" << YAML::Value << apical_control;
  out << YAML::Key << "soil_density_friction" << YAML::Value << soil_density_friction;
  out << YAML::Key << "root_distance_control" << YAML::Value << root_distance_control;

  out << YAML::Key << "apical_dominance" << YAML::Value << apical_dominance;
  out << YAML::Key << "apical_dominance_loss" << YAML::Value << apical_dominance_loss;
  out << YAML::Key << "tropism_intensity" << YAML::Value << tropism_intensity;
  out << YAML::Key << "tropism_switch_probability" << YAML::Value << tropism_switch_probability;
  out << YAML::Key << "tropism_switch_base_distance_factor" << YAML::Value << tropism_switch_base_distance_factor;

  out << YAML::Key << "conductance_model" << YAML::Value << conductance_model;
  out << YAML::Key << "pipe_model_exponent" << YAML::Value << pipe_model_exponent;
  out << YAML::Key << "respiration_rate" << YAML::Value << respiration_rate;
  out << YAML::Key << "conductance_multiplier" << YAML::Value << conductance_multiplier;
  out << YAML::Key << "carbohydrate_sink_multiplier" << YAML::Value << carbohydrate_sink_multiplier;
  out << YAML::Key << "spring_reactivation_rate" << YAML::Value << spring_reactivation_rate;
}

void BasicRootDescriptor::Deserialize(const YAML::Node& in) {
  if (in["base_root_node_count"])
    base_root_node_count = in["base_root_node_count"].as<int>();
  if (in["straight_tap_root"])
    straight_tap_root = in["straight_tap_root"].as<float>();

  if (in["growth_rate"])
    growth_rate = in["growth_rate"].as<float>();

  branching_angle_graph.Load("branching_angle_graph", in);
  roll_angle_graph.Load("roll_angle_graph", in);
  apical_angle_graph.Load("apical_angle_graph", in);

  if (in["lateral_node_flushing_probability"])
    lateral_node_flushing_probability = in["lateral_node_flushing_probability"].as<float>();

  if (in["root_node_length"])
    root_node_length = in["root_node_length"].as<float>();
  if (in["root_node_length_thickness_factor"])
    root_node_length_thickness_factor = in["root_node_length_thickness_factor"].as<float>();
  if (in["end_node_thickness"])
    end_node_thickness = in["end_node_thickness"].as<float>();
  if (in["thickness_accumulation_factor"])
    thickness_accumulation_factor = in["thickness_accumulation_factor"].as<float>();
  if (in["thickness_age_factor"])
    thickness_age_factor = in["thickness_age_factor"].as<float>();

  if (in["apical_control"])
    apical_control = in["apical_control"].as<float>();
  if (in["root_distance_control"])
    root_distance_control = in["root_distance_control"].as<float>();
  if (in["soil_density_friction"])
    soil_density_friction = in["soil_density_friction"].as<glm::vec2>();

  if (in["apical_dominance"])
    apical_dominance = in["apical_dominance"].as<float>();
  if (in["apical_dominance_loss"])
    apical_dominance_loss = in["apical_dominance_loss"].as<float>();

  if (in["tropism_intensity"])
    tropism_intensity = in["tropism_intensity"].as<float>();
  if (in["tropism_switch_probability"])
    tropism_switch_probability = in["tropism_switch_probability"].as<float>();
  if (in["tropism_switch_base_distance_factor"])
    tropism_switch_base_distance_factor = in["tropism_switch_base_distance_factor"].as<float>();

  if (in["conductance_model"])
    conductance_model = in["conductance_model"].as<int>();
  if (in["pipe_model_exponent"])
    pipe_model_exponent = in["pipe_model_exponent"].as<float>();
  if (in["respiration_rate"])
    respiration_rate = in["respiration_rate"].as<float>();
  if (in["conductance_multiplier"])
    conductance_multiplier = in["conductance_multiplier"].as<float>();
  if (in["carbohydrate_sink_multiplier"])
    carbohydrate_sink_multiplier = in["carbohydrate_sink_multiplier"].as<float>();
  if (in["spring_reactivation_rate"])
    spring_reactivation_rate = in["spring_reactivation_rate"].as<float>();
}

bool BasicRootDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  changed = ImGui::DragFloat("Growth rate", &growth_rate, 0.01f, 0.0f, 10.0f) || changed;
  changed = ImGui::DragFloat("Straight Tap Root", &straight_tap_root, 0.1f, 0.0f, 100.f) || changed;
  if (ImGui::TreeNodeEx("Root node", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragInt("Base node count", &base_root_node_count, 1, 0, 3) || changed;
    changed = ImGui::DragFloat("Lateral node flushing prob", &lateral_node_flushing_probability, 0.01f, 0.01f, 1.0f) ||
              changed;
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

    changed = ImGui::DragFloat("Root node length", &root_node_length, 0.001f) || changed;
    changed = ImGui::DragFloat("Root node length thickness factor", &root_node_length_thickness_factor, 0.0001f, 0.0f,
                               1.0f) ||
              changed;
    changed =
        ImGui::DragFloat3("Thickness min/factor/age", &end_node_thickness, 0.0001f, 0.0f, 1.0f, "%.6f") || changed;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Root Shape Control", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragFloat("Apical control", &apical_control, 0.01f) || changed;
    changed = ImGui::DragFloat2("Inhibitor val/loss", &apical_dominance, 0.01f) || changed;
    changed = ImGui::DragFloat("Root distance control", &root_distance_control, 0.01f) || changed;
    changed = ImGui::DragFloat2("Soil Friction/Speed", &soil_density_friction.x, 0.01f) || changed;
    changed = ImGui::DragFloat("Tropism intensity", &tropism_intensity, 0.01f) || changed;
    changed = ImGui::DragFloat2("Tropism switch prob/dist", &tropism_switch_probability, 0.01f) || changed;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Source/Sink Solver", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragFloat("Respiration Rate (%)", &respiration_rate, 0.001f, 0.0f, 1.0f, "%.3f") || changed;
    changed = ImGui::DragFloat("Conductance Multiplier (Turnover/Day)", &conductance_multiplier, 0.1f, 0.0f, 100.0f,
                               "%.6f") ||
              changed;
    const char* items[] = {"Area / Length", "Pipe Model", "Multiplier Only"};
    changed = ImGui::Combo("Conductance Model", &conductance_model, items, IM_ARRAYSIZE(items)) || changed;
    if (conductance_model == 1) {
      changed = ImGui::DragFloat("Pipe Model Exponent", &pipe_model_exponent, 0.1f, 0.1f, 8.0f) || changed;
    }
    changed = ImGui::DragFloat("Sink Multiplier", &carbohydrate_sink_multiplier, 0.1f, 0.0f, 100.0f) || changed;
    changed =
        ImGui::DragFloat("Spring Reactivation Rate", &spring_reactivation_rate, 0.01f, 0.0f, 1.0f, "%.3f") || changed;
    ImGui::TreePop();
  }

  return changed;
}