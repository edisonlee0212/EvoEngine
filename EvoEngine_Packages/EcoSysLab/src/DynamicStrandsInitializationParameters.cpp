#include "DynamicStrandsInitializationParameters.hpp"

#include "BasicFoliageDescriptor.hpp"
#include "SDKInspectionAdapters.hpp"

using namespace eco_sys_lab_package;

bool DynamicStrandsInitializeParameters::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Min segment length", &min_segment_length, 0.001f, 0.001f, max_segment_length))
    changed = true;
  if (ImGui::DragFloat("Max segment length", &max_segment_length, 0.001f, min_segment_length, 1.0f))
    changed = true;
  if (ImGui::DragInt("Uniform subdivision", &uniform_subdivision, 1, 1, 16)) {
    uniform_subdivision = glm::clamp(uniform_subdivision, 1, 16);
    changed = true;
  }
  if (ImGui::DragFloat("Neighbor vertical range", &neighbor_vertical_range, 0.01f, 0.01f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Neighbor horizontal range", &neighbor_horizontal_range, 0.01f, 0.01f, 10.0f))
    changed = true;
  if (ImGui::TreeNodeEx("Physical properties", ImGuiTreeNodeFlags_DefaultOpen)) {
    static bool show_damage_graph = false;
    ImGui::Checkbox("Show damage graph", &show_damage_graph);
    if (show_damage_graph) {
      changed = evo_engine::DrawProceduralNoiseGraph(damage_graph, "Damage graph", editor_layer) || changed;
    }

    if (ImGui::DragFloat3("Damage scale factor", &damage_scale_factor.x, 0.001f, 0.f, 1.f)) {
      changed = true;
    }

    if (ImGui::DragFloat("Sapwood offset", &sapwood_offset, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Sapwood transition", &wood_transition, 0.001f, 0.001f, 1.f)) {
      wood_transition = glm::clamp(wood_transition, 0.001f, 1.f);
      changed = true;
    }
    if (ImGui::TreeNode("Wood material")) {
      ImGui::Checkbox("Show modulus graph", &show_modulus_graph);
      if (show_modulus_graph) {
        changed = modulus_graph.ShowGraph("modulus graph", editor_layer) || changed;
      }
      ImGui::TreePop();
    }

    ImGui::Checkbox("Show strength graph", &show_strength_graph);
    if (show_strength_graph) {
      changed = strength_graph.ShowGraph("Strength", editor_layer) || changed;
    }
    if (ImGui::Checkbox("Trunk", &trunk_additional_strength)) {
      changed = true;
    }

    if (trunk_additional_strength) {
      ImGui::Checkbox("Show trunk biological properties graph", &show_biological_properties_graph);
      if (show_biological_properties_graph) {
        changed = biological_properties_graph.ShowGraph("biological properties", editor_layer) || changed;
      }
    }

    if (ImGui::TreeNode("Foliage attachments")) {
      if (leaf_position_alpha.Draw("Leaf position alpha"))
        changed = true;

      if (leaf_rotation_alpha.Draw("Leaf rotation alpha"))
        changed = true;

      if (max_leaf_position_strain.Draw("Max leaf position strain"))
        changed = true;

      if (max_leaf_rotation_strain.Draw("Max leaf rotation strain"))
        changed = true;

      ImGui::TreePop();
    }

    ImGui::TreePop();
  }

  editor_layer->DragAndDropButton<BasicFoliageDescriptor>(foliage_descriptor, "Foliage Descriptor");

  if (ImGui::TreeNode("Meshing Properties")) {
#ifdef USE_CGAL
    if (ImGui::Checkbox("Use CGAL", &use_cgal))
      changed = true;
#endif  // USE_CGAL
    if (ImGui::Checkbox("Triangulate per bundle", &triangulate_per_bundle))
      changed = true;
    if (ImGui::DragFloat("Alpha", &alpha, 0.000001f, 0.0f, 1.0f, "%.6f"))
      changed = true;
    if (ImGui::DragFloat("Bifurcation Alpha", &bifurcation_alpha, 0.000001f, 0.0f, 1.0f, "%.6f"))
      changed = true;
    if (ImGui::DragFloat("Max Distance Squared", &max_dist_squared, 0.000001f, 0.0f, 1.0f, "%.6f"))
      changed = true;
    if (ImGui::Checkbox("Use cubic Hermite spline", &use_cubic_hermite_spline))
      changed = true;
    if (ImGui::DragInt("Min bundle size", &min_bundle_size, 1, 1, 100))
      changed = true;

    ImGui::TreePop();
  }

  return changed;
}

void DynamicStrandsInitializeParameters::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "min_segment_length" << YAML::Value << min_segment_length;
  out << YAML::Key << "max_segment_length" << YAML::Value << max_segment_length;
  out << YAML::Key << "uniform_subdivision" << YAML::Value << uniform_subdivision;

  damage_graph.Save("damage_graph", out);

  out << YAML::Key << "damage_scale_factor" << YAML::Value << damage_scale_factor;
  out << YAML::Key << "neighbor_vertical_range" << YAML::Value << neighbor_vertical_range;
  out << YAML::Key << "neighbor_horizontal_range" << YAML::Value << neighbor_horizontal_range;

  out << YAML::Key << "sapwood_offset" << YAML::Value << sapwood_offset;
  out << YAML::Key << "wood_transition" << YAML::Value << wood_transition;

  modulus_graph.Save("modulus_graph", out);

  strength_graph.Save("strength_graph", out);

  out << YAML::Key << "trunk_additional_strength" << YAML::Value << trunk_additional_strength;
  biological_properties_graph.Save("biological_properties_graph", out);

  leaf_position_alpha.Save("leaf_position_alpha", out);
  leaf_rotation_alpha.Save("leaf_rotation_alpha", out);
  max_leaf_position_strain.Save("max_leaf_position_strain", out);
  max_leaf_rotation_strain.Save("max_leaf_rotation_strain", out);

  out << YAML::Key << "root_transform" << YAML::Value << root_transform.value;
  out << YAML::Key << "use_cgal" << YAML::Value << use_cgal;
  out << YAML::Key << "triangulate_per_bundle" << YAML::Value << triangulate_per_bundle;

  out << YAML::Key << "alpha" << YAML::Value << alpha;
  out << YAML::Key << "bifurcation_alpha" << YAML::Value << bifurcation_alpha;
  out << YAML::Key << "max_dist_squared" << YAML::Value << max_dist_squared;
  out << YAML::Key << "use_cubic_hermite_spline" << YAML::Value << use_cubic_hermite_spline;
  out << YAML::Key << "min_bundle_size" << YAML::Value << min_bundle_size;

  foliage_descriptor.Save("foliage_descriptor", out);

  out << YAML::EndMap;
}

void DynamicStrandsInitializeParameters::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& in_parameters = in[name];

    if (in_parameters["min_segment_length"])
      min_segment_length = in_parameters["min_segment_length"].as<float>();
    if (in_parameters["max_segment_length"])
      max_segment_length = in_parameters["max_segment_length"].as<float>();

    if (in_parameters["uniform_subdivision"])
      uniform_subdivision = in_parameters["uniform_subdivision"].as<int>();

    damage_graph.Load("damage_graph", in_parameters);
    if (in_parameters["damage_scale_factor"])
      damage_scale_factor = in_parameters["damage_scale_factor"].as<glm::vec3>();
    if (in_parameters["neighbor_vertical_range"])
      neighbor_vertical_range = in_parameters["neighbor_vertical_range"].as<float>();
    if (in_parameters["neighbor_horizontal_range"])
      neighbor_horizontal_range = in_parameters["neighbor_horizontal_range"].as<float>();

    if (in_parameters["sapwood_offset"])
      sapwood_offset = in_parameters["sapwood_offset"].as<float>();
    if (in_parameters["wood_transition"])
      wood_transition = in_parameters["wood_transition"].as<float>();
    modulus_graph.Load("modulus_graph", in_parameters);

    strength_graph.Load("strength_graph", in_parameters);

    if (in_parameters["trunk_additional_strength"])
      trunk_additional_strength = in_parameters["trunk_additional_strength"].as<bool>();
    biological_properties_graph.Load("biological_properties_graph", in_parameters);

    leaf_position_alpha.Load("leaf_position_alpha", in_parameters);
    leaf_rotation_alpha.Load("leaf_rotation_alpha", in_parameters);
    max_leaf_position_strain.Load("max_leaf_position_strain", in_parameters);
    max_leaf_rotation_strain.Load("max_leaf_rotation_strain", in_parameters);

    if (in_parameters["root_transform"])
      root_transform.value = in_parameters["root_transform"].as<glm::mat4>();

    if (in_parameters["use_cgal"])
      use_cgal = in_parameters["use_cgal"].as<bool>();
    if (in_parameters["triangulate_per_bundle"])
      triangulate_per_bundle = in_parameters["triangulate_per_bundle"].as<bool>();

    if (in_parameters["alpha"])
      alpha = in_parameters["alpha"].as<float>();
    if (in_parameters["bifurcation_alpha"])
      bifurcation_alpha = in_parameters["bifurcation_alpha"].as<float>();
    if (in_parameters["max_dist_squared"])
      max_dist_squared = in_parameters["max_dist_squared"].as<float>();
    if (in_parameters["use_cubic_hermite_spline"])
      use_cubic_hermite_spline = in_parameters["use_cubic_hermite_spline"].as<bool>();
    if (in_parameters["min_bundle_size"])
      min_bundle_size = in_parameters["min_bundle_size"].as<int>();

    foliage_descriptor.Load("foliage_descriptor", in_parameters);
  }
}
