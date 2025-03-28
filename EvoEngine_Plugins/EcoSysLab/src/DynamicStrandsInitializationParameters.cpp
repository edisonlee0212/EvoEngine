#include "DynamicStrandsInitializationParameters.hpp"

#include "FoliageDescriptor.hpp"

using namespace eco_sys_lab_plugin;

bool DynamicStrandsInitializeParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
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
    if (ImGui::TreeNode("Damage")) {
      if (damage.OnInspect()) {
        changed = true;
      }
      ImGui::TreePop();
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
      if (ImGui::DragFloat2("Density", &density.x, 1.f, 1, 1000)) {
        changed = true;
      }

      if (ImGui::DragFloat2("Shear/Stretch modulus", &max_stretch_shear_modulus.x, 0.01f, 0.f, 1000.f)) {
        changed = true;
      }
      if (ImGui::DragFloat2("Bending modulus", &max_bending_modulus.x, 0.01f, 0.f, 1000.f)) {
        changed = true;
      }
      if (ImGui::DragFloat2("Twisting modulus", &max_twisting_modulus.x, 0.01f, 0.f, 1000.f)) {
        changed = true;
      }
      ImGui::TreePop();
    }

    if (ImGui::DragFloat2("Shear/Stretch strength", &shear_stretch_strength.x, 1.f, 0.f, 2000.f)) {
      changed = true;
    }

    if (ImGui::DragFloat2("Bending strength", &bending_strength.x, 1.f, 0.f, 2000.f)) {
      changed = true;
    }
    if (ImGui::DragFloat2("Twisting strength", &twisting_strength.x, 1.f, 0.f, 2000.f)) {
      changed = true;
    }
    if (ImGui::DragFloat2("Bundle strength", &bundle_strength.x, 1.f, 0.f, 2000.f)) {
      changed = true;
    }

    if (ImGui::DragFloat2("Segment Pair strength", &connectivity_strength.x, 1.f, 0.f, 2000.f)) {
      changed = true;
    }
    if (ImGui::Checkbox("Trunk", &trunk_additional_strength)) {
      changed = true;
    }
    if (trunk_additional_strength) {
      if (ImGui::DragFloat("Trunk offset", &trunk_offset, 0.01f, 0.0f, 1.0f)) {
        changed = true;
      }
      if (ImGui::DragFloat("Trunk transition", &trunk_transition, 0.01f, 0.001f, 1.f)) {
        trunk_transition = glm::clamp(trunk_transition, 0.001f, 10.f);
        changed = true;
      }
      if (ImGui::DragFloat("Trunk additional strength", &trunk_additional_strength_factor, 0.01f, 0.001f, 1.f)) {
        trunk_additional_strength_factor = glm::clamp(trunk_additional_strength_factor, 0.0f, 1.f);
        changed = true;
      }
    }

    if (ImGui::TreeNode("Foliage attachments")) {
      if (leaf_position_alpha.OnInspect("Leaf position alpha"))
        changed = true;

      if (leaf_rotation_alpha.OnInspect("Leaf rotation alpha"))
        changed = true;

      if (max_leaf_position_strain.OnInspect("Max leaf position strain"))
        changed = true;

      if (max_leaf_rotation_strain.OnInspect("Max leaf rotation strain"))
        changed = true;

      ImGui::TreePop();
    }

    ImGui::TreePop();
  }

  editor_layer->DragAndDropButton<FoliageDescriptor>(foliage_descriptor, "Foliage Descriptor");

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

      ImGui::TreePop();
  }

  return changed;
}

void DynamicStrandsInitializeParameters::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "min_segment_length" << YAML::Value << min_segment_length;
  out << YAML::Key << "max_segment_length" << YAML::Value << max_segment_length;
  out << YAML::Key << "uniform_subdivision" << YAML::Value << uniform_subdivision;

  damage.Save("damage", out);

  out << YAML::Key << "damage_scale_factor" << YAML::Value << damage_scale_factor;
  out << YAML::Key << "neighbor_vertical_range" << YAML::Value << neighbor_vertical_range;
  out << YAML::Key << "neighbor_horizontal_range" << YAML::Value << neighbor_horizontal_range;

  out << YAML::Key << "sapwood_offset" << YAML::Value << sapwood_offset;
  out << YAML::Key << "wood_transition" << YAML::Value << wood_transition;

  out << YAML::Key << "density" << YAML::Value << density;
  out << YAML::Key << "max_stretch_shear_modulus" << YAML::Value << max_stretch_shear_modulus;
  out << YAML::Key << "max_bending_modulus" << YAML::Value << max_bending_modulus;
  out << YAML::Key << "max_twisting_modulus" << YAML::Value << max_twisting_modulus;

  out << YAML::Key << "shear_stretch_strength" << YAML::Value << shear_stretch_strength;
  out << YAML::Key << "bending_strength" << YAML::Value << bending_strength;
  out << YAML::Key << "twisting_strength" << YAML::Value << twisting_strength;
  out << YAML::Key << "bundle_strength" << YAML::Value << bundle_strength;
  out << YAML::Key << "connectivity_strength" << YAML::Value << connectivity_strength;

  out << YAML::Key << "trunk_additional_strength" << YAML::Value << trunk_additional_strength;
  out << YAML::Key << "trunk_offset" << YAML::Value << trunk_offset;
  out << YAML::Key << "trunk_transition" << YAML::Value << trunk_transition;
  out << YAML::Key << "trunk_additional_strength_factor" << YAML::Value << trunk_additional_strength_factor;

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

    damage.Load("damage", in_parameters);
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

    if (in_parameters["density"])
      density = in_parameters["density"].as<glm::vec2>();
    if (in_parameters["max_stretch_shear_modulus"])
      max_stretch_shear_modulus = in_parameters["max_stretch_shear_modulus"].as<glm::vec2>();
    if (in_parameters["max_bending_modulus"])
      max_bending_modulus = in_parameters["max_bending_modulus"].as<glm::vec2>();
    if (in_parameters["max_twisting_modulus"])
      max_twisting_modulus = in_parameters["max_twisting_modulus"].as<glm::vec2>();

    if (in_parameters["shear_stretch_strength"])
      shear_stretch_strength = in_parameters["shear_stretch_strength"].as<glm::vec2>();
    if (in_parameters["bending_strength"])
      bending_strength = in_parameters["bending_strength"].as<glm::vec2>();
    if (in_parameters["twisting_strength"])
      twisting_strength = in_parameters["twisting_strength"].as<glm::vec2>();
    if (in_parameters["bundle_strength"])
      bundle_strength = in_parameters["bundle_strength"].as<glm::vec2>();
    if (in_parameters["connectivity_strength"])
      connectivity_strength = in_parameters["connectivity_strength"].as<glm::vec2>();

    if (in_parameters["trunk_additional_strength"])
      trunk_additional_strength = in_parameters["trunk_additional_strength"].as<bool>();
    if (in_parameters["trunk_offset"])
      trunk_offset = in_parameters["trunk_offset"].as<float>();
    if (in_parameters["trunk_transition"])
      trunk_transition = in_parameters["trunk_transition"].as<float>();
    if (in_parameters["trunk_additional_strength_factor"])
      trunk_additional_strength_factor = in_parameters["trunk_additional_strength_factor"].as<float>();

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