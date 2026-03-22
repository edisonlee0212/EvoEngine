#include "CropDescriptor.hpp"

namespace digital_agriculture_plugin {

void CropDescriptor::OnCreate() {
  // Phenology defaults (sorghum-like cultivar).
  base_temperature = 8.0f;
  plastochron_gdd = 40.0f;
  final_leaf_number = 16;

  stem_elongation_gdd = 400.0f;
  flowering_gdd = 800.0f;
  grain_filling_gdd = 1000.0f;
  maturity_gdd = 1500.0f;
  leaf_growth_duration_gdd = 120.0f;
  senescence_onset_gdd = 100.0f;

  // Leaf geometry — PlottedDistribution with mean curve over normalized rank.
  max_leaf_length.mean = {0.0f, 2.5f, Curve2D(0.165f, 0.247f)};
  max_leaf_length.deviation = {0.0f, 0.0f, Curve2D(0.0f, 0.0f)};

  max_leaf_width.mean = {0.0f, 0.075f, Curve2D(0.5f, 0.5f)};
  max_leaf_width.deviation = {0.0f, 0.0f, Curve2D(0.0f, 0.0f)};

  leaf_sheath_length.mean = {0.0f, 0.15f, Curve2D(0.3f, 0.7f)};
  leaf_sheath_length.deviation = {0.0f, 0.0f, Curve2D(0.0f, 0.0f)};

  // Leaf shape parameters.
  leaf_roll_angle.mean = {-1.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_roll_angle.deviation = {0.0f, 6.0f, Curve2D(0.3f, 1.0f)};

  leaf_branching_angle.mean = {0.0f, 55.0f, Curve2D(0.5f, 0.2f)};
  leaf_branching_angle.deviation = {0.0f, 3.0f, Curve2D(0.67f, 0.225f)};

  leaf_curling.mean = {0.0f, 90.0f, Curve2D(0.3f, 0.7f)};
  leaf_curling.deviation = {0.0f, 1.0f, Curve2D(0.0f, 0.0f)};

  leaf_bending.mean = {-180.0f, 180.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_bending_acceleration.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending_acceleration.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_bending_smoothness.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending_smoothness.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_waviness.mean = {0.0f, 20.0f, Curve2D(0.5f, 0.5f)};
  leaf_waviness.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_waviness_frequency.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_waviness_frequency.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  // Internode geometry.
  max_internode_length.mean = {0.0f, 0.45f, Curve2D(0.5f, 0.5f)};
  max_internode_length.deviation = {0.0f, 0.15f, Curve2D(0.5f, 0.5f)};

  max_internode_diameter.mean = {0.0f, 0.028f, Curve2D(0.5f, 0.5f)};
  max_internode_diameter.deviation = {0.0f, 0.0f, Curve2D(0.0f, 0.0f)};

  // Stem.
  stem_tilt_angle.mean = 0.0f;
  stem_tilt_angle.deviation = 0.0f;

  // Along-organ shape curves.
  width_along_stem = Curve2D(1.0f, 0.1f);
  width_along_leaf = Curve2D(0.5f, 0.1f);
  curling_along_leaf = Curve2D(1.0f, 1.0f);
  waviness_along_leaf = Curve2D(0.0f, 0.5f);

  // Panicle (none by default).
  panicle_size.mean = glm::vec2(0.0f);
  panicle_seed_amount.mean = 0.0f;
  panicle_seed_radius.mean = 0.002f;

  // Carbon.
  specific_leaf_area = 20.0f;
  max_stem_reserve_fraction = 0.3f;
}

bool CropDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::TreeNodeEx("Phenology", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= ImGui::DragFloat("Base temperature (C)", &base_temperature, 0.1f, 0.0f, 20.0f);
    changed |= ImGui::DragFloat("Plastochron (GDD)", &plastochron_gdd, 1.0f, 1.0f, 200.0f);
    changed |= ImGui::DragInt("Final leaf number", &final_leaf_number, 1, 1, 30);
    changed |= ImGui::DragFloat("Stem elongation (GDD)", &stem_elongation_gdd, 1.0f);
    changed |= ImGui::DragFloat("Flowering (GDD)", &flowering_gdd, 1.0f);
    changed |= ImGui::DragFloat("Grain filling (GDD)", &grain_filling_gdd, 1.0f);
    changed |= ImGui::DragFloat("Maturity (GDD)", &maturity_gdd, 1.0f);
    changed |= ImGui::DragFloat("Leaf growth duration (GDD)", &leaf_growth_duration_gdd, 1.0f, 1.0f, 500.0f);
    changed |= ImGui::DragFloat("Senescence onset (GDD)", &senescence_onset_gdd, 1.0f, 0.0f, 500.0f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaf Geometry (per-rank)")) {
    changed |= max_leaf_length.OnInspect("Max leaf length");
    changed |= max_leaf_width.OnInspect("Max leaf width");
    changed |= leaf_sheath_length.OnInspect("Sheath length");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaf Shape (per-rank)")) {
    changed |= leaf_roll_angle.OnInspect("Roll angle");
    changed |= leaf_branching_angle.OnInspect("Branching angle");
    changed |= leaf_curling.OnInspect("Curling");
    changed |= leaf_bending.OnInspect("Bending");
    changed |= leaf_bending_acceleration.OnInspect("Bending acceleration");
    changed |= leaf_bending_smoothness.OnInspect("Bending smoothness");
    changed |= leaf_waviness.OnInspect("Waviness");
    changed |= leaf_waviness_frequency.OnInspect("Waviness frequency");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Internode Geometry (per-rank)")) {
    changed |= max_internode_length.OnInspect("Max internode length");
    changed |= max_internode_diameter.OnInspect("Max internode diameter");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Stem")) {
    changed |= stem_tilt_angle.OnInspect("Stem tilt angle");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Shape Curves")) {
    changed |= width_along_stem.OnInspect("Width along stem");
    changed |= width_along_leaf.OnInspect("Width along leaf");
    changed |= curling_along_leaf.OnInspect("Curling along leaf");
    changed |= waviness_along_leaf.OnInspect("Waviness along leaf");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Carbon")) {
    changed |= ImGui::DragFloat("Specific leaf area (m2/kg)", &specific_leaf_area, 0.1f, 1.0f, 100.0f);
    changed |= ImGui::DragFloat("Max stem reserve fraction", &max_stem_reserve_fraction, 0.01f, 0.0f, 1.0f);
    ImGui::TreePop();
  }

  return changed;
}

void CropDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "base_temperature" << YAML::Value << base_temperature;
  out << YAML::Key << "plastochron_gdd" << YAML::Value << plastochron_gdd;
  out << YAML::Key << "final_leaf_number" << YAML::Value << final_leaf_number;
  out << YAML::Key << "stem_elongation_gdd" << YAML::Value << stem_elongation_gdd;
  out << YAML::Key << "flowering_gdd" << YAML::Value << flowering_gdd;
  out << YAML::Key << "grain_filling_gdd" << YAML::Value << grain_filling_gdd;
  out << YAML::Key << "maturity_gdd" << YAML::Value << maturity_gdd;
  out << YAML::Key << "leaf_growth_duration_gdd" << YAML::Value << leaf_growth_duration_gdd;
  out << YAML::Key << "senescence_onset_gdd" << YAML::Value << senescence_onset_gdd;

  max_leaf_length.Save("max_leaf_length", out);
  max_leaf_width.Save("max_leaf_width", out);
  leaf_sheath_length.Save("leaf_sheath_length", out);
  leaf_roll_angle.Save("leaf_roll_angle", out);
  leaf_branching_angle.Save("leaf_branching_angle", out);
  leaf_curling.Save("leaf_curling", out);
  leaf_bending.Save("leaf_bending", out);
  leaf_bending_acceleration.Save("leaf_bending_acceleration", out);
  leaf_bending_smoothness.Save("leaf_bending_smoothness", out);
  leaf_waviness.Save("leaf_waviness", out);
  leaf_waviness_frequency.Save("leaf_waviness_frequency", out);
  max_internode_length.Save("max_internode_length", out);
  max_internode_diameter.Save("max_internode_diameter", out);
  stem_tilt_angle.Save("stem_tilt_angle", out);

  width_along_stem.Save("width_along_stem", out);
  width_along_leaf.Save("width_along_leaf", out);
  curling_along_leaf.Save("curling_along_leaf", out);
  waviness_along_leaf.Save("waviness_along_leaf", out);

  panicle_size.Save("panicle_size", out);
  panicle_seed_amount.Save("panicle_seed_amount", out);
  panicle_seed_radius.Save("panicle_seed_radius", out);

  out << YAML::Key << "specific_leaf_area" << YAML::Value << specific_leaf_area;
  out << YAML::Key << "max_stem_reserve_fraction" << YAML::Value << max_stem_reserve_fraction;
}

void CropDescriptor::Deserialize(const YAML::Node& in) {
  if (in["base_temperature"]) base_temperature = in["base_temperature"].as<float>();
  if (in["plastochron_gdd"]) plastochron_gdd = in["plastochron_gdd"].as<float>();
  if (in["final_leaf_number"]) final_leaf_number = in["final_leaf_number"].as<int>();
  if (in["stem_elongation_gdd"]) stem_elongation_gdd = in["stem_elongation_gdd"].as<float>();
  if (in["flowering_gdd"]) flowering_gdd = in["flowering_gdd"].as<float>();
  if (in["grain_filling_gdd"]) grain_filling_gdd = in["grain_filling_gdd"].as<float>();
  if (in["maturity_gdd"]) maturity_gdd = in["maturity_gdd"].as<float>();
  if (in["leaf_growth_duration_gdd"]) leaf_growth_duration_gdd = in["leaf_growth_duration_gdd"].as<float>();
  if (in["senescence_onset_gdd"]) senescence_onset_gdd = in["senescence_onset_gdd"].as<float>();

  max_leaf_length.Load("max_leaf_length", in);
  max_leaf_width.Load("max_leaf_width", in);
  leaf_sheath_length.Load("leaf_sheath_length", in);
  leaf_roll_angle.Load("leaf_roll_angle", in);
  leaf_branching_angle.Load("leaf_branching_angle", in);
  leaf_curling.Load("leaf_curling", in);
  leaf_bending.Load("leaf_bending", in);
  leaf_bending_acceleration.Load("leaf_bending_acceleration", in);
  leaf_bending_smoothness.Load("leaf_bending_smoothness", in);
  leaf_waviness.Load("leaf_waviness", in);
  leaf_waviness_frequency.Load("leaf_waviness_frequency", in);
  max_internode_length.Load("max_internode_length", in);
  max_internode_diameter.Load("max_internode_diameter", in);
  stem_tilt_angle.Load("stem_tilt_angle", in);

  width_along_stem.Load("width_along_stem", in);
  width_along_leaf.Load("width_along_leaf", in);
  curling_along_leaf.Load("curling_along_leaf", in);
  waviness_along_leaf.Load("waviness_along_leaf", in);

  panicle_size.Load("panicle_size", in);
  panicle_seed_amount.Load("panicle_seed_amount", in);
  panicle_seed_radius.Load("panicle_seed_radius", in);

  if (in["specific_leaf_area"]) specific_leaf_area = in["specific_leaf_area"].as<float>();
  if (in["max_stem_reserve_fraction"]) max_stem_reserve_fraction = in["max_stem_reserve_fraction"].as<float>();
}

}  // namespace digital_agriculture_plugin
