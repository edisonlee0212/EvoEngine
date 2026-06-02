#include "CropDescriptor.hpp"
#include "SorghumGenerator.hpp"

namespace digital_agriculture_package {

void CropDescriptor::OnCreate() {
  // ============================================================================
  // Defaults calibrated to Season 11 sorghum generator.
  //
  // Season 11 describes a vegetative-stage sorghum (~14 leaves, ~0.6 m height,
  // no panicle).  With plastochron=40 GDD, that corresponds to roughly GDD 600.
  // The genotype maxima below are set so that a fully-developed leaf matches
  // the Season 11 dimensions, and the whole-plant growth from seedling to
  // maturity is biologically plausible for a short-statured grain sorghum.
  // ============================================================================

  // -- Phenology --
  base_temperature = 8.0f;
  plastochron_gdd = 40.0f;
  final_leaf_number = 16;

  stem_elongation_gdd = 400.0f;
  flowering_gdd = 800.0f;
  grain_filling_gdd = 1000.0f;
  maturity_gdd = 1500.0f;
  leaf_growth_duration_gdd = 120.0f;
  senescence_onset_gdd = 100.0f;

  // -- Leaf geometry (per-rank) --
  // Season 11 leaf_length: [0, 1.16] curve ~(0.254 -> 0.55 peak -> 0.519).
  // Bottom leaves ~0.30 m, mid-rank leaves ~0.64 m, flag leaf ~0.60 m.
  max_leaf_length.mean = {0.0f, 1.16f, Curve2D(0.254f, 0.519f)};
  max_leaf_length.deviation = {0.0f, 0.0f, Curve2D(0.0f, 0.0f)};

  // Season 11 leaf_width: [0, 0.1] flat 0.5 => 0.05 m at all ranks.
  max_leaf_width.mean = {0.0f, 0.1f, Curve2D(0.5f, 0.5f)};
  max_leaf_width.deviation = {0.0f, 0.06f, Curve2D(0.5f, 0.5f)};

  // Sheath length ~10-20% of blade length, increasing basipetally.
  leaf_sheath_length.mean = {0.0f, 0.18f, Curve2D(0.3f, 0.7f)};
  leaf_sheath_length.deviation = {0.0f, 0.0f, Curve2D(0.0f, 0.0f)};

  // -- Leaf shape (per-rank) — from Season 11 --
  // Roll angle: mean centred at 0 (midpoint of [-1,1]), deviation increases acropetally.
  leaf_roll_angle.mean = {-1.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_roll_angle.deviation = {0.0f, 8.0f, Curve2D(0.3f, 1.0f)};

  // Branching angle: ~27 deg bottom to ~11 deg top.
  leaf_branching_angle.mean = {0.0f, 55.0f, Curve2D(0.5f, 0.2f)};
  leaf_branching_angle.deviation = {0.0f, 2.0f, Curve2D(0.565f, 0.239f)};

  // Curling: 27 deg at bottom, 63 deg at top.
  leaf_curling.mean = {0.0f, 90.0f, Curve2D(0.3f, 0.7f)};
  leaf_curling.deviation = {0.0f, 0.0f, Curve2D(0.0f, 0.0f)};

  // Bending: lower leaves droop (~98 deg), top leaves nearly erect (~-1 deg).
  leaf_bending.mean = {-180.0f, 180.0f, Curve2D(0.773f, 0.497f)};
  leaf_bending.deviation = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};

  // Bending acceleration ~0.8 throughout.
  leaf_bending_acceleration.mean = {0.0f, 1.0f, Curve2D(0.781f, 0.813f)};
  leaf_bending_acceleration.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_bending_smoothness.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending_smoothness.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  // Waviness: Season 11 is very subtle (~0.03 m amplitude).
  leaf_waviness.mean = {0.0f, 0.15f, Curve2D(0.193f, 0.194f)};
  leaf_waviness.deviation = {0.0f, 0.2f, Curve2D(0.496f, 0.501f)};

  leaf_waviness_frequency.mean = {0.0f, 0.1f, Curve2D(0.5f, 0.5f)};
  leaf_waviness_frequency.deviation = {0.0f, 0.1f, Curve2D(0.5f, 0.5f)};

  // -- Internode geometry --
  // Calibrated so 16 internodes yield ~1.14 m (45 in) total height at maturity.
  // Lower internodes shorter (0.25 * 0.14 = 0.035 m), upper longer (0.75 * 0.14 = 0.105 m).
  // Sum ≈ 16 * 0.14 * 0.5 = 1.12 m.
  max_internode_length.mean = {0.0f, 0.14f, Curve2D(0.25f, 0.75f)};
  max_internode_length.deviation = {0.0f, 0.01f, Curve2D(0.5f, 0.5f)};

  // Season 11 stem_width = 0.014 (half-width); diameter = 0.028.
  max_internode_diameter.mean = {0.0f, 0.028f, Curve2D(0.5f, 0.5f)};
  max_internode_diameter.deviation = {0.0f, 0.0f, Curve2D(0.0f, 0.0f)};

  // -- Stem --
  stem_tilt_angle.mean = 0.0f;
  stem_tilt_angle.deviation = 1.5f;

  // -- Along-organ shape curves (from Season 11) --
  // Stem tapers from full width at base to ~69% at top.
  width_along_stem = Curve2D(1.0f, 0.688f);
  // Leaf blade: starts ~32% of max width at ligule, tapers to near zero at tip.
  width_along_leaf = Curve2D(0.315f, 0.016f);
  // Uniform curling along the leaf (Season 11 generator uses {1,1}).
  curling_along_leaf = Curve2D(1.0f, 1.0f);
  // Waviness rises toward the tip.
  waviness_along_leaf = Curve2D(0.0f, 0.5f);

  // -- Panicle (absent at Season 11 stage, appears at flowering) --
  panicle_size.mean = glm::vec2(0.0f);
  panicle_seed_amount.mean = 0.0f;
  panicle_seed_radius.mean = 0.002f;

  // -- Carbon --
  specific_leaf_area = 20.0f;
  max_stem_reserve_fraction = 0.3f;
}

bool CropDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  static AssetRef import_sg;
  if (editor_layer->DragAndDropButton<SorghumGenerator>(import_sg, "Import from SorghumGenerator")) {
    if (const auto sg = import_sg.Get<SorghumGenerator>()) {
      InitFromSorghumGenerator(*sg);
      changed = true;
    }
    import_sg = {};
  }

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

void CropDescriptor::InitFromSorghumGenerator(const SorghumGenerator& sg) {
  // -- Leaf count --
  final_leaf_number = static_cast<int>(glm::clamp(sg.leaf_amount.mean, 2.0f, 128.0f));

  // -- Direct PlottedDistribution copies (same parameterization on t = rank / (N-1)) --
  max_leaf_length = sg.leaf_length;
  max_leaf_width = sg.leaf_width;
  leaf_curling = sg.leaf_curling;
  leaf_roll_angle = sg.leaf_roll_angle;
  leaf_branching_angle = sg.leaf_branching_angle;
  leaf_bending = sg.leaf_bending;
  leaf_bending_acceleration = sg.leaf_bending_acceleration;
  leaf_bending_smoothness = sg.leaf_bending_smoothness;
  leaf_waviness = sg.leaf_waviness;
  leaf_waviness_frequency = sg.leaf_waviness_frequency;

  // -- Direct Curve2D copies --
  width_along_stem = sg.width_along_stem;
  width_along_leaf = sg.width_along_leaf;
  curling_along_leaf = sg.curling_along_leaf;
  waviness_along_leaf = sg.waviness_along_leaf;

  // -- Scalar copies --
  stem_tilt_angle = sg.stem_tilt_angle;
  panicle_size = sg.panicle_size;
  panicle_seed_amount = sg.panicle_seed_amount;
  panicle_seed_radius = sg.panicle_seed_radius;

  // -- Conversions --
  // SG internode_length is a SingleDistribution (uniform across all ranks).
  // CD max_internode_length is a PlottedDistribution (per-rank).
  // Create a flat curve at the SG mean value.
  max_internode_length.mean = {0.0f, sg.internode_length.mean, Curve2D(1.0f, 1.0f)};
  max_internode_length.deviation = {0.0f, sg.internode_length.deviation, Curve2D(1.0f, 1.0f)};

  // SG stem_width is half-width; CD max_internode_diameter is full diameter.
  max_internode_diameter.mean = {0.0f, sg.stem_width.mean * 2.0f, Curve2D(0.5f, 0.5f)};
  max_internode_diameter.deviation = {0.0f, sg.stem_width.deviation * 2.0f, Curve2D(0.5f, 0.5f)};

  // -- leaf_sheath_length: no SG counterpart, keep existing defaults --
  // -- Phenology GDD values: kept unchanged --
}

}  // namespace digital_agriculture_package
