#include "MaizeGenerator.hpp"
#include "ProjectManager.hpp"
#include "MaizeLayer.hpp"

#include "Plot2D.hpp"
#include "Scene.hpp"
#include "Maize.hpp"
#include "MaizeSpline.hpp"
#include "Times.hpp"

using namespace digital_agriculture_plugin;

void MaizeGenerator::OnCreate() {
  // Initialize with the same defaults as SorghumGenerator (reuse sorghum data for now).

  stem_tilt_angle.mean = 0.0f;
  stem_tilt_angle.deviation = 0.0f;
  internode_length.mean = 0.449999988f;
  internode_length.deviation = 0.150000006f;
  stem_width.mean = 0.0140000004f;
  stem_width.deviation = 0.0f;

  leaf_amount.mean = 9.0f;
  leaf_amount.deviation = 1.0f;

  leaf_starting_point.mean = {0.0f, 1.0f, Curve2D(0.1f, 1.0f)};
  leaf_starting_point.deviation = {0.0f, 1.0f, Curve2D(0.0f, 0.0f)};

  leaf_curling.mean = {0.0f, 90.0f, Curve2D(0.3f, 0.7f)};
  leaf_curling.deviation = {0.0f, 1.0f, Curve2D(0.0f, 0.0f)};
  leaf_roll_angle.mean = {-1.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_roll_angle.deviation = {0.0f, 6.0f, Curve2D(0.3f, 1.0f)};

  leaf_branching_angle.mean = {0.0f, 55.0f, Curve2D(0.5f, 0.2f)};
  leaf_branching_angle.deviation = {0.0f, 3.0f, Curve2D(0.67f, 0.225f)};

  leaf_bending.mean = {-180.0f, 180.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_bending_acceleration.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending_smoothness.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending_acceleration.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_waviness.mean = {0.0f, 20.0f, Curve2D(0.5f, 0.5f)};
  leaf_waviness.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_waviness_frequency.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_waviness_frequency.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_length.mean = {0.0f, 2.5f, Curve2D(0.165f, 0.247f)};
  leaf_length.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_width.mean = {0.0f, 0.075f, Curve2D(0.5f, 0.5f)};
  leaf_width.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  width_along_stem = Curve2D(1.0f, 0.1f);
  width_along_leaf = Curve2D(0.5f, 0.1f);
  waviness_along_leaf = Curve2D(0.0f, 0.5f);
}

void MaizeGenerator::Apply(const std::shared_ptr<MaizeState>& target_maize_state, const unsigned seed) const {
  // This is a direct copy of SorghumGenerator::Apply — evaluates all curves at full maturity.
  if (seed > 0)
    srand(seed);

  // No panicle for maize
  target_maize_state->panicle.seed_amount = 0;
  target_maize_state->panicle.panicle_size = glm::vec3(0);
  target_maize_state->panicle.seed_radius = 0;

  // Stem
  constexpr auto up_direction = glm::vec3(0, 1, 0);
  auto front_direction = glm::vec3(0, 0, -1);
  front_direction = glm::rotate(front_direction, glm::radians(glm::linearRand(0.0f, 360.0f)), up_direction);

  target_maize_state->stem.direction = glm::normalize(glm::rotate(
      up_direction, glm::radians(glm::gaussRand(stem_tilt_angle.mean, stem_tilt_angle.deviation)), front_direction));
  const int leaf_size = static_cast<int>(glm::clamp(leaf_amount.GetValue(), 2.0f, 128.0f));
  target_maize_state->stem.length =
      internode_length.GetValue() * static_cast<float>(leaf_size) / (1.f - leaf_starting_point.GetValue(0));
  target_maize_state->stem.width_along_stem = {0.0f, stem_width.GetValue(), width_along_stem};

  // Leaves
  target_maize_state->leaves.resize(leaf_size);
  for (int leaf_index = 0; leaf_index < leaf_size; leaf_index++) {
    const float step = static_cast<float>(leaf_index) / (static_cast<float>(leaf_size) - 1.0f);
    auto& leaf_state = target_maize_state->leaves[leaf_index];
    leaf_state.index = leaf_index;
    leaf_state.starting_point = leaf_starting_point.GetValue(step);
    leaf_state.length = leaf_length.GetValue(step);
    if (leaf_state.length == 0.0f)
      continue;

    leaf_state.waviness_along_leaf = {0.0f, leaf_waviness.GetValue(step) * 2.0f, waviness_along_leaf};
    leaf_state.width_along_leaf = {0.0f, leaf_width.GetValue(step) * 2.0f, width_along_leaf};
    const auto curling = glm::clamp(leaf_curling.GetValue(step), 0.0f, 90.0f) / 90.0f;
    leaf_state.curling_along_leaf = {0.0f, curling * 90.0f, {1.f, 1.f}};
    leaf_state.branching_angle = leaf_branching_angle.GetValue(step);
    leaf_state.roll_angle = glm::mod((leaf_index % 2) * 180.0f + leaf_roll_angle.GetValue(step), 360.0f);
    auto bending = leaf_bending.GetValue(step);
    bending = (bending + 180) / 360.0f;
    const auto bending_acceleration = leaf_bending_acceleration.GetValue(step);
    const auto bending_smoothness = leaf_bending_smoothness.GetValue(step);

    leaf_state.bending_along_leaf = {-180.0f, 180.0f, {0.5f, bending}};
    const glm::vec2 middle = glm::mix(glm::vec2(0, bending), glm::vec2(1, 0.5f), bending_acceleration);
    auto& bending_along_leaf_curve = leaf_state.bending_along_leaf.curve.UnsafeGetValues();
    bending_along_leaf_curve.clear();
    bending_along_leaf_curve.emplace_back(-0.1f, 0.0f);
    bending_along_leaf_curve.emplace_back(0.0f, 0.5f);
    glm::vec2 left_delta = {middle.x, middle.y - 0.5f};
    bending_along_leaf_curve.push_back(left_delta * (1.0f - bending_smoothness));
    glm::vec2 right_delta = {middle.x - 1.0f, bending - middle.y};
    bending_along_leaf_curve.push_back(right_delta * (1.0f - bending_smoothness));
    bending_along_leaf_curve.emplace_back(1.0f, bending);
    bending_along_leaf_curve.emplace_back(0.1f, 0.0f);

    leaf_state.waviness_frequency = leaf_waviness_frequency.GetValue(step);
    leaf_state.waviness_period_start = glm::vec2(glm::linearRand(0.f, 100.f), glm::linearRand(0.f, 100.f));
  }
}

void MaizeGenerator::ApplyGrowth(const std::shared_ptr<MaizeState>& target_maize_state, float age) const {
  age = glm::clamp(age, 0.0f, 1.0f);
  const int total_leaves = static_cast<int>(target_maize_state->leaves.size());
  if (total_leaves == 0)
    return;

  // -- Plastochron model --
  // One plastochron = 1.0 / total_leaves of the total age range.
  // Leaf i emerges at age = i * plastochron.
  // Each leaf takes exactly 2 plastochrons to reach full size.
  const float plastochron = 1.0f / (static_cast<float>(total_leaves) + 1.0f);

  // Stem elongation is completely linear.
  const float full_stem_length = target_maize_state->stem.length;
  target_maize_state->stem.length = full_stem_length * age + 0.1;

  // Scale stem width from 0.5 to 1.0 based on age
  target_maize_state->stem.width_along_stem.max_value *= glm::mix(0.5f, 1.0f, age);

  // Zero out panicle (tassel omitted).
  target_maize_state->panicle.seed_amount = 0;
  target_maize_state->panicle.panicle_size = glm::vec3(0);

  // Process each leaf.
  for (int i = 0; i < total_leaves; i++) {
    auto& leaf = target_maize_state->leaves[i];
    // Scale plastochron so that the last leaf finishes exactly at age 1.0
    // Last leaf index is i = total_leaves - 1.
    // Emergence = i * P. Maturity = i*P + 2*P = (i+2)*P.
    // We want Maturity(last_leaf) = 1.0
    // (total_leaves - 1 + 2) * P_adjusted = 1.0
    // (total_leaves + 1) * P_adjusted = 1.0
    // P_adjusted = 1.0 / (total_leaves + 1)
    
    // Original plastochron logic:
    // const float plastochron = 1.0f / static_cast<float>(total_leaves); 
    // This assumes plastochron divides the age 0..1 into N segments, but leaves need 2 segments to grow.
    
    // New plastochron calculation to ensure full maturity at age 1.0:
    const float adjusted_plastochron = 1.0f / (static_cast<float>(total_leaves) + 1.0f);
    
    const float emergence_time = static_cast<float>(i) * adjusted_plastochron;
    const float maturity_time = emergence_time + 2.0f * adjusted_plastochron;

    if (age < emergence_time + 0.1) {
      // Leaf has not emerged yet — mark as dead/invisible.
      leaf.dead = true;
      leaf.length = 0.0f;
      continue;
    }

    leaf.dead = false;

    // Individual leaf growth factor: 0 at emergence, 1 at maturity.
    const float leaf_growth = glm::clamp((age - emergence_time) / (maturity_time - emergence_time), 0.0f, 1.0f);

    // Leaves take 3 plastochrons to reach total branching angle.
    const float branching_maturity_time = emergence_time + 3.0f * adjusted_plastochron;
    const float branching_growth =
        glm::clamp((age - emergence_time) / (branching_maturity_time - emergence_time), 0.0f, 1.0f);

    // Leaves width scales from 0.25 to 1.0 in 3 plastochrons.
    const float width_maturity_time = emergence_time + 3.0f * adjusted_plastochron;
    const float width_growth_factor =
        glm::clamp((age - emergence_time) / (width_maturity_time - emergence_time), 0.0f, 1.0f);
    const float width_scale = glm::mix(0.25f, 1.0f, width_growth_factor);

    // Scale leaf length by growth factor.
    leaf.length *= leaf_growth;

    // Scale branching angle by growth factor (starts upright, opens to full angle).
    leaf.branching_angle *= branching_growth;

    // Gravity bending remains static — no modification to bending_along_leaf.

    // Scale width proportionally so the leaf looks natural during growth.
    // Base scale is leaf_growth (0->1) to ensure it starts at 0, combined with width_scale (0.25->1).
    leaf.width_along_leaf.max_value *= leaf_growth * width_scale;

    // Store the growth progress of the leaf to be used later in geometry generation for scaling.
    leaf.current_growth = leaf_growth;

    // Adjust starting point along stem to match current stem length.
    // The starting_point is a 0-1 ratio along the full stem; it stays the same
    // since we already scaled the stem length.
  }
}

Entity MaizeGenerator::CreateEntity(const unsigned int seed) {
  const auto scene = Application::GetActiveScene();
  const auto entity = scene->CreateEntity(GetTitle());
  const auto maize = scene->GetOrSetPrivateComponent<Maize>(entity).lock();
  const auto maize_state = AssetManager::CreateTemporaryAsset<MaizeState>();
  
  maize->plant_age = 1.0f; // Default age
  Apply(maize_state, seed);
  ApplyGrowth(maize_state, maize->plant_age);
  
  maize->maize_state = maize_state;
  maize->maize_generator = GetSelf();
  maize->GenerateGeometryEntities(MaizeMeshGeneratorSettings{});
  return entity;
}

bool MaizeGenerator::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::Button("Instantiate")) {
    preview_entity_ = CreateEntity();
    changed = true;
  }

  if (ImGui::Button("Import from Sorghum Asset")) {
    // Attempt to load the season12_model.yaml
    try {
      // Assuming the path relative to the project root or absolute path
      const std::vector<std::string> search_paths = {
          "Project/Assets/SorghumModels/season12_model.yaml",
          "Assets/SorghumModels/season12_model.yaml",
          "season12_model.yaml"};

      std::filesystem::path found_path;
      for (const auto& path : search_paths) {
        if (std::filesystem::exists(path)) {
          found_path = path;
          break;
        }
      }

      if (!found_path.empty()) {
        YAML::Node imported_data = YAML::LoadFile(found_path.string());
        Deserialize(imported_data);
        EVOENGINE_LOG("Successfully copied Sorghum distributions from " + found_path.string() +
                      " into MaizeGenerator.");
        changed = true;
      } else {
        // Fallback: Open file dialog so user can pick it
        FileUtils::OpenFile(
            "Import Sorghum Asset", "Sorghum Generator", {".sg", ".yaml"},
            [&](const std::filesystem::path& path) {
              try {
                YAML::Node imported_data = YAML::LoadFile(path.string());
                Deserialize(imported_data);
                EVOENGINE_LOG("Successfully copied Sorghum distributions from " + path.string() +
                              " into MaizeGenerator.");
                // We can't set changed = true here effectively because lambda is async or executed later,
                // but for immediacy in UI we might need a flag. In this specific framework, callbacks are usually immediate.
              } catch (const std::exception& e) {
                EVOENGINE_ERROR("Failed to load Sorghum YAML: " + std::string(e.what()));
              }
            },
            false);
      }
    } catch (const std::exception& e) {
      EVOENGINE_ERROR("Failed to load Sorghum YAML: " + std::string(e.what()));
    }
  }

  ImGui::Separator();

  static bool show_params = false;
  ImGui::Checkbox("Show morphology parameters", &show_params);
  if (show_params) {
    if (ImGui::TreeNodeEx("Stem settings", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (stem_tilt_angle.OnInspect("Stem tilt angle", 0.001f, "The tilt angle for stem"))
        changed = true;
      if (internode_length.OnInspect("Length", 0.01f, "The length of the stem"))
        changed = true;
      if (stem_width.OnInspect("Width", 0.001f, "The overall width of the stem"))
        changed = true;
      if (ImGui::TreeNode("Stem Details")) {
        if (width_along_stem.OnInspect("Width along stem"))
          changed = true;
        ImGui::TreePop();
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Leaves settings", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (leaf_amount.OnInspect("Num of leaves", 1.0f, "The total amount of leaves"))
        changed = true;

      static PlottedDistributionSettings leaf_starting_point_settings = {
          0.01f, {0.01f, false, true, ""}, {0.01f, false, false, ""}, "Starting point of each leaf along stem."};
      if (this->leaf_starting_point.OnInspect("Starting point along stem", leaf_starting_point_settings))
        changed = true;

      static PlottedDistributionSettings leaf_curling_settings = {
          0.01f, {0.01f, false, true, ""}, {0.01f, false, false, ""}, "The leaf curling."};
      if (this->leaf_curling.OnInspect("Leaf curling", leaf_curling_settings))
        changed = true;

      static PlottedDistributionSettings leaf_roll_angle_settings = {0.01f, {}, {}, "The polar angle of leaf."};
      if (this->leaf_roll_angle.OnInspect("Roll angle", leaf_roll_angle_settings))
        changed = true;

      static PlottedDistributionSettings leaf_branching_angle_settings = {
          0.01f, {}, {}, "The branching angle of the leaf."};
      if (this->leaf_branching_angle.OnInspect("Branching angle", leaf_branching_angle_settings))
        changed = true;

      static PlottedDistributionSettings leaf_bending_settings = {
          1.0f, {1.0f, false, true, ""}, {}, "The bending of the leaf."};
      if (this->leaf_bending.OnInspect("Bending", leaf_bending_settings))
        changed = true;

      static PlottedDistributionSettings leaf_bending_accel_settings = {
          0.01f, {0.01f, false, true, ""}, {}, "Changes of bending along the leaf."};
      if (this->leaf_bending_acceleration.OnInspect("Bending acceleration", leaf_bending_accel_settings))
        changed = true;

      static PlottedDistributionSettings leaf_bending_smooth_settings = {
          0.01f, {0.01f, false, true, ""}, {}, "Smoothness of bending along the leaf."};
      if (this->leaf_bending_smoothness.OnInspect("Bending smoothness", leaf_bending_smooth_settings))
        changed = true;

      if (leaf_waviness.OnInspect("Waviness"))
        changed = true;
      if (leaf_waviness_frequency.OnInspect("Waviness Frequency"))
        changed = true;
      if (leaf_length.OnInspect("Length"))
        changed = true;
      if (leaf_width.OnInspect("Width"))
        changed = true;

      if (ImGui::TreeNode("Per leaf settings")) {
        if (ImGui::TreeNode("Width along leaf")) {
          if (width_along_leaf.OnInspect("Width along leaf"))
            changed = true;
          ImGui::TreePop();
        }
        if (ImGui::TreeNode("Waviness along leaf")) {
          if (waviness_along_leaf.OnInspect("Waviness along leaf"))
            changed = true;
          ImGui::TreePop();
        }
        if (ImGui::TreeNode("Curling along leaf")) {
          if (curling_along_leaf.OnInspect("Curling along leaf"))
            changed = true;
          ImGui::TreePop();
        }
        ImGui::TreePop();
      }
      ImGui::TreePop();
    }
  }

  // Invalidate cached state when morphology params change (but not from age slider).
  if (changed && cached_state_) {
    cached_state_.reset();
  }

  return changed;
}

void MaizeGenerator::Serialize(YAML::Emitter& out) const {
  stem_tilt_angle.Save("stem_tilt_angle", out);
  internode_length.Save("internode_length", out);
  stem_width.Save("stem_width", out);

  leaf_amount.Save("leaf_amount", out);
  leaf_starting_point.Save("leaf_starting_point", out);
  leaf_curling.Save("leaf_curling", out);

  leaf_roll_angle.Save("leaf_roll_angle", out);
  leaf_branching_angle.Save("leaf_branching_angle", out);

  leaf_bending.Save("leaf_bending", out);
  leaf_bending_acceleration.Save("leaf_bending_acceleration", out);
  leaf_bending_smoothness.Save("leaf_bending_smoothness", out);
  leaf_waviness.Save("leaf_waviness", out);
  leaf_waviness_frequency.Save("leaf_waviness_frequency", out);
  leaf_length.Save("leaf_length", out);
  leaf_width.Save("leaf_width", out);

  width_along_stem.Save("width_along_stem", out);
  width_along_leaf.Save("width_along_leaf", out);
  waviness_along_leaf.Save("waviness_along_leaf", out);
  curling_along_leaf.Save("curling_along_leaf", out);
}

void MaizeGenerator::Deserialize(const YAML::Node& in) {
  stem_tilt_angle.Load("stem_tilt_angle", in);
  internode_length.Load("internode_length", in);
  stem_width.Load("stem_width", in);

  leaf_amount.Load("leaf_amount", in);
  leaf_starting_point.Load("leaf_starting_point", in);
  leaf_curling.Load("leaf_curling", in);

  leaf_roll_angle.Load("leaf_roll_angle", in);
  leaf_branching_angle.Load("leaf_branching_angle", in);

  leaf_bending.Load("leaf_bending", in);
  leaf_bending_acceleration.Load("leaf_bending_acceleration", in);
  leaf_bending_smoothness.Load("leaf_bending_smoothness", in);
  leaf_waviness.Load("leaf_waviness", in);
  leaf_waviness_frequency.Load("leaf_waviness_frequency", in);
  leaf_length.Load("leaf_length", in);
  leaf_width.Load("leaf_width", in);

  width_along_stem.Load("width_along_stem", in);
  width_along_leaf.Load("width_along_leaf", in);
  waviness_along_leaf.Load("waviness_along_leaf", in);
  curling_along_leaf.Load("curling_along_leaf", in);
}

std::shared_ptr<Texture2D> MaizeGenerator::GenerateThumbnailTexture() {
  // Reuse sorghum generator icon for now.
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(std::filesystem::absolute(std::filesystem::path("./DigitalAgricultureResources") /
                                                "Icons/SorghumGenerator.png"));
  }
  return thumbnail;
}
