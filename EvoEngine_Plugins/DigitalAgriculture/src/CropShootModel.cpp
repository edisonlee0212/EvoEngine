#include "CropShootModel.hpp"

#ifdef ECOSYSLAB_PLUGIN

#include "SorghumLayer.hpp"
#include "SorghumState.hpp"

namespace digital_agriculture_plugin {

// ============================================================================
// Lifecycle
// ============================================================================

void CropShootModel::Initialize(const std::shared_ptr<CropDescriptor>& descriptor, const unsigned int seed) {
  if (initialized_)
    Clear();

  descriptor_ = descriptor;
  random_engine_ = std::mt19937(seed);

  // Copy genotype constants into skeleton-level data.
  auto& skel_data = skeleton_.data;
  skel_data.base_temperature = descriptor_->base_temperature;
  skel_data.plastochron_gdd = descriptor_->plastochron_gdd;
  skel_data.final_leaf_number = descriptor_->final_leaf_number;
  skel_data.cumulative_gdd = 0.0f;
  skel_data.total_phytomers_initiated = 0;
  skel_data.stem_elongation_started = false;
  skel_data.flowering_started = false;
  skel_data.grain_filling_started = false;

  // Create the very first phytomer (the base node of the skeleton).
  // The Skeleton constructor with initial_node_count=1 creates the root node + flow pair.
  skeleton_ = CropSkeleton(1);
  // Re-apply skeleton-level data after reconstruction.
  skeleton_.data = skel_data;

  const eco_sys_lab_plugin::SkeletonNodeHandle root_handle = 0;
  auto& root = skeleton_.RefNode(root_handle);
  root.info.global_position = glm::vec3(0.0f);
  root.info.global_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  root.info.length = 0.0f;

  auto& phytomer = root.data;
  phytomer.phytomer_index = 0;
  phytomer.rank = 0;
  phytomer.emerged_at_gdd = 0.0f;
  phytomer.thermal_age = 0.0f;
  phytomer.leaf.phase = PhytomerPhase::Emerging;

  // Populate genotype targets for the first phytomer.
  const float t = 0.0f;  // normalized rank for the first leaf
  phytomer.leaf.max_length = descriptor_->max_leaf_length.GetValue(t);
  phytomer.leaf.max_width = descriptor_->max_leaf_width.GetValue(t);
  phytomer.leaf.sheath_length = descriptor_->leaf_sheath_length.GetValue(t);
  phytomer.leaf.roll_angle = descriptor_->leaf_roll_angle.GetValue(t);
  phytomer.leaf.branching_angle = descriptor_->leaf_branching_angle.GetValue(t);
  phytomer.leaf.curling = descriptor_->leaf_curling.GetValue(t);
  phytomer.leaf.bending = descriptor_->leaf_bending.GetValue(t);
  phytomer.leaf.waviness = descriptor_->leaf_waviness.GetValue(t);
  phytomer.leaf.waviness_frequency = descriptor_->leaf_waviness_frequency.GetValue(t);
  phytomer.internode.max_length = descriptor_->max_internode_length.GetValue(t);
  phytomer.internode.max_diameter = descriptor_->max_internode_diameter.GetValue(t);

  skel_data.total_phytomers_initiated = 1;

  skeleton_.SortLists();
  initialized_ = true;
}

void CropShootModel::Clear() {
  skeleton_ = CropSkeleton();
  descriptor_.reset();
  initialized_ = false;
}

// ============================================================================
// Growth
// ============================================================================

void CropShootModel::Grow(const float daily_mean_temperature) {
  if (!initialized_ || !descriptor_)
    return;

  const float delta_gdd = AccumulateGdd(daily_mean_temperature);
  if (delta_gdd <= 0.0f)
    return;

  // 1. Phenology: check phase transitions.
  auto& skel_data = skeleton_.data;
  if (!skel_data.stem_elongation_started && skel_data.cumulative_gdd >= descriptor_->stem_elongation_gdd)
    skel_data.stem_elongation_started = true;
  if (!skel_data.flowering_started && skel_data.cumulative_gdd >= descriptor_->flowering_gdd)
    skel_data.flowering_started = true;
  if (!skel_data.grain_filling_started && skel_data.cumulative_gdd >= descriptor_->grain_filling_gdd)
    skel_data.grain_filling_started = true;

  // 2. Initiate new phytomers if plastochron threshold is crossed.
  InitiatePhytomers(delta_gdd);

  // 3. Grow existing organs.
  GrowOrgans(delta_gdd);

  // 4. Update phytomer phases.
  UpdatePhases();

  // 5. Recalculate geometry (positions, rotations, plant height).
  RecalculateGeometry();
}

float CropShootModel::AccumulateGdd(const float daily_mean_temperature) {
  auto& skel_data = skeleton_.data;
  skel_data.daily_temperature = daily_mean_temperature;
  const float delta_gdd = glm::max(0.0f, daily_mean_temperature - skel_data.base_temperature);
  skel_data.cumulative_gdd += delta_gdd;
  return delta_gdd;
}

void CropShootModel::InitiatePhytomers(const float delta_gdd) {
  auto& skel_data = skeleton_.data;
  if (skel_data.total_phytomers_initiated >= skel_data.final_leaf_number)
    return;

  // Find the topmost (apical) node — the last one in the sorted list.
  const auto& sorted = skeleton_.PeekSortedNodeList();
  if (sorted.empty())
    return;
  auto apex_handle = sorted.back();

  // Check if cumulative GDD has crossed the next plastochron threshold.
  const float next_threshold =
      static_cast<float>(skel_data.total_phytomers_initiated) * skel_data.plastochron_gdd;
  while (skel_data.cumulative_gdd >= next_threshold &&
         skel_data.total_phytomers_initiated < skel_data.final_leaf_number) {
    apex_handle = CreatePhytomer(apex_handle);
    if (skel_data.total_phytomers_initiated >= skel_data.final_leaf_number)
      break;
  }
  skeleton_.SortLists();
}

eco_sys_lab_plugin::SkeletonNodeHandle CropShootModel::CreatePhytomer(
    const eco_sys_lab_plugin::SkeletonNodeHandle parent_handle) {
  auto& skel_data = skeleton_.data;
  const int idx = skel_data.total_phytomers_initiated;
  const float n = static_cast<float>(descriptor_->final_leaf_number - 1);
  const float t = n > 0.0f ? static_cast<float>(idx) / n : 0.0f;

  auto new_handle = skeleton_.Extend(parent_handle, false);  // prolongation, not branching
  auto& node = skeleton_.RefNode(new_handle);

  auto& phytomer = node.data;
  phytomer.phytomer_index = idx;
  phytomer.rank = 0;  // main culm
  phytomer.emerged_at_gdd = skel_data.cumulative_gdd;
  phytomer.thermal_age = 0.0f;
  phytomer.leaf.phase = PhytomerPhase::Emerging;

  // Set genotype targets from descriptor at this rank.
  phytomer.leaf.max_length = descriptor_->max_leaf_length.GetValue(t);
  phytomer.leaf.max_width = descriptor_->max_leaf_width.GetValue(t);
  phytomer.leaf.sheath_length = descriptor_->leaf_sheath_length.GetValue(t);
  phytomer.leaf.roll_angle = descriptor_->leaf_roll_angle.GetValue(t);
  phytomer.leaf.branching_angle = descriptor_->leaf_branching_angle.GetValue(t);
  phytomer.leaf.curling = descriptor_->leaf_curling.GetValue(t);
  phytomer.leaf.bending = descriptor_->leaf_bending.GetValue(t);
  phytomer.leaf.waviness = descriptor_->leaf_waviness.GetValue(t);
  phytomer.leaf.waviness_frequency = descriptor_->leaf_waviness_frequency.GetValue(t);
  phytomer.internode.max_length = descriptor_->max_internode_length.GetValue(t);
  phytomer.internode.max_diameter = descriptor_->max_internode_diameter.GetValue(t);

  // Alternating phyllotaxy: odd leaves offset 180°.
  if (idx % 2 == 1) {
    phytomer.leaf.roll_angle += glm::radians(180.0f);
  }

  skel_data.total_phytomers_initiated++;
  return new_handle;
}

void CropShootModel::GrowOrgans(const float delta_gdd) {
  const auto& skel_data = skeleton_.data;
  const auto& sorted = skeleton_.PeekSortedNodeList();

  float total_leaf_area = 0.0f;

  for (const auto handle : sorted) {
    auto& node = skeleton_.RefNode(handle);
    auto& phytomer = node.data;
    phytomer.thermal_age += delta_gdd;

    auto& leaf = phytomer.leaf;
    auto& internode = phytomer.internode;

    // Leaf growth: linear progression toward max over leaf_growth_duration_gdd.
    if (leaf.phase == PhytomerPhase::Growing || leaf.phase == PhytomerPhase::Emerging) {
      leaf.thermal_time_since_emergence += delta_gdd;

      const float growth_gdd = descriptor_->leaf_growth_duration_gdd;
      const float progress = glm::clamp(leaf.thermal_time_since_emergence / growth_gdd, 0.0f, 1.0f);

      leaf.length = progress * leaf.max_length;
      leaf.width = progress * leaf.max_width;
      leaf.leaf_area = leaf.length * leaf.width * 0.75f;  // form factor ~0.75 for grasses

      if (leaf.phase == PhytomerPhase::Emerging && leaf.thermal_time_since_emergence > 0.0f)
        leaf.phase = PhytomerPhase::Growing;
    }

    // Internode growth: only after stem elongation has started.
    if (skel_data.stem_elongation_started) {
      const float growth_gdd = descriptor_->leaf_growth_duration_gdd;
      const float internode_progress = glm::clamp(phytomer.thermal_age / growth_gdd, 0.0f, 1.0f);
      internode.length = internode_progress * internode.max_length;
      internode.diameter = internode_progress * internode.max_diameter;
    }

    total_leaf_area += leaf.leaf_area;
    node.info.length = internode.length;
    node.info.thickness = internode.diameter * 0.5f;
  }

  skeleton_.data.total_leaf_area = total_leaf_area;
}

void CropShootModel::UpdatePhases() {
  const auto& skel_data = skeleton_.data;
  const auto& sorted = skeleton_.PeekSortedNodeList();

  for (const auto handle : sorted) {
    auto& phytomer = skeleton_.RefNode(handle).data;
    auto& leaf = phytomer.leaf;

    switch (leaf.phase) {
      case PhytomerPhase::Growing:
        if (leaf.thermal_time_since_emergence >= descriptor_->leaf_growth_duration_gdd)
          leaf.phase = PhytomerPhase::Mature;
        break;
      case PhytomerPhase::Mature:
        if (skel_data.cumulative_gdd >= descriptor_->maturity_gdd + descriptor_->senescence_onset_gdd)
          leaf.phase = PhytomerPhase::Senescent;
        break;
      case PhytomerPhase::Senescent: {
        // Simple: chlorophyll decays linearly over senescence_onset_gdd duration.
        const float time_in_senescence =
            skel_data.cumulative_gdd - (descriptor_->maturity_gdd + descriptor_->senescence_onset_gdd);
        leaf.chlorophyll_fraction =
            glm::clamp(1.0f - time_in_senescence / descriptor_->senescence_onset_gdd, 0.0f, 1.0f);
        if (leaf.chlorophyll_fraction <= 0.0f)
          leaf.phase = PhytomerPhase::Dead;
        break;
      }
      default:
        break;
    }
  }
}

void CropShootModel::RecalculateGeometry() {
  const auto& sorted = skeleton_.PeekSortedNodeList();
  if (sorted.empty())
    return;

  float plant_height = 0.0f;

  // Stem direction: take from descriptor tilt angle (small random tilt).
  const glm::vec3 stem_direction = glm::vec3(0.0f, 1.0f, 0.0f);

  glm::vec3 current_position = glm::vec3(0.0f);
  for (const auto handle : sorted) {
    auto& node = skeleton_.RefNode(handle);
    const float internode_length = node.data.internode.length;

    node.info.global_position = current_position;
    node.info.global_rotation = glm::quatLookAt(-stem_direction, glm::vec3(0.0f, 0.0f, 1.0f));
    node.info.length = internode_length;

    current_position += stem_direction * internode_length;
    plant_height += internode_length;
  }

  skeleton_.data.plant_height = plant_height;
}

// ============================================================================
// Access
// ============================================================================

int CropShootModel::GetPhytomerCount() const {
  return skeleton_.data.total_phytomers_initiated;
}

float CropShootModel::GetCumulativeGdd() const {
  return skeleton_.data.cumulative_gdd;
}

// ============================================================================
// Bridge to legacy rendering pipeline
// ============================================================================

void CropShootModel::ToSorghumState(const std::shared_ptr<SorghumState>& target) const {
  if (!initialized_ || !descriptor_)
    return;

  const auto& skel_data = skeleton_.data;
  const auto& sorted = skeleton_.PeekSortedNodeList();

  // -- Stem --
  target->stem.direction = glm::vec3(0.0f, 1.0f, 0.0f);
  // Ensure a minimum stem length so the rendering pipeline always gets valid geometry.
  target->stem.length = glm::max(skel_data.plant_height, 0.01f);
  target->stem.width_along_stem = {0.0f, 0.014f, descriptor_->width_along_stem};

  // Compute actual stem width from the topmost internode diameter.
  float max_diameter = 0.014f;  // sensible fallback
  for (const auto handle : sorted) {
    const auto& p = skeleton_.PeekNode(handle).data;
    max_diameter = glm::max(max_diameter, p.internode.diameter);
  }
  target->stem.width_along_stem.max_value = max_diameter * 0.5f;

  // -- Panicle --
  target->panicle.panicle_size = glm::vec3(0.0f);
  target->panicle.seed_amount = 0;

  // -- Leaves --
  target->leaves.clear();
  if (skel_data.plant_height <= 0.001f)
    return;

  float cumulative_height = 0.0f;
  for (const auto handle : sorted) {
    const auto& phytomer = skeleton_.PeekNode(handle).data;
    const auto& leaf = phytomer.leaf;

    // Skip dead or very small leaves.
    if (leaf.phase == PhytomerPhase::Dead || leaf.length < 0.001f) {
      cumulative_height += phytomer.internode.length;
      continue;
    }

    SorghumLeafState leaf_state;
    leaf_state.index = phytomer.phytomer_index;
    leaf_state.starting_point = glm::clamp(cumulative_height / skel_data.plant_height, 0.0f, 1.0f);
    leaf_state.length = leaf.length;
    leaf_state.roll_angle = glm::degrees(leaf.roll_angle);
    leaf_state.branching_angle = glm::degrees(leaf.branching_angle);
    leaf_state.dead = (leaf.phase == PhytomerPhase::Senescent && leaf.chlorophyll_fraction <= 0.01f);

    // Shape curves from descriptor, scaled by per-leaf values.
    leaf_state.width_along_leaf = {0.0f, leaf.width * 2.0f, descriptor_->width_along_leaf};
    leaf_state.curling_along_leaf = {0.0f, leaf.curling * 90.0f, descriptor_->curling_along_leaf};
    leaf_state.waviness_along_leaf = {0.0f, leaf.waviness * 2.0f, descriptor_->waviness_along_leaf};
    leaf_state.waviness_frequency = leaf.waviness_frequency;

    // Bending: use descriptor bending helper curves.
    const float bending_normalized = (leaf.bending + 180.0f) / 360.0f;
    leaf_state.bending_along_leaf = {-180.0f, 180.0f, {0.5f, bending_normalized}};

    // Random waviness phase per leaf.
    std::uniform_real_distribution<float> dist(0.0f, 100.0f);
    auto rng = random_engine_;  // copy for const
    leaf_state.waviness_period_start = glm::vec2(dist(rng), dist(rng));

    target->leaves.push_back(leaf_state);
    cumulative_height += phytomer.internode.length;
  }
}

void CropShootModel::ToSorghumDescriptor(const std::shared_ptr<SorghumDescriptor>& target) const {
  // First convert to SorghumState, then use its existing Apply() pipeline.
  auto temp_state = std::make_shared<SorghumState>();
  ToSorghumState(temp_state);
  temp_state->Apply(target);
}

// ============================================================================
// Serialization (placeholder)
// ============================================================================

void CropShootModel::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::BeginMap;
  out << YAML::Key << "initialized" << YAML::Value << initialized_;
  out << YAML::Key << "cumulative_gdd" << YAML::Value << skeleton_.data.cumulative_gdd;
  out << YAML::Key << "total_phytomers_initiated" << YAML::Value << skeleton_.data.total_phytomers_initiated;
  out << YAML::EndMap;
}

void CropShootModel::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& node = in[name];
    if (node["initialized"])
      initialized_ = node["initialized"].as<bool>();
  }
}

}  // namespace digital_agriculture_plugin

#endif  // ECOSYSLAB_PLUGIN
