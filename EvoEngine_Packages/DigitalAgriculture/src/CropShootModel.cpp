#include "CropShootModel.hpp"

#ifdef ECOSYSLAB_PACKAGE

#include "SorghumLayer.hpp"
#include "SorghumState.hpp"

namespace digital_agriculture_package {

// Deterministic sampling from a PlottedDistribution using our own RNG
// instead of glm::gaussRand (which uses the non-resettable global std::rand).
static float SamplePlotted(const PlottedDistribution<float>& pd, float t, std::mt19937& rng) {
  const float mean_val = pd.mean.GetValue(t);
  const float dev_val = pd.deviation.GetValue(t);
  if (dev_val <= 0.0f)
    return mean_val;
  std::normal_distribution<float> dist(mean_val, dev_val);
  return dist(rng);
}

// ============================================================================
// Lifecycle
// ============================================================================

void CropShootModel::Initialize(const std::shared_ptr<CropDescriptor>& descriptor, const unsigned int seed) {
  if (initialized_)
    Clear();

  descriptor_ = descriptor;
  random_engine_ = std::mt19937(seed);

  // Prepare genotype constants for the skeleton-level data.
  CropSkeletonData skel_data{};
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
  // Apply genotype constants onto the newly constructed skeleton.
  skeleton_.data = skel_data;

  const eco_sys_lab_package::SkeletonNodeHandle root_handle = 0;
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
  // Use SamplePlotted (our own RNG) instead of PlottedDistribution::GetValue
  // (which uses global std::rand and causes jitter across reinitializations).
  const float t = 0.0f;  // normalized rank for the first leaf
  phytomer.leaf.max_length = SamplePlotted(descriptor_->max_leaf_length, t, random_engine_);
  phytomer.leaf.max_width = SamplePlotted(descriptor_->max_leaf_width, t, random_engine_);
  phytomer.leaf.sheath_length = SamplePlotted(descriptor_->leaf_sheath_length, t, random_engine_);
  phytomer.leaf.roll_angle = SamplePlotted(descriptor_->leaf_roll_angle, t, random_engine_);
  phytomer.leaf.max_branching_angle = SamplePlotted(descriptor_->leaf_branching_angle, t, random_engine_);
  phytomer.leaf.branching_angle = 0.0f;  // starts vertical
  phytomer.leaf.curling = SamplePlotted(descriptor_->leaf_curling, t, random_engine_);
  phytomer.leaf.bending = SamplePlotted(descriptor_->leaf_bending, t, random_engine_);
  phytomer.leaf.waviness = SamplePlotted(descriptor_->leaf_waviness, t, random_engine_);
  phytomer.leaf.waviness_frequency = SamplePlotted(descriptor_->leaf_waviness_frequency, t, random_engine_);
  phytomer.leaf.bending_acceleration = SamplePlotted(descriptor_->leaf_bending_acceleration, t, random_engine_);
  phytomer.leaf.bending_smoothness = SamplePlotted(descriptor_->leaf_bending_smoothness, t, random_engine_);
  phytomer.internode.max_length = SamplePlotted(descriptor_->max_internode_length, t, random_engine_);
  phytomer.internode.max_diameter = SamplePlotted(descriptor_->max_internode_diameter, t, random_engine_);

  skeleton_.data.total_phytomers_initiated = 1;

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

void CropShootModel::AdvanceByDeltaGdd(const float delta_gdd) {
  auto& skel_data = skeleton_.data;
  skel_data.cumulative_gdd += delta_gdd;

  // 1. Phenology: check phase transitions.
  if (!skel_data.stem_elongation_started && skel_data.cumulative_gdd >= descriptor_->stem_elongation_gdd) {
    skel_data.stem_elongation_started = true;
    // Use the actual threshold GDD, not the current cumulative_gdd.
    // This ensures correct elongation timing even with large step sizes
    // (e.g. a single GrowByDeltaGdd(760) call from GrowCropToGdd).
    skel_data.stem_elongation_start_gdd = descriptor_->stem_elongation_gdd;
  }
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

void CropShootModel::Grow(const float daily_mean_temperature) {
  if (!initialized_ || !descriptor_)
    return;

  const float delta_gdd = AccumulateGdd(daily_mean_temperature);
  if (delta_gdd <= 0.0f)
    return;

  AdvanceByDeltaGdd(delta_gdd);
}

void CropShootModel::GrowByDeltaGdd(const float delta_gdd) {
  if (!initialized_ || !descriptor_ || delta_gdd <= 0.0f)
    return;

  // Store temperature for any code that reads it, keeping it unchanged.
  AdvanceByDeltaGdd(delta_gdd);
}

float CropShootModel::AccumulateGdd(const float daily_mean_temperature) {
  auto& skel_data = skeleton_.data;
  skel_data.daily_temperature = daily_mean_temperature;
  // Compute delta only — AdvanceByDeltaGdd owns the cumulative_gdd update.
  // Previously this function also incremented cumulative_gdd, which caused a
  // double-accumulation bug when Grow() called AccumulateGdd then AdvanceByDeltaGdd.
  const float delta_gdd = glm::max(0.0f, daily_mean_temperature - skel_data.base_temperature);
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
  // Recalculate threshold each iteration so at most one phytomer is created per threshold crossing.
  while (skel_data.total_phytomers_initiated < skel_data.final_leaf_number) {
    const float next_threshold =
        static_cast<float>(skel_data.total_phytomers_initiated) * skel_data.plastochron_gdd;
    if (skel_data.cumulative_gdd < next_threshold)
      break;
    apex_handle = CreatePhytomer(apex_handle);
  }
  skeleton_.SortLists();
}

eco_sys_lab_package::SkeletonNodeHandle CropShootModel::CreatePhytomer(
    const eco_sys_lab_package::SkeletonNodeHandle parent_handle) {
  auto& skel_data = skeleton_.data;
  const int idx = skel_data.total_phytomers_initiated;
  const float n = static_cast<float>(descriptor_->final_leaf_number - 1);
  const float t = n > 0.0f ? static_cast<float>(idx) / n : 0.0f;

  auto new_handle = skeleton_.Extend(parent_handle, false);  // prolongation, not branching
  auto& node = skeleton_.RefNode(new_handle);

  auto& phytomer = node.data;
  phytomer.phytomer_index = idx;
  phytomer.rank = 0;  // main culm
  // Use the actual plastochron threshold instead of cumulative_gdd so that
  // large-step growth (e.g. single GrowByDeltaGdd call) computes correct
  // per-phytomer thermal ages.
  phytomer.emerged_at_gdd = static_cast<float>(idx) * skel_data.plastochron_gdd;
  phytomer.thermal_age = 0.0f;
  phytomer.leaf.phase = PhytomerPhase::Emerging;

  // Sample genotype targets deterministically using our own RNG
  // to avoid jitter from glm::gaussRand's global std::rand() state.
  phytomer.leaf.max_length = SamplePlotted(descriptor_->max_leaf_length, t, random_engine_);
  phytomer.leaf.max_width = SamplePlotted(descriptor_->max_leaf_width, t, random_engine_);
  phytomer.leaf.sheath_length = SamplePlotted(descriptor_->leaf_sheath_length, t, random_engine_);
  phytomer.leaf.roll_angle = SamplePlotted(descriptor_->leaf_roll_angle, t, random_engine_);        // degrees
  phytomer.leaf.max_branching_angle = SamplePlotted(descriptor_->leaf_branching_angle, t, random_engine_); // degrees (target)
  phytomer.leaf.branching_angle = 0.0f;  // starts vertical, deploys gradually
  phytomer.leaf.curling = SamplePlotted(descriptor_->leaf_curling, t, random_engine_);              // degrees [0,90]
  phytomer.leaf.bending = SamplePlotted(descriptor_->leaf_bending, t, random_engine_);              // degrees [-180,180]
  phytomer.leaf.waviness = SamplePlotted(descriptor_->leaf_waviness, t, random_engine_);
  phytomer.leaf.waviness_frequency = SamplePlotted(descriptor_->leaf_waviness_frequency, t, random_engine_);
  phytomer.internode.max_length = SamplePlotted(descriptor_->max_internode_length, t, random_engine_);
  phytomer.internode.max_diameter = SamplePlotted(descriptor_->max_internode_diameter, t, random_engine_);

  // Bending acceleration and smoothness for proper bending curve construction.
  phytomer.leaf.bending_acceleration = SamplePlotted(descriptor_->leaf_bending_acceleration, t, random_engine_);
  phytomer.leaf.bending_smoothness = SamplePlotted(descriptor_->leaf_bending_smoothness, t, random_engine_);

  // Distichous phyllotaxis: sorghum/maize leaves alternate in two ranks (180°).
  // Add a small random roll deviation on top of the strict alternation.
  if (idx % 2 == 1) {
    phytomer.leaf.roll_angle += 180.0f;
  }

  skel_data.total_phytomers_initiated++;
  return new_handle;
}

void CropShootModel::GrowOrgans(const float delta_gdd) {
  const auto& skel_data = skeleton_.data;
  const auto& sorted = skeleton_.PeekSortedNodeList();
  const float growth_gdd = descriptor_->leaf_growth_duration_gdd;

  float total_leaf_area = 0.0f;

  for (const auto handle : sorted) {
    auto& node = skeleton_.RefNode(handle);
    auto& phytomer = node.data;

    // Compute thermal age and leaf thermal time from absolute timestamps.
    // This produces correct results regardless of step size (incremental or
    // single large jump).
    phytomer.thermal_age = glm::max(0.0f, skel_data.cumulative_gdd - phytomer.emerged_at_gdd);

    auto& leaf = phytomer.leaf;
    auto& internode = phytomer.internode;

    // ---- Leaf blade growth ----
    // Linear progression toward max dimensions over leaf_growth_duration_gdd.
    if (leaf.phase == PhytomerPhase::Growing || leaf.phase == PhytomerPhase::Emerging) {
      leaf.thermal_time_since_emergence = phytomer.thermal_age;

      const float progress = glm::clamp(leaf.thermal_time_since_emergence / growth_gdd, 0.0f, 1.0f);

      leaf.length = progress * leaf.max_length;
      leaf.width = progress * leaf.max_width;
      leaf.leaf_area = leaf.length * leaf.width * 0.75f;  // form factor ~0.75 for grasses

      // --- Leaf deployment: branching angle unfolds from vertical ---
      // In real grasses, the leaf emerges inside the whorl pointing upward (0°),
      // and gradually deploys to its genotype insertion angle as the blade
      // elongates past the ligule of the enclosing sheath.
      // We use an ease-out (sqrt) curve so the leaf opens rapidly at first
      // then settles into its final angle.
      leaf.branching_angle = glm::sqrt(progress) * leaf.max_branching_angle;

      if (leaf.phase == PhytomerPhase::Emerging && leaf.thermal_time_since_emergence > 0.0f)
        leaf.phase = PhytomerPhase::Growing;
    }

    // ---- Internode elongation ----
    // In real grasses, internodes are compressed (rosette) during the vegetative
    // phase and elongate acropetally during the stem elongation phase.
    // We model this as:
    //   - Before stem elongation: internodes grow to a small rosette fraction
    //     (~5% of max), giving the seedling a visible but short stem.
    //   - After stem elongation begins: each internode smoothly transitions from
    //     its current rosette length to full length over one leaf_growth_duration.
    //     The transition uses GDD elapsed since elongation started, so old
    //     internodes don't pop instantly.
    {
      constexpr float rosette_fraction = 0.05f;
      const float age_progress = glm::clamp(phytomer.thermal_age / growth_gdd, 0.0f, 1.0f);

      float target_fraction;
      if (!skel_data.stem_elongation_started) {
        // Vegetative phase: compressed nodes.
        target_fraction = rosette_fraction;
      } else {
        // Stem elongation phase: smoothly ramp from rosette to full length.
        // GDD elapsed since elongation started, normalized over growth_gdd.
        const float elongation_age = skel_data.cumulative_gdd - skel_data.stem_elongation_start_gdd;
        const float elongation_progress = glm::clamp(elongation_age / growth_gdd, 0.0f, 1.0f);
        target_fraction = glm::mix(rosette_fraction, 1.0f, elongation_progress);
      }

      internode.length = age_progress * target_fraction * internode.max_length;
      internode.diameter = age_progress * internode.max_diameter;
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
  // Use actual plant height so leaf starting_point fractions are consistent with
  // the rendered stem length.  SorghumStemState::Apply uses max(4, length/step)
  // for node count, so very small values are safe.
  target->stem.length = glm::max(skel_data.plant_height, 1e-4f);
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
  // Use a safe positive denominator for starting_point so we never divide by zero
  // even when the stem is still in the very early seedling stage (near-zero height).
  // Do NOT gate on plant_height: that caused a visual "explosion" because leaves
  // accumulate thermal time from day 1, so when the gate finally opened the leaf
  // was already centimetres long and appeared to "pop" in at full size.
  const float safe_plant_height = glm::max(skel_data.plant_height, 1e-4f);

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
    leaf_state.starting_point = glm::clamp(cumulative_height / safe_plant_height, 0.0f, 1.0f);
    leaf_state.length = leaf.length;
    // roll_angle and branching_angle are already in degrees (from PlottedDistribution).
    leaf_state.roll_angle = glm::mod(leaf.roll_angle, 360.0f);
    leaf_state.branching_angle = leaf.branching_angle;
    leaf_state.dead = (leaf.phase == PhytomerPhase::Senescent && leaf.chlorophyll_fraction <= 0.01f);

    // Shape curves from descriptor, scaled by per-leaf values.
    // width and waviness use *2.0 to match the SorghumGenerator convention
    // (the Plot2D max_value is half-width; the full bilateral width is 2x).
    leaf_state.width_along_leaf = {0.0f, leaf.width * 2.0f, descriptor_->width_along_leaf};
    leaf_state.waviness_along_leaf = {0.0f, leaf.waviness * 2.0f, descriptor_->waviness_along_leaf};
    leaf_state.waviness_frequency = leaf.waviness_frequency;

    // Curling: clamp to [0,90] deg, normalize to [0,1], then multiply back.
    // This matches SorghumGenerator: clamp(val,0,90)/90 * 90 with a flat {1,1} curve.
    const float curling_clamped = glm::clamp(leaf.curling, 0.0f, 90.0f);
    leaf_state.curling_along_leaf = {0.0f, curling_clamped, {1.0f, 1.0f}};

    // Bending: build the full bezier curve matching SorghumGenerator::Apply().
    const float bending_normalized = (leaf.bending + 180.0f) / 360.0f;
    const float bending_acceleration = leaf.bending_acceleration;
    const float bending_smoothness = leaf.bending_smoothness;
    leaf_state.bending_along_leaf = {-180.0f, 180.0f, {0.5f, bending_normalized}};
    const glm::vec2 middle = glm::mix(glm::vec2(0, bending_normalized), glm::vec2(1, 0.5f), bending_acceleration);
    auto& bending_along_leaf_curve = leaf_state.bending_along_leaf.curve.UnsafeGetValues();
    bending_along_leaf_curve.clear();
    bending_along_leaf_curve.emplace_back(-0.1f, 0.0f);
    bending_along_leaf_curve.emplace_back(0.0f, 0.5f);
    const glm::vec2 left_delta = {middle.x, middle.y - 0.5f};
    bending_along_leaf_curve.push_back(left_delta * (1.0f - bending_smoothness));
    const glm::vec2 right_delta = {middle.x - 1.0f, bending_normalized - middle.y};
    bending_along_leaf_curve.push_back(right_delta * (1.0f - bending_smoothness));
    bending_along_leaf_curve.emplace_back(1.0f, bending_normalized);
    bending_along_leaf_curve.emplace_back(0.1f, 0.0f);

    // Deterministic waviness phase per leaf, derived from phytomer index.
    std::uniform_real_distribution<float> phase_dist(0.0f, 100.0f);
    std::mt19937 leaf_rng(static_cast<unsigned>(phytomer.phytomer_index * 7919 + 1009));
    leaf_state.waviness_period_start = glm::vec2(phase_dist(leaf_rng), phase_dist(leaf_rng));

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

}  // namespace digital_agriculture_package

#endif  // ECOSYSLAB_PACKAGE
