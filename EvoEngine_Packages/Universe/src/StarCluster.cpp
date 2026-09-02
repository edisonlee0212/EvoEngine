#include "StarCluster.hpp"

#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "GraphicsPipeline.hpp"
#include "Platform.hpp"
#include "Serialization.hpp"

using namespace universe_package;

namespace {
uint64_t SplitMix64(uint64_t value) {
  value += 0x9e3779b97f4a7c15ull;
  value = (value ^ (value >> 30u)) * 0xbf58476d1ce4e5b9ull;
  value = (value ^ (value >> 27u)) * 0x94d049bb133111ebull;
  return value ^ (value >> 31u);
}

double UnitSample(const uint64_t seed, const StarId id, const uint64_t stream) {
  const uint64_t bits = SplitMix64(seed ^ SplitMix64(id) ^ SplitMix64(stream));
  return static_cast<double>(bits >> 11u) * (1.0 / 9007199254740992.0);
}

std::pair<double, double> GaussianPair(const uint64_t seed, const StarId id, const uint64_t stream) {
  const double u1 = (std::max)(UnitSample(seed, id, stream), std::numeric_limits<double>::min());
  const double u2 = UnitSample(seed, id, stream + 1u);
  const double radius = std::sqrt(-2.0 * std::log(u1));
  const double angle = glm::two_pi<double>() * u2;
  return {radius * std::cos(angle), radius * std::sin(angle)};
}

template <typename T>
void Read(const YAML::Node& in, const char* name, T& value) {
  if (in[name])
    value = in[name].as<T>();
}
}  // namespace

StarBaseSample StarCluster::GenerateBaseSample(const uint64_t seed, const StarId id) {
  const auto xy = GaussianPair(seed, id, 1u);
  const auto z = GaussianPair(seed, id, 3u);
  return {UnitSample(seed, id, 0u), xy.first, xy.second, z.first};
}

StarId StarCluster::AddStar() {
  const StarId id = next_id_++;
  dense_indices_[id] = star_ids_.size();
  star_ids_.emplace_back(id);
  base_samples_.emplace_back(GenerateBaseSample(seed, id));
  ++population_revision_;
  gpu_resources_dirty_ = true;
  return id;
}

std::vector<StarId> StarCluster::AddStars(const size_t count) {
  std::vector<StarId> ids;
  ids.reserve(count);
  star_ids_.reserve(star_ids_.size() + count);
  base_samples_.reserve(base_samples_.size() + count);
  for (size_t i = 0; i < count; ++i) {
    const StarId id = next_id_++;
    dense_indices_[id] = star_ids_.size();
    star_ids_.emplace_back(id);
    base_samples_.emplace_back(GenerateBaseSample(seed, id));
    ids.emplace_back(id);
  }
  if (count != 0) {
    ++population_revision_;
    gpu_resources_dirty_ = true;
  }
  return ids;
}

bool StarCluster::RemoveStar(const StarId id) {
  const auto found = dense_indices_.find(id);
  if (found == dense_indices_.end())
    return false;
  const size_t index = found->second;
  const size_t last = star_ids_.size() - 1;
  if (index != last) {
    star_ids_[index] = star_ids_[last];
    base_samples_[index] = base_samples_[last];
    dense_indices_[star_ids_[index]] = index;
  }
  star_ids_.pop_back();
  base_samples_.pop_back();
  dense_indices_.erase(found);
  ++population_revision_;
  gpu_resources_dirty_ = true;
  return true;
}

void StarCluster::ClearStars() {
  if (star_ids_.empty())
    return;
  star_ids_.clear();
  base_samples_.clear();
  dense_indices_.clear();
  ++population_revision_;
  gpu_resources_dirty_ = true;
}

void StarCluster::Reseed(const uint64_t new_seed) {
  seed = new_seed;
  for (size_t i = 0; i < star_ids_.size(); ++i)
    base_samples_[i] = GenerateBaseSample(seed, star_ids_[i]);
  ++population_revision_;
  gpu_resources_dirty_ = true;
}

size_t StarCluster::GetStarCount() const {
  return star_ids_.size();
}

size_t StarCluster::GetCapacity() const {
  return capacity_;
}

uint64_t StarCluster::GetPopulationRevision() const {
  return population_revision_;
}

uint64_t StarCluster::GetParameterRevision() const {
  return parameter_revision_;
}

const std::vector<StarId>& StarCluster::GetStarIds() const {
  return star_ids_;
}

const std::vector<StarBaseSample>& StarCluster::GetBaseSamples() const {
  return base_samples_;
}

void StarCluster::MarkParametersDirty() {
  ++parameter_revision_;
}

StarClusterGpuParameters StarCluster::BuildGpuParameters(const glm::dmat4& world_transform,
                                                         const double simulation_time) const {
  const double disk_a = disk_diameter * disk_eccentricity;
  const double disk_b = disk_diameter * (1.0 - disk_eccentricity);
  const double center_a = center_diameter * center_eccentricity;
  const double center_b = center_diameter * (1.0 - center_eccentricity);
  const double core_diameter = center_diameter + (disk_a + disk_b - center_diameter) * core_proportion;
  const double core_a = core_diameter * core_eccentricity;
  const double core_b = core_diameter * (1.0 - core_eccentricity);
  StarClusterGpuParameters result{};
  result.population_revision = population_revision_;
  result.star_count = static_cast<uint32_t>(star_ids_.size());
  result.ellipse0 = {disk_a, disk_b, core_a, core_b};
  result.ellipse1 = {center_a, center_b, core_proportion, twist};
  result.spread_speed = {y_spread, xz_spread, disk_speed, core_speed};
  result.speed_tilt = {center_speed, disk_tilt_x, disk_tilt_z, core_tilt_x};
  result.tilt_radius = {core_tilt_z, center_tilt_x, center_tilt_z, visual_radius};
  result.center_offset = glm::dvec4(center_offset, 0.0);
  result.center_position = glm::dvec4(center_position, 0.0);
  result.world0 = world_transform[0];
  result.world1 = world_transform[1];
  result.world2 = world_transform[2];
  result.world3 = world_transform[3];
  result.disk_color_intensity = glm::vec4(disk_color, disk_emission_intensity);
  result.core_color_intensity = glm::vec4(core_color, core_emission_intensity);
  result.center_color_intensity = glm::vec4(center_color, center_emission_intensity);
  result.time_padding = {simulation_time, static_cast<double>(alpha), 0.0, 0.0};
  return result;
}

double StarCluster::AdvanceClock(const double global_time) {
  if (!clock_initialized_) {
    runtime_time_ = 0.0;
    last_global_time_ = global_time;
    clock_initialized_ = true;
  } else {
    if (!paused)
      runtime_time_ += (global_time - last_global_time_) * time_scale;
    last_global_time_ = global_time;
  }
  return phase + runtime_time_;
}

void StarCluster::RebuildDenseIndex() {
  dense_indices_.clear();
  base_samples_.resize(star_ids_.size());
  StarId largest = 0;
  for (size_t i = 0; i < star_ids_.size(); ++i) {
    dense_indices_[star_ids_[i]] = i;
    base_samples_[i] = GenerateBaseSample(seed, star_ids_[i]);
    largest = (std::max)(largest, star_ids_[i]);
  }
  next_id_ = (std::max)(next_id_, largest + 1u);
}

void StarCluster::ResetAuthoringState() {
  const StarCluster defaults;
  seed = defaults.seed;
  y_spread = defaults.y_spread;
  xz_spread = defaults.xz_spread;
  disk_diameter = defaults.disk_diameter;
  disk_eccentricity = defaults.disk_eccentricity;
  core_proportion = defaults.core_proportion;
  core_eccentricity = defaults.core_eccentricity;
  center_diameter = defaults.center_diameter;
  center_eccentricity = defaults.center_eccentricity;
  disk_speed = defaults.disk_speed;
  core_speed = defaults.core_speed;
  center_speed = defaults.center_speed;
  disk_tilt_x = defaults.disk_tilt_x;
  disk_tilt_z = defaults.disk_tilt_z;
  core_tilt_x = defaults.core_tilt_x;
  core_tilt_z = defaults.core_tilt_z;
  center_tilt_x = defaults.center_tilt_x;
  center_tilt_z = defaults.center_tilt_z;
  twist = defaults.twist;
  center_offset = defaults.center_offset;
  center_position = defaults.center_position;
  disk_color = defaults.disk_color;
  core_color = defaults.core_color;
  center_color = defaults.center_color;
  disk_emission_intensity = defaults.disk_emission_intensity;
  core_emission_intensity = defaults.core_emission_intensity;
  center_emission_intensity = defaults.center_emission_intensity;
  alpha = defaults.alpha;
  visual_radius = defaults.visual_radius;
  time_scale = defaults.time_scale;
  phase = defaults.phase;
  paused = defaults.paused;
  star_ids_.clear();
  base_samples_.clear();
  dense_indices_.clear();
  next_id_ = 1;
  population_revision_ = 1;
  parameter_revision_ = 1;
}

void StarCluster::ResetRuntimeState() {
  capacity_ = 0;
  base_sample_buffer_.reset();
  frame_slots_.clear();
  gpu_resources_dirty_ = true;
  runtime_time_ = 0.0;
  last_global_time_ = 0.0;
  clock_initialized_ = false;
  computed_population_revision_ = 0;
  rendered_population_revision_ = 0;
  rendered_frame_slot_ = 0;
  rendered_count_ = 0;
  completed_update_count_ = 0;
  completed_simulation_time_ = 0.0;
  completed_world_positions_.clear();
  inspection_readback_supported_ = true;
  position_readback_requested_ = false;
  last_compute_status_ = "Not submitted";
  last_render_status_ = "Not rendered";
  last_readback_status_ = "No completed readback";
}

void StarCluster::OnCreate() {
  ResetAuthoringState();
  ResetRuntimeState();
}

void StarCluster::OnDestroy() {
  if (base_sample_buffer_ || !frame_slots_.empty())
    Platform::WaitForFrameSubmissions("Destroy Star Cluster runtime resources");
  ResetRuntimeState();
  ResetAuthoringState();
}

void StarCluster::PostCloneAction(const std::shared_ptr<IPrivateComponent>&) {
  RebuildDenseIndex();
  ResetRuntimeState();
}

StarClusterRenderPacket StarCluster::BuildRenderPacket(const uint32_t frame_index) {
  if (frame_index >= frame_slots_.size())
    return {};
  const auto& slot = frame_slots_[frame_index];
  if (!slot.render_descriptor_set || slot.submitted_count == 0)
    return {};
  rendered_population_revision_ = slot.submitted_population_revision;
  rendered_frame_slot_ = frame_index;
  rendered_count_ = static_cast<uint32_t>(slot.submitted_count);
  last_render_status_ =
      "Registered " + std::to_string(rendered_count_) + " stars from frame slot " + std::to_string(frame_index);
  return {slot.render_descriptor_set, rendered_population_revision_, frame_index, rendered_count_};
}

uint32_t StarCluster::RecordForwardDraw(const StarClusterRenderPacket& packet, const VkCommandBuffer command_buffer,
                                        GraphicsPipeline& pipeline) {
  if (!packet.descriptor_set || packet.star_count == 0)
    return 0;
  pipeline.BindDescriptorSet(command_buffer, 1, packet.descriptor_set->GetVkDescriptorSet());
  Platform::Draw(command_buffer, 6u, packet.star_count);
  return packet.star_count * 6u;
}

bool universe_package::InspectStarCluster(InspectorContext&, StarCluster& cluster) {
  bool changed = false;
  uint64_t edited_seed = cluster.seed;
  if (ImGui::InputScalar("Seed", ImGuiDataType_U64, &edited_seed))
    cluster.Reseed(edited_seed);
  changed |= ImGui::Checkbox("Paused", &cluster.paused);
  changed |= ImGui::DragScalar("Time scale", ImGuiDataType_Double, &cluster.time_scale, 0.1f);
  changed |= ImGui::DragScalar("Phase", ImGuiDataType_Double, &cluster.phase, 1.0f);
  changed |= ImGui::DragScalar("Visual radius", ImGuiDataType_Double, &cluster.visual_radius, 0.01f);
  if (ImGui::TreeNode("Density wave")) {
    changed |= ImGui::DragScalar("Disk diameter", ImGuiDataType_Double, &cluster.disk_diameter, 1.0f);
    changed |= ImGui::DragScalar("Disk eccentricity", ImGuiDataType_Double, &cluster.disk_eccentricity, 0.01f);
    changed |= ImGui::DragScalar("Core proportion", ImGuiDataType_Double, &cluster.core_proportion, 0.01f);
    changed |= ImGui::DragScalar("Core eccentricity", ImGuiDataType_Double, &cluster.core_eccentricity, 0.01f);
    changed |= ImGui::DragScalar("Center diameter", ImGuiDataType_Double, &cluster.center_diameter, 1.0f);
    changed |= ImGui::DragScalar("Center eccentricity", ImGuiDataType_Double, &cluster.center_eccentricity, 0.01f);
    changed |= ImGui::DragScalar("Y spread", ImGuiDataType_Double, &cluster.y_spread, 0.001f);
    changed |= ImGui::DragScalar("XZ spread", ImGuiDataType_Double, &cluster.xz_spread, 0.001f);
    changed |= ImGui::DragScalar("Twist", ImGuiDataType_Double, &cluster.twist, 1.0f);
    changed |= ImGui::DragScalarN("Center offset", ImGuiDataType_Double, &cluster.center_offset.x, 3, 1.0f);
    changed |= ImGui::DragScalarN("Center position", ImGuiDataType_Double, &cluster.center_position.x, 3, 1.0f);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Movement")) {
    changed |= ImGui::DragScalar("Disk speed", ImGuiDataType_Double, &cluster.disk_speed, 0.1f);
    changed |= ImGui::DragScalar("Core speed", ImGuiDataType_Double, &cluster.core_speed, 0.1f);
    changed |= ImGui::DragScalar("Center speed", ImGuiDataType_Double, &cluster.center_speed, 0.1f);
    changed |= ImGui::DragScalar("Disk X tilt", ImGuiDataType_Double, &cluster.disk_tilt_x, 1.0f);
    changed |= ImGui::DragScalar("Disk Z tilt", ImGuiDataType_Double, &cluster.disk_tilt_z, 1.0f);
    changed |= ImGui::DragScalar("Core X tilt", ImGuiDataType_Double, &cluster.core_tilt_x, 1.0f);
    changed |= ImGui::DragScalar("Core Z tilt", ImGuiDataType_Double, &cluster.core_tilt_z, 1.0f);
    changed |= ImGui::DragScalar("Center X tilt", ImGuiDataType_Double, &cluster.center_tilt_x, 1.0f);
    changed |= ImGui::DragScalar("Center Z tilt", ImGuiDataType_Double, &cluster.center_tilt_z, 1.0f);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Colors and emission")) {
    changed |= ImGui::ColorEdit3("Disk color", &cluster.disk_color.x);
    changed |= ImGui::ColorEdit3("Core color", &cluster.core_color.x);
    changed |= ImGui::ColorEdit3("Center color", &cluster.center_color.x);
    changed |= ImGui::DragFloat("Disk emission", &cluster.disk_emission_intensity, 0.01f);
    changed |= ImGui::DragFloat("Core emission", &cluster.core_emission_intensity, 0.01f);
    changed |= ImGui::DragFloat("Center emission", &cluster.center_emission_intensity, 0.01f);
    changed |= ImGui::SliderFloat("Alpha", &cluster.alpha, 0.0f, 1.0f);
    ImGui::TreePop();
  }
  static int add_count = 1000;
  ImGui::DragInt("Population edit count", &add_count, 1.0f, 1, 100000);
  if (ImGui::Button("Add stars"))
    (void)cluster.AddStars(static_cast<size_t>(add_count));
  ImGui::SameLine();
  if (ImGui::Button("Clear stars"))
    cluster.ClearStars();
  if (changed)
    cluster.MarkParametersDirty();
  ImGui::Separator();
  ImGui::Text("Count: %zu, capacity: %zu", cluster.GetStarCount(), cluster.GetCapacity());
  ImGui::Text("Population revision: %llu, parameter revision: %llu",
              static_cast<unsigned long long>(cluster.population_revision_),
              static_cast<unsigned long long>(cluster.parameter_revision_));
  ImGui::Text("Computed revision: %llu", static_cast<unsigned long long>(cluster.computed_population_revision_));
  ImGui::Text("Rendered revision: %llu, slot: %u, count: %u",
              static_cast<unsigned long long>(cluster.rendered_population_revision_), cluster.rendered_frame_slot_,
              cluster.rendered_count_);
  ImGui::TextWrapped("Compute: %s", cluster.last_compute_status_.c_str());
  ImGui::TextWrapped("Render: %s", cluster.last_render_status_.c_str());
  ImGui::TextWrapped("Readback: %s", cluster.last_readback_status_.c_str());
  ImGui::Text("Completed update: %llu at simulation time %.6f",
              static_cast<unsigned long long>(cluster.completed_update_count_), cluster.completed_simulation_time_);
  if (ImGui::TreeNode("Star positions")) {
    cluster.position_readback_requested_ = cluster.inspection_readback_supported_;
    ImGui::Text("Inspection readback: %s", cluster.inspection_readback_supported_ ? "requested" : "unavailable");
    ImGui::Text("Completed GPU positions: %zu", cluster.completed_world_positions_.size());
    if (ImGui::BeginTable("StarPositionTable", 4,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                          ImVec2(0.0f, 260.0f))) {
      ImGui::TableSetupColumn("Star ID");
      ImGui::TableSetupColumn("X");
      ImGui::TableSetupColumn("Y");
      ImGui::TableSetupColumn("Z");
      ImGui::TableHeadersRow();
      ImGuiListClipper clipper;
      const auto position_count = (std::min)(cluster.completed_world_positions_.size(), cluster.star_ids_.size());
      clipper.Begin(static_cast<int>(position_count));
      while (clipper.Step()) {
        for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
          const auto index = static_cast<size_t>(i);
          const auto& position = cluster.completed_world_positions_[index];
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          ImGui::Text("%llu", static_cast<unsigned long long>(cluster.star_ids_[index]));
          ImGui::TableSetColumnIndex(1);
          ImGui::Text("%.6f", position.x);
          ImGui::TableSetColumnIndex(2);
          ImGui::Text("%.6f", position.y);
          ImGui::TableSetColumnIndex(3);
          ImGui::Text("%.6f", position.z);
        }
      }
      ImGui::EndTable();
    }
    ImGui::TreePop();
  }
  return changed;
}

void universe_package::SerializeStarCluster(YAML::Emitter& out, const StarCluster& cluster) {
  Serialization::SerializeVector("star_ids", cluster.star_ids_, out);
#define WRITE_VALUE(name) out << YAML::Key << #name << YAML::Value << cluster.name
  WRITE_VALUE(next_id_);
  WRITE_VALUE(seed);
  WRITE_VALUE(y_spread);
  WRITE_VALUE(xz_spread);
  WRITE_VALUE(disk_diameter);
  WRITE_VALUE(disk_eccentricity);
  WRITE_VALUE(core_proportion);
  WRITE_VALUE(core_eccentricity);
  WRITE_VALUE(center_diameter);
  WRITE_VALUE(center_eccentricity);
  WRITE_VALUE(disk_speed);
  WRITE_VALUE(core_speed);
  WRITE_VALUE(center_speed);
  WRITE_VALUE(disk_tilt_x);
  WRITE_VALUE(disk_tilt_z);
  WRITE_VALUE(core_tilt_x);
  WRITE_VALUE(core_tilt_z);
  WRITE_VALUE(center_tilt_x);
  WRITE_VALUE(center_tilt_z);
  WRITE_VALUE(twist);
  WRITE_VALUE(center_offset);
  WRITE_VALUE(center_position);
  WRITE_VALUE(disk_color);
  WRITE_VALUE(core_color);
  WRITE_VALUE(center_color);
  WRITE_VALUE(disk_emission_intensity);
  WRITE_VALUE(core_emission_intensity);
  WRITE_VALUE(center_emission_intensity);
  WRITE_VALUE(alpha);
  WRITE_VALUE(visual_radius);
  WRITE_VALUE(time_scale);
  WRITE_VALUE(phase);
  WRITE_VALUE(paused);
#undef WRITE_VALUE
}

void universe_package::DeserializeStarCluster(const YAML::Node& in, StarCluster& cluster) {
  Serialization::DeserializeVector("star_ids", cluster.star_ids_, in);
#define READ_VALUE(name) Read(in, #name, cluster.name)
  READ_VALUE(next_id_);
  READ_VALUE(seed);
  READ_VALUE(y_spread);
  READ_VALUE(xz_spread);
  READ_VALUE(disk_diameter);
  READ_VALUE(disk_eccentricity);
  READ_VALUE(core_proportion);
  READ_VALUE(core_eccentricity);
  READ_VALUE(center_diameter);
  READ_VALUE(center_eccentricity);
  READ_VALUE(disk_speed);
  READ_VALUE(core_speed);
  READ_VALUE(center_speed);
  READ_VALUE(disk_tilt_x);
  READ_VALUE(disk_tilt_z);
  READ_VALUE(core_tilt_x);
  READ_VALUE(core_tilt_z);
  READ_VALUE(center_tilt_x);
  READ_VALUE(center_tilt_z);
  READ_VALUE(twist);
  READ_VALUE(center_offset);
  READ_VALUE(center_position);
  READ_VALUE(disk_color);
  READ_VALUE(core_color);
  READ_VALUE(center_color);
  READ_VALUE(disk_emission_intensity);
  READ_VALUE(core_emission_intensity);
  READ_VALUE(center_emission_intensity);
  READ_VALUE(alpha);
  READ_VALUE(visual_radius);
  READ_VALUE(time_scale);
  READ_VALUE(phase);
  READ_VALUE(paused);
#undef READ_VALUE
  cluster.RebuildDenseIndex();
  ++cluster.population_revision_;
  cluster.ResetRuntimeState();
}
