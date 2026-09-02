#pragma once

#include "IPrivateComponent.hpp"
#include "Mesh.hpp"

namespace evo_engine {
class Buffer;
class DescriptorSet;
class GraphicsPipeline;
struct InspectorContext;
}  // namespace evo_engine

namespace universe_package {
using namespace evo_engine;

using StarId = uint64_t;

struct alignas(8) StarBaseSample {
  double orbital_proportion = 0.0;
  double gaussian_x = 0.0;
  double gaussian_y = 0.0;
  double gaussian_z = 0.0;
};

struct alignas(16) StarClusterGpuParameters {
  uint64_t population_revision = 0;
  uint32_t star_count = 0;
  uint32_t padding0 = 0;
  glm::dvec4 ellipse0{};
  glm::dvec4 ellipse1{};
  glm::dvec4 spread_speed{};
  glm::dvec4 speed_tilt{};
  glm::dvec4 tilt_radius{};
  glm::dvec4 center_offset{};
  glm::dvec4 center_position{};
  glm::dvec4 world0{1.0, 0.0, 0.0, 0.0};
  glm::dvec4 world1{0.0, 1.0, 0.0, 0.0};
  glm::dvec4 world2{0.0, 0.0, 1.0, 0.0};
  glm::dvec4 world3{0.0, 0.0, 0.0, 1.0};
  glm::vec4 disk_color_intensity{};
  glm::vec4 core_color_intensity{};
  glm::vec4 center_color_intensity{};
  glm::dvec4 time_padding{};
};

struct alignas(16) StarClusterGpuResult {
  glm::dvec4 world_position_radius{};
  glm::vec4 color_emission{};
  glm::vec4 alpha_padding{1.0f, 0.0f, 0.0f, 0.0f};
};

static_assert(sizeof(StarBaseSample) == 32);
static_assert(sizeof(StarClusterGpuParameters) == 448);
static_assert(sizeof(StarClusterGpuResult) == 64);

struct StarClusterFrameSlot {
  std::shared_ptr<Buffer> parameter_buffer;
  std::shared_ptr<Buffer> result_buffer;
  std::shared_ptr<Buffer> inspection_readback_buffer;
  std::shared_ptr<DescriptorSet> compute_descriptor_set;
  std::shared_ptr<DescriptorSet> render_descriptor_set;
  uint64_t submitted_population_revision = 0;
  uint64_t submitted_frame = 0;
  double submitted_simulation_time = 0.0;
  size_t submitted_count = 0;
  bool readback_pending = false;
};

struct StarClusterRenderPacket {
  std::shared_ptr<DescriptorSet> descriptor_set;
  uint64_t population_revision = 0;
  uint32_t frame_slot = 0;
  uint32_t star_count = 0;
};

class StarCluster final : public IPrivateComponent {
  friend class UniverseLayer;
  friend bool InspectStarCluster(InspectorContext& context, StarCluster& cluster);
  friend void SerializeStarCluster(YAML::Emitter& out, const StarCluster& cluster);
  friend void DeserializeStarCluster(const YAML::Node& in, StarCluster& cluster);

 public:
  uint64_t seed = 1;
  double y_spread = 0.05;
  double xz_spread = 0.015;
  double disk_diameter = 3000.0;
  double disk_eccentricity = 0.5;
  double core_proportion = 0.4;
  double core_eccentricity = 0.7;
  double center_diameter = 10.0;
  double center_eccentricity = 0.3;
  double disk_speed = 1.0;
  double core_speed = 5.0;
  double center_speed = 10.0;
  double disk_tilt_x = 0.0;
  double disk_tilt_z = 0.0;
  double core_tilt_x = 0.0;
  double core_tilt_z = 0.0;
  double center_tilt_x = 0.0;
  double center_tilt_z = 0.0;
  double twist = 360.0;
  glm::dvec3 center_offset{};
  glm::dvec3 center_position{};
  glm::vec3 disk_color{0.0f, 0.0f, 1.0f};
  glm::vec3 core_color{1.0f, 1.0f, 0.0f};
  glm::vec3 center_color{1.0f};
  float disk_emission_intensity = 3.0f;
  float core_emission_intensity = 2.0f;
  float center_emission_intensity = 1.0f;
  float alpha = 1.0f;
  double visual_radius = 0.25;
  double time_scale = 50.0;
  double phase = 1000000.0;
  bool paused = false;

  [[nodiscard]] StarId AddStar();
  [[nodiscard]] std::vector<StarId> AddStars(size_t count);
  bool RemoveStar(StarId id);
  void ClearStars();
  void Reseed(uint64_t new_seed);
  [[nodiscard]] size_t GetStarCount() const;
  [[nodiscard]] size_t GetCapacity() const;
  [[nodiscard]] uint64_t GetPopulationRevision() const;
  [[nodiscard]] uint64_t GetParameterRevision() const;
  [[nodiscard]] const std::vector<StarId>& GetStarIds() const;
  [[nodiscard]] const std::vector<StarBaseSample>& GetBaseSamples() const;
  [[nodiscard]] static StarBaseSample GenerateBaseSample(uint64_t seed, StarId id);
  [[nodiscard]] StarClusterGpuParameters BuildGpuParameters(const glm::dmat4& world_transform,
                                                            double simulation_time) const;
  void MarkParametersDirty();

  void OnCreate() override;
  void OnDestroy() override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;

 private:
  std::vector<StarId> star_ids_;
  std::vector<StarBaseSample> base_samples_;
  std::unordered_map<StarId, size_t> dense_indices_;
  StarId next_id_ = 1;
  uint64_t population_revision_ = 1;
  uint64_t parameter_revision_ = 1;

  size_t capacity_ = 0;
  std::shared_ptr<Buffer> base_sample_buffer_;
  std::vector<StarClusterFrameSlot> frame_slots_;
  bool gpu_resources_dirty_ = true;
  double runtime_time_ = 0.0;
  double last_global_time_ = 0.0;
  bool clock_initialized_ = false;
  uint64_t computed_population_revision_ = 0;
  uint64_t rendered_population_revision_ = 0;
  uint32_t rendered_frame_slot_ = 0;
  uint32_t rendered_count_ = 0;
  uint64_t completed_update_count_ = 0;
  double completed_simulation_time_ = 0.0;
  std::vector<glm::dvec3> completed_world_positions_;
  bool inspection_readback_supported_ = true;
  bool position_readback_requested_ = false;
  std::string last_compute_status_ = "Not submitted";
  std::string last_render_status_ = "Not rendered";
  std::string last_readback_status_ = "No completed readback";

  void RebuildDenseIndex();
  void ResetAuthoringState();
  void ResetRuntimeState();
  [[nodiscard]] double AdvanceClock(double global_time);
  [[nodiscard]] StarClusterRenderPacket BuildRenderPacket(uint32_t frame_index);
  static uint32_t RecordForwardDraw(const StarClusterRenderPacket& packet, VkCommandBuffer command_buffer,
                                    GraphicsPipeline& pipeline);
};

bool InspectStarCluster(InspectorContext& context, StarCluster& cluster);
void SerializeStarCluster(YAML::Emitter& out, const StarCluster& cluster);
void DeserializeStarCluster(const YAML::Node& in, StarCluster& cluster);
}  // namespace universe_package
