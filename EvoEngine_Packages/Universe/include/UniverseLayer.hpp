#pragma once

#include "Material.hpp"
#include "PerlinNoiseStage.hpp"
#include "PlanetTerrain.hpp"
#include "StarCluster.hpp"
#include "StarDemoCamera.hpp"
#include "StarFollow.hpp"
#include "StarPicking.hpp"
#include "Strands.hpp"
#include "TerrainChunk.hpp"

namespace universe_package {
using namespace evo_engine;

struct alignas(8) StarBaseSample {
  double orbital_proportion = 0.0;
  double gaussian_x = 0.0;
  double gaussian_y = 0.0;
  double gaussian_z = 0.0;
  double gaussian_radius = 0.0;
  double orbital_phase = 0.0;
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
  glm::dvec4 center_offset{};  // w: normalized radius deviation.
  glm::dvec4 center_position{};
  glm::dvec4 world0{1.0, 0.0, 0.0, 0.0};
  glm::dvec4 world1{0.0, 1.0, 0.0, 0.0};
  glm::dvec4 world2{0.0, 0.0, 1.0, 0.0};
  glm::dvec4 world3{0.0, 0.0, 0.0, 1.0};
  glm::vec4 disk_color_intensity{};
  glm::vec4 core_color_intensity{};
  glm::vec4 center_color_intensity{};
  glm::dvec4 time_padding{};  // time, alpha, minimum radius, maximum radius.
};

struct alignas(16) StarClusterGpuResult {
  glm::dvec4 world_position_radius{};
  glm::vec4 color_emission{};
  glm::vec4 alpha_padding{1.0f, 0.0f, 0.0f, 0.0f};
};

static_assert(sizeof(StarBaseSample) == 48);
static_assert(sizeof(StarClusterGpuParameters) == 448);
static_assert(sizeof(StarClusterGpuResult) == 64);

struct StarClusterComputePushConstant {
  uint32_t parameter_index = 0;
  uint32_t star_offset = 0;
};
static_assert(sizeof(StarClusterComputePushConstant) == 8);

struct StarClusterInput {
  std::shared_ptr<StarCluster> cluster;
  glm::dmat4 world_transform{1.0};
  bool enabled = true;
};

struct StarClusterRange {
  uint64_t identity = 0;
  uint64_t seed = 0;
  uint32_t offset = 0;
  uint32_t count = 0;
  uint64_t layout_revision = 0;
  bool operator==(const StarClusterRange& other) const {
    return identity == other.identity && seed == other.seed && offset == other.offset && count == other.count &&
           layout_revision == other.layout_revision;
  }
};

struct StarOrbit {
  double proportion = 0, length = 0;
  uint64_t capacity = 0;
  uint32_t occupied = 0;
  std::vector<double> arc_lengths;
};

struct StarOrbitLayout {
  std::array<double, 7> geometry{};
  std::vector<StarOrbit> orbits;
  uint64_t capacity = 0, revision = 0;
  double radial_spacing = 0;
  std::string status;
  bool Update(const StarCluster& cluster);
  std::vector<StarBaseSample> Allocate(uint64_t seed, uint32_t count);
};

// CPU state owned by UniverseLayer; separate from GPU allocations for deterministic tests.
struct StarClusterBatch {
  struct Clock {
    std::weak_ptr<StarCluster> component;
    uint64_t component_handle = 0;
    uint64_t identity = 0;
    double elapsed = 0.0;
    double last_global_time = 0.0;
    StarOrbitLayout layout;
    uint32_t active_count = 0;
  };
  std::unordered_map<const StarCluster*, Clock> clocks;
  std::vector<StarBaseSample> samples;
  std::vector<StarClusterRange> ranges;
  std::vector<StarClusterGpuParameters> parameters;
  std::vector<glm::dvec3> gaussian_bounds;
  uint64_t population_revision = 0;
  uint64_t next_identity = 1;
  bool Update(const std::vector<StarClusterInput>& inputs, double global_time, uint64_t maximum_stars = UINT32_MAX);
};

StarBaseSample GenerateStarBaseSample(uint64_t seed, uint32_t ordinal);
StarClusterGpuParameters BuildStarClusterParameters(const StarCluster& cluster, const glm::dmat4& world_transform,
                                                    double simulation_time);
void ApplyStarRadiusScale(StarClusterGpuParameters& parameters, double scale);
inline constexpr double kGalaxyDisplayScale = 0.001;
void ApplyStarDisplayFrame(StarClusterGpuParameters& parameters, const glm::dmat4& display_transform,
                           double radius_scale);
glm::vec2 StarDistanceCompression(float far_distance);
double CompressStarDistance(double distance, double start, double limit);
void ConfigureStarRenderStates(GraphicsPipeline& pipeline, const glm::ivec4& viewport, bool depth_write);

struct StarBatchFrameSlot {
  std::shared_ptr<Buffer> parameter_buffer;
  std::shared_ptr<Buffer> result_buffer;
  std::shared_ptr<DescriptorSet> compute_descriptor_set;
  std::shared_ptr<DescriptorSet> render_descriptor_set;
};

struct StarBatchRenderPacket {
  std::shared_ptr<DescriptorSet> descriptor_set;
  uint64_t population_revision = 0;
  uint32_t frame_slot = 0;
  uint32_t star_count = 0;
  bool depth_write = true;
  float fade_strength = 1.0f;
};

enum class StarOrbitDisplay { All, Occupied, Selected };

std::vector<double> SelectStarOrbitProportions(const StarOrbitLayout& layout, StarOrbitDisplay mode,
                                               const StarPickSnapshot& selected, const StarClusterRange& range,
                                               const std::vector<StarBaseSample>& samples);

struct StarOrbitStrandCache {
  StarClusterGpuParameters key{};
  std::vector<double> proportions;
  std::vector<StrandPoint> points;
  std::vector<uint32_t> starts;
  std::shared_ptr<Strands> asset;
  bool Update(const StarClusterGpuParameters& parameters, const std::vector<double>& orbits, float radius);
};

bool InspectUniverseLayer(InspectorContext& context, class UniverseLayer& layer);

class UniverseLayer final : public ILayer {
  friend bool InspectUniverseLayer(InspectorContext& context, UniverseLayer& layer);

 public:
  void RegisterTypes(Application& application) override;
  bool depth_write = true;
  float star_fade_strength = 0.5f;
  bool show_orbit_strands = false;
  StarOrbitDisplay orbit_display = StarOrbitDisplay::All;
  float orbit_strand_radius = 0.1f;

 private:
  std::shared_ptr<DescriptorSetLayout> star_cluster_layout_;
  std::shared_ptr<ComputePipeline> star_cluster_pipeline_;
  std::shared_ptr<DescriptorSetLayout> star_render_layout_;
  std::shared_ptr<GraphicsPipeline> star_render_pipeline_;
  std::unique_ptr<BufferUploadArena> parameter_upload_arena_;
  double global_simulation_time_ = 0.0;
  uint64_t frame_number_ = 0;
  uint32_t registered_render_cluster_count_ = 0;
  uint32_t registered_render_star_count_ = 0;
  bool fp64_supported_ = false;
  bool compute_ready_ = false;
  bool render_ready_ = false;
  std::string gpu_status_ = "Not initialized";
  std::weak_ptr<Scene> procedural_galaxy_scene_;
  std::weak_ptr<Scene> simulation_scene_;
  StarClusterBatch batch_;
  std::shared_ptr<Buffer> base_sample_buffer_;
  std::vector<StarBatchFrameSlot> frame_slots_;
  size_t star_capacity_ = 0;
  size_t cluster_capacity_ = 0;
  uint64_t computed_revision_ = 0;
  uint64_t rendered_revision_ = 0;
  uint32_t render_slot_ = 0;
  uint32_t draws_this_frame_ = 0;
  StarPicker star_picker_;
  StarFollowState star_follow_;
  StarViewTransition star_view_;
  bool demo_needs_framing_ = false;
  StarDemoCameraOverride demo_main_camera_, demo_scene_camera_;
  std::shared_ptr<GraphicsPipeline> star_hover_pipeline_;
  float pick_minimum_radius_ = 3.0f;
  uint64_t last_viewport_click_ = 0;
  uint64_t last_follow_toggle_ = 0;
  bool pick_benchmark_ = false;
  bool follow_benchmark_ = false;
  std::string pick_camera_name_ = "Main camera";
  std::unordered_map<uint64_t, StarOrbitStrandCache> orbit_strands_;
  std::shared_ptr<Material> orbit_material_;
  size_t displayed_orbits_ = 0;
  uint32_t orbit_draws_ = 0;
  double orbit_rebuild_ms_ = 0, orbit_upload_ms_ = 0;
  std::string orbit_status_ = "Disabled";

  void OnCreate() override;
  void OnDestroy() override;
  void Update() override;
  void InitializeGpuResources();
  void ConfigureProceduralGalaxyDemoIfNeeded();
  void ResetSimulation();
  void EnsureBatchResources(bool population_changed);
  void RegisterForwardRendering(StarBatchRenderPacket packet);
  void RenderOrbitStrands(const std::vector<StarClusterGpuParameters>& parameters);
  void UpdatePickingInput(const std::shared_ptr<Scene>& scene);
  void UpdatePlanetTerrain(const std::shared_ptr<Scene>& scene) const;
};
}  // namespace universe_package
