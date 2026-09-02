#pragma once

#include "PerlinNoiseStage.hpp"
#include "PlanetTerrain.hpp"
#include "StarCluster.hpp"
#include "StarDemoCamera.hpp"
#include "StarFollow.hpp"
#include "StarPicking.hpp"
#include "TerrainChunk.hpp"

namespace universe_package {
using namespace evo_engine;

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
  bool operator==(const StarClusterRange& other) const {
    return identity == other.identity && seed == other.seed && offset == other.offset && count == other.count;
  }
};

// CPU state owned by UniverseLayer; separate from GPU allocations for deterministic tests.
struct StarClusterBatch {
  struct Clock {
    std::weak_ptr<StarCluster> component;
    uint64_t component_handle = 0;
    uint64_t identity = 0;
    double elapsed = 0.0;
    double last_global_time = 0.0;
  };
  std::unordered_map<const StarCluster*, Clock> clocks;
  std::vector<StarBaseSample> samples;
  std::vector<StarClusterRange> ranges;
  std::vector<StarClusterGpuParameters> parameters;
  uint64_t population_revision = 0;
  uint64_t next_identity = 1;
  bool Update(const std::vector<StarClusterInput>& inputs, double global_time);
};

StarBaseSample GenerateStarBaseSample(uint64_t seed, uint32_t ordinal);
StarClusterGpuParameters BuildStarClusterParameters(const StarCluster& cluster, const glm::dmat4& world_transform,
                                                    double simulation_time);
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
};

bool InspectUniverseLayer(InspectorContext& context, class UniverseLayer& layer);

class UniverseLayer final : public ILayer {
  friend bool InspectUniverseLayer(InspectorContext& context, UniverseLayer& layer);

 public:
  void RegisterTypes(Application& application) override;
  bool depth_write = true;

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
  StarDemoCameraOverride demo_main_camera_, demo_scene_camera_;
  std::shared_ptr<GraphicsPipeline> star_hover_pipeline_;
  float pick_minimum_radius_ = 3.0f;
  uint64_t last_viewport_click_ = 0;
  uint64_t last_follow_toggle_ = 0;
  bool pick_benchmark_ = false;
  bool follow_benchmark_ = false;
  std::string pick_camera_name_ = "Main camera";

  void OnCreate() override;
  void OnDestroy() override;
  void Update() override;
  void InitializeGpuResources();
  void ConfigureProceduralGalaxyDemoIfNeeded();
  void ResetSimulation();
  void EnsureBatchResources(bool population_changed);
  void RegisterForwardRendering(StarBatchRenderPacket packet);
  void UpdatePickingInput(const std::shared_ptr<Scene>& scene);
  void UpdatePlanetTerrain(const std::shared_ptr<Scene>& scene) const;
};
}  // namespace universe_package
