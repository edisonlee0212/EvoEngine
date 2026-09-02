#pragma once

#include "PerlinNoiseStage.hpp"
#include "PlanetTerrain.hpp"
#include "StarCluster.hpp"
#include "TerrainChunk.hpp"

namespace universe_package {
using namespace evo_engine;

bool InspectUniverseLayer(InspectorContext& context, class UniverseLayer& layer);

class UniverseLayer final : public ILayer {
  friend bool InspectUniverseLayer(InspectorContext& context, UniverseLayer& layer);

 public:
  void RegisterTypes(Application& application) override;

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

  void OnCreate() override;
  void OnDestroy() override;
  void Update() override;
  void InitializeGpuResources();
  void ConfigureProceduralGalaxyDemoIfNeeded();
  void EnsureClusterResources(StarCluster& cluster);
  void ConsumeCompletedSlot(StarCluster& cluster, uint32_t frame_index);
  [[nodiscard]] std::function<void(VkCommandBuffer)> PrepareClusterCompute(StarCluster& cluster, uint32_t frame_index);
  void RegisterForwardRendering(std::vector<StarClusterRenderPacket> render_packets);
  void UpdatePlanetTerrain(const std::shared_ptr<Scene>& scene) const;
};
}  // namespace universe_package
