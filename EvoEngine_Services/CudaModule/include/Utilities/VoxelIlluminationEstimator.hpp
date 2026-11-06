#pragma once

#include "EvoEngine_SDK_PCH.hpp"

#include "CUDAModule.hpp"
#include "IPrivateComponent.hpp"
#include "IlluminationVoxelGrid.hpp"

namespace evo_engine {
class EditorLayer;
class ParticleInfoList;

class VoxelIlluminationEstimator : public IPrivateComponent {
  std::vector<IlluminationVoxel> voxels_;
  std::shared_ptr<ParticleInfoList> voxel_info_list_;
  bool visualization_dirty_ = true;

  void RebuildVisualization();

 public:
  glm::vec3 volume_size = glm::vec3(5.0f, 3.0f, 5.0f);
  glm::ivec3 voxel_resolution = glm::ivec3(10, 6, 10);
  int samples_per_voxel = 4;
  int seed = 0;
  float push_normal_distance = 0.0f;
  RayProperties ray_properties = {4, 64};

  bool show_voxels = true;
  bool show_sampling_volume = true;
  bool automatic_intensity_range = true;
  bool depth_test = true;
  float voxel_fill = 0.9f;
  float voxel_opacity = 0.65f;
  float minimum_intensity = 0.0f;
  float maximum_intensity = 1.0f;

  void Estimate();
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
  [[nodiscard]] const std::vector<IlluminationVoxel>& PeekVoxels() const;
};
}  // namespace evo_engine
