#pragma once
#include "AssetRef.hpp"
#include "Camera.hpp"
#include "IPrivateComponent.hpp"
#include "Texture2D.hpp"

namespace evo_engine {
class GpuRayTracerCamera : public IPrivateComponent {
 public:
  struct CaptureParameters {
    uint32_t sampled_count = 0;
    uint32_t bounce = 5;
  };
  CaptureParameters capture_parameters{};
  [[nodiscard]] float GetSizeRatio() const;
  void UpdateCameraInfoBlock(CameraInfoBlock& camera_info_block, const GlobalTransform& global_transform) const;
  float near_distance = 0.1f;
  float far_distance = 200.0f;
  float fov = 120;
  void OnCreate() override;
  AssetRef texture_ref;
  void OnDestroy() override;
  bool per_frame_capture = false;
  void LateUpdate() override;
  void Capture();
  glm::uvec2 resolution = {128, 128};
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace evo_engine