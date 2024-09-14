#pragma once
#include "IPrivateComponent.hpp"
#include "AssetRef.hpp"
#include "Camera.hpp"
#include "Texture2D.hpp"
namespace evo_engine {
class CpuRayTracerCamera : public IPrivateComponent {
 public:
  struct CaptureParameters {
    size_t sample = 1;
    size_t bounce = 5;
  };
  CaptureParameters capture_parameters {};
  [[nodiscard]] float GetSizeRatio() const;
  void UpdateCameraInfoBlock(CameraInfoBlock& camera_info_block, const GlobalTransform& global_transform) const;
  float near_distance = 0.1f;
  float far_distance = 200.0f;
  float fov = 120;
  void OnCreate() override;
  AssetRef texture_ref;
  void Capture(const CaptureParameters& parameters, const std::shared_ptr<Texture2D>& target_texture) const;
  glm::uvec2 resolution = {64, 64};
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace evo_engine