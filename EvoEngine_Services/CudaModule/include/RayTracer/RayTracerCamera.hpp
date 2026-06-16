#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include <IPrivateComponent.hpp>
#include <RenderTexture.hpp>

#include "CUDAModule.hpp"
#include "CameraSettings.hpp"
#include "Platform.hpp"

namespace evo_engine {
class RayTracerCamera : public IPrivateComponent {
  friend class RayTracerLayer;
  friend class OptiXRayTracer;
  friend void SerializeRayTracerCamera(YAML::Emitter& out, const RayTracerCamera& target);
  friend void DeserializeRayTracerCamera(const YAML::Node& in, RayTracerCamera& target);
  CameraProperties camera_properties_;
  bool rendered_ = false;
  bool main_camera_ = false;
  AssetRef skybox_;

 public:
  void SetSkybox(const std::shared_ptr<Cubemap>& cubemap);

  void SetMainCamera(bool value);
  bool allow_auto_resize = true;
  std::shared_ptr<RenderTexture> render_texture;
  RayProperties ray_properties;
  glm::uvec2 frame_size;
  void Ready(const glm::vec3& position, const glm::quat& rotation);
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
  void SetFov(float value);
  void SetAperture(float value);
  void SetFocalLength(float value);
  void SetMaxDistance(float value);
  void SetOutputType(OutputType value);
  void SetAccumulate(bool value);
  void SetGamma(float value);

  void ApplyCameraSettings(const CameraSettings& camera_settings);

  [[nodiscard]] glm::mat4 GetProjection() const;
  void SetDenoiserStrength(float value);
  void OnCreate() override;
  void OnDestroy() override;
  RayTracerCamera& operator=(const RayTracerCamera& source);
  void Render();
  void Render(const RayProperties& ray_properties);
  void Render(const RayProperties& ray_properties, const EnvironmentProperties& environment_properties);
  void RenderSpectral();
  void RenderSpectral(const RayProperties& ray_properties);
  void RenderSpectral(const RayProperties& ray_properties, const EnvironmentProperties& environment_properties);
};
}  // namespace evo_engine
