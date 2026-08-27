#pragma once

#include "Bound.hpp"
#include "Transform.hpp"

#include <memory>

namespace evo_engine {
class EVOENGINE_API Camera;
class EVOENGINE_API Material;
class EVOENGINE_API Mesh;
class EVOENGINE_API Scene;
class EVOENGINE_API Texture2D;

struct OffscreenPreviewSettings {
  glm::uvec2 resolution = {512, 512};
  glm::vec4 clear_color = glm::vec4(0.05f, 0.055f, 0.06f, 1.0f);
  float camera_distance_multiplier = 1.15f;
  glm::vec2 subject_rotation = glm::vec2(0.0f);
  float camera_zoom = 1.0f;
};

class EVOENGINE_API OffscreenPreviewRenderer {
 public:
  static void Reset();
  [[nodiscard]] static std::shared_ptr<Texture2D> RenderMaterial(const std::shared_ptr<Material>& material,
                                                                 const OffscreenPreviewSettings& settings = {});
  [[nodiscard]] static std::shared_ptr<Texture2D> RenderMesh(const std::shared_ptr<Mesh>& mesh,
                                                             const std::shared_ptr<Material>& material = {},
                                                             const OffscreenPreviewSettings& settings = {});

 private:
  [[nodiscard]] static std::shared_ptr<Texture2D> RenderMeshWithMaterial(const std::shared_ptr<Mesh>& mesh,
                                                                         const std::shared_ptr<Material>& material,
                                                                         const Bound& focus_bound,
                                                                         const OffscreenPreviewSettings& settings,
                                                                         bool material_presentation);
  [[nodiscard]] static std::shared_ptr<Texture2D> CopyColorTexture(const std::shared_ptr<Camera>& camera,
                                                                   const OffscreenPreviewSettings& settings);
  [[nodiscard]] static GlobalTransform CreateCameraTransform(const Bound& focus_bound,
                                                             const OffscreenPreviewSettings& settings);
};

}  // namespace evo_engine
