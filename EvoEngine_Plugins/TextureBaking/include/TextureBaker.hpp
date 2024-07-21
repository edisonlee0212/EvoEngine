#pragma once

namespace evo_engine {
class TextureBaker {
 public:
  enum class RayCastingDirectionMode { Default, AggressiveSmoothing };

  enum class SamplingMethod { Nearest, Bilinear };
  /**
   * @enum UnresolvedPixelMode
   * @brief Default: use unresolved_geometry_color to fill unresolved pixels during ray casting\n
   * AutoFill: First try multiple ray casting (unresolved_geometry_resample_count) and dilate to fill all unresolved
   * pixels
   */
  enum class UnresolvedPixelMode { Default, Dilate };

  struct Parameters {
    bool discard_alpha = false;

    bool diffuse_enabled = true;
    bool normal_enabled = true;

    bool roughness_enabled = true;
    bool metallic_enabled = true;
    bool ao_enabled = true;

    glm::uvec2 texture_resolution = {512, 512};
    
    bool force_rewrite_uv = false;

    float ray_casting_range = 0.3f;

    float thin_scale_factor = 0.01f;

    bool cull_back_face = true;

    glm::vec4 empty_space_color = glm::vec4(0.f);

    RayCastingDirectionMode ray_casting_direction_mode = RayCastingDirectionMode::Default;
    SamplingMethod sampling_method = SamplingMethod::Nearest;

    UnresolvedPixelMode unresolved_pixel_mode =
        UnresolvedPixelMode::Default;  ///< How to deal with pixels corresponding to unresolved geometry.
    glm::vec4 unresolved_pixel_color = glm::vec4(0, 0, 0, 1);

    int32_t post_dilate_distance = -1;
  };
  static void Execute(const Parameters& parameters, const std::shared_ptr<Mesh>& reference_mesh,
                      const std::shared_ptr<Material>& reference_material, const std::shared_ptr<Mesh>& target_mesh,
                      const std::shared_ptr<Material>& target_material);
};
}  // namespace evo_engine