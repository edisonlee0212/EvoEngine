#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Camera.hpp"
#include "Lights.hpp"
#include "RenderInstanceStorage.hpp"

#include <array>
#include <cmath>
#include <cstddef>

using namespace evo_engine;

namespace {
constexpr float kTolerance = 1.0e-4f;

std::array<glm::vec3, 8> FrustumCorners(const std::shared_ptr<Camera>& camera, const float near_plane,
                                        const float far_plane, const glm::vec3& position = glm::vec3(0.0f),
                                        const glm::quat& rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
  std::array<glm::vec3, 8> corners{};
  Camera::CalculateFrustumPoints(camera, near_plane, far_plane, position, rotation, corners.data());
  return corners;
}

Bound TestWorldBound() {
  Bound bound;
  bound.min = glm::vec3(-1000.0f);
  bound.max = glm::vec3(1000.0f);
  return bound;
}

RenderInstanceStorage::DirectionalShadowCascadeFitResult Fit(const RenderSettings::ShadowCascadeFitMode mode,
                                                             const std::array<glm::vec3, 8>& corners,
                                                             const glm::ivec2 viewport = glm::ivec2(4096),
                                                             const float filter_radius_world = 0.0f) {
  return RenderInstanceStorage::CalculateDirectionalShadowCascadeFit(
      {mode, corners, TestWorldBound(), glm::normalize(glm::vec3(0.35f, -1.0f, 0.2f)),
       glm::normalize(glm::vec3(0.0f, 0.2f, 1.0f)), viewport, filter_radius_world});
}

bool IsFinite(const glm::mat4& matrix) {
  for (int column = 0; column < 4; ++column) {
    for (int row = 0; row < 4; ++row) {
      if (!std::isfinite(matrix[column][row])) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

TEST(DirectionalShadowCascadeFit, StableSphereIsTheTransientStartupDefault) {
  const RenderSettings settings;
  EXPECT_EQ(settings.shadow_cascade_fit_mode, RenderSettings::ShadowCascadeFitMode::StableSphere);
  EXPECT_EQ(settings.directional_pcf_sample_amount, 16);
  EXPECT_EQ(settings.pcf_sample_amount, 32);
  EXPECT_EQ(static_cast<int>(RenderSettings::ShadowCascadeFitMode::StableSphere), 0);
  EXPECT_EQ(static_cast<int>(RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb), 1);
  EXPECT_STREQ(RenderSettings::GetShadowCascadeFitModeName(settings.shadow_cascade_fit_mode), "Stable Sphere");
  EXPECT_STREQ(RenderSettings::GetShadowCascadeFitModeName(RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb),
               "Tight Light-Space AABB");
}

TEST(DirectionalShadowCascadeFit, DirectionalBiasDefaultsRemainUnchanged) {
  DirectionalLight light;
  light.OnCreate();
  EXPECT_FLOAT_EQ(light.bias, 0.0f);
  EXPECT_FLOAT_EQ(light.slope_bias, 0.0f);
  EXPECT_FLOAT_EQ(light.normal_offset, 1.0f);
}

TEST(DirectionalShadowCascadeFit, PerCameraSplitsAndFitsTrackDistinctCameraConfigurations) {
  const RenderSettings settings;
  auto first_camera = std::make_shared<Camera>();
  first_camera->camera_settings.near_distance = 0.1f;
  first_camera->camera_settings.fov = 90.0f;
  first_camera->Resize({1920, 1080});
  const glm::vec3 first_position(0.0f, 2.0f, 5.0f);
  const glm::quat first_rotation(glm::radians(glm::vec3(0.0f, 15.0f, 0.0f)));

  auto second_camera = std::make_shared<Camera>();
  second_camera->camera_settings.near_distance = 5.0f;
  second_camera->camera_settings.fov = 145.0f;
  second_camera->Resize({900, 1600});
  const glm::vec3 second_position(-12.0f, 8.0f, 3.0f);
  const glm::quat second_rotation(glm::radians(glm::vec3(-18.0f, 70.0f, 5.0f)));

  const auto near_splits = settings.GetShadowCascadeSplitDistances(first_camera->camera_settings.near_distance);
  const auto far_splits = settings.GetShadowCascadeSplitDistances(second_camera->camera_settings.near_distance);
  for (int split = 0; split < 3; ++split) {
    EXPECT_NE(near_splits[split], far_splits[split]);
  }
  EXPECT_FLOAT_EQ(near_splits.w, 400.0f);
  EXPECT_FLOAT_EQ(far_splits.w, 400.0f);

  const auto first_fit = Fit(RenderSettings::ShadowCascadeFitMode::StableSphere,
                             FrustumCorners(first_camera, first_camera->camera_settings.near_distance, near_splits.x,
                                            first_position, first_rotation));
  const auto second_fit = Fit(RenderSettings::ShadowCascadeFitMode::StableSphere,
                              FrustumCorners(second_camera, second_camera->camera_settings.near_distance, far_splits.x,
                                             second_position, second_rotation));
  EXPECT_NE(first_fit.light_space_matrix[0][0], second_fit.light_space_matrix[0][0]);
  EXPECT_NE(first_fit.light_space_matrix[3][0], second_fit.light_space_matrix[3][0]);
}

TEST(DirectionalShadowCascadeFit, TransitionOverlapMatchesShaderPolicy) {
  RenderSettings settings;
  settings.shadow_cascade_transition_width = 5.0f;
  constexpr float near_distance = 0.1f;
  const auto splits = settings.GetShadowCascadeSplitDistances(near_distance);
  for (int boundary = 0; boundary < 3; ++boundary) {
    const auto previous_split = boundary == 0 ? 0.0f : splits[boundary - 1];
    const auto available_width = glm::min(splits[boundary] - previous_split, splits[boundary + 1] - splits[boundary]);
    EXPECT_FLOAT_EQ(settings.GetShadowCascadeTransitionHalfWidth(boundary, near_distance),
                    glm::min(settings.shadow_cascade_transition_width, available_width) * 0.5f);
  }

  settings.shadow_cascade_transition_width = 0.0f;
  for (int boundary = 0; boundary < 3; ++boundary) {
    EXPECT_FLOAT_EQ(settings.GetShadowCascadeTransitionHalfWidth(boundary, near_distance), 0.0f);
  }
}

TEST(DirectionalShadowCascadeFit, DerivedFitsContainCascadeTransitionBands) {
  auto camera = std::make_shared<Camera>();
  camera->camera_settings.fov = 145.0f;
  camera->Resize({2560, 1080});
  constexpr float near_distance = 0.1f;
  RenderSettings settings;
  const auto splits = settings.GetShadowCascadeSplitDistances(near_distance);

  for (const auto mode : {RenderSettings::ShadowCascadeFitMode::StableSphere,
                          RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb}) {
    for (int cascade = 0; cascade < 4; ++cascade) {
      const auto nominal_start = cascade == 0 ? near_distance : splits[cascade - 1];
      const auto fit_start =
          cascade == 0 ? nominal_start
                       : (glm::max)(near_distance, nominal_start - settings.GetShadowCascadeTransitionHalfWidth(
                                                                       cascade - 1, near_distance));
      const auto fit_end = cascade == 3
                               ? splits[cascade]
                               : (glm::min)(splits.w, splits[cascade] + settings.GetShadowCascadeTransitionHalfWidth(
                                                                            cascade, near_distance));
      const auto fit_corners = FrustumCorners(camera, fit_start, fit_end);
      const auto fit = Fit(mode, fit_corners);
      for (const auto& corner : fit_corners) {
        const auto clip = fit.light_space_matrix * glm::vec4(corner, 1.0f);
        const auto ndc = glm::vec3(clip) / clip.w;
        EXPECT_LE(glm::abs(ndc.x), 1.0f + kTolerance);
        EXPECT_LE(glm::abs(ndc.y), 1.0f + kTolerance);
      }
    }
  }
}

TEST(DirectionalShadowCascadeFit, FrustumCornersMatchRenderedProjection) {
  struct Case {
    float fov;
    glm::uvec2 resolution;
    float near_plane;
    float far_plane;
    glm::vec3 position;
    glm::quat rotation;
  };
  const std::array cases = {
      Case{120.0f, {1920, 1080}, 0.1f, 80.0f, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)},
      Case{150.0f,
           {2560, 1080},
           0.5f,
           250.0f,
           glm::vec3(4.0f, -2.0f, 8.0f),
           glm::quat(glm::radians(glm::vec3(12.0f, 35.0f, -4.0f)))},
      Case{70.0f,
           {900, 1600},
           2.0f,
           40.0f,
           glm::vec3(-8.0f, 3.0f, 2.0f),
           glm::quat(glm::radians(glm::vec3(-20.0f, 5.0f, 10.0f)))},
  };

  for (const auto& test_case : cases) {
    auto camera = std::make_shared<Camera>();
    camera->camera_settings.fov = test_case.fov;
    camera->camera_settings.near_distance = test_case.near_plane;
    camera->camera_settings.far_distance = test_case.far_plane;
    camera->Resize(test_case.resolution);
    const auto corners =
        FrustumCorners(camera, test_case.near_plane, test_case.far_plane, test_case.position, test_case.rotation);
    const auto front = test_case.rotation * glm::vec3(0.0f, 0.0f, -1.0f);
    const auto up = test_case.rotation * glm::vec3(0.0f, 1.0f, 0.0f);
    const auto view = glm::lookAt(test_case.position, test_case.position + front, up);
    const auto projection_view = camera->GetProjection() * view;
    for (size_t index = 0; index < corners.size(); ++index) {
      const auto clip = projection_view * glm::vec4(corners[index], 1.0f);
      const auto ndc = glm::vec3(clip) / clip.w;
      EXPECT_NEAR(glm::abs(ndc.x), 1.0f, 2.0e-4f);
      EXPECT_NEAR(glm::abs(ndc.y), 1.0f, 2.0e-4f);
      EXPECT_NEAR(ndc.z, index < 4 ? 0.0f : 1.0f, 2.0e-4f);
    }
  }
}

TEST(DirectionalShadowCascadeFit, ZeroToOneDepthSelectsTheMatchingTightAabbCascade) {
  auto camera = std::make_shared<Camera>();
  camera->camera_settings.near_distance = 0.1f;
  camera->camera_settings.far_distance = 1000.0f;
  camera->Resize({1920, 1080});

  const auto projection = camera->GetProjection();
  const auto a = projection[2][2];
  const auto b = projection[3][2];
  const auto near_plane = glm::abs(b / a);
  const auto far_plane = glm::abs(b / (a + 1.0f));
  EXPECT_NEAR(near_plane, camera->camera_settings.near_distance, 1.0e-4f);
  EXPECT_NEAR(far_plane, camera->camera_settings.far_distance, 1.0f);

  const RenderSettings settings;
  const auto splits = settings.GetShadowCascadeSplitDistances(near_plane);
  for (int expected_cascade = 0; expected_cascade < 4; ++expected_cascade) {
    const auto slice_start = expected_cascade == 0 ? near_plane : splits[expected_cascade - 1];
    const auto slice_end = splits[expected_cascade];
    const auto view_depth = (slice_start + slice_end) * 0.5f;
    const auto clip = projection * glm::vec4(0.0f, 0.0f, -view_depth, 1.0f);
    const auto ndc_depth = clip.z / clip.w;
    const auto linear_depth = near_plane * far_plane / (far_plane - ndc_depth * (far_plane - near_plane));

    int selected_cascade = -1;
    for (int cascade = 0; cascade < 4; ++cascade) {
      if (linear_depth < splits[cascade]) {
        selected_cascade = cascade;
        break;
      }
    }
    EXPECT_NEAR(linear_depth, view_depth, view_depth * 5.0e-4f);
    EXPECT_EQ(selected_cascade, expected_cascade);

    const auto fit =
        Fit(RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb, FrustumCorners(camera, slice_start, slice_end));
    const auto receiver_clip = fit.light_space_matrix * glm::vec4(0.0f, 0.0f, -view_depth, 1.0f);
    const auto receiver_ndc = glm::vec3(receiver_clip) / receiver_clip.w;
    EXPECT_LE(glm::abs(receiver_ndc.x), 1.0f + kTolerance);
    EXPECT_LE(glm::abs(receiver_ndc.y), 1.0f + kTolerance);
    EXPECT_GE(receiver_ndc.z, -kTolerance);
    EXPECT_LE(receiver_ndc.z, 1.0f + kTolerance);
  }
}

TEST(DirectionalShadowCascadeFit, DerivedFitsContainEverySliceCorner) {
  auto camera = std::make_shared<Camera>();
  camera->camera_settings.fov = 145.0f;
  camera->Resize({2560, 1080});
  const RenderSettings settings;
  const auto near_plane = camera->camera_settings.near_distance;
  for (const auto mode : {RenderSettings::ShadowCascadeFitMode::StableSphere,
                          RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb}) {
    for (int cascade = 0; cascade < 4; ++cascade) {
      const auto start = cascade == 0 ? near_plane : settings.GetShadowCascadeSplitDistance(cascade - 1, near_plane);
      const auto end = settings.GetShadowCascadeSplitDistance(cascade, near_plane);
      const auto corners = FrustumCorners(camera, start, end);
      const auto fit = Fit(mode, corners);
      ASSERT_TRUE(IsFinite(fit.light_space_matrix));
      for (const auto& corner : corners) {
        const auto clip = fit.light_space_matrix * glm::vec4(corner, 1.0f);
        const auto ndc = glm::vec3(clip) / clip.w;
        EXPECT_LE(glm::abs(ndc.x), 1.0f + kTolerance);
        EXPECT_LE(glm::abs(ndc.y), 1.0f + kTolerance);
        EXPECT_GE(ndc.z, -kTolerance);
        EXPECT_LE(ndc.z, 1.0f + kTolerance);
      }
    }
  }
}

TEST(DirectionalShadowCascadeFit, StableSphereQuantizesRadiusAndMotionToTexels) {
  auto camera = std::make_shared<Camera>();
  camera->camera_settings.fov = 120.0f;
  camera->Resize({1920, 1080});
  const auto base = FrustumCorners(camera, 0.1f, 75.0f);
  auto moved = base;
  for (auto& corner : moved) {
    corner += glm::vec3(0.0137f, -0.0073f, 0.019f);
  }

  for (const auto viewport : {glm::ivec2(8192), glm::ivec2(4096)}) {
    const auto first = Fit(RenderSettings::ShadowCascadeFitMode::StableSphere, base, viewport);
    const auto second = Fit(RenderSettings::ShadowCascadeFitMode::StableSphere, moved, viewport);
    const auto radius = (first.orthographic_max.x - first.orthographic_min.x) * 0.5f;
    EXPECT_NEAR(radius * 16.0f, glm::round(radius * 16.0f), kTolerance);
    EXPECT_FLOAT_EQ(first.orthographic_max.x, first.orthographic_max.y);
    const auto probe = glm::vec4(7.0f, -3.0f, 5.0f, 1.0f);
    const auto first_clip = first.light_space_matrix * probe;
    const auto second_clip = second.light_space_matrix * probe;
    const auto texel_motion = glm::vec2(second_clip - first_clip) * glm::vec2(viewport) * 0.5f;
    EXPECT_NEAR(texel_motion.x, glm::round(texel_motion.x), 2.0e-3f);
    EXPECT_NEAR(texel_motion.y, glm::round(texel_motion.y), 2.0e-3f);
  }
}

TEST(DirectionalShadowCascadeFit, TightAabbKeepsIndependentAxesAndDoesNotSnap) {
  auto camera = std::make_shared<Camera>();
  camera->camera_settings.fov = 120.0f;
  camera->Resize({2560, 1080});
  const auto base = FrustumCorners(camera, 0.1f, 100.0f);
  auto moved = base;
  for (auto& corner : moved) {
    corner += glm::vec3(0.0137f, -0.0073f, 0.019f);
  }
  const auto first = Fit(RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb, base);
  const auto second = Fit(RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb, moved);
  const auto half_extent = (first.orthographic_max - first.orthographic_min) * 0.5f;
  EXPECT_GT(glm::abs(half_extent.x - half_extent.y), 1.0f);
  const auto probe = glm::vec4(7.0f, -3.0f, 5.0f, 1.0f);
  const auto texel_motion = glm::vec2((second.light_space_matrix - first.light_space_matrix) * probe) * 4096.0f;
  EXPECT_TRUE(glm::abs(texel_motion.x - glm::round(texel_motion.x)) > 1.0e-3f ||
              glm::abs(texel_motion.y - glm::round(texel_motion.y)) > 1.0e-3f);
}

TEST(DirectionalShadowCascadeFit, ZeroViewportAndDegenerateSceneRemainFinite) {
  auto camera = std::make_shared<Camera>();
  const auto corners = FrustumCorners(camera, 0.1f, 10.0f);
  Bound point_bound;
  point_bound.min = glm::vec3(0.0f);
  point_bound.max = glm::vec3(0.0f);
  for (const auto mode : {RenderSettings::ShadowCascadeFitMode::StableSphere,
                          RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb}) {
    const auto fit = RenderInstanceStorage::CalculateDirectionalShadowCascadeFit(
        {mode, corners, point_bound, glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::ivec2(0)});
    EXPECT_TRUE(IsFinite(fit.light_space_matrix));
    EXPECT_GE(fit.light_space_depth_half_extent, 0.001f);
  }
}

TEST(DirectionalShadowCascadeFit, DirectionalLightBlockKeepsHostShaderLayout) {
  EXPECT_EQ(offsetof(DirectionalLightInfoBlock, direction), 0u);
  EXPECT_EQ(offsetof(DirectionalLightInfoBlock, light_space_matrix), 48u);
  EXPECT_EQ(offsetof(DirectionalLightInfoBlock, light_frustum_width), 304u);
  EXPECT_EQ(offsetof(DirectionalLightInfoBlock, light_frustum_height), 320u);
  EXPECT_EQ(offsetof(DirectionalLightInfoBlock, light_frustum_distance), 336u);
  EXPECT_EQ(offsetof(DirectionalLightInfoBlock, reserved_parameters), 352u);
  EXPECT_EQ(offsetof(DirectionalLightInfoBlock, viewport), 368u);
  EXPECT_EQ(sizeof(DirectionalLightInfoBlock), 384u);
}

TEST(DirectionalShadowCascadeFit, DirectionalShadowSamplerUsesLinearDepthComparison) {
  const auto sampler_info = Lighting::GetDirectionalShadowSamplerCreateInfo();
  EXPECT_EQ(sampler_info.magFilter, VK_FILTER_LINEAR);
  EXPECT_EQ(sampler_info.minFilter, VK_FILTER_LINEAR);
  EXPECT_EQ(sampler_info.mipmapMode, VK_SAMPLER_MIPMAP_MODE_NEAREST);
  EXPECT_EQ(sampler_info.addressModeU, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE);
  EXPECT_EQ(sampler_info.addressModeV, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE);
  EXPECT_EQ(sampler_info.addressModeW, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE);
  EXPECT_EQ(sampler_info.anisotropyEnable, VK_FALSE);
  EXPECT_FLOAT_EQ(sampler_info.maxAnisotropy, 1.0f);
  EXPECT_EQ(sampler_info.compareEnable, VK_TRUE);
  EXPECT_EQ(sampler_info.compareOp, VK_COMPARE_OP_LESS_OR_EQUAL);
  EXPECT_FLOAT_EQ(sampler_info.minLod, 0.0f);
  EXPECT_FLOAT_EQ(sampler_info.maxLod, 0.0f);
}

TEST(DirectionalShadowCascadeFit, FilterGuardKeepsComparisonFootprintInsideViewport) {
  auto camera = std::make_shared<Camera>();
  camera->camera_settings.fov = 70.0f;
  camera->Resize({1920, 1080});
  const auto base_corners = FrustumCorners(camera, 0.1f, 40.0f);
  constexpr float filter_radius_world = 0.25f;
  constexpr glm::ivec2 viewport(4096, 2048);
  for (const auto translation : {glm::vec3(0.0f), glm::vec3(0.0137f, -0.0073f, 0.019f)}) {
    auto corners = base_corners;
    for (auto& corner : corners) {
      corner += translation;
    }
    for (const auto mode : {RenderSettings::ShadowCascadeFitMode::StableSphere,
                            RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb}) {
      const auto fit = Fit(mode, corners, viewport, filter_radius_world);
      const auto half_extent = (fit.orthographic_max - fit.orthographic_min) * 0.5f;
      const auto required_margin =
          glm::vec2(filter_radius_world) / (2.0f * half_extent) + glm::vec2(0.5f) / glm::vec2(viewport);
      for (const auto& corner : corners) {
        const auto clip = fit.light_space_matrix * glm::vec4(corner, 1.0f);
        const auto light_uv = glm::vec2(clip) * 0.5f + 0.5f;
        EXPECT_GE(light_uv.x, required_margin.x - 2.0e-5f);
        EXPECT_GE(light_uv.y, required_margin.y - 2.0e-5f);
        EXPECT_LE(light_uv.x, 1.0f - required_margin.x + 2.0e-5f);
        EXPECT_LE(light_uv.y, 1.0f - required_margin.y + 2.0e-5f);
      }
    }
  }
}

TEST(DirectionalShadowCascadeFit, OffCameraCastersAlongLightDirectionRemainInsideDerivedFits) {
  auto camera = std::make_shared<Camera>();
  camera->camera_settings.fov = 95.0f;
  camera->Resize({1920, 1080});
  const auto corners = FrustumCorners(camera, 0.1f, 80.0f);
  glm::vec3 receiver(0.0f);
  for (const auto& corner : corners) {
    receiver += corner;
  }
  receiver /= static_cast<float>(corners.size());
  const auto caster = receiver + glm::normalize(glm::vec3(0.35f, -1.0f, 0.2f)) * 100.0f;
  for (const auto mode : {RenderSettings::ShadowCascadeFitMode::StableSphere,
                          RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb}) {
    const auto fit = Fit(mode, corners);
    const auto receiver_clip = fit.light_space_matrix * glm::vec4(receiver, 1.0f);
    const auto caster_clip = fit.light_space_matrix * glm::vec4(caster, 1.0f);
    EXPECT_NEAR(receiver_clip.x, caster_clip.x, kTolerance);
    EXPECT_NEAR(receiver_clip.y, caster_clip.y, kTolerance);
    EXPECT_LE(glm::abs(caster_clip.x), 1.0f + kTolerance);
    EXPECT_LE(glm::abs(caster_clip.y), 1.0f + kTolerance);
    EXPECT_GE(caster_clip.z, -kTolerance);
    EXPECT_LE(caster_clip.z, 1.0f + kTolerance);
  }
}
