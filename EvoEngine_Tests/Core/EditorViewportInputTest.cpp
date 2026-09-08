#include "EvoEngine_SDK_PCH.hpp"

#include "Camera.hpp"
#include "EditorLayer.hpp"
#include "gtest/gtest.h"
#include "imgui.h"

#include <limits>

using namespace evo_engine;

namespace evo_engine {
struct EditorCameraRebaseTestAccess {
  static void SetTransition(EditorLayer& layer, const glm::vec3& from, const glm::vec3& to) {
    layer.previous_position_ = from;
    layer.target_position_ = to;
    layer.previous_rotation_ = layer.target_rotation_ = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    layer.transition_time_ = 1.0f;
    layer.transition_timer_ = 3.0f;
    layer.transition_preserves_world_up_ = true;
    layer.lock_camera = true;
    layer.scene_camera_free_fly_state_.smoothed_move_velocity = {2.0f, 0.0f, 0.0f};
  }
  static void CheckTransition(const EditorLayer& layer, const glm::dmat4& transform) {
    EXPECT_LT(glm::length(layer.previous_position_ - glm::vec3(transform * glm::dvec4(1, 2, 3, 1))), 1e-5f);
    EXPECT_LT(glm::length(layer.target_position_ - glm::vec3(transform * glm::dvec4(4, 5, 6, 1))), 1e-5f);
    const auto rotation = glm::quat_cast(glm::mat3(transform));
    EXPECT_NEAR(glm::abs(glm::dot(layer.previous_rotation_, rotation)), 1.0f, 1e-6f);
    EXPECT_NEAR(glm::abs(glm::dot(layer.target_rotation_, rotation)), 1.0f, 1e-6f);
    EXPECT_LT(glm::length(layer.transition_up_ - rotation * glm::vec3(0, 1, 0)), 1e-5f);
    EXPECT_LT(glm::length(layer.scene_camera_free_fly_state_.smoothed_move_velocity - rotation * glm::vec3(2, 0, 0)),
              1e-5f);
    EXPECT_FLOAT_EQ(layer.transition_time_, 1.0f);
    EXPECT_FLOAT_EQ(layer.transition_timer_, 3.0f);
    EXPECT_TRUE(layer.transition_preserves_world_up_);
    EXPECT_TRUE(layer.lock_camera);
  }
  static void Finalize(EditorLayer& layer, EditorViewportInput& input, const bool blocked = false) {
    layer.FinalizeViewportInput(input, blocked);
  }
};
}  // namespace evo_engine

TEST(EditorViewportInput, MapsImageRectangleWithTextureYOrientation) {
  glm::vec2 uv;
  ASSERT_TRUE(EditorLayer::MapViewportCursor({100.0f, 200.0f}, {800.0f, 600.0f}, {300.0f, 350.0f}, uv));
  EXPECT_FLOAT_EQ(uv.x, 0.25f);
  EXPECT_FLOAT_EQ(uv.y, 0.75f);
  ASSERT_TRUE(EditorLayer::MapViewportCursor({100.0f, 200.0f}, {800.0f, 600.0f}, {100.0f, 200.0f}, uv));
  EXPECT_FLOAT_EQ(uv.x, 0.0f);
  EXPECT_FLOAT_EQ(uv.y, 1.0f);
}

TEST(EditorViewportInput, RejectsLetterboxAndExclusiveImageEdges) {
  glm::vec2 uv;
  for (const auto point :
       {glm::vec2(99.0f, 350.0f), glm::vec2(300.0f, 199.0f), glm::vec2(900.0f, 350.0f), glm::vec2(300.0f, 800.0f)}) {
    EXPECT_FALSE(EditorLayer::MapViewportCursor({100.0f, 200.0f}, {800.0f, 600.0f}, point, uv));
    EXPECT_EQ(uv, glm::vec2(0.0f));
  }
}

TEST(EditorViewportInput, SupportsDetachedWindowsAndDisplayScalingWithoutRenderSize) {
  glm::vec2 unscaled_uv;
  glm::vec2 scaled_uv;
  ASSERT_TRUE(EditorLayer::MapViewportCursor({-1920.0f, -400.0f}, {800.0f, 600.0f}, {-1720.0f, -250.0f}, unscaled_uv));
  ASSERT_TRUE(EditorLayer::MapViewportCursor({-1920.0f, -400.0f}, {1600.0f, 1200.0f}, {-1520.0f, -100.0f}, scaled_uv));
  EXPECT_EQ(unscaled_uv, scaled_uv);
}

TEST(EditorViewportInput, RejectsEmptyAndNonFiniteRectanglesAndCursors) {
  glm::vec2 uv;
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  const auto infinity = std::numeric_limits<float>::infinity();
  for (const auto size :
       {glm::vec2(0.0f, 100.0f), glm::vec2(100.0f, -1.0f), glm::vec2(nan, 100.0f), glm::vec2(100.0f, infinity)})
    EXPECT_FALSE(EditorLayer::MapViewportCursor({}, size, {1.0f, 1.0f}, uv));
  EXPECT_FALSE(EditorLayer::MapViewportCursor({}, {100.0f, 100.0f}, {nan, 1.0f}, uv));
  EXPECT_FALSE(EditorLayer::MapViewportCursor({infinity, 0.0f}, {100.0f, 100.0f}, {1.0f, 1.0f}, uv));
}

TEST(EditorViewportInput, DefaultSnapshotIsInactiveAndHasNoClick) {
  const EditorViewportInput input;
  EXPECT_TRUE(input.camera.expired());
  EXPECT_TRUE(input.scene.expired());
  EXPECT_FALSE(input.focused);
  EXPECT_FALSE(input.visible);
  EXPECT_FALSE(input.cursor_valid);
  EXPECT_EQ(input.click_sequence, 0u);
  EXPECT_EQ(input.follow_toggle_sequence, 0u);
}

TEST(EditorViewportInput, RebasePreservesPoseAndDoesNotMoveOtherCameras) {
  EditorLayer layer;
  auto scene_camera = std::make_shared<Camera>();
  layer.RegisterEditorCamera(scene_camera);
  auto other_camera = std::make_shared<Camera>();
  layer.RegisterEditorCamera(other_camera);
  layer.RefEditorCameraPosition(other_camera->GetHandle()) = {8, 9, 10};
  const glm::vec3 position(4, 5, 6);
  const auto orientation = glm::angleAxis(0.35f, glm::normalize(glm::vec3(1, 2, 3)));
  layer.SetSceneCameraPosition(position);
  layer.SetSceneCameraRotation(orientation);
  const auto transform = glm::translate(glm::dmat4(1), glm::dvec3(10, -20, 30)) *
                         glm::mat4_cast(glm::angleAxis(0.75, glm::normalize(glm::dvec3(2, 1, 3))));
  layer.RebaseSceneCamera(transform);
  EXPECT_LT(glm::length(layer.GetSceneCameraPosition() - glm::vec3(transform * glm::dvec4(position, 1))), 1e-5f);
  const auto expected_rotation = glm::quat(glm::quat_cast(glm::dmat3(transform)) * glm::dquat(orientation));
  EXPECT_NEAR(glm::abs(glm::dot(layer.GetSceneCameraRotation(), expected_rotation)), 1.0f, 1e-6f);
  layer.RebaseSceneCamera(glm::inverse(transform));
  EXPECT_LT(glm::length(layer.GetSceneCameraPosition() - position), 1e-5f);
  EXPECT_NEAR(glm::abs(glm::dot(layer.GetSceneCameraRotation(), orientation)), 1.0f, 1e-6f);
  EXPECT_EQ(layer.RefEditorCameraPosition(other_camera->GetHandle()), glm::vec3(8, 9, 10));
}

TEST(EditorViewportInput, RebaseTransformsActiveTransitionAndVelocityWithoutRestarting) {
  EditorLayer layer;
  layer.RegisterEditorCamera(std::make_shared<Camera>());
  layer.SetSceneCameraPosition({2, 3, 4});
  layer.SetSceneCameraRotation(glm::quat(1, 0, 0, 0));
  EditorCameraRebaseTestAccess::SetTransition(layer, {1, 2, 3}, {4, 5, 6});
  const auto transform = glm::translate(glm::dmat4(1), glm::dvec3(10, 20, 30)) *
                         glm::mat4_cast(glm::angleAxis(0.75, glm::normalize(glm::dvec3(1, 2, 3))));
  layer.RebaseSceneCamera(transform);
  EditorCameraRebaseTestAccess::CheckTransition(layer, transform);
}

TEST(EditorViewportInput, SpaceToggleRequiresVisibleFocusedUncapturedViewportAndDoesNotRepeat) {
  auto* previous_context = ImGui::GetCurrentContext();
  auto* context = ImGui::CreateContext();
  auto& io = ImGui::GetIO();
  io.IniFilename = nullptr;
  io.DisplaySize = {800, 600};
  io.DeltaTime = 1.0f / 60.0f;
  unsigned char* pixels;
  int width, height;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  EditorLayer layer;
  io.AddKeyEvent(ImGuiKey_Space, true);
  ImGui::NewFrame();
  ImGui::Begin("Viewport input test");
  EditorViewportInput input;
  input.visible = true;
  EditorCameraRebaseTestAccess::Finalize(layer, input);
  EXPECT_EQ(input.follow_toggle_sequence, 0u);
  input.focused = true;
  input.visible = false;
  EditorCameraRebaseTestAccess::Finalize(layer, input);
  EXPECT_EQ(input.follow_toggle_sequence, 0u);
  input.visible = true;
  EditorCameraRebaseTestAccess::Finalize(layer, input, true);
  EXPECT_EQ(input.follow_toggle_sequence, 0u);
  io.WantTextInput = true;
  EditorCameraRebaseTestAccess::Finalize(layer, input);
  EXPECT_EQ(input.follow_toggle_sequence, 0u);
  io.WantTextInput = false;
  EditorCameraRebaseTestAccess::Finalize(layer, input);
  EXPECT_NE(input.follow_toggle_sequence, 0u);
  ImGui::End();
  ImGui::EndFrame();
  input.follow_toggle_sequence = 0;
  ImGui::NewFrame();
  ImGui::Begin("Viewport input test");
  EditorCameraRebaseTestAccess::Finalize(layer, input);
  EXPECT_EQ(input.follow_toggle_sequence, 0u);
  ImGui::End();
  ImGui::EndFrame();
  ImGui::DestroyContext(context);
  ImGui::SetCurrentContext(previous_context);
}
