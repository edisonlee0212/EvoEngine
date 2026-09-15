#include <gtest/gtest.h>
#include "WindowModePolicy.hpp"
using namespace evo_engine;
TEST(WindowModePolicy, PreservesModeAndPlacementHistory) {
  WindowModePolicy policy({10, 20, 1280, 720});
  EXPECT_EQ(policy.ToggleTarget(WindowDisplayMode::Windowed), WindowDisplayMode::BorderlessFullscreen);
  policy.RecordApplied(WindowDisplayMode::BorderlessWindowed, {30, 40, 900, 600});
  policy.RecordApplied(WindowDisplayMode::ExclusiveFullscreen, {0, 0, 3840, 2160});
  EXPECT_EQ(policy.ToggleTarget(WindowDisplayMode::Windowed), WindowDisplayMode::ExclusiveFullscreen);
  EXPECT_EQ(policy.ToggleTarget(WindowDisplayMode::ExclusiveFullscreen), WindowDisplayMode::BorderlessWindowed);
  const auto placement = policy.WindowedPlacement();
  EXPECT_EQ(placement.width, 900);
  EXPECT_EQ(placement.height, 600);
}
TEST(WindowModePolicy, PermissionsAreIndependentOfModeSwitching) {
  WindowModePolicy policy({0, 0, 1280, 720});
  policy.SetPermissions(false, true);
  EXPECT_FALSE(policy.AllowsUserResize());
  EXPECT_TRUE(policy.AllowsResolutionChange());
  EXPECT_EQ(policy.ToggleTarget(WindowDisplayMode::Windowed), WindowDisplayMode::BorderlessFullscreen);
  policy.SetPermissions(true, false);
  EXPECT_TRUE(policy.AllowsUserResize());
  EXPECT_FALSE(policy.AllowsResolutionChange());
  EXPECT_EQ(policy.ToggleTarget(WindowDisplayMode::BorderlessFullscreen), WindowDisplayMode::Windowed);
}
