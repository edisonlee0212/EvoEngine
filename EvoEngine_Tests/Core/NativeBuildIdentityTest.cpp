#include <gtest/gtest.h>
#include "NativeBuildIdentity.hpp"

using namespace evo_engine;

TEST(NativeBuildIdentity, RunningSdkHasCompleteMatchingIdentity) {
  const auto& identity = GetNativeBuildIdentity();
  ASSERT_NE(identity.sdk_source_id, nullptr);
  EXPECT_EQ(std::string(identity.sdk_source_id).size(), 64);
  EXPECT_TRUE(IsNativeBuildCompatible(identity, identity));
  EXPECT_EQ(identity.with_editor, EVOENGINE_WITH_EDITOR != 0);
}

TEST(NativeBuildIdentity, RejectsIncompatibleNativeInputs) {
  const auto& expected = GetNativeBuildIdentity();
  const char* NativeBuildIdentity::* fields[] = {
      &NativeBuildIdentity::sdk_source_id, &NativeBuildIdentity::compiler_id, &NativeBuildIdentity::compiler_version,
      &NativeBuildIdentity::configuration, &NativeBuildIdentity::platform,    &NativeBuildIdentity::architecture};
  for (const auto field : fields) {
    auto candidate = expected;
    candidate.*field = "incompatible";
    std::string reason;
    EXPECT_FALSE(IsNativeBuildCompatible(expected, candidate, &reason));
    EXPECT_FALSE(reason.empty());
    candidate.*field = nullptr;
    EXPECT_FALSE(IsNativeBuildCompatible(expected, candidate));
  }
  auto candidate = expected;
  candidate.with_editor = !expected.with_editor;
  EXPECT_TRUE(IsNativeBuildCompatible(expected, candidate));
  EXPECT_FALSE(IsNativeBuildCompatible({}, {}));
}
