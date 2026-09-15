#include <gtest/gtest.h>
#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "Serialization.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_package;
namespace {
std::string SaveTree(const Tree& tree) {
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeTree(out, tree);
  out << YAML::EndMap;
  return out.c_str();
}
}  // namespace
TEST(TreeModelRevision, DeserializationInvalidatesDerivedStateWithoutChangingSavedFormat) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  ASSERT_TRUE(evo_engine::Serialization::RegisterSerializationHandler<evo_engine::AssetRef>(
      [](YAML::Emitter& out, const evo_engine::AssetRef& reference) {
        reference.Serialize(out);
      },
      [](const YAML::Node& in, evo_engine::AssetRef& reference) {
        reference.Deserialize(in);
      },
      {}, "AssetRef"));
  Tree tree;
  const auto saved = SaveTree(tree);
  const auto shoot_before = tree.GetShootModelRevision();
  const auto root_before = tree.GetRootModelRevision();
  DeserializeTree(YAML::Load("{}"), tree);
  EXPECT_GT(tree.GetShootModelRevision().content, shoot_before.content);
  EXPECT_GT(tree.GetShootModelRevision().topology, shoot_before.topology);
  EXPECT_GT(tree.GetRootModelRevision().content, root_before.content);
  EXPECT_GT(tree.GetRootModelRevision().topology, root_before.topology);
  EXPECT_EQ(SaveTree(tree), saved);
  const auto revision = tree.GetShootModelRevision();
  DeserializeTree(YAML::Load("{}"), tree);
  EXPECT_GT(tree.GetShootModelRevision().content, revision.content);
  EXPECT_EQ(SaveTree(tree), saved);
}
