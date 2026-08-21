#include "EvoEngine_SDK_PCH.hpp"

#ifdef min
#  undef min
#endif
#ifdef max
#  undef max
#endif

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "OrganMeshChannel.hpp"
#include "Scene.hpp"

using namespace evo_engine;
using namespace l_system_package;

namespace {
struct InteractivePublishTestContext {
  Application app;
  ApplicationContextScope scope{app};
  std::shared_ptr<Scene> scene;

  InteractivePublishTestContext() {
    ApplicationInitializationSettings settings;
    settings.allow_empty_project = true;
    settings.load_default_resources = false;
    settings.load_project_assets = false;
    settings.load_project_start_scene = false;
    settings.enable_runtime_packages = false;
    app.Initialize(settings);
    scene = AssetManager::CreateTemporaryAsset<Scene>();
    app.Attach(scene);
  }
};

std::vector<Vertex> MakeVertices(const size_t count) {
  std::vector<Vertex> vertices(count);
  for (size_t index = 0; index < count; ++index) {
    vertices[index].position = {static_cast<float>(index & 1u), static_cast<float>((index >> 1u) & 1u), 0.0f};
    vertices[index].normal = {0.0f, 0.0f, 1.0f};
  }
  return vertices;
}

VertexAttributes MakeAttributes() {
  VertexAttributes attributes;
  attributes.normal = true;
  attributes.tangent = true;
  return attributes;
}
}  // namespace

TEST(LSystemInteractivePublish, KeepsDisplayedMeshUntilReplacementIsReady) {
  InteractivePublishTestContext context;
  const auto root = context.scene->CreateEntity("Plant");
  OrganMeshChannel channel(context.scene, root, "Leaves");
  const auto displayed_before = channel.GetMesh();

  channel.Stage(MakeAttributes(), MakeVertices(3), {{0u, 1u, 2u}});
  EXPECT_TRUE(channel.Flush());
  EXPECT_EQ(channel.GetMesh(), displayed_before);
  EXPECT_TRUE(channel.HasPending());

  EXPECT_TRUE(channel.Flush());
  EXPECT_NE(channel.GetMesh(), displayed_before);
  EXPECT_EQ(channel.GetMesh()->GetTriangleAmount(), 1u);
  EXPECT_FALSE(channel.HasPending());
}

TEST(LSystemInteractivePublish, CoalescesToLatestPayloadWhileAReplacementIsPending) {
  InteractivePublishTestContext context;
  const auto root = context.scene->CreateEntity("Plant");
  OrganMeshChannel channel(context.scene, root, "Leaves");

  channel.Stage(MakeAttributes(), MakeVertices(3), {{0u, 1u, 2u}});
  ASSERT_TRUE(channel.Flush());

  channel.Stage(MakeAttributes(), MakeVertices(4), {{0u, 1u, 2u}, {1u, 3u, 2u}});
  channel.Stage(MakeAttributes(), MakeVertices(5), {{0u, 1u, 2u}, {1u, 3u, 2u}, {1u, 4u, 3u}});

  ASSERT_TRUE(channel.Flush());
  EXPECT_EQ(channel.GetMesh()->GetTriangleAmount(), 1u);
  EXPECT_TRUE(channel.HasPending());

  ASSERT_TRUE(channel.Flush());
  EXPECT_EQ(channel.GetMesh()->GetTriangleAmount(), 3u);
  EXPECT_FALSE(channel.HasPending());
}
