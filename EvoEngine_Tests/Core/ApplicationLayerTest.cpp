#include <gtest/gtest.h>
#include "Application.hpp"

using namespace evo_engine;

namespace {
class LifetimeLayer : public ILayer {
 public:
  std::shared_ptr<int> resource = std::make_shared<int>(1);
};
}  // namespace

TEST(ApplicationLayer, RemovingLayerReleasesDerivedMembersBeforePackageLifetime) {
  Application app;
  ApplicationContextScope scope(app);
  std::weak_ptr<int> resource;
  bool lifetime_released = false;
  auto lifetime = std::shared_ptr<void>(new int(0), [&](void* value) {
    EXPECT_TRUE(resource.expired());
    lifetime_released = true;
    delete static_cast<int*>(value);
  });
  auto layer = app.PushLayer<LifetimeLayer>("Lifetime", "Test", std::move(lifetime));
  resource = layer->resource;
  std::weak_ptr<ILayer> weak_layer = layer;
  layer.reset();
  app.PopLayer<LifetimeLayer>();
  EXPECT_TRUE(resource.expired());
  EXPECT_TRUE(weak_layer.expired());
  EXPECT_TRUE(lifetime_released);
}
