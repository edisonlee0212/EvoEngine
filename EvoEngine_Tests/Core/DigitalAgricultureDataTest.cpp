#include "DigitalAgriculture_PCH.hpp"
#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "BtfMaterial.hpp"
#include "BtfMeshRenderer.hpp"
#include "CBTFGroup.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "PARSensorGroup.hpp"

using namespace digital_agriculture_package;
using namespace evo_engine;

namespace {
class DigitalAgricultureDataTest : public testing::Test {
 protected:
  Application application_;
  ApplicationContextScope application_scope_{application_};

  void SetUp() override {
    Serialization::RegisterSerializationHandler<AssetRef>(
        [](YAML::Emitter& out, const AssetRef& reference) {
          reference.Serialize(out);
        },
        [](const YAML::Node& in, AssetRef& reference) {
          reference.Deserialize(in);
        },
        "DigitalAgricultureTests", "AssetRef");
  }

  void TearDown() override {
    Serialization::UnregisterSerializationHandlersByOwner("DigitalAgricultureTests");
  }
};

template <typename T>
YAML::Node Serialize(const T& value, void (*serializer)(YAML::Emitter&, const T&)) {
  YAML::Emitter out;
  out << YAML::BeginMap;
  serializer(out, value);
  out << YAML::EndMap;
  return YAML::Load(out.c_str());
}
}  // namespace

TEST_F(DigitalAgricultureDataTest, BtfMaterialRoundTripsWithoutDeviceState) {
  BtfMaterial source;
  source.btf_base.has_data = true;
  source.btf_base.gamma = 2.2f;
  source.btf_base.pdf6d.row_size = 7;
  source.shared_coordinates_beta_angles = {-90.0f, 0.0f, 90.0f};
  source.pdf6d = {1, 2, 3};
  source.pdf6d_scales = {0.25f, 0.5f, 1.0f};

  BtfMaterial restored;
  DeserializeBtfMaterial(Serialize(source, SerializeBtfMaterial), restored);

  EXPECT_TRUE(restored.btf_base.has_data);
  EXPECT_FLOAT_EQ(restored.btf_base.gamma, 2.2f);
  EXPECT_EQ(restored.btf_base.pdf6d.row_size, 7);
  EXPECT_EQ(restored.shared_coordinates_beta_angles, source.shared_coordinates_beta_angles);
  EXPECT_EQ(restored.pdf6d, source.pdf6d);
  EXPECT_EQ(restored.pdf6d_scales, source.pdf6d_scales);
}

TEST_F(DigitalAgricultureDataTest, ParSensorSamplesRoundTrip) {
  PARSensorGroup source;
  source.samplers.resize(1);
  source.samplers[0].v_0.position = {1.0f, 2.0f, 3.0f};
  source.samplers[0].direction = {0.0f, 1.0f, 0.0f};
  source.samplers[0].energy = {4.0f, 5.0f, 6.0f};
  source.samplers[0].back_face = false;

  PARSensorGroup restored;
  DeserializePARSensorGroup(Serialize(source, SerializePARSensorGroup), restored);

  ASSERT_EQ(restored.samplers.size(), 1);
  EXPECT_FLOAT_EQ(restored.samplers[0].v_0.position.x, 1.0f);
  EXPECT_FLOAT_EQ(restored.samplers[0].direction.y, 1.0f);
  EXPECT_FLOAT_EQ(restored.samplers[0].energy.z, 6.0f);
  EXPECT_FALSE(restored.samplers[0].back_face);
}

TEST_F(DigitalAgricultureDataTest, BtfComponentRetainsBothAssetReferences) {
  BtfMeshRenderer renderer;
  std::vector<AssetRef> references;
  renderer.CollectAssetRef(references);
  EXPECT_EQ(references.size(), 2);

  const auto serialized = Serialize(renderer, SerializeBtfMeshRenderer);
  EXPECT_TRUE(serialized["mesh"]);
  EXPECT_TRUE(serialized["btf"]);
}

TEST_F(DigitalAgricultureDataTest, CbtfGroupRetainsAssetReferences) {
  CBTFGroup source;
  source.btfs.emplace_back();

  CBTFGroup restored;
  DeserializeCBTFGroup(Serialize(source, SerializeCBTFGroup), restored);

  EXPECT_EQ(restored.btfs.size(), 1);
  std::vector<AssetRef> references;
  restored.CollectAssetRef(references);
  EXPECT_EQ(references.size(), 1);
}
