//
// Created by lllll on 10/17/2022.
//

#include "CBTFGroup.hpp"

#include "BtfMaterial.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"

using namespace digital_agriculture_package;

void CBTFGroup::CollectAssetRef(std::vector<AssetRef>& list) {
  for (const auto& i : btfs)
    list.push_back(i);
}
void digital_agriculture_package::SerializeCBTFGroup(YAML::Emitter& out, const CBTFGroup& target) {
  if (!target.btfs.empty()) {
    out << YAML::Key << "btfs" << YAML::Value << YAML::BeginSeq;
    for (auto& c_btf : target.btfs) {
      out << YAML::BeginMap;
      c_btf.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}
void digital_agriculture_package::DeserializeCBTFGroup(const YAML::Node& in, CBTFGroup& target) {
  if (auto in_cbt_fs = in["btfs"]) {
    for (const auto& i : in_cbt_fs) {
      AssetRef ref;
      ref.Deserialize(i);
      target.btfs.emplace_back(ref);
    }
  }
}
std::shared_ptr<BtfMaterial> CBTFGroup::GetRandom() {
  if (!btfs.empty()) {
    return btfs[glm::linearRand(0, static_cast<int>(btfs.size()) - 1)].Get<BtfMaterial>();
  }
  return {};
}
