#pragma once
#include <IHandle.hpp>
namespace YAML {
class Emitter;
class Node;
}  // namespace YAML

namespace evo_engine {
class ISerializable : public IHandle {
  friend class Serialization;
  friend class IAsset;
  friend class Entities;
  friend class Scene;
  friend class Serialization;
  friend struct EntityMetadata;
  friend class AssetRecord;
  friend class Folder;
  std::string type_name_;

 public:
  void Save(const std::string &name, YAML::Emitter &out) const;
  void Load(const std::string &name, const YAML::Node &in);
  [[nodiscard]] std::string GetTypeName() {
    return type_name_;
  }
  virtual ~ISerializable() = default;
  virtual void Serialize(YAML::Emitter &out) const {
  }
  virtual void Deserialize(const YAML::Node &in) {
  }
};
}  // namespace evo_engine
