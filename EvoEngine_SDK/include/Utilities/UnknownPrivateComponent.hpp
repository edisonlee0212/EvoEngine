
#pragma once
#include "Application.hpp"

#include "IAsset.hpp"
#include "ILayer.hpp"
#include "IPrivateComponent.hpp"
#include "ISystem.hpp"

namespace evo_engine {

class EVOENGINE_API UnknownRuntimePayload {
 protected:
  std::string original_type_name_{};
  YAML::Node serialized_node_{};

  void SerializePayload(YAML::Emitter& out, const std::vector<std::string>& skipped_keys) const;

 public:
  void SetOriginalTypeName(const std::string& type_name);
  [[nodiscard]] const std::string& GetOriginalTypeName() const;
  void SetSerializedNode(const YAML::Node& node);
  [[nodiscard]] const YAML::Node& GetSerializedNode() const;
};

/**
 * @class UnknownPrivateComponent
 * @brief Represents a private component with an unknown type.
 *
 * This class is a special type of private component that is used to handle
 * components with unknown or unsupported types. It inherits from
 * IPrivateComponent.
 */
class UnknownPrivateComponent : public IPrivateComponent, public UnknownRuntimePayload {
  /**
   * @brief Grants access to the Scene class.
   */
  friend class Scene;

  /**
   * @brief Grants access to the PrivateComponentHolder struct.
   */
  friend struct PrivateComponentHolder;

 public:
};

class UnknownAsset : public IAsset, public UnknownRuntimePayload {
 public:
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }
};

class UnknownSystem : public ISystem, public UnknownRuntimePayload {
 public:
};

class EVOENGINE_API UnknownLayer : public ILayer {
  std::string original_type_name_{};

 public:
  void SetOriginalTypeName(const std::string& type_name);
  [[nodiscard]] const std::string& GetOriginalTypeName() const;
};

struct UnknownDataComponent : IDataComponent {
  uint64_t original_type_hash = 0;
};

}  // namespace evo_engine
