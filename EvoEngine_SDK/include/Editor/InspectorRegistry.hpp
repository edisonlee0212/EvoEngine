#pragma once

#include "EvoEngineAPI.hpp"
#include "IAsset.hpp"
#include "ILayer.hpp"
#include "IPrivateComponent.hpp"
#include "ISystem.hpp"

#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>

namespace evo_engine {
class EditorLayer;
class Scene;

struct InspectorContext {
  std::shared_ptr<EditorLayer> editor_layer;
  std::shared_ptr<Scene> scene;
};

class InspectorRegistry final {
 public:
  using Handler = std::function<bool(InspectorContext&, void*)>;

  struct HandlerInfo {
    std::type_index type = typeid(void);
    std::string type_name;
    std::string owner_name;
  };

  static EVOENGINE_API InspectorRegistry& GetInstance();

  void Clear();

  template <typename T>
  bool RegisterInspector(std::function<bool(InspectorContext&, T&)> handler, std::string owner_name = {},
                         std::string type_name = {}) {
    if (!handler) {
      return false;
    }
    if (type_name.empty()) {
      type_name = typeid(T).name();
    }
    return RegisterInspector(
        typeid(T),
        [handler = std::move(handler)](InspectorContext& context, void* target) {
          return handler(context, *static_cast<T*>(target));
        },
        std::move(owner_name), std::move(type_name));
  }

  template <typename T>
  bool RegisterDefaultInspector(std::string owner_name = {}, std::string type_name = {}) {
    (void)owner_name;
    (void)type_name;
    return false;
  }

  template <typename T>
  bool UnregisterInspector() {
    return UnregisterInspector(typeid(T));
  }

  template <typename T>
  [[nodiscard]] bool HasInspector() const {
    return HasInspector(typeid(T));
  }

  bool RegisterInspector(const std::type_info& type, Handler handler, std::string owner_name = {},
                         std::string type_name = {});
  bool UnregisterInspector(const std::type_info& type);
  size_t UnregisterOwner(const std::string& owner_name);

  [[nodiscard]] bool HasInspector(const std::type_info& type) const;
  [[nodiscard]] const HandlerInfo* FindInspector(const std::type_info& type) const;

  bool Inspect(InspectorContext& context, IAsset& asset) const;
  bool Inspect(InspectorContext& context, IPrivateComponent& component) const;
  bool Inspect(InspectorContext& context, ISystem& system) const;
  bool Inspect(InspectorContext& context, ILayer& layer) const;

 private:
  struct HandlerRecord {
    Handler handler;
    HandlerInfo info;
  };

  [[nodiscard]] const HandlerRecord* FindRecord(const std::type_info& type) const;

  std::unordered_map<std::type_index, HandlerRecord> handlers_;
};
}  // namespace evo_engine
