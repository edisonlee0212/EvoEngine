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
#include <vector>

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
  using BatchHandler = std::function<bool(InspectorContext&, const std::vector<void*>&)>;

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
  bool RegisterBatchInspector(
      std::function<bool(InspectorContext&, const std::vector<std::reference_wrapper<T>>&)> handler,
      std::string owner_name = {}, std::string type_name = {}) {
    if (!handler)
      return false;
    if (type_name.empty())
      type_name = typeid(T).name();
    return RegisterBatchInspector(
        typeid(T),
        [handler = std::move(handler)](InspectorContext& context, const std::vector<void*>& targets) {
          std::vector<std::reference_wrapper<T>> typed_targets;
          typed_targets.reserve(targets.size());
          for (auto* target : targets)
            typed_targets.emplace_back(*static_cast<T*>(target));
          return handler(context, typed_targets);
        },
        std::move(owner_name), std::move(type_name));
  }

  template <typename T>
  bool UnregisterInspector() {
    return UnregisterInspector(typeid(T));
  }
  template <typename T>
  bool UnregisterBatchInspector() {
    return UnregisterBatchInspector(typeid(T));
  }

  template <typename T>
  [[nodiscard]] bool HasInspector() const {
    return HasInspector(typeid(T));
  }
  template <typename T>
  [[nodiscard]] bool HasBatchInspector() const {
    return FindBatchInspector(typeid(T)) != nullptr;
  }

  bool RegisterInspector(const std::type_info& type, Handler handler, std::string owner_name = {},
                         std::string type_name = {});
  bool RegisterBatchInspector(const std::type_info& type, BatchHandler handler, std::string owner_name = {},
                              std::string type_name = {});
  bool UnregisterInspector(const std::type_info& type);
  bool UnregisterBatchInspector(const std::type_info& type);
  size_t UnregisterOwner(const std::string& owner_name);

  [[nodiscard]] bool HasInspector(const std::type_info& type) const;
  [[nodiscard]] const HandlerInfo* FindInspector(const std::type_info& type) const;
  [[nodiscard]] const HandlerInfo* FindBatchInspector(const std::type_info& type) const;

  bool Inspect(InspectorContext& context, IAsset& asset) const;
  bool Inspect(InspectorContext& context, IPrivateComponent& component) const;
  bool Inspect(InspectorContext& context, ISystem& system) const;
  bool Inspect(InspectorContext& context, ILayer& layer) const;
  bool InspectBatch(InspectorContext& context, const std::vector<std::shared_ptr<IPrivateComponent>>& components) const;

 private:
  struct HandlerRecord {
    Handler handler;
    HandlerInfo info;
  };
  struct BatchHandlerRecord {
    BatchHandler handler;
    HandlerInfo info;
  };

  [[nodiscard]] const HandlerRecord* FindRecord(const std::type_info& type) const;

  std::unordered_map<std::type_index, HandlerRecord> handlers_;
  std::unordered_map<std::type_index, BatchHandlerRecord> batch_handlers_;
};
}  // namespace evo_engine
