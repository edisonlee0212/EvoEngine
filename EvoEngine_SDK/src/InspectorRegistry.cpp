#include "InspectorRegistry.hpp"

#include "IAsset.hpp"
#include "ILayer.hpp"
#include "IPrivateComponent.hpp"
#include "ISystem.hpp"

using namespace evo_engine;

InspectorRegistry& InspectorRegistry::GetInstance() {
  static InspectorRegistry registry;
  return registry;
}

void InspectorRegistry::Clear() {
  handlers_.clear();
}

bool InspectorRegistry::RegisterInspector(const std::type_info& type, Handler handler, std::string owner_name,
                                          std::string type_name) {
  if (!handler) {
    return false;
  }
  if (type_name.empty()) {
    type_name = type.name();
  }

  const std::type_index type_index(type);
  handlers_.insert_or_assign(type_index, HandlerRecord{std::move(handler), HandlerInfo{type_index, std::move(type_name),
                                                                                       std::move(owner_name)}});
  return true;
}

bool InspectorRegistry::UnregisterInspector(const std::type_info& type) {
  return handlers_.erase(std::type_index(type)) != 0;
}

size_t InspectorRegistry::UnregisterOwner(const std::string& owner_name) {
  if (owner_name.empty()) {
    return 0;
  }

  size_t removed = 0;
  for (auto it = handlers_.begin(); it != handlers_.end();) {
    if (it->second.info.owner_name == owner_name) {
      it = handlers_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }
  return removed;
}

bool InspectorRegistry::HasInspector(const std::type_info& type) const {
  return FindRecord(type) != nullptr;
}

const InspectorRegistry::HandlerInfo* InspectorRegistry::FindInspector(const std::type_info& type) const {
  if (const auto* record = FindRecord(type)) {
    return &record->info;
  }
  return nullptr;
}

bool InspectorRegistry::Inspect(InspectorContext& context, IAsset& asset) const {
  if (const auto* record = FindRecord(typeid(asset))) {
    return record->handler(context, dynamic_cast<void*>(&asset));
  }
  return false;
}

bool InspectorRegistry::Inspect(InspectorContext& context, IPrivateComponent& component) const {
  if (const auto* record = FindRecord(typeid(component))) {
    return record->handler(context, dynamic_cast<void*>(&component));
  }
  return false;
}

bool InspectorRegistry::Inspect(InspectorContext& context, ISystem& system) const {
  if (const auto* record = FindRecord(typeid(system))) {
    return record->handler(context, dynamic_cast<void*>(&system));
  }
  return false;
}

bool InspectorRegistry::Inspect(InspectorContext& context, ILayer& layer) const {
  if (const auto* record = FindRecord(typeid(layer))) {
    return record->handler(context, dynamic_cast<void*>(&layer));
  }
  return false;
}

const InspectorRegistry::HandlerRecord* InspectorRegistry::FindRecord(const std::type_info& type) const {
  const auto search = handlers_.find(std::type_index(type));
  return search == handlers_.end() ? nullptr : &search->second;
}
