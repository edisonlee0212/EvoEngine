#include "NodeGraphEditor.hpp"
#include "Application.hpp"

using namespace evo_engine;

namespace {
using GraphContexts =
    std::map<std::weak_ptr<IAsset>, std::unique_ptr<ImNodesEditorContext>, std::owner_less<std::weak_ptr<IAsset>>>;
auto& Contexts() {
  static std::unordered_map<Application*, GraphContexts> contexts;
  return contexts;
}
}  // namespace
ImNodesEditorContext& NodeGraphEditor::GetContext(const std::shared_ptr<IAsset>& owner) {
  auto* application = &ApplicationContext::Get();
  auto [it, inserted] = Contexts().try_emplace(application);
  if (inserted) {
    static_cast<void>(application->RegisterCleanupFunction([application] {
      Contexts().erase(application);
    }));
  }
  auto& contexts = it->second;
  for (auto entry = contexts.begin(); entry != contexts.end();) {
    if (entry->first.expired())
      entry = contexts.erase(entry);
    else
      ++entry;
  }
  auto& context = contexts[owner];
  if (!context)
    context = std::make_unique<ImNodesEditorContext>();
  return *context;
}
void NodeGraphEditor::Clear() {
  const auto it = Contexts().find(ApplicationContext::TryGet());
  if (it != Contexts().end())
    it->second.clear();
}
