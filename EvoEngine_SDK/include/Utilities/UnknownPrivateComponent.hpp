#pragma once
#include "Application.hpp"

#include "IPrivateComponent.hpp"

namespace evo_engine {
class UnknownPrivateComponent : public IPrivateComponent {
  std::string original_type_name_{};
  friend class Scene;
  friend struct PrivateComponentHolder;

 public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

}  // namespace evo_engine
