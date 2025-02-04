
#pragma once
#include "Application.hpp"

#include "IPrivateComponent.hpp"

namespace evo_engine {

/**
 * @class UnknownPrivateComponent
 * @brief Represents a private component with an unknown type.
 *
 * This class is a special type of private component that is used to handle
 * components with unknown or unsupported types. It inherits from
 * IPrivateComponent.
 */
class UnknownPrivateComponent : public IPrivateComponent {
  /**
   * @brief Stores the original type name of the unknown component.
   */
  std::string original_type_name_{};

  /**
   * @brief Grants access to the Scene class.
   */
  friend class Scene;

  /**
   * @brief Grants access to the PrivateComponentHolder struct.
   */
  friend struct PrivateComponentHolder;

 public:
  /**
   * @brief Performs inspection of the component in the editor.
   *
   * @param editor_layer A shared pointer to the EditorLayer instance.
   * @return True if the inspection was successful, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

}  // namespace evo_engine
