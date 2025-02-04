
#pragma once
#include "IPrivateComponent.hpp"

namespace evo_engine {

/**
 * @class AnimationPlayer
 * @brief A class responsible for playing animations within the evo_engine.
 *
 * This class allows the control of animations, with features such as auto-play
 * and adjustable playback speed.
 */
class AnimationPlayer : public IPrivateComponent {
 public:
  /**
   * @brief Indicates whether the animation should auto-play on start.
   * Default value is true.
   */
  bool auto_play = true;

  /**
   * @brief The speed at which the animation auto-plays.
   * Default value is 30.0f.
   */
  float auto_play_speed = 30.0f;

  /**
   * @brief Updates the animation playback logic.
   */
  void Update() override;

  /**
   * @brief Renders the inspector properties of the animation player in the editor.
   *
   * @param editor_layer A shared pointer to the editor layer.
   * @return True if the inspection was successful, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  // /**
  // * @brief Serializes the animation player's data into a YAML emitter.
  // * @param out The YAML emitter to save data into.
  // */
  // void Save(YAML::Emitter& out) override;

  // /**
  // * @brief Deserializes the animation player's data from a YAML node.
  // * @param in The YAML node containing the serialized data.
  // */
  // void Deserialize(const YAML::Node& in) override;
};

}  // namespace evo_engine
