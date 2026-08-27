
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
class EVOENGINE_API AnimationPlayer : public IPrivateComponent {
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
};

}  // namespace evo_engine
