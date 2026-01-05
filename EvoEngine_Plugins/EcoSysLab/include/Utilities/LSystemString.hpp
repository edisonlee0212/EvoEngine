#pragma once

#include "Skeleton.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @enum LSystemCommandType
 * @brief Enumerates the different types of L-system commands.
 */
enum class LSystemCommandType {
  Unknown,  ///< Unknown command type.
  /**
   * @brief Command F - Move forward.
   */
  Forward,
  /**
   * @brief Command + - Turn left.
   */
  TurnLeft,
  /**
   * @brief Command - - Turn right.
   */
  TurnRight,
  /**
   * @brief Command ^ - Pitch up.
   */
  PitchUp,
  /**
   * @brief Command & - Pitch down.
   */
  PitchDown,
  /**
   * @brief Command \ - Roll left.
   */
  RollLeft,
  /**
   * @brief Command / - Roll right.
   */
  RollRight,
  /**
   * @brief Command [ - Push current state onto stack.
   */
  Push,
  /**
   * @brief Command ] - Pop top state from stack.
   */
  Pop
};

/**
 * @struct LSystemCommand
 * @brief Represents an individual L-system command with an associated value.
 */
struct LSystemCommand {
  LSystemCommandType m_type = LSystemCommandType::Unknown;  ///< Type of command.
  float m_value = 0.0f;                                     ///< Command-specific value.
};

/**
 * @class LSystemString
 * @brief Represents an L-system string and provides functionality to parse and manipulate it.
 */
class LSystemString : public IAsset {
 protected:
  /**
   * @brief Saves the L-system string to a file.
   * @param path File path where the L-system string should be saved.
   * @return True if the save operation was successful, otherwise false.
   */
  bool SaveInternal(const std::filesystem::path& path) const override;

  /**
   * @brief Loads the L-system string from a file.
   * @param path File path from which to load the L-system string.
   * @return True if the load operation was successful, otherwise false.
   */
  bool LoadInternal(const std::filesystem::path& path) override;

 public:
  float m_internodeLength = 1.0f;    ///< Length of each internode in the L-system.
  float m_thicknessFactor = 0.5f;    ///< Factor determining the thickness of branches.
  float m_endNodeThickness = 0.02f;  ///< Thickness of the final branch node.

  /**
   * @brief Parses an L-system string into a sequence of commands.
   * @param string The L-system string to parse.
   */
  void ParseLString(const std::string& string);

  /**
   * @brief Inspects the L-system string within the editor.
   * @param editor_layer Shared pointer to the editor layer.
   * @return True if the asset's content remains unmodified during inspection, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  std::vector<LSystemCommand> m_commands;  ///< A sequence of parsed L-system commands.
};
}  // namespace eco_sys_lab_plugin