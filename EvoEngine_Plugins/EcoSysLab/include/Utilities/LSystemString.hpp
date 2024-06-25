#pragma once

#include "Skeleton.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
enum class LSystemCommandType {
  Unknown,
  /**
   * Command F
   */
  Forward,
  /**
   * Command +
   */
  TurnLeft,
  /**
   * Command -
   */
  TurnRight,
  /**
   * Command ^
   */
  PitchUp,
  /**
   * Command &
   */
  PitchDown,
  /**
   * Command \
   */
  RollLeft,
  /**
   * Command /
   */
  RollRight,
  /**
   * Command [
   */
  Push,
  /**
   * Command ]
   */
  Pop
};

struct LSystemCommand {
  LSystemCommandType m_type = LSystemCommandType::Unknown;
  float m_value = 0.0f;
};

class LSystemString : public IAsset {
 protected:
  bool SaveInternal(const std::filesystem::path& path) const override;

  bool LoadInternal(const std::filesystem::path& path) override;

 public:
  float m_internodeLength = 1.0f;
  float m_thicknessFactor = 0.5f;
  float m_endNodeThickness = 0.02f;

  void ParseLString(const std::string& string);

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  std::vector<LSystemCommand> m_commands;
};
}  // namespace eco_sys_lab