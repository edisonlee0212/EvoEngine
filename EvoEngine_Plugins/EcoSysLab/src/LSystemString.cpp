#include "LSystemString.hpp"

using namespace eco_sys_lab;

bool LSystemString::LoadInternal(const std::filesystem::path& path) {
  if (path.extension().string() == ".lstring") {
    auto string = FileUtils::LoadFileAsString(path);
    ParseLString(string);
    return true;
  }
  return false;
}

bool LSystemString::SaveInternal(const std::filesystem::path& path) const {
  if (path.extension().string() == ".lstring") {
    std::ofstream of;
    of.open(path.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (of.is_open()) {
      std::string output;
      for (const auto& command : m_commands) {
        switch (command.m_type) {
          case LSystemCommandType::Forward: {
            output += "F(";
            output += std::to_string(command.m_value);
            output += ")";
          } break;
          case LSystemCommandType::PitchUp: {
            output += "^(";
            output += std::to_string(command.m_value);
            output += ")";
          } break;
          case LSystemCommandType::PitchDown: {
            output += "&(";
            output += std::to_string(command.m_value);
            output += ")";
          } break;
          case LSystemCommandType::TurnLeft: {
            output += "+(";
            output += std::to_string(command.m_value);
            output += ")";
          } break;
          case LSystemCommandType::TurnRight: {
            output += "-(";
            output += std::to_string(command.m_value);
            output += ")";
          } break;
          case LSystemCommandType::RollLeft: {
            output += "\\(";
            output += std::to_string(command.m_value);
            output += ")";
          } break;
          case LSystemCommandType::RollRight: {
            output += "/(";
            output += std::to_string(command.m_value);
            output += ")";
          } break;
          case LSystemCommandType::Push: {
            output += "[";
          } break;
          case LSystemCommandType::Pop: {
            output += "]";
          } break;
        }
        output += "\n";
      }
      of.write(output.c_str(), output.size());
      of.flush();
    }
    return true;
  }
  return false;
}

void LSystemString::ParseLString(const std::string& string) {
  std::istringstream iss(string);
  std::string line;
  int stack_check = 0;
  while (std::getline(iss, line)) {
    if (line.empty())
      continue;
    LSystemCommand command;
    switch (line[0]) {
      case 'F': {
        command.m_type = LSystemCommandType::Forward;
      } break;
      case '+': {
        command.m_type = LSystemCommandType::TurnLeft;
      } break;
      case '-': {
        command.m_type = LSystemCommandType::TurnRight;
      } break;
      case '^': {
        command.m_type = LSystemCommandType::PitchUp;
      } break;
      case '&': {
        command.m_type = LSystemCommandType::PitchDown;
      } break;
      case '\\': {
        command.m_type = LSystemCommandType::RollLeft;
      } break;
      case '/': {
        command.m_type = LSystemCommandType::RollRight;
      } break;
      case '[': {
        command.m_type = LSystemCommandType::Push;
        stack_check++;
      } break;
      case ']': {
        command.m_type = LSystemCommandType::Pop;
        stack_check--;
      } break;
    }
    if (command.m_type == LSystemCommandType::Unknown)
      continue;
    if (command.m_type != LSystemCommandType::Push && command.m_type != LSystemCommandType::Pop) {
      command.m_value = std::stof(line.substr(2));
      if (command.m_type == LSystemCommandType::Forward && command.m_value > 2.0f) {
        command.m_value = 3.0f;
      }
    }
    m_commands.push_back(command);
  }
  if (stack_check != 0) {
    EVOENGINE_ERROR("Stack check failed! Something wrong with the string!");
    m_commands.clear();
  }
}

bool LSystemString::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Text(("Command Size: " + std::to_string(m_commands.size())).c_str());
  if (ImGui::DragFloat("Internode Length", &m_internodeLength))
    changed = true;
  if (ImGui::DragFloat("Thickness Factor", &m_thicknessFactor))
    changed = true;
  if (ImGui::DragFloat("End node thickness", &m_endNodeThickness))
    changed = true;
  return changed;
}