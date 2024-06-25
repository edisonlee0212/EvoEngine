#include "Noises.hpp"
#include "glm/gtc/noise.hpp"
using namespace eco_sys_lab;

void NoiseDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "type" << YAML::Value << type;
  out << YAML::Key << "frequency" << YAML::Value << frequency;
  out << YAML::Key << "intensity" << YAML::Value << intensity;
  out << YAML::Key << "multiplier" << YAML::Value << multiplier;
  out << YAML::Key << "min" << YAML::Value << min;
  out << YAML::Key << "max" << YAML::Value << max;
  out << YAML::Key << "offset" << YAML::Value << offset;
  out << YAML::Key << "shift" << YAML::Value << shift;
  out << YAML::Key << "ridgid" << YAML::Value << ridgid;
}

void NoiseDescriptor::Deserialize(const YAML::Node& in) {
  if (in["type"])
    type = in["type"].as<unsigned>();
  if (in["frequency"])
    frequency = in["frequency"].as<float>();
  if (in["intensity"])
    intensity = in["intensity"].as<float>();
  if (in["multiplier"])
    multiplier = in["multiplier"].as<float>();
  if (in["min"])
    min = in["min"].as<float>();
  if (in["max"])
    max = in["max"].as<float>();
  if (in["offset"])
    offset = in["offset"].as<float>();
  if (in["shift"])
    shift = in["shift"].as<glm::vec3>();
  if (in["ridgid"])
    ridgid = in["ridgid"].as<bool>();
}

Noise2D::Noise2D() {
  min_max = glm::vec2(-1000, 1000);
  noise_descriptors.clear();
  noise_descriptors.emplace_back();
}
Noise3D::Noise3D() {
  min_max = glm::vec2(-1000, 1000);
  noise_descriptors.clear();
  noise_descriptors.emplace_back();
}

bool Noise2D::OnInspect() {
  bool changed = false;
  if (ImGui::DragFloat2("Global Min/max", &min_max.x, 0, -1000, 1000)) {
    changed = true;
  }
  if (ImGui::Button("New start descriptor")) {
    changed = true;
    noise_descriptors.emplace_back();
  }
  for (int i = 0; i < noise_descriptors.size(); i++) {
    if (ImGui::TreeNodeEx(("No." + std::to_string(i)).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Remove")) {
        noise_descriptors.erase(noise_descriptors.begin() + i);
        changed = true;
        ImGui::TreePop();
        continue;
      }
      changed =
          ImGui::Combo("Type", {"Constant", "Linear", "Simplex", "Perlin"}, noise_descriptors[i].type) || changed;
      switch (static_cast<NoiseType>(noise_descriptors[i].type)) {
        case NoiseType::Perlin:
          changed =
              ImGui::DragFloat("Frequency", &noise_descriptors[i].frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Intensity", &noise_descriptors[i].intensity, 0.00001f, 1, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Multiplier", &noise_descriptors[i].multiplier, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &noise_descriptors[i].min, 0.01f, -99999, noise_descriptors[i].max)) {
            changed = true;
            noise_descriptors[i].min = glm::min(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          if (ImGui::DragFloat("Max", &noise_descriptors[i].max, 0.01f, noise_descriptors[i].min, 99999)) {
            changed = true;
            noise_descriptors[i].max = glm::max(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          changed = ImGui::DragFloat("Offset", &noise_descriptors[i].offset, 0.01f) || changed;
          changed = ImGui::Checkbox("Ridgid", &noise_descriptors[i].ridgid) || changed;
          break;
        case NoiseType::Simplex:
          changed =
              ImGui::DragFloat("Frequency", &noise_descriptors[i].frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Intensity", &noise_descriptors[i].intensity, 0.00001f, 1, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Multiplier", &noise_descriptors[i].multiplier, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &noise_descriptors[i].min, 0.01f, -99999, noise_descriptors[i].max)) {
            changed = true;
            noise_descriptors[i].min = glm::min(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          if (ImGui::DragFloat("Max", &noise_descriptors[i].max, 0.01f, noise_descriptors[i].min, 99999)) {
            changed = true;
            noise_descriptors[i].max = glm::max(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          changed = ImGui::DragFloat("Offset", &noise_descriptors[i].offset, 0.01f) || changed;
          changed = ImGui::Checkbox("Ridgid", &noise_descriptors[i].ridgid) || changed;
          break;
        case NoiseType::Constant:
          changed = ImGui::DragFloat("Value", &noise_descriptors[i].offset, 0.00001f, 0, 0, "%.5f") || changed;
          break;
        case NoiseType::Linear:
          changed =
              ImGui::DragFloat("X multiplier", &noise_descriptors[i].frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Y multiplier", &noise_descriptors[i].intensity, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::DragFloat("Base", &noise_descriptors[i].offset, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &noise_descriptors[i].min, 0.01f, -99999, noise_descriptors[i].max)) {
            changed = true;
            noise_descriptors[i].min = glm::min(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          if (ImGui::DragFloat("Max", &noise_descriptors[i].max, 0.01f, noise_descriptors[i].min, 99999)) {
            changed = true;
            noise_descriptors[i].max = glm::max(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          changed = ImGui::Checkbox("Ridgid", &noise_descriptors[i].ridgid) || changed;
          break;
      }
      ImGui::TreePop();
    }
  }
  return changed;
}

void Noise2D::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "min_max" << YAML::Value << min_max;
  out << YAML::Key << "noise_descriptors" << YAML::BeginSeq;
  for (const auto& i : noise_descriptors) {
    out << YAML::BeginMap;
    i.Serialize(out);
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
}

void Noise2D::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& node = in[name];
    if (node["min_max"])
      min_max = node["min_max"].as<glm::vec2>();

    if (node["noise_descriptors"]) {
      noise_descriptors.clear();
      for (const auto& i : node["noise_descriptors"]) {
        noise_descriptors.emplace_back();
        auto& back = noise_descriptors.back();
        back.Deserialize(i);
      }
    }
  }
}

void Noise2D::RandomOffset(const float min, const float max) {
  for (auto& i : noise_descriptors)
    i.offset = glm::linearRand(min, max);
}

void Noise3D::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "min_max" << YAML::Value << min_max;
  out << YAML::Key << "noise_descriptors" << YAML::BeginSeq;
  for (const auto& i : noise_descriptors) {
    i.Serialize(out);
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
}

void Noise3D::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& node = in[name];
    if (node["min_max"])
      min_max = node["min_max"].as<glm::vec2>();

    if (node["noise_descriptors"]) {
      noise_descriptors.clear();
      for (const auto& i : node["noise_descriptors"]) {
        noise_descriptors.emplace_back();
        auto& back = noise_descriptors.back();
        back.Deserialize(i);
      }
    }
  }
}

float Noise2D::GetValue(const glm::vec2& position) const {
  float ret_val = 0;

  for (const auto& noise_descriptor : noise_descriptors) {
    const auto actual_position = position + glm::vec2(noise_descriptor.shift);
    float noise = 0;
    switch (static_cast<NoiseType>(noise_descriptor.type)) {
      case NoiseType::Perlin:
        noise = glm::perlin(noise_descriptor.frequency * actual_position + glm::vec2(noise_descriptor.offset)) *
                noise_descriptor.multiplier;
        noise = glm::clamp(noise, noise_descriptor.min, noise_descriptor.max);
        break;
      case NoiseType::Simplex:
        noise = glm::simplex(noise_descriptor.frequency * actual_position + glm::vec2(noise_descriptor.offset)) *
                noise_descriptor.multiplier;
        noise = glm::clamp(noise, noise_descriptor.min, noise_descriptor.max);
        break;
      case NoiseType::Constant:
        noise = noise_descriptor.offset;
        break;
      case NoiseType::Linear:
        noise = noise_descriptor.offset + noise_descriptor.frequency * actual_position.x +
                noise_descriptor.intensity * actual_position.y;
        noise = glm::clamp(noise, noise_descriptor.min, noise_descriptor.max);
        break;
    }
    noise = glm::pow(noise, noise_descriptor.intensity);
    if (noise_descriptor.ridgid) {
      noise = -glm::abs(noise);
    }
    ret_val += noise;
  }
  return glm::clamp(ret_val, min_max.x, min_max.y);
}

bool Noise3D::OnInspect() {
  bool changed = false;
  if (ImGui::DragFloat2("Global Min/max", &min_max.x, 0, -1000, 1000)) {
    changed = true;
  }
  if (ImGui::Button("New start descriptor")) {
    changed = true;
    noise_descriptors.emplace_back();
  }
  for (int i = 0; i < noise_descriptors.size(); i++) {
    if (ImGui::TreeNodeEx(("No." + std::to_string(i)).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Remove")) {
        noise_descriptors.erase(noise_descriptors.begin() + i);
        changed = true;
        ImGui::TreePop();
        continue;
      }
      changed =
          ImGui::Combo("Type", {"Constant", "Linear", "Simplex", "Perlin"}, noise_descriptors[i].type) || changed;
      switch (static_cast<NoiseType>(noise_descriptors[i].type)) {
        case NoiseType::Perlin:
          changed =
              ImGui::DragFloat("Frequency", &noise_descriptors[i].frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Intensity", &noise_descriptors[i].intensity, 0.00001f, 1, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Multiplier", &noise_descriptors[i].multiplier, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &noise_descriptors[i].min, 0.01f, -99999, noise_descriptors[i].max)) {
            changed = true;
            noise_descriptors[i].min = glm::min(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          if (ImGui::DragFloat("Max", &noise_descriptors[i].max, 0.01f, noise_descriptors[i].min, 99999)) {
            changed = true;
            noise_descriptors[i].max = glm::max(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          changed = ImGui::DragFloat("Offset", &noise_descriptors[i].offset, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::DragFloat3("Shift", &noise_descriptors[i].shift.x, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::Checkbox("Ridgid", &noise_descriptors[i].ridgid) || changed;
          break;
        case NoiseType::Simplex:
          changed =
              ImGui::DragFloat("Frequency", &noise_descriptors[i].frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Intensity", &noise_descriptors[i].intensity, 0.00001f, 1, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Multiplier", &noise_descriptors[i].multiplier, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &noise_descriptors[i].min, 0.01f, -99999, noise_descriptors[i].max)) {
            changed = true;
            noise_descriptors[i].min = glm::min(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          if (ImGui::DragFloat("Max", &noise_descriptors[i].max, 0.01f, noise_descriptors[i].min, 99999)) {
            changed = true;
            noise_descriptors[i].max = glm::max(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          changed = ImGui::DragFloat("Offset", &noise_descriptors[i].offset, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::DragFloat3("Shift", &noise_descriptors[i].shift.x, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::Checkbox("Ridgid", &noise_descriptors[i].ridgid) || changed;
          break;
        case NoiseType::Constant:
          changed = ImGui::DragFloat("Value", &noise_descriptors[i].offset, 0.00001f, 0, 0, "%.5f") || changed;
          break;
        case NoiseType::Linear:
          changed =
              ImGui::DragFloat("X multiplier", &noise_descriptors[i].frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Y multiplier", &noise_descriptors[i].intensity, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Z multiplier", &noise_descriptors[i].multiplier, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::DragFloat("Base", &noise_descriptors[i].offset, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &noise_descriptors[i].min, 0.01f, -99999, noise_descriptors[i].max)) {
            changed = true;
            noise_descriptors[i].min = glm::min(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          if (ImGui::DragFloat("Max", &noise_descriptors[i].max, 0.01f, noise_descriptors[i].min, 99999)) {
            changed = true;
            noise_descriptors[i].max = glm::max(noise_descriptors[i].min, noise_descriptors[i].max);
          }
          changed = ImGui::DragFloat3("Shift", &noise_descriptors[i].shift.x, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::Checkbox("Ridgid", &noise_descriptors[i].ridgid) || changed;
          break;
      }
      ImGui::TreePop();
    }
  }
  return changed;
}

float Noise3D::GetValue(const glm::vec3& position) const {
  float ret_val = 0;
  for (const auto& noise_descriptor : noise_descriptors) {
    const auto actual_position = position + noise_descriptor.shift;
    float noise = 0;
    switch (static_cast<NoiseType>(noise_descriptor.type)) {
      case NoiseType::Perlin:
        noise = glm::perlin(noise_descriptor.frequency * actual_position + glm::vec3(noise_descriptor.offset)) *
                noise_descriptor.multiplier;
        noise = glm::clamp(noise, noise_descriptor.min, noise_descriptor.max);
        break;
      case NoiseType::Simplex:
        noise = glm::simplex(noise_descriptor.frequency * actual_position + glm::vec3(noise_descriptor.offset)) *
                noise_descriptor.multiplier;
        noise = glm::clamp(noise, noise_descriptor.min, noise_descriptor.max);
        break;
      case NoiseType::Constant:
        noise = noise_descriptor.offset;
        break;
      case NoiseType::Linear:
        noise = noise_descriptor.offset + noise_descriptor.frequency * actual_position.x +
                noise_descriptor.intensity * actual_position.y + noise_descriptor.multiplier * actual_position.z;
        noise = glm::clamp(noise, noise_descriptor.min, noise_descriptor.max);
        break;
    }
    noise = glm::pow(noise, noise_descriptor.intensity);
    if (noise_descriptor.ridgid) {
      noise = -glm::abs(noise);
    }

    ret_val += noise;
  }
  return glm::clamp(ret_val, min_max.x, min_max.y);
}