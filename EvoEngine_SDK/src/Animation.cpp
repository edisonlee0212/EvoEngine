#include "Animation.hpp"
using namespace evo_engine;
void Bone::Animate(const std::string& target_name, const float& animation_time, const glm::mat4& parent_transform,
                   const glm::mat4& root_transform, std::vector<glm::mat4>& results) {
  glm::mat4 global_transform = parent_transform;
  if (const auto search = animations.find(target_name); search != animations.end()) {
    const auto translation = animations[target_name].InterpolatePosition(animation_time);
    const auto rotation = animations[target_name].InterpolateRotation(animation_time);
    const auto scale = animations[target_name].InterpolateScaling(animation_time);
    global_transform *= translation * rotation * scale;
  }

  results[index] = global_transform;
  for (auto& i : children) {
    i->Animate(target_name, animation_time, global_transform, root_transform, results);
  }
}

bool Bone::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNode((name + "##" + std::to_string(index)).c_str())) {
    ImGui::Text("Controller: ");
    ImGui::SameLine();
    for (auto& i : children) {
      if (i->OnInspect())
        changed = true;
    }
    ImGui::TreePop();
  }
  return changed;
}


int BoneKeyFrames::GetPositionIndex(const float& animation_time) const {
  const int size = positions.size();
  for (int index = 0; index < size - 1; ++index) {
    if (animation_time < positions[index + 1].time_stamp)
      return index;
  }
  return size - 2;
}

int BoneKeyFrames::GetRotationIndex(const float& animation_time) const {
  const int size = rotations.size();
  for (int index = 0; index < size - 1; ++index) {
    if (animation_time < rotations[index + 1].time_stamp)
      return index;
  }
  return size - 2;
}

int BoneKeyFrames::GetScaleIndex(const float& animation_time) const {
  const int size = scales.size();
  for (int index = 0; index < size - 1; ++index) {
    if (animation_time < scales[index + 1].time_stamp)
      return index;
  }
  return size - 2;
}

float BoneKeyFrames::GetScaleFactor(const float& last_time_stamp, const float& next_time_stamp,
                                    const float& animation_time) {
  const float mid_way_length = animation_time - last_time_stamp;
  const float frames_diff = next_time_stamp - last_time_stamp;
  if (frames_diff == 0.0f)
    return 0.0f;
  return glm::clamp(mid_way_length / frames_diff, 0.0f, 1.0f);
}

glm::mat4 BoneKeyFrames::InterpolatePosition(const float& animation_time) const {
  if (1 == positions.size())
    return glm::translate(glm::mat4(1.0f), positions[0].value);

  const int p0_index = GetPositionIndex(animation_time);
  const int p1_index = p0_index + 1;
  const float scale_factor =
      GetScaleFactor(positions[p0_index].time_stamp, positions[p1_index].time_stamp, animation_time);
  const glm::vec3 final_position = glm::mix(positions[p0_index].value, positions[p1_index].value, scale_factor);
  return glm::translate(final_position);
}

glm::mat4 BoneKeyFrames::InterpolateRotation(const float& animation_time) const {
  if (1 == rotations.size()) {
    const auto rotation = glm::normalize(rotations[0].value);
    return glm::mat4_cast(rotation);
  }

  const int p0_index = GetRotationIndex(animation_time);
  const int p1_index = p0_index + 1;
  const float scale_factor =
      GetScaleFactor(rotations[p0_index].time_stamp, rotations[p1_index].time_stamp, animation_time);
  glm::quat final_rotation = glm::slerp(rotations[p0_index].value, rotations[p1_index].value, scale_factor);
  final_rotation = glm::normalize(final_rotation);
  return glm::mat4_cast(final_rotation);
}

glm::mat4 BoneKeyFrames::InterpolateScaling(const float& animation_time) const {
  if (1 == scales.size())
    return glm::scale(scales[0].m_value);

  const int p0_index = GetScaleIndex(animation_time);
  const int p1_index = p0_index + 1;
  const float scale_factor = GetScaleFactor(scales[p0_index].time_stamp, scales[p1_index].time_stamp, animation_time);
  const glm::vec3 final_scale = glm::mix(scales[p0_index].m_value, scales[p1_index].m_value, scale_factor);
  return glm::scale(final_scale);
}



std::shared_ptr<Bone>& Animation::UnsafeGetRootBone() {
  return root_bone;
}

std::map<std::string, float>& Animation::UnsafeGetAnimationLengths() {
  return animation_length;
}

bool Animation::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (!root_bone)
    return changed;
  ImGui::Text(("Bone size: " + std::to_string(bone_size)).c_str());
  if (root_bone->OnInspect())
    changed = true;

  return changed;
}

void Animation::Animate(const std::string& name, const float& animation_time, const glm::mat4& root_transform,
                        std::vector<glm::mat4>& results) {
  if (animation_length.find(name) == animation_length.end() || !root_bone) {
    return;
  }
  root_bone->Animate(name, animation_time, root_transform, root_transform, results);
}

std::string Animation::GetFirstAvailableAnimationName() const {
  if (animation_length.empty()) {
    throw std::runtime_error("Animation Empty!");
  }
  return animation_length.begin()->first;
}

float Animation::GetAnimationLength(const std::string& animation_name) const {
  if (const auto search = animation_length.find(animation_name); search != animation_length.end()) {
    return search->second;
  }
  return -1.0f;
}

bool Animation::HasAnimation(const std::string& animation_name) const {
  const auto search = animation_length.find(animation_name);
  return search != animation_length.end();
}

bool Animation::IsEmpty() const {
  return animation_length.empty();
}

void Animation::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "bone_size" << YAML::Value << bone_size;

  if (!animation_length.empty()) {
    out << YAML::Key << "animation_length" << YAML::Value << YAML::BeginSeq;
    for (const auto& i : animation_length) {
      out << YAML::BeginMap;
      out << YAML::Key << "Name" << YAML::Value << i.first;
      out << YAML::Key << "Length" << YAML::Value << i.second;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
  if (root_bone) {
    out << YAML::Key << "root_bone" << YAML::Value << YAML::BeginMap;
    root_bone->Serialize(out);
    out << YAML::EndMap;
  }
}
void Animation::Deserialize(const YAML::Node& in) {
  bone_size = in["bone_size"].as<size_t>();
  auto in_animation_name_and_length = in["animation_length"];
  animation_length.clear();
  if (in_animation_name_and_length) {
    for (const auto& i : in_animation_name_and_length) {
      animation_length.insert({i["Name"].as<std::string>(), i["Length"].as<float>()});
    }
  }
  if (in["root_bone"]) {
    root_bone = std::make_shared<Bone>();
    root_bone->Deserialize(in["root_bone"]);
  }
}

void BoneKeyFrames::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "max_time_stamp" << YAML::Value << max_time_stamp;
  if (!positions.empty()) {
    out << YAML::Key << "positions" << YAML::Value
        << YAML::Binary((const unsigned char*)positions.data(), positions.size() * sizeof(BonePosition));
  }
  if (!rotations.empty()) {
    out << YAML::Key << "rotations" << YAML::Value
        << YAML::Binary((const unsigned char*)rotations.data(), rotations.size() * sizeof(BoneRotation));
  }
  if (!scales.empty()) {
    out << YAML::Key << "scales" << YAML::Value
        << YAML::Binary((const unsigned char*)scales.data(), scales.size() * sizeof(BoneScale));
  }
}

void BoneKeyFrames::Deserialize(const YAML::Node& in) {
  max_time_stamp = in["max_time_stamp"].as<float>();
  if (in["positions"]) {
    const auto in_positions = in["positions"].as<YAML::Binary>();
    positions.resize(in_positions.size() / sizeof(BonePosition));
    std::memcpy(positions.data(), in_positions.data(), positions.size() * sizeof(BonePosition));
  }
  if (in["rotations"]) {
    const auto in_rotations = in["rotations"].as<YAML::Binary>();
    rotations.resize(in_rotations.size() / sizeof(BoneRotation));
    std::memcpy(rotations.data(), in_rotations.data(), rotations.size() * sizeof(BoneRotation));
  }
  if (in["scales"]) {
    const auto in_scales = in["scales"].as<YAML::Binary>();
    scales.resize(in_scales.size() / sizeof(BoneScale));
    std::memcpy(scales.data(), in_scales.data(), scales.size() * sizeof(BoneScale));
  }
}

void Bone::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "name" << YAML::Value << name;
  out << YAML::Key << "offset_matrix" << YAML::Value << offset_matrix.value;
  out << YAML::Key << "index" << YAML::Value << index;
  if (!animations.empty()) {
    out << YAML::Key << "animations" << YAML::Value << YAML::BeginSeq;
    for (const auto& i : animations) {
      out << YAML::BeginMap;
      {
        out << YAML::Key << "Name" << YAML::Value << i.first;
        out << YAML::Key << "BoneKeyFrames" << YAML::Value << YAML::BeginMap;
        i.second.Serialize(out);
        out << YAML::EndMap;
      }
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
  if (!children.empty()) {
    out << YAML::Key << "children" << YAML::Value << YAML::BeginSeq;
    for (const auto& i : children) {
      out << YAML::BeginMap;
      i->Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}
void Bone::Deserialize(const YAML::Node& in) {
  name = in["name"].as<std::string>();
  offset_matrix.value = in["offset_matrix"].as<glm::mat4>();
  index = in["index"].as<size_t>();
  animations.clear();
  children.clear();
  if (auto in_animations = in["animations"]) {
    for (const auto& i : in_animations) {
      BoneKeyFrames key_frames;
      key_frames.Deserialize(i["BoneKeyFrames"]);
      animations.insert({i["Name"].as<std::string>(), std::move(key_frames)});
    }
  }

  if (auto in_children = in["children"]) {
    for (const auto& i : in_children) {
      children.push_back(std::make_shared<Bone>());
      children.back()->Deserialize(i);
    }
  }
}