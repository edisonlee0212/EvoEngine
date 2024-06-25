#include "Animator.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "EditorLayer.hpp"
#include "Times.hpp"
using namespace evo_engine;

void Animator::Setup() {
  if (const auto animation = animation_.Get<Animation>()) {
    bone_size_ = animation->bone_size;
    if (animation->UnsafeGetRootBone() && bone_size_ != 0) {
      transform_chain_.resize(bone_size_);
      names_.resize(bone_size_);
      bones_.resize(bone_size_);
      BoneSetter(animation->UnsafeGetRootBone());
      offset_matrices_.resize(bone_size_);
      for (const auto& i : bones_)
        offset_matrices_[i->index] = i->offset_matrix.value;
      if (!animation->IsEmpty()) {
        current_activated_animation_ = animation->GetFirstAvailableAnimationName();
        current_animation_time_ = 0.0f;
      }
    }
  }
}
void Animator::OnDestroy() {
  transform_chain_.clear();
  offset_matrices_.clear();
  names_.clear();
  animation_.Clear();
  bones_.clear();
}
void Animator::Setup(const std::shared_ptr<Animation>& target_animation) {
  animation_.Set<Animation>(target_animation);
  Setup();
}

bool Animator::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  auto animation = animation_.Get<Animation>();
  const Animation* previous = animation.get();
  editor_layer->DragAndDropButton<Animation>(animation_, "Animation");
  if (previous != animation.get() && animation) {
    Setup(animation);
    animation = animation_.Get<Animation>();
  }
  if (animation) {
    if (bone_size_ != 0) {
      if (!animation->HasAnimation(current_activated_animation_)) {
        current_activated_animation_ = animation->GetFirstAvailableAnimationName();
        current_animation_time_ = 0.0f;
      }
      if (ImGui::BeginCombo("Animations##Animator",
                            current_activated_animation_
                                .c_str()))  // The second parameter is the label previewed before opening the combo.
      {
        for (auto& i : animation->UnsafeGetAnimationLengths()) {
          const bool selected =
              current_activated_animation_ ==
              i.first;  // You can store your selection however you want, outside or inside your objects
          if (ImGui::Selectable(i.first.c_str(), selected)) {
            current_activated_animation_ = i.first;
            current_animation_time_ = 0.0f;
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();  // You may set the initial focus when opening the combo (scrolling
                                           // + for keyboard navigation support)
          }
        }
        ImGui::EndCombo();
      }
      ImGui::SliderFloat("Animation time", &current_animation_time_, 0.0f,
                         animation->GetAnimationLength(current_activated_animation_));
    }
  }
  return changed;
}
float Animator::GetCurrentAnimationTimePoint() const {
  return current_animation_time_;
}

std::string Animator::GetCurrentAnimationName() {
  return current_activated_animation_;
}

void Animator::Animate(const std::string& animation_name, const float time) {
  const auto animation = animation_.Get<Animation>();
  if (!animation)
    return;
  const auto search = animation->UnsafeGetAnimationLengths().find(animation_name);
  if (search == animation->UnsafeGetAnimationLengths().end()) {
    EVOENGINE_ERROR("Animation not found!")
    return;
  }
  current_activated_animation_ = animation_name;
  current_animation_time_ = glm::mod(time, search->second);
}
void Animator::Animate(const float time) {
  const auto animation = animation_.Get<Animation>();
  if (!animation)
    return;
  current_animation_time_ = glm::mod(time, animation->GetAnimationLength(current_activated_animation_));
}
void Animator::Apply() {
  const auto animation = animation_.Get<Animation>();
  if (animation && !animation->IsEmpty()) {
    if (!animation->HasAnimation(current_activated_animation_)) {
      current_activated_animation_ = animation->GetFirstAvailableAnimationName();
      current_animation_time_ = 0.0f;
    }
    if (const auto owner = GetOwner(); owner.GetIndex() != 0) {
      animation->Animate(current_activated_animation_, current_animation_time_, glm::mat4(1.0f), transform_chain_);
      ApplyOffsetMatrices();
    }
  }
}

void Animator::BoneSetter(const std::shared_ptr<Bone>& bone_walker) {
  names_[bone_walker->index] = bone_walker->name;
  bones_[bone_walker->index] = bone_walker;
  for (auto& i : bone_walker->children) {
    BoneSetter(i);
  }
}

void Animator::Setup(const std::vector<std::string>& name, const std::vector<glm::mat4>& offset_matrices) {
  bones_.clear();
  bone_size_ = 0;
  transform_chain_.resize(offset_matrices.size());
  names_ = name;
  offset_matrices_ = offset_matrices;
}

void Animator::ApplyOffsetMatrices() {
  for (int i = 0; i < transform_chain_.size(); i++) {
    transform_chain_[i] *= offset_matrices_[i];
  }
}

glm::mat4 Animator::GetReverseTransform(const int bone_index) const {
  return transform_chain_[bone_index] * glm::inverse(bones_[bone_index]->offset_matrix.value);
}
void Animator::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}

void Animator::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(animation_);
}

void Animator::Serialize(YAML::Emitter& out) const {
  animation_.Save("animation_", out);
  out << YAML::Key << "current_activated_animation_" << YAML::Value << current_activated_animation_;
  out << YAML::Key << "current_animation_time_" << YAML::Value << current_animation_time_;

  if (!transform_chain_.empty()) {
    out << YAML::Key << "transform_chain_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(transform_chain_.data()),
                        transform_chain_.size() * sizeof(glm::mat4));
  }
  if (!offset_matrices_.empty()) {
    out << YAML::Key << "offset_matrices_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(offset_matrices_.data()),
                        offset_matrices_.size() * sizeof(glm::mat4));
  }
  if (!names_.empty()) {
    out << YAML::Key << "names_" << YAML::Value << YAML::BeginSeq;
    for (const auto& name : names_) {
      out << YAML::BeginMap;
      out << YAML::Key << "Name" << YAML::Value << name;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}

void Animator::Deserialize(const YAML::Node& in) {
  animation_.Load("animation_", in);
  if (animation_.Get<Animation>()) {
    current_activated_animation_ = in["current_activated_animation_"].as<std::string>();
    current_animation_time_ = in["current_animation_time_"].as<float>();
    Setup();
  }
  if (in["transform_chain_"]) {
    const auto chains = in["transform_chain_"].as<YAML::Binary>();
    transform_chain_.resize(chains.size() / sizeof(glm::mat4));
    std::memcpy(transform_chain_.data(), chains.data(), chains.size());
  }
  if (in["offset_matrices_"]) {
    const auto matrices = in["offset_matrices_"].as<YAML::Binary>();
    offset_matrices_.resize(matrices.size() / sizeof(glm::mat4));
    std::memcpy(offset_matrices_.data(), matrices.data(), matrices.size());
  }
  if (in["names_"]) {
    for (const auto& i : in["names_"]) {
      names_.push_back(i["Name"].as<std::string>());
    }
  }
}

std::shared_ptr<Animation> Animator::GetAnimation() {
  return animation_.Get<Animation>();
}