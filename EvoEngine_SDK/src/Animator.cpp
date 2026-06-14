#include "Animator.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
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
  ClearAnimation();
}

void Animator::ClearAnimation() {
  transform_chain_.clear();
  offset_matrices_.clear();
  names_.clear();
  animation_.Clear();
  bones_.clear();
  bone_size_ = 0;
  current_activated_animation_.clear();
  current_animation_time_ = 0.0f;
}

void Animator::Setup(const std::shared_ptr<Animation>& target_animation) {
  ClearAnimation();
  animation_.Set<Animation>(target_animation);
  Setup();
}

float Animator::GetCurrentAnimationTimePoint() const {
  return current_animation_time_;
}

std::string Animator::GetCurrentAnimationName() const {
  return current_activated_animation_;
}

size_t Animator::GetBoneSize() const {
  return bone_size_;
}

const std::vector<glm::mat4>& Animator::PeekTransformChain() const {
  return transform_chain_;
}

std::vector<glm::mat4>& Animator::RefTransformChain() {
  return transform_chain_;
}

const std::vector<glm::mat4>& Animator::PeekOffsetMatrices() const {
  return offset_matrices_;
}

std::vector<glm::mat4>& Animator::RefOffsetMatrices() {
  return offset_matrices_;
}

const std::vector<std::string>& Animator::PeekBoneNames() const {
  return names_;
}

std::vector<std::string>& Animator::RefBoneNames() {
  return names_;
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

std::shared_ptr<Animation> Animator::GetAnimation() {
  return animation_.Get<Animation>();
}

const AssetRef& Animator::PeekAnimationRef() const {
  return animation_;
}

AssetRef& Animator::RefAnimationRef() {
  return animation_;
}

void Animator::RestorePlaybackState(const std::string& animation_name, const float time) {
  current_activated_animation_ = animation_name;
  current_animation_time_ = time;
}

void Animator::RebuildAnimationState() {
  Setup();
}
