#pragma once
#include "Animation.hpp"
#include "Scene.hpp"
namespace evo_engine
{
class Animator final : public IPrivateComponent
{
    std::vector<std::shared_ptr<Bone>> bones_;
    friend class SkinnedMeshRenderer;
    friend class RenderLayer;

    std::vector<glm::mat4> transform_chain_;
    std::vector<glm::mat4> offset_matrices_;
    std::vector<std::string> names_;
    AssetRef animation_;
    size_t bone_size_ = 0;
    void BoneSetter(const std::shared_ptr<Bone> &bone_walker);
    void Setup();
    std::string current_activated_animation_;
    float current_animation_time_;
    void Apply();
  public:
    /**
     * Only set offset matrices, so the animator can be used as ragDoll.
     * @param name Name of the bones
     * @param offset_matrices The collection of offset matrices.
     */
    void Setup(const std::vector<std::string> &name, const std::vector<glm::mat4> &offset_matrices);
    void ApplyOffsetMatrices();
    [[nodiscard]] glm::mat4 GetReverseTransform(int bone_index) const;

    [[nodiscard]] float GetCurrentAnimationTimePoint() const;
    [[nodiscard]] std::string GetCurrentAnimationName();
    void Animate(const std::string& animation_name, float time);
    void Animate(float time);

    void OnDestroy() override;
    void Setup(const std::shared_ptr<Animation> &target_animation);
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

    void PostCloneAction(const std::shared_ptr<IPrivateComponent> &target) override;
    [[nodiscard]] std::shared_ptr<Animation> GetAnimation();
    void Serialize(YAML::Emitter &out) const override;
    void Deserialize(const YAML::Node &in) override;
    void CollectAssetRef(std::vector<AssetRef> &list) override;
};
} // namespace evo_engine