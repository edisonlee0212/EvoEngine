#pragma once

using namespace evo_engine;
namespace eco_sys_lab {
class ObjectRotator : public IPrivateComponent {
 public:
  float rotate_speed;
  glm::vec3 rotation = glm::vec3(0, 0, 0);

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  void FixedUpdate() override;

  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;
};
}  // namespace eco_sys_lab
