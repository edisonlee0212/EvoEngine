#pragma once
#include "Collider.hpp"
#include "Entities.hpp"
using namespace physx;
namespace evo_engine {
class RigidBody : public IPrivateComponent {
  glm::mat4 shape_transform_ =
      glm::translate(glm::vec3(0.0f)) * glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(1.0f));
  bool draw_bounds_ = false;

  std::vector<AssetRef> colliders_;

  bool static_ = false;
  friend class PhysicsSystem;
  friend class PhysicsLayer;
  PxRigidActor *rigid_actor_ = nullptr;

  float density_ = 10.0f;
  PxVec3 mass_center_ = PxVec3(0.0f);
  bool current_registered_ = false;
  PxVec3 linear_velocity_ = PxVec3(0.0f);
  PxVec3 angular_velocity_ = PxVec3(0.0f);
  friend class Joint;
  bool kinematic_ = false;
  PxReal linear_damping_ = 0.5;
  PxReal angular_damping_ = 0.5;

  PxU32 min_position_iterations_ = 4;
  PxU32 min_velocity_iterations_ = 1;
  bool gravity_ = true;

 public:
  [[nodiscard]] bool Registered() const;
  void AttachCollider(std::shared_ptr<Collider> &collider);
  void DetachCollider(size_t index);
  [[nodiscard]] bool IsKinematic() const;
  void SetSolverIterations(unsigned position = 4, unsigned velocity = 1);
  void SetEnableGravity(bool value);
  void SetLinearDamping(float value);
  void SetAngularDamping(float value);
  void SetKinematic(bool value);
  void SetDensityAndMassCenter(float value, const glm::vec3 &center = glm::vec3(0.0f));
  void SetLinearVelocity(const glm::vec3 &velocity);
  void SetAngularVelocity(const glm::vec3 &velocity);
  void SetStatic(bool value);
  bool IsStatic() const;
  void SetShapeTransform(const glm::mat4 &value);
  void OnDestroy() override;
  void RecreateBody();
  void OnCreate() override;
  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;

  void AddForce(const glm::vec3 &force) const;
  void AddTorque(const glm::vec3 &torque) const;

  void Serialize(YAML::Emitter &out) const override;
  void Deserialize(const YAML::Node &in) override;
  void CollectAssetRef(std::vector<AssetRef> &list) override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent> &target) override;
};
}  // namespace evo_engine
