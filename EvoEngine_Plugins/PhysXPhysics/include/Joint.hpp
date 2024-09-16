#pragma once
#include "RigidBody.hpp"

#include "PrivateComponentRef.hpp"
using namespace physx;
namespace evo_engine {
enum class JointType {
  Fixed = 0,
  /*
  Distance,
  Spherical,
  Revolute,
  Prismatic,
   */
  D6 = 1
};
enum class MotionAxis { X = 0, Y = 1, Z = 2, TwistX = 3, SwingY = 4, SwingZ = 5 };
enum class MotionType { Locked = 0, Limited = 1, Free = 2 };
enum class DriveType { X = 0, Y = 1, Z = 2, Swing = 3, Twist = 4, Slerp = 5 };
class Joint : public IPrivateComponent {
  JointType joint_type_ = JointType::Fixed;
  friend class PhysicsLayer;
  PxJoint *joint_;
  glm::vec3 local_position1_;
  glm::vec3 local_position2_;
  glm::quat local_rotation1_;
  glm::quat local_rotation2_;
  bool linked_ = false;
#pragma region Fixed
  void FixedGui();
#pragma endregion
  /*
#pragma region Distance
  float m_maxDistance = FLT_MIN;
  float m_minDistance = FLT_MAX;
  bool m_maxDistanceEnabled = false;
  bool m_minDistanceEnabled = false;
  float m_stiffness = 0;
  float m_damping = 0;
  void SetMax(float value, const bool &enabled);
  void SetMin(float value, const bool &enabled);
  void SetStiffness(float value);
  void SetDamping(float value);
  void DistanceGui();
#pragma endregion
#pragma region Spherical
  void SphericalGui();
#pragma endregion
#pragma region Revolute
  void RevoluteGui();
#pragma endregion
#pragma region Prismatic
  void PrismaticGui();
#pragma endregion
   */
#pragma region D6
  PxD6Motion::Enum motion_types_[6] = {PxD6Motion::Enum::eLOCKED};
  PxD6JointDrive drives_[6] = {PxD6JointDrive()};
  void D6Gui();
#pragma endregion
  bool TypeCheck(const JointType &type);

 public:
  PrivateComponentRef rigid_body1;
  PrivateComponentRef rigid_body2;
/*
#pragma region Fixed
#pragma endregion
#pragma region Distance
#pragma endregion
#pragma region Spherical
#pragma endregion
#pragma region Revolute
#pragma endregion
#pragma region Prismatic
#pragma endregion
 */
#pragma region D6
  void SetMotion(const MotionAxis &axis, const MotionType &type);
  void SetDistanceLimit(float extent, float stiffness = 0, float damping = 0);
  void SetDrive(const DriveType &type, float stiffness = 0, float damping = 0, const bool &is_acceleration = true);
#pragma endregion
  void SetType(const JointType &type);
  void Unlink();
  bool Linked();
  void OnCreate() override;

  void Link(const Entity &entity, bool reverse = false);
  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;
  void OnDestroy() override;

  void Relink(const std::unordered_map<Handle, Handle> &map, const std::shared_ptr<Scene> &scene) override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent> &target) override;

  void Serialize(YAML::Emitter &out) const override;
  void Deserialize(const YAML::Node &in) override;
};
}  // namespace evo_engine
