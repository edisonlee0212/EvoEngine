#include "Joint.hpp"
#include "EditorLayer.hpp"
#include "RigidBody.hpp"
#include "Scene.hpp"
using namespace evo_engine;
#pragma region Fixed
void Joint::FixedGui() {
}
#pragma endregion
/*
#pragma region Distance
void Joint::DistanceGui()
{
        if (ImGui::DragFloat("Min", &m_minDistance, 0.1f, FLT_MIN, m_maxDistance))
        {
                SetMin(m_minDistance, m_minDistanceEnabled);
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Enabled", &m_minDistanceEnabled))
        {
                SetMin(m_minDistance, m_minDistanceEnabled);
        }
        if (ImGui::DragFloat("Max", &m_maxDistance, 0.1f, m_minDistance, FLT_MAX))
        {
                SetMax(m_maxDistance, m_maxDistanceEnabled);
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Enabled", &m_maxDistanceEnabled))
        {
                SetMax(m_maxDistance, m_maxDistanceEnabled);
        }

        if (ImGui::DragFloat("Stiffness", &m_stiffness))
        {
                SetStiffness(m_stiffness);
        }
        if (ImGui::DragFloat("Damping", &m_damping))
        {
                SetDamping(m_damping);
        }
}
void Joint::SetMax(float value, const bool &enabled)
{
        if (!joint_)
                return;
        if (!TypeCheck(JointType::Distance))
                return;
        if (m_maxDistance != value || m_maxDistanceEnabled != enabled)
        {
                m_maxDistance = value;
                m_maxDistanceEnabled = enabled;
                static_cast<PxDistanceJoint *>(joint_)->setDistanceJointFlag(
                        PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, m_maxDistanceEnabled);
                static_cast<PxDistanceJoint *>(joint_)->setMaxDistance(m_maxDistance);
        }
}
void Joint::SetMin(float value, const bool &enabled)
{
        if (!joint_)
                return;
        if (!TypeCheck(JointType::Distance))
                return;
        if (m_minDistance != value || m_maxDistanceEnabled != enabled)
        {
                m_minDistance = value;
                m_minDistanceEnabled = enabled;
                static_cast<PxDistanceJoint *>(joint_)->setDistanceJointFlag(
                        PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, m_minDistanceEnabled);
                static_cast<PxDistanceJoint *>(joint_)->setMinDistance(m_minDistance);
        }
}
void Joint::SetStiffness(float value)
{
        if (!joint_)
                return;
        if (!TypeCheck(JointType::Distance))
                return;
        if (m_stiffness != value)
        {
                m_stiffness = value;
                static_cast<PxDistanceJoint *>(joint_)->setStiffness(m_stiffness);
        }
}
void Joint::SetDamping(float value)
{
        if (!joint_)
                return;
        if (!TypeCheck(JointType::Distance))
                return;
        if (m_damping != value)
        {
                m_damping = value;
                static_cast<PxDistanceJoint *>(joint_)->setDamping(m_damping);
        }
}

#pragma endregion
#pragma region Spherical
void Joint::SphericalGui()
{
}
#pragma endregion
#pragma region Revolute
void Joint::RevoluteGui()
{
}
#pragma endregion
#pragma region Prismatic
void Joint::PrismaticGui()
{
}
#pragma endregion
 */
#pragma region D6

void Joint::D6Gui() {
  auto* joint = static_cast<PxD6Joint*>(joint_);
}
#pragma endregion
void Joint::Unlink() {
  if (!linked_)
    return;
  if (joint_) {
    joint_->release();
    joint_ = nullptr;
  }
  linked_ = false;
}
bool Joint::Linked() {
  return linked_;
}

void Joint::OnCreate() {
}

bool Joint::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  static int type = 0;
  type = (int)joint_type_;
  const char* joint_type_names[]{"Fixed", "D6"};
  if (ImGui::Combo("Joint Type", &type, joint_type_names, IM_ARRAYSIZE(joint_type_names))) {
    SetType((JointType)type);
    changed = true;
  }
  const auto stored_rigid_body1 = rigid_body1.Get<RigidBody>();
  const auto stored_rigid_body2 = rigid_body2.Get<RigidBody>();
  if (editor_layer->DragAndDropButton<RigidBody>(rigid_body1, "Link 1"))
    changed = true;
  if (editor_layer->DragAndDropButton<RigidBody>(rigid_body2, "Link 2"))
    changed = true;
  if (rigid_body1.Get<RigidBody>() != stored_rigid_body1 || rigid_body2.Get<RigidBody>() != stored_rigid_body2) {
    Unlink();
  }
  if (joint_) {
    switch (joint_type_) {
      case JointType::Fixed:
        FixedGui();
        break;
        /*
case JointType::Distance:
        DistanceGui();
        break;
case JointType::Spherical:
        SphericalGui();
        break;
case JointType::Revolute:
        RevoluteGui();
        break;
case JointType::Prismatic:
        PrismaticGui();
        break;
         */
      case JointType::D6:
        D6Gui();
        break;
    }
  }
  return changed;
}

void Joint::OnDestroy() {
  Unlink();
  rigid_body1.Clear();
  rigid_body2.Clear();
}

bool Joint::TypeCheck(const JointType& type) {
  if (joint_type_ != type) {
    EVOENGINE_ERROR("Wrong joint type!");
    return false;
  }
  return true;
}
void Joint::SetType(const JointType& type) {
  if (type != joint_type_) {
    joint_type_ = type;
    Unlink();
  }
}

void Joint::SetMotion(const MotionAxis& axis, const MotionType& type) {
  if (!joint_)
    return;
  if (!TypeCheck(JointType::D6))
    return;
  motion_types_[static_cast<int>(axis)] = static_cast<PxD6Motion::Enum>(type);
  static_cast<PxD6Joint*>(joint_)->setMotion(static_cast<PxD6Axis::Enum>(axis), static_cast<PxD6Motion::Enum>(type));
}
void Joint::SetDrive(const DriveType& type, float stiffness, float damping, const bool& is_acceleration) {
  if (!joint_)
    return;
  if (!TypeCheck(JointType::D6))
    return;
  drives_[static_cast<int>(type)].stiffness = stiffness;
  drives_[static_cast<int>(type)].damping = damping;
  drives_[static_cast<int>(type)].flags =
      static_cast<PxD6JointDriveFlag::Enum>(is_acceleration ? PxU32(PxD6JointDriveFlag::eACCELERATION) : 0);
  static_cast<PxD6Joint*>(joint_)->setDrive(static_cast<PxD6Drive::Enum>(type), drives_[static_cast<int>(type)]);
}
void Joint::SetDistanceLimit(float extent, float stiffness, float damping) {
  if (!joint_)
    return;
  if (!TypeCheck(JointType::D6))
    return;
  const auto spring = PxSpring(stiffness, damping);
  static_cast<PxD6Joint*>(joint_)->setDistanceLimit(PxJointLinearLimit(extent, spring));
}
void Joint::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
  joint_ = nullptr;
  linked_ = false;
}
void Joint::Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
  rigid_body1.Relink(map, scene);
  rigid_body2.Relink(map, scene);
}

void Joint::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_jointType" << YAML::Value << (unsigned)joint_type_;
  out << YAML::Key << "local_position1_" << YAML::Value << local_position1_;
  out << YAML::Key << "local_position2_" << YAML::Value << local_position2_;
  out << YAML::Key << "local_rotation1_" << YAML::Value << local_rotation1_;
  out << YAML::Key << "local_rotation2_" << YAML::Value << local_rotation2_;

  rigid_body1.Save("rigid_body1", out);
  rigid_body2.Save("rigid_body2", out);

  switch (joint_type_) {
    case JointType::Fixed:

      break;
      /*
case JointType::Distance:
      DistanceGui();
      break;
case JointType::Spherical:
      SphericalGui();
      break;
case JointType::Revolute:
      RevoluteGui();
      break;
case JointType::Prismatic:
      PrismaticGui();
      break;
       */
    case JointType::D6:
      out << YAML::Key << "motion_types_" << YAML::Value << YAML::BeginSeq;
      for (int i = 0; i < 6; i++) {
        out << YAML::BeginMap;
        out << YAML::Key << "Index" << YAML::Value << i;
        out << YAML::Key << "MotionType" << YAML::Value << (unsigned)motion_types_[i];
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
      out << YAML::Key << "drives_" << YAML::Value << YAML::BeginSeq;
      for (int i = 0; i < 6; i++) {
        out << YAML::BeginMap;
        out << YAML::Key << "Index" << YAML::Value << i;
        out << YAML::Key << "Stiffness" << YAML::Value << (float)drives_[i].stiffness;
        out << YAML::Key << "Damping" << YAML::Value << (float)drives_[i].damping;
        out << YAML::Key << "Flags" << YAML::Value << (unsigned)drives_[i].flags;
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
      break;
  }
}
void Joint::Deserialize(const YAML::Node& in) {
  joint_type_ = (JointType)in["m_jointType"].as<unsigned>();
  local_position1_ = in["local_position1_"].as<glm::vec3>();
  local_position2_ = in["local_position2_"].as<glm::vec3>();
  local_rotation1_ = in["local_rotation1_"].as<glm::quat>();
  local_rotation2_ = in["local_rotation2_"].as<glm::quat>();

  rigid_body1.Load("rigid_body1", in, GetScene());
  rigid_body2.Load("rigid_body2", in, GetScene());

  switch (joint_type_) {
    case JointType::Fixed:
      break;
      /*
case JointType::Distance:
      DistanceGui();
      break;
case JointType::Spherical:
      SphericalGui();
      break;
case JointType::Revolute:
      RevoluteGui();
      break;
case JointType::Prismatic:
      PrismaticGui();
      break;
       */
    case JointType::D6:
      auto in_motion_types = in["motion_types_"];
      for (const auto& in_motion_type : in_motion_types) {
        int index = in_motion_type["Index"].as<int>();
        motion_types_[index] = (PxD6Motion::Enum)in_motion_type["MotionType"].as<unsigned>();
      }
      auto in_drives = in["drives_"];
      for (const auto& in_drive : in_drives) {
        int index = in_drive["Index"].as<int>();
        drives_[index].stiffness = in_drive["Stiffness"].as<float>();
        drives_[index].damping = in_drive["Damping"].as<float>();
        drives_[index].flags = (PxD6JointDriveFlag::Enum)in_drive["Flags"].as<unsigned>();
      }
      break;
  }

  linked_ = false;
  joint_ = nullptr;
}
void Joint::Link(const Entity& entity, bool reverse) {
  auto scene = GetScene();
  const auto owner = GetOwner();
  if (scene->HasPrivateComponent<RigidBody>(owner) && scene->HasPrivateComponent<RigidBody>(entity)) {
    if (!reverse) {
      rigid_body1.Set(scene->GetOrSetPrivateComponent<RigidBody>(owner).lock());
      rigid_body2.Set(scene->GetOrSetPrivateComponent<RigidBody>(entity).lock());
    } else {
      rigid_body2.Set(scene->GetOrSetPrivateComponent<RigidBody>(owner).lock());
      rigid_body1.Set(scene->GetOrSetPrivateComponent<RigidBody>(entity).lock());
    }
    Unlink();
  }
}
