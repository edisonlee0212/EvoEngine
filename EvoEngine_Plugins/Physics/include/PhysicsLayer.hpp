#pragma once
#include <ILayer.hpp>
#include <Joint.hpp>
#include <PhysicsMaterial.hpp>

#include <Collider.hpp>
using namespace physx;
namespace YAML {
class Node;
class Emitter;
template <>
struct convert<PxVec2> {
  static Node encode(const PxVec2 &rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    return node;
  }

  static bool decode(const Node &node, PxVec2 &rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<float>();
    rhs.y = node[1].as<float>();
    return true;
  }
};
template <>
struct convert<PxVec3> {
  static Node encode(const PxVec3 &rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node &node, PxVec3 &rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<float>();
    rhs.y = node[1].as<float>();
    rhs.z = node[2].as<float>();
    return true;
  }
};
template <>
struct convert<PxVec4> {
  static Node encode(const PxVec4 &rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node &node, PxVec4 &rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<float>();
    rhs.y = node[1].as<float>();
    rhs.z = node[2].as<float>();
    rhs.w = node[3].as<float>();
    return true;
  }
};
template <>
struct convert<PxMat44> {
  static Node encode(const PxMat44 &rhs) {
    Node node;
    node.push_back(rhs[0]);
    node.push_back(rhs[1]);
    node.push_back(rhs[2]);
    node.push_back(rhs[3]);
    return node;
  }

  static bool decode(const Node &node, PxMat44 &rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs[0] = node[0].as<PxVec4>();
    rhs[1] = node[1].as<PxVec4>();
    rhs[2] = node[2].as<PxVec4>();
    rhs[3] = node[3].as<PxVec4>();
    return true;
  }
};
}  // namespace YAML

namespace evo_engine {
class RigidBody;
YAML::Emitter &operator<<(YAML::Emitter &out, const PxVec2 &v);
YAML::Emitter &operator<<(YAML::Emitter &out, const PxVec3 &v);
YAML::Emitter &operator<<(YAML::Emitter &out, const PxVec4 &v);
YAML::Emitter &operator<<(YAML::Emitter &out, const PxMat44 &v);

class PhysicsScene;
class PhysicsLayer : public ILayer {
  PxPvdTransport *pvd_transport_ = nullptr;
  PxDefaultAllocator allocator_;
  PxDefaultErrorCallback error_callback_;
  PxFoundation *physics_foundation_ = nullptr;
  PxPhysics *physics_ = nullptr;
  PxDefaultCpuDispatcher *dispatcher_ = nullptr;
  PxPvd *phys_vis_debugger_ = nullptr;
  friend class RigidBody;
  friend class Joint;
  friend class Articulation;
  friend class PhysicsScene;
  friend class PhysicsSystem;
  friend class PhysicsMaterial;
  friend class Collider;
  void UploadRigidBodyShapes(const std::shared_ptr<Scene> &scene, const std::shared_ptr<PhysicsScene> &physics_scene,
                             const std::vector<Entity> *rigid_body_entities);
  void UploadJointLinks(const std::shared_ptr<Scene>& scene, const std::shared_ptr<PhysicsScene>& physics_scene,
                        const std::vector<Entity>* joint_entities);

 public:
  std::shared_ptr<PhysicsMaterial> default_physics_material;
  void UploadTransforms(const std::shared_ptr<Scene> &scene, const bool &update_all, const bool &freeze = false);
  void UploadRigidBodyShapes(const std::shared_ptr<Scene> &scene);
  void UploadJointLinks(const std::shared_ptr<Scene> &scene);
  void UploadTransform(const GlobalTransform &global_transform, const std::shared_ptr<RigidBody> &rigid_body);
  void PreUpdate() override;
  void OnCreate() override;
  void OnDestroy() override;
};

class PhysicsScene {
  PxScene *physics_scene_ = nullptr;
  friend class PhysicsSystem;
  friend class PhysicsLayer;

 public:
  void Simulate(float time) const;
  PhysicsScene();
  ~PhysicsScene();
};
class PhysicsSystem : public ISystem {
  void DownloadRigidBodyTransforms(const std::vector<Entity> *rigid_body_entities) const;

 public:
  std::shared_ptr<PhysicsScene> m_scene;
  void DownloadRigidBodyTransforms() const;
  void OnEnable() override;
  void OnCreate() override;
  void OnDestroy() override;
  void FixedUpdate() override;
  void Simulate(float time) const;
};
}  // namespace evo_engine