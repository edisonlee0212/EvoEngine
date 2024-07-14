#include "PhysicsLayer.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "Jobs.hpp"
#include "Joint.hpp"
#include "ProjectManager.hpp"
#include "RigidBody.hpp"
#include "Scene.hpp"
#include "Times.hpp"
#include "TransformGraph.hpp"
using namespace evo_engine;
PrivateComponentRegistration<Joint> joint_registry("Joint");
PrivateComponentRegistration<RigidBody> rigidbody_registry("RigidBody");
AssetRegistration<Collider> collider_registry("Collider", {".uecollider"});
AssetRegistration<PhysicsMaterial> physics_material_registry("PhysicsMaterial", {".evephysicsmaterial"});
SystemRegistration<PhysicsSystem> physics_system_registry("PhysicsSystem");

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const PxVec2 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const PxVec3 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const PxVec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const PxMat44 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v[0] << v[1] << v[2] << v[3] << YAML::EndSeq;
  return out;
}

void PhysicsLayer::UploadTransform(const GlobalTransform &global_transform,
                                   const std::shared_ptr<RigidBody> &rigid_body) {
  GlobalTransform ltw;
  ltw.value = global_transform.value * rigid_body->shape_transform_;
  ltw.SetScale(glm::vec3(1.0f));

  if (rigid_body->current_registered_ && rigid_body->kinematic_) {
    static_cast<PxRigidDynamic *>(rigid_body->rigid_actor_)
        ->setKinematicTarget(PxTransform(*(PxMat44 *)(void *)&ltw.value));
  } else {
    rigid_body->rigid_actor_->setGlobalPose(PxTransform(*(PxMat44 *)(void *)&ltw.value));
  }
}

void PhysicsLayer::PreUpdate() {
  const bool playing = Application::IsPlaying();
  const auto active_scene = GetScene();
  UploadRigidBodyShapes(active_scene);
  UploadTransforms(active_scene, !playing);
  UploadJointLinks(active_scene);
}
void PhysicsLayer::OnCreate() {
  physics_foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, allocator_, error_callback_);
  if (!physics_foundation_)
    EVOENGINE_ERROR("PxCreateFoundation failed!");
#ifdef NDEBUG
  physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *physics_foundation_, PxTolerancesScale());
#else
  pvd_transport_ = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
  if (pvd_transport_ != NULL) {
    phys_vis_debugger_ = PxCreatePvd(*physics_foundation_);
    phys_vis_debugger_->connect(*pvd_transport_, PxPvdInstrumentationFlag::eALL);
  }
  physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *physics_foundation_, PxTolerancesScale(), true, phys_vis_debugger_);
  PxInitExtensions(*physics_, phys_vis_debugger_);
#endif  // NDEBUG
  dispatcher_ = PxDefaultCpuDispatcherCreate(8);

#pragma region Physics
  default_physics_material = Resources::CreateResource<PhysicsMaterial>("Default");
#pragma endregion
}

#define PX_RELEASE(x) \
  if (x) {            \
    x->release();     \
    x = nullptr;      \
  }
void PhysicsLayer::OnDestroy() {
  PX_RELEASE(dispatcher_);
  PX_RELEASE(physics_);
  PX_RELEASE(phys_vis_debugger_);
  PX_RELEASE(pvd_transport_);

  // PX_RELEASE(physics_foundation_);
}
void PhysicsLayer::UploadTransforms(const std::shared_ptr<Scene> &scene, const bool &update_all, const bool &freeze) {
  if (!scene)
    return;
  if (const std::vector<Entity> *entities = scene->UnsafeGetPrivateComponentOwnersList<RigidBody>();
      entities != nullptr) {
    for (auto entity : *entities) {
      const auto rigid_body = scene->GetOrSetPrivateComponent<RigidBody>(entity).lock();
      auto global_transform = scene->GetDataComponent<GlobalTransform>(entity);
      global_transform.value = global_transform.value * rigid_body->shape_transform_;
      global_transform.SetScale(glm::vec3(1.0f));
      if (rigid_body->current_registered_) {
        if (rigid_body->kinematic_) {
          if (freeze || update_all) {
            rigid_body->rigid_actor_->setGlobalPose(PxTransform(*(PxMat44 *)(void *)&global_transform.value));
          } else {
            static_cast<PxRigidDynamic *>(rigid_body->rigid_actor_)
                ->setKinematicTarget(PxTransform(*(PxMat44 *)(void *)&global_transform.value));
          }
        } else if (update_all) {
          rigid_body->rigid_actor_->setGlobalPose(PxTransform(*(PxMat44 *)(void *)&global_transform.value));
          if (freeze) {
            rigid_body->SetLinearVelocity(glm::vec3(0.0f));
            rigid_body->SetAngularVelocity(glm::vec3(0.0f));
          }
        }
      }
    }
  }
}

void PhysicsSystem::OnCreate() {
  m_scene = std::make_shared<PhysicsScene>();
  const auto physics_scene = m_scene->physics_scene_;
  if (!physics_scene)
    return;
  if (PxPvdSceneClient *pvd_client = physics_scene->getScenePvdClient()) {
    pvd_client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
    pvd_client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
    pvd_client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
  }
  Enable();
}

void PhysicsSystem::OnDestroy() {
}

void PhysicsSystem::FixedUpdate() {
  Simulate(Times::TimeStep());
}

void PhysicsScene::Simulate(float time) const {
  if (!physics_scene_)
    return;
  physics_scene_->simulate(time);
  physics_scene_->fetchResults(true);
}

void PhysicsSystem::Simulate(float time) const {
  const std::vector<Entity> *rigid_body_entities = GetScene()->UnsafeGetPrivateComponentOwnersList<RigidBody>();
  if (!rigid_body_entities)
    return;
  m_scene->Simulate(time);
  DownloadRigidBodyTransforms(rigid_body_entities);

  TransformGraph::GetInstance().physics_system_override_ = true;
}

void PhysicsSystem::OnEnable() {
}
void PhysicsSystem::DownloadRigidBodyTransforms() const {
  if (const std::vector<Entity> *rigid_body_entities = GetScene()->UnsafeGetPrivateComponentOwnersList<RigidBody>())
    DownloadRigidBodyTransforms(rigid_body_entities);
}

void PhysicsLayer::UploadRigidBodyShapes(const std::shared_ptr<Scene> &scene,
                                         const std::shared_ptr<PhysicsScene> &physics_scene,
                                         const std::vector<Entity> *rigid_body_entities) {
#pragma region Update shape
  for (auto entity : *rigid_body_entities) {
    const auto rigid_body = scene->GetOrSetPrivateComponent<RigidBody>(entity).lock();
    if (const bool should_register =
            scene->IsEntityValid(entity) && scene->IsEntityEnabled(entity) && rigid_body->IsEnabled();
        rigid_body->current_registered_ == false && should_register) {
      rigid_body->current_registered_ = true;
      physics_scene->physics_scene_->addActor(*rigid_body->rigid_actor_);
    } else if (rigid_body->current_registered_ == true && !should_register) {
      rigid_body->current_registered_ = false;
      physics_scene->physics_scene_->removeActor(*rigid_body->rigid_actor_);
    }
  }
#pragma endregion
}
void PhysicsLayer::UploadRigidBodyShapes(const std::shared_ptr<Scene> &scene) {
  if (!scene)
    return;
  const auto physics_system = scene->GetSystem<PhysicsSystem>();
  if (!physics_system)
    return;
  if (const std::vector<Entity> *rigid_body_entities = scene->UnsafeGetPrivateComponentOwnersList<RigidBody>()) {
    const auto physics_scene = physics_system->m_scene;
    UploadRigidBodyShapes(scene, physics_scene, rigid_body_entities);
  }
}
void PhysicsLayer::UploadJointLinks(const std::shared_ptr<Scene> &scene) {
  if (!scene)
    return;
  const auto physics_system = scene->GetSystem<PhysicsSystem>();
  if (!physics_system)
    return;
  if (const std::vector<Entity> *joint_entities = scene->UnsafeGetPrivateComponentOwnersList<Joint>()) {
    const auto physics_scene = physics_system->m_scene;
    UploadJointLinks(scene, physics_scene, joint_entities);
  }
}
void PhysicsLayer::UploadJointLinks(const std::shared_ptr<Scene> &scene,
                                    const std::shared_ptr<PhysicsScene> &physics_scene,
                                    const std::vector<Entity> *joint_entities) {
  const auto physics_layer = Application::GetLayer<PhysicsLayer>();
  if (!physics_layer)
    return;
#pragma region Update shape
  for (auto entity : *joint_entities) {
    auto joint = scene->GetOrSetPrivateComponent<Joint>(entity).lock();
    const auto rigid_body1 = joint->rigid_body1.Get<RigidBody>();
    const auto rigid_body2 = joint->rigid_body2.Get<RigidBody>();
    if (const bool should_register = scene->IsEntityValid(entity) && scene->IsEntityEnabled(entity) &&
                                     joint->IsEnabled() && (rigid_body1 && rigid_body2);
        !joint->linked_ && should_register) {
      auto owner_gt = scene->GetDataComponent<GlobalTransform>(rigid_body1->GetOwner());
      owner_gt.SetScale(glm::vec3(1.0f));
      auto linker_gt = scene->GetDataComponent<GlobalTransform>(rigid_body2->GetOwner());
      linker_gt.SetScale(glm::vec3(1.0f));
      Transform transform;
      transform.value = glm::inverse(owner_gt.value) * linker_gt.value;
      joint->local_position1_ = glm::vec3(0.0f);
      joint->local_rotation1_ = glm::vec3(0.0f);

      joint->local_position2_ = transform.GetPosition();
      joint->local_rotation2_ = transform.GetRotation();

      switch (joint->joint_type_) {
        case JointType::Fixed:
          joint->joint_ = PxFixedJointCreate(
              *physics_layer->physics_, rigid_body2->rigid_actor_,
              PxTransform(PxVec3(joint->local_position1_.x, joint->local_position1_.y, joint->local_position1_.z),
                          PxQuat(joint->local_rotation1_.x, joint->local_rotation1_.y, joint->local_rotation1_.z,
                                 joint->local_rotation1_.w)),
              rigid_body1->rigid_actor_,
              PxTransform(PxVec3(joint->local_position2_.x, joint->local_position2_.y, joint->local_position2_.z),
                          PxQuat(joint->local_rotation2_.x, joint->local_rotation2_.y, joint->local_rotation2_.z,
                                 joint->local_rotation2_.w)));
          break;
          /*
      case JointType::Distance:
          joint_ = PxDistanceJointCreate(
              *PhysicsManager::GetInstance().physics_,
              rigidBody2->rigid_actor_,
              PxTransform(
                  PxVec3(local_position1_.x, local_position1_.y, local_position1_.z),
                  PxQuat(local_rotation1_.x, local_rotation1_.y, local_rotation1_.z, local_rotation1_.w)),
              rigidBody1->rigid_actor_,
              PxTransform(
                  PxVec3(local_position2_.x, local_position2_.y, local_position2_.z),
                  PxQuat(local_rotation2_.x, local_rotation2_.y, local_rotation2_.z, local_rotation2_.w)));
          break;
      case JointType::Spherical: {
          joint_ = PxSphericalJointCreate(
              *PhysicsManager::GetInstance().physics_,
              rigidBody1->rigid_actor_,
              PxTransform(
                  PxVec3(local_position2_.x, local_position2_.y, local_position2_.z),
                  PxQuat(local_rotation2_.x, local_rotation2_.y, local_rotation2_.z, local_rotation2_.w)),
              rigidBody2->rigid_actor_,
              PxTransform(
                  PxVec3(local_position1_.x, local_position1_.y, local_position1_.z),
                  PxQuat(local_rotation1_.x, local_rotation1_.y, local_rotation1_.z, local_rotation1_.w)));
          // static_cast<PxSphericalJoint *>(joint_)->setLimitCone(PxJointLimitCone(PxPi / 2, PxPi / 6,
      0.01f));
          // static_cast<PxSphericalJoint
      *>(joint_)->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED,
          // true);
      }
      break;
      case JointType::Revolute:
          joint_ = PxRevoluteJointCreate(
              *PhysicsManager::GetInstance().physics_,
              rigidBody2->rigid_actor_,
              PxTransform(
                  PxVec3(local_position1_.x, local_position1_.y, local_position1_.z),
                  PxQuat(local_rotation1_.x, local_rotation1_.y, local_rotation1_.z, local_rotation1_.w)),
              rigidBody1->rigid_actor_,
              PxTransform(
                  PxVec3(local_position2_.x, local_position2_.y, local_position2_.z),
                  PxQuat(local_rotation2_.x, local_rotation2_.y, local_rotation2_.z, local_rotation2_.w)));
          break;
      case JointType::Prismatic:
          joint_ = PxPrismaticJointCreate(
              *PhysicsManager::GetInstance().physics_,
              rigidBody2->rigid_actor_,
              PxTransform(
                  PxVec3(local_position1_.x, local_position1_.y, local_position1_.z),
                  PxQuat(local_rotation1_.x, local_rotation1_.y, local_rotation1_.z, local_rotation1_.w)),
              rigidBody1->rigid_actor_,
              PxTransform(
                  PxVec3(local_position2_.x, local_position2_.y, local_position2_.z),
                  PxQuat(local_rotation2_.x, local_rotation2_.y, local_rotation2_.z, local_rotation2_.w)));
          break;
           */
        case JointType::D6:
          joint->joint_ = PxD6JointCreate(
              *physics_layer->physics_, rigid_body2->rigid_actor_,
              PxTransform(PxVec3(joint->local_position1_.x, joint->local_position1_.y, joint->local_position1_.z),
                          PxQuat(joint->local_rotation1_.x, joint->local_rotation1_.y, joint->local_rotation1_.z,
                                 joint->local_rotation1_.w)),
              rigid_body1->rigid_actor_,
              PxTransform(PxVec3(joint->local_position2_.x, joint->local_position2_.y, joint->local_position2_.z),
                          PxQuat(joint->local_rotation2_.x, joint->local_rotation2_.y, joint->local_rotation2_.z,
                                 joint->local_rotation2_.w)));
          for (int i = 0; i < 6; i++) {
            joint->SetMotion((MotionAxis)i, (MotionType)joint->motion_types_[i]);
          }
          for (int i = 0; i < 6; i++) {
            joint->SetDrive((DriveType)i, joint->drives_[i].stiffness, joint->drives_[i].damping,
                            joint->drives_[i].flags == PxD6JointDriveFlag::eACCELERATION);
          }
          break;
      }
      joint->linked_ = true;
    } else if (joint->linked_ && !should_register) {
      joint->Unlink();
    }
  }
#pragma endregion
}

void PhysicsSystem::DownloadRigidBodyTransforms(const std::vector<Entity> *rigid_body_entities) const {
  const auto scene = GetScene();
  auto &list = rigid_body_entities;
  Jobs::RunParallelFor(rigid_body_entities->size(), [&](unsigned index) {
    const auto rigid_body_entity = list->at(index);
    if (const auto rigid_body = scene->GetOrSetPrivateComponent<RigidBody>(rigid_body_entity).lock();
        rigid_body->current_registered_ && !rigid_body->kinematic_) {
      PxTransform transform = rigid_body->rigid_actor_->getGlobalPose();
      glm::vec3 position = *(glm::vec3 *)(void *)&transform.p;
      glm::quat rotation = *(glm::quat *)(void *)&transform.q;
      glm::vec3 scale = scene->GetDataComponent<GlobalTransform>(rigid_body_entity).GetScale();
      GlobalTransform global_transform;
      global_transform.SetValue(position, rotation, scale);
      scene->SetDataComponent(rigid_body_entity, global_transform);
      if (!rigid_body->static_) {
        PxRigidBody *rb = static_cast<PxRigidBody *>(rigid_body->rigid_actor_);
        rigid_body->linear_velocity_ = rb->getLinearVelocity();
        rigid_body->angular_velocity_ = rb->getAngularVelocity();
      }
    }
  });
}

PhysicsScene::PhysicsScene() {
  const auto physics_layer = Application::GetLayer<PhysicsLayer>();
  if (!physics_layer)
    return;
  auto physics = physics_layer->physics_;
  PxSceneDesc scene_desc(physics->getTolerancesScale());
  scene_desc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
  scene_desc.solverType = PxSolverType::eTGS;
  scene_desc.cpuDispatcher = physics_layer->dispatcher_;
  scene_desc.filterShader = PxDefaultSimulationFilterShader;
  physics_scene_ = physics->createScene(scene_desc);
}

PhysicsScene::~PhysicsScene() {
  PX_RELEASE(physics_scene_);
}