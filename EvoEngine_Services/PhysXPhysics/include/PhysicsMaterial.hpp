#pragma once
#include <PxPhysicsAPI.h>
#include "Entities.hpp"
#include "IAsset.hpp"

namespace evo_engine {
using namespace physx;
class PhysicsMaterial : public IAsset {
  friend class PhysicsLayer;
  friend class Collider;
  friend void SerializePhysicsMaterial(YAML::Emitter &out, const PhysicsMaterial &target);
  friend void DeserializePhysicsMaterial(const YAML::Node &in, PhysicsMaterial &target);
  PxMaterial *value_;
  float static_friction_ = 0.02f;
  float dynamic_friction_ = 0.02f;
  float restitution_ = 0.8f;

 public:
  void SetDynamicFriction(const float &value);
  void SetStaticFriction(const float &value);
  void SetRestitution(const float &value);
  void OnCreate() override;
  void OnGui();
  ~PhysicsMaterial();
};
}  // namespace evo_engine
