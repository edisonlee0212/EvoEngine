#pragma once
#include "Entities.hpp"
#include "IAsset.hpp"
#include <PxPhysicsAPI.h>
using namespace physx;
namespace evo_engine
{
class PhysicsMaterial : public IAsset
{
    friend class PhysicsLayer;
    friend class Collider;
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

    void Serialize(YAML::Emitter &out) const override;
    void Deserialize(const YAML::Node &in) override;
};
} // namespace evo_engine