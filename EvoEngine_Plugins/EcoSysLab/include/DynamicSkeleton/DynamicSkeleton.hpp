#pragma once
#include "Plot2D.hpp"
#include "Skeleton.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
struct DynamicSkeletonNodeParticle {
  glm::vec3 x0 = glm::vec3{0.f};
  glm::vec3 x = glm::vec3{0.f};
  glm::vec3 last_x = glm::vec3{0.f};
  glm::vec3 v = glm::vec3{0.f};
  glm::vec3 a = glm::vec3{0.f};
};

struct DynamicSkeletonNodeData {
  DynamicSkeletonNodeParticle particle0;
  DynamicSkeletonNodeParticle particle1;
  glm::quat q0{};
  glm::quat q{};
  glm::quat last_q{};
  glm::vec3 angular_v = glm::vec3{0.f};
  glm::vec3 torque = glm::vec3(0.f);
  glm::quat rest_darboux_vector{};
  float inv_mass = 0;
  float mass = 0.f;
  glm::vec3 inertia_tensor = glm::vec3{0.f};
  float shear_stretch_alpha = 0;
  glm::vec3 inv_inertia_tensor = glm::vec3{0.f};
  glm::mat3 inertia_w{};
  glm::mat3 inv_inertia_w{};

  float length;
  float radius;

  float max_stretch_shear_modulus;

  float max_bending_modulus;
  float max_twisting_modulus;

  float bending_alpha = 0.0f;
  float torsion_alpha = 0.0f;
};

struct DynamicSkeletonFlowData {};
struct DynamicSkeletonSkeletonData {};
typedef Skeleton<DynamicSkeletonSkeletonData, DynamicSkeletonFlowData, DynamicSkeletonNodeData> DtsSkeleton;

class DynamicSkeleton {
 public:
  DtsSkeleton dts_skeleton;
  struct InitializeParameters {
    bool static_root = true;
    SingleDistribution<float> wood_density = {600.0f, 1.0f};
    SingleDistribution<float> max_shear_modulus = {9.5f, 0.1f};
    SingleDistribution<float> max_youngs_modulus = {9.5f, 0.1f};

    SingleDistribution<float> max_bending_modulus = {1.f, .1f};
    SingleDistribution<float> max_twisting_modulus = {1.f, .1f};

    GlobalTransform root_transform{};
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };
  struct PhysicsParameters {
    float time_step = 0.01f;
    int sub_step = 25;

    int constraint_iteration = 1;

    bool enable_disconnection = false;
    bool enable_breaking = false;
    float velocity_damping = 0.001f;
    float angular_velocity_damping = 0.0001f;

    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  struct VisualizationParameters {
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
  void Initialize(const InitializeParameters& initialize_parameters,
                  const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton);

  void Physics(const PhysicsParameters& physics_parameters, const std::function<void()>& pre_step_action,
               const std::function<void()>& sub_step_action);

 private:
  uint32_t frame_index = 0;
  void Initialize(const InitializeParameters& initialize_parameters);

  void PreStep(const PhysicsParameters& physics_parameters);
  void Prediction(const PhysicsParameters& physics_parameters);
  void ApplyStiffRodConstraint(const PhysicsParameters& physics_parameters);
  void VelocityUpdate(const PhysicsParameters& physics_parameters);
};

template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
void DynamicSkeleton::Initialize(const InitializeParameters& initialize_parameters,
                                 const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton) {
  dts_skeleton.Clone(src_skeleton);
  Initialize(initialize_parameters);
}
}  // namespace eco_sys_lab_plugin