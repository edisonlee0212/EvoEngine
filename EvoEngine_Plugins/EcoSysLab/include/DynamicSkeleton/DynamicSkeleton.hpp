#pragma once
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

  float inv_mass = 0;
  glm::vec3 inertia_tensor = glm::vec3{0.f};
  float shearing_alpha = 0;
  glm::vec3 inv_inertia_tensor = glm::vec3{0.f};
  float stretching_alpha = 0;
  glm::mat3 inertia_w{};
  glm::mat3 inv_inertia_w{};
};

struct DynamicSkeletonFlowData {};
struct DynamicSkeletonSkeletonData {};
typedef Skeleton<DynamicSkeletonSkeletonData, DynamicSkeletonFlowData, DynamicSkeletonNodeData> DtsSkeleton;

class DynamicSkeleton {
 public:
  DtsSkeleton dts_skeleton;
  struct InitializeParameters {
    bool static_root = true;
    float wood_density = 600.f;
    GlobalTransform root_transform{};
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };
  float time_step = 0.01f;
  int sub_step = 10;
  struct PhysicsParameters {
    int constraint_iteration = 5;
    bool enable_disconnection = false;
    bool enable_breaking = false;
    float velocity_damping = 0.005f;
    float angular_velocity_damping = 0.0005f;

    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  struct VisualizationParameters {};

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