#pragma once
#include "Plot2D.hpp"
#include "Skeleton.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @brief Represents a particle associated with a dynamic skeleton node.
 */
struct DynamicSkeletonNodeParticle {
  glm::vec3 x0 = glm::vec3{0.f};      ///< Initial position.
  glm::vec3 x = glm::vec3{0.f};       ///< Current position.
  glm::vec3 last_x = glm::vec3{0.f};  ///< Last recorded position.
  glm::vec3 v = glm::vec3{0.f};       ///< Linear velocity.
  glm::vec3 a = glm::vec3{0.f};       ///< Acceleration.
};

/**
 * @brief Holds dynamic data for a skeleton node.
 */
struct DynamicSkeletonNodeData {
  DynamicSkeletonNodeParticle particle0;          ///< First particle node.
  DynamicSkeletonNodeParticle particle1;          ///< Second particle node.
  glm::quat q0{};                                 ///< Initial orientation.
  glm::quat q{};                                  ///< Current orientation.
  glm::quat last_q{};                             ///< Last orientation.
  glm::vec3 angular_v = glm::vec3{0.f};           ///< Angular velocity.
  glm::vec3 torque = glm::vec3(0.f);              ///< Applied torque.
  glm::quat rest_darboux_vector{};                ///< Rest state representation.
  float inv_mass = 0;                             ///< Inverse mass.
  float mass = 0.f;                               ///< Mass of the node.
  glm::vec3 inertia_tensor = glm::vec3{0.f};      ///< Inertia tensor.
  float shear_stretch_alpha = 0;                  ///< Shear and stretch compliance.
  glm::vec3 inv_inertia_tensor = glm::vec3{0.f};  ///< Inverse inertia tensor.
  glm::mat3 inertia_w{};                          ///< World-space inertia.
  glm::mat3 inv_inertia_w{};                      ///< Inverse world-space inertia.

  float length;  ///< Length of the node.
  float radius;  ///< Radius of the node.

  float max_stretch_shear_modulus;  ///< Maximum stretch-shear resistance.

  float max_bending_modulus;   ///< Maximum bending flexibility.
  float max_twisting_modulus;  ///< Maximum twisting flexibility.

  float bending_alpha = 0.0f;  ///< Bending compliance factor.
  float torsion_alpha = 0.0f;  ///< Torsion compliance factor.
};

/**
 * @brief Struct for dynamic skeleton flow data.
 */
struct DynamicSkeletonFlowData {};

/**
 * @brief Struct for dynamic skeleton structure data.
 */
struct DynamicSkeletonSkeletonData {};

/**
 * @brief Typedef for a specialized Skeleton with dynamic behavior.
 */
typedef Skeleton<DynamicSkeletonSkeletonData, DynamicSkeletonFlowData, DynamicSkeletonNodeData> DtsSkeleton;

/**
 * @brief Represents a dynamic skeleton with physics simulation properties.
 */
class DynamicSkeleton {
 public:
  DtsSkeleton dts_skeleton;  ///< The underlying dynamic skeleton structure.

  /**
   * @brief Parameters for skeleton initialization.
   */
  struct InitializeParameters {
    bool static_root = true;                                      ///< Whether the root node remains static.
    SingleDistribution<float> wood_density = {600.0f, 1.0f};      ///< Density of the wood-like material.
    SingleDistribution<float> max_shear_modulus = {9.5f, 0.1f};   ///< Maximum shear modulus.
    SingleDistribution<float> max_youngs_modulus = {9.5f, 0.1f};  ///< Maximum Young's modulus.

    SingleDistribution<float> max_bending_modulus = {1.f, .1f};   ///< Maximum bending modulus.
    SingleDistribution<float> max_twisting_modulus = {1.f, .1f};  ///< Maximum twisting modulus.

    GlobalTransform root_transform{};  ///< Initial root transformation.

    /**
     * @brief Inspects the initialization parameters in the editor.
     * @param editor_layer The editor layer reference.
     * @return True if the asset's content is not modified.
     */
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  /**
   * @brief Parameters governing physics behavior.
   */
  struct PhysicsParameters {
    float time_step = 0.01f;  ///< Time step for simulation.
    int sub_step = 25;        ///< Number of sub-steps per simulation step.

    int constraint_iteration = 1;  ///< Number of constraint iterations.

    bool enable_disconnection = false;         ///< If true, parts can disconnect.
    bool enable_breaking = false;              ///< If true, parts can break.
    float velocity_damping = 0.001f;           ///< Linear velocity damping.
    float angular_velocity_damping = 0.0001f;  ///< Angular velocity damping.

    /**
     * @brief Inspects the physics parameters in the editor.
     * @param editor_layer The editor layer reference.
     * @return True if the asset's content is not modified.
     */
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  /**
   * @brief Parameters controlling visualization.
   */
  struct VisualizationParameters {
    /**
     * @brief Inspects the visualization parameters in the editor.
     * @param editor_layer The editor layer reference.
     * @return True if the asset's content is not modified.
     */
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  /**
   * @brief Initializes the dynamic skeleton using another skeleton as input.
   * @tparam SrcSkeletonData Data type for the source skeleton structure.
   * @tparam SrcFlowData Data type for the source skeleton flow.
   * @tparam SrcNodeData Data type for the source skeleton node.
   * @param initialize_parameters Parameters for initialization.
   * @param src_skeleton The source skeleton to clone from.
   */
  template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
  void Initialize(const InitializeParameters& initialize_parameters,
                  const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton);

  /**
   * @brief Executes physics simulation on the dynamic skeleton.
   * @param physics_parameters Configuration for the physics update.
   * @param pre_step_action Function to execute before stepping.
   * @param sub_step_action Function to execute at each sub-step.
   */
  void Physics(const PhysicsParameters& physics_parameters, const std::function<void()>& pre_step_action,
               const std::function<void()>& sub_step_action);

 private:
  uint32_t frame_index = 0;  ///< Frame counter.

  /**
   * @brief Initializes the skeleton.
   * @param initialize_parameters The initialization parameters.
   */
  void Initialize(const InitializeParameters& initialize_parameters);

  /**
   * @brief Performs pre-step calculations before physics iterations.
   * @param physics_parameters The physics parameters.
   */
  void PreStep(const PhysicsParameters& physics_parameters);

  /**
   * @brief Predicts new positions and orientations for simulation.
   * @param physics_parameters The physics parameters.
   */
  void Prediction(const PhysicsParameters& physics_parameters);

  /**
   * @brief Applies stiff rod constraints to the skeleton.
   * @param physics_parameters The physics parameters.
   */
  void ApplyStiffRodConstraint(const PhysicsParameters& physics_parameters);

  /**
   * @brief Updates velocity based on constraints and forces.
   * @param physics_parameters The physics parameters.
   */
  void VelocityUpdate(const PhysicsParameters& physics_parameters);
};

template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
void DynamicSkeleton::Initialize(const InitializeParameters& initialize_parameters,
                                 const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton) {
  dts_skeleton.Clone(src_skeleton);
  Initialize(initialize_parameters);
}
}  // namespace eco_sys_lab_plugin