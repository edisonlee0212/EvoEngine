#include "DynamicSkeleton.hpp"

#include "DsColliders.hpp"
#include "DynamicStrands.hpp"

using namespace eco_sys_lab_plugin;

bool DynamicSkeleton::InitializeParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::TreeNode("Material Properties")) {
    if (wood_density.OnInspect("Wood Density"))
      changed = true;
    if (max_youngs_modulus.OnInspect("Wood Young's modulus"))
      changed = true;
    if (max_shear_modulus.OnInspect("Wood Shear modulus"))
      changed = true;
    if (max_bending_modulus.OnInspect("Wood Bending modulus"))
      changed = true;
    if (max_twisting_modulus.OnInspect("Wood Torsion modulus"))
      changed = true;
    ImGui::TreePop();
  }

  return changed;
}

bool DynamicSkeleton::PhysicsParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::DragFloat("Time step", &time_step, 0.001f, 0.001f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Sub step", &sub_step, 1, 1, 100)) {
    changed = true;
  }
  if (ImGui::Checkbox("Breaking", &enable_breaking)) {
    changed = true;
  }
  if (ImGui::Checkbox("Disconnection", &enable_disconnection)) {
    changed = true;
  }
  if (ImGui::DragInt("Constraint Iteration", &constraint_iteration, 1, 1, 500))
    changed = true;
  if (ImGui::DragFloat("Velocity damping", &velocity_damping, 0.01f, 0.01f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Angular velocity damping", &angular_velocity_damping, 0.00001f, 0.0f, 1.0f, "%.5f"))
    changed = true;

  return changed;
}

bool DynamicSkeleton::VisualizationParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  return changed;
}

void DynamicSkeleton::Initialize(const InitializeParameters& initialize_parameters) {
  dts_skeleton.SortLists();
  dts_skeleton.CalculateRegulatedGlobalRotation();
  frame_index = 0;
  const auto& sorted_node_list = dts_skeleton.PeekSortedNodeList();
  if (sorted_node_list.empty())
    return;
  Jobs::RunParallelFor(sorted_node_list.size(), [&](const auto i) {
    auto& node = dts_skeleton.RefNode(sorted_node_list[i]);
    const auto& node_info = node.info;
    auto& node_data = node.data;
    node_data.particle0.x0 = node_data.particle0.x = node_data.particle0.last_x =
        initialize_parameters.root_transform.TransformPoint(node_info.global_position);

    node_data.particle0.v = node_data.particle1.v = node_data.particle0.a = node_data.particle1.a = glm::vec3(0.f);
    node_data.q0 = node.data.q = node.data.last_q =
        initialize_parameters.root_transform.GetRotation() * node_info.regulated_global_rotation;
    node_data.angular_v = node_data.torque = glm::vec3(0.f);

    node_data.length = glm::max(1e-6f, node_info.length);
    node_data.particle1.x0 = node_data.particle1.x = node_data.particle1.last_x =
        initialize_parameters.root_transform.TransformPoint(
            node_info.global_position +
            glm::normalize(node_info.regulated_global_rotation * glm::vec3(0, 0, -1)) * node_data.length);

    node_data.radius = glm::max(1e-6f, node_info.thickness * .5f);
    node_data.mass = glm::max(1e-6f, node_data.radius * node_data.radius * glm::pi<float>() *
                                         initialize_parameters.wood_density.GetValue() * node_data.length);
    node_data.inv_mass = 1.f / node_data.mass;
    node_data.inertia_tensor =
        DynamicStrands::ComputeInertiaTensorRod(node_data.mass, node_data.radius, node_data.length);
    node_data.inv_inertia_tensor = 1.f / node_data.inertia_tensor;

    const float area = glm::pi<float>() * node_data.radius * node_data.radius;
    node_data.max_stretch_shear_modulus = glm::max(1e-9f, initialize_parameters.max_youngs_modulus.GetValue()) * 1e9f;

    node_data.shear_stretch_alpha = 1.f / (node_data.max_stretch_shear_modulus * area / node_data.length);

    node_data.max_bending_modulus = glm::max(1e-9f, initialize_parameters.max_bending_modulus.GetValue()) * 1e9f;
    node_data.max_twisting_modulus = glm::max(1e-9f, initialize_parameters.max_twisting_modulus.GetValue()) * 1e9f;
    const float average_segment_radius = node_data.radius;
    const float average_segment_length = node_data.length;

    const auto second_moment_of_area = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.25f;
    const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.5f;
    node_data.bending_alpha =
        1.f / (node_data.max_bending_modulus * second_moment_of_area / glm::pow(average_segment_length, 3.f));
    node_data.torsion_alpha = 1.f / (node_data.max_twisting_modulus * polar_moment_of_inertia / average_segment_length);
  });
  Jobs::RunParallelFor(sorted_node_list.size(), [&](const auto i) {
    auto& node = dts_skeleton.RefNode(sorted_node_list[i]);
    const auto& node_info = node.info;
    auto& node_data = node.data;
    const auto parent_handle = node.GetParentHandle();
    if (parent_handle != -1) {
      node_data.rest_darboux_vector = glm::conjugate(dts_skeleton.PeekNode(parent_handle).data.q0) * node_data.q0;
    }
  });
}

void DynamicSkeleton::Physics(const PhysicsParameters& physics_parameters, const std::function<void()>& pre_step_action,
                              const std::function<void()>& sub_step_action) {
  pre_step_action();
  for (int sub_step_index = 0; sub_step_index < physics_parameters.sub_step; sub_step_index++) {
    PreStep(physics_parameters);
    sub_step_action();
    Prediction(physics_parameters);
    for (int iteration_i = 0; iteration_i < physics_parameters.constraint_iteration; iteration_i++) {
      ApplyStiffRodConstraint(physics_parameters);
    }
    const auto scene = Application::GetActiveScene();
    const auto* box_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsBoxCollider>();
    const auto* sphere_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsSphereCollider>();
    const auto* cylinder_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsCylinderCollider>();
    if (box_collider_entities && !box_collider_entities->empty()) {
      for (const auto& i : *box_collider_entities) {
        const auto box_collider = scene->GetOrSetPrivateComponent<DsBoxCollider>(i).lock();
      }
    }
    if (sphere_collider_entities && !sphere_collider_entities->empty()) {
      for (const auto& i : *sphere_collider_entities) {
        const auto sphere_collider = scene->GetOrSetPrivateComponent<DsSphereCollider>(i).lock();
      }
    }
    if (cylinder_collider_entities && !cylinder_collider_entities->empty()) {
      for (const auto& i : *cylinder_collider_entities) {
        const auto cylinder_collider = scene->GetOrSetPrivateComponent<DsCylinderCollider>(i).lock();
      }
    }
    VelocityUpdate(physics_parameters);
  }
  frame_index++;
}

void DynamicSkeleton::PreStep(const PhysicsParameters& physics_parameters) {
  const auto& sorted_node_list = dts_skeleton.PeekSortedNodeList();
  Jobs::RunParallelFor(sorted_node_list.size(), [&](const auto i) {
    auto& node = dts_skeleton.RefNode(sorted_node_list[i]);
    const auto& node_info = node.info;
    auto& node_data = node.data;
    node_data.torque = glm::vec3(0.f);
    node_data.particle0.a = glm::vec3(0.f);
    node_data.particle1.a = glm::vec3(0.f);
  });
}

void DynamicSkeleton::Prediction(const PhysicsParameters& physics_parameters) {
  const auto sub_time_step = physics_parameters.time_step / physics_parameters.sub_step;
  auto predict_node = [&](DynamicSkeletonNodeData& node_data) {
    auto update_inertia_w = [](DynamicSkeletonNodeData& target_node_data) {
      // Update w
      const glm::mat3 rotation_matrix = mat3_cast(target_node_data.q);
      auto inertia_tensor_diag =
          glm::mat3(target_node_data.inertia_tensor.x, 0.0, 0.0, 0.0, target_node_data.inertia_tensor.y, 0.0, 0.0, 0.0,
                    target_node_data.inertia_tensor.z);
      target_node_data.inertia_w = rotation_matrix * inertia_tensor_diag * glm::transpose(rotation_matrix);
      auto inverse_inertia_tensor_diag =
          glm::mat3(target_node_data.inv_inertia_tensor.x, 0.0, 0.0, 0.0, target_node_data.inv_inertia_tensor.y, 0.0,
                    0.0, 0.0, target_node_data.inv_inertia_tensor.z);
      target_node_data.inv_inertia_w = rotation_matrix * inverse_inertia_tensor_diag * glm::transpose(rotation_matrix);
    };

    update_inertia_w(node_data);
    // Calculate angular velocity and apply torque.
    if (node_data.inv_mass != 0.0f) {
      node_data.angular_v *= 1.0f - physics_parameters.angular_velocity_damping * glm::length(node_data.angular_v);
      node_data.angular_v +=
          sub_time_step * node_data.inv_inertia_w *
          (node_data.torque - glm::cross(node_data.angular_v, node_data.inertia_w * node_data.angular_v));

      node_data.particle0.v *= 1.0f - physics_parameters.velocity_damping * glm::length(node_data.particle0.v);
      node_data.particle0.v += node_data.particle0.a * sub_time_step;

      node_data.particle1.v *= 1.0f - physics_parameters.velocity_damping * glm::length(node_data.particle1.v);
      node_data.particle1.v += node_data.particle1.a * sub_time_step;
    }
    // Shift rotation values
    node_data.last_q = node_data.q;
    // Apply angular velocity
    const glm::quat angular_velocity_q = {0.0f, node_data.angular_v.x, node_data.angular_v.y, node_data.angular_v.z};

    node_data.q += 0.5f * sub_time_step * (angular_velocity_q * node_data.q);
    node_data.q = normalize(node_data.q);

    // Update w
    update_inertia_w(node_data);
    /*
    glm::vec2 shear_stretch_strain =
        shear_stretch_strain(node_data.particle0.x, node_data.particle1.x, node_data.q, node_data.rest_length);

    node_data.shear_stretch_strain = shear_stretch_strain;
    // Update moisture content.

    // Update shear_stretch_strain_limit
    node_data.shear_stretch_strain_limit = node_data.max_shear_stretch_strain;
    */
    // Shift position values
    node_data.particle0.last_x = node_data.particle0.x;
    // Apply velocity
    node_data.particle0.x += sub_time_step * node_data.particle0.v;

    // Shift position values
    node_data.particle1.last_x = node_data.particle1.x;
    // Apply velocity
    node_data.particle1.x += sub_time_step * node_data.particle1.v;
  };

  const auto& sorted_node_list = dts_skeleton.PeekSortedNodeList();
  Jobs::RunParallelFor(sorted_node_list.size(), [&](const auto i) {
    auto& node = dts_skeleton.RefNode(sorted_node_list[i]);
    auto& node_data = node.data;
    predict_node(node_data);
  });
}

void DynamicSkeleton::ApplyStiffRodConstraint(const PhysicsParameters& physics_parameters) {
  const auto project_shear_stretch_constraint =
      [](const float& inv_time_step, const glm::vec3& p0, const glm::vec3& p1, const glm::quat& q,
         const float& inv_mass_p0, const float& inv_mass_p1, const float& inv_mass_q, const glm::vec3& alpha,
         const float& rest_length, glm::vec3& x0_correction, glm::vec3& x1_correction, glm::quat& q_correction) {
        glm::vec3 d3;
        d3[0] = -2.0f * (q.x * q.z + q.w * q.y);
        d3[1] = -2.0f * (q.y * q.z - q.w * q.x);
        d3[2] = -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z;

        glm::vec3 lambda = p1 - p0 - d3 * rest_length;
        const glm::mat3 r = glm::mat3_cast(q);
        lambda = glm::transpose(r) * lambda;
        const float factor = glm::max(1e-9f, inv_mass_p0 + inv_mass_p1 + 4.0f * inv_mass_q * rest_length * rest_length);

        const float t2 = inv_time_step * inv_time_step;
        const auto alpha_factor = glm::vec3(t2 * alpha.x, t2 * alpha.y, t2 * alpha.z);
        lambda.x /= glm::max(1e-9f, factor + alpha_factor.x);
        lambda.y /= glm::max(1e-9f, factor + alpha_factor.y);
        lambda.z /= glm::max(1e-9f, factor + alpha_factor.z);

        lambda = r * lambda;

        x0_correction = inv_mass_p0 * lambda;
        x1_correction = -inv_mass_p1 * lambda;

        const auto q_e_3_bar = glm::quat(-q.z, q.y, -q.x, q.w);
        q_correction = glm::quat(0.f, lambda.x, lambda.y, lambda.z) * q_e_3_bar;
        q_correction *= 2.f * inv_mass_q * rest_length;
      };
  const auto project_bend_twist_constraint = [](const float& inv_time_step, const glm::quat& q0,
                                                const float& inv_mass_q0, const glm::quat& q1, const float& inv_mass_q1,
                                                const glm::vec3& alpha, const glm::quat& rest_darboux_vector,
                                                glm::quat& q0_correction, glm::quat& q1_correction) {
    const auto squared_norm = [](const glm::quat& q) {
      return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    };
    glm::quat lambda = glm::conjugate(q0) * q1;
    const glm::quat lambda_plus = lambda + rest_darboux_vector;
    lambda -= rest_darboux_vector;
    if (squared_norm(lambda) > squared_norm(lambda_plus))
      lambda = lambda_plus;
    const float factor = glm::max(1e-9f, inv_mass_q0 + inv_mass_q1);

    const float t2 = inv_time_step * inv_time_step;
    const auto alpha_factor = glm::vec3(t2 * alpha.x, t2 * alpha.y, t2 * alpha.z);
    lambda.x /= glm::max(1e-9f, factor + alpha_factor.x);
    lambda.y /= glm::max(1e-9f, factor + alpha_factor.y);
    lambda.z /= glm::max(1e-9f, factor + alpha_factor.z);

    lambda.w = 0.0f;

    q0_correction = q1 * lambda * inv_mass_q0;
    q1_correction = q0 * lambda * inv_mass_q1 * -1.0f;
  };

  const auto sub_time_step = physics_parameters.time_step / physics_parameters.sub_step;
  const auto inv_time_step = 1.f / sub_time_step;

  const auto& sorted_node_list = dts_skeleton.PeekSortedNodeList();

  for (const auto node_handle : sorted_node_list) {
    auto& node = dts_skeleton.RefNode(node_handle);
    auto& node_data = node.data;
    const auto& child_handles = node.PeekChildHandles();
    // 1. Bend & twist
    if (!child_handles.empty()) {
      auto parent_q_sum = glm::quat(0, 0, 0, 0);
      float q_weight = 0.f;
      for (const auto& child_handle : child_handles) {
        auto& child_node_data = dts_skeleton.RefNode(child_handle).data;
        glm::quat q_correction, child_q_correction;
        const auto alpha = glm::vec3(node_data.bending_alpha, node_data.bending_alpha, node_data.torsion_alpha);
        project_bend_twist_constraint(inv_time_step, node_data.q, node_data.inv_mass, child_node_data.q,
                                      child_node_data.inv_mass, alpha, child_node_data.rest_darboux_vector,
                                      q_correction, child_q_correction);
        q_weight += child_node_data.mass;
        parent_q_sum += q_correction * child_node_data.mass;
        child_node_data.q = glm::normalize(child_node_data.q + child_q_correction);
      }
      parent_q_sum /= q_weight;
      node_data.q = glm::normalize(node_data.q + parent_q_sum);
    }

    // 2. Stretch & shear
    glm::vec3 x0_correction, x1_correction;
    glm::quat q_correction;
    const auto alpha = glm::vec3(node_data.shear_stretch_alpha);
    project_shear_stretch_constraint(inv_time_step, node_data.particle0.x, node_data.particle1.x, node_data.q,
                                     node_data.inv_mass, node_data.inv_mass, node_data.inv_mass, alpha,
                                     node.data.length, x0_correction, x1_correction, q_correction);

    node_data.particle0.x += x0_correction;
    node_data.particle1.x += x1_correction;
    node_data.q = glm::normalize(node_data.q + q_correction);

    // 3. Connection
    if (!child_handles.empty()) {
      if (node_data.inv_mass == 0.f) {
        for (const auto& child_handle : child_handles) {
          auto& child_node_data = dts_skeleton.RefNode(child_handle).data;
          child_node_data.particle0.x = node_data.particle1.x;
        }
      } else {
        glm::vec3 position_sum = node_data.particle1.x * node_data.mass;
        float weight_sum = node_data.mass;
        for (const auto& child_handle : child_handles) {
          const auto& child_node_data = dts_skeleton.PeekNode(child_handle).data;
          position_sum += child_node_data.particle0.x * child_node_data.mass;
          weight_sum += child_node_data.mass;
        }
        position_sum /= weight_sum;
        for (const auto& child_handle : child_handles) {
          auto& child_node_data = dts_skeleton.RefNode(child_handle).data;
          child_node_data.particle0.x = position_sum;
        }
        node_data.particle1.x = position_sum;
      }
    }
  }
}

void DynamicSkeleton::VelocityUpdate(const PhysicsParameters& physics_parameters) {
  const auto& sorted_node_list = dts_skeleton.PeekSortedNodeList();
  const auto sub_time_step = physics_parameters.time_step / physics_parameters.sub_step;
  const auto inv_time_step = 1.f / sub_time_step;
  Jobs::RunParallelFor(sorted_node_list.size(), [&](const auto i) {
    auto& node = dts_skeleton.RefNode(sorted_node_list[i]);
    const auto& node_info = node.info;
    auto& node_data = node.data;
    // Calculate angular velocity and apply torque.
    auto angular_velocity = glm::vec3(0, 0, 0);
    auto p0_velocity = glm::vec3(0.0f, 0.0f, 0.0f);
    auto p1_velocity = glm::vec3(0.0f, 0.0f, 0.0f);
    if (node_data.inv_mass != 0.0f) {
      const glm::quat rotation = node_data.q * glm::conjugate(node_data.last_q);
      angular_velocity = glm::vec3(rotation.x, rotation.y, rotation.z) * 2.0f * inv_time_step;
      p0_velocity = inv_time_step * (node_data.particle0.x - node_data.particle0.last_x);
      p1_velocity = inv_time_step * (node_data.particle1.x - node_data.particle1.last_x);
    }
    node_data.angular_v = angular_velocity;
    node_data.particle0.v = p0_velocity;
    node_data.particle1.v = p1_velocity;
  });
}
