#include "DynamicSkeleton.hpp"

#include "DsColliders.hpp"
#include "DynamicStrands.hpp"

using namespace eco_sys_lab_plugin;

bool DynamicSkeleton::InitializeParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Wood density", &wood_density, 0.001f, 0.001f, 1.0f))
    changed = true;
  return changed;
}

bool DynamicSkeleton::PhysicsParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  /*
  if (ImGui::DragFloat("Time step", &time_step, 0.001f, 0.001f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Sub step", &sub_step, 1, 1, 100)) {
    changed = true;
  }*/
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
    node_data.particle1.x0 = node_data.particle1.x = node_data.particle1.last_x =
        initialize_parameters.root_transform.TransformPoint(node_info.GetGlobalEndPosition());
    node_data.particle0.v = node_data.particle1.v = node_data.particle0.a = node_data.particle1.a = glm::vec3(0.f);
    node_data.q0 = node.data.q = node.data.last_q =
        initialize_parameters.root_transform.GetRotation() * node_info.global_rotation;
    node_data.angular_v = node_data.torque = glm::vec3(0.f);
    const float mass = node_info.thickness * node_info.thickness * glm::pi<float>() * 0.25f *
                       initialize_parameters.wood_density * node_info.length;
    node_data.inv_mass = 1.f / mass;
    node_data.inertia_tensor = DynamicStrands::ComputeInertiaTensorRod(mass, node_info.thickness, node_info.length);
    node_data.inv_inertia_tensor = 1.f / node_data.inertia_tensor;
  });
  if (initialize_parameters.static_root) {
    dts_skeleton.RefNode(sorted_node_list[0]).data.inv_mass = 0.f;
  }
}

void DynamicSkeleton::Physics(const PhysicsParameters& physics_parameters, const std::function<void()>& pre_step_action,
                              const std::function<void()>& sub_step_action) {
  pre_step_action();
  for (int sub_step_index = 0; sub_step_index < sub_step; sub_step_index++) {
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
  auto predict_node = [](const float sub_time_step, const PhysicsParameters& parameters,
                         DynamicSkeletonNodeData& node_data) {
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
      node_data.angular_v *= 1.0f - parameters.angular_velocity_damping * glm::length(node_data.angular_v);
      node_data.angular_v +=
          sub_time_step * node_data.inv_inertia_w *
          (node_data.torque - glm::cross(node_data.angular_v, node_data.inertia_w * node_data.angular_v));

      node_data.particle0.v *= 1.0f - parameters.velocity_damping * glm::length(node_data.particle0.v);
      node_data.particle0.v += node_data.particle0.a * sub_time_step;

      node_data.particle1.v *= 1.0f - parameters.velocity_damping * glm::length(node_data.particle1.v);
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
    predict_node(time_step / sub_step, physics_parameters, node_data);
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

  const auto sub_time_step = time_step / sub_step;
  const auto inv_time_step = 1.f / sub_time_step;

  const auto& sorted_node_list = dts_skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_node_list) {
    // 1. If you have a parent node, apply bend and twist
    auto& node = dts_skeleton.RefNode(node_handle);
    auto& node_data = node.data;
    const auto parent_handle = node.GetParentHandle();
    float parent_inv_mass = node_data.inv_mass;
    float children_inv_mass = node_data.inv_mass;

    if (parent_handle != -1) {
      auto& parent_node = dts_skeleton.RefNode(parent_handle);
      const auto& parent_node_data = parent_node.data;
      parent_inv_mass = parent_node_data.inv_mass;
    } else {
      parent_inv_mass = 0.f;
    }
    const auto& child_handles = node.PeekChildHandles();
    if (!child_handles.empty()) {
      float children_mass = 0.f;
      for (const auto& child_handle : node.PeekChildHandles()) {
        const auto child_inv_mass = dts_skeleton.PeekNode(child_handle).data.inv_mass;
        if (child_inv_mass != 0.f) {
          children_mass += 1.f / child_inv_mass;
        } else {
          children_mass = 0.f;
          break;
        }
      }
      if (children_mass != 0.f) {
        children_inv_mass = 1.f / children_mass;
      }
    }

    // 2. Stretch & shear
    glm::vec3 x0_correction, x1_correction;
    glm::quat q_correction;
    project_shear_stretch_constraint(inv_time_step, node_data.particle0.x, node_data.particle1.x, node_data.q,
                                     parent_inv_mass, children_inv_mass, (parent_inv_mass + children_inv_mass) / 2.f,
                                     glm::vec3(0.0f), node.info.length, x0_correction, x1_correction, q_correction);

    node_data.particle0.x += x0_correction;
    node_data.particle1.x += x1_correction;
    node_data.q = glm::normalize(node_data.q + q_correction);
    // 3. Connect
    if (parent_handle != -1) {
      dts_skeleton.RefNode(parent_handle).data.particle1.x = node_data.particle0.x;
    }
    q_correction.x = q_correction.y = q_correction.z = q_correction.w = 0.f;
    if (!child_handles.empty()) {
      for (const auto& child_handle : child_handles) {
        dts_skeleton.RefNode(child_handle).data.particle0.x = node_data.particle1.x;

        auto& child_node = dts_skeleton.RefNode(child_handle);
        auto& child_node_data = child_node.data;
        glm::quat child_q_correction, local_q_correction;
        const auto rest_darboux_vector = glm::conjugate(node_data.q0) * child_node_data.q0;
        project_bend_twist_constraint(inv_time_step, node_data.q, node_data.inv_mass, child_node_data.q,
                                      child_node_data.inv_mass, glm::vec3(0.0f), rest_darboux_vector,
                                      local_q_correction, child_q_correction);
        child_node_data.q = glm::normalize(child_node_data.q + child_q_correction);
        q_correction += local_q_correction;
      }
      q_correction.w /= child_handles.size();
      q_correction.x /= child_handles.size();
      q_correction.y /= child_handles.size();
      q_correction.z /= child_handles.size();
      node_data.q = glm::normalize(node_data.q + q_correction);
    }
  }
}

void DynamicSkeleton::VelocityUpdate(const PhysicsParameters& physics_parameters) {
  const auto& sorted_node_list = dts_skeleton.PeekSortedNodeList();
  const auto sub_time_step = time_step / sub_step;
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
