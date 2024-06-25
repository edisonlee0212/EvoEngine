#pragma once
#include "Entity.hpp"
namespace evo_engine {
struct TransformUpdateFlag : IDataComponent {
  bool global_transform_modified = false;
  bool transform_modified = false;
};

struct GlobalTransform : IDataComponent {
  glm::mat4 value =
      glm::translate(glm::vec3(0.0f)) * glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(1.0f));
  bool operator==(const GlobalTransform &other) const {
    return other.value == value;
  }
#pragma region Get &set
  bool Decompose(glm::vec3 &translation, glm::vec3 &euler_angles, glm::vec3 &scale) const {
    using namespace glm;
    using T = float;
    mat4 local_matrix(value);

    // Normalize the matrix.
    if (epsilonEqual(local_matrix[3][3], static_cast<T>(0), epsilon<T>()))
      return false;

    for (length_t i = 0; i < 4; ++i)
      for (length_t j = 0; j < 4; ++j)
        local_matrix[i][j] /= local_matrix[3][3];

    // perspectiveMatrix is used to solve for perspective, but it also provides
    // an easy way to test for singularity of the upper 3x3 component.
    mat4 perspective_matrix(local_matrix);

    for (length_t i = 0; i < 3; i++)
      perspective_matrix[i][3] = static_cast<T>(0);
    perspective_matrix[3][3] = static_cast<T>(1);

    if (epsilonEqual(determinant(perspective_matrix), static_cast<T>(0), epsilon<T>()))
      return false;

    // First, isolate perspective.  This is the messiest.
    if (epsilonNotEqual(local_matrix[0][3], static_cast<T>(0), epsilon<T>()) ||
        epsilonNotEqual(local_matrix[1][3], static_cast<T>(0), epsilon<T>()) ||
        epsilonNotEqual(local_matrix[2][3], static_cast<T>(0), epsilon<T>())) {
      // Clear the perspective partition
      local_matrix[0][3] = local_matrix[1][3] = local_matrix[2][3] = static_cast<T>(0);
      local_matrix[3][3] = static_cast<T>(1);
    }

    // Next take care of translation (easy).
    translation = vec3(local_matrix[3]);
    local_matrix[3] = vec4(0, 0, 0, local_matrix[3].w);

    vec3 row[3];

    // Now get scale and shear.
    for (length_t i = 0; i < 3; ++i)
      for (length_t j = 0; j < 3; ++j)
        row[i][j] = local_matrix[i][j];

    // Compute X scale factor and normalize first row.
    scale.x = length(row[0]);  // v3Length(Row[0]);

    row[0] = glm::detail::scale(row[0], static_cast<T>(1));

    // Compute XY shear factor and make 2nd row orthogonal to 1st.
    glm::vec3 skew;
    skew.z = dot(row[0], row[1]);
    row[1] = glm::detail::combine(row[1], row[0], static_cast<T>(1), -skew.z);

    // Now, compute Y scale and normalize 2nd row.
    scale.y = length(row[1]);
    row[1] = glm::detail::scale(row[1], static_cast<T>(1));
    // skew.z /= scale.y;

    // Compute XZ and YZ shears, orthogonality 3rd row.
    skew.y = glm::dot(row[0], row[2]);
    row[2] = glm::detail::combine(row[2], row[0], static_cast<T>(1), -skew.y);
    skew.x = glm::dot(row[1], row[2]);
    row[2] = glm::detail::combine(row[2], row[1], static_cast<T>(1), -skew.x);

    // Next, get Z scale and normalize 3rd row.
    scale.z = length(row[2]);
    row[2] = glm::detail::scale(row[2], static_cast<T>(1));
    // skew.y /= scale.z;
    // skew.x /= scale.z;

    // At this point, the matrix (in rows[]) is orthonormal.
    // Check for a coordinate system flip.  If the determinant
    // is -1, then negate the matrix and the scaling factors.
    if (const vec3 p_dum3 = cross(row[1], row[2]); dot(row[0], p_dum3) < 0) {
      for (length_t i = 0; i < 3; i++) {
        scale[i] *= static_cast<T>(-1);
        row[i] *= static_cast<T>(-1);
      }
    }

    euler_angles.y = glm::asin(-row[0][2]);
    if (glm::cos(euler_angles.y) != 0.f) {
      euler_angles.x = atan2(row[1][2], row[2][2]);
      euler_angles.z = atan2(row[0][1], row[0][0]);
    } else {
      euler_angles.x = atan2(-row[2][0], row[1][1]);
      euler_angles.z = 0;
    }
    return true;
  }
  bool Decompose(glm::vec3 &translation, glm::quat &rotation, glm::vec3 &scale) const {
    using namespace glm;
    using T = float;
    mat4 local_matrix(value);

    // Normalize the matrix.
    if (epsilonEqual(local_matrix[3][3], static_cast<T>(0), epsilon<T>()))
      return false;

    for (length_t i = 0; i < 4; ++i)
      for (length_t j = 0; j < 4; ++j)
        local_matrix[i][j] /= local_matrix[3][3];

    // perspectiveMatrix is used to solve for perspective, but it also provides
    // an easy way to test for singularity of the upper 3x3 component.
    mat4 perspective_matrix(local_matrix);

    for (length_t i = 0; i < 3; i++)
      perspective_matrix[i][3] = static_cast<T>(0);
    perspective_matrix[3][3] = static_cast<T>(1);

    /// TODO: Fixme!
    if (epsilonEqual(determinant(perspective_matrix), static_cast<T>(0), epsilon<T>()))
      return false;

    // First, isolate perspective.  This is the messiest.
    if (epsilonNotEqual(local_matrix[0][3], static_cast<T>(0), epsilon<T>()) ||
        epsilonNotEqual(local_matrix[1][3], static_cast<T>(0), epsilon<T>()) ||
        epsilonNotEqual(local_matrix[2][3], static_cast<T>(0), epsilon<T>())) {
      // Clear the perspective partition
      local_matrix[0][3] = local_matrix[1][3] = local_matrix[2][3] = static_cast<T>(0);
      local_matrix[3][3] = static_cast<T>(1);
    }

    // Next take care of translation (easy).
    translation = vec3(local_matrix[3]);
    local_matrix[3] = vec4(0, 0, 0, local_matrix[3].w);

    vec3 row[3];

    // Now get scale and shear.
    for (length_t i = 0; i < 3; ++i)
      for (length_t j = 0; j < 3; ++j)
        row[i][j] = local_matrix[i][j];

    // Compute X scale factor and normalize first row.
    scale.x = length(row[0]);  // v3Length(Row[0]);

    row[0] = glm::detail::scale(row[0], static_cast<T>(1));

    // Compute XY shear factor and make 2nd row orthogonal to 1st.
    glm::vec3 skew;
    skew.z = dot(row[0], row[1]);
    row[1] = glm::detail::combine(row[1], row[0], static_cast<T>(1), -skew.z);

    // Now, compute Y scale and normalize 2nd row.
    scale.y = length(row[1]);
    row[1] = glm::detail::scale(row[1], static_cast<T>(1));
    // skew.z /= scale.y;

    // Compute XZ and YZ shears, orthogonal 3rd row.
    skew.y = glm::dot(row[0], row[2]);
    row[2] = glm::detail::combine(row[2], row[0], static_cast<T>(1), -skew.y);
    skew.x = glm::dot(row[1], row[2]);
    row[2] = glm::detail::combine(row[2], row[1], static_cast<T>(1), -skew.x);

    // Next, get Z scale and normalize 3rd row.
    scale.z = length(row[2]);
    row[2] = glm::detail::scale(row[2], static_cast<T>(1));
    // skew.y /= scale.z;
    // skew.x /= scale.z;

    // At this point, the matrix (in rows[]) is orthonormal.
    // Check for a coordinate system flip.  If the determinant
    // is -1, then negate the matrix and the scaling factors.
    if (const vec3 p_dum3 = cross(row[1], row[2]); dot(row[0], p_dum3) < 0) {
      for (length_t i = 0; i < 3; i++) {
        scale[i] *= static_cast<T>(-1);
        row[i] *= static_cast<T>(-1);
      }
    }
    T root;
    if (const T trace = row[0].x + row[1].y + row[2].z; trace > static_cast<T>(0)) {
      root = glm::sqrt(trace + static_cast<T>(1.0));
      rotation.w = static_cast<T>(0.5) * root;
      root = static_cast<T>(0.5) / root;
      rotation.x = root * (row[1].z - row[2].y);
      rotation.y = root * (row[2].x - row[0].z);
      rotation.z = root * (row[0].y - row[1].x);
    }  // End if > 0
    else {
      static int next[3] = {1, 2, 0};
      int i = 0;
      if (row[1].y > row[0].x)
        i = 1;
      if (row[2].z > row[i][i])
        i = 2;
      const int j = next[i];
      const int k = next[j];

      root = glm::sqrt(row[i][i] - row[j][j] - row[k][k] + static_cast<T>(1.0));

      rotation[i] = static_cast<T>(0.5) * root;
      root = static_cast<T>(0.5) / root;
      rotation[j] = root * (row[i][j] + row[j][i]);
      rotation[k] = root * (row[i][k] + row[k][i]);
      rotation.w = root * (row[j][k] - row[k][j]);
    }  // End if <= 0

    return true;
  }

  [[nodiscard]] glm::vec3 GetPosition() const {
    return value[3];
  }
  [[nodiscard]] glm::vec3 GetScale() const {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    Decompose(trans, rotation, scale);
    return scale;
  }
  [[nodiscard]] glm::quat GetRotation() const {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    Decompose(trans, rotation, scale);
    return rotation;
  }
  [[nodiscard]] glm::vec3 GetEulerRotation() const {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::vec3 rotation;
    Decompose(trans, rotation, scale);
    return rotation;
  }
  void SetPosition(const glm::vec3 &new_position) {
    this->value[3].x = new_position.x;
    this->value[3].y = new_position.y;
    this->value[3].z = new_position.z;
  }
  void SetScale(const glm::vec3 &new_scale) {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    Decompose(trans, rotation, scale);
    this->value = glm::translate(trans) * glm::mat4_cast(rotation) * glm::scale(new_scale);
  }
  void SetRotation(const glm::quat &new_rotation) {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    Decompose(trans, rotation, scale);
    this->value = glm::translate(trans) * glm::mat4_cast(new_rotation) * glm::scale(scale);
  }
  void SetEulerRotation(const glm::vec3 &new_euler_rotation) {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    Decompose(trans, rotation, scale);
    this->value = glm::translate(trans) * glm::mat4_cast(glm::quat(new_euler_rotation)) * glm::scale(scale);
  }
  void SetValue(const glm::vec3 &position, const glm::vec3 &euler_rotation, const glm::vec3 &scale) {
    value = glm::translate(position) * glm::mat4_cast(glm::quat(euler_rotation)) * glm::scale(scale);
  }
  void SetValue(const glm::vec3 &position, const glm::quat &rotation, const glm::vec3 &scale) {
    value = glm::translate(position) * glm::mat4_cast(rotation) * glm::scale(scale);
  }
#pragma endregion
};
struct Transform : IDataComponent {
  glm::mat4 value =
      glm::translate(glm::vec3(0.0f)) * glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(1.0f));
  bool operator==(const GlobalTransform &other) const {
    return other.value == value;
  }
#pragma region Get &set
  bool Decompose(glm::vec3 &translation, glm::vec3 &euler_angles, glm::vec3 &scale) const {
    using namespace glm;
    using T = float;
    mat4 local_matrix(value);

    // Normalize the matrix.
    if (epsilonEqual(local_matrix[3][3], static_cast<T>(0), epsilon<T>()))
      return false;

    for (length_t i = 0; i < 4; ++i)
      for (length_t j = 0; j < 4; ++j)
        local_matrix[i][j] /= local_matrix[3][3];

    // perspectiveMatrix is used to solve for perspective, but it also provides
    // an easy way to test for singularity of the upper 3x3 component.
    mat4 perspective_matrix(local_matrix);

    for (length_t i = 0; i < 3; i++)
      perspective_matrix[i][3] = static_cast<T>(0);
    perspective_matrix[3][3] = static_cast<T>(1);

    if (epsilonEqual(determinant(perspective_matrix), static_cast<T>(0), epsilon<T>()))
      return false;

    // First, isolate perspective.  This is the messiest.
    if (epsilonNotEqual(local_matrix[0][3], static_cast<T>(0), epsilon<T>()) ||
        epsilonNotEqual(local_matrix[1][3], static_cast<T>(0), epsilon<T>()) ||
        epsilonNotEqual(local_matrix[2][3], static_cast<T>(0), epsilon<T>())) {
      // Clear the perspective partition
      local_matrix[0][3] = local_matrix[1][3] = local_matrix[2][3] = static_cast<T>(0);
      local_matrix[3][3] = static_cast<T>(1);
    }

    // Next take care of translation (easy).
    translation = vec3(local_matrix[3]);
    local_matrix[3] = vec4(0, 0, 0, local_matrix[3].w);

    vec3 row[3];

    // Now get scale and shear.
    for (length_t i = 0; i < 3; ++i)
      for (length_t j = 0; j < 3; ++j)
        row[i][j] = local_matrix[i][j];

    // Compute X scale factor and normalize first row.
    scale.x = length(row[0]);  // v3Length(Row[0]);

    row[0] = glm::detail::scale(row[0], static_cast<T>(1));

    // Compute XY shear factor and make 2nd row orthogonal to 1st.
    glm::vec3 skew;
    skew.z = dot(row[0], row[1]);
    row[1] = glm::detail::combine(row[1], row[0], static_cast<T>(1), -skew.z);

    // Now, compute Y scale and normalize 2nd row.
    scale.y = length(row[1]);
    row[1] = glm::detail::scale(row[1], static_cast<T>(1));
    // skew.z /= scale.y;

    // Compute XZ and YZ shears, orthogonality 3rd row.
    skew.y = glm::dot(row[0], row[2]);
    row[2] = glm::detail::combine(row[2], row[0], static_cast<T>(1), -skew.y);
    skew.x = glm::dot(row[1], row[2]);
    row[2] = glm::detail::combine(row[2], row[1], static_cast<T>(1), -skew.x);

    // Next, get Z scale and normalize 3rd row.
    scale.z = length(row[2]);
    row[2] = glm::detail::scale(row[2], static_cast<T>(1));
    // skew.y /= scale.z;
    // skew.x /= scale.z;

    // At this point, the matrix (in rows[]) is orthonormal.
    // Check for a coordinate system flip.  If the determinant
    // is -1, then negate the matrix and the scaling factors.
    if (const vec3 p_dum3 = cross(row[1], row[2]); dot(row[0], p_dum3) < 0) {
      for (length_t i = 0; i < 3; i++) {
        scale[i] *= static_cast<T>(-1);
        row[i] *= static_cast<T>(-1);
      }
    }

    euler_angles.y = glm::asin(-row[0][2]);
    if (glm::cos(euler_angles.y) != 0.f) {
      euler_angles.x = atan2(row[1][2], row[2][2]);
      euler_angles.z = atan2(row[0][1], row[0][0]);
    } else {
      euler_angles.x = atan2(-row[2][0], row[1][1]);
      euler_angles.z = 0;
    }
    return true;
  }
  bool Decompose(glm::vec3 &translation, glm::quat &rotation, glm::vec3 &scale) const {
    using namespace glm;
    using T = float;
    mat4 local_matrix(value);

    // Normalize the matrix.
    if (epsilonEqual(local_matrix[3][3], static_cast<T>(0), epsilon<T>()))
      return false;

    for (length_t i = 0; i < 4; ++i)
      for (length_t j = 0; j < 4; ++j)
        local_matrix[i][j] /= local_matrix[3][3];

    // perspectiveMatrix is used to solve for perspective, but it also provides
    // an easy way to test for singularity of the upper 3x3 component.
    mat4 perspective_matrix(local_matrix);

    for (length_t i = 0; i < 3; i++)
      perspective_matrix[i][3] = static_cast<T>(0);
    perspective_matrix[3][3] = static_cast<T>(1);

    /// TODO: Fixme!
    if (epsilonEqual(determinant(perspective_matrix), static_cast<T>(0), epsilon<T>()))
      return false;

    // First, isolate perspective.  This is the messiest.
    if (epsilonNotEqual(local_matrix[0][3], static_cast<T>(0), epsilon<T>()) ||
        epsilonNotEqual(local_matrix[1][3], static_cast<T>(0), epsilon<T>()) ||
        epsilonNotEqual(local_matrix[2][3], static_cast<T>(0), epsilon<T>())) {
      // Clear the perspective partition
      local_matrix[0][3] = local_matrix[1][3] = local_matrix[2][3] = static_cast<T>(0);
      local_matrix[3][3] = static_cast<T>(1);
    }

    // Next take care of translation (easy).
    translation = vec3(local_matrix[3]);
    local_matrix[3] = vec4(0, 0, 0, local_matrix[3].w);

    vec3 row[3];

    // Now get scale and shear.
    for (length_t i = 0; i < 3; ++i)
      for (length_t j = 0; j < 3; ++j)
        row[i][j] = local_matrix[i][j];

    // Compute X scale factor and normalize first row.
    scale.x = length(row[0]);  // v3Length(Row[0]);

    row[0] = glm::detail::scale(row[0], static_cast<T>(1));

    // Compute XY shear factor and make 2nd row orthogonal to 1st.
    glm::vec3 skew;
    skew.z = dot(row[0], row[1]);
    row[1] = glm::detail::combine(row[1], row[0], static_cast<T>(1), -skew.z);

    // Now, compute Y scale and normalize 2nd row.
    scale.y = length(row[1]);
    row[1] = glm::detail::scale(row[1], static_cast<T>(1));
    // skew.z /= scale.y;

    // Compute XZ and YZ shears, orthogonal 3rd row.
    skew.y = glm::dot(row[0], row[2]);
    row[2] = glm::detail::combine(row[2], row[0], static_cast<T>(1), -skew.y);
    skew.x = glm::dot(row[1], row[2]);
    row[2] = glm::detail::combine(row[2], row[1], static_cast<T>(1), -skew.x);

    // Next, get Z scale and normalize 3rd row.
    scale.z = length(row[2]);
    row[2] = glm::detail::scale(row[2], static_cast<T>(1));
    // skew.y /= scale.z;
    // skew.x /= scale.z;

    // At this point, the matrix (in rows[]) is orthonormal.
    // Check for a coordinate system flip.  If the determinant
    // is -1, then negate the matrix and the scaling factors.
    if (const vec3 p_dum3 = cross(row[1], row[2]); dot(row[0], p_dum3) < 0) {
      for (length_t i = 0; i < 3; i++) {
        scale[i] *= static_cast<T>(-1);
        row[i] *= static_cast<T>(-1);
      }
    }
    T root;
    if (const T trace = row[0].x + row[1].y + row[2].z; trace > static_cast<T>(0)) {
      root = glm::sqrt(trace + static_cast<T>(1.0));
      rotation.w = static_cast<T>(0.5) * root;
      root = static_cast<T>(0.5) / root;
      rotation.x = root * (row[1].z - row[2].y);
      rotation.y = root * (row[2].x - row[0].z);
      rotation.z = root * (row[0].y - row[1].x);
    }  // End if > 0
    else {
      static int next[3] = {1, 2, 0};
      int i = 0;
      if (row[1].y > row[0].x)
        i = 1;
      if (row[2].z > row[i][i])
        i = 2;
      const int j = next[i];
      const int k = next[j];

      root = glm::sqrt(row[i][i] - row[j][j] - row[k][k] + static_cast<T>(1.0));

      rotation[i] = static_cast<T>(0.5) * root;
      root = static_cast<T>(0.5) / root;
      rotation[j] = root * (row[i][j] + row[j][i]);
      rotation[k] = root * (row[i][k] + row[k][i]);
      rotation.w = root * (row[j][k] - row[k][j]);
    }  // End if <= 0

    return true;
  }

  [[nodiscard]] glm::vec3 GetPosition() const {
    return value[3];
  }
  [[nodiscard]] glm::vec3 GetScale() const {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    Decompose(trans, rotation, scale);
    return scale;
  }
  [[nodiscard]] glm::quat GetRotation() const {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    Decompose(trans, rotation, scale);
    return rotation;
  }
  [[nodiscard]] glm::vec3 GetEulerRotation() const {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::vec3 rotation;
    Decompose(trans, rotation, scale);
    return rotation;
  }
  void SetPosition(const glm::vec3 &new_position) {
    this->value[3].x = new_position.x;
    this->value[3].y = new_position.y;
    this->value[3].z = new_position.z;
  }
  void SetScale(const glm::vec3 &new_scale) {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    Decompose(trans, rotation, scale);
    this->value = glm::translate(trans) * glm::mat4_cast(rotation) * glm::scale(new_scale);
  }
  void SetRotation(const glm::quat &new_rotation) {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    Decompose(trans, rotation, scale);
    this->value = glm::translate(trans) * glm::mat4_cast(new_rotation) * glm::scale(scale);
  }
  void SetEulerRotation(const glm::vec3 &new_euler_rotation) {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    Decompose(trans, rotation, scale);
    this->value = glm::translate(trans) * glm::mat4_cast(glm::quat(new_euler_rotation)) * glm::scale(scale);
  }
  void SetValue(const glm::vec3 &position, const glm::vec3 &euler_rotation, const glm::vec3 &scale) {
    value = glm::translate(position) * glm::mat4_cast(glm::quat(euler_rotation)) * glm::scale(scale);
  }
  void SetValue(const glm::vec3 &position, const glm::quat &rotation, const glm::vec3 &scale) {
    value = glm::translate(position) * glm::mat4_cast(rotation) * glm::scale(scale);
  }
#pragma endregion
};
}  // namespace evo_engine