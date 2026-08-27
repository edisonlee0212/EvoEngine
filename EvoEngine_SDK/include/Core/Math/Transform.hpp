
#pragma once
#include "Entity.hpp"

namespace evo_engine {

/**
 * @brief Component representing the flags for transformation updates.
 */
struct TransformUpdateFlag : IDataComponent {
  /// Indicates if the global transformation was modified.
  bool global_transform_modified = false;

  /// Indicates if the transform was modified.
  bool transform_modified = false;
};

/**
 * @brief Component for storing and manipulating global transformations.
 */
struct EVOENGINE_API GlobalTransform : IDataComponent {
  /// The transformation matrix value.
  glm::mat4 value =
      glm::translate(glm::vec3(0.0f)) * glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(1.0f));

  /**
   * @brief Equality operator.
   * @param other The other GlobalTransform object to compare.
   * @return True if both transforms are equal, otherwise false.
   */
  bool operator==(const GlobalTransform &other) const {
    return other.value == value;
  }

  /**
   * @brief Inequality operator.
   * @param other The other GlobalTransform object to compare.
   * @return True if both transforms are not equal, otherwise false.
   */
  bool operator!=(const GlobalTransform &other) const {
    return other.value != value;
  }

  /**
   * @brief Decomposes the transformation matrix into translation, rotation (Euler angles), and scale.
   * @param translation Vector to store the translation.
   * @param euler_angles Vector to store the Euler angles of rotation.
   * @param scale Vector to store the scale.
   * @return True if decomposition is successful, otherwise false.
   */
  bool Decompose(glm::vec3 &translation, glm::vec3 &euler_angles, glm::vec3 &scale) const;

  /**
   * @brief Decomposes the transformation matrix into translation, rotation (quaternion), and scale.
   * @param translation Vector to store the translation.
   * @param rotation Quaternion to store the rotation.
   * @param scale Vector to store the scale.
   * @return True if decomposition is successful, otherwise false.
   */
  bool Decompose(glm::vec3 &translation, glm::quat &rotation, glm::vec3 &scale) const;

  /**
   * @brief Gets the position from the transformation matrix.
   * @return The position as a glm::vec3.
   */
  [[nodiscard]] glm::vec3 GetPosition() const;

  /**
   * @brief Gets the scale from the transformation matrix.
   * @return The scale as a glm::vec3.
   */
  [[nodiscard]] glm::vec3 GetScale() const;

  /**
   * @brief Gets the rotation from the transformation matrix as a quaternion.
   * @return The rotation as a glm::quat.
   */
  [[nodiscard]] glm::quat GetRotation() const;

  /**
   * @brief Gets the rotation from the transformation matrix as Euler angles.
   * @return The rotation as a glm::vec3 of Euler angles.
   */
  [[nodiscard]] glm::vec3 GetEulerRotation() const;

  /**
   * @brief Sets the position in the transformation matrix.
   * @param new_position The new position to set.
   */
  void SetPosition(const glm::vec3 &new_position);

  /**
   * @brief Sets the scale in the transformation matrix.
   * @param new_scale The new scale to set.
   */
  void SetScale(const glm::vec3 &new_scale);

  /**
   * @brief Sets the rotation in the transformation matrix using a quaternion.
   * @param new_rotation The new rotation to set.
   */
  void SetRotation(const glm::quat &new_rotation);

  /**
   * @brief Sets the rotation in the transformation matrix using Euler angles.
   * @param new_euler_rotation The new rotation to set as Euler angles.
   */
  void SetEulerRotation(const glm::vec3 &new_euler_rotation);

  /**
   * @brief Sets the transformation matrix using position, Euler rotation, and scale.
   * @param position The position to set.
   * @param euler_rotation The Euler rotation to set.
   * @param scale The scale to set.
   */
  void SetValue(const glm::vec3 &position, const glm::vec3 &euler_rotation, const glm::vec3 &scale);

  /**
   * @brief Sets the transformation matrix using position, quaternion rotation, and scale.
   * @param position The position to set.
   * @param rotation The quaternion rotation to set.
   * @param scale The scale to set.
   */
  void SetValue(const glm::vec3 &position, const glm::quat &rotation, const glm::vec3 &scale);

  /**
   * @brief Transforms a point using the transformation matrix.
   * @param point The point to transform.
   * @return The transformed point as a glm::vec3.
   */
  glm::vec3 TransformPoint(const glm::vec3 &point) const;

  /**
   * @brief Transforms a vector using the transformation matrix.
   * @param vector The vector to transform.
   * @return The transformed vector as a glm::vec3.
   */
  glm::vec3 TransformVector(const glm::vec3 &vector) const;
};

/**
 * @brief Component for storing and manipulating local transformations.
 */
struct EVOENGINE_API Transform : IDataComponent {
  /// The transformation matrix value.
  glm::mat4 value =
      glm::translate(glm::vec3(0.0f)) * glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(1.0f));

  /**
   * @brief Equality operator.
   * @param other The other Transform object to compare.
   * @return True if both transforms are equal, otherwise false.
   */
  bool operator==(const Transform &other) const {
    return other.value == value;
  }

  /**
   * @brief Inequality operator.
   * @param other The other Transform object to compare.
   * @return True if both transforms are not equal, otherwise false.
   */
  bool operator!=(const Transform &other) const {
    return other.value != value;
  }

  /**
   * @brief Decomposes the transformation matrix into translation, rotation (Euler angles), and scale.
   * @param translation Vector to store the translation.
   * @param euler_angles Vector to store the Euler angles of rotation.
   * @param scale Vector to store the scale.
   * @return True if decomposition is successful, otherwise false.
   */
  bool Decompose(glm::vec3 &translation, glm::vec3 &euler_angles, glm::vec3 &scale) const;

  /**
   * @brief Decomposes the transformation matrix into translation, rotation (quaternion), and scale.
   * @param translation Vector to store the translation.
   * @param rotation Quaternion to store the rotation.
   * @param scale Vector to store the scale.
   * @return True if decomposition is successful, otherwise false.
   */
  bool Decompose(glm::vec3 &translation, glm::quat &rotation, glm::vec3 &scale) const;

  /**
   * @brief Gets the position from the transformation matrix.
   * @return The position as a glm::vec3.
   */
  [[nodiscard]] glm::vec3 GetPosition() const;

  /**
   * @brief Gets the scale from the transformation matrix.
   * @return The scale as a glm::vec3.
   */
  [[nodiscard]] glm::vec3 GetScale() const;

  /**
   * @brief Gets the rotation from the transformation matrix as a quaternion.
   * @return The rotation as a glm::quat.
   */
  [[nodiscard]] glm::quat GetRotation() const;

  /**
   * @brief Gets the rotation from the transformation matrix as Euler angles.
   * @return The rotation as a glm::vec3 of Euler angles.
   */
  [[nodiscard]] glm::vec3 GetEulerRotation() const;

  /**
   * @brief Sets the position in the transformation matrix.
   * @param new_position The new position to set.
   */
  void SetPosition(const glm::vec3 &new_position);

  /**
   * @brief Sets the scale in the transformation matrix.
   * @param new_scale The new scale to set.
   */
  void SetScale(const glm::vec3 &new_scale);

  /**
   * @brief Sets the rotation in the transformation matrix using a quaternion.
   * @param new_rotation The new rotation to set.
   */
  void SetRotation(const glm::quat &new_rotation);

  /**
   * @brief Sets the rotation in the transformation matrix using Euler angles.
   * @param new_euler_rotation The new rotation to set as Euler angles.
   */
  void SetEulerRotation(const glm::vec3 &new_euler_rotation);

  /**
   * @brief Sets the transformation matrix using position, Euler rotation, and scale.
   * @param position The position to set.
   * @param euler_rotation The Euler rotation to set.
   * @param scale The scale to set.
   */
  void SetValue(const glm::vec3 &position, const glm::vec3 &euler_rotation, const glm::vec3 &scale);

  /**
   * @brief Sets the transformation matrix using position, quaternion rotation, and scale.
   * @param position The position to set.
   * @param rotation The quaternion rotation to set.
   * @param scale The scale to set.
   */
  void SetValue(const glm::vec3 &position, const glm::quat &rotation, const glm::vec3 &scale);

  /**
   * @brief Transforms a point using the transformation matrix.
   * @param point The point to transform.
   * @return The transformed point as a glm::vec3.
   */
  glm::vec3 TransformPoint(const glm::vec3 &point) const;

  /**
   * @brief Transforms a vector using the transformation matrix.
   * @param vector The vector to transform.
   * @return The transformed vector as a glm::vec3.
   */
  glm::vec3 TransformVector(const glm::vec3 &vector) const;
};

}  // namespace evo_engine
