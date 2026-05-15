#pragma once

#include "CUDABuffer.hpp"

#include "glm/glm.hpp"

namespace evo_engine {
struct SharedCoordinates {
  // false ... use uniform distribution in Beta
  // true ... use uniform distribution in cos(Beta)
  bool use_cos_beta;

  int beta_size;
  float alpha_step;
  int alpha_size;
  float theta_step;
  int theta_size;
  float phi_step;
  int phi_size;

  // the BTF single point coordinates in degrees
  float beta;   // 1D
  float alpha;  // 2D
  float theta;  // 3D
  float phi;    // 4D

  // interpolation values for Pdf1D
  int current_beta_low_bound;
  float beta_weight;
  float beta_min2;

  // interpolation values for Pdf2D
  int current_alpha_low_bound;
  float alpha_weight;
  float alpha_min2;

  // interpolation values for Pdf3D
  int current_theta_low_bound;
  float theta_weight;
  float theta_min2;

  // interpolation values for Pdf4D
  int current_phi_low_bound;
  float phi_weight;

  float scale;

  bool hdr_flag;
  bool use_btf_flag;
#pragma region CUDA
  // the values to be used for interpolation in beta coordinate
  CudaBuffer beta_angles_buffer;
  float *beta_angles_device_ptr;  // the sequence of values used
  // Here we set the structure for particular angle beta
  __device__ void SetForAngleBetaDeg(const float &target_beta) {
    assert(target_beta > -90.001f && target_beta < 90.001f);
    beta = target_beta;
    if (use_cos_beta) {
      current_beta_low_bound = glm::clamp(
          static_cast<int>((glm::sin(glm::radians(target_beta)) + 1.0f) / 2.0f * (beta_size - 1)), 0, beta_size - 2);
      beta_weight =
          (target_beta - beta_angles_device_ptr[current_beta_low_bound]) /
          (beta_angles_device_ptr[current_beta_low_bound + 1] - beta_angles_device_ptr[current_beta_low_bound]);
      assert(beta_weight > -0.001f && beta_weight < 1.001f);
    } else {
      // The angles are quantized uniformly in degrees
      const float step_beta = 180.0f / static_cast<float>(beta_size - 1);
      current_beta_low_bound = glm::clamp(static_cast<int>((beta + 90.0f) / step_beta), 0, beta_size - 2);
      beta_weight = (beta + 90.0f - static_cast<float>(current_beta_low_bound) * step_beta) / step_beta;
      assert(beta_weight > -0.001f && beta_weight < 1.001f);
    }
  }

  // Here we set the structure for particular angle alpha
  __device__ void SetForAngleAlphaDeg(const float &target_alpha) {
    assert(target_alpha > -90.001f && target_alpha < 90.001f);
    alpha = target_alpha;
    current_alpha_low_bound = glm::clamp(static_cast<int>((target_alpha + 90.0f) / alpha_step), 0, alpha_size - 2);
    alpha_weight = (target_alpha + 90.f - static_cast<float>(current_alpha_low_bound) * alpha_step) / alpha_step;
    assert(alpha_weight > -0.001f && alpha_weight < 1.001f);
  }

  // Here we set the structure for particular angle alpha
  __device__ void SetForAnglePhiDeg(const float &target_phi) {
    assert(target_phi > -0.001f && target_phi < 360.001f);
    phi = target_phi;
    current_phi_low_bound = glm::clamp(static_cast<int>(target_phi / phi_step), 0, phi_size - 1);
    phi_weight = (target_phi - static_cast<float>(current_phi_low_bound) * phi_step) / phi_step;
    assert(phi_weight > -0.001f && phi_weight < 1.001f);
  }

  // Here we set the structure for particular angle alpha
  __device__ void SetForAngleThetaDeg(const float &target_theta) {
    assert(target_theta > -0.001f && target_theta < 90.001f);
    theta = target_theta;
    current_theta_low_bound = glm::clamp(static_cast<int>(target_theta / theta_step), 0, theta_size - 2);
    theta_weight = (target_theta - static_cast<float>(current_theta_low_bound) * theta_step) / theta_step;
    assert(theta_weight > -0.001f && theta_weight < 1.001f);
  }

#pragma endregion
  void Set(const bool &use_btf_flag, const bool &use_cos_beta, const int &num_of_beta, const int &num_of_alpha,
           const float &step_alpha, const int &num_of_theta, const float &step_theta, const int &num_of_phi,
           const float &step_phi) {
    this->use_btf_flag = use_btf_flag;
    this->use_cos_beta = use_cos_beta;
    this->beta_size = num_of_beta;
    this->alpha_size = num_of_alpha;
    this->alpha_step = step_alpha;
    this->theta_size = num_of_theta;
    this->theta_step = step_theta;
    this->phi_step = step_phi;
    this->phi_size = num_of_phi;
    this->hdr_flag = false;
  }
};

__device__ inline void ConvertThetaPhiToBetaAlpha(const float theta, const float phi, float &beta, float &alpha,
                                                  const SharedCoordinates &tc) {
  if (tc.use_btf_flag) {
    const float x = cos(phi - tc.phi) * sin(theta);
    const float y = sin(phi - tc.phi) * sin(theta);
    // float z = cos(thetaI);

    beta = asin(glm::clamp(y, -1.0f, 1.0f));
    const float cos_beta = cos(beta);

    if (cos_beta < 0.001f) {
      alpha = 0.0f;
      return;
    }
    const float tmp = glm::clamp(-x / cos_beta, -1.0f, 1.0f);
    alpha = asin(tmp);
    return;
  }

  // This is 3D vector
  glm::vec3 xyz;
  // Here we convert the angles to 3D vector
  xyz[0] = glm::cos(phi) * glm::sin(theta);
  xyz[1] = glm::sin(phi) * glm::sin(theta);
  xyz[2] = glm::cos(theta);

  // Here we convert 3D vector to alpha-beta parametrization over hemisphere
  beta = glm::asin(xyz[0]);
  const float cos_beta = glm::cos(beta);
  if (cos_beta < 0.001f) {
    alpha = 0.0f;
    return;
  }
  const float aux = glm::clamp(xyz[1] / cos_beta, -1.0f, 1.0f);
  alpha = glm::asin(aux);
}
}  // namespace evo_engine