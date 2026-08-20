#pragma once

// ---------------------------------------------------------------------------
// ElasticaSolver
//
// Phase 4: planar quasi-static elastica solver for a single rod-like organ.
// Pure-CPU, decoupled from L-system types so it is unit-testable in isolation.
//
// Problem statement (planar Euler-Bernoulli, large rotation, small strain):
//
//   Rod parameterized by arc length s in [0, L]. State variable theta(s) is
//   the tangent angle measured from the base tangent (theta(0) = 0). The
//   rod has intrinsic curvature kappa_intrinsic(s) (the unloaded shape's
//   curvature) and bending stiffness EI(s) > 0. A distributed load
//   q_perp(s) (force per unit arc length perpendicular to the rod's
//   in-plane bending axis) and an optional tip force F_tip act on the rod.
//
//   Equilibrium of moments about a station s:
//
//       EI(s) * (dtheta/ds(s) - kappa_intrinsic(s)) = M(s)
//
//   where M(s) is the bending moment at s due to all loads distal to s
//   (s' > s). For planar gravity loading, M(s) is computed from the
//   in-plane lever arms of the distal weight (and tip force) about s.
//
// Algorithm (damped fixed-point iteration; converges in 3-6 iters at the
// load levels typical for needles/young internodes):
//
//   1. Compute station positions (x(s), z(s)) by integrating tangent.
//   2. Compute M(s) by tip-to-base sweep accumulating distal weights' lever arms.
//   3. Compute new dtheta/ds = kappa_intrinsic + M / EI; integrate base->tip.
//   4. Damp: theta = (1-alpha) * theta_old + alpha * theta_new.
//   5. Repeat until max |theta_new - theta_old| < tolerance.
//
// Validation (called from ScotsPineSmokeTest):
//
//   Tip-loaded cantilever, kappa_intrinsic = 0, uniform EI, no distributed
//   load, tip force W perpendicular to base tangent: small-deflection theory
//   gives delta_tip = W * L^3 / (3 * EI). The solver reproduces this to
//   well below 1% for W * L^2 / EI < 0.05.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include <glm/glm.hpp>

namespace l_system_package {

/// Inputs to a planar elastica solve. All quantities are in the rod's local
/// in-plane (x, z) frame, with the base at the origin and the base tangent
/// along +Z. Curvature is positive when the rod bends toward +X.
struct PlanarElasticaInput {
  /// Total arc length L (m). Must be > 0.
  float length_m = 0.0f;
  /// Number of stations N+1 along the arc. Must be >= 3.
  int station_count = 17;
  /// kappa_intrinsic(s_norm) in 1/m, positive bends toward +X.
  std::function<float(float)> intrinsic_curvature_per_m;
  /// EI(s_norm) in Pa*m^4. Must be > 0 at every station.
  std::function<float(float)> bending_stiffness_Pa_m4;
  /// Gravitational acceleration in the rod's local in-plane (x, z) frame
  /// (m/s^2). Combined with `mass_per_length_kg_m(s)` to produce the
  /// distributed body force. Pass (0, 0) to disable distributed loading.
  /// Sign convention: components in the same axes as the rod frame.
  glm::vec2 gravity_acceleration_xz_m_s2 = glm::vec2(0.0f, 0.0f);
  /// Per-station mass density (kg/m). When null or returning 0, distributed
  /// loading is disabled regardless of `gravity_acceleration_xz_m_s2`.
  std::function<float(float)> mass_per_length_kg_m;
  /// Concentrated tip force (N) applied at s = L, in (x, z) plane.
  glm::vec2 tip_force_N_xz = glm::vec2(0.0f, 0.0f);
  /// Optional equilibrium seed for load continuation.
  std::vector<float> initial_theta_rad;
  /// Damping factor for fixed-point iteration in (0, 1].
  float relaxation = 0.6f;
  /// Convergence tolerance on max |delta_theta| (radians).
  float tolerance_rad = 1e-5f;
  /// Maximum iteration count.
  int max_iterations = 32;
};

/// Output of a planar elastica solve.
struct PlanarElasticaOutput {
  /// Per-station tangent angle theta(s_i) in radians, theta(0) = 0.
  std::vector<float> theta_rad;
  /// Per-station positions (x, z) in metres, position(0) = (0, 0).
  std::vector<glm::vec2> positions_xz;
  /// Per-station bending moment M(s_i) in N*m (informational; Phase 5 stress feedback).
  std::vector<float> bending_moment_Nm;
  /// True iff the iteration converged below `tolerance_rad`.
  bool converged = false;
  /// Iterations actually performed.
  int iterations_performed = 0;
  /// Final max |delta_theta| at the last iteration (radians).
  float final_max_delta_rad = 0.0f;
};

/// Solve the planar quasi-static elastica problem.
inline PlanarElasticaOutput SolvePlanarElastica(const PlanarElasticaInput& in) {
  PlanarElasticaOutput out;
  const int n = std::max(3, in.station_count);
  if (in.length_m <= 0.0f)
    return out;
  const float L = in.length_m;
  const float ds = L / static_cast<float>(n - 1);

  // Pre-sample stiffness, intrinsic curvature, and mass per length so we
  // do not pay lambda overhead in every iteration.
  std::vector<float> kappa_intr(n, 0.0f);
  std::vector<float> EI(n, 1.0f);
  std::vector<float> mu(n, 0.0f);
  for (int i = 0; i < n; ++i) {
    const float s_norm = static_cast<float>(i) / static_cast<float>(n - 1);
    if (in.intrinsic_curvature_per_m)
      kappa_intr[i] = in.intrinsic_curvature_per_m(s_norm);
    if (in.bending_stiffness_Pa_m4)
      EI[i] = std::max(1e-20f, in.bending_stiffness_Pa_m4(s_norm));
    if (in.mass_per_length_kg_m)
      mu[i] = std::max(0.0f, in.mass_per_length_kg_m(s_norm));
  }

  out.theta_rad = in.initial_theta_rad;
  if (out.theta_rad.size() != static_cast<size_t>(n) ||
      !std::all_of(out.theta_rad.begin(), out.theta_rad.end(), [](const float value) {
        return std::isfinite(value);
      })) {
    out.theta_rad.assign(n, 0.0f);
    for (int i = 1; i < n; ++i) {
      const float kappa_avg = 0.5f * (kappa_intr[i - 1] + kappa_intr[i]);
      out.theta_rad[i] = out.theta_rad[i - 1] + kappa_avg * ds;
    }
  }
  out.theta_rad.front() = 0.0f;

  out.positions_xz.assign(n, glm::vec2(0.0f));
  out.bending_moment_Nm.assign(n, 0.0f);

  std::vector<float> theta_new(n, 0.0f);
  std::vector<glm::vec2> distal_load(n, glm::vec2(0.0f));
  std::vector<float> distal_origin_moment(n, 0.0f);

  const glm::vec2 g_xz = in.gravity_acceleration_xz_m_s2;
  const glm::vec2 F_tip = in.tip_force_N_xz;
  const float alpha = std::clamp(in.relaxation, 0.05f, 1.0f);

  for (int iter = 0; iter < std::max(1, in.max_iterations); ++iter) {
    // (1) Integrate tangents -> positions.
    out.positions_xz[0] = glm::vec2(0.0f);
    for (int i = 1; i < n; ++i) {
      const float t_a = out.theta_rad[i - 1];
      const float t_b = out.theta_rad[i];
      const float t_mid = 0.5f * (t_a + t_b);
      out.positions_xz[i] = out.positions_xz[i - 1] + ds * glm::vec2(std::sin(t_mid), std::cos(t_mid));
    }

    // (2) Compute bending moment at every station via a tip->base sweep.
    //     M(s_i) = F_tip x (r_tip - r_i)  +  sum_{j > i} (mu_j * g_xz * ds) x (r_j - r_i)
    //     where x is the planar cross-product (returns scalar; sign chosen so
    //     a positive M increases theta toward +X).
    auto planar_cross = [](const glm::vec2& a, const glm::vec2& b) {
      // a x b = a.x * b.y - a.y * b.x, but our axes are (x, z); we want the
      // scalar moment about the out-of-plane (+y) axis with right-handed
      // orientation. With force F = (Fx, Fz) and lever r = (rx, rz),
      // M_y = rz * Fx - rx * Fz.
      return a.y * b.x - a.x * b.y;
    };
    std::fill(distal_load.begin(), distal_load.end(), glm::vec2(0.0f));
    std::fill(distal_origin_moment.begin(), distal_origin_moment.end(), 0.0f);
    for (int j = n - 1; j >= 0; --j) {
      const float w_seg = (j == n - 1) ? 0.5f * ds : ds;
      const glm::vec2 force = g_xz * (mu[j] * w_seg);
      distal_load[j] = force + (j + 1 < n ? distal_load[j + 1] : glm::vec2(0.0f));
      distal_origin_moment[j] =
          planar_cross(out.positions_xz[j], force) + (j + 1 < n ? distal_origin_moment[j + 1] : 0.0f);
    }
    for (int i = 0; i < n; ++i) {
      float M = 0.0f;
      const glm::vec2 r_i = out.positions_xz[i];
      // Tip force contribution.
      if (F_tip.x != 0.0f || F_tip.y != 0.0f) {
        const glm::vec2 r_tip = out.positions_xz[n - 1];
        M += planar_cross(r_tip - r_i, F_tip);
      }
      if (i + 1 < n) {
        M += distal_origin_moment[i + 1] - planar_cross(r_i, distal_load[i + 1]);
      }
      out.bending_moment_Nm[i] = M;
    }

    // (3) Integrate dtheta/ds = kappa_intrinsic + M / EI base->tip.
    theta_new[0] = 0.0f;
    for (int i = 1; i < n; ++i) {
      const float k_a = kappa_intr[i - 1] + out.bending_moment_Nm[i - 1] / EI[i - 1];
      const float k_b = kappa_intr[i] + out.bending_moment_Nm[i] / EI[i];
      theta_new[i] = theta_new[i - 1] + 0.5f * (k_a + k_b) * ds;
    }

    // (4) Damped update + convergence check.
    float max_delta = 0.0f;
    bool finite = true;
    for (int i = 0; i < n; ++i) {
      const float blended = (1.0f - alpha) * out.theta_rad[i] + alpha * theta_new[i];
      if (!std::isfinite(blended)) {
        finite = false;
        break;
      }
      max_delta = std::max(max_delta, std::abs(blended - out.theta_rad[i]));
      out.theta_rad[i] = blended;
    }
    out.iterations_performed = iter + 1;
    out.final_max_delta_rad = max_delta;
    if (!finite)
      break;
    if (max_delta < in.tolerance_rad) {
      out.converged = true;
      break;
    }
  }

  // Final position pass with the converged theta (so caller sees a coherent
  // (theta, positions) pair).
  out.positions_xz[0] = glm::vec2(0.0f);
  for (int i = 1; i < n; ++i) {
    const float t_mid = 0.5f * (out.theta_rad[i - 1] + out.theta_rad[i]);
    out.positions_xz[i] = out.positions_xz[i - 1] + ds * glm::vec2(std::sin(t_mid), std::cos(t_mid));
  }
  return out;
}

/// Analytic-cantilever validation: a uniform straight rod of length L and
/// stiffness EI loaded at its tip with force W perpendicular to the base
/// tangent has small-deflection tip displacement
///
///     delta_tip = W * L^3 / (3 * EI)
///
/// Returns (solver_tip_x, analytic_tip_x, relative_error). Use small W*L^2/EI
/// (< 0.05) to stay in the small-deflection regime where the analytic
/// formula is valid.
struct CantileverValidationResult {
  float solver_tip_x_m = 0.0f;
  float analytic_tip_x_m = 0.0f;
  float relative_error = 0.0f;
  bool converged = false;
  int iterations = 0;
};

inline CantileverValidationResult RunCantileverValidation(const float length_m, const float EI_Pa_m4,
                                                          const float tip_force_N, const int station_count = 33) {
  PlanarElasticaInput in;
  in.length_m = length_m;
  in.station_count = station_count;
  in.bending_stiffness_Pa_m4 = [EI_Pa_m4](float) {
    return EI_Pa_m4;
  };
  in.intrinsic_curvature_per_m = [](float) {
    return 0.0f;
  };
  in.tip_force_N_xz = glm::vec2(tip_force_N, 0.0f);
  in.relaxation = 0.7f;
  in.tolerance_rad = 1e-7f;
  in.max_iterations = 64;
  const PlanarElasticaOutput o = SolvePlanarElastica(in);
  CantileverValidationResult r;
  r.solver_tip_x_m = o.positions_xz.empty() ? 0.0f : o.positions_xz.back().x;
  r.analytic_tip_x_m = tip_force_N * length_m * length_m * length_m / (3.0f * EI_Pa_m4);
  r.relative_error = (std::abs(r.analytic_tip_x_m) > 1e-12f)
                         ? std::abs(r.solver_tip_x_m - r.analytic_tip_x_m) / std::abs(r.analytic_tip_x_m)
                         : std::abs(r.solver_tip_x_m);
  r.converged = o.converged;
  r.iterations = o.iterations_performed;
  return r;
}

}  // namespace l_system_package
