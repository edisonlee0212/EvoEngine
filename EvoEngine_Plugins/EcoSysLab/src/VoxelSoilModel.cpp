#include "VoxelSoilModel.hpp"

#include <cassert>
#include <iostream>

#include <fstream>
#include <glm/gtx/string_cast.hpp>

using namespace eco_sys_lab;
using namespace std;
using namespace glm;

/* Coordinate system

The voxel position is its center.
Each voxel is dx wide.

                                <-dx ->
                                -------------------------
                                |     |     |     |     |
                                |  x  |  x  |  x  |  x  |
                                |     |     |     |     |
                                -------------------------
                                   |     |     |     |
                                   |     |     |     |
X-Coordinate:   -- 0 --- 1 --- 2 --- 3 -----

The 'm_volumePositionMin' stores the lower left corner of the lower left voxel.
I.e. for m_volumePositionMin=(0, 0) and m_resolution=(2, 2), and m_dx=1,
the voxel centers are at 0.5 and 1.5.

*/

void VoxelSoilModel::Initialize(const SoilParameters& p, const SoilSurface& soilSurface,
                                const std::vector<SoilLayer>& soilLayers) {
  // Update version so the debug visualization is also updated.
  m_version++;
  m_diffusionForce = p.m_diffusionForce;
  m_gravityForce = p.m_gravityForce;
  m_nutrientForce = p.m_nutrientForce;
  m_dt = p.m_deltaTime;
  m_time_since_start_in_hrs = 0.f;
  m_time_since_start_requested = 0.f;

  m_resolution = p.m_voxelResolution;
  m_dx = p.m_deltaX;
  m_boundingBoxMin = p.m_boundingBoxMin;

  m_voxel_volume_in_cm3 = (m_dx * m_dx * m_dx) / 1e-6;
  m_water_g_per_cm3 = 1;
  m_nutrient_unit_per_cm3 = 1;

  m_boundary_x = p.m_boundary_x;
  m_boundary_y = p.m_boundary_y;
  m_boundary_z = p.m_boundary_z;

  m_water_sources.clear();
  m_nutrient_sources.clear();

  m_blur_3x3_idx = vector<ivec3>({
      {-1, -1, -1}, {0, -1, -1}, {1, -1, -1},

      {-1, 0, -1},  {0, 0, -1},  {1, 0, -1},

      {-1, 1, -1},  {0, 1, -1},  {1, 1, -1},

      {-1, -1, 0},  {0, -1, 0},  {1, -1, 0},

      {-1, 0, 0},   {0, 0, 0},   {1, 0, 0},

      {-1, 1, 0},   {0, 1, 0},   {1, 1, 0},

      {-1, -1, 1},  {0, -1, 1},  {1, -1, 1},

      {-1, 0, 1},   {0, 0, 1},   {1, 0, 1},

      {-1, 1, 1},   {0, 1, 1},   {1, 1, 1},
  });

  m_blur_3x3_weights = vector<float>({0.009188900331780544, 0.025493013475061985, 0.009188900331780544,

                                      0.025493013475061978, 0.0707259533321939,   0.025493013475061978,

                                      0.009188900331780544, 0.025493013475061985, 0.009188900331780544,

                                      0.025493013475061978, 0.0707259533321939,   0.025493013475061978,

                                      0.0707259533321939,   0.19621691565184837,  0.0707259533321939,

                                      0.025493013475061978, 0.0707259533321939,   0.025493013475061978,

                                      0.009188900331780544, 0.025493013475061985, 0.009188900331780544,

                                      0.025493013475061978, 0.0707259533321939,   0.025493013475061978,

                                      0.009188900331780544, 0.025493013475061985, 0.009188900331780544});

  m_initialized = true;

  auto numVoxels = m_resolution.x * m_resolution.y * m_resolution.z;

  m_rnd = std::mt19937(std::random_device()());

  m_material_id.resize(numVoxels);
  m_material_id = -1;

  // intermediate variables
  auto empty = Field(numVoxels);
  empty = 0.f;

  m_d = empty;
  // also to initialize their size
  m_w = empty;
  m_n = empty;

  m_w_grad_x = empty;
  m_w_grad_y = empty;
  m_w_grad_z = empty;

  m_div_diff_x = empty;
  m_div_diff_y = empty;
  m_div_diff_z = empty;

  m_div_diff_n_x = empty;
  m_div_diff_n_y = empty;
  m_div_diff_n_z = empty;

  m_div_grav_x = empty;
  m_div_grav_y = empty;
  m_div_grav_z = empty;

  m_div_grav_n_x = empty;
  m_div_grav_n_y = empty;
  m_div_grav_n_z = empty;

  m_l = empty;

  // Capacity
  m_c = Field(numVoxels);  // initialize with 1s
  m_c = 1.0f;

  // SetField(m_c, vec3(-10, -3, -10), vec3(10, 0, 10), 2);
  // BlurField(m_c);

  // ChangeField(m_c, vec3(0, 0, 0),   5000, 3);
  // ChangeField(m_c, vec3(-1, 0, -1), 5000, 3);
  // ChangeField(m_c, vec3(1, 0, -1),  5000, 3);
  // ChangeField(m_c, vec3(-1, 0, 1),  5000, 3);
  // ChangeField(m_c, vec3(1, 0, 1),   5000, 3);

  // Permeability
  m_p = Field(numVoxels);  // initialize with 1s
  m_p = 0.0f;

  // SetField(m_p, vec3(-6.4, -5, -6.4), vec3(6.4, 1, 6.4), 0.5f);
  SetField(m_p, vec3(-2, -5, -6.4), vec3(0, 1, 6.4), 0.0f);
  BlurField(m_p);
  // BlurField(m_p);

  m_soilSurface = soilSurface;
  m_soilLayers = soilLayers;

  BuildFromLayers();

  Reset();
}

void eco_sys_lab::VoxelSoilModel::BuildFromLayers() {
  auto rain_field = Field(m_w.size());
  rain_field = 0.f;
  // Height axis is Y:

  for (auto x = 0; x < m_resolution.x; ++x) {
    for (auto z = 0; z < m_resolution.z; ++z) {
      auto pos = GetPositionFromCoordinate({x, 0, z});  // y value does not matter
      vec2 pos_2d(pos.x, pos.z);
      auto groundHeight = glm::clamp(m_soilSurface.m_height({pos.x, pos.z}), m_boundingBoxMin.y,
                                     m_boundingBoxMin.y + m_resolution.y * m_dx);
      pos.y = groundHeight;

      // insert water 2 voxels below ground
      auto rain_coord = GetCoordinateFromPosition(pos) + ivec3(0, -1, 0);
      if (CoordinateInsideVolume(rain_coord))
        rain_field[Index(rain_coord)] = 0.1;

      for (auto y = m_resolution.y - 1; y >= 0; --y) {
        float current_height = groundHeight;
        auto voxel_height = m_boundingBoxMin.y + y * m_dx + (m_dx / 2.0);

        // find material index:
        auto idx = 0;
        while (voxel_height < current_height && idx < m_soilLayers.size() - 1) {
          idx++;
          current_height -= glm::max(0.f, m_soilLayers[idx].m_thickness(pos_2d));
        }

        SetVoxel({x, y, z}, m_soilLayers[idx].m_mat);
      }
    }
  }

  // blur everything to make it smooth
  BlurField(m_c);
  BlurField(m_p);
  BlurField(m_d);

  Source rain_source;
  BlurField(rain_field);
  // BlurField(rain_field);
  for (auto i = 0; i < rain_field.size(); ++i) {
    if (rain_field[i] > 0) {
      rain_source.idx.push_back(i);
      rain_source.amounts.push_back(rain_field[i]);
    }
  }
  AddWaterSource(move(rain_source));
}

void VoxelSoilModel::Reset() {
  assert(m_initialized);

  m_time_since_start_in_hrs = 0.f;
  m_time_since_start_requested = 0.f;

  // why not reset water here? what is the purpose of this function now??

  // Water
  // m_w = 0.f;
  // Nutrients
  // m_n = 0.f;
  // SetField(m_n, vec3(-10, -3, -10), vec3(10, 0, 10), 2);
  // BlurField(m_n);
  // create some nutrients
  // ChangeNutrient(vec3(0,0,0),  20000, 4);
  // ChangeNutrient(vec3(1,3,0),   5000, 3);
  // ChangeNutrient(vec3(-5,-3,1), 10500, 5);

  UpdateStats();

  m_version++;
}

void VoxelSoilModel::Convolution3(const Field& input, Field& output, const vector<int>& indices,
                                  const vector<float>& weights) const {
  auto entries = m_resolution.x * m_resolution.y * m_resolution.z;
  assert(input.size() == entries);
  assert(output.size() == entries);
  assert(indices.size() == weights.size());

  // for a 3D convolution:
  assert(m_resolution.x >= 3);
  assert(m_resolution.y >= 3);
  assert(m_resolution.z >= 3);

  // iterate over all indices that are not part of the boundary, where the whole convolution kernel can be applied
  for (auto x = 1; x < m_resolution.x - 1; ++x) {
    for (auto y = 1; y < m_resolution.y - 1; ++y) {
      for (auto z = 1; z < m_resolution.z - 1; ++z) {
        auto i = Index(x, y, z);
        output[i] = 0.f;
        for (auto j = 0; j < indices.size(); ++j) {
          output[i] += input[i + indices[j]] * weights[j];
        }
      }
    }
  }
}

void eco_sys_lab::VoxelSoilModel::Boundary_Wrap_Axis(const Field& input, Field& output,
                                                     const std::vector<int>& indices_1D,
                                                     const std::vector<float>& weights, int lim_a, int lim_b, int lim_f,
                                                     std::function<int(int, int, int)> WrapIndex) const {
  for (int a = 0; a < lim_a; ++a) {
    for (int b = 0; b < lim_b; ++b) {
      output[WrapIndex(a, b, 0)] = 0;
      output[WrapIndex(a, b, lim_f - 1)] = 0;

      for (auto i = 0u; i < indices_1D.size(); ++i) {
        auto idx = indices_1D[i];
        auto w = weights[i];
        if (idx < 0) {
          output[WrapIndex(a, b, 0)] += input[WrapIndex(a, b, lim_f + idx)] * w;
          output[WrapIndex(a, b, lim_f - 1)] += input[WrapIndex(a, b, lim_f - 1 + idx)] * w;
        } else if (idx > 0) {
          output[WrapIndex(a, b, 0)] += input[WrapIndex(a, b, idx)] * w;
          output[WrapIndex(a, b, lim_f - 1)] += input[WrapIndex(a, b, idx - 1)] * w;
        } else {
          output[WrapIndex(a, b, 0)] += input[WrapIndex(a, b, 0)] * w;
          output[WrapIndex(a, b, lim_f - 1)] += input[WrapIndex(a, b, lim_f - 1)] * w;
        }
      }
    }
  }
}

void eco_sys_lab::VoxelSoilModel::Boundary_Wrap_X(const Field& input, Field& output, const std::vector<int>& indices_1D,
                                                  const std::vector<float>& weights) const {
  auto WrapIndex = [&](int a, int b, int f) {
    return Index(f, a, b);
  };

  Boundary_Wrap_Axis(input, output, indices_1D, weights, m_resolution.y, m_resolution.z, m_resolution.x, WrapIndex);
}

void eco_sys_lab::VoxelSoilModel::Boundary_Wrap_Y(const Field& input, Field& output, const std::vector<int>& indices_1D,
                                                  const std::vector<float>& weights) const {
  auto WrapIndex = [&](int a, int b, int f) {
    return Index(a, f, b);
  };

  Boundary_Wrap_Axis(input, output, indices_1D, weights, m_resolution.x, m_resolution.z, m_resolution.y, WrapIndex);
}

void eco_sys_lab::VoxelSoilModel::Boundary_Wrap_Z(const Field& input, Field& output, const std::vector<int>& indices_1D,
                                                  const std::vector<float>& weights) const {
  auto WrapIndex = [&](int a, int b, int f) {
    return Index(a, b, f);
  };

  Boundary_Wrap_Axis(input, output, indices_1D, weights, m_resolution.x, m_resolution.y, m_resolution.z, WrapIndex);
}

void eco_sys_lab::VoxelSoilModel::Boundary_Barrier_Axis(const Field& input, Field& output,
                                                        const std::vector<int>& indices_1D,
                                                        const std::vector<float>& weights, int lim_a, int lim_b,
                                                        int lim_f, std::function<int(int, int, int)> WrapIndex) const {
  /*
  Out of bonds indices (v[-1] etc.) are undefined. However, using the mirror method, we can substitute them:

  v[-1] == v[0]
  v[-2] == v[1]

  v[lim]   == v[lim-1]
  v[lim+1] == v[lim-2]

  */

  for (int a = 0; a < lim_a; ++a) {
    for (int b = 0; b < lim_b; ++b) {
      output[WrapIndex(a, b, 0)] = 0;
      output[WrapIndex(a, b, lim_f - 1)] = 0;

      for (auto i = 0u; i < indices_1D.size(); ++i) {
        auto idx = indices_1D[i];
        auto w = weights[i];
        if (idx < 0) {
          output[WrapIndex(a, b, 0)] += input[WrapIndex(a, b, -idx - 1)] * w;
          output[WrapIndex(a, b, lim_f - 1)] += input[WrapIndex(a, b, lim_f - 1 + idx)] * w;
        } else if (idx > 0) {
          output[WrapIndex(a, b, 0)] += input[WrapIndex(a, b, idx)] * w;
          output[WrapIndex(a, b, lim_f - 1)] += input[WrapIndex(a, b, lim_f - idx)] * w;
        } else {
          output[WrapIndex(a, b, 0)] += input[WrapIndex(a, b, 0)] * w;
          output[WrapIndex(a, b, lim_f - 1)] += input[WrapIndex(a, b, lim_f - 1)] * w;
        }
      }
    }
  }
}

void eco_sys_lab::VoxelSoilModel::Boundary_Barrier_X(const Field& input, Field& output,
                                                     const std::vector<int>& indices_1D,
                                                     const std::vector<float>& weights) const {
  auto WrapIndex = [&](int a, int b, int f) {
    return Index(f, a, b);
  };

  Boundary_Barrier_Axis(input, output, indices_1D, weights, m_resolution.y, m_resolution.z, m_resolution.x, WrapIndex);
}

void eco_sys_lab::VoxelSoilModel::Boundary_Barrier_Y(const Field& input, Field& output,
                                                     const std::vector<int>& indices_1D,
                                                     const std::vector<float>& weights) const {
  auto WrapIndex = [&](int a, int b, int f) {
    return Index(a, f, b);
  };

  Boundary_Barrier_Axis(input, output, indices_1D, weights, m_resolution.x, m_resolution.z, m_resolution.y, WrapIndex);
}

void eco_sys_lab::VoxelSoilModel::Boundary_Barrier_Z(const Field& input, Field& output,
                                                     const std::vector<int>& indices_1D,
                                                     const std::vector<float>& weights) const {
  auto WrapIndex = [&](int a, int b, int f) {
    return Index(a, b, f);
  };

  Boundary_Barrier_Axis(input, output, indices_1D, weights, m_resolution.x, m_resolution.y, m_resolution.z, WrapIndex);
}

void eco_sys_lab::VoxelSoilModel::AddWaterSource(Source&& source) {
  m_water_sources.emplace_back(source);
}

void eco_sys_lab::VoxelSoilModel::AddNutrientSource(Source&& source) {
  m_nutrient_sources.emplace_back(source);
}

bool VoxelSoilModel::Initialized() const {
  return m_initialized;
}

float VoxelSoilModel::GetTime() const {
  return m_time_since_start_in_hrs;
}

float AbsorptionValueGaussian(int region_width, int border_distance) {
  auto a = exp(-((float)border_distance * (float)border_distance) / (0.2 * (float)region_width * (float)region_width));
  return 1 - a;
}

void VoxelSoilModel::Step() {
  assert(m_initialized);

  const auto num_voxels = m_w.size();
  Field tmp(num_voxels);

  const auto grad_index_x = vector<int>({
      Index(-1, 0, 0),
      Index(+1, 0, 0),
  });
  const auto grad_index_y = vector<int>({
      Index(0, -1, 0),
      Index(0, +1, 0),
  });
  const auto grad_index_z = vector<int>({
      Index(0, 0, -1),
      Index(0, 0, +1),
  });
  const auto grad_index_1D = vector<int>({-1, 1});

  // ----------------- diffusion -----------------
  {
    if (true) {
      m_l = m_w / m_c;
    } else  // don't use capacity forces
      m_l = m_w;

    const auto wx_d = 1.0f / (2.0f * m_dx);
    const auto grad_weights = vector<float>({-wx_d, wx_d});

    // compute gradient dw
    Convolution3(m_l, m_w_grad_x, grad_index_x, grad_weights);
    Convolution3(m_l, m_w_grad_y, grad_index_y, grad_weights);
    Convolution3(m_l, m_w_grad_z, grad_index_z, grad_weights);

    if (Boundary::wrap == m_boundary_x)
      Boundary_Wrap_X(m_l, m_w_grad_x, grad_index_1D, grad_weights);
    if (Boundary::block == m_boundary_x)
      Boundary_Barrier_X(m_l, m_w_grad_x, grad_index_1D, grad_weights);

    if (Boundary::wrap == m_boundary_y)
      Boundary_Wrap_Y(m_l, m_w_grad_y, grad_index_1D, grad_weights);
    if (Boundary::block == m_boundary_y)
      Boundary_Barrier_Y(m_l, m_w_grad_y, grad_index_1D, grad_weights);

    if (Boundary::wrap == m_boundary_z)
      Boundary_Wrap_Z(m_l, m_w_grad_z, grad_index_1D, grad_weights);
    if (Boundary::block == m_boundary_z)
      Boundary_Barrier_Z(m_l, m_w_grad_z, grad_index_1D, grad_weights);

    // apply effect of permeability
    // it must be applied after computing the gradient, since it is inhomogeneous!
    m_w_grad_x *= m_p;
    m_w_grad_y *= m_p;
    m_w_grad_z *= m_p;

    // compute divergence
    Convolution3(m_w_grad_x, m_div_diff_x, grad_index_x, grad_weights);
    Convolution3(m_w_grad_y, m_div_diff_y, grad_index_y, grad_weights);
    Convolution3(m_w_grad_z, m_div_diff_z, grad_index_z, grad_weights);

    if (Boundary::wrap == m_boundary_x)
      Boundary_Wrap_X(m_w_grad_x, m_div_diff_x, grad_index_1D, grad_weights);
    if (Boundary::block == m_boundary_x)
      Boundary_Barrier_X(m_w_grad_x, m_div_diff_x, grad_index_1D, grad_weights);

    if (Boundary::wrap == m_boundary_y)
      Boundary_Wrap_Y(m_w_grad_y, m_div_diff_y, grad_index_1D, grad_weights);
    if (Boundary::block == m_boundary_y)
      Boundary_Barrier_Y(m_w_grad_y, m_div_diff_y, grad_index_1D, grad_weights);

    if (Boundary::wrap == m_boundary_z)
      Boundary_Wrap_Y(m_w_grad_z, m_div_diff_z, grad_index_1D, grad_weights);
    if (Boundary::block == m_boundary_z)
      Boundary_Barrier_Z(m_w_grad_z, m_div_diff_z, grad_index_1D, grad_weights);

    // divergence for nutrients
    tmp = m_w_grad_x * m_diffusionForce * m_n;
    Convolution3(tmp, m_div_diff_n_x, grad_index_x, grad_weights);
    if (Boundary::wrap == m_boundary_x)
      Boundary_Wrap_X(tmp, m_div_diff_n_x, grad_index_1D, grad_weights);
    if (Boundary::block == m_boundary_x)
      Boundary_Barrier_X(tmp, m_div_diff_n_x, grad_index_1D, grad_weights);

    tmp = m_w_grad_y * m_diffusionForce * m_n;
    Convolution3(tmp, m_div_diff_n_y, grad_index_y, grad_weights);
    if (Boundary::wrap == m_boundary_y)
      Boundary_Wrap_Y(tmp, m_div_diff_n_y, grad_index_1D, grad_weights);
    if (Boundary::block == m_boundary_y)
      Boundary_Barrier_Y(tmp, m_div_diff_n_y, grad_index_1D, grad_weights);

    tmp = m_w_grad_z * m_diffusionForce * m_n;
    Convolution3(tmp, m_div_diff_n_z, grad_index_z, grad_weights);
    if (Boundary::wrap == m_boundary_z)
      Boundary_Wrap_Z(tmp, m_div_diff_n_z, grad_index_1D, grad_weights);
    if (Boundary::block == m_boundary_z)
      Boundary_Barrier_Z(tmp, m_div_diff_n_z, grad_index_1D, grad_weights);

    m_div_diff_x *= m_diffusionForce;
    m_div_diff_y *= m_diffusionForce;
    m_div_diff_z *= m_diffusionForce;
  }

  // ------------ gravity ------------

  auto wp = m_w * m_p;
  auto wpn = wp * m_n;

  // TODO: the weights are computed from the gravity force. however this is inhomogeneously altered by the permeability.
  // A better integration scheme is required that accounts for this and is still stable.

  // X direction:
  {
    auto a_x = m_gravityForce.x;
    auto wx = a_x * 1.f / (2.f * m_dx);
    auto theta = (a_x * m_dt / m_dx) * (a_x * m_dt / m_dx);
    auto wt = theta * 1 / (2 * m_dt);

    const auto idx = vector<int>({
        Index(1, 0, 0),
        Index(-1, 0, 0),
        Index(1, 0, 0),
        Index(0, 0, 0),
        Index(-1, 0, 0),
    });
    const auto idx_1D = vector<int>({1, -1, 1, 0, -1});

    const auto weights = vector<float>({-wx, wx, wt, -2 * wt, wt});

    Convolution3(wp, m_div_grav_x, idx, weights);
    if (Boundary::wrap == m_boundary_x)
      Boundary_Wrap_X(wp, m_div_grav_x, idx_1D, weights);
    if (Boundary::block == m_boundary_x)
      Boundary_Barrier_X(wp, m_div_grav_x, idx_1D, weights);

    // gravity force on nutrients
    Convolution3(wpn, m_div_grav_n_x, idx, weights);
    if (Boundary::wrap == m_boundary_x)
      Boundary_Wrap_X(wpn, m_div_grav_n_x, idx_1D, weights);
    if (Boundary::block == m_boundary_x)
      Boundary_Barrier_X(wpn, m_div_grav_n_x, idx_1D, weights);
  }

  // Y direction:
  {
    auto a_y = m_gravityForce.y;
    auto wx = a_y * 1.f / (2.f * m_dx);
    auto theta = (a_y * m_dt / m_dx) * (a_y * m_dt / m_dx);
    auto wt = theta * 1 / (2 * m_dt);

    const auto idx = vector<int>({
        Index(0, 1, 0),
        Index(0, -1, 0),
        Index(0, 1, 0),
        Index(0, 0, 0),
        Index(0, -1, 0),
    });
    const auto idx_1D = vector<int>({1, -1, 1, 0, -1});

    const auto weights = vector<float>({-wx, wx, wt, -2 * wt, wt});

    Convolution3(wp, m_div_grav_y, idx, weights);
    if (Boundary::wrap == m_boundary_y)
      Boundary_Wrap_Y(wp, m_div_grav_y, idx_1D, weights);
    if (Boundary::block == m_boundary_y)
      Boundary_Barrier_Y(wp, m_div_grav_y, idx_1D, weights);

    // gravity force on nutrients
    Convolution3(wpn, m_div_grav_n_y, idx, weights);
    if (Boundary::wrap == m_boundary_y)
      Boundary_Wrap_Y(wpn, m_div_grav_n_y, idx_1D, weights);
    if (Boundary::block == m_boundary_y)
      Boundary_Barrier_Y(wpn, m_div_grav_n_y, idx_1D, weights);
  }

  // Z direction:
  {
    auto a_z = m_gravityForce.z;
    auto wx = a_z * 1.f / (2.f * m_dx);
    auto theta = (a_z * m_dt / m_dx) * (a_z * m_dt / m_dx);
    auto wt = theta * 1 / (2 * m_dt);

    const auto idx = vector<int>({
        Index(0, 0, 1),
        Index(0, 0, -1),
        Index(0, 0, 1),
        Index(0, 0, 0),
        Index(0, 0, -1),
    });
    const auto idx_1D = vector<int>({1, -1, 1, 0, -1});

    const auto weights = vector<float>({-wx, wx, wt, -2 * wt, wt});

    Convolution3(wp, m_div_grav_z, idx, weights);
    if (Boundary::wrap == m_boundary_z)
      Boundary_Wrap_Z(wp, m_div_grav_z, idx_1D, weights);
    if (Boundary::block == m_boundary_z)
      Boundary_Barrier_Z(wp, m_div_grav_z, idx_1D, weights);

    // gravity force on nutrients
    Convolution3(wpn, m_div_grav_n_z, idx, weights);
    if (Boundary::wrap == m_boundary_z)
      Boundary_Wrap_Z(wpn, m_div_grav_n_z, idx_1D, weights);
    if (Boundary::block == m_boundary_z)
      Boundary_Barrier_Z(wpn, m_div_grav_n_z, idx_1D, weights);
  }

  // apply all the fluxes:
  for (auto i = 0; i < num_voxels; ++i) {
    auto divergence =
        (m_div_diff_x[i] + m_div_diff_y[i] + m_div_diff_z[i]) + (m_div_grav_x[i] + m_div_grav_y[i] + m_div_grav_z[i]);
    // ToDo: Also apply source terms here
    m_w[i] += m_dt * divergence;

    // update nutrients:
    auto divergence_nut = (m_div_diff_n_x[i] + m_div_diff_n_y[i] + m_div_diff_n_z[i]) +
                          (m_div_grav_n_x[i] + m_div_grav_n_y[i] + m_div_grav_n_z[i]);
    m_n[i] += m_dt * divergence_nut * m_nutrientForce;
  }

  // absorbing boundary regions
  int region_width = 5;

  if (Boundary::absorb == m_boundary_x) {
    for (auto i = 0; i < region_width; ++i) {
      auto a = AbsorptionValueGaussian(region_width, i);
      for (auto y = 0u; y < m_resolution.y; ++y) {
        for (auto z = 0u; z < m_resolution.z; ++z) {
          m_w[Index(i, y, z)] *= a;
          m_w[Index(m_resolution.x - 1 - i, y, z)] *= a;
        }
      }
    }
  }

  if (Boundary::absorb == m_boundary_y) {
    for (auto i = 0; i < region_width; ++i) {
      auto a = AbsorptionValueGaussian(region_width, i);
      for (auto x = 0u; x < m_resolution.x; ++x) {
        for (auto z = 0u; z < m_resolution.z; ++z) {
          m_w[Index(x, i, z)] *= a;
          m_w[Index(x, m_resolution.y - 1 - i, z)] *= a;
        }
      }
    }
  }

  if (Boundary::absorb == m_boundary_z) {
    for (auto i = 0; i < region_width; ++i) {
      auto a = AbsorptionValueGaussian(region_width, i);
      for (auto x = 0u; x < m_resolution.x; ++x) {
        for (auto y = 0u; y < m_resolution.y; ++y) {
          m_w[Index(x, y, i)] *= a;
          m_w[Index(x, y, m_resolution.z - 1 - i)] *= a;
        }
      }
    }
  }

  m_time_since_start_in_hrs += m_dt;

  m_version++;
}

void eco_sys_lab::VoxelSoilModel::Irrigation() {
  // ChangeWater(vec3(0, 2, 0), m_irrigationAmount, 0.5);

  for (auto& s : m_water_sources)
    s.Apply(m_w);
  for (auto& s : m_nutrient_sources)
    s.Apply(m_n);
  /*
          m_rnd = std::mt19937(27);

          auto bb_min = GetBoundingBoxMin();
          auto bb_max = GetBoundingBoxMax();
          std::uniform_real_distribution<> dist_x(bb_min.x, bb_max.x);
          std::uniform_real_distribution<> dist_y(bb_min.y, bb_max.y);
          std::uniform_real_distribution<> dist_z(bb_min.z, bb_max.z);

          std::uniform_real_distribution<> width(0.5, 2);



          for(auto i=0; i<30; ++i)
          {
                  auto pos = vec3(dist_x(m_rnd), dist_y(m_rnd), dist_z(m_rnd));
                  auto amount = m_irrigationAmount * GetDensity(pos);
                  ChangeWater(pos, amount, width(m_rnd));
          }

          if ((int)(m_time / 20.0) % 2 == 0)
          {
                  ChangeWater(vec3(-18, 0, 15), 10, 8);
          }
          if ((int)((m_time + 5) / 19.0) % 2 == 0)
          {
                  ChangeWater(vec3(0, 20, -15), -10, 15);
          }*/
}

float VoxelSoilModel::IntegrateWater(const glm::vec3& position, float width) const {
  auto water_in_cm3 = IntegrateFieldValue(m_w, position, width);
  return water_in_cm3 * m_water_g_per_cm3;
}

float VoxelSoilModel::GetWaterDensity(const vec3& position) const {
  return GetField(m_w, position, 0.0f);
}

float VoxelSoilModel::IntegrateNutrient(const vec3& position, float width) const {
  auto nutrient_in_cm3 = IntegrateFieldValue(m_n, position, width);
  return nutrient_in_cm3 * m_nutrient_unit_per_cm3;
}

float VoxelSoilModel::GetNutrientDensity(const vec3& position) const {
  return GetField(m_n, position, 0.0f);
}

void eco_sys_lab::VoxelSoilModel::Run(float t_in_hrs) {
  m_time_since_start_requested += t_in_hrs;
  while (m_time_since_start_requested - m_time_since_start_in_hrs >= m_dt) {
    Irrigation();
    Step();
  }
}

float VoxelSoilModel::GetDensity(const vec3& position) const {
  return GetField(m_d, position, 1000.0f);
}

float eco_sys_lab::VoxelSoilModel::GetCapacity(const glm::vec3& position) const {
  return GetField(m_c, position, 1.0f);
}

void eco_sys_lab::VoxelSoilModel::SetVoxel(const glm::ivec3& coordinate, const SoilPhysicalMaterial& material) {
  auto idx = Index(coordinate.x, coordinate.y, coordinate.z);
  auto position = GetPositionFromCoordinate({coordinate.x, coordinate.y, coordinate.z});
  m_material_id[idx] = material.m_id;
  m_c[idx] = material.m_c(position);
  m_p[idx] = material.m_p(position);
  m_d[idx] = material.m_d(position);

  m_n[idx] = material.m_n(position);
  m_w[idx] = material.m_w(position);
}

float eco_sys_lab::VoxelSoilModel::GetField(const Field& field, const glm::vec3& position, float default_value) const {
  if (!PositionInsideVolume(position))
    return default_value;
  return field[Index(GetCoordinateFromPosition(position))];
}

void VoxelSoilModel::ChangeField(Field& field, const vec3& center, float amount_in_cm3, float width_in_m) {
  // TODO: Remove Code DUPLICATION!!! (IntegrateFieldValue)
  width_in_m /= 3.0;  // seems ok :D
  auto cutoff = 3.0;  // how much of the gaussian to keep

  auto voxel_min = GetCoordinateFromPosition(center - vec3(width_in_m * cutoff)) - ivec3(2);
  auto voxel_max = GetCoordinateFromPosition(center + vec3(width_in_m * cutoff)) + ivec3(2);

  voxel_min = glm::max(voxel_min, ivec3(0));
  voxel_max = glm::min(voxel_max, static_cast<ivec3>(m_resolution) - ivec3(1));

  // the <= is important here
  float volume_of_gaussian_in_cm3 = 0.f;  // count the weighted number of voxels and multiply by volume of 1 voxel
  for (auto z = voxel_min.z; z <= voxel_max.z; ++z) {
    for (auto y = voxel_min.y; y <= voxel_max.y; ++y) {
      for (auto x = voxel_min.x; x <= voxel_max.x; ++x) {
        auto pos = GetPositionFromCoordinate({x, y, z});
        auto l = glm::length(pos - center);
        auto v = glm::exp(-l * l / (2 * width_in_m * width_in_m));

        volume_of_gaussian_in_cm3 += v;
      }
    }
  }
  volume_of_gaussian_in_cm3 *= m_voxel_volume_in_cm3;

  for (auto z = voxel_min.z; z <= voxel_max.z; ++z) {
    for (auto y = voxel_min.y; y <= voxel_max.y; ++y) {
      for (auto x = voxel_min.x; x <= voxel_max.x; ++x) {
        auto pos = GetPositionFromCoordinate({x, y, z});
        auto l = glm::length(pos - center);
        auto v = glm::exp(-l * l / (2 * width_in_m * width_in_m));

        field[Index(x, y, z)] += v * amount_in_cm3 / volume_of_gaussian_in_cm3;
      }
    }
  }
}

float VoxelSoilModel::IntegrateFieldValue(const Field& field, const vec3& center, float width_in_m) const {
  float result = 0.f;

  // TODO: Remove Code DUPLICATION!!! (ChangeField)
  width_in_m /= 3.0;  // seems ok :D
  auto cutoff = 3.0;  // how much of the gaussian to keep

  auto voxel_min = GetCoordinateFromPosition(center - vec3(width_in_m * cutoff)) - ivec3(2);
  auto voxel_max = GetCoordinateFromPosition(center + vec3(width_in_m * cutoff)) + ivec3(2);

  voxel_min = glm::max(voxel_min, ivec3(0));
  voxel_max = glm::min(voxel_max, static_cast<ivec3>(m_resolution) - ivec3(1));

  float sum = glm::exp(0.f);  // center voxel has weight 1, all others have a lower value
  // the <= is important here
  for (auto z = voxel_min.z; z <= voxel_max.z; ++z) {
    for (auto y = voxel_min.y; y <= voxel_max.y; ++y) {
      for (auto x = voxel_min.x; x <= voxel_max.x; ++x) {
        auto pos = GetPositionFromCoordinate({x, y, z});
        auto l = glm::length(pos - center);
        auto v = glm::exp(-l * l / (2 * width_in_m * width_in_m));

        result += v * field[Index(x, y, z)] / sum;
      }
    }
  }

  return result * m_voxel_volume_in_cm3;
}

void eco_sys_lab::VoxelSoilModel::SetField(Field& field, const vec3& bb_min, const vec3& bb_max, float value) {
  auto idx_min = GetCoordinateFromPosition(bb_min);
  auto idx_max = GetCoordinateFromPosition(bb_max);
  idx_min = glm::clamp(idx_min, ivec3(0, 0, 0), m_resolution);
  idx_max = glm::clamp(idx_max, ivec3(0, 0, 0), m_resolution);

  for (int z = idx_min.z; z < idx_max.z; ++z) {
    for (int y = idx_min.y; y < idx_max.y; ++y) {
      for (int x = idx_min.x; x < idx_max.x; ++x) {
        field[Index(x, y, z)] = value;
      }
    }
  }
}

void eco_sys_lab::VoxelSoilModel::BlurField(Field& field) {
  // this will ignore corners and introduce artifacts
  /*
  Convolution3(field, tmp, m_blur_3x3_idx, m_blur_3x3_weights);
  Convolution3(tmp, field, m_blur_3x3_idx, m_blur_3x3_weights);
  */

  Field tmp(field.size());

  for (int z = 0; z < m_resolution.z; ++z) {
    for (int y = 0; y < m_resolution.y; ++y) {
      for (int x = 0; x < m_resolution.x; ++x) {
        // iterate over the blur kernel, ignore pixels that are out of the field.
        float total_weight = 0.0f;
        tmp[Index(x, y, z)] = 0.0f;

        for (int i = 0; i < m_blur_3x3_idx.size(); ++i) {
          ivec3 idx = ivec3(x, y, z) + m_blur_3x3_idx[i];
          // method 1: Ignore outlier
          /*
          {
                  if(    idx.x>=0 && idx.x<m_resolution.x
                          && idx.y>=0 && idx.y<m_resolution.y
                          && idx.z>=0 && idx.z<m_resolution.z)
                  {
                          total_weight += m_blur_3x3_weights[i];
                          tmp[Index(x, y, z)] += field[Index(idx)] * m_blur_3x3_weights[i];
                  }
          }*/

          // Method 2: clamp outlier
          {
            // cout << "before " << to_string(idx) << endl;
            idx = glm::clamp(idx, ivec3(0, 0, 0), m_resolution - ivec3(1));
            // cout << "after  " << to_string(idx) << endl;
            total_weight += m_blur_3x3_weights[i];
            tmp[Index(x, y, z)] += field[Index(idx)] * m_blur_3x3_weights[i];
          }
        }
        tmp[Index(x, y, z)] /= total_weight;
      }
    }
  }
  field = tmp;
}

void VoxelSoilModel::ChangeWater(const vec3& center, float amount_in_g, float width) {
  auto amount_in_cm3 = amount_in_g / m_water_g_per_cm3;
  ChangeField(m_w, center, amount_in_cm3, width);
}

void VoxelSoilModel::ChangeDensity(const vec3& center, float amount, float width) {
  ChangeField(m_d, center, amount, width);
}

void VoxelSoilModel::ChangeNutrient(const vec3& center, float amount_in_AU, float width) {
  auto amount_in_cm3 = amount_in_AU / m_nutrient_unit_per_cm3;
  ChangeField(m_n, center, amount_in_cm3, width);
}

void eco_sys_lab::VoxelSoilModel::ChangeCapacity(const glm::vec3& center, float amount, float width) {
  ChangeField(m_c, center, amount, width);
}

int eco_sys_lab::VoxelSoilModel::Index(const ivec3& resolution, int x, int y, int z) {
  return x + y * resolution.x + z * resolution.x * resolution.y;
}

int VoxelSoilModel::Index(const int x, const int y, const int z) const {
  return Index(m_resolution, x, y, z);
}

int eco_sys_lab::VoxelSoilModel::Index(const ivec3& resolution, const ivec3& c) {
  return Index(resolution, c.x, c.y, c.z);
}

int VoxelSoilModel::Index(const ivec3& c) const {
  return Index(m_resolution, c.x, c.y, c.z);
}

ivec3 VoxelSoilModel::GetCoordinateFromIndex(const int index) const {
  return {index % m_resolution.x, index % (m_resolution.x * m_resolution.y) / m_resolution.x,
          index / (m_resolution.x * m_resolution.y)};
}

ivec3 VoxelSoilModel::GetCoordinateFromPosition(const vec3& pos) const {
  // return {
  //	floor((pos.x - (m_boundingBoxMin.x + m_dx/2.0)) / m_dx),
  //	floor((pos.y - (m_boundingBoxMin.y + m_dx/2.0)) / m_dx),
  //	floor((pos.z - (m_boundingBoxMin.z + m_dx/2.0)) / m_dx)
  // };
  return {floor((pos.x - m_boundingBoxMin.x) / m_dx), floor((pos.y - m_boundingBoxMin.y) / m_dx),
          floor((pos.z - m_boundingBoxMin.z) / m_dx)};
}

vec3 VoxelSoilModel::GetPositionFromCoordinate(const ivec3& coordinate) const {
  return GetPositionFromCoordinate(coordinate, m_dx, m_dx, m_dx);
}

vec3 eco_sys_lab::VoxelSoilModel::GetPositionFromCoordinate(const glm::ivec3& coordinate, float dx, float dy,
                                                            float dz) const {
  return {m_boundingBoxMin.x + (dx / 2.0) + coordinate.x * dx, m_boundingBoxMin.y + (dy / 2.0) + coordinate.y * dy,
          m_boundingBoxMin.z + (dz / 2.0) + coordinate.z * dz};
}

ivec3 VoxelSoilModel::GetVoxelResolution() const {
  return m_resolution;
}

float VoxelSoilModel::GetVoxelSize() const {
  return m_dx;
}

vec3 VoxelSoilModel::GetBoundingBoxMin() const {
  return m_boundingBoxMin;
}

vec3 eco_sys_lab::VoxelSoilModel::GetBoundingBoxMax() const {
  return m_boundingBoxMin + vec3(m_resolution) * m_dx;
}
vec3 eco_sys_lab::VoxelSoilModel::GetBoundingBoxCenter() const {
  return m_boundingBoxMin + vec3(m_resolution) * m_dx * 0.5f;
}

bool eco_sys_lab::VoxelSoilModel::PositionInsideVolume(const glm::vec3& p) const {
  auto min = GetBoundingBoxMin();
  auto max = GetBoundingBoxMax();
  if (p.x < min.x || p.y < min.y || p.z < min.z)
    return false;
  if (p.x >= max.x || p.y >= max.y || p.z >= max.z)
    return false;
  return true;
}

bool eco_sys_lab::VoxelSoilModel::CoordinateInsideVolume(const glm::ivec3& coordinate) const {
  if (coordinate.x < 0 || coordinate.y < 0 || coordinate.z < 0)
    return false;
  if (coordinate.x >= m_resolution.x || coordinate.y >= m_resolution.y || coordinate.z >= m_resolution.z)
    return false;
  return true;
}

void eco_sys_lab::VoxelSoilModel::Source::Apply(Field& target) {
  for (auto i = 0u; i < idx.size(); ++i)
    target[idx[i]] += amounts[i];
}

void VoxelSoilModel::GetSoilTextureSlideZ(bool backFacing, float z, const glm::vec2& xyMin, const glm::vec2& xyMax,
                                          std::vector<glm::vec4>& albedoData, std::vector<glm::vec3>& normalData,
                                          std::vector<float>& roughnessData, std::vector<float>& metallicData,
                                          glm::ivec2& outputResolution, float waterFactor, float nutrientFactor,
                                          float blur_width) {
  const float rangeX = glm::clamp(xyMax.x, 0.0f, 0.99f) - glm::clamp(xyMin.x, 0.0f, 0.99f);
  const float rangeY = glm::clamp(xyMax.y, 0.0f, 0.99f) - glm::clamp(xyMin.y, 0.0f, 0.99f);
  outputResolution.x = rangeX * m_materialTextureResolution.x;
  outputResolution.y = rangeY * m_materialTextureResolution.y;

  const float slize_z_position = GetPositionFromCoordinate(ivec3(0, 0, glm::clamp(z, 0.0f, 0.99f) * m_resolution.z)).z;
  const float tex_dx = m_dx * static_cast<float>(m_resolution.x) / static_cast<float>(m_materialTextureResolution.x);
  const float tex_dy = m_dx * static_cast<float>(m_resolution.y) / static_cast<float>(m_materialTextureResolution.y);

  const int texCoordXStart = glm::clamp(xyMin.x, 0.0f, 0.99f) * static_cast<float>(m_materialTextureResolution.x);
  const int texCoordYStart = glm::clamp(xyMin.y, 0.0f, 0.99f) * static_cast<float>(m_materialTextureResolution.y);

  albedoData.resize(outputResolution.x * outputResolution.y);
  normalData.resize(outputResolution.x * outputResolution.y);
  roughnessData.resize(outputResolution.x * outputResolution.y);
  metallicData.resize(outputResolution.x * outputResolution.y);

  for (auto texCoordX = 0; texCoordX < outputResolution.x; ++texCoordX) {
    for (auto texCoordY = 0; texCoordY < outputResolution.y; ++texCoordY) {
      auto outputTex_idx =
          (backFacing ? outputResolution.x - texCoordX - 1 : texCoordX) + texCoordY * outputResolution.x;
      auto texture_idx = texCoordXStart + texCoordX + (texCoordYStart + texCoordY) * m_materialTextureResolution.x;
      int gridCoordX = texCoordXStart + texCoordX;
      int gridCoordY = texCoordYStart + texCoordY;
      glm::vec3 texel_position = GetPositionFromCoordinate(ivec3(gridCoordX, gridCoordY, 0), tex_dx, tex_dy, m_dx);
      texel_position.z = slize_z_position;
      if (!PositionInsideVolume(texel_position)) {
        albedoData[outputTex_idx] = glm::vec4(0.f);
        normalData[outputTex_idx] = glm::vec3(0, 0, 1);
        roughnessData[outputTex_idx] = 0.8f;
        metallicData[outputTex_idx] = 0.2f;
      } else {
        GetSoilTextureColorForPosition(texel_position, texture_idx, blur_width, albedoData[outputTex_idx],
                                       normalData[outputTex_idx], roughnessData[outputTex_idx],
                                       metallicData[outputTex_idx], waterFactor, nutrientFactor);
        albedoData[outputTex_idx] = glm::vec4(1.0f);
        if (texel_position.y > m_soilSurface.m_height({texel_position.x, texel_position.z}) + 0.01f) {
          albedoData[outputTex_idx].w = 0.0f;
        }
      }
    }
  }
}

void VoxelSoilModel::GetSoilTextureSlideX(bool backFacing, float x, const glm::vec2& yzMin, const glm::vec2& yzMax,
                                          std::vector<glm::vec4>& albedoData, std::vector<glm::vec3>& normalData,
                                          std::vector<float>& roughnessData, std::vector<float>& metallicData,
                                          glm::ivec2& outputResolution, float waterFactor, float nutrientFactor,
                                          float blur_width) {
  const float rangeZ = glm::clamp(yzMax.x, 0.0f, 0.99f) - glm::clamp(yzMin.x, 0.0f, 0.99f);
  const float rangeY = glm::clamp(yzMax.y, 0.0f, 0.99f) - glm::clamp(yzMin.y, 0.0f, 0.99f);
  outputResolution.x = rangeZ * m_materialTextureResolution.x;
  outputResolution.y = rangeY * m_materialTextureResolution.y;
  const float slize_x_position = GetPositionFromCoordinate(ivec3(glm::clamp(x, 0.0f, 0.99f) * m_resolution.x, 0, 0)).x;
  const float tex_dz = m_dx * static_cast<float>(m_resolution.z) / static_cast<float>(m_materialTextureResolution.x);
  const float tex_dy = m_dx * static_cast<float>(m_resolution.y) / static_cast<float>(m_materialTextureResolution.y);

  const int texCoordXStart = glm::clamp(yzMin.x, 0.0f, 0.99f) * static_cast<float>(m_materialTextureResolution.x);
  const int texCoordYStart = glm::clamp(yzMin.y, 0.0f, 0.99f) * static_cast<float>(m_materialTextureResolution.y);

  albedoData.resize(outputResolution.x * outputResolution.y);
  normalData.resize(outputResolution.x * outputResolution.y);
  roughnessData.resize(outputResolution.x * outputResolution.y);
  metallicData.resize(outputResolution.x * outputResolution.y);

  for (auto texCoordX = 0; texCoordX < outputResolution.x; ++texCoordX) {
    for (auto texCoordY = 0; texCoordY < outputResolution.y; ++texCoordY) {
      auto outputTex_idx =
          (backFacing ? outputResolution.x - texCoordX - 1 : texCoordX) + texCoordY * outputResolution.x;
      auto texture_idx = texCoordXStart + texCoordX + (texCoordYStart + texCoordY) * m_materialTextureResolution.x;
      int gridCoordZ = texCoordXStart + texCoordX;
      int gridCoordY = texCoordYStart + texCoordY;
      glm::vec3 texel_position = GetPositionFromCoordinate(ivec3(0, gridCoordY, gridCoordZ), m_dx, tex_dy, tex_dz);
      texel_position.x = slize_x_position;
      if (!PositionInsideVolume(texel_position)) {
        albedoData[outputTex_idx] = glm::vec4(0.f);
        normalData[outputTex_idx] = glm::vec3(0, 0, 1);
        roughnessData[outputTex_idx] = 0.8f;
        metallicData[outputTex_idx] = 0.2f;
      } else {
        GetSoilTextureColorForPosition(texel_position, texture_idx, blur_width, albedoData[outputTex_idx],
                                       normalData[outputTex_idx], roughnessData[outputTex_idx],
                                       metallicData[outputTex_idx], waterFactor, nutrientFactor);
        albedoData[outputTex_idx] = glm::vec4(1.0f);
        if (texel_position.y > m_soilSurface.m_height({texel_position.x, texel_position.z}) + 0.01f) {
          albedoData[outputTex_idx].w = 0.0f;
        }
      }
    }
  }
}

void eco_sys_lab::VoxelSoilModel::GetSoilTextureColorForPosition(const glm::vec3& position, int texture_idx,
                                                                 float blur_width, glm::vec4& albedo, glm::vec3& normal,
                                                                 float& roughness, float& metallic, float waterFactor,
                                                                 float nutrientFactor) {
  const float blur_kernel_width = m_dx * m_dx * blur_width * blur_width;
  auto soil_voxel_base = GetCoordinateFromPosition(position);
  std::map<int, float> contributing_materials;  // we need to store the total some for each material:

  // do some gaussian blending
  float waterLevel = 0.0f;
  float nutrientLevel = 0.0f;
  for (auto i = 0; i < m_blur_3x3_idx.size(); ++i)  // iterate over blur kernel
  {
    auto soil_voxel = soil_voxel_base + m_blur_3x3_idx[i];
    if (CoordinateInsideVolume(soil_voxel)) {
      // fetch material
      auto material_id = m_material_id[Index(soil_voxel)];
      if (material_id < 0 || material_id >= m_soilLayers.size()) {
        albedo = glm::vec4(0.f);
        normal = glm::vec3(0, 0, 1);
        roughness = 0.8f;
        metallic = 0.2f;
        return;
      }
      const auto& material = m_soilLayers[material_id].m_mat;
      const auto& texPtr = material.m_soilMaterialTexture;
      if (!texPtr) {
        albedo = glm::vec4(0.f);
        normal = glm::vec3(0, 0, 1);
        roughness = 0.8f;
        metallic = 0.2f;
        return;
      }
      const auto p = GetPositionFromCoordinate(soil_voxel);
      const auto dist = glm::length(p - position);

      // compute weight:
      const float weight = glm::exp(-dist * dist / blur_kernel_width);

      if (contributing_materials.find(material_id) == contributing_materials.end())
        contributing_materials[material_id] = 0;

      auto heightmap_height = texPtr->m_height_map[texture_idx];
      contributing_materials[material_id] += weight * heightmap_height * heightmap_height;
      // total_weight +=  weight * tex.m_height_map[texture_idx];

      // output_color += tex.m_color_map[texture_idx] * weight;

      waterLevel += m_w[Index(soil_voxel)] * waterFactor * weight;
      nutrientLevel += m_n[Index(soil_voxel)] * nutrientFactor * weight;
    }
  }

  // max texture:
  /*
  using pair_type = decltype(contributing_materials)::value_type;
  auto max_mat = std::max_element(contributing_materials.begin(), contributing_materials.end(), []
  (const pair_type & p1, const pair_type & p2)
  {
  return p1.second < p2.second;
  }
  );
  output[texture_idx] = textures[max_mat->first]->m_color_map[texture_idx];
  */

  // blend according to weights:
  float total_weight = 0;
  albedo = glm::vec4(0.0f);
  normal = glm::vec3(0.f);
  metallic = 0.0f;
  roughness = 0.0f;
  for (auto& p : contributing_materials) {
    auto weight = p.second * p.second;
    auto& textures = m_soilLayers[p.first].m_mat.m_soilMaterialTexture;
    albedo += textures->m_color_map[texture_idx] * weight;
    normal += textures->m_normal_map[texture_idx] * weight;
    metallic += textures->m_metallic_map[texture_idx] * weight;
    roughness += textures->m_roughness_map[texture_idx] * weight;
    total_weight += weight;
  }
  albedo /= total_weight;
  waterLevel /= total_weight;
  nutrientLevel /= total_weight;
  albedo = glm::mix(albedo, glm::vec4(glm::vec3(albedo * 0.2f), 1.0f), glm::clamp(waterLevel, 0.0f, 1.0f));
  albedo = glm::mix(albedo, albedo * 0.8f + glm::vec4(0.0, 0.2, 0, 0.2), glm::clamp(nutrientLevel, 0.0f, 1.0f));

  normal /= total_weight;
  metallic /= total_weight;
  roughness /= total_weight;
}

/////////////////////////////////////////

void eco_sys_lab::VoxelSoilModel::UpdateStats() {
  // count total water:
  m_w_sum_in_g = 0.f;
  m_n_sum = 0.f;
  for (auto i = 0; i < m_w.size(); ++i) {
    m_w_sum_in_g += m_w[i];
    m_n_sum += m_n[i];
  }
  m_w_sum_in_g *= m_voxel_volume_in_cm3 * m_water_g_per_cm3;
  m_n_sum *= m_voxel_volume_in_cm3 * m_nutrient_unit_per_cm3;

  auto max_p = m_p.max();
  auto max_l = m_l.max();
  auto max_grav = glm::max(m_gravityForce.x, glm::max(m_gravityForce.y, m_gravityForce.z));
  m_max_speed_diff = max_p * max_l * m_diffusionForce;
  m_max_speed_grav = max_p * max_l * max_grav;
}

void eco_sys_lab::VoxelSoilModel::Test_InitializeEmpty(glm::uvec3 resolution) {
  SoilParameters p;
  p.m_boundary_x = Boundary::wrap;
  p.m_boundary_y = Boundary::wrap;
  p.m_boundary_z = Boundary::wrap;
  p.m_boundingBoxMin = vec3(0, 0, 0);
  p.m_voxelResolution = resolution;
  p.m_deltaX = 1.f / static_cast<float>(resolution.x);
  p.m_gravityForce = vec3(0, 0, 0);
  p.m_diffusionForce = 1;
  p.m_deltaTime = 0.1;

  const auto lambda_0 = [](const vec2& p) {
    return 0.f;
  };
  const auto lambda_1 = [](const vec2& p) {
    return 1.f;
  };
  const auto lambda_10 = [](const vec2& p) {
    return 10.f;
  };

  SoilSurface surface;
  surface.m_height = lambda_0;

  SoilPhysicalMaterial mat;
  mat.m_c = lambda_1;
  mat.m_d = lambda_1;
  mat.m_p = lambda_1;
  mat.m_n = lambda_0;
  mat.m_w = lambda_0;

  auto soil_layers = std::vector<SoilLayer>({SoilLayer({mat, lambda_10})});

  Initialize(p, surface, soil_layers);

  m_water_sources.clear();  // clear rain source
}

void eco_sys_lab::VoxelSoilModel::Test_WaterDensity() {
  auto water_density_test = [this]() {
    cout << "\nWater Density test:" << endl;

    m_w = 0.f;
    UpdateStats();
    cout << "Initial water: " << m_w_sum_in_g << endl;

    ChangeWater(vec3(0.5, 0.5, 0.5), 1000, 0.5);
    UpdateStats();
    cout << "Adding 1000: " << m_w_sum_in_g << endl;

    ChangeWater(vec3(0.5, 0.5, 0.5), 500, 2);
    UpdateStats();
    cout << "Adding 500: " << m_w_sum_in_g << endl;

    ChangeWater(vec3(0.5, 0.5, 0.5), 500, 0.1);
    UpdateStats();
    cout << "Adding 500: " << m_w_sum_in_g << endl;

    ChangeWater(vec3(0.1, 0.8, 0.5), 1000, 0.1);
    UpdateStats();
    cout << "Adding 1000: " << m_w_sum_in_g << endl;

    cout << "Water density at center: " << GetWaterDensity(vec3(0.5, 0.5, 0.5)) << endl;
  };

  auto nutrient_density_test = [this]() {
    cout << "\nNutrient Density test:" << endl;
    m_n = 0.f;
    UpdateStats();
    cout << "Initial nutrient: " << m_n_sum << endl;

    ChangeNutrient(vec3(0.0, 0.5, 0.5), 100, 0.5);
    UpdateStats();
    cout << "Adding 100: " << m_n_sum << endl;
    cout << "Nutrient density at center: " << GetNutrientDensity(vec3(0.5, 0.5, 0.5)) << endl;
  };

  auto water_fetch_test = [this]() {
    cout << "\nFetch Water test:" << endl;

    m_w = 0.f;
    auto positon = vec3(0.51, 0.52, 0.53);
    ChangeWater(positon, 100, 0.5);
    UpdateStats();
    cout << "Total water (added 100): " << m_w_sum_in_g << endl;
    cout << "Fetch Water larger width: " << IntegrateWater(positon, 0.8) << endl;
    cout << "Fetch Water same width: " << IntegrateWater(positon, 0.5) << endl;
    cout << "Fetch Water smaller width: " << IntegrateWater(positon, 0.2) << endl;
    cout << "Fetch Water tiny width: " << IntegrateWater(positon, 0.01) << endl;
    cout << "Water density at center: " << GetWaterDensity(positon) << endl;
  };

  auto nutrient_fetch_test = [this]() {
    cout << "\nFetch Nutrient test:" << endl;

    m_n = 0.f;
    auto positon = vec3(0.3, 0.5211, 0.53);
    ChangeNutrient(positon, 17, 0.1);
    UpdateStats();
    cout << "Total nutrient (added 17): " << m_n_sum << endl;
    cout << "Fetch Nutrient larger: " << IntegrateNutrient(positon, 0.8) << endl;
    cout << "Fetch Nutrient medium: " << IntegrateNutrient(positon, 0.5) << endl;
    cout << "Fetch Nutrient small:  " << IntegrateNutrient(positon, 0.2) << endl;
    cout << "Fetch Nutrient tiny:   " << IntegrateNutrient(positon, 0.05) << endl;

    m_n = 1.f;
    cout << "Set Nutrient Density to constant 1" << endl;
    UpdateStats();
    cout << "Total nutrient: " << m_n_sum << endl;
    cout << "Fetch Nutrient Density 1, width 0.1 : " << IntegrateNutrient(positon, 0.1) << endl;
    cout << "Fetch Nutrient Density 1, width 0.2 : " << IntegrateNutrient(positon, 0.2) << endl;
  };

  Test_InitializeEmpty(uvec3(100, 100, 100));
  cout << "\n Resolution: " << m_resolution.x << "\n";

  water_density_test();
  nutrient_density_test();
  water_fetch_test();
  nutrient_fetch_test();

  // change resolution and test again:
  Test_InitializeEmpty(uvec3(64, 64, 64));
  cout << "\n Resolution: " << m_resolution.x << "\n";

  water_density_test();
  nutrient_density_test();
  water_fetch_test();
  nutrient_fetch_test();

  // change resolution and test again:
  Test_InitializeEmpty(uvec3(30, 30, 30));
  cout << "\n Resolution: " << m_resolution.x << "\n";

  water_density_test();
  nutrient_density_test();
  water_fetch_test();
  nutrient_fetch_test();
}

void eco_sys_lab::VoxelSoilModel::Test_PermeabilitySpeed() {
  auto perm_setup = [this](uvec3 resolution = {50, 100, 50}, float water_width = 0.2f) {
    Test_InitializeEmpty(resolution);
    m_boundary_x = Boundary::absorb;
    m_boundary_y = Boundary::absorb;
    m_boundary_z = Boundary::absorb;
    m_dt = 0.01f;
    m_gravityForce = vec3(0, -1, 0);
    m_diffusionForce = 0.0f;
    ChangeWater({0.5, 1.5, 0.5}, 100, water_width);
  };

  auto center_of_mass = [this]() {
    vec3 position(0.f);

    float density_sum = 0;

    for (auto z = 0; z < m_resolution.z; ++z) {
      for (auto y = 0; y < m_resolution.y; ++y) {
        for (auto x = 0; x < m_resolution.x; ++x) {
          auto density = m_w[Index(x, y, z)];
          density_sum += density;
          position += GetPositionFromCoordinate({x, y, z}) * density;
        }
      }
    }

    position /= density_sum;
    return position;
  };

  auto test_timesteps = [this]() {
    m_dt = 1.f;
    Run(1);
    cout << "run for 1: " << GetTime() << endl;
    Run(0.6);
    cout << "run for 0.6: " << GetTime() << endl;
    Run(0.5);
    cout << "run for 0.5: " << GetTime() << endl;
    Run(3.8);
    cout << "run for 3.8: " << GetTime() << endl;
    Run(0.2);
    cout << "run for 0.2: " << GetTime() << endl;

    m_dt = 0.1f;
    Reset();
    cout << "Reset" << endl;
    Run(1);
    cout << "run for 1: " << GetTime() << endl;
    Run(0.6);
    cout << "run for 0.6: " << GetTime() << endl;
    Run(0.5);
    cout << "run for 0.5: " << GetTime() << endl;
    Run(3.8);
    cout << "run for 3.8: " << GetTime() << endl;
    Run(0.2);
    cout << "run for 0.2: " << GetTime() << endl;

    Reset();
  };

  // test time:
  // test_timesteps();

  auto perform_measurement = [&](string filename, int steps = 200) {
    cout << "Starting " << filename << "..." << endl;
    auto file = fstream(filename, ios_base::out | ios_base::trunc);
    for (auto i = 0; i < steps; ++i) {
      Step();
      UpdateStats();
      auto center = center_of_mass();
      file << GetTime() << "," << center.x << ", " << center.y << ", " << center.z << ", " << m_w_sum_in_g << endl;
    }
    cout << "done." << endl;
  };

  perm_setup();
  perform_measurement("Water_Speed_01_reference.csv");

  perm_setup();
  m_dt = 0.008f;
  perform_measurement("Water_Speed_02_dt.csv");

  perm_setup({64, 128, 64});
  perform_measurement("Water_Speed_03_resolution.csv");

  perm_setup();
  m_p = 0.75f;
  perform_measurement("Water_Speed_04_permeability.csv");

  perm_setup();
  m_diffusionForce = 0.1f;
  m_dt = 0.0025;  // much smaller time step with diffusion
  perform_measurement("Water_Speed_05_diffusion.csv");

  perm_setup();
  m_gravityForce = vec3(0, -0.6, 0);
  perform_measurement("Water_Speed_06_gravity.csv");

  perm_setup({50, 100, 50}, 0.075);
  perform_measurement("Water_Speed_07_water_width.csv");

  perm_setup();
  m_c = 0.5;
  m_diffusionForce = 0.2f;
  m_dt = 0.001;
  perform_measurement("Water_Speed_08_capacity_low.csv", 600);

  perm_setup();
  m_c = 1.5;
  m_diffusionForce = 0.2f;
  m_dt = 0.001;
  perform_measurement("Water_Speed_09_capacity_high.csv", 600);

  perm_setup();
  m_c = 0.5;
  m_diffusionForce = 0.05f;
  m_dt = 0.001;
  perform_measurement("Water_Speed_10_capacity_low_diffusion_low.csv", 600);
}

void eco_sys_lab::VoxelSoilModel::Test_NutrientTransport(float p, float c,
                                                         const std::shared_ptr<SoilMaterialTexture>& texture) {
  auto setup = [this, texture](float permeability, float capacity) {
    SoilParameters p;  // standard values as defined when this function was first added are fine.
    p.m_voxelResolution = {32, 48, 32};
    p.m_boundingBoxMin = {-3.2, -6.4, -3.2};
    p.m_deltaX = 0.2f;
    p.m_boundary_x = Boundary::wrap;
    p.m_boundary_y = Boundary::absorb;
    p.m_boundary_z = Boundary::wrap;
    p.m_nutrientForce = 1.f;

    const auto Value = [](float v) {
      return [v](const vec2& p) {
        return v;
      };
    };

    const auto l_0 = [](const vec2& p) {
      return 0.f;
    };
    const auto l_1 = [](const vec2& p) {
      return 1.f;
    };
    const auto l_10 = [](const vec2& p) {
      return 10.f;
    };
    const auto l_height = [](const vec2& p) {
      return 1.5f;
    };

    SoilSurface surface;
    surface.m_height = l_height;

    SoilPhysicalMaterial Air;
    Air.m_c = Value(1);
    Air.m_d = Value(0);
    Air.m_p = Value(0);
    Air.m_n = Value(0);
    Air.m_w = Value(0);
    Air.m_soilMaterialTexture = texture;

    SoilPhysicalMaterial nutrient;
    nutrient.m_c = l_0;
    nutrient.m_d = l_1;
    nutrient.m_p = l_0;
    nutrient.m_n = l_10;
    nutrient.m_w = l_0;
    nutrient.m_soilMaterialTexture = texture;

    SoilPhysicalMaterial soil;
    soil.m_c = l_0;
    soil.m_d = l_1;
    soil.m_p = l_0;
    soil.m_n = l_0;
    soil.m_w = l_0;
    soil.m_soilMaterialTexture = texture;

    auto soil_layers = std::vector<SoilLayer>({SoilLayer({Air, Value(0)}), SoilLayer({soil, Value(0.2)}),
                                               SoilLayer({nutrient, Value(0.5)}), SoilLayer({soil, Value(10)})});

    for (int i = 0; i < soil_layers.size(); i++) {
      soil_layers[i].m_mat.m_id = i;
    }

    Initialize(p, surface, soil_layers);

    // m_water_sources.clear(); // clear rain source

    m_p = permeability;
    m_c = capacity;
    m_dt = 0.1;  // seems stable enough
  };

  setup(p, c);  // sand material

  // to create a nutrient source, we copy the water source:
  // m_nutrient_sources.push_back(*m_water_sources.begin());
}

void eco_sys_lab::VoxelSoilModel::Test_NutrientTransport_Sand(const std::shared_ptr<SoilMaterialTexture>& texture) {
  Test_NutrientTransport(0.5, 100, texture);
}

void eco_sys_lab::VoxelSoilModel::Test_NutrientTransport_Loam(const std::shared_ptr<SoilMaterialTexture>& texture) {
  Test_NutrientTransport(0.1, 75, texture);
}

void eco_sys_lab::VoxelSoilModel::Test_NutrientTransport_Silt(const std::shared_ptr<SoilMaterialTexture>& texture) {
  Test_NutrientTransport(0.033, 50, texture);
}
