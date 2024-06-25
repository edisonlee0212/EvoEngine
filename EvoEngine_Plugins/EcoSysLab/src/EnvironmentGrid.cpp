#include "EnvironmentGrid.hpp"
#include "SimulationSettings.hpp"
using namespace eco_sys_lab;

float EnvironmentGrid::Sample(const glm::vec3& position, glm::vec3& light_direction) const {
  const auto coordinate = voxel_grid.GetCoordinate(position);
  const auto& data = voxel_grid.Peek(coordinate);
  light_direction = data.light_direction;
  return data.light_intensity;

  const auto resolution = voxel_grid.GetResolution();
  const auto voxel_center = voxel_grid.GetPosition(coordinate);
  float top_intensity;
  if (coordinate.y < resolution.y - 1)
    top_intensity = voxel_grid.Peek(glm::ivec3(coordinate.x, coordinate.y + 1, coordinate.z)).light_intensity;
  else
    top_intensity = data.light_intensity;

  top_intensity = (top_intensity + data.light_intensity) / 2.f;
  float bottom_intensity;
  if (coordinate.y > 0)
    bottom_intensity = voxel_grid.Peek(glm::ivec3(coordinate.x, coordinate.y - 1, coordinate.z)).light_intensity;
  else
    bottom_intensity = data.light_intensity;
  bottom_intensity = (bottom_intensity + data.light_intensity) / 2.f;
  const float a = (position.y - voxel_center.y + 0.5f * voxel_size) / voxel_size;
  assert(a < 1.f);
  return glm::mix(bottom_intensity, top_intensity, a);
}

void EnvironmentGrid::AddShadowValue(const glm::vec3& position, const float value) {
  auto& data = voxel_grid.Ref(position);
  data.self_shadow += value;
}

void EnvironmentGrid::LightPropagation(const SimulationSettings& simulation_settings) {
  const auto resolution = voxel_grid.GetResolution();
  const int shadow_disk_size = glm::ceil(simulation_settings.detection_radius / voxel_size);
  Jobs::RunParallelFor(resolution.x * resolution.z, [&](size_t i) {
    const int x = i / resolution.z;
    const int z = i % resolution.z;
    auto& target_voxel = voxel_grid.Ref(glm::ivec3(x, resolution.y - 1, z));
    target_voxel.light_intensity = simulation_settings.skylight_intensity;
  });
  for (int y = resolution.y - 2; y >= 0; y--) {
    Jobs::RunParallelFor(resolution.x * resolution.z, [&](size_t i) {
      const int x = i / resolution.z;
      const int z = i % resolution.z;
      float sum = 0.0f;
      float max = 0.f;

      for (int x_offset = -shadow_disk_size; x_offset <= shadow_disk_size; x_offset++) {
        for (int z_offset = -shadow_disk_size; z_offset <= shadow_disk_size; z_offset++) {
          if (y + 1 > resolution.y - 1)
            continue;
          const auto other_voxel_center = glm::ivec3(x + x_offset, y + 1, z + z_offset);
          const auto position_diff = voxel_grid.GetPosition(other_voxel_center) - voxel_grid.GetPosition(glm::ivec3(x, y, z));
          const float distance = glm::length(position_diff);
          if (distance > simulation_settings.detection_radius)
            continue;
          const float base_loss_factor = voxel_size / distance;
          const float distance_loss = glm::pow(glm::max(0.0f, base_loss_factor), simulation_settings.shadow_distance_loss);
          if (x + x_offset < 0 || x + x_offset > resolution.x - 1 || z + z_offset < 0 || z + z_offset > resolution.z - 1) {
            sum += distance_loss;
            max += distance_loss;
          } else {
            const auto& target_voxel = voxel_grid.Ref(other_voxel_center);
            sum += glm::max(target_voxel.light_intensity * distance_loss * (1.f - target_voxel.self_shadow), 0.0f);
            max += distance_loss;
          }
        }
      }
      auto& voxel = voxel_grid.Ref(glm::ivec3(x, y, z));
      voxel.light_intensity = glm::clamp(sum / max, 0.0f, 1.0f - simulation_settings.environment_light_intensity) +
                               simulation_settings.environment_light_intensity;
    });
  }
  for (int iteration = 0; iteration < simulation_settings.blur_iteration; iteration++) {
    for (int y = resolution.y - 2; y >= 0; y--) {
      Jobs::RunParallelFor(resolution.x * resolution.z, [&](size_t i) {
        const int x = i / resolution.z;
        const int z = i % resolution.z;

        const float self_intensity = voxel_grid.Ref(glm::ivec3(x, y, z)).light_intensity;
        float intensity = self_intensity * .4f;
        intensity += voxel_grid.Ref(glm::ivec3(x, y + 1, z)).light_intensity * .1f;
        if (y > 0)
          intensity += voxel_grid.Ref(glm::ivec3(x, y - 1, z)).light_intensity * .1f;
        else
          intensity += self_intensity * .1f;

        if (x > 0)
          intensity += voxel_grid.Ref(glm::ivec3(x - 1, y, z)).light_intensity * .1f;
        else
          intensity += self_intensity * .1f;

        if (z > 0)
          intensity += voxel_grid.Ref(glm::ivec3(x, y, z - 1)).light_intensity * .1f;
        else
          intensity += self_intensity * .1f;

        if (x < resolution.x - 1)
          intensity += voxel_grid.Ref(glm::ivec3(x + 1, y, z)).light_intensity * .1f;
        else
          intensity += self_intensity * .1f;

        if (z < resolution.z - 1)
          intensity += voxel_grid.Ref(glm::ivec3(x, y, z + 1)).light_intensity * .1f;
        else
          intensity += self_intensity * .1f;

        voxel_grid.Ref(glm::ivec3(x, y, z)).light_intensity = intensity;
      });
    }
  }
  const int light_space_size = glm::ceil(simulation_settings.detection_radius / voxel_size);
  for (int y = resolution.y - 1; y >= 0; y--) {
    Jobs::RunParallelFor(resolution.x * resolution.z, [&](unsigned i) {
      const int x = i / resolution.z;
      const int z = i % resolution.z;
      glm::vec3 sum = glm::vec3(0.0f);
      for (int x_offset = -light_space_size; x_offset <= light_space_size; x_offset++) {
        for (int z_offset = -light_space_size; z_offset <= light_space_size; z_offset++) {
          for (int y_offset = 1; y_offset <= light_space_size; y_offset++) {
            if (y + y_offset < 0 || y + y_offset > resolution.y - 1)
              continue;
            const auto other_voxel_center = glm::ivec3(x + x_offset, y + y_offset, z + z_offset);
            const auto position_diff = voxel_grid.GetPosition(other_voxel_center) - voxel_grid.GetPosition(glm::ivec3(x, y, z));
            const float distance = glm::length(position_diff);
            if (distance > simulation_settings.detection_radius)
              continue;

            if (x + x_offset < 0 || x + x_offset > resolution.x - 1 || z + z_offset < 0 ||
                z + z_offset > resolution.z - 1) {
              sum += position_diff;
            } else {
              const auto& target_voxel = voxel_grid.Ref(other_voxel_center);
              sum += target_voxel.light_intensity * position_diff;
            }
          }
        }
      }
      auto& voxel = voxel_grid.Ref(glm::ivec3(x, y, z));
      if (glm::length(sum) > glm::epsilon<float>())
        voxel.light_direction = glm::normalize(sum);
      else
        voxel.light_direction = glm::vec3(0.0f, 1.0f, 0.0f);
    });
  }
}

void EnvironmentGrid::AddBiomass(const glm::vec3& position, const float value) {
  auto& data = voxel_grid.Ref(position);
  data.total_biomass += value;
}

void EnvironmentGrid::AddNode(const InternodeVoxelRegistration& registration) {
  auto& data = voxel_grid.Ref(registration.position);
  data.internode_voxel_registrations.emplace_back(registration);
}
