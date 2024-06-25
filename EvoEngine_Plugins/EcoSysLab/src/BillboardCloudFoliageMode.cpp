
#include "BillboardCloud.hpp"
using namespace evo_engine;

std::vector<BillboardCloud::Cluster> BillboardCloud::StochasticClusterize(
    std::vector<ClusterTriangle> operating_triangles, const ClusterizationSettings& clusterize_settings) {
  BoundingSphere bounding_sphere;
  bounding_sphere.Initialize(elements);
  const auto& settings = clusterize_settings.foliage_clusterization_settings;
  float epsilon = bounding_sphere.radius * glm::clamp(1.f - settings.density, 0.05f, 1.f);

  skipped_triangles.clear();
  auto remaining_triangles = operating_triangles;
  std::vector<Cluster> ret_val;

  int epoch = 0;
  while (!remaining_triangles.empty()) {
    Cluster new_cluster;
    float max_area = 0.f;
    std::vector<int> selected_triangle_indices;
    std::mutex vote_mutex;
    Jobs::RunParallelFor(settings.iteration, [&](unsigned iteration) {
      int seed_triangle_index = glm::linearRand(0, static_cast<int>(remaining_triangles.size()) - 1);
      ClusterTriangle seed_triangle = remaining_triangles.at(seed_triangle_index);
      const auto perturb0 = glm::linearRand(-epsilon, epsilon);
      const auto perturb1 = glm::linearRand(-epsilon, epsilon);
      const auto perturb2 = glm::linearRand(-epsilon, epsilon);
      const auto& seed_triangle_element = elements.at(seed_triangle.element_index);
      const auto& seed_triangle_indices = seed_triangle_element.triangles.at(seed_triangle.triangle_index);
      const auto seed_triangle_normal = CalculateNormal(seed_triangle);
      glm::vec3 seed_triangle_p0 =
          seed_triangle_element.vertices.at(seed_triangle_indices.x).position + perturb0 * seed_triangle_normal;
      glm::vec3 seed_triangle_p1 =
          seed_triangle_element.vertices.at(seed_triangle_indices.y).position + perturb1 * seed_triangle_normal;
      glm::vec3 seed_triangle_p2 =
          seed_triangle_element.vertices.at(seed_triangle_indices.z).position + perturb2 * seed_triangle_normal;
      auto test_plane_normal =
          glm::normalize(glm::cross(seed_triangle_p0 - seed_triangle_p1, seed_triangle_p0 - seed_triangle_p2));
      if (glm::dot(test_plane_normal, seed_triangle_normal) < 0.f) {
        test_plane_normal = -test_plane_normal;
      }
      float test_plane_distance = glm::dot(seed_triangle_p0, test_plane_normal);
      float area = 0.f;
      std::vector<int> current_pending_removal_triangles;
      std::vector<ClusterTriangle> triangles_for_cluster;
      for (int test_triangle_index = 0; test_triangle_index < remaining_triangles.size(); test_triangle_index++) {
        const auto& test_triangle = remaining_triangles.at(test_triangle_index);
        const auto& test_triangle_element = elements.at(test_triangle.element_index);
        const auto& test_triangle_indices = test_triangle_element.triangles.at(test_triangle.triangle_index);
        const auto& test_triangle_p0 = test_triangle_element.vertices.at(test_triangle_indices.x).position;
        const auto& test_triangle_p1 = test_triangle_element.vertices.at(test_triangle_indices.y).position;
        const auto& test_triangle_p2 = test_triangle_element.vertices.at(test_triangle_indices.z).position;

        if (!settings.fill_band &&
            glm::abs(test_plane_distance - glm::dot(test_triangle_p0, test_plane_normal)) <=
                epsilon * settings.sample_range &&
            glm::abs(test_plane_distance - glm::dot(test_triangle_p1, test_plane_normal)) <=
                epsilon * settings.sample_range &&
            glm::abs(test_plane_distance - glm::dot(test_triangle_p2, test_plane_normal)) <=
                epsilon * settings.sample_range) {
          triangles_for_cluster.emplace_back(remaining_triangles.at(test_triangle_index));
        }
        if (glm::abs(test_plane_distance - glm::dot(test_triangle_p0, test_plane_normal)) > epsilon)
          continue;
        if (glm::abs(test_plane_distance - glm::dot(test_triangle_p1, test_plane_normal)) > epsilon)
          continue;
        if (glm::abs(test_plane_distance - glm::dot(test_triangle_p2, test_plane_normal)) > epsilon)
          continue;
        // increment projected area (Angular area Contribution)
        // use projected area Contribution
        float angle = glm::acos(glm::abs(glm::dot(test_plane_normal, CalculateNormal(test_triangle))));
        float angular = (glm::pi<float>() / 2.f - angle) / (glm::pi<float>() / 2.f);
        area += CalculateArea(test_triangle) * angular;

        // save reference to T with billboard plane
        current_pending_removal_triangles.emplace_back(test_triangle_index);
      }

      if (settings.fill_band) {
        for (auto& operating_triangle : operating_triangles) {
          const auto& test_triangle = operating_triangle;
          const auto& test_triangle_element = elements.at(test_triangle.element_index);
          const auto& test_triangle_indices = test_triangle_element.triangles.at(test_triangle.triangle_index);
          const auto& test_triangle_p0 = test_triangle_element.vertices.at(test_triangle_indices.x).position;
          const auto& test_triangle_p1 = test_triangle_element.vertices.at(test_triangle_indices.y).position;
          const auto& test_triangle_p2 = test_triangle_element.vertices.at(test_triangle_indices.z).position;

          if (glm::abs(test_plane_distance - glm::dot(test_triangle_p0, test_plane_normal)) <=
                  epsilon * settings.sample_range &&
              glm::abs(test_plane_distance - glm::dot(test_triangle_p1, test_plane_normal)) <=
                  epsilon * settings.sample_range &&
              glm::abs(test_plane_distance - glm::dot(test_triangle_p2, test_plane_normal)) <=
                  epsilon * settings.sample_range) {
            triangles_for_cluster.emplace_back(operating_triangle);
          }
        }
      }
      if (!current_pending_removal_triangles.empty()) {
        std::lock_guard lock(vote_mutex);
        if (area > max_area) {
          // Update cluster.
          new_cluster.cluster_plane = Plane(test_plane_normal, test_plane_distance);
          new_cluster.triangles = triangles_for_cluster;
          selected_triangle_indices = current_pending_removal_triangles;
        }
      }
    });

    if (selected_triangle_indices.empty()) {
      skipped_triangles.insert(skipped_triangles.end(), remaining_triangles.begin(), remaining_triangles.end());
      break;
    }

    if (!new_cluster.triangles.empty())
      ret_val.emplace_back(std::move(new_cluster));

    // Remove selected triangle from the remaining triangle.
    for (auto it = selected_triangle_indices.rbegin(); it != selected_triangle_indices.rend(); ++it) {
      remaining_triangles[*it] = remaining_triangles.back();
      remaining_triangles.pop_back();
    }
    epoch++;
    if (settings.timeout != 0 && epoch >= settings.timeout) {
      EVOENGINE_ERROR("Stochastic clustering timeout!")
      break;
    }
  }
  return ret_val;
}
