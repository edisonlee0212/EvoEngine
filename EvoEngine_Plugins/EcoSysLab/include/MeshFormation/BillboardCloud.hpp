#pragma once

#include "Material.hpp"
#include "Mesh.hpp"

namespace evo_engine {
class BillboardCloud {
 public:
  template <typename T>
  static void DilateChannels(const std::vector<size_t>& channels, std::vector<T>& data, std::vector<bool>& valid_pixels,
                     size_t max_dist, const glm::uvec2& resolution, bool diagonals);

  template <typename T>
  static void Dilate(std::vector<T>& data, std::vector<bool>& valid_pixels,
                     size_t max_dist, const glm::uvec2& resolution, bool diagonals);

  struct ClusterTriangle {
    int element_index = -1;
    int triangle_index = -1;

    int index;
  };
  struct ProjectedTriangle {
    Vertex projected_vertices[3];
    Handle material_handle;
  };

  struct Element {
    /**
     * Vertices must be in model space.
     */
    std::vector<Vertex> vertices;
    std::shared_ptr<Material> material;
    std::vector<glm::uvec3> triangles;

    [[nodiscard]] glm::vec3 CalculateCentroid(int triangle_index) const;
    [[nodiscard]] float CalculateArea(int triangle_index) const;
    [[nodiscard]] float CalculateNormalDistance(int triangle_index) const;
    [[nodiscard]] glm::vec3 CalculateNormal(int triangle_index) const;

    [[nodiscard]] glm::vec3 CalculateCentroid(const glm::uvec3& triangle) const;
    [[nodiscard]] float CalculateArea(const glm::uvec3& triangle) const;
    [[nodiscard]] glm::vec3 CalculateNormal(const glm::uvec3& triangle) const;
    [[nodiscard]] float CalculateNormalDistance(const glm::uvec3& triangle) const;

    [[nodiscard]] std::vector<std::vector<unsigned>> CalculateLevelSets(const glm::vec3& direction = glm::vec3(0, 1,
                                                                                                               0));
  };
  struct BoundingSphere {
    glm::vec3 center = glm::vec3(0.f);
    float radius = 0.f;

    void Initialize(const std::vector<Element>& target_elements);
  };

  struct Rectangle {
    glm::vec2 points[4];

    glm::vec2 tex_coords[4];

    void Update();

    glm::vec2 center;
    glm::vec2 x_axis;
    glm::vec2 y_axis;

    float width;
    float height;

    [[nodiscard]] glm::vec2 Transform(const glm::vec2& target) const;
    [[nodiscard]] glm::vec3 Transform(const glm::vec3& target) const;
  };
  struct ProjectSettings {
    bool OnInspect();
  };

  struct JoinSettings {
    bool OnInspect();
  };

  struct RasterizeSettings {
    bool debug_opaque = false;
    bool transfer_albedo_map = true;
    bool transfer_normal_map = true;
    bool transfer_roughness_map = true;
    bool transfer_metallic_map = true;
    bool transfer_ao_map = false;

    int dilate = 0;

    glm::ivec2 base_resolution = glm::ivec2(2048);
    glm::ivec2 output_albedo_resolution = glm::ivec2(2048);
    glm::ivec2 output_material_props_resolution = glm::ivec2(512);

    bool OnInspect();
  };

  enum class ClusterizationMode {
    FlipBook,
    Original,
    Foliage,
  };

  struct FoliageClusterizationSettings {
    float density = 0.9f;
    int iteration = 400;
    int timeout = 0;
    float sample_range = 1.f;

    bool fill_band = true;
    bool OnInspect();
  };
  struct OriginalClusterizationSettings {
    float epsilon_percentage = 0.01f;
    int discretization_size = 10;
    int timeout = 0;
    bool skip_remain_triangles = false;
    bool OnInspect();
  };

  struct ClusterizationSettings {
    bool append = true;
    FoliageClusterizationSettings foliage_clusterization_settings{};
    OriginalClusterizationSettings original_clusterization_settings{};
    unsigned clusterize_mode = static_cast<unsigned>(ClusterizationMode::Foliage);

    bool OnInspect();
  };

  struct GenerateSettings {
    ClusterizationSettings clusterization_settings{};
    ProjectSettings project_settings{};
    JoinSettings join_settings{};
    RasterizeSettings rasterize_settings{};

    bool OnInspect(const std::string& title);
  };

  struct Cluster {
    Plane cluster_plane = Plane(glm::vec3(0, 0, 1), 0.f);

    std::vector<ClusterTriangle> triangles;

    std::vector<ProjectedTriangle> projected_triangles;
    /**
     * Billboard's bounding rectangle.
     */
    Rectangle rectangle;
    /**
     * Billboard's corresponding mesh.
     */
    std::vector<Vertex> billboard_vertices;
    std::vector<glm::uvec3> billboard_triangles;
  };

  std::vector<ClusterTriangle> CollectTriangles() const;

  std::vector<ClusterTriangle> skipped_triangles;

  std::vector<Element> elements;
  std::vector<Cluster> clusters;

  std::shared_ptr<Mesh> billboard_cloud_mesh;
  std::shared_ptr<Material> billboard_cloud_material;

  void Clusterize(const ClusterizationSettings& clusterize_settings);
  void Project(const ProjectSettings& project_settings);
  void Join(const JoinSettings& join_settings);
  void Rasterize(const RasterizeSettings& rasterize_settings);

  void Generate(const GenerateSettings& generate_settings);
  void Process(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material);
  void Process(const std::shared_ptr<Prefab>& prefab);
  void Process(const std::shared_ptr<Scene>& scene, const Entity& entity);

  [[nodiscard]] Entity BuildEntity(const std::shared_ptr<Scene>& scene) const;

  [[nodiscard]] std::vector<glm::vec3> ExtractPointCloud(float density) const;

 private:
  [[nodiscard]] glm::vec3 CalculateCentroid(const ClusterTriangle& triangle) const;
  [[nodiscard]] float CalculateArea(const ClusterTriangle& triangle) const;
  [[nodiscard]] float CalculateNormalDistance(const ClusterTriangle& triangle) const;
  [[nodiscard]] glm::vec3 CalculateNormal(const ClusterTriangle& triangle) const;
  void Project(Cluster& cluster, const ProjectSettings& project_settings) const;
  std::vector<Cluster> StochasticClusterize(std::vector<ClusterTriangle> operating_triangles,
                                            const ClusterizationSettings& clusterize_settings);
  std::vector<Cluster> DefaultClusterize(std::vector<ClusterTriangle> operating_triangles,
                                         const ClusterizationSettings& clusterize_settings);

  void ProcessPrefab(const std::shared_ptr<Prefab>& current_prefab, const Transform& parent_model_space_transform);
  void ProcessEntity(const std::shared_ptr<Scene>& scene, const Entity& entity,
                     const Transform& parent_model_space_transform);

  // Adopted from https://github.com/DreamVersion/RotatingCalipers
  class RotatingCalipers {
   public:
    static std::vector<glm::vec2> ConvexHull(std::vector<glm::vec2> points);
    static Rectangle GetMinAreaRectangle(std::vector<glm::vec2> points);
  };
};

template <typename T>
void BillboardCloud::DilateChannels(const std::vector<size_t>& channels, std::vector<T>& data, std::vector<bool>& valid_pixels,
                            const size_t max_dist, const glm::uvec2& resolution, const bool diagonals) {
  const int w = static_cast<int>(resolution.x);
  const int h = static_cast<int>(resolution.y);
  int iteration = 0;
  while (max_dist == 0 || iteration < max_dist) {
    iteration++;
    bool any_pixel_updated = false;
    std::vector<bool> update_lists = valid_pixels;
    Jobs::RunParallelFor(data.size(), [&](const unsigned pixel_index) {
      if (valid_pixels[pixel_index])
        return;
      auto is_valid_pixel = [](const std::vector<bool>& vp, const int x_index, const int y_index, const int tex_width,
                               const int tex_height) {
        if (x_index < 0 || y_index < 0 || x_index >= tex_width || y_index >= tex_height) {
          return false;
        }
        return vp[x_index + y_index * tex_width];
      };

      const auto pixel_coord_x = static_cast<int>(pixel_index % w);
      const auto pixel_coord_y = static_cast<int>(pixel_index / w);

      bool is_valid = is_valid_pixel(valid_pixels, pixel_coord_x - 1, pixel_coord_y, w, h) ||
                      is_valid_pixel(valid_pixels, pixel_coord_x + 1, pixel_coord_y, w, h) ||
                      is_valid_pixel(valid_pixels, pixel_coord_x, pixel_coord_y + 1, w, h) ||
                      is_valid_pixel(valid_pixels, pixel_coord_x, pixel_coord_y - 1, w, h);

      if (diagonals) {
        is_valid = is_valid || is_valid_pixel(valid_pixels, pixel_coord_x - 1, pixel_coord_y - 1, w, h) ||
                   is_valid_pixel(valid_pixels, pixel_coord_x - 1, pixel_coord_y + 1, w, h) ||
                   is_valid_pixel(valid_pixels, pixel_coord_x + 1, pixel_coord_y - 1, w, h) ||
                   is_valid_pixel(valid_pixels, pixel_coord_x + 1, pixel_coord_y + 1, w, h);
      }

      if (is_valid) {
        float sum_weight = 0;
        T sum_color = {};
        for (int i = -1; i <= 1; i++) {
          for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0)
              continue;
            const auto test_x = pixel_coord_x + i;
            const auto test_y = pixel_coord_y + j;
            if (is_valid_pixel(valid_pixels, test_x, test_y, w, h)) {
              const float weight = 1.0f / static_cast<float>(abs(i) + abs(j));
              sum_weight += weight;
              const auto idx = test_x + w * test_y;
              sum_color += data[idx] * weight;
            }
          }
        }
        sum_color /= sum_weight;
        for (const auto& channel : channels) {
          data[pixel_index][channel] = sum_color[channel];
        }
        update_lists[pixel_index] = true;
        any_pixel_updated = true;
      }
    });
    valid_pixels = std::move(update_lists);
    if (!any_pixel_updated) {
      break;
    }
  }
}

template <typename T>
void BillboardCloud::Dilate(std::vector<T>& data, std::vector<bool>& valid_pixels, const size_t max_dist,
    const glm::uvec2& resolution, const bool diagonals) {
  const int w = static_cast<int>(resolution.x);
  const int h = static_cast<int>(resolution.y);
  int iteration = 0;
  while (max_dist == 0 || iteration < max_dist) {
    iteration++;
    bool any_pixel_updated = false;
    std::vector<bool> update_lists = valid_pixels;
    Jobs::RunParallelFor(data.size(), [&](const unsigned pixel_index) {
      if (valid_pixels[pixel_index])
        return;
      auto is_valid_pixel = [](const std::vector<bool>& vp, const int x_index, const int y_index, const int tex_width,
                               const int tex_height) {
        if (x_index < 0 || y_index < 0 || x_index >= tex_width || y_index >= tex_height) {
          return false;
        }
        return vp[x_index + y_index * tex_width];
      };

      const auto pixel_coord_x = static_cast<int>(pixel_index % w);
      const auto pixel_coord_y = static_cast<int>(pixel_index / w);

      bool is_valid = is_valid_pixel(valid_pixels, pixel_coord_x - 1, pixel_coord_y, w, h) ||
                      is_valid_pixel(valid_pixels, pixel_coord_x + 1, pixel_coord_y, w, h) ||
                      is_valid_pixel(valid_pixels, pixel_coord_x, pixel_coord_y + 1, w, h) ||
                      is_valid_pixel(valid_pixels, pixel_coord_x, pixel_coord_y - 1, w, h);

      if (diagonals) {
        is_valid = is_valid || is_valid_pixel(valid_pixels, pixel_coord_x - 1, pixel_coord_y - 1, w, h) ||
                   is_valid_pixel(valid_pixels, pixel_coord_x - 1, pixel_coord_y + 1, w, h) ||
                   is_valid_pixel(valid_pixels, pixel_coord_x + 1, pixel_coord_y - 1, w, h) ||
                   is_valid_pixel(valid_pixels, pixel_coord_x + 1, pixel_coord_y + 1, w, h);
      }

      if (is_valid) {
        float sum_weight = 0;
        T sum_color = {};
        for (int i = -1; i <= 1; i++) {
          for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0)
              continue;
            const auto test_x = pixel_coord_x + i;
            const auto test_y = pixel_coord_y + j;
            if (is_valid_pixel(valid_pixels, test_x, test_y, w, h)) {
              const float weight = 1.0f / static_cast<float>(abs(i) + abs(j));
              sum_weight += weight;
              const auto idx = test_x + w * test_y;
              sum_color += data[idx] * weight;
            }
          }
        }
        sum_color /= sum_weight;
        data[pixel_index] = sum_color;
        update_lists[pixel_index] = true;
        any_pixel_updated = true;
      }
    });
    valid_pixels = std::move(update_lists);
    if (!any_pixel_updated) {
      break;
    }
  }
}
}  // namespace evo_engine