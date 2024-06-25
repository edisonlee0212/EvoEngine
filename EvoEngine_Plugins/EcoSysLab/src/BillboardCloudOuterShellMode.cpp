#include "BillboardCloud.hpp"
#include "Dense"
using namespace evo_engine;

class Discretization {
 public:
  static Eigen::Vector3f ConvertVec3(const glm::vec3& point) {
    return {point.x, point.y, point.z};
  }

  static glm::vec3 ConvertVec3(const Eigen::Vector3f& point) {
    return {point(0), point(1), point(2)};
  }

  static std::vector<Eigen::Vector3f> ConvertVec3List(const std::vector<glm::vec3>& points) {
    std::vector<Eigen::Vector3f> data;
    data.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
      data[i] = ConvertVec3(points[i]);
    }
    return data;
  }

  static std::vector<glm::vec3> ConvertVec3List(const std::vector<Eigen::Vector3f>& points) {
    std::vector<glm::vec3> data;
    data.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
      data[i] = ConvertVec3(points[i]);
    }
    return data;
  }

  static std::pair<glm::vec3, glm::vec3> FitPlaneFromPoints(const std::vector<glm::vec3>& points) {
    const auto converted_points = ConvertVec3List(points);

    // copy coordinates to matrix in Eigen format
    size_t point_size = converted_points.size();
    // Eigen::Matrix< Vector3::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
    Eigen::MatrixXf coord(3, point_size);
    for (size_t i = 0; i < point_size; ++i)
      coord.col(i) = converted_points[i];

    // calculate centroid
    Eigen::Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    // subtract centroid
    coord.row(0).array() -= centroid(0);
    coord.row(1).array() -= centroid(1);
    coord.row(2).array() -= centroid(2);

    // we only need the left-singular matrix here
    //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(coord, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3f plane_normal = svd.matrixU().rightCols<1>();

    return std::make_pair(ConvertVec3(centroid), ConvertVec3(plane_normal));
  }
  class Bin {
   public:
    float density;
    float theta_min;
    float theta_max;
    float phi_min;
    float phi_max;
    float ro_min;
    float ro_max;
    float theta_center;
    float phi_center;
    float ro_center;
    glm::vec3 center_normal;

    Bin()
        : density(0.0f),
          theta_min(0.0f),
          theta_max(0.0f),
          phi_min(0.0f),
          phi_max(0.0f),
          ro_min(0.0f),
          ro_max(0.0f),
          theta_center(0.0f),
          phi_center(0.0f),
          ro_center(0.0f),
          center_normal(0.f) {
    }

    Bin(const float theta_min, const float theta_max, const float phi_min, const float phi_max, const float ro_min,
        const float ro_max, const float density = 0)
        : density(density),
          theta_min(theta_min),
          theta_max(theta_max),
          phi_min(phi_min),
          phi_max(phi_max),
          ro_min(ro_min),
          ro_max(ro_max) {
      CalculateCenter();
      CalculateCenterNormal();
    }

    Bin(const Bin& bin) {
      density = bin.density;
      theta_min = bin.theta_min;
      theta_max = bin.theta_max;
      phi_min = bin.phi_min;
      phi_max = bin.phi_max;
      ro_min = bin.ro_min;
      ro_max = bin.ro_max;
      theta_center = bin.theta_center;
      phi_center = bin.phi_center;
      ro_center = bin.ro_center;
      center_normal = bin.center_normal;
    }

    Bin& operator=(const Bin& bin) = default;

   private:
    void CalculateCenter() {
      theta_center = (theta_min + theta_max) / 2;
      phi_center = (phi_min + phi_max) / 2;
      ro_center = (ro_min + ro_max) / 2;
    }

    // calculate bin center's normal
    void CalculateCenterNormal() {
      const float z = glm::sin(phi_center);
      const float xy = glm::cos(phi_center);
      const float x = xy * glm::cos(theta_center);
      const float y = xy * glm::sin(theta_center);

      center_normal = glm::vec3(x, y, z);
    }
  };
  /// m_bins
  std::vector<std::vector<std::vector<Bin>>> m_bins;
  /// fail-safe mode para
  bool fail_safe_mode_triggered;
  /// fitted plane in fail-safe mode
  std::vector<BillboardCloud::ClusterTriangle> best_fitted_plane_valid_triangle;

  float epsilon;
  float weight_penalty;
  /// discretization num
  int discretize_theta_num;
  int discretize_phi_num;
  int discretize_ro_num;
  /// range
  float theta_max;
  float theta_min;
  float phi_max;
  float phi_min;
  float ro_max;
  float ro_min;
  /// gap
  float theta_gap;
  float phi_gap;
  float ro_gap;

  Discretization(const float ro_max, const float epsilon, const int discretize_theta_num, const int discretize_phi_num,
                 const int discretize_ro_num)
      : fail_safe_mode_triggered(false),
        epsilon(epsilon),
        weight_penalty(10),
        discretize_theta_num(discretize_theta_num),
        discretize_phi_num(discretize_phi_num),
        discretize_ro_num(discretize_ro_num),
        theta_max(2 * glm::pi<float>()),
        theta_min(0),
        phi_max(glm::pi<float>() / 2),  // recommend "10" in paper
        phi_min(-glm::pi<float>() / 2),
        ro_max(ro_max),
        ro_min(0) {
    theta_gap = (theta_max - theta_min) / static_cast<float>(discretize_theta_num);
    phi_gap = (phi_max - phi_min) / static_cast<float>(discretize_phi_num);
    ro_gap = (ro_max - ro_min) / static_cast<float>(discretize_ro_num);

    for (int i = 0; i < discretize_ro_num; i++) {
      std::vector<std::vector<Bin>> tmp1;
      for (int j = 0; j < discretize_phi_num; j++) {
        std::vector<Bin> tmp2;
        for (int k = 0; k < discretize_theta_num; k++) {
          const float theta_min_tmp = theta_gap * k + theta_min;
          const float theta_max_tmp = theta_min_tmp + theta_gap;
          const float phi_min_tmp = phi_gap * j + phi_min;
          const float phi_max_tmp = phi_min_tmp + phi_gap;
          const float ro_min_tmp = ro_gap * i + ro_min;
          const float ro_max_tmp = ro_min_tmp + ro_gap;
          Bin bin(theta_min_tmp, theta_max_tmp, phi_min_tmp, phi_max_tmp, ro_min_tmp, ro_max_tmp);
          tmp2.emplace_back(bin);
        }
        tmp1.emplace_back(tmp2);
      }
      m_bins.emplace_back(tmp1);
    }
  }
  /// trans the spherical coordinate of a plane into the normal vector
  static glm::vec3 SphericalCoordToNormal(const float theta, const float phi) {
    const float z = glm::sin(phi);
    const float xy = glm::cos(phi);
    const float x = xy * glm::cos(theta);
    const float y = xy * glm::sin(theta);
    return {x, y, z};
  }
  /// compute the min and max value of ro in the case of triangle is valid for the specific theta and phi range
  [[nodiscard]] bool ComputeRoMinMax(glm::vec2& min_max, const BillboardCloud::Element& element,
                                     const BillboardCloud::ClusterTriangle& cluster_triangle, float cur_theta_min,
                                     float cur_theta_max, float cur_phi_min, float cur_phi_max) const {
    const auto normal1 = SphericalCoordToNormal(cur_theta_min, cur_phi_min);
    const auto normal2 = SphericalCoordToNormal(cur_theta_min, cur_phi_max);
    const auto normal3 = SphericalCoordToNormal(cur_theta_max, cur_phi_min);
    const auto normal4 = SphericalCoordToNormal(cur_theta_max, cur_phi_max);

    const auto& triangle = element.triangles.at(cluster_triangle.triangle_index);
    const auto p0 = element.vertices.at(triangle.x).position;
    const auto p1 = element.vertices.at(triangle.y).position;
    const auto p2 = element.vertices.at(triangle.z).position;

    const float ro_p0_n1 = glm::dot(p0, normal1);
    const float ro_p0_n2 = glm::dot(p0, normal2);
    const float ro_p0_n3 = glm::dot(p0, normal3);
    const float ro_p0_n4 = glm::dot(p0, normal4);

    const float ro_p1_n1 = glm::dot(p1, normal1);
    const float ro_p1_n2 = glm::dot(p1, normal2);
    const float ro_p1_n3 = glm::dot(p1, normal3);
    const float ro_p1_n4 = glm::dot(p1, normal4);

    const float ro_p2_n1 = glm::dot(p2, normal1);
    const float ro_p2_n2 = glm::dot(p2, normal2);
    const float ro_p2_n3 = glm::dot(p2, normal3);
    const float ro_p2_n4 = glm::dot(p2, normal4);

    const float tmp0[] = {ro_p0_n1 - epsilon, ro_p0_n2 - epsilon, ro_p0_n3 - epsilon, ro_p0_n4 - epsilon};
    const float tmp1[] = {ro_p1_n1 - epsilon, ro_p1_n2 - epsilon, ro_p1_n3 - epsilon, ro_p1_n4 - epsilon};
    const float tmp2[] = {ro_p2_n1 - epsilon, ro_p2_n2 - epsilon, ro_p2_n3 - epsilon, ro_p2_n4 - epsilon};
    const float ro_p0_min_n = glm::min(glm::min(tmp0[0], tmp0[1]), glm::min(tmp0[2], tmp0[3]));
    const float ro_p1_min_n = glm::min(glm::min(tmp1[0], tmp1[1]), glm::min(tmp1[2], tmp1[3]));
    const float ro_p2_min_n = glm::min(glm::min(tmp2[0], tmp2[1]), glm::min(tmp2[2], tmp2[3]));
    const float tmp3[] = {ro_p0_min_n, ro_p1_min_n, ro_p2_min_n};
    // roMin

    float ro_max_p_min_n = glm::max(tmp3[0], glm::max(tmp3[1], tmp3[2]));

    const float tmp4[] = {ro_p0_n1 + epsilon, ro_p0_n2 + epsilon, ro_p0_n3 + epsilon, ro_p0_n4 + epsilon};
    const float tmp5[] = {ro_p1_n1 + epsilon, ro_p1_n2 + epsilon, ro_p1_n3 + epsilon, ro_p1_n4 + epsilon};
    const float tmp6[] = {ro_p2_n1 + epsilon, ro_p2_n2 + epsilon, ro_p2_n3 + epsilon, ro_p2_n4 + epsilon};
    const float ro_p0_max_n = glm::max(glm::max(tmp4[0], tmp4[1]), glm::max(tmp4[2], tmp4[3]));
    const float ro_p1_max_n = glm::max(glm::max(tmp5[0], tmp5[1]), glm::max(tmp5[2], tmp5[3]));
    const float ro_p2_max_n = glm::max(glm::max(tmp6[0], tmp6[1]), glm::max(tmp6[2], tmp6[3]));
    const float tmp7[] = {ro_p0_max_n, ro_p1_max_n, ro_p2_max_n};
    // roMax
    float ro_min_p_max_n = glm::min(tmp7[0], glm::min(tmp7[1], tmp7[2]));
    if (ro_min_p_max_n < ro_min)
      return false;

    ro_max_p_min_n = glm::clamp(ro_max_p_min_n, ro_min, ro_max);
    ro_min_p_max_n = glm::min(ro_min_p_max_n, ro_max);
    min_max = {ro_max_p_min_n, ro_min_p_max_n};
    return true;
  }
  void UpdateDensity(const std::vector<BillboardCloud::Element>& elements,
                     const std::vector<BillboardCloud::ClusterTriangle>& cluster_triangles, const bool add) {
    if (cluster_triangles.empty()) {
      EVOENGINE_ERROR(
          "ERROR: The size of the input triangles is 0, that means in the last iteration, there is no fitted plane "
          "found!")
      return;
    }

    const float time = clock();
    for (auto& cluster_triangle : cluster_triangles) {
      const auto& element = elements.at(cluster_triangle.element_index);
      const auto triangle_normal = element.CalculateNormal(cluster_triangle.triangle_index);
      const auto triangle_area = element.CalculateArea(cluster_triangle.triangle_index);
      for (int i = 0; i < discretize_phi_num; i++)  // phiCoord
      {
        for (int j = 0; j < discretize_theta_num; j++)  // thetaCoord
        {
          glm::vec2 ro_min_max;
          if (!ComputeRoMinMax(ro_min_max, element, cluster_triangle, theta_min + j * theta_gap,
                               theta_min + (j + 1) * theta_gap, phi_min + i * phi_gap,
                               phi_min + (i + 1) * phi_gap)) {
            continue;
          }
          // if (roMaxMin.x < 0.f && roMaxMin.y < 0.f)
          //	continue;
          const float current_ro_min = ro_min_max.x;
          const float current_ro_max = ro_min_max.y;

          // add coverage, m_bins between (roMin, roMax)
          const int ro_coord_min = glm::clamp(static_cast<int>((current_ro_min - current_ro_min) / ro_gap), 0, discretize_ro_num - 1);
          const int ro_coord_max = glm::clamp(static_cast<int>((current_ro_max - current_ro_min) / ro_gap), 0, discretize_ro_num - 1);

          if (ro_coord_max - ro_coord_min > 2) {
            if (add) {
              // use the center point's normal of the bin to calculate the projected triangle area
              m_bins[ro_coord_min][i][j].density +=
                  triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[ro_coord_min][i][j].center_normal)) *
                  ((current_ro_min + static_cast<float>(ro_coord_min + 1) * ro_gap - current_ro_min) / ro_gap);
            } else {
              m_bins[ro_coord_min][i][j].density -=
                  triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[ro_coord_min][i][j].center_normal)) *
                  ((current_ro_min + static_cast<float>(ro_coord_min + 1) * ro_gap - current_ro_min) / ro_gap);
            }
            for (int k = ro_coord_min + 1; k < ro_coord_max; k++) {
              if (add) {
                m_bins[k][i][j].density +=
                    triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[k][i][j].center_normal));
              } else {
                m_bins[k][i][j].density -=
                    triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[k][i][j].center_normal));
              }
            }
            if (add) {
              m_bins[ro_coord_max][i][j].density +=
                  triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[ro_coord_max][i][j].center_normal)) *
                  ((current_ro_max - (static_cast<float>(ro_coord_max) * ro_gap + current_ro_min)) / ro_gap);
            } else {
              m_bins[ro_coord_max][i][j].density -=
                  triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[ro_coord_max][i][j].center_normal)) *
                  ((current_ro_max - (static_cast<float>(ro_coord_max) * ro_gap + current_ro_min)) / ro_gap);
            }
          } else if (ro_coord_max - ro_coord_min == 1) {
            if (add) {
              m_bins[ro_coord_min][i][j].density +=
                  triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[ro_coord_min][i][j].center_normal)) *
                  ((current_ro_min + static_cast<float>(ro_coord_min + 1) * ro_gap - current_ro_min) / ro_gap);
            } else {
              m_bins[ro_coord_min][i][j].density -=
                  triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[ro_coord_min][i][j].center_normal)) *
                  ((current_ro_min + static_cast<float>(ro_coord_min + 1) * ro_gap - current_ro_min) / ro_gap);
            }

            if (add) {
              m_bins[ro_coord_max][i][j].density +=
                  triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[ro_coord_max][i][j].center_normal)) *
                  ((current_ro_max - (static_cast<float>(ro_coord_max) * ro_gap + current_ro_min)) / ro_gap);
            } else {
              m_bins[ro_coord_max][i][j].density -=
                  triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[ro_coord_max][i][j].center_normal)) *
                  ((current_ro_max - (static_cast<float>(ro_coord_max) * ro_gap + current_ro_min)) / ro_gap);
            }
          } else if (ro_coord_max - ro_coord_min == 0) {
            if (add) {
              m_bins[ro_coord_min][i][j].density +=
                  triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[ro_coord_min][i][j].center_normal));
            } else {
              m_bins[ro_coord_min][i][j].density -=
                  triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[ro_coord_min][i][j].center_normal));
            }
          }

          // add penalty ,m_bins between (ro_min - epsilon, ro_min)
          if (current_ro_min - epsilon - current_ro_min > 0) {
            const int ro_min_minus_epsilon_coord = (current_ro_min - epsilon - current_ro_min) / ro_gap;
            for (int m = ro_min_minus_epsilon_coord; m <= ro_coord_min; m++) {
              if (add) {
                m_bins[m][i][j].density -=
                    triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[m][i][j].center_normal)) * weight_penalty;
              } else {
                m_bins[m][i][j].density +=
                    triangle_area * glm::abs(glm::dot(triangle_normal, m_bins[m][i][j].center_normal)) * weight_penalty;
              }
            }
          }
        }
      }
    }
    EVOENGINE_LOG("Updating_density... [time :" + std::to_string((clock() - time) / 1000.f) + "s]");
  }

  [[nodiscard]] float ComputeMaxDensity(glm::ivec3& bin_index) const {
    // pick bin with max density
    float max_density = -FLT_MAX;
    for (int i = 0; i < discretize_ro_num; i++) {
      for (int j = 0; j < discretize_phi_num; j++) {
        for (int k = 0; k < discretize_theta_num; k++) {
          if (float tmp = m_bins[i][j][k].density; max_density < tmp) {
            max_density = tmp;
            bin_index.x = i;
            bin_index.y = j;
            bin_index.z = k;
          }
        }
      }
    }
    return max_density;
  }

  [[nodiscard]] std::vector<BillboardCloud::ClusterTriangle> ComputeBinValidSet(
      const std::vector<BillboardCloud::Element>& elements,
      const std::vector<BillboardCloud::ClusterTriangle>& cluster_triangles, const Bin& bin) const {
    std::vector<BillboardCloud::ClusterTriangle> bin_valid_set;
    for (auto& cluster_triangle : cluster_triangles) {
      const auto& element = elements.at(cluster_triangle.element_index);
      // we use the notion of "simple validity":
      // that is a bin is valid for a triangle as long as there exists a valid plane for the triangle in the bin
      // if the ro min and ro max is in the range of bin's ro range, we think this triangle is valid for the bin
      glm::vec2 current_ro_min_max;
      if (!ComputeRoMinMax(current_ro_min_max, element, cluster_triangle, bin.theta_min, bin.theta_max, bin.phi_min,
                           bin.phi_max)) {
        continue;
      }

      if (!(current_ro_min_max.y < bin.ro_min) && !(current_ro_min_max.x > bin.ro_max)) {
        bin_valid_set.emplace_back(cluster_triangle);
      }
    }
    return bin_valid_set;
  }
  [[nodiscard]] std::vector<int> ComputePlaneValidSetIndex(const std::vector<BillboardCloud::Element>& elements,
                                             const std::vector<BillboardCloud::ClusterTriangle>& cluster_triangles,
                                             const Plane& plane) const {
    std::vector<int> plane_valid_set_index;
    const auto plane_normal = plane.GetNormal();
    const auto plane_distance = plane.GetDistance();
    for (int i = 0; i < cluster_triangles.size(); i++) {
      const auto& cluster_triangle = cluster_triangles.at(i);
      const auto& element = elements.at(cluster_triangle.element_index);
      const auto& triangle = element.triangles.at(cluster_triangle.triangle_index);
      const auto p0 = element.vertices.at(triangle.x).position;
      const auto p1 = element.vertices.at(triangle.y).position;
      const auto p2 = element.vertices.at(triangle.z).position;

      const float ro_p0_n = glm::abs(glm::dot(p0, plane_normal));
      const float ro_p1_n = glm::abs(glm::dot(p1, plane_normal));
      const float ro_p2_n = glm::abs(glm::dot(p2, plane_normal));
      const float tmp0[] = {ro_p0_n - epsilon, ro_p1_n - epsilon, ro_p2_n - epsilon};
      const float tmp1[] = {ro_p0_n + epsilon, ro_p1_n + epsilon, ro_p2_n + epsilon};

      const float ro_min_tmp = glm::min(tmp0[0], glm::min(tmp0[1], tmp0[2]));
      const float ro_max_tmp = glm::max(tmp1[0], glm::max(tmp1[1], tmp1[2]));

      if (plane_distance > ro_min_tmp && plane_distance < ro_max_tmp)
        plane_valid_set_index.emplace_back(i);
    }
    return plane_valid_set_index;
  }
  void ComputeDensity(const std::vector<BillboardCloud::Element>& elements,
                      const std::vector<BillboardCloud::ClusterTriangle>& cluster_triangles, Bin& bin) const {
    for (auto& cluster_triangle : cluster_triangles) {
      const auto& element = elements.at(cluster_triangle.element_index);
      const auto triangle_normal = element.CalculateNormal(cluster_triangle.triangle_index);
      const auto triangle_area = element.CalculateArea(cluster_triangle.triangle_index);
      glm::vec2 ro_min_max;
      if (!ComputeRoMinMax(ro_min_max, element, cluster_triangle, bin.theta_min, bin.theta_max, bin.phi_min,
                           bin.phi_max)) {
        continue;
      }
      // add coverage
      const float cur_ro_min = ro_min_max.x;
      const float cur_ro_max = ro_min_max.y;
      const float cur_ro_gap = bin.ro_max - bin.ro_min;
      if (cur_ro_min < bin.ro_min && cur_ro_max > bin.ro_min && cur_ro_max < bin.ro_max) {
        bin.density +=
            triangle_area * glm::abs(glm::dot(triangle_normal, bin.center_normal)) * (cur_ro_max - bin.ro_min) / cur_ro_gap;
      } else if (cur_ro_min > bin.ro_min && cur_ro_min < bin.ro_max && cur_ro_max > bin.ro_max) {
        bin.density +=
            triangle_area * glm::abs(glm::dot(triangle_normal, bin.center_normal)) * (bin.ro_max - cur_ro_min) / cur_ro_gap;
      } else if (cur_ro_min >= bin.ro_min && cur_ro_max <= bin.ro_max) {
        bin.density += triangle_area * glm::abs(glm::dot(triangle_normal, bin.center_normal));
      }
    }
  }

  std::vector<Bin> GetNeighbors(const Bin& bin) const {
    const float cur_bin_theta_gap = bin.theta_max - bin.theta_min;
    const float cur_bin_phi_gap = bin.phi_max - bin.phi_min;
    const float cur_bin_ro_gap = bin.ro_max - bin.ro_min;

    std::vector<Bin> bins_tmp;
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          float neighbor_theta_min = bin.theta_min + cur_bin_theta_gap * k;
          float neighbor_theta_max = neighbor_theta_min + cur_bin_theta_gap;
          float neighbor_phi_min = bin.phi_min + cur_bin_phi_gap * j;
          float neighbor_phi_max = neighbor_phi_min + cur_bin_phi_gap;
          const float neighbor_ro_min = bin.ro_min + cur_bin_ro_gap * i;
          const float neighbor_ro_max = neighbor_ro_min + cur_bin_ro_gap;

          if (neighbor_phi_min < phi_min) {
            neighbor_phi_min = -(neighbor_phi_min - phi_min);
            neighbor_theta_min = neighbor_theta_min + glm::pi<float>();
          }
          if (neighbor_phi_max > phi_max) {
            neighbor_phi_max = glm::pi<float>() / 2 - (neighbor_phi_max - phi_max);
            neighbor_theta_min = neighbor_theta_min + glm::pi<float>();
          }

          if (neighbor_theta_min > 2 * glm::pi<float>()) {
            neighbor_theta_min = neighbor_theta_min - 2 * glm::pi<float>();
          }
          if (neighbor_theta_max > 2 * glm::pi<float>()) {
            neighbor_theta_max = neighbor_theta_max - 2 * glm::pi<float>();
          }

          // if the bin's ro range is outside the specific range, discard it!
          if (neighbor_ro_min < ro_min || neighbor_ro_max > ro_max)
            continue;

          Bin bin_tmp(neighbor_theta_min, neighbor_theta_max, neighbor_phi_min, neighbor_phi_max, neighbor_ro_min, neighbor_ro_max);
          bins_tmp.emplace_back(bin_tmp);
        }
      }
    }
    return bins_tmp;
  }

  static std::vector<Bin> SubdivideBin(const Bin& bin) {
    std::vector<Bin> bins_tmp;
    for (int i = 0; i <= 1; i++) {
      for (int j = 0; j <= 1; j++) {
        for (int k = 0; k <= 1; k++) {
          const float cur_theta_min = bin.theta_min + (bin.theta_max - bin.theta_min) / 2 * k;
          const float cur_theta_max = cur_theta_min + (bin.theta_max - bin.theta_min) / 2;
          const float cur_phi_min = bin.phi_min + (bin.phi_max - bin.phi_min) / 2 * j;
          const float cur_phi_max = cur_phi_min + (bin.phi_max - bin.phi_min) / 2;
          const float cur_ro_min = bin.ro_min + (bin.ro_max - bin.ro_min) / 2 * i;
          const float cur_ro_max = cur_ro_min + (bin.ro_max - bin.ro_min) / 2;

          Bin bin_tmp(cur_theta_min, cur_theta_max, cur_phi_min, cur_phi_max, cur_ro_min, cur_ro_max);
          bins_tmp.emplace_back(bin_tmp);
        }
      }
    }
    return bins_tmp;
  }
  [[nodiscard]] Plane RefineBin(const std::vector<BillboardCloud::Element>& elements,
                                const std::vector<BillboardCloud::ClusterTriangle>& valid_set,
                                const Bin& max_density_bin) {
    Plane center_plane{max_density_bin.center_normal, max_density_bin.ro_center};
    std::vector<int> center_plane_valid_set_index = ComputePlaneValidSetIndex(elements, valid_set, center_plane);
    if (center_plane_valid_set_index.size() == valid_set.size()) {
      if (max_density_bin.theta_center > glm::pi<float>()) {
        center_plane = Plane(-center_plane.GetNormal(), center_plane.GetDistance());
      }
      float max_dis = 0.0f;
      for (auto& cluster_triangle : valid_set) {
        const auto& element = elements.at(cluster_triangle.element_index);
        const auto& triangle = element.triangles.at(cluster_triangle.triangle_index);
        const auto p0 = element.vertices.at(triangle.x).position;
        const auto p1 = element.vertices.at(triangle.y).position;
        const auto p2 = element.vertices.at(triangle.z).position;
        float d0 = center_plane.CalculatePointDistance(p0);
        float d1 = center_plane.CalculatePointDistance(p1);
        float d2 = center_plane.CalculatePointDistance(p2);
        max_dis = d0 > d1 ? d0 : d1;
        max_dis = max_dis > d2 ? max_dis : d2;
      }
      return center_plane;
    }

    Bin bin_max;
    bin_max.density = FLT_MIN;

    // pick the bin and its 26 neighbors (if have)
    std::vector<Bin> neighbor_bins = GetNeighbors(max_density_bin);
    for (auto& neighbor_bin : neighbor_bins) {
      // subdivide the bin into 8 bins
      std::vector<Bin> subdivided_bins = SubdivideBin(neighbor_bin);
      for (auto& subdivided_bin : subdivided_bins) {
        // pick the subdivide bin with max density
        ComputeDensity(elements, valid_set, subdivided_bin);
        if (subdivided_bin.density > bin_max.density) {
          bin_max = subdivided_bin;
        }
      }
    }
    std::vector<BillboardCloud::ClusterTriangle> bin_max_valid_set = ComputeBinValidSet(elements, valid_set, bin_max);
    if (bin_max_valid_set.empty()) {
      EVOENGINE_ERROR("ERROR: subBinMax has no valid set, we will simply return the last densest bin's center plane!")

      // if the centerPlane has no valid set in the current remain sets, the iteration will end up with infinite loop!!!
      if (!center_plane_valid_set_index.empty()) {
        EVOENGINE_ERROR("INFO: but last densest bin's center plane has valid set")
        EVOENGINE_ERROR("INFO: so we can simply return the last densest bin's center plane!")

        return center_plane;
      } else {
        EVOENGINE_ERROR("ERROR: the centerPlane has no valid set in the current remain sets too")
        EVOENGINE_ERROR("INFO: so we return the best fitted plane of the last densest bin's valid set ")

        fail_safe_mode_triggered = true;
        best_fitted_plane_valid_triangle = valid_set;

        std::vector<glm::vec3> points;
        for (auto& cluster_triangle : valid_set) {
          const auto& element = elements.at(cluster_triangle.element_index);
          const auto& triangle = element.triangles.at(cluster_triangle.triangle_index);
          points.emplace_back(element.vertices.at(triangle.x).position);
          points.emplace_back(element.vertices.at(triangle.y).position);
          points.emplace_back(element.vertices.at(triangle.z).position);
        }

        auto fitted = FitPlaneFromPoints(points);

        auto centroid = fitted.first;
        auto normal = fitted.second;
        if (glm::dot(centroid, normal) < 0) {
          normal = -normal;
        }
        float distance = glm::abs(glm::dot(centroid, normal));
        return {normal, distance};
      }
    }
    return RefineBin(elements, bin_max_valid_set, bin_max);
  }
};

std::vector<BillboardCloud::Cluster> BillboardCloud::DefaultClusterize(
    std::vector<ClusterTriangle> operating_triangles, const ClusterizationSettings& clusterize_settings) {
  BoundingSphere bounding_sphere;
  bounding_sphere.Initialize(elements);
  const auto& settings = clusterize_settings.original_clusterization_settings;
  const int ro_num = static_cast<int>(1.5f / settings.epsilon_percentage);
  const float epsilon = bounding_sphere.radius * settings.epsilon_percentage;

  float max_normal_distance = 0.f;
  for (const auto& triangle : operating_triangles) {
    max_normal_distance = glm::max(max_normal_distance, CalculateNormalDistance(triangle));
  }

  skipped_triangles.clear();

  std::vector<Cluster> ret_val;

  int epoch = 0;

  Discretization discretization(max_normal_distance, epsilon, settings.discretization_size,
                                settings.discretization_size, ro_num);
  discretization.UpdateDensity(elements, operating_triangles, true);
  bool updated = true;
  while (!operating_triangles.empty() && updated) {
    updated = false;
    for (int i = 0; i < operating_triangles.size(); i++) {
      operating_triangles[i].index = i;
    }
    glm::ivec3 max_density_bin_coordinate;
    [[maybe_unused]] float max_density = discretization.ComputeMaxDensity(max_density_bin_coordinate);
    const auto& max_density_bin =
        discretization.m_bins[max_density_bin_coordinate.x][max_density_bin_coordinate.y][max_density_bin_coordinate.z];
    if (const auto bin_valid_set = discretization.ComputeBinValidSet(elements, operating_triangles, max_density_bin); !bin_valid_set.empty()) {
      std::vector<int> selected_triangle_indices;
      Cluster new_cluster;
      std::vector<ClusterTriangle> plane_valid_set;
      new_cluster.cluster_plane = discretization.RefineBin(elements, bin_valid_set, max_density_bin);

      if (discretization.fail_safe_mode_triggered) {
        plane_valid_set = discretization.best_fitted_plane_valid_triangle;
        discretization.best_fitted_plane_valid_triangle.clear();

        EVOENGINE_ERROR("Fitted_triangle_num: " + std::to_string(plane_valid_set.size()))

        // update density by removing the fitted triangle
        discretization.UpdateDensity(elements, plane_valid_set, false);

        // store bbc and corresponding fitted triangles
        new_cluster.triangles = plane_valid_set;
        // bbc.emplace_back(refinedPlane);

        for (auto& triangle : plane_valid_set) {
          selected_triangle_indices.emplace_back(triangle.index);
        }
        discretization.fail_safe_mode_triggered = false;
      } else {
        // get the fitted triangles index in the whole triangles
        selected_triangle_indices =
            discretization.ComputePlaneValidSetIndex(elements, operating_triangles, new_cluster.cluster_plane);

        EVOENGINE_ERROR("Fitted_triangle_num: " + std::to_string(selected_triangle_indices.size()))

        for (int index : selected_triangle_indices) {
          plane_valid_set.emplace_back(operating_triangles[index]);
        }

        // update density by removing the fitted triangles
        discretization.UpdateDensity(elements, plane_valid_set, false);

        // store bbc and corresponding fitted triangles
        new_cluster.triangles = plane_valid_set;
      }
      updated = true;
      ret_val.emplace_back(std::move(new_cluster));
      // Remove selected triangle from the remaining triangle.
      for (auto it = selected_triangle_indices.rbegin(); it != selected_triangle_indices.rend(); ++it) {
        operating_triangles[*it] = operating_triangles.back();
        operating_triangles.pop_back();
      }
      epoch++;
      if (settings.timeout != 0 && epoch >= settings.timeout) {
        EVOENGINE_ERROR("Default clustering timeout!")
        break;
      }
    }
  }
  skipped_triangles.clear();
  if (settings.skip_remain_triangles) {
    skipped_triangles = operating_triangles;
  } else {
    Cluster new_cluster;
    std::vector<glm::vec3> points;
    for (auto& cluster_triangle : operating_triangles) {
      const auto& element = elements.at(cluster_triangle.element_index);
      const auto& triangle = element.triangles.at(cluster_triangle.triangle_index);
      points.emplace_back(element.vertices.at(triangle.x).position);
      points.emplace_back(element.vertices.at(triangle.y).position);
      points.emplace_back(element.vertices.at(triangle.z).position);
    }

    const auto fitted = Discretization::FitPlaneFromPoints(points);

    auto centroid = fitted.first;
    auto normal = fitted.second;
    if (glm::dot(centroid, normal) < 0) {
      normal = -normal;
    }
    float distance = glm::abs(glm::dot(centroid, normal));
    new_cluster.cluster_plane = {normal, distance};
    new_cluster.triangles = operating_triangles;
    ret_val.emplace_back(std::move(new_cluster));
  }
  return ret_val;
}