#include "BillboardCloud.hpp"
#include "Prefab.hpp"
#include "xatlas.h"
using namespace billboard_clouds_plugin;
#pragma region Projection

std::vector<BillboardCloud::ClusterTriangle> BillboardCloud::CollectTriangles() const {
  std::vector<ClusterTriangle> ret_val;
  for (int element_index = 0; element_index < elements.size(); element_index++) {
    const auto& element = elements.at(element_index);
    for (int triangle_index = 0; triangle_index < element.triangles.size(); triangle_index++) {
      ClusterTriangle cluster_triangle;
      cluster_triangle.element_index = element_index;
      cluster_triangle.triangle_index = triangle_index;
      ret_val.emplace_back(cluster_triangle);
    }
  }
  return ret_val;
}

glm::vec3 BillboardCloud::CalculateCentroid(const ClusterTriangle& triangle) const {
  return elements.at(triangle.element_index).CalculateCentroid(triangle.triangle_index);
}

float BillboardCloud::CalculateArea(const ClusterTriangle& triangle) const {
  return elements.at(triangle.element_index).CalculateArea(triangle.triangle_index);
}

float BillboardCloud::CalculateNormalDistance(const ClusterTriangle& triangle) const {
  return elements.at(triangle.element_index).CalculateNormalDistance(triangle.triangle_index);
}

glm::vec3 BillboardCloud::CalculateNormal(const ClusterTriangle& triangle) const {
  return elements.at(triangle.element_index).CalculateNormal(triangle.triangle_index);
}

inline void TransformVertex(const Vertex& v, Vertex& t_v, const glm::mat4& transform) {
  t_v = v;
  t_v.normal = glm::normalize(transform * glm::vec4(v.normal, 0.f));
  t_v.tangent = glm::normalize(transform * glm::vec4(v.tangent, 0.f));
  t_v.position = transform * glm::vec4(v.position, 1.f);
}

inline void TransformVertex(Vertex& v, const glm::mat4& transform) {
  v.normal = glm::normalize(transform * glm::vec4(v.normal, 0.f));
  v.tangent = glm::normalize(transform * glm::vec4(v.tangent, 0.f));
  v.position = transform * glm::vec4(v.position, 1.f);
}

glm::vec3 BillboardCloud::Element::CalculateCentroid(const int triangle_index) const {
  return CalculateCentroid(triangles.at(triangle_index));
}

float BillboardCloud::Element::CalculateArea(const int triangle_index) const {
  return CalculateArea(triangles.at(triangle_index));
}

float BillboardCloud::Element::CalculateNormalDistance(const int triangle_index) const {
  const auto centroid = CalculateCentroid(triangle_index);
  auto normal = CalculateNormal(triangle_index);
  if (glm::dot(centroid, normal) < 0) {
    normal = -normal;
  }
  return glm::abs(glm::dot(centroid, normal));
}

glm::vec3 BillboardCloud::Element::CalculateNormal(const int triangle_index) const {
  return CalculateNormal(triangles.at(triangle_index));
}

glm::vec3 BillboardCloud::Element::CalculateCentroid(const glm::uvec3& triangle) const {
  const auto& a = vertices[triangle.x].position;
  const auto& b = vertices[triangle.y].position;
  const auto& c = vertices[triangle.z].position;

  return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3};
}

float BillboardCloud::Element::CalculateArea(const glm::uvec3& triangle) const {
  const auto& p0 = vertices[triangle.x].position;
  const auto& p1 = vertices[triangle.y].position;
  const auto& p2 = vertices[triangle.z].position;
  const float a = glm::length(p0 - p1);
  const float b = glm::length(p2 - p1);
  const float c = glm::length(p0 - p2);
  const float d = (a + b + c) / 2;
  return glm::sqrt(d * (d - a) * (d - b) * (d - c));
}

glm::vec3 BillboardCloud::Element::CalculateNormal(const glm::uvec3& triangle) const {
  const auto& p0 = vertices[triangle.x].position;
  const auto& p1 = vertices[triangle.y].position;
  const auto& p2 = vertices[triangle.z].position;
  return glm::normalize(glm::cross(p0 - p1, p0 - p2));
}

float BillboardCloud::Element::CalculateNormalDistance(const glm::uvec3& triangle) const {
  const auto centroid = CalculateCentroid(triangle);
  auto normal = CalculateNormal(triangle);
  if (glm::dot(centroid, normal) < 0) {
    normal = -normal;
  }
  return glm::abs(glm::dot(centroid, normal));
}

void BillboardCloud::Rectangle::Update() {
  center = (points[0] + points[2]) * .5f;
  const auto v_x = points[1] - points[0];
  const auto v_y = points[2] - points[1];
  x_axis = glm::normalize(v_x);
  y_axis = glm::normalize(v_y);

  width = glm::length(v_x);
  height = glm::length(v_y);
}

struct CpuDepthBuffer {
  std::vector<float> depth_buffer;
  std::vector<bool> modified;
  std::vector<std::mutex> pixel_locks;
  int width = 0;
  int height = 0;
  CpuDepthBuffer(const size_t width, const size_t height) {
    this->depth_buffer = std::vector<float>(width * height);
    this->modified = std::vector<bool>(width * height);
    Reset();
    this->pixel_locks = std::vector<std::mutex>(width * height);
    this->width = width;
    this->height = height;
  }

  void Reset() {
    std::fill(depth_buffer.begin(), depth_buffer.end(), -FLT_MAX);
    std::fill(modified.begin(), modified.end(), false);
  }

  [[nodiscard]] bool CompareZ(const int u, const int v, const float z) {
    if (u < 0 || v < 0 || u > width - 1 || v > height - 1)
      return false;
    if (const int uv = u + width * v; z >= depth_buffer[uv]) {
      std::lock_guard lock(pixel_locks[uv]);
      return true;
    }
    return false;
  }

  bool Update(const int u, const int v, const float z) {
    if (u < 0 || v < 0 || u > width - 1 || v > height - 1)
      return false;
    if (const int uv = u + width * v; z >= depth_buffer[uv]) {
      std::lock_guard lock(pixel_locks[uv]);
      depth_buffer[uv] = z;
      modified[uv] = true;
      return true;
    }
    return false;
  }
};

template <typename T>
struct CpuColorBuffer {
  std::vector<T> color_buffer;
  std::vector<std::mutex> pixel_locks;
  int width = 0;
  int height = 0;
  CpuColorBuffer(const size_t width, const size_t height) {
    this->color_buffer = std::vector<T>(width * height);
    std::fill(this->color_buffer.begin(), this->color_buffer.end(), T(0.f));
    this->pixel_locks = std::vector<std::mutex>(width * height);
    this->width = width;
    this->height = height;
  }

  void FillColor(const T& val) {
    std::fill(color_buffer.begin(), color_buffer.end(), val);
  }

  void SetPixel(const int u, const int v, const T& color) {
    if (u < 0 || v < 0 || u > width - 1 || v > height - 1)
      return;

    const int uv = u + width * v;
    std::lock_guard lock(pixel_locks[uv]);
    color_buffer[uv] = color;
  }
};

struct PointComparator {
  bool operator()(const glm::vec2& a, const glm::vec2& b) const {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  }
};

glm::vec2 BillboardCloud::Rectangle::Transform(const glm::vec2& target) const {
  glm::vec2 ret_val = target;
  // Recenter
  ret_val -= center;
  const float x = glm::dot(ret_val, x_axis);
  const float y = glm::dot(ret_val, y_axis);
  ret_val = glm::vec2(x, y) + glm::vec2(width, height) * .5f;
  return ret_val;
}

glm::vec3 BillboardCloud::Rectangle::Transform(const glm::vec3& target) const {
  glm::vec2 ret_val = target;
  // Recenter
  ret_val -= center;
  const float x = glm::dot(ret_val, x_axis);
  const float y = glm::dot(ret_val, y_axis);
  ret_val = glm::vec2(x, y) + glm::vec2(width, height) * .5f;
  return {ret_val, target.z};
}

float Cross(const glm::vec2& origin, const glm::vec2& a, const glm::vec2& b) {
  return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

std::vector<glm::vec2> BillboardCloud::RotatingCalipers::ConvexHull(std::vector<glm::vec2> points) {
  const size_t point_size = points.size();
  size_t k = 0;
  if (point_size <= 3)
    return points;

  std::vector<glm::vec2> ret_val(2 * point_size);
  std::sort(points.begin(), points.end(), PointComparator());

  for (size_t i = 0; i < point_size; ++i) {
    while (k >= 2 && Cross(ret_val[k - 2], ret_val[k - 1], points[i]) <= 0)
      k--;
    ret_val[k++] = points[i];
  }

  for (size_t i = point_size - 1, t = k + 1; i > 0; --i) {
    while (k >= t && Cross(ret_val[k - 2], ret_val[k - 1], points[i - 1]) <= 0)
      k--;
    ret_val[k++] = points[i - 1];
  }
  ret_val.resize(k - 1);
  return ret_val;
}

struct MinAreaState {
  size_t bottom;
  size_t left;
  float height;
  float width;
  float base_a;
  float base_b;
  float area;
};

BillboardCloud::Rectangle BillboardCloud::RotatingCalipers::GetMinAreaRectangle(std::vector<glm::vec2> points) {
  auto convex_hull = ConvexHull(std::move(points));
  float min_area = FLT_MAX;
  size_t left = 0, bottom = 0, right = 0, top = 0;

  /* rotating calipers sides will always have coordinates
   (a,b) (-b,a) (-a,-b) (b, -a)
   */
  /* this is a first base vector (a,b) initialized by (1,0) */

  glm::vec2 pt0 = convex_hull[0];
  float left_x = pt0.x;
  float right_x = pt0.x;
  float top_y = pt0.y;
  float bottom_y = pt0.y;

  size_t n = convex_hull.size();

  std::vector<glm::vec2> list(n);
  std::vector<float> lengths(n);

  for (size_t i = 0; i < n; i++) {
    if (pt0.x < left_x) {
      left_x = pt0.x;
      left = i;
    }
    if (pt0.x > right_x) {
      right_x = pt0.x;
      right = i;
    }
    if (pt0.y > top_y) {
      top_y = pt0.y;
      top = i;
    }
    if (pt0.y < bottom_y) {
      bottom_y = pt0.y;
      bottom = i;
    }

    glm::vec2 pt = convex_hull[(i + 1) & (i + 1 < n ? -1 : 0)];
    float dx = pt.x - pt0.x;
    float dy = pt.y - pt0.y;

    list[i].x = dx;
    list[i].y = dy;

    lengths[i] = 1.f / sqrt(dx * dx + dy * dy);
    pt0 = pt;
  }

  // find convex hull orientation
  float ax = list[n - 1].x;
  float ay = list[n - 1].y;
  float orientation = 0, base_a = 0, base_b = 0;

  for (size_t i = 0; i < n; i++) {
    float bx = list[i].x;
    float by = list[i].y;
    if (float convexity = ax * by - ay * bx; convexity != 0.f) {
      orientation = convexity > 0 ? 1.0 : -1.0;
      break;
    }
    ax = bx;
    ay = by;
  }

  base_a = orientation;

  /*****************************************************************************************/
  /*                         init calipers position                                        */
  size_t seq[4];
  seq[0] = bottom;
  seq[1] = right;
  seq[2] = top;
  seq[3] = left;

  /*****************************************************************************************/
  /*                         Main loop - evaluate angles and rotate calipers               */

  MinAreaState min_area_state;

  /* all the edges will be checked while rotating calipers by 90 degrees */
  for (size_t k = 0; k < n; k++) {
    /* sinus of minimal angle */
    /*float sinus;*/

    /* compute cosine of angle between calipers side and polygon edge */
    /* dp - dot product */
    float dp0 = base_a * list[seq[0]].x + base_b * list[seq[0]].y;
    float dp1 = -base_b * list[seq[1]].x + base_a * list[seq[1]].y;
    float dp2 = -base_a * list[seq[2]].x - base_b * list[seq[2]].y;
    float dp3 = base_b * list[seq[3]].x - base_a * list[seq[3]].y;

    float cos_alpha = dp0 * lengths[seq[0]];
    float max_cos = cos_alpha;
    /* number of calipers edges, that has minimal angle with edge */
    int main_element = 0;

    /* choose minimal angle */
    cos_alpha = dp1 * lengths[seq[1]];
    max_cos = cos_alpha > max_cos ? (main_element = 1, cos_alpha) : max_cos;
    cos_alpha = dp2 * lengths[seq[2]];
    max_cos = cos_alpha > max_cos ? (main_element = 2, cos_alpha) : max_cos;
    cos_alpha = dp3 * lengths[seq[3]];
    max_cos = cos_alpha > max_cos ? (main_element = 3, cos_alpha) : max_cos;

    /*rotate calipers*/
    // get next base
    size_t temp_point = seq[main_element];
    float lead_x = list[temp_point].x * lengths[temp_point];
    float lead_y = list[temp_point].y * lengths[temp_point];
    switch (main_element) {
      case 0:
        base_a = lead_x;
        base_b = lead_y;
        break;
      case 1:
        base_a = lead_y;
        base_b = -lead_x;
        break;
      case 2:
        base_a = -lead_x;
        base_b = -lead_y;
        break;
      case 3:
        base_a = -lead_y;
        base_b = lead_x;
        break;
    }

    /* change base point of main edge */
    seq[main_element] += 1;
    seq[main_element] = seq[main_element] == n ? 0 : seq[main_element];

    float dx = convex_hull[seq[1]].x - convex_hull[seq[3]].x;
    float dy = convex_hull[seq[1]].y - convex_hull[seq[3]].y;
    float width = dx * base_a + dy * base_b;
    dx = convex_hull[seq[2]].x - convex_hull[seq[0]].x;
    dy = convex_hull[seq[2]].y - convex_hull[seq[0]].y;
    float height = -dx * base_b + dy * base_a;
    if (float area = width * height; area <= min_area) {
      min_area = area;
      min_area_state.base_a = base_a;
      min_area_state.base_b = base_b;
      min_area_state.width = width;
      min_area_state.height = height;
      min_area_state.left = seq[3];
      min_area_state.bottom = seq[0];
      min_area_state.area = area;
    }
  }

  float a1 = min_area_state.base_a;
  float b1 = min_area_state.base_b;

  float a2 = -min_area_state.base_b;
  float b2 = min_area_state.base_a;

  float c1 = a1 * convex_hull[min_area_state.left].x + convex_hull[min_area_state.left].y * b1;
  float c2 = a2 * convex_hull[min_area_state.bottom].x + convex_hull[min_area_state.bottom].y * b2;

  float id = 1.f / (a1 * b2 - a2 * b1);

  float px = (c1 * b2 - c2 * b1) * id;
  float py = (a1 * c2 - a2 * c1) * id;

  glm::vec2 out0(px, py);
  glm::vec2 out1(a1 * min_area_state.width, b1 * min_area_state.width);
  glm::vec2 out2(a2 * min_area_state.height, b2 * min_area_state.height);

  Rectangle ret_val;

  ret_val.points[0] = out0;
  ret_val.points[1] = out0 + out1;
  ret_val.points[2] = out0 + out1 + out2;
  ret_val.points[3] = out0 + out2;

  return ret_val;
}

inline float EdgeFunction(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
  return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]);
}

void Barycentric3D(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, float& c0, float& c1,
                   float& c2) {
  const auto &v0 = b - a, v1 = c - a, v2 = p - a;
  const float d00 = glm::dot(v0, v0);
  const float d01 = glm::dot(v0, v1);
  const float d11 = glm::dot(v1, v1);
  const float d20 = glm::dot(v2, v0);
  const float d21 = glm::dot(v2, v1);
  const float den = d00 * d11 - d01 * d01;
  c1 = (d11 * d20 - d01 * d21) / den;
  c2 = (d00 * d21 - d01 * d20) / den;
  c0 = 1.0f - c1 - c2;
}

inline void Barycentric2D(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b, const glm::vec2& c, float& c0,
                          float& c1, float& c2) {
  const auto v0 = b - a, v1 = c - a, v2 = p - a;
  if (const float den = v0.x * v1.y - v1.x * v0.y; den == 0.f) {
    c1 = c2 = 0.f;
    c0 = 1.f;
  } else {
    c1 = (v2.x * v1.y - v1.x * v2.y) / den;
    c2 = (v0.x * v2.y - v2.x * v0.y) / den;
    c0 = 1.0f - c1 - c2;
  }
}
inline float Area(const glm::vec2& a, const glm::vec2& b) {
  return a.x * b.y - a.y * b.x;
}

inline void Barycentric2D(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b, const glm::vec2& c,
                          const glm::vec2& d, float& c0, float& c1, float& c2, float& c3) {
  float r[4], t[4], u[4];
  const glm::vec2 v[4] = {a, b, c, d};
  glm::vec2 s[4];
  for (int i = 0; i < 4; i++) {
    s[i] = v[i] - p;
    r[i] = length(s[i]);
  }
  for (int i = 0; i < 4; i++) {
    const float area = Area(s[i], s[(i + 1) % 4]);
    const float dot_result = glm::dot(s[i], s[(i + 1) % 4]);
    if (area == 0.f)
      t[i] = 0.f;
    else {
      t[i] = (r[i] * r[(i + 1) % 4] - dot_result) / area;
    }
  }
  for (int i = 0; i < 4; i++) {
    if (r[i] == 0.f)
      u[i] = 0.f;
    else
      u[i] = (t[(i + 3) % 4] + t[i]) / r[i];
  }
  const auto sum = u[0] + u[1] + u[2] + u[3];
  assert(sum != 0.f);
  c0 = u[0] / sum;
  c1 = u[1] / sum;
  c2 = u[2] / sum;
  c3 = u[3] / sum;
}

void BillboardCloud::Project(const ProjectSettings& project_settings) {
  for (auto& cluster : clusters)
    Project(cluster, project_settings);
}

void BillboardCloud::Join(const JoinSettings& join_settings) {
  xatlas::Atlas* atlas = xatlas::Create();

  for (const auto& cluster : clusters) {
    xatlas::MeshDecl mesh_decl;
    mesh_decl.vertexCount = cluster.billboard_vertices.size();
    mesh_decl.vertexPositionData = cluster.billboard_vertices.data();
    mesh_decl.vertexPositionStride = sizeof(Vertex);
    mesh_decl.indexCount = cluster.billboard_triangles.size() * 3;
    mesh_decl.indexData = cluster.billboard_triangles.data();
    mesh_decl.indexFormat = xatlas::IndexFormat::UInt32;
    xatlas::AddMeshError error = xatlas::AddMesh(atlas, mesh_decl, 1);
    if (error != xatlas::AddMeshError::Success) {
      EVOENGINE_ERROR("Error adding mesh!");
      break;
    }
  }
  xatlas::AddMeshJoin(atlas);
  xatlas::Generate(atlas);
  std::vector<Vertex> billboard_cloud_vertices;
  billboard_cloud_vertices.resize(clusters.size() * 4);
  std::vector<glm::uvec3> billboard_cloud_triangles;
  billboard_cloud_triangles.resize(clusters.size() * 2);
  Jobs::RunParallelFor(clusters.size(), [&](const unsigned cluster_index) {
    const xatlas::Mesh& mesh = atlas->meshes[cluster_index];
    auto& cluster = clusters[cluster_index];
    cluster.rectangle.tex_coords[0].x = mesh.vertexArray[0].uv[0] / static_cast<float>(atlas->width);
    cluster.rectangle.tex_coords[0].y = mesh.vertexArray[0].uv[1] / static_cast<float>(atlas->height);
    cluster.rectangle.tex_coords[1].x = mesh.vertexArray[1].uv[0] / static_cast<float>(atlas->width);
    cluster.rectangle.tex_coords[1].y = mesh.vertexArray[1].uv[1] / static_cast<float>(atlas->height);
    cluster.rectangle.tex_coords[2].x = mesh.vertexArray[2].uv[0] / static_cast<float>(atlas->width);
    cluster.rectangle.tex_coords[2].y = mesh.vertexArray[2].uv[1] / static_cast<float>(atlas->height);
    cluster.rectangle.tex_coords[3].x = mesh.vertexArray[3].uv[0] / static_cast<float>(atlas->width);
    cluster.rectangle.tex_coords[3].y = mesh.vertexArray[3].uv[1] / static_cast<float>(atlas->height);

    billboard_cloud_vertices[4 * cluster_index] = cluster.billboard_vertices[0];
    billboard_cloud_vertices[4 * cluster_index + 1] = cluster.billboard_vertices[1];
    billboard_cloud_vertices[4 * cluster_index + 2] = cluster.billboard_vertices[2];
    billboard_cloud_vertices[4 * cluster_index + 3] = cluster.billboard_vertices[3];

    billboard_cloud_vertices[4 * cluster_index].tex_coord = cluster.rectangle.tex_coords[0];
    billboard_cloud_vertices[4 * cluster_index + 1].tex_coord = cluster.rectangle.tex_coords[1];
    billboard_cloud_vertices[4 * cluster_index + 2].tex_coord = cluster.rectangle.tex_coords[2];
    billboard_cloud_vertices[4 * cluster_index + 3].tex_coord = cluster.rectangle.tex_coords[3];

    billboard_cloud_triangles[2 * cluster_index] = cluster.billboard_triangles[0] + glm::uvec3(cluster_index * 4);
    billboard_cloud_triangles[2 * cluster_index + 1] = cluster.billboard_triangles[1] + glm::uvec3(cluster_index * 4);
  });

  billboard_cloud_mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
  billboard_cloud_mesh->SetVertices({false, false, true, false}, billboard_cloud_vertices, billboard_cloud_triangles);
  xatlas::Destroy(atlas);
}
struct PbrMaterial {
  glm::vec4 base_albedo = glm::vec4(1.f);
  float base_roughness = 0.3f;
  float base_metallic = 0.3f;
  float base_ao = 1.f;
  glm::ivec2 albedo_texture_resolution = glm::ivec2(-1);
  std::vector<glm::vec4> albedo_texture_data;

  glm::ivec2 normal_texture_resolution = glm::ivec2(-1);
  std::vector<glm::vec3> normal_texture_data;

  glm::ivec2 roughness_texture_resolution = glm::ivec2(-1);
  std::vector<float> roughness_texture_data;

  glm::ivec2 metallic_texture_resolution = glm::ivec2(-1);
  std::vector<float> metallic_texture_data;

  glm::ivec2 ao_texture_resolution = glm::ivec2(-1);
  std::vector<float> ao_texture_data;
  void Clear();
  void ApplyMaterial(const std::shared_ptr<Material>& material,
                     const BillboardCloud::RasterizeSettings& rasterize_settings);
};
void PbrMaterial::Clear() {
  base_albedo = glm::vec4(1.f);
  base_roughness = 0.3f;
  base_metallic = 0.3f;
  base_ao = 1.f;
  albedo_texture_resolution = glm::ivec2(-1);
  normal_texture_resolution = glm::ivec2(-1);
  roughness_texture_resolution = glm::ivec2(-1);
  metallic_texture_resolution = glm::ivec2(-1);
  ao_texture_resolution = glm::ivec2(-1);

  albedo_texture_data.clear();
  normal_texture_data.clear();
  roughness_texture_data.clear();
  metallic_texture_data.clear();
  ao_texture_data.clear();
}

void PbrMaterial::ApplyMaterial(const std::shared_ptr<Material>& material,
                                const BillboardCloud::RasterizeSettings& rasterize_settings) {
  base_albedo = glm::vec4(material->material_properties.albedo_color, 1.f - material->material_properties.transmission);
  base_roughness = material->material_properties.roughness;
  base_metallic = material->material_properties.metallic;
  base_ao = 1.f;

  if (const auto albedo_texture = material->GetAlbedoTexture();
      rasterize_settings.transfer_albedo_map && albedo_texture) {
    albedo_texture->GetRgbaChannelData(albedo_texture_data);
    albedo_texture_resolution = albedo_texture->GetResolution();
  }
  if (const auto normal_texture = material->GetNormalTexture();
      rasterize_settings.transfer_normal_map && normal_texture) {
    normal_texture->GetRgbChannelData(normal_texture_data);
    normal_texture_resolution = normal_texture->GetResolution();
  }
  if (const auto roughness_texture = material->GetRoughnessTexture();
      rasterize_settings.transfer_roughness_map && roughness_texture) {
    roughness_texture->GetRedChannelData(roughness_texture_data);
    roughness_texture_resolution = roughness_texture->GetResolution();
  }
  if (const auto metallic_texture = material->GetMetallicTexture();
      rasterize_settings.transfer_metallic_map && metallic_texture) {
    metallic_texture->GetRedChannelData(metallic_texture_data);
    metallic_texture_resolution = metallic_texture->GetResolution();
  }
  if (const auto ao_texture = material->GetAoTexture(); rasterize_settings.transfer_ao_map && ao_texture) {
    ao_texture->GetRedChannelData(ao_texture_data);
    ao_texture_resolution = ao_texture->GetResolution();
  }
}

void BillboardCloud::Rasterize(const RasterizeSettings& rasterize_settings) {
  if (rasterize_settings.base_resolution.x < 1 || rasterize_settings.base_resolution.y < 1)
    return;
  std::unordered_map<Handle, PbrMaterial> pbr_materials;
  float average_roughness = 0.f;
  float average_metallic = 0.0f;
  float average_ao = 0.f;
  for (auto& element : elements) {
    const auto& material = element.material;
    auto material_handle = material->GetHandle();
    pbr_materials[material_handle].ApplyMaterial(material, rasterize_settings);

    average_roughness += material->material_properties.roughness;
    average_metallic += material->material_properties.metallic;
    average_ao += 1.f;
  }
  average_roughness /= elements.size();
  average_metallic /= elements.size();
  average_ao /= elements.size();

  CpuDepthBuffer depth_buffer(rasterize_settings.base_resolution.x, rasterize_settings.base_resolution.y);

  CpuColorBuffer<glm::vec4> albedo_frame_buffer(rasterize_settings.base_resolution.x,
                                                rasterize_settings.base_resolution.y);
  CpuColorBuffer<glm::vec3> normal_frame_buffer(rasterize_settings.base_resolution.x,
                                                rasterize_settings.base_resolution.y);
  CpuColorBuffer<float> roughness_frame_buffer(rasterize_settings.base_resolution.x,
                                               rasterize_settings.base_resolution.y);
  CpuColorBuffer<float> metallic_frame_buffer(rasterize_settings.base_resolution.x,
                                              rasterize_settings.base_resolution.y);
  CpuColorBuffer<float> ao_frame_buffer(rasterize_settings.base_resolution.x, rasterize_settings.base_resolution.y);

  if (rasterize_settings.debug_opaque) {
    average_roughness = 1.f;
    average_metallic = 0.f;
    average_ao = 1.f;
    Jobs::RunParallelFor(clusters.size(), [&](const unsigned cluster_index) {
      const auto& cluster = clusters[cluster_index];
      const glm::vec3 color = glm::ballRand(1.f);
      const auto& bounding_rectangle = cluster.rectangle;

      for (int triangle_index = 0; triangle_index < 2; triangle_index++) {
        glm::vec3 texture_space_vertices[3];
        if (triangle_index == 0) {
          texture_space_vertices[0] =
              glm::vec3(bounding_rectangle.tex_coords[0] * glm::vec2(rasterize_settings.base_resolution), 0.f);
          texture_space_vertices[1] =
              glm::vec3(bounding_rectangle.tex_coords[1] * glm::vec2(rasterize_settings.base_resolution), 0.f);
          texture_space_vertices[2] =
              glm::vec3(bounding_rectangle.tex_coords[2] * glm::vec2(rasterize_settings.base_resolution), 0.f);
        } else {
          texture_space_vertices[0] =
              glm::vec3(bounding_rectangle.tex_coords[2] * glm::vec2(rasterize_settings.base_resolution), 0.f);
          texture_space_vertices[1] =
              glm::vec3(bounding_rectangle.tex_coords[3] * glm::vec2(rasterize_settings.base_resolution), 0.f);
          texture_space_vertices[2] =
              glm::vec3(bounding_rectangle.tex_coords[0] * glm::vec2(rasterize_settings.base_resolution), 0.f);
        }
        // Bound check;
        auto min_bound = glm::vec2(FLT_MAX, FLT_MAX);
        auto max_bound = glm::vec2(-FLT_MAX, -FLT_MAX);
        for (const auto& texture_space_vertex : texture_space_vertices) {
          min_bound = glm::min(glm::vec2(texture_space_vertex), min_bound);
          max_bound = glm::max(glm::vec2(texture_space_vertex), max_bound);
        }

        const auto left = static_cast<int>(min_bound.x - 0.5f);
        const auto right = static_cast<int>(max_bound.x + 0.5f);
        const auto top = static_cast<int>(min_bound.y - 0.5f);
        const auto bottom = static_cast<int>(max_bound.y + 0.5f);
        for (auto u = left; u <= right; u++) {
          for (auto v = top; v <= bottom; v++) {
            const auto p = glm::vec3(u + .5f, v + .5f, 0.f);
            float bc0, bc1, bc2;
            Barycentric2D(p, texture_space_vertices[0], texture_space_vertices[1], texture_space_vertices[2], bc0, bc1,
                          bc2);
            if (bc0 < 0.f || bc1 < 0.f || bc2 < 0.f)
              continue;
            auto albedo = glm::vec4(color, 1.f);
            albedo_frame_buffer.SetPixel(u, v, albedo);
          }
        }
      }
    });
  }

  for (auto& cluster : clusters) {
    // Calculate texture size on atlas
    const auto& bounding_rectangle = cluster.rectangle;

    // Rasterization
    Jobs::RunParallelFor(cluster.projected_triangles.size(), [&](const unsigned triangle_index) {
      const auto& triangle = cluster.projected_triangles[triangle_index];
      const auto& v0 = triangle.projected_vertices[0];
      const auto& v1 = triangle.projected_vertices[1];
      const auto& v2 = triangle.projected_vertices[2];
      const auto& material = pbr_materials.at(triangle.material_handle);

      glm::vec3 texture_space_vertices[3];
      for (int i = 0; i < 3; i++) {
        const auto p = glm::vec2(triangle.projected_vertices[i].position.x, triangle.projected_vertices[i].position.y);
        const auto r0 = bounding_rectangle.points[0];
        const auto r1 = bounding_rectangle.points[1];
        const auto r2 = bounding_rectangle.points[2];
        const auto r3 = bounding_rectangle.points[3];

        float bc0, bc1, bc2, bc3;
        Barycentric2D(p, r0, r1, r2, r3, bc0, bc1, bc2, bc3);
        const auto texture_space_position =
            (bounding_rectangle.tex_coords[0] * bc0 + bounding_rectangle.tex_coords[1] * bc1 +
             bounding_rectangle.tex_coords[2] * bc2 + bounding_rectangle.tex_coords[3] * bc3) *
            glm::vec2(rasterize_settings.base_resolution);
        texture_space_vertices[i].x = texture_space_position.x;
        texture_space_vertices[i].y = texture_space_position.y;
        texture_space_vertices[i].z = triangle.projected_vertices[i].position.z;
      }

      // Bound check;
      auto min_bound = glm::vec2(FLT_MAX, FLT_MAX);
      auto max_bound = glm::vec2(-FLT_MAX, -FLT_MAX);
      for (const auto& texture_space_vertex : texture_space_vertices) {
        min_bound = glm::min(glm::vec2(texture_space_vertex), min_bound);
        max_bound = glm::max(glm::vec2(texture_space_vertex), max_bound);
      }

      const auto left = static_cast<int>(min_bound.x - 0.5f);
      const auto right = static_cast<int>(max_bound.x + 0.5f);
      const auto top = static_cast<int>(min_bound.y - 0.5f);
      const auto bottom = static_cast<int>(max_bound.y + 0.5f);
      for (auto u = left; u <= right; u++) {
        for (auto v = top; v <= bottom; v++) {
          const auto p = glm::vec3(u + .5f, v + .5f, 0.f);
          float bc0, bc1, bc2;
          Barycentric2D(p, texture_space_vertices[0], texture_space_vertices[1], texture_space_vertices[2], bc0, bc1,
                        bc2);
          if (bc0 < 0.f || bc1 < 0.f || bc2 < 0.f)
            continue;
          float z = bc0 * v0.position.z + bc1 * v1.position.z + bc2 * v2.position.z;
          // Early depth check.
          if (!depth_buffer.CompareZ(u, v, z))
            continue;
          const auto tex_coords = bc0 * v0.tex_coord + bc1 * v1.tex_coord + bc2 * v2.tex_coord;
          auto albedo = material.base_albedo;
          float roughness = material.base_roughness;
          float metallic = material.base_metallic;
          float ao = material.base_ao;
          if (!material.albedo_texture_data.empty()) {
            int texture_x = static_cast<int>(material.albedo_texture_resolution.x * tex_coords.x) %
                            material.albedo_texture_resolution.x;
            int texture_y = static_cast<int>(material.albedo_texture_resolution.y * tex_coords.y) %
                            material.albedo_texture_resolution.y;
            if (texture_x < 0)
              texture_x += material.albedo_texture_resolution.x;
            if (texture_y < 0)
              texture_y += material.albedo_texture_resolution.y;

            const auto index = texture_y * material.albedo_texture_resolution.x + texture_x;
            albedo = material.albedo_texture_data[index];
          }
          // Alpha discard
          if (albedo.a < 0.1f)
            continue;
          if (!depth_buffer.Update(u, v, z))
            continue;
          auto normal = glm::normalize(bc0 * v0.normal + bc1 * v1.normal + bc2 * v2.normal);
          if (!material.normal_texture_data.empty()) {
            auto tangent = glm::normalize(bc0 * v0.tangent + bc1 * v1.tangent + bc2 * v2.tangent);
            const auto bi_tangent = glm::cross(normal, tangent);
            const auto tbn = glm::mat3(tangent, bi_tangent, normal);

            int texture_x = static_cast<int>(material.normal_texture_resolution.x * tex_coords.x) %
                            material.normal_texture_resolution.x;
            int texture_y = static_cast<int>(material.normal_texture_resolution.y * tex_coords.y) %
                            material.normal_texture_resolution.y;
            if (texture_x < 0)
              texture_x += material.normal_texture_resolution.x;
            if (texture_y < 0)
              texture_y += material.normal_texture_resolution.y;

            const auto index = texture_y * material.normal_texture_resolution.x + texture_x;
            const auto sampled = glm::normalize(material.normal_texture_data[index]) * 2.0f - glm::vec3(1.0f);
            normal = glm::normalize(tbn * sampled);
          }
          if (glm::dot(normal, glm::vec3(0, 0, 1)) < 0)
            normal = -normal;

          if (!material.roughness_texture_data.empty()) {
            int texture_x = static_cast<int>(material.roughness_texture_resolution.x * tex_coords.x) %
                            material.roughness_texture_resolution.x;
            int texture_y = static_cast<int>(material.roughness_texture_resolution.y * tex_coords.y) %
                            material.roughness_texture_resolution.y;
            if (texture_x < 0)
              texture_x += material.roughness_texture_resolution.x;
            if (texture_y < 0)
              texture_y += material.roughness_texture_resolution.y;

            const auto index = texture_y * material.roughness_texture_resolution.x + texture_x;
            roughness = material.roughness_texture_data[index];
          }
          if (!material.metallic_texture_data.empty()) {
            int texture_x = static_cast<int>(material.metallic_texture_resolution.x * tex_coords.x) %
                            material.metallic_texture_resolution.x;
            int texture_y = static_cast<int>(material.metallic_texture_resolution.y * tex_coords.y) %
                            material.metallic_texture_resolution.y;
            if (texture_x < 0)
              texture_x += material.metallic_texture_resolution.x;
            if (texture_y < 0)
              texture_y += material.metallic_texture_resolution.y;

            const auto index = texture_y * material.metallic_texture_resolution.x + texture_x;
            metallic = material.metallic_texture_data[index];
          }
          if (!material.ao_texture_data.empty()) {
            int texture_x =
                static_cast<int>(material.ao_texture_resolution.x * tex_coords.x) % material.ao_texture_resolution.x;
            int texture_y =
                static_cast<int>(material.ao_texture_resolution.y * tex_coords.y) % material.ao_texture_resolution.y;
            if (texture_x < 0)
              texture_x += material.ao_texture_resolution.x;
            if (texture_y < 0)
              texture_y += material.ao_texture_resolution.y;

            const auto index = texture_y * material.ao_texture_resolution.x + texture_x;
            ao = material.ao_texture_data[index];
          }

          depth_buffer.Update(u, v, z);
          albedo_frame_buffer.SetPixel(u, v, albedo);
          normal = normal * 0.5f + glm::vec3(0.5f);
          normal_frame_buffer.SetPixel(u, v, normal);
          roughness_frame_buffer.SetPixel(u, v, roughness);
          metallic_frame_buffer.SetPixel(u, v, metallic);
          ao_frame_buffer.SetPixel(u, v, ao);
        }
      }
    });
  }
  if (rasterize_settings.dilate != -1) {
    auto valid_pixels = depth_buffer.modified;
    DilateChannels({0, 1, 2}, albedo_frame_buffer.color_buffer, valid_pixels, rasterize_settings.dilate,
           glm::uvec2(albedo_frame_buffer.width, albedo_frame_buffer.height), false);
    DilateChannels({0, 1, 2}, albedo_frame_buffer.color_buffer, valid_pixels, rasterize_settings.dilate,
           glm::uvec2(albedo_frame_buffer.width, albedo_frame_buffer.height), true);

    valid_pixels = depth_buffer.modified;
    Dilate(normal_frame_buffer.color_buffer, valid_pixels, rasterize_settings.dilate,
           glm::uvec2(normal_frame_buffer.width, normal_frame_buffer.height), false);
    Dilate(normal_frame_buffer.color_buffer, valid_pixels, rasterize_settings.dilate,
           glm::uvec2(normal_frame_buffer.width, normal_frame_buffer.height), true);

    valid_pixels = depth_buffer.modified;
    Dilate(roughness_frame_buffer.color_buffer, valid_pixels, rasterize_settings.dilate,
           glm::uvec2(roughness_frame_buffer.width, roughness_frame_buffer.height), false);
    Dilate(roughness_frame_buffer.color_buffer, valid_pixels, rasterize_settings.dilate,
           glm::uvec2(roughness_frame_buffer.width, roughness_frame_buffer.height), true);

    valid_pixels = depth_buffer.modified;
    Dilate(metallic_frame_buffer.color_buffer, valid_pixels, rasterize_settings.dilate,
           glm::uvec2(metallic_frame_buffer.width, metallic_frame_buffer.height), false);
    Dilate(metallic_frame_buffer.color_buffer, valid_pixels, rasterize_settings.dilate,
           glm::uvec2(metallic_frame_buffer.width, metallic_frame_buffer.height), true);

    valid_pixels = depth_buffer.modified;
    Dilate(ao_frame_buffer.color_buffer, valid_pixels, rasterize_settings.dilate,
           glm::uvec2(ao_frame_buffer.width, ao_frame_buffer.height), false);
    Dilate(ao_frame_buffer.color_buffer, valid_pixels, rasterize_settings.dilate,
           glm::uvec2(ao_frame_buffer.width, ao_frame_buffer.height), true);
  }

  billboard_cloud_material = ProjectManager::CreateTemporaryAsset<Material>();

  std::shared_ptr<Texture2D> albedo_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
  if (rasterize_settings.base_resolution == rasterize_settings.output_albedo_resolution) {
    albedo_texture->SetRgbaChannelData(albedo_frame_buffer.color_buffer,
                                       glm::uvec2(albedo_frame_buffer.width, albedo_frame_buffer.height));
  } else {
    std::vector<glm::vec4> res;
    Texture2D::Resize(albedo_frame_buffer.color_buffer, glm::uvec2(rasterize_settings.base_resolution), res, rasterize_settings.output_albedo_resolution);
    albedo_texture->SetRgbaChannelData(
        res, glm::uvec2(rasterize_settings.output_albedo_resolution.x, rasterize_settings.output_albedo_resolution.y));
  }
  albedo_texture->UnsafeUploadDataImmediately();
  billboard_cloud_material->SetAlbedoTexture(albedo_texture);

  std::shared_ptr<Texture2D> normal_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
  if (rasterize_settings.base_resolution == rasterize_settings.output_material_props_resolution) {
    normal_texture->SetRgbChannelData(normal_frame_buffer.color_buffer,
                                      glm::uvec2(normal_frame_buffer.width, normal_frame_buffer.height));
  } else {
    std::vector<glm::vec3> res;
    Texture2D::Resize(normal_frame_buffer.color_buffer, glm::uvec2(rasterize_settings.base_resolution), res, rasterize_settings.output_material_props_resolution);
    normal_texture->SetRgbChannelData(res, glm::uvec2(rasterize_settings.output_material_props_resolution.x,
                                                      rasterize_settings.output_material_props_resolution.y));
  }
  normal_texture->UnsafeUploadDataImmediately();
  billboard_cloud_material->SetNormalTexture(normal_texture);

  std::shared_ptr<Texture2D> roughness_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
  if (rasterize_settings.base_resolution == rasterize_settings.output_material_props_resolution) {
    roughness_texture->SetRedChannelData(roughness_frame_buffer.color_buffer,
                                      glm::uvec2(roughness_frame_buffer.width, roughness_frame_buffer.height));
  } else {
    std::vector<float> res;
    Texture2D::Resize(roughness_frame_buffer.color_buffer, glm::uvec2(rasterize_settings.base_resolution), res, rasterize_settings.output_material_props_resolution);
    roughness_texture->SetRedChannelData(res, glm::uvec2(rasterize_settings.output_material_props_resolution.x,
                                                      rasterize_settings.output_material_props_resolution.y));
  }
  roughness_texture->UnsafeUploadDataImmediately();
  billboard_cloud_material->SetRoughnessTexture(roughness_texture);

  std::shared_ptr<Texture2D> metallic_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
  if (rasterize_settings.base_resolution == rasterize_settings.output_material_props_resolution) {
    metallic_texture->SetRedChannelData(metallic_frame_buffer.color_buffer,
                                      glm::uvec2(metallic_frame_buffer.width, metallic_frame_buffer.height));
  } else {
    std::vector<float> res;
    Texture2D::Resize(metallic_frame_buffer.color_buffer, glm::uvec2(rasterize_settings.base_resolution), res, rasterize_settings.output_material_props_resolution);
    metallic_texture->SetRedChannelData(res, glm::uvec2(rasterize_settings.output_material_props_resolution.x,
                                                      rasterize_settings.output_material_props_resolution.y));
  }
  metallic_texture->UnsafeUploadDataImmediately();
  billboard_cloud_material->SetMetallicTexture(metallic_texture);

  std::shared_ptr<Texture2D> ao_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
  if (rasterize_settings.base_resolution == rasterize_settings.output_material_props_resolution) {
    ao_texture->SetRedChannelData(ao_frame_buffer.color_buffer,
                                      glm::uvec2(ao_frame_buffer.width, ao_frame_buffer.height));
  } else {
    std::vector<float> res;
    Texture2D::Resize(ao_frame_buffer.color_buffer, glm::uvec2(rasterize_settings.base_resolution), res, rasterize_settings.output_material_props_resolution);
    ao_texture->SetRedChannelData(res, glm::uvec2(rasterize_settings.output_material_props_resolution.x,
                                                      rasterize_settings.output_material_props_resolution.y));
  }
  ao_texture->UnsafeUploadDataImmediately();
  billboard_cloud_material->SetAoTexture(ao_texture);
}

void BillboardCloud::Generate(const GenerateSettings& generate_settings) {
  Clusterize(generate_settings.clusterization_settings);
  Project(generate_settings.project_settings);
  Join(generate_settings.join_settings);
  Rasterize(generate_settings.rasterize_settings);
}

void BillboardCloud::Project(Cluster& cluster, const ProjectSettings& project_settings) const {
  const auto billboard_front_axis = cluster.cluster_plane.GetNormal();
  auto billboard_up_axis =
      glm::vec3(billboard_front_axis.y, billboard_front_axis.z, billboard_front_axis.x);  // cluster.m_planeYAxis;
  const auto billboard_left_axis = glm::normalize(glm::cross(billboard_front_axis, billboard_up_axis));
  billboard_up_axis = glm::normalize(glm::cross(billboard_left_axis, billboard_front_axis));
  glm::mat4 rotate_matrix =
      glm::transpose(glm::mat4(glm::vec4(billboard_left_axis, 0.0f), glm::vec4(billboard_up_axis, 0.0f),
                               glm::vec4(billboard_front_axis, 0.0f), glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)));
  cluster.projected_triangles.resize(cluster.triangles.size());
  Jobs::RunParallelFor(cluster.triangles.size(), [&](const unsigned triangle_index) {
    const auto& cluster_triangle = cluster.triangles.at(triangle_index);
    auto& projected_triangle = cluster.projected_triangles[triangle_index];
    const auto& element = elements.at(cluster_triangle.element_index);
    const auto& vertices = element.vertices;
    const auto& triangle = element.triangles.at(cluster_triangle.triangle_index);
    auto& v0 = vertices.at(triangle.x);
    auto& v1 = vertices.at(triangle.y);
    auto& v2 = vertices.at(triangle.z);

    auto& p_v0 = projected_triangle.projected_vertices[0];
    auto& p_v1 = projected_triangle.projected_vertices[1];
    auto& p_v2 = projected_triangle.projected_vertices[2];

    TransformVertex(v0, p_v0, rotate_matrix);
    TransformVertex(v1, p_v1, rotate_matrix);
    TransformVertex(v2, p_v2, rotate_matrix);

    projected_triangle.material_handle = element.material->GetHandle();
  });

  std::vector<glm::vec2> points;
  points.resize(cluster.projected_triangles.size() * 3);
  Jobs::RunParallelFor(cluster.projected_triangles.size(), [&](const unsigned triangle_index) {
    const auto& projected_triangle = cluster.projected_triangles.at(triangle_index);
    points.at(triangle_index * 3) = glm::vec2(projected_triangle.projected_vertices[0].position.x,
                                              projected_triangle.projected_vertices[0].position.y);
    points.at(triangle_index * 3 + 1) = glm::vec2(projected_triangle.projected_vertices[1].position.x,
                                                  projected_triangle.projected_vertices[1].position.y);
    points.at(triangle_index * 3 + 2) = glm::vec2(projected_triangle.projected_vertices[2].position.x,
                                                  projected_triangle.projected_vertices[2].position.y);
  });

  // Calculate bounding triangle.
  assert(points.size() > 2);
  if (points.size() == 3) {
    const auto& p0 = points[0];
    const auto& p1 = points[1];
    const auto& p2 = points[2];

    const auto e0 = glm::distance(p0, p1);
    const auto e1 = glm::distance(p1, p2);
    const auto e2 = glm::distance(p2, p0);
    glm::vec2 longest_edge_start, longest_edge_end, other_point;
    if (e0 >= e1 && e0 >= e2) {
      longest_edge_start = p0;
      longest_edge_end = p1;
      other_point = p2;

    } else if (e1 >= e0 && e1 >= e2) {
      longest_edge_start = p1;
      longest_edge_end = p2;
      other_point = p0;
    } else {
      longest_edge_start = p2;
      longest_edge_end = p0;
      other_point = p1;
    }
    float length = glm::length(longest_edge_end - longest_edge_start);
    glm::vec2 length_vector = glm::normalize(longest_edge_end - longest_edge_start);
    float projected_distance = glm::dot(other_point - longest_edge_start, length_vector);
    glm::vec2 projected_point = longest_edge_start + projected_distance * length_vector;
    float width = glm::distance(other_point, projected_point);
    glm::vec2 width_vector = glm::normalize(other_point - projected_point);
    cluster.rectangle.points[0] = longest_edge_start;
    cluster.rectangle.points[3] = longest_edge_start + length * length_vector;
    cluster.rectangle.points[1] = cluster.rectangle.points[0] + width * width_vector;
    cluster.rectangle.points[2] = cluster.rectangle.points[3] + width * width_vector;
  } else {
    cluster.rectangle = RotatingCalipers::GetMinAreaRectangle(std::move(points));
  }
  cluster.rectangle.Update();
  // Generate billboard mesh
  cluster.billboard_vertices.resize(4);
  const auto inverse_rotate_matrix = glm::inverse(rotate_matrix);
  cluster.billboard_vertices[0].position =
      inverse_rotate_matrix * glm::vec4(cluster.rectangle.points[0].x, cluster.rectangle.points[0].y, 0.f, 0.f);
  cluster.rectangle.tex_coords[0] = cluster.billboard_vertices[0].tex_coord = glm::vec2(0, 0);
  cluster.billboard_vertices[1].position =
      inverse_rotate_matrix * glm::vec4(cluster.rectangle.points[1].x, cluster.rectangle.points[1].y, 0.f, 0.f);
  cluster.rectangle.tex_coords[1] = cluster.billboard_vertices[1].tex_coord = glm::vec2(1, 0);
  cluster.billboard_vertices[2].position =
      inverse_rotate_matrix * glm::vec4(cluster.rectangle.points[2].x, cluster.rectangle.points[2].y, 0.f, 0.f);
  cluster.rectangle.tex_coords[2] = cluster.billboard_vertices[2].tex_coord = glm::vec2(1, 1);
  cluster.billboard_vertices[3].position =
      inverse_rotate_matrix * glm::vec4(cluster.rectangle.points[3].x, cluster.rectangle.points[3].y, 0.f, 0.f);
  cluster.rectangle.tex_coords[3] = cluster.billboard_vertices[3].tex_coord = glm::vec2(0, 1);

  cluster.billboard_vertices[0].position -= cluster.cluster_plane.d * billboard_front_axis;
  cluster.billboard_vertices[1].position -= cluster.cluster_plane.d * billboard_front_axis;
  cluster.billboard_vertices[2].position -= cluster.cluster_plane.d * billboard_front_axis;
  cluster.billboard_vertices[3].position -= cluster.cluster_plane.d * billboard_front_axis;

  cluster.billboard_triangles.resize(2);
  cluster.billboard_triangles[0] = {0, 1, 2};
  cluster.billboard_triangles[1] = {2, 3, 0};
}

#pragma endregion
#pragma region IO
Entity BillboardCloud::BuildEntity(const std::shared_ptr<Scene>& scene) const {
  if (!billboard_cloud_mesh || !billboard_cloud_material)
    return {};
  const auto owner = scene->CreateEntity("Billboard Cloud");

  const auto projected_element_entity = scene->CreateEntity("Billboard Cloud");
  const auto element_mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(projected_element_entity).lock();
  element_mesh_renderer->mesh = billboard_cloud_mesh;
  element_mesh_renderer->material = billboard_cloud_material;
  scene->SetParent(projected_element_entity, owner);

  return owner;
}

void BillboardCloud::BoundingSphere::Initialize(const std::vector<Element>& target_elements) {
  size_t triangle_size = 0;
  auto position_sum = glm::vec3(0.f);

  // TODO: Parallelize
  for (const auto& element : target_elements) {
    for (const auto& triangle : element.triangles) {
      triangle_size++;
      position_sum += element.CalculateCentroid(triangle);
    }
  }
  center = position_sum / static_cast<float>(triangle_size);

  radius = 0.f;
  for (const auto& element : target_elements) {
    for (const auto& vertex : element.vertices) {
      radius = glm::max(radius, glm::distance(center, vertex.position));
    }
  }
}

std::vector<glm::vec3> BillboardCloud::ExtractPointCloud(const float density) const {
  std::vector<glm::vec3> points;
  BoundingSphere bounding_sphere;
  bounding_sphere.Initialize(elements);

  const auto div = glm::pow(bounding_sphere.radius * density, 2.f);
  for (const auto& element : elements) {
    std::unordered_set<unsigned> selected_indices;
    for (const auto& triangle : element.triangles) {
      const auto& v0 = element.vertices[triangle.x].position;
      const auto& v1 = element.vertices[triangle.y].position;
      const auto& v2 = element.vertices[triangle.z].position;
      const auto area = element.CalculateArea(triangle);
      const int point_count = glm::max(static_cast<int>(area / div), 1);
      for (int i = 0; i < point_count; i++) {
        float a = glm::linearRand(0.f, 1.f);
        float b = glm::linearRand(0.f, 1.f);
        if (a + b >= 1.f) {
          a = 1.f - a;
          b = 1.f - b;
        }
        const auto point = v0 + a * (v1 - v0) + b * (v2 - v0);
        points.emplace_back(point);
      }
    }
  }

  return points;
}

void BillboardCloud::ProcessPrefab(const std::shared_ptr<Prefab>& current_prefab,
                                   const Transform& parent_model_space_transform) {
  Transform transform{};
  for (const auto& data_component : current_prefab->data_components) {
    if (data_component.data_component_type == Typeof<Transform>()) {
      transform = *std::reinterpret_pointer_cast<Transform>(data_component.data_component);
    }
  }
  transform.value = parent_model_space_transform.value * transform.value;
  for (const auto& private_component : current_prefab->private_components) {
    if (private_component.private_component->GetTypeName() == "MeshRenderer") {
      std::vector<AssetRef> asset_refs;
      private_component.private_component->CollectAssetRef(asset_refs);
      std::shared_ptr<Mesh> mesh{};
      std::shared_ptr<Material> material{};
      for (auto& asset_ref : asset_refs) {
        if (const auto test_mesh = asset_ref.Get<Mesh>()) {
          mesh = test_mesh;
        } else if (const auto test_material = asset_ref.Get<Material>()) {
          material = test_material;
        }
      }
      if (mesh && material) {
        elements.emplace_back();
        auto& element = elements.back();
        element.vertices = mesh->UnsafeGetVertices();
        element.material = material;
        element.triangles = mesh->UnsafeGetTriangles();
        Jobs::RunParallelFor(element.vertices.size(), [&](const unsigned vertex_index) {
          TransformVertex(element.vertices.at(vertex_index), parent_model_space_transform.value);
        });
      }
    }
  }
  for (const auto& child_prefab : current_prefab->child_prefabs) {
    ProcessPrefab(child_prefab, transform);
  }
}

void BillboardCloud::ProcessEntity(const std::shared_ptr<Scene>& scene, const Entity& entity,
                                   const Transform& parent_model_space_transform) {
  Transform transform = scene->GetDataComponent<Transform>(entity);
  transform.value = parent_model_space_transform.value * transform.value;
  if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    const auto mesh = mesh_renderer->mesh.Get<Mesh>();
    const auto material = mesh_renderer->material.Get<Material>();
    if (mesh && material) {
      elements.emplace_back();
      auto& element = elements.back();
      element.vertices = mesh->UnsafeGetVertices();
      element.material = material;
      element.triangles = mesh->UnsafeGetTriangles();
      Jobs::RunParallelFor(element.vertices.size(), [&](const unsigned vertex_index) {
        TransformVertex(element.vertices.at(vertex_index), parent_model_space_transform.value);
      });
    }
  }

  for (const auto& child_entity : scene->GetChildren(entity)) {
    ProcessEntity(scene, child_entity, transform);
  }
}

#pragma endregion
#pragma region Clusterization
void BillboardCloud::Process(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material) {
  elements.emplace_back();
  auto& element = elements.back();
  element.vertices = mesh->UnsafeGetVertices();
  element.material = material;
  element.triangles = mesh->UnsafeGetTriangles();
}

void BillboardCloud::Process(const std::shared_ptr<Prefab>& prefab) {
  ProcessPrefab(prefab, Transform());
}

void BillboardCloud::Process(const std::shared_ptr<Scene>& scene, const Entity& entity) {
  ProcessEntity(scene, entity, Transform());
}

void BillboardCloud::Clusterize(const ClusterizationSettings& clusterize_settings) {
  clusters.clear();
  switch (clusterize_settings.clusterize_mode) {
    case static_cast<unsigned>(ClusterizationMode::FlipBook): {
      clusters.emplace_back();
      auto& cluster = clusters.back();
      cluster.triangles = CollectTriangles();
    } break;
    case static_cast<unsigned>(ClusterizationMode::Foliage): {
      std::vector<ClusterTriangle> operating_triangles = CollectTriangles();
      clusters = StochasticClusterize(std::move(operating_triangles), clusterize_settings);
    } break;
    case static_cast<unsigned>(ClusterizationMode::Original): {
      std::vector<ClusterTriangle> operating_triangles = CollectTriangles();
      clusters = DefaultClusterize(std::move(operating_triangles), clusterize_settings);
    } break;
  }
}

#pragma endregion

bool BillboardCloud::OriginalClusterizationSettings::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNode("Original clusterization settings")) {
    if (ImGui::DragFloat("Epsilon percentage", &epsilon_percentage, 0.01f, 0.01f, 1.f))
      changed = true;
    if (ImGui::DragInt("Discretization size", &discretization_size, 1, 1, 1000))
      changed = true;
    if (ImGui::DragInt("Timeout", &timeout, 1, 1, 1000))
      changed = true;

    ImGui::Checkbox("Skip remaining triangles", &skip_remain_triangles);
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::FoliageClusterizationSettings::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNode("Foliage clusterization settings")) {
    if (ImGui::DragFloat("Complexity", &density, 0.01f, 0.0f, 0.95f))
      changed = true;
    if (ImGui::DragInt("Iteration", &iteration, 1, 1, 1000))
      changed = true;
    if (ImGui::DragInt("Timeout", &timeout, 1, 1, 1000))
      changed = true;
    if (ImGui::DragFloat("Density", &sample_range, 0.01f, 0.1f, 2.f))
      changed = true;

    if (ImGui::Checkbox("Fill band", &fill_band))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::ClusterizationSettings::OnInspect() {
  bool changed = false;

  if (ImGui::TreeNode("Clusterization settings")) {
    if (ImGui::Combo("Clusterize mode", {"FlipBook", "Original", "Foliage"}, clusterize_mode)) {
      changed = true;
    }
    switch (clusterize_mode) {
      case static_cast<unsigned>(ClusterizationMode::FlipBook):
        break;
      case static_cast<unsigned>(ClusterizationMode::Foliage): {
        if (foliage_clusterization_settings.OnInspect())
          changed = true;
      } break;
      case static_cast<unsigned>(ClusterizationMode::Original): {
        if (original_clusterization_settings.OnInspect())
          changed = true;
      } break;
    }
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::ProjectSettings::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNode("Project settings")) {
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::JoinSettings::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNode("Join settings")) {
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::RasterizeSettings::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNode("Rasterize settings")) {
    if (ImGui::Checkbox("(Debug) Opaque", &debug_opaque))
      changed = true;
    if (ImGui::Checkbox("Transfer albedo texture", &transfer_albedo_map))
      changed = true;
    if (ImGui::Checkbox("Transfer normal texture", &transfer_normal_map))
      changed = true;
    if (ImGui::Checkbox("Transfer roughness texture", &transfer_roughness_map))
      changed = true;
    if (ImGui::Checkbox("Transfer metallic texture", &transfer_metallic_map))
      changed = true;
    if (ImGui::Checkbox("Transfer ao texture", &transfer_ao_map))
      changed = true;
    if (ImGui::DragInt2("Base resolution", &base_resolution.x, 1, 1, 8192))
      changed = true;
    if (ImGui::DragInt2("Output color resolution", &output_albedo_resolution.x, 1, 1, 8192))
      changed = true;
    if (ImGui::DragInt2("Output material props resolution", &output_material_props_resolution.x, 1, 1, 8192))
      changed = true;
    if (ImGui::DragInt("Dilate", &dilate, 1, -1, 1024))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::GenerateSettings::OnInspect(const std::string& title) {
  bool changed = false;
  if (ImGui::TreeNodeEx(title.c_str())) {
    if (clusterization_settings.OnInspect())
      changed = true;
    if (project_settings.OnInspect())
      changed = true;
    if (join_settings.OnInspect())
      changed = true;
    if (rasterize_settings.OnInspect())
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}
