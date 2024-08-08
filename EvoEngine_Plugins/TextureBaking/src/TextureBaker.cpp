#include "TextureBaker.hpp"

#include "CpuRayTracer.hpp"
#include "Times.hpp"
using namespace evo_engine;

#define TEXTURE_BAKER_REPORT_TIME 1;

inline glm::vec3 Barycentric(const glm::vec2& p, const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& p2) {
  const glm::vec2 v0 = p1 - p0;
  const glm::vec2 v1 = p2 - p0;
  const glm::vec2 v2 = p - p0;
  const float s = 1.0f / (v0.x * v1.y - v1.x * v0.y);
  const float i = (v2.x * v1.y - v1.x * v2.y) * s;
  const float j = (v0.x * v2.y - v2.x * v0.y) * s;
  return {1.0f - i - j, i, j};
}

struct CompressedMapUv {
  glm::uvec2 resolution{};
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<glm::vec3> tangents;
  std::vector<glm::vec3> directions;
  std::vector<uint32_t> indices;
  CompressedMapUv() = default;
  void Initialize(const std::vector<Vertex>& target_vertices, const std::vector<glm::uvec3>& target_triangles,
                  const std::vector<glm::vec3>& ray_directions, const glm::uvec2& target_resolution);
};

void CompressedMapUv::Initialize(const std::vector<Vertex>& target_vertices,
                                 const std::vector<glm::uvec3>& target_triangles,
                                 const std::vector<glm::vec3>& ray_directions, const glm::uvec2& target_resolution) {
  resolution = target_resolution;
  std::vector<glm::vec3> uncompressed_positions;
  std::vector<glm::vec3> uncompressed_normals;
  std::vector<glm::vec3> uncompressed_tangents;
  std::vector<glm::vec3> uncompressed_directions;
  size_t pixel_count = resolution.x * resolution.y;
  uncompressed_positions.resize(pixel_count, glm::vec3());
  uncompressed_normals.resize(pixel_count, glm::vec3());
  uncompressed_tangents.resize(pixel_count, glm::vec3());
  uncompressed_directions.resize(pixel_count, glm::vec3());
  const glm::vec2 scale(static_cast<float>(resolution.x), static_cast<float>(resolution.y));
  const glm::vec2 pixel_size = glm::vec2(1.0f) / scale;
  const glm::vec2 half_pixel_size = pixel_size * 0.5f;
  for (const auto& triangle : target_triangles) {
    const auto& v0 = target_vertices[triangle.x];
    const auto& v1 = target_vertices[triangle.y];
    const auto& v2 = target_vertices[triangle.z];
    const auto& d0 = ray_directions[triangle.x];
    const auto& d1 = ray_directions[triangle.y];
    const auto& d2 = ray_directions[triangle.z];
    const glm::vec2 u01 = (v0.tex_coord - half_pixel_size) * scale;
    const glm::vec2 u11 = (v1.tex_coord - half_pixel_size) * scale;
    const glm::vec2 u21 = (v2.tex_coord - half_pixel_size) * scale;
    const uint32_t x_min =
        std::min(static_cast<uint32_t>(std::fmaxf(std::roundf(std::fminf(u01.x, std::fminf(u11.x, u21.x))), 0.0)),
                 resolution.x - 1);
    const uint32_t y_min =
        std::min(static_cast<uint32_t>(std::fmaxf(std::roundf(std::fminf(u01.y, std::fminf(u11.y, u21.y))), 0.0)),
                 resolution.y - 1);
    const uint32_t x_max =
        std::min(static_cast<uint32_t>(std::fmaxf(std::roundf(std::fmaxf(u01.x, std::fmaxf(u11.x, u21.x))), 0.0)),
                 resolution.x - 1);
    const uint32_t y_max =
        std::min(static_cast<uint32_t>(std::fmaxf(std::roundf(std::fmaxf(u01.y, std::fmaxf(u11.y, u21.y))), 0.0)),
                 resolution.y - 1);
    for (uint32_t y = y_min; y <= y_max; ++y) {
      for (uint32_t x = x_min; x <= x_max; ++x) {
        const glm::vec2 xy(static_cast<float>(x), static_cast<float>(y));
        const glm::vec2 uv = xy * pixel_size + half_pixel_size;  // this has a question!
        const glm::vec3 b = Barycentric(uv, v0.tex_coord, v1.tex_coord, v2.tex_coord);
        if (b.x >= -0.001f && b.x <= 1 && b.y >= -0.001f && b.y <= 1 && b.z >= -0.001f && b.z <= 1) {
          const auto i = y * resolution.x + x;
          uncompressed_positions[i] = v0.position * b.x + v1.position * b.y + v2.position * b.z;
          uncompressed_directions[i] = glm::normalize(d0 * b.x + d1 * b.y + d2 * b.z);
          uncompressed_normals[i] = glm::normalize(v0.normal * b.x + v1.normal * b.y + v2.normal * b.z);
          uncompressed_tangents[i] = glm::normalize(v0.tangent * b.x + v1.tangent * b.y + v2.tangent * b.z);
        }
      }
    }
  }
  for (size_t i = 0; i < uncompressed_directions.size(); ++i) {
    if (const auto& n = uncompressed_directions[i]; glm::dot(n, n) > 0.5f) {
      indices.emplace_back(static_cast<uint32_t>(i));
    }
  }
  positions.resize(indices.size());
  directions.resize(indices.size());
  normals.resize(indices.size());
  tangents.resize(indices.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    const size_t idx = indices[i];
    positions[i] = uncompressed_positions[idx];
    directions[i] = uncompressed_directions[idx];
    normals[i] = uncompressed_normals[idx];
    tangents[i] = uncompressed_tangents[idx];
  }
}

struct RayCastingResult {
  bool miss = false;
  CpuRayTracer::HitInfo hit_info;
};

std::vector<RayCastingResult> RayCastingSolver(const TextureBaker::Parameters& parameters,
                                               const CompressedMapUv& compressed_map_uv,
                                               const CpuRayTracer& ray_tracer, const float thin_scale,
                                               const float max_ray_casting_distance) {
  auto ray_casting_results = std::vector(compressed_map_uv.indices.size(), RayCastingResult());
  Jobs::RunParallelFor(compressed_map_uv.positions.size(), [&](const size_t i) {
    CpuRayTracer::RayDescriptor ray_descriptor{};
    auto& result = ray_casting_results[i];
    ray_descriptor.flags = parameters.cull_back_face ? CpuRayTracer::TraceFlags::CullBackFace
                                                     : CpuRayTracer::TraceFlags::None;
    //  Compare and get closest triangle.
    //  Diagram:
    //
    //  (1)    m_r_c_d    | t_s
    //  <-----------------|----<
    //                    |     m_r_c_d - t_s
    //  (2)               |    >------------->

    //  1. Try to find triangle that's within thin scale in normal direction.
    const auto& sample_origin = compressed_map_uv.positions[i];
    const auto& sample_direction = compressed_map_uv.directions[i];
    ray_descriptor.origin = sample_origin + sample_direction * thin_scale;
    ray_descriptor.direction = -sample_direction;
    ray_descriptor.t_max = thin_scale + max_ray_casting_distance;
    bool hit = false;
    ray_tracer.Trace(
        ray_descriptor,
        [&](const CpuRayTracer::HitInfo& hit_info) {
          result.hit_info = hit_info;
          hit = true;
        },
        [&] {
        },
        [&](const auto&) {
        });
    // 2. If we can't find any triangle, reverse direction and search again.
    if (!hit) {
      ray_descriptor.direction = sample_direction;
      ray_descriptor.t_max = max_ray_casting_distance - thin_scale;
      ray_descriptor.flags = parameters.cull_back_face ? CpuRayTracer::TraceFlags::CullFrontFace
                                                       : CpuRayTracer::TraceFlags::None;
      ray_tracer.Trace(
          ray_descriptor,
          [&](const CpuRayTracer::HitInfo& hit_info) {
            result.hit_info = hit_info;
            hit = true;
          },
          [&] {
          },
          [&](const auto&) {
          });
    }
    result.miss = !hit;
  });
  return ray_casting_results;
}

template <typename T>
void DilateUnresolvedPixels(const glm::uvec2 resolution, const bool diagonal,
                            const std::vector<size_t>& unresolved_pixel_indices, std::vector<T>& colors,
                            std::vector<uint8_t>& valid_pixels) {
  auto is_valid_pixel = [](const std::vector<uint8_t>& pixels, const int x, const int y, const int w,
                           const int h) -> uint8_t {
    if (x < 0 || y < 0 || x >= w || y >= h) {
      return 0u;
    }
    return pixels[x + y * w];
  };
  for (;;) {
    std::vector<size_t> updated_pixel_indices;
    int w = static_cast<int>(resolution.x);
    int h = static_cast<int>(resolution.y);
    std::mutex update_mutex;
    Jobs::RunParallelFor(unresolved_pixel_indices.size(), [&](const auto unresolved_list_index) {
      const size_t pixel_index = unresolved_pixel_indices[unresolved_list_index];
      if (valid_pixels[pixel_index])
        return;
      const auto center_x = static_cast<int>(pixel_index % resolution.x);
      const auto center_y = static_cast<int>(pixel_index / resolution.x);
      auto is_valid = is_valid_pixel(valid_pixels, center_x - 1, center_y, w, h) ||
                      is_valid_pixel(valid_pixels, center_x + 1, center_y, w, h) ||
                      is_valid_pixel(valid_pixels, center_x, center_y + 1, w, h) ||
                      is_valid_pixel(valid_pixels, center_x, center_y - 1, w, h);
      if (diagonal) {
        is_valid = is_valid || is_valid_pixel(valid_pixels, center_x - 1, center_y - 1, w, h) ||
                   is_valid_pixel(valid_pixels, center_x - 1, center_y + 1, w, h) ||
                   is_valid_pixel(valid_pixels, center_x + 1, center_y - 1, w, h) ||
                   is_valid_pixel(valid_pixels, center_x + 1, center_y + 1, w, h);
      }
      if (is_valid) {
        float sum_weight = 0;
        T sum_color{};
        for (int i = -1; i <= 1; i++) {
          for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0)
              continue;
            const auto test_x = center_x + i;
            const auto test_y = center_y + j;
            if (is_valid_pixel(valid_pixels, test_x, test_y, w, h)) {
              const float weight = 1.0f / static_cast<float>(abs(i) + abs(j));
              sum_weight += weight;
              const auto sample_pixel_index = test_x + w * test_y;
              sum_color += colors[sample_pixel_index] * weight;
            }
          }
        }
        sum_color /= sum_weight;
        colors[pixel_index] = sum_color;
        std::lock_guard lock(update_mutex);
        updated_pixel_indices.push_back(pixel_index);
      }
    });
    if (updated_pixel_indices.empty())
      break;
    for (const auto& i : updated_pixel_indices) {
      valid_pixels[i] = true;
    }
  }
}
template <typename T>
void Dilate(const glm::uvec2 resolution, const int32_t dilate_distance, const bool diagonal, std::vector<T>& colors,
            std::vector<uint8_t>& valid_pixels) {
  auto is_valid_pixel = [](const std::vector<uint8_t>& pixels, const int x, const int y, const int w,
                           const int h) -> uint8_t {
    if (x < 0 || y < 0 || x >= w || y >= h) {
      return 0u;
    }
    return pixels[x + y * w];
  };
  for (int32_t dilate_index = 0; dilate_index != dilate_distance; dilate_index++) {
    std::vector<size_t> updated_pixel_indices;
    int w = static_cast<int>(resolution.x);
    int h = static_cast<int>(resolution.y);
    std::mutex update_mutex;
    Jobs::RunParallelFor(colors.size(), [&](const auto pixel_index) {
      if (valid_pixels[pixel_index])
        return;
      const auto center_x = static_cast<int>(pixel_index % resolution.x);
      const auto center_y = static_cast<int>(pixel_index / resolution.x);
      auto is_valid = is_valid_pixel(valid_pixels, center_x - 1, center_y, w, h) ||
                      is_valid_pixel(valid_pixels, center_x + 1, center_y, w, h) ||
                      is_valid_pixel(valid_pixels, center_x, center_y + 1, w, h) ||
                      is_valid_pixel(valid_pixels, center_x, center_y - 1, w, h);
      if (diagonal) {
        is_valid = is_valid || is_valid_pixel(valid_pixels, center_x - 1, center_y - 1, w, h) ||
                   is_valid_pixel(valid_pixels, center_x - 1, center_y + 1, w, h) ||
                   is_valid_pixel(valid_pixels, center_x + 1, center_y - 1, w, h) ||
                   is_valid_pixel(valid_pixels, center_x + 1, center_y + 1, w, h);
      }
      if (is_valid) {
        float sum_weight = 0;
        T sum_color{};
        for (int i = -1; i <= 1; i++) {
          for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0)
              continue;
            const auto test_x = center_x + i;
            const auto test_y = center_y + j;
            if (is_valid_pixel(valid_pixels, test_x, test_y, w, h)) {
              const float weight = 1.0f / static_cast<float>(abs(i) + abs(j));
              sum_weight += weight;
              const auto sample_pixel_index = test_x + w * test_y;
              sum_color += colors[sample_pixel_index] * weight;
            }
          }
        }
        sum_color /= sum_weight;
        colors[pixel_index] = sum_color;
        std::lock_guard lock(update_mutex);
        updated_pixel_indices.push_back(pixel_index);
      }
    });
    if (updated_pixel_indices.empty())
      break;
    for (const auto& i : updated_pixel_indices) {
      valid_pixels[i] = true;
    }
  }
}

template <typename T>
T ImageSampling(const std::vector<T>& image_data, const glm::uvec2 image_resolution, const glm::vec2& uv,
                const TextureBaker::SamplingMethod sampling_method) {
  const auto x = uv.x * static_cast<float>(image_resolution.x);
  const auto y = uv.y * static_cast<float>(image_resolution.y);
  switch (sampling_method) {
    case TextureBaker::SamplingMethod::Nearest:
      return image_data[glm::clamp(static_cast<uint32_t>(x), 0u, image_resolution.x) +
                        image_resolution.x * glm::clamp(static_cast<uint32_t>(y), 0u, image_resolution.y)];
    case TextureBaker::SamplingMethod::Bilinear:
      break;
  }
  return T(0.f);
};

template <typename T>
void DilatePostProcess(const TextureBaker::Parameters& parameters, const std::vector<size_t>& unresolved_pixel_indices,
                       std::vector<T>& target_color, std::vector<uint8_t>& valid_pixels) {
  if (!unresolved_pixel_indices.empty()) {
    DilateUnresolvedPixels(parameters.texture_resolution, false, unresolved_pixel_indices, target_color, valid_pixels);
    DilateUnresolvedPixels(parameters.texture_resolution, true, unresolved_pixel_indices, target_color, valid_pixels);
  }
  Dilate(parameters.texture_resolution, parameters.post_dilate_distance, false, target_color, valid_pixels);
  Dilate(parameters.texture_resolution, parameters.post_dilate_distance, true, target_color, valid_pixels);
}

template <typename T>
void BakeColor(const TextureBaker::Parameters& parameters, const CompressedMapUv& compressed_map_uv,
               const std::vector<RayCastingResult>& ray_casting_results,
               const std::vector<Vertex>& reference_mesh_vertices,
               const std::vector<glm::uvec3>& reference_mesh_triangles, const std::shared_ptr<Texture2D>& ref_texture,
               std::vector<T>& target_color, double& loading_time, double& processing_time) {
  auto time = Times::Now();
  processing_time += Times::Now() - time;
  time = Times::Now();
  std::vector<T> ref_data{};
  std::vector<uint8_t> valid_pixels(target_color.size(), 0);
  ref_texture->GetData(ref_data);
  const auto ref_resolution = ref_texture->GetResolution();
  loading_time += Times::Now() - time;
  time = Times::Now();
  std::vector<size_t> unresolved_pixel_indices{};
  std::mutex unresolved_pixel_mutex{};
  Jobs::RunParallelFor(ray_casting_results.size(), [&](const size_t i) {
    const auto pixel_index = compressed_map_uv.indices[i];
    if (const auto& ray_casting_result = ray_casting_results[i]; !ray_casting_result.miss) {
      const auto& triangle = reference_mesh_triangles[ray_casting_result.hit_info.triangle_index];
      const auto& reference_v0 = reference_mesh_vertices[triangle.x];
      const auto& reference_v1 = reference_mesh_vertices[triangle.y];
      const auto& reference_v2 = reference_mesh_vertices[triangle.z];
      const auto uv = reference_v0.tex_coord * ray_casting_result.hit_info.barycentric.x +
                      reference_v1.tex_coord * ray_casting_result.hit_info.barycentric.y +
                      reference_v2.tex_coord * ray_casting_result.hit_info.barycentric.z;
      target_color[pixel_index] = ImageSampling(ref_data, ref_resolution, uv, parameters.sampling_method);
      valid_pixels[pixel_index] = true;
    } else {
      switch (parameters.unresolved_pixel_mode) {
        case TextureBaker::UnresolvedPixelMode::Default:
          target_color[pixel_index] = parameters.unresolved_pixel_color;
          valid_pixels[pixel_index] = true;
          break;
        case TextureBaker::UnresolvedPixelMode::Dilate: {
          std::lock_guard lock(unresolved_pixel_mutex);
          unresolved_pixel_indices.emplace_back(pixel_index);
        } break;
      }
    }
  });
  DilatePostProcess(parameters, unresolved_pixel_indices, target_color, valid_pixels);
  processing_time += Times::Now() - time;
}

void BakeNormal(const TextureBaker::Parameters& parameters, const CompressedMapUv& compressed_map_uv,
                const std::vector<RayCastingResult>& ray_casting_results,
                const std::vector<Vertex>& reference_mesh_vertices,
                const std::vector<glm::uvec3>& reference_mesh_triangles, const std::shared_ptr<Texture2D>& ref_texture,
                std::vector<glm::vec3>& target_normals, double& loading_time, double& processing_time) {
  auto time = Times::Now();
  processing_time += Times::Now() - time;
  time = Times::Now();
  std::vector<glm::vec3> ref_data{};
  std::vector<uint8_t> valid_pixels(target_normals.size(), 0);
  glm::uvec2 ref_resolution;
  bool has_texture = false;
  if (ref_texture) {
    has_texture = true;
    ref_texture->GetData(ref_data);
    ref_resolution = ref_texture->GetResolution();
  }
  loading_time += Times::Now() - time;
  time = Times::Now();
  std::vector<size_t> unresolved_pixel_indices{};
  std::mutex unresolved_pixel_mutex{};
  Jobs::RunParallelFor(ray_casting_results.size(), [&](const size_t i) {
    const auto pixel_index = compressed_map_uv.indices[i];
    if (const auto& ray_casting_result = ray_casting_results[i]; !ray_casting_result.miss) {
      const auto& triangle = reference_mesh_triangles[ray_casting_result.hit_info.triangle_index];
      const auto& reference_v0 = reference_mesh_vertices[triangle.x];
      const auto& reference_v1 = reference_mesh_vertices[triangle.y];
      const auto& reference_v2 = reference_mesh_vertices[triangle.z];
      const auto uv = reference_v0.tex_coord * ray_casting_result.hit_info.barycentric.x +
                      reference_v1.tex_coord * ray_casting_result.hit_info.barycentric.y +
                      reference_v2.tex_coord * ray_casting_result.hit_info.barycentric.z;
      auto reference_normal = reference_v0.normal * ray_casting_result.hit_info.barycentric.x +
                              reference_v1.normal * ray_casting_result.hit_info.barycentric.y +
                              reference_v2.normal * ray_casting_result.hit_info.barycentric.z;
      const auto reference_tangent = reference_v0.tangent * ray_casting_result.hit_info.barycentric.x +
                                     reference_v1.tangent * ray_casting_result.hit_info.barycentric.y +
                                     reference_v2.tangent * ray_casting_result.hit_info.barycentric.z;
      const glm::vec3 reference_bitangent = glm::cross(reference_normal, reference_tangent);
      const auto reference_tbn = glm::mat3(reference_tangent, reference_bitangent, reference_normal);
      const auto& target_normal = compressed_map_uv.normals[i];
      const auto& target_tangent = compressed_map_uv.tangents[i];
      const auto target_bitangent = glm::cross(target_normal, target_tangent);
      const auto target_tbn = glm::mat3(target_tangent, target_bitangent, target_normal);
      if (has_texture) {
        auto texture_normal = ImageSampling(ref_data, ref_resolution, uv, parameters.sampling_method);
        texture_normal = texture_normal * 2.0f - glm::vec3(1.0f);
        const auto mesh_space_normal = glm::normalize(reference_tbn * texture_normal);
        reference_normal = glm::normalize(glm::inverse(target_tbn) * mesh_space_normal);
      } else {
        reference_normal = glm::normalize(glm::inverse(target_tbn) * reference_normal);
      }
      target_normals[pixel_index] = reference_normal * 0.5f + glm::vec3(0.5f);
      valid_pixels[pixel_index] = true;
    } else {
      switch (parameters.unresolved_pixel_mode) {
        case TextureBaker::UnresolvedPixelMode::Default:
          target_normals[pixel_index] = glm::vec3(0.f);
          valid_pixels[pixel_index] = true;
          break;
        case TextureBaker::UnresolvedPixelMode::Dilate: {
          std::lock_guard lock(unresolved_pixel_mutex);
          unresolved_pixel_indices.emplace_back(pixel_index);
        } break;
      }
    }
  });
  DilatePostProcess(parameters, unresolved_pixel_indices, target_normals, valid_pixels);
  processing_time += Times::Now() - time;
}

void TextureBaker::Execute(const Parameters& parameters, const std::shared_ptr<Mesh>& reference_mesh,
                           const std::shared_ptr<Material>& reference_material,
                           const std::shared_ptr<Mesh>& target_mesh, const std::shared_ptr<Material>& target_material) {
  auto time = Times::Now();
  const auto& reference_mesh_vertices = reference_mesh->UnsafeGetVertices();
  const auto& reference_mesh_triangles = reference_mesh->UnsafeGetTriangles();
  const auto& target_mesh_vertices = target_mesh->UnsafeGetVertices();
  const auto& target_mesh_triangles = target_mesh->UnsafeGetTriangles();
  std::vector<glm::vec3> ray_directions(target_mesh_vertices.size());
  switch (parameters.ray_casting_direction_mode) {
    case RayCastingDirectionMode::Default:
      for (uint32_t vertex_index = 0; vertex_index < target_mesh_vertices.size(); vertex_index++) {
        ray_directions[vertex_index] = target_mesh_vertices[vertex_index].normal;
      }
      break;
    case RayCastingDirectionMode::AggressiveSmoothing: {
      // Construct smoothed normals to reduce ray misses and ray overlaps
      auto comp = [](const glm::vec3& lhs, const glm::vec3& rhs) {
        return std::tie(lhs.x, lhs.y, lhs.z) < std::tie(rhs.x, rhs.y, rhs.z);
      };
      std::map<glm::vec3, glm::vec3, decltype(comp)> normals_map(comp);
      auto try_add_normal = [&](const glm::vec3& p, const glm::vec3& n) {
        if (const auto it = normals_map.find(p); it != normals_map.end()) {
          it->second += n;
        }
        normals_map[p] = n;
      };
      for (const auto& triangle : target_mesh_triangles) {
        const auto& p0 = target_mesh_vertices[triangle.x].position;
        const auto& p1 = target_mesh_vertices[triangle.y].position;
        const auto& p2 = target_mesh_vertices[triangle.z].position;
        const auto n = glm::normalize(glm::cross(p1 - p0, p2 - p0));
        if (std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (n.x == n.y && n.y == n.z && n.z == 0))
          continue;
        try_add_normal(p0, n);
        try_add_normal(p1, n);
        try_add_normal(p2, n);
      }
      for (auto& n : normals_map) {
        n.second = glm::normalize(n.second);
      }
      for (uint32_t vertex_index = 0; vertex_index < target_mesh_vertices.size(); vertex_index++) {
        ray_directions[vertex_index] = normals_map.at(target_mesh_vertices[vertex_index].position);
      }
    } break;
  }
  CpuRayTracer ray_tracer;
  ray_tracer.Initialize(reference_mesh);
  float diagonal_length = glm::length(target_mesh->GetBound().Size());
  float max_ray_casting_distance = parameters.ray_casting_range * diagonal_length;
  float thin_scale = parameters.thin_scale_factor * diagonal_length;
  CompressedMapUv compressed_map_uv;
  std::vector<RayCastingResult> ray_casting_result;
  compressed_map_uv.Initialize(target_mesh_vertices, target_mesh_triangles, ray_directions,
                               parameters.texture_resolution);
  ray_casting_result =
      RayCastingSolver(parameters, compressed_map_uv, ray_tracer, thin_scale, max_ray_casting_distance);
  double loading_time = 0.;
  double processing_time = Times::Now() - time;
  if (const auto ref_diffuse_texture = reference_material->GetAlbedoTexture();
      parameters.diffuse_enabled && ref_diffuse_texture) {
    std::vector diffuse_color(parameters.texture_resolution.x * parameters.texture_resolution.y,
                              parameters.empty_space_color);
    BakeColor(parameters, compressed_map_uv, ray_casting_result, reference_mesh_vertices, reference_mesh_triangles,
              ref_diffuse_texture, diffuse_color, loading_time, processing_time);
    const auto target_diffuse_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
    target_diffuse_texture->SetRgbaChannelData(diffuse_color, parameters.texture_resolution);
    target_material->SetAlbedoTexture(target_diffuse_texture);
  }
  const auto ref_normal_texture = reference_material->GetNormalTexture();
  if (parameters.normal_enabled) {
    std::vector normal_color(parameters.texture_resolution.x * parameters.texture_resolution.y, glm::vec3(0.f));
    BakeNormal(parameters, compressed_map_uv, ray_casting_result, reference_mesh_vertices, reference_mesh_triangles,
               ref_normal_texture, normal_color, loading_time, processing_time);
    const auto target_normal_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
    target_normal_texture->SetRgbChannelData(normal_color, parameters.texture_resolution);
    target_material->SetNormalTexture(target_normal_texture);
  }
  if (const auto ref_roughness_texture = reference_material->GetRoughnessTexture();
      parameters.roughness_enabled && ref_roughness_texture) {
    std::vector roughness_color(parameters.texture_resolution.x * parameters.texture_resolution.y, glm::vec3(0.f));
    BakeColor(parameters, compressed_map_uv, ray_casting_result, reference_mesh_vertices, reference_mesh_triangles,
              ref_roughness_texture, roughness_color, loading_time, processing_time);
    const auto target_roughness_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
    target_roughness_texture->SetRgbChannelData(roughness_color, parameters.texture_resolution);
    target_material->SetRoughnessTexture(target_roughness_texture);
  }
  if (const auto ref_metallic_texture = reference_material->GetMetallicTexture();
      parameters.metallic_enabled && ref_metallic_texture) {
    std::vector metallic_color(parameters.texture_resolution.x * parameters.texture_resolution.y, glm::vec3(0.f));
    BakeColor(parameters, compressed_map_uv, ray_casting_result, reference_mesh_vertices, reference_mesh_triangles,
              ref_metallic_texture, metallic_color, loading_time, processing_time);
    const auto target_metallic_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
    target_metallic_texture->SetRgbChannelData(metallic_color, parameters.texture_resolution);
    target_material->SetMetallicTexture(target_metallic_texture);
  }
  if (const auto ref_ao_texture = reference_material->GetAoTexture(); parameters.ao_enabled && ref_ao_texture) {
    std::vector ao_color(parameters.texture_resolution.x * parameters.texture_resolution.y, glm::vec3(0.f));
    BakeColor(parameters, compressed_map_uv, ray_casting_result, reference_mesh_vertices, reference_mesh_triangles,
              ref_ao_texture, ao_color, loading_time, processing_time);
    const auto target_ao_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
    target_ao_texture->SetRgbChannelData(ao_color, parameters.texture_resolution);
    target_material->SetAoTexture(target_ao_texture);
  }
#ifdef TEXTURE_BAKER_REPORT_TIME
  EVOENGINE_LOG("Texture baking finished!\nLoading time: " + std::to_string(loading_time) +
                "s. \nProcessing time: " + std::to_string(processing_time) + "s.")
#endif
}