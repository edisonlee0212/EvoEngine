#include "VisibilityTest.hpp"
#include "RayTracer.hpp"
#include "Times.hpp"

using namespace mesh_repair_plugin;

void VisibilityTest::GenerateSamples(const std::shared_ptr<Mesh>& mesh,
                                        const VisibilityTestParams& visibility_test_params,
                                        std::vector<VisibilityTestSample>& output_samples) {
  const auto& input_triangles = mesh->UnsafeGetTriangles();
  const auto& input_vertices = mesh->UnsafeGetVertices();
  auto triangle_areas = std::vector<float>(input_triangles.size());
  float total_triangle_area = 0.f;
  Jobs::RunParallelFor(triangle_areas.size(), [&](const size_t triangle_index) {
    const auto& triangle = input_triangles[triangle_index];
    const auto p0 = input_vertices[triangle.x];
    const auto p1 = input_vertices[triangle.y];
    const auto p2 = input_vertices[triangle.z];
    const auto a = glm::length(p0.position - p1.position);
    const auto b = glm::length(p2.position - p1.position);
    const auto c = glm::length(p0.position - p2.position);
    const auto d = (a + b + c) / 2;
    if (const auto area_sq = d * (d - a) * (d - b) * (d - c); area_sq != 0.f) {
      const auto area = sqrt(area_sq);
      triangle_areas.at(triangle_index) = area;
      total_triangle_area += area;
    } else
      triangle_areas.at(triangle_index) = 0.f;
  });
  const auto per_area_sample = static_cast<float>(visibility_test_params.sample_budget) / total_triangle_area;
  output_samples.resize(input_triangles.size());
  uint32_t total_samples = 0;
  for (uint32_t triangle_index = 0; triangle_index < output_samples.size(); triangle_index++) {
    auto& output_sample = output_samples.at(triangle_index);
    output_sample.triangle_index = triangle_index;
    output_sample.sample_count = glm::max(visibility_test_params.sample_minimum,
                                          static_cast<uint32_t>(per_area_sample * triangle_areas.at(triangle_index)));
    total_samples += output_sample.sample_count;
  }
  EVOENGINE_LOG("Total Generated Samples: " + std::to_string(total_samples))
}

void VisibilityTest::Execute(const std::shared_ptr<Mesh>& mesh,
                                       const std::vector<VisibilityTestSample>& input_samples,
                                       const VisibilityTestParams& visibility_test_params,
                                       std::vector<Visibility>& visibility_results) {
  const auto& input_triangles = mesh->UnsafeGetTriangles();
  const auto& input_vertices = mesh->UnsafeGetVertices();
  RayTracer cpu_ray_tracer;

  cpu_ray_tracer.Initialize(mesh);
  visibility_results.resize(input_triangles.size());
  // Trace rays...
  Jobs::RunParallelFor(input_triangles.size(), [&](const size_t triangle_index) {
    // for(uint32_t triangle_index = 0; triangle_index < input_triangles.size(); triangle_index++){
    const auto& sample = input_samples.at(triangle_index);
    RandomSampler random_sampler;
    RayTracer::RayDescriptor current_ray_descriptor{};
    if (visibility_test_params.cull_back_faces)
      current_ray_descriptor.flags =
          RayTracer::TraceFlags::CullBackFace;
    random_sampler.SetSeed(triangle_index);
    glm::vec3 current_ray_direction;
    bool visible = false;
    for (uint32_t sample_index = 0; sample_index < sample.sample_count; sample_index++) {
      auto source_triangle = input_triangles[triangle_index];
      glm::vec3 barycentric;
      //[0,1] -> [e,1-e]
      constexpr auto e = 1e-3f;
      const auto rand2d = random_sampler.Get2D();
      const auto a = sqrt(rand2d[0]) * (1 - 2 * e) + e;
      const auto b = rand2d[1] * (1 - 2 * e) + e;
      barycentric.x = 1.f - a;
      barycentric.y = a * (1 - b);
      barycentric.z = b * a;
      for (uint32_t current_depth = 0; current_depth < visibility_test_params.sample_depth; current_depth++) {
        const auto& p0 = input_vertices[source_triangle.x].position;
        const auto& p1 = input_vertices[source_triangle.y].position;
        const auto& p2 = input_vertices[source_triangle.z].position;
        auto current_ray_origin = barycentric.x * p0 + barycentric.y * p1 + barycentric.z * p2;
        // Handle degenerate triangles.
        if (p0 == p1 && p1 == p2) {
          break;
        }
        const auto normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
        auto randomly_sample_hemisphere = [](const glm::vec3& n, RandomSampler& r_s) {
          const auto tangent = glm::vec3(n.y, n.z, n.x);
          const auto bi_tangent = glm::normalize(cross(n, tangent));
          const auto temp2d = r_s.Get2D();
          auto z = temp2d[0];
          z = glm::max(z, 1e-3f);
          const auto r = sqrt(glm::max(0.f, 1.f - z * z));
          const auto phi = 2 * glm::pi<float>() * temp2d[1];
          const auto scatter_sample = glm::vec3(r * cos(phi), r * sin(phi), z);
          const auto scatter_sample_world =
              tangent * scatter_sample[0] + bi_tangent * scatter_sample[1] + n * scatter_sample[2];
          return scatter_sample_world;
        };
        bool sample_on_back_face = false;
        if ((current_depth == 0 && random_sampler.Get1D() > 0.5f) ||
            (current_depth != 0 && glm::dot(current_ray_direction, normal) > 0.f)) {
          sample_on_back_face = true;
        }
        if (sample_on_back_face) {
          current_ray_origin -= normal * 1e-3f;
          current_ray_direction = randomly_sample_hemisphere(-normal, random_sampler);
        } else {
          current_ray_origin += normal * 1e-3f;
          current_ray_direction = randomly_sample_hemisphere(normal, random_sampler);
        }

        current_ray_descriptor.origin = current_ray_origin;
        current_ray_descriptor.direction = glm::normalize(current_ray_direction);
        current_ray_descriptor.t_min = 0.f;
        current_ray_descriptor.t_max = FLT_MAX;
        cpu_ray_tracer.Trace(
            current_ray_descriptor,
            [&](const RayTracer::HitInfo& hit_info) {
              barycentric = hit_info.barycentric;
              source_triangle = input_triangles[hit_info.triangle_index];
            },
            [&]() {
              visible = true;
            },
            [](const RayTracer::HitInfo& hit_info) {
            });
        if (visible) {
          break;
        }
      }
      if (visible) {
        break;
      }
    }
    auto visibility = visibility_results.at(triangle_index);
    visibility.visible = visible;
    visibility.entity = {};
  });
}

void VisibilityTest::Execute(const std::shared_ptr<Mesh>& mesh,
                                       const VisibilityTestParams& visibility_test_params,
                                       std::vector<Visibility>& visibility_results) {
  std::vector<VisibilityTestSample> samples;
  GenerateSamples(mesh, visibility_test_params, samples);
  const auto time = Times::Now();
  Execute(mesh, samples, visibility_test_params, visibility_results);
  const auto elapsed_time = Times::Now() - time;
  EVOENGINE_LOG("Visibility test finished in: " + std::to_string(elapsed_time) + " second(s).")
}

void VisibilityTest::Execute(const std::shared_ptr<Scene>& scene, const Entity& entity,
                                       const VisibilityTestParams& visibility_test_params,
                                       std::vector<Visibility>& visibility_results) {
  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
  const auto mesh = mesh_renderer->mesh.Get<Mesh>();
  std::vector<VisibilityTestSample> input_samples;
  GenerateSamples(mesh, visibility_test_params, input_samples);
  const auto time = Times::Now();
  const auto& input_triangles = mesh->UnsafeGetTriangles();
  auto input_vertices = mesh->UnsafeGetVertices();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(entity);
  for (auto& v : input_vertices) {
    v.position = global_transform.TransformPoint(v.position);
  }

  struct VisibilityTestingMeshData {};
  struct VisibilityTestingNodeData {};
  RayTracer cpu_ray_tracer;

  cpu_ray_tracer.Initialize(
      scene,
      [](uint32_t, const std::shared_ptr<Mesh>&) {
      },
      [](uint32_t, const Entity&) {
      });
  visibility_results.resize(input_triangles.size());
  // Trace rays...
  Jobs::RunParallelFor(input_triangles.size(), [&](const size_t triangle_index) {
    const auto& sample = input_samples.at(triangle_index);
    RandomSampler random_sampler;
    RayTracer::RayDescriptor current_ray_descriptor{};
    if (visibility_test_params.cull_back_faces)
      current_ray_descriptor.flags =
          RayTracer::TraceFlags::CullBackFace;
    random_sampler.SetSeed(triangle_index);
    glm::vec3 current_ray_direction;
    bool visible = false;
    for (uint32_t sample_index = 0; sample_index < sample.sample_count; sample_index++) {
      const auto& source_triangle = input_triangles[triangle_index];
      glm::vec3 barycentric;
      //[0,1] -> [e,1-e]
      constexpr auto e = 1e-3f;
      const auto rand2d = random_sampler.Get2D();
      const auto a = sqrt(rand2d.x) * (1 - 2 * e) + e;
      const auto b = rand2d.y * (1 - 2 * e) + e;
      barycentric.x = 1.f - a;
      barycentric.y = a * (1 - b);
      barycentric.z = b * a;

      const auto& p0 = input_vertices[source_triangle.x].position;
      const auto& p1 = input_vertices[source_triangle.y].position;
      const auto& p2 = input_vertices[source_triangle.z].position;
      auto current_ray_origin = barycentric.x * p0 + barycentric.y * p1 + barycentric.z * p2;
      if (p0 == p1 && p1 == p2) {
        break;
      }
      auto current_triangle_normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
      for (uint32_t current_depth = 0; current_depth < visibility_test_params.sample_depth; current_depth++) {
        auto randomly_sample_hemisphere = [](const glm::vec3& n, RandomSampler& r_s) {
          const auto tangent = glm::vec3(n.y, n.z, n.x);
          const auto bi_tangent = glm::normalize(cross(n, tangent));
          const auto temp2d = r_s.Get2D();
          auto z = temp2d[0];
          z = glm::max(z, 1e-3f);
          const auto r = sqrt(glm::max(0.f, 1.f - z * z));
          const auto phi = 2 * glm::pi<float>() * temp2d[1];
          const auto scatter_sample = glm::vec3(r * cos(phi), r * sin(phi), z);
          const auto scatter_sample_world =
              tangent * scatter_sample[0] + bi_tangent * scatter_sample[1] + n * scatter_sample[2];
          return scatter_sample_world;
        };
        bool sample_on_back_face = false;
        if ((current_depth == 0 && random_sampler.Get1D() > 0.5f) ||
            (current_depth != 0 && glm::dot(current_ray_direction, current_triangle_normal) > 0.f)) {
          sample_on_back_face = true;
        }
        if (sample_on_back_face) {
          if (current_depth != 0)
            current_ray_origin -= current_triangle_normal * 1e-3f;
          current_ray_direction = randomly_sample_hemisphere(-current_triangle_normal, random_sampler);
        } else {
          if (current_depth != 0)
            current_ray_origin += current_triangle_normal * 1e-3f;
          current_ray_direction = randomly_sample_hemisphere(current_triangle_normal, random_sampler);
        }

        current_ray_descriptor.origin = current_ray_origin;
        current_ray_descriptor.direction = glm::normalize(current_ray_direction);
        current_ray_descriptor.t_min = 0.f;
        current_ray_descriptor.t_max = FLT_MAX;
        cpu_ray_tracer.Trace(
            current_ray_descriptor,
            [&](const RayTracer::HitInfo& hit_info) {
              current_ray_origin = hit_info.hit;
              current_triangle_normal = glm::normalize(hit_info.normal);
            },
            [&]() {
              visible = true;
            },
            [](const RayTracer::HitInfo& hit_info) {
            });
        if (visible) {
          break;
        }
      }
      if (visible) {
        break;
      }
    }
    auto& visibility = visibility_results.at(triangle_index);
    visibility.visible = visible;
    visibility.triangle_index = static_cast<uint32_t>(triangle_index);
    visibility.entity = entity;
  });

  const auto elapsed_time = Times::Now() - time;
  EVOENGINE_LOG("Visibility test finished in: " + std::to_string(elapsed_time) + " second(s).")
}