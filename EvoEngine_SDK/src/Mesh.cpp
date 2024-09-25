#include "Mesh.hpp"

#include "ClassRegistry.hpp"
#include "Console.hpp"
#include "GeometryStorage.hpp"
#include "Jobs.hpp"
#include "Platform.hpp"
#include "Utilities.hpp"
using namespace evo_engine;

bool Mesh::SaveInternal(const std::filesystem::path& path) const {
  if (path.extension() == ".evemesh") {
    return IAsset::SaveInternal(path);
  }
  if (path.extension() == ".obj") {
    std::ofstream of;
    of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
    if (of.is_open()) {
      std::string start = "#Mesh exporter, by Bosheng Li";
      start += "\n";
      of.write(start.c_str(), start.size());
      of.flush();
      if (!triangles_.empty()) {
        unsigned start_index = 1;
        std::string header =
            "#Vertices: " + std::to_string(vertices_.size()) + ", tris: " + std::to_string(triangles_.size());
        header += "\n";
        of.write(header.c_str(), header.size());
        of.flush();
        std::stringstream data;
#pragma region Data collection
        for (auto i = 0; i < vertices_.size(); i++) {
          auto& vertex_position = vertices_.at(i).position;
          auto& color = vertices_.at(i).color;
          data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                      std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                      std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
        }
        for (const auto& vertex : vertices_) {
          data << "vn " + std::to_string(vertex.normal.x) + " " + std::to_string(vertex.normal.y) + " " +
                      std::to_string(vertex.normal.z) + "\n";
        }

        for (const auto& vertex : vertices_) {
          data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
        }
        // data += "s off\n";
        data << "# List of indices for faces vertices, with (x, y, z).\n";
        auto& triangles = triangles_;
        for (auto i = 0; i < triangles_.size(); i++) {
          const auto triangle = triangles[i];
          const auto f1 = triangle.x + start_index;
          const auto f2 = triangle.y + start_index;
          const auto f3 = triangle.z + start_index;
          data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                      std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                      std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
        }
        start_index += vertices_.size();
#pragma endregion
        const auto result = data.str();
        of.write(result.c_str(), result.size());
        of.flush();
      }
      of.close();
      return true;
    }
    EVOENGINE_ERROR("Can't open file!");
    return false;
  }
  return false;
}

bool Mesh::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Text(("Vertices size: " + std::to_string(vertices_.size())).c_str());
  ImGui::Text(("Triangle amount: " + std::to_string(triangles_.size())).c_str());
  if (!vertices_.empty()) {
    FileUtils::SaveFile(
        "Export as OBJ", "Mesh", {".obj"},
        [&](const std::filesystem::path& path) {
          Export(path);
        },
        false);
  }
  /*
  static bool visualize = true;
  static std::shared_ptr<Camera> visualizationCamera;
  static ImVec2 visualizationCameraResolution = { 200, 200 };
  if (visualize) {
          if (!visualizationCamera) {
                  visualizationCamera = Serialization::ProduceSerializable<Camera>();
                  visualizationCamera->clear_color = glm::vec3(0.0f);
                  visualizationCamera->use_clear_color = true;
                  visualizationCamera->OnCreate();
          }
          else
          {
                  // Show texture first;
                  // Render for next frame;
                  visualizationCamera->ResizeResolution(visualizationCameraResolution.x,
  visualizationCameraResolution.y); visualizationCamera->Clear(); auto renderLayer =
  Application::GetLayer<RenderLayer>(); static GlobalTransform visCameraGT;
                  renderLayer->RenderToCamera(visualizationCamera, visCameraGT);
                  ImGui::Image(
                          reinterpret_cast<ImTextureID>(visualizationCamera->GetTexture()->UnsafeGetGLTexture()->Id()),
                          visualizationCameraResolution,
                          ImVec2(0, 1),
                          ImVec2(1, 0));
          }
  }
  */
  return changed;
}

void Mesh::OnCreate() {
  version_ = 0;
  bound_ = Bound();
  triangle_range_ = std::make_shared<RangeDescriptor>();
  meshlet_range_ = std::make_shared<RangeDescriptor>();
}

Mesh::~Mesh() {
  GeometryStorage::FreeMesh(GetHandle());
  triangle_range_.reset();
  meshlet_range_.reset();
}

void Mesh::DrawIndexed(VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                       const int instances_count) const {
  if (instances_count == 0)
    return;
  global_pipeline_state.ApplyAllStates(vk_command_buffer);
  vkCmdDrawIndexed(vk_command_buffer, triangle_range_->prev_frame_index_count * 3, instances_count,
                   triangle_range_->prev_frame_offset * 3, 0, 0);
}

void Mesh::SetVertices(const VertexAttributes& vertex_attributes, const std::vector<Vertex>& vertices,
                       const std::vector<unsigned>& indices) {
  if (indices.size() % 3 != 0) {
    EVOENGINE_ERROR("Triangle size wrong!");
    return;
  }
  std::vector<glm::uvec3> triangles;
  triangles.resize(indices.size() / 3);
  memcpy(triangles.data(), indices.data(), indices.size() * sizeof(unsigned));
  SetVertices(vertex_attributes, vertices, triangles);
}

void Mesh::SetVertices(const VertexAttributes& vertex_attributes, const std::vector<Vertex>& vertices,
                       const std::vector<glm::uvec3>& triangles) {
  if (vertices.empty() || triangles.empty()) {
#ifndef NDEBUG
    EVOENGINE_LOG("Vertices or triangles empty!");
#endif
    return;
  }
  vertices_ = vertices;
  triangles_ = triangles;

#pragma region Bound
  glm::vec3 min_bound = vertices_.at(0).position;
  glm::vec3 max_bound = vertices_.at(0).position;
  for (const auto& vertex : vertices_) {
    min_bound = glm::vec3((glm::min)(min_bound.x, vertex.position.x), (glm::min)(min_bound.y, vertex.position.y),
                          (glm::min)(min_bound.z, vertex.position.z));
    max_bound = glm::vec3((glm::max)(max_bound.x, vertex.position.x), (glm::max)(max_bound.y, vertex.position.y),
                          (glm::max)(max_bound.z, vertex.position.z));
  }
  bound_.max = max_bound;
  bound_.min = min_bound;
#pragma endregion
  if (!vertex_attributes.normal)
    RecalculateNormal();
  if (!vertex_attributes.tangent)
    RecalculateTangent();

  vertex_attributes_ = vertex_attributes;
  vertex_attributes_.normal = true;
  vertex_attributes_.tangent = true;

  // MergeVertices();

  if (version_ != 0)
    GeometryStorage::FreeMesh(GetHandle());
  GeometryStorage::AllocateMesh(GetHandle(), vertices_, triangles_, meshlet_range_, triangle_range_);

  version_++;

  /*
  if (Platform::Constants::support_ray_tracing) {
    blas_ = std::make_shared<BLAS>(vertices, triangles);
  }
  */

  saved_ = false;
}

void Mesh::MergeVertices() {
  for (uint32_t i = 0; i < vertices_.size() - 1; i++) {
    for (uint32_t j = i + 1; j < vertices_.size(); j++) {
      auto& vi = vertices_.at(i);
      const auto& vj = vertices_.at(j);
      if (glm::distance(vi.position, vj.position) > glm::epsilon<float>()) {
        continue;
      }
      vi.tex_coord = (vi.tex_coord + vj.tex_coord) * 0.5f;
      vi.color = (vi.color + vj.color) * 0.5f;
      vertices_.at(j) = vertices_.back();
      for (auto& triangle : triangles_) {
        if (triangle.x == j)
          triangle.x = i;
        else if (triangle.x == vertices_.size() - 1)
          triangle.x = j;
        if (triangle.y == j)
          triangle.y = i;
        else if (triangle.y == vertices_.size() - 1)
          triangle.y = j;
        if (triangle.z == j)
          triangle.z = i;
        else if (triangle.z == vertices_.size() - 1)
          triangle.z = j;
      }
      vertices_.pop_back();
      j--;
    }
  }
}

size_t Mesh::GetVerticesAmount() const {
  return vertices_.size();
}

size_t Mesh::GetTriangleAmount() const {
  return triangles_.size();
}

void Mesh::RecalculateNormal() {
  auto normal_lists = std::vector<std::vector<glm::vec3>>();
  const auto size = vertices_.size();
  for (auto i = 0; i < size; i++) {
    normal_lists.emplace_back();
  }
  for (const auto& triangle : triangles_) {
    const auto& i1 = triangle.x;
    const auto& i2 = triangle.y;
    const auto& i3 = triangle.z;
    const auto& v1 = vertices_[i1].position;
    const auto& v2 = vertices_[i2].position;
    const auto& v3 = vertices_[i3].position;
    auto normal = glm::normalize(glm::cross(v1 - v2, v1 - v3));
    normal_lists[i1].push_back(normal);
    normal_lists[i2].push_back(normal);
    normal_lists[i3].push_back(normal);
  }
  for (auto i = 0; i < size; i++) {
    auto normal = glm::vec3(0.0f);
    for (const auto& j : normal_lists[i]) {
      normal += j;
    }
    vertices_[i].normal = glm::normalize(normal);
  }
}

void Mesh::RecalculateTangent() {
  auto tangent_lists = std::vector<std::vector<glm::vec3>>();
  const auto size = vertices_.size();
  for (auto i = 0; i < size; i++) {
    tangent_lists.emplace_back();
  }
  for (const auto& triangle : triangles_) {
    const auto i1 = triangle.x;
    const auto i2 = triangle.y;
    const auto i3 = triangle.z;
    const auto& p1 = vertices_[i1].position;
    const auto& p2 = vertices_[i2].position;
    const auto& p3 = vertices_[i3].position;
    const auto& uv1 = vertices_[i1].tex_coord;
    const auto& uv2 = vertices_[i2].tex_coord;
    const auto& uv3 = vertices_[i3].tex_coord;

    const auto e21 = p2 - p1;
    const auto d21 = uv2 - uv1;
    const auto e31 = p3 - p1;
    const auto d31 = uv3 - uv1;
    const float f = 1.0f / (d21.x * d31.y - d31.x * d21.y);
    auto tangent =
        f * glm::vec3(d31.y * e21.x - d21.y * e31.x, d31.y * e21.y - d21.y * e31.y, d31.y * e21.z - d21.y * e31.z);
    tangent_lists[i1].push_back(tangent);
    tangent_lists[i2].push_back(tangent);
    tangent_lists[i3].push_back(tangent);
  }
  for (auto i = 0; i < size; i++) {
    auto tangent = glm::vec3(0.0f);
    for (const auto& j : tangent_lists[i]) {
      tangent += j;
    }
    vertices_[i].tangent = glm::normalize(tangent);
  }
}

float Mesh::CalculateTriangleArea(const glm::uvec3& triangle) const {
  const auto& p0 = vertices_[triangle.x].position;
  const auto& p1 = vertices_[triangle.y].position;
  const auto& p2 = vertices_[triangle.z].position;
  const float a = glm::length(p0 - p1);
  const float b = glm::length(p2 - p1);
  const float c = glm::length(p0 - p2);
  const float d = (a + b + c) / 2;
  return glm::sqrt(d * (d - a) * (d - b) * (d - c));
}

glm::vec3 Mesh::CalculateCentroid(const glm::uvec3& triangle) const {
  const auto& a = vertices_[triangle.x].position;
  const auto& b = vertices_[triangle.y].position;
  const auto& c = vertices_[triangle.z].position;

  return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3};
}

std::vector<Vertex>& Mesh::UnsafeGetVertices() {
  return vertices_;
}

std::vector<glm::uvec3>& Mesh::UnsafeGetTriangles() {
  return triangles_;
}

Bound Mesh::GetBound() const {
  return bound_;
}

void Mesh::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "vertex_attributes_" << YAML::BeginMap;
  vertex_attributes_.Serialize(out);
  out << YAML::EndMap;

  if (!vertices_.empty() && !triangles_.empty()) {
    out << YAML::Key << "vertices_" << YAML::Value
        << YAML::Binary((const unsigned char*)vertices_.data(), vertices_.size() * sizeof(Vertex));
    out << YAML::Key << "triangles_" << YAML::Value
        << YAML::Binary((const unsigned char*)triangles_.data(), triangles_.size() * sizeof(glm::uvec3));
  }
}

void Mesh::Deserialize(const YAML::Node& in) {
  if (in["vertex_attributes_"]) {
    vertex_attributes_.Deserialize(in["vertex_attributes_"]);
  } else {
    vertex_attributes_ = {};
    vertex_attributes_.normal = true;
    vertex_attributes_.tangent = true;
    vertex_attributes_.tex_coord = true;
    vertex_attributes_.color = true;
  }

  if (in["vertices_"] && in["triangles_"]) {
    const auto& vertex_data = in["vertices_"].as<YAML::Binary>();
    std::vector<Vertex> vertices;
    vertices.resize(vertex_data.size() / sizeof(Vertex));
    std::memcpy(vertices.data(), vertex_data.data(), vertex_data.size());

    const auto& triangle_data = in["triangles_"].as<YAML::Binary>();
    std::vector<glm::uvec3> triangles;
    triangles.resize(triangle_data.size() / sizeof(glm::uvec3));
    std::memcpy(triangles.data(), triangle_data.data(), triangle_data.size());

    SetVertices(vertex_attributes_, vertices, triangles);
  }
}
void VertexAttributes::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "normal" << YAML::Value << normal;
  out << YAML::Key << "tangent" << YAML::Value << tangent;
  out << YAML::Key << "tex_coord" << YAML::Value << tex_coord;
  out << YAML::Key << "color" << YAML::Value << color;
}

void VertexAttributes::Deserialize(const YAML::Node& in) {
  if (in["normal"])
    normal = in["normal"].as<bool>();
  if (in["tangent"])
    tangent = in["tangent"].as<bool>();
  if (in["tex_coord"])
    tex_coord = in["tex_coord"].as<bool>();
  if (in["color"])
    color = in["color"].as<bool>();
}

void ParticleInfoList::OnCreate() {
  range_descriptor_ = std::make_shared<RangeDescriptor>();
  GeometryStorage::AllocateParticleInfo(GetHandle(), range_descriptor_);
}

ParticleInfoList::~ParticleInfoList() {
  GeometryStorage::FreeParticleInfo(range_descriptor_);
}

void ParticleInfoList::Serialize(YAML::Emitter& out) const {
  Serialization::SerializeVector("particle_infos", GeometryStorage::PeekParticleInfoList(range_descriptor_), out);
}

void ParticleInfoList::Deserialize(const YAML::Node& in) {
  if (in["particle_infos"]) {
    std::vector<ParticleInfo> particle_infos;
    Serialization::DeserializeVector("particle_infos", particle_infos, in);
    GeometryStorage::UpdateParticleInfo(range_descriptor_, particle_infos);
  }
}

void ParticleInfoList::ApplyRays(const std::vector<Ray>& rays, const glm::vec4& color, const float ray_width) const {
  std::vector<ParticleInfo> particle_infos;
  particle_infos.resize(rays.size());
  Jobs::RunParallelFor(rays.size(), [&](unsigned i) {
    auto& ray = rays[i];
    const auto rotation = glm::quatLookAt(ray.direction, {ray.direction.y, ray.direction.z, ray.direction.x});
    const auto rotation_mat = glm::mat4_cast(rotation);
    const auto model = glm::translate((ray.start + ray.direction * ray.length / 2.0f)) * rotation_mat *
                       glm::scale(glm::vec3(ray_width, ray_width, ray.length));
    particle_infos[i].instance_matrix.value = model;
    particle_infos[i].instance_color = color;
  });
  GeometryStorage::UpdateParticleInfo(range_descriptor_, particle_infos);
}

void ParticleInfoList::ApplyRays(const std::vector<Ray>& rays, const std::vector<glm::vec4>& colors,
                                 const float ray_width) const {
  std::vector<ParticleInfo> particle_infos;
  particle_infos.resize(rays.size());
  Jobs::RunParallelFor(rays.size(), [&](unsigned i) {
    auto& ray = rays[i];
    const auto rotation = glm::quatLookAt(ray.direction, {ray.direction.y, ray.direction.z, ray.direction.x});
    const auto rotation_mat = glm::mat4_cast(rotation);
    const auto model = glm::translate(ray.start + ray.direction * ray.length / 2.0f) * rotation_mat *
                       glm::scale(glm::vec3(ray_width, ray_width, ray.length));
    particle_infos[i].instance_matrix.value = model;
    particle_infos[i].instance_color = colors[i];
  });
  GeometryStorage::UpdateParticleInfo(range_descriptor_, particle_infos);
}

void ParticleInfoList::ApplyConnections(const std::vector<glm::vec3>& starts, const std::vector<glm::vec3>& ends,
                                        const glm::vec4& color, const float ray_width) const {
  std::vector<ParticleInfo> particle_infos;
  particle_infos.resize(starts.size());
  Jobs::RunParallelFor(starts.size(), [&](unsigned i) {
    const auto& start = starts[i];
    const auto& end = ends[i];
    const auto direction = glm::normalize(end - start);
    const auto rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
    const glm::mat4 rotation_mat = glm::mat4_cast(rotation);
    const auto model = glm::translate((start + end) / 2.0f) * rotation_mat *
                       glm::scale(glm::vec3(ray_width, ray_width, glm::distance(end, start)));
    particle_infos[i].instance_matrix.value = model;
    particle_infos[i].instance_color = color;
  });
  GeometryStorage::UpdateParticleInfo(range_descriptor_, particle_infos);
}

void ParticleInfoList::ApplyConnections(const std::vector<glm::vec3>& starts, const std::vector<glm::vec3>& ends,
                                        const std::vector<glm::vec4>& colors, const float ray_width) const {
  std::vector<ParticleInfo> particle_infos;
  particle_infos.resize(starts.size());
  Jobs::RunParallelFor(starts.size(), [&](unsigned i) {
    const auto& start = starts[i];
    const auto& end = ends[i];
    const auto direction = glm::normalize(end - start);
    const auto rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
    const auto rotation_mat = glm::mat4_cast(rotation);
    const auto model = glm::translate((start + end) / 2.0f) * rotation_mat *
                       glm::scale(glm::vec3(ray_width, ray_width, glm::distance(end, start)));
    particle_infos[i].instance_matrix.value = model;
    particle_infos[i].instance_color = colors[i];
  });
  GeometryStorage::UpdateParticleInfo(range_descriptor_, particle_infos);
}

void ParticleInfoList::ApplyConnections(const std::vector<glm::vec3>& starts, const std::vector<glm::vec3>& ends,
                                        const std::vector<glm::vec4>& colors,
                                        const std::vector<float>& ray_widths) const {
  std::vector<ParticleInfo> particle_infos;
  particle_infos.resize(starts.size());
  Jobs::RunParallelFor(starts.size(), [&](unsigned i) {
    const auto& start = starts[i];
    const auto& end = ends[i];
    const auto& width = ray_widths[i];
    const auto direction = glm::normalize(end - start);
    const auto rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
    const auto rotation_mat = glm::mat4_cast(rotation);
    const auto model = glm::translate((start + end) / 2.0f) * rotation_mat *
                       glm::scale(glm::vec3(width, width, glm::distance(end, start)));
    particle_infos[i].instance_matrix.value = model;
    particle_infos[i].instance_color = colors[i];
  });
  GeometryStorage::UpdateParticleInfo(range_descriptor_, particle_infos);
}

void ParticleInfoList::SetParticleInfos(const std::vector<ParticleInfo>& particle_infos) const {
  GeometryStorage::UpdateParticleInfo(range_descriptor_, particle_infos);
}

const std::vector<ParticleInfo>& ParticleInfoList::PeekParticleInfoList() const {
  return GeometryStorage::PeekParticleInfoList(range_descriptor_);
}

const std::shared_ptr<DescriptorSet>& ParticleInfoList::GetDescriptorSet() const {
  return GeometryStorage::PeekDescriptorSet(range_descriptor_);
}
