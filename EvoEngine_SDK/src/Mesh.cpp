#include "Mesh.hpp"

#include "ClassRegistry.hpp"
#include "Console.hpp"
#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "Jobs.hpp"
#include "MikkTangentSpace.hpp"

#include <numeric>
#include "Platform.hpp"
#include "Serialization.hpp"
#include "Utilities.hpp"
using namespace evo_engine;

bool Mesh::SaveInternal(const std::filesystem::path& path) const {
  if (path.extension() == ".evemesh") {
    return Serialization::SaveAssetAsYaml(*this, path);
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

bool Mesh::RegisterAssetIoHandlers(const std::string& owner_name, const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<Mesh>(
      [](const Mesh& asset, const std::filesystem::path& path) {
        return asset.SaveInternal(path);
      },
      {},
      [](const Mesh& asset, const std::filesystem::path&) {
        return asset.SupportsStagedLoading();
      },
      {}, {}, owner_name, type_name);
}

std::shared_ptr<Texture2D> Mesh::GenerateThumbnailTexture() {
  return EditorLayer::FindIcon("Mesh");
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
  Platform::DrawIndexed(vk_command_buffer, triangle_range_->prev_frame_index_count * 3, instances_count,
                        triangle_range_->prev_frame_offset * 3);
}

void Mesh::SetVertices(const VertexAttributes& vertex_attributes, const std::vector<Vertex>& vertices,
                       const std::vector<unsigned>& indices, const int tangent_tex_coord,
                       std::vector<uint32_t>* source_vertex_indices) {
  if (indices.size() % 3 != 0) {
    EVOENGINE_ERROR("Triangle size wrong!");
    return;
  }
  std::vector<glm::uvec3> triangles;
  triangles.resize(indices.size() / 3);
  memcpy(triangles.data(), indices.data(), indices.size() * sizeof(unsigned));
  SetVertices(vertex_attributes, vertices, triangles, tangent_tex_coord, source_vertex_indices);
}

void Mesh::SetVertices(const VertexAttributes& vertex_attributes, const std::vector<Vertex>& vertices,
                       const std::vector<glm::uvec3>& triangles, const int tangent_tex_coord,
                       std::vector<uint32_t>* source_vertex_indices) {
  if (vertices.empty() || triangles.empty()) {
#ifndef NDEBUG
    EVOENGINE_LOG("Vertices or triangles empty!");
#endif
    return;
  }
  ClearMorphTargets();
  vertices_ = vertices;
  if (source_vertex_indices) {
    source_vertex_indices->resize(vertices_.size());
    std::iota(source_vertex_indices->begin(), source_vertex_indices->end(), 0u);
  }
  triangles_.clear();
  triangles_.reserve(triangles.size());
  for (const auto& triangle : triangles) {
    const auto& i1 = triangle.x;
    const auto& i2 = triangle.y;
    const auto& i3 = triangle.z;
    if (i1 < 0 || i1 >= vertices_.size())
      continue;
    if (i2 < 0 || i2 >= vertices_.size())
      continue;
    if (i3 < 0 || i3 >= vertices_.size())
      continue;
    triangles_.emplace_back() = triangle;
  }

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
    GenerateMikkTangents(vertices_, triangles_, tangent_tex_coord, source_vertex_indices);

  vertex_attributes_ = vertex_attributes;
  vertex_attributes_.normal = true;
  vertex_attributes_.tangent = true;

  // MergeVertices();

  if (version_ != 0)
    GeometryStorage::FreeMesh(GetHandle());

  auto v_c = vertices_;
  auto t_c = triangles_;
  GeometryStorage::AllocateMesh(GetHandle(), v_c, t_c, meshlet_range_, triangle_range_);

  version_++;
  if (Platform::RayAccelerationStructureEnabled()) {
    blas_ = BottomLevelAccelerationStructure::CreateStatic(meshlet_range_, triangle_range_, v_c);
  }

  saved_ = false;
}

void Mesh::ClearMorphTargets() {
  morph_targets_.clear();
  default_morph_weights_.clear();
  morph_base_vertices_.clear();
}

void Mesh::SetMorphTargets(std::vector<MorphTarget> morph_targets, std::vector<float> default_weights,
                           std::vector<Vertex> morph_base_vertices) {
  const auto valid = [&](const MorphTarget& target) {
    const auto valid_stream = [&](const std::vector<glm::vec3>& stream) {
      return stream.empty() || stream.size() == vertices_.size();
    };
    return valid_stream(target.position_deltas) && valid_stream(target.normal_deltas) &&
           valid_stream(target.tangent_deltas);
  };
  for (size_t index = morph_targets.size(); index-- > 0;) {
    if (valid(morph_targets[index])) {
      continue;
    }
    morph_targets.erase(morph_targets.begin() + index);
    if (index < default_weights.size()) {
      default_weights.erase(default_weights.begin() + index);
    }
  }
  if (morph_targets.empty()) {
    ClearMorphTargets();
    version_++;
    saved_ = false;
    return;
  }
  default_weights.resize(morph_targets.size(), 0.0f);
  if (morph_base_vertices.size() != vertices_.size()) {
    EVOENGINE_ERROR("Morph base vertex count does not match the mesh.")
    ClearMorphTargets();
    version_++;
    saved_ = false;
    return;
  }
  const auto default_vertices = evo_engine::BuildMorphedVertices(
      ComposeMorphBaseVertices(vertices_, morph_base_vertices, morph_targets), morph_targets, {}, default_weights);
  if (!MorphVertexStreamsMatch(vertices_, default_vertices)) {
    const auto attributes = vertex_attributes_;
    const auto triangles = triangles_;
    SetVertices(attributes, default_vertices, triangles);
  }
  morph_targets_ = std::move(morph_targets);
  default_morph_weights_ = std::move(default_weights);
  morph_base_vertices_ = std::move(morph_base_vertices);
  version_++;
  saved_ = false;
}

const std::vector<MorphTarget>& Mesh::PeekMorphTargets() const {
  return morph_targets_;
}

const std::vector<float>& Mesh::GetDefaultMorphWeights() const {
  return default_morph_weights_;
}

const std::vector<Vertex>& Mesh::PeekMorphBaseVertices() const {
  return morph_base_vertices_;
}

std::vector<Vertex> Mesh::BuildMorphedVertices(const std::vector<float>& weights) const {
  if (morph_targets_.empty()) {
    return vertices_;
  }
  auto resolved_weights = default_morph_weights_;
  for (size_t index = 0; index < std::min(resolved_weights.size(), weights.size()); index++) {
    resolved_weights[index] = weights[index];
  }
  return evo_engine::BuildMorphedVertices(ComposeMorphBaseVertices(vertices_, morph_base_vertices_, morph_targets_),
                                          morph_targets_, {}, resolved_weights);
}

void Mesh::MergeVertices() {
  ClearMorphTargets();
  for (uint32_t i = 0; i < vertices_.size() - 1; i++) {
    for (uint32_t j = i + 1; j < vertices_.size(); j++) {
      auto& vi = vertices_.at(i);
      const auto& vj = vertices_.at(j);
      if (glm::distance(vi.position, vj.position) > glm::epsilon<float>()) {
        continue;
      }
      vi.tex_coord = (vi.tex_coord + vj.tex_coord) * 0.5f;
      vi.tex_coord_1 = (vi.tex_coord_1 + vj.tex_coord_1) * 0.5f;
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

uint32_t Mesh::GetVerticesAmount() const {
  return vertices_.size();
}

uint32_t Mesh::GetTriangleAmount() const {
  return triangles_.size();
}

void Mesh::RecalculateNormal() {
  ClearMorphTargets();
  auto normal_lists = std::vector<std::vector<glm::vec3>>();
  const auto size = vertices_.size();
  for (auto i = 0; i < size; i++) {
    normal_lists.emplace_back();
  }
  for (const auto& triangle : triangles_) {
    const auto& i1 = triangle.x;
    const auto& i2 = triangle.y;
    const auto& i3 = triangle.z;
    if (i1 >= vertices_.size())
      continue;
    if (i2 >= vertices_.size())
      continue;
    if (i3 >= vertices_.size())
      continue;
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

void Mesh::RecalculateTangent(const int tex_coord) {
  ClearMorphTargets();
  GenerateMikkTangents(vertices_, triangles_, tex_coord);
}

const std::shared_ptr<RangeDescriptor>& Mesh::GetTriangleRange() const {
  return triangle_range_;
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

const VertexAttributes& Mesh::GetVertexAttributes() const {
  return vertex_attributes_;
}

const std::vector<Vertex>& Mesh::PeekVertices() const {
  return vertices_;
}

const std::vector<glm::uvec3>& Mesh::PeekTriangles() const {
  return triangles_;
}

Bound Mesh::GetBound() const {
  return bound_;
}

std::shared_ptr<BottomLevelAccelerationStructure> Mesh::GetBlas() const {
  return blas_;
}

void VertexAttributes::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "normal" << YAML::Value << normal;
  out << YAML::Key << "tangent" << YAML::Value << tangent;
  out << YAML::Key << "tex_coord" << YAML::Value << tex_coord;
  out << YAML::Key << "tex_coord_1" << YAML::Value << tex_coord_1;
  out << YAML::Key << "tex_coord_2" << YAML::Value << tex_coord_2;
  out << YAML::Key << "tex_coord_3" << YAML::Value << tex_coord_3;
  out << YAML::Key << "color" << YAML::Value << color;
}

void VertexAttributes::Deserialize(const YAML::Node& in) {
  if (in["normal"])
    normal = in["normal"].as<bool>();
  if (in["tangent"])
    tangent = in["tangent"].as<bool>();
  if (in["tex_coord"])
    tex_coord = in["tex_coord"].as<bool>();
  if (in["tex_coord_1"])
    tex_coord_1 = in["tex_coord_1"].as<bool>();
  if (in["tex_coord_2"])
    tex_coord_2 = in["tex_coord_2"].as<bool>();
  if (in["tex_coord_3"])
    tex_coord_3 = in["tex_coord_3"].as<bool>();
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

void ParticleInfoList::ApplyRays(const std::vector<Ray>& rays, const glm::vec4& color, const float ray_width) const {
  std::vector<ParticleInfo> particle_infos;
  particle_infos.resize(rays.size());
  Jobs::RunParallelFor(rays.size(), [&](size_t i) {
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
  Jobs::RunParallelFor(rays.size(), [&](size_t i) {
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
  Jobs::RunParallelFor(starts.size(), [&](size_t i) {
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
  Jobs::RunParallelFor(starts.size(), [&](size_t i) {
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
  Jobs::RunParallelFor(starts.size(), [&](size_t i) {
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
