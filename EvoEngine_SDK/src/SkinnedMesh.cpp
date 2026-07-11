#include "SkinnedMesh.hpp"
#include "Mesh.hpp"

#include "Application.hpp"
#include "GeometryStorage.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Serialization.hpp"
using namespace evo_engine;

namespace {
bool AddBoneTransform(glm::mat4& bone_transform, const std::vector<glm::mat4>& bone_matrices, const int bone_id,
                      const float weight) {
  if (weight == 0.0f || bone_id < 0 || static_cast<size_t>(bone_id) >= bone_matrices.size()) {
    return false;
  }
  bone_transform += bone_matrices[bone_id] * weight;
  return true;
}

glm::vec3 NormalizeOrFallback(const glm::vec3& value, const glm::vec3& fallback) {
  const auto length_squared = glm::dot(value, value);
  if (length_squared <= 0.0f) {
    return fallback;
  }
  return value * glm::inversesqrt(length_squared);
}

float TangentHandedness(const glm::vec3& tangent, const glm::vec3& bitangent, const glm::vec3& normal) {
  return glm::dot(glm::cross(tangent, bitangent), normal) < 0.0f ? -1.0f : 1.0f;
}
}  // namespace

void SkinnedVertexAttributes::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "normal" << YAML::Value << normal;
  out << YAML::Key << "tangent" << YAML::Value << tangent;
  out << YAML::Key << "tex_coord" << YAML::Value << tex_coord;
  out << YAML::Key << "color" << YAML::Value << color;
}

void SkinnedVertexAttributes::Deserialize(const YAML::Node& in) {
  if (in["normal"])
    normal = in["normal"].as<bool>();
  if (in["tangent"])
    tangent = in["tangent"].as<bool>();
  if (in["tex_coord"])
    tex_coord = in["tex_coord"].as<bool>();
  if (in["color"])
    color = in["color"].as<bool>();
}

const std::shared_ptr<DescriptorSet>& BoneMatrices::GetDescriptorSet() const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  return descriptor_set_[current_frame_index];
}

VkDescriptorBufferInfo BoneMatrices::GetPreviousBufferInfo() const {
  VkDescriptorBufferInfo buffer_info{};
  buffer_info.buffer = previous_bone_matrices_buffer_[Platform::GetCurrentFrameIndex()]->GetVkBuffer();
  buffer_info.offset = 0;
  buffer_info.range = VK_WHOLE_SIZE;
  return buffer_info;
}

BoneMatrices::BoneMatrices() {
  VkBufferCreateInfo bone_matrices_crate_info{};
  bone_matrices_crate_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  bone_matrices_crate_info.size = 256 * sizeof(glm::mat4);
  bone_matrices_crate_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  bone_matrices_crate_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frames_in_flight = Platform::GetMaxFramesInFlight();
  for (int i = 0; i < max_frames_in_flight; i++) {
    bone_matrices_buffer_.emplace_back(std::make_unique<Buffer>(bone_matrices_crate_info, allocation_create_info));
    previous_bone_matrices_buffer_.emplace_back(
        std::make_unique<Buffer>(bone_matrices_crate_info, allocation_create_info));
    descriptor_set_.emplace_back(std::make_shared<DescriptorSet>(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetBoneMatricesDescriptorSetLayout()));
  }
}

uint32_t BoneMatrices::GetVersion() const {
  return version_;
}

void BoneMatrices::UploadData() {
  version_++;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  if (!value.empty())
    bone_matrices_buffer_[current_frame_index]->UploadVector(value);
  VkDescriptorBufferInfo buffer_info;
  buffer_info.offset = 0;
  buffer_info.buffer = bone_matrices_buffer_[current_frame_index]->GetVkBuffer();
  buffer_info.range = VK_WHOLE_SIZE;
  descriptor_set_[current_frame_index]->UpdateBufferDescriptorBinding(0, buffer_info);
}

void BoneMatrices::UploadPreviousData(const std::vector<glm::mat4>& matrices) {
  if (!matrices.empty()) {
    previous_bone_matrices_buffer_[Platform::GetCurrentFrameIndex()]->UploadVector(matrices);
  }
}

Vertex evo_engine::BuildSkinnedRayTracingVertex(const SkinnedVertex& skinned_vertex,
                                                const std::vector<glm::mat4>& bone_matrices) {
  Vertex vertex{};
  vertex.vertex_info1 = skinned_vertex.vertex_info1;
  vertex.vertex_info2 = skinned_vertex.vertex_info2;
  vertex.vertex_info3 = skinned_vertex.vertex_info3;
  vertex.color = skinned_vertex.color;
  vertex.tex_coord = skinned_vertex.tex_coord;
  vertex.vertex_info4 = skinned_vertex.vertex_info4;

  glm::mat4 bone_transform(0.0f);
  bool has_valid_weight = false;
  for (int i = 0; i < 4; i++) {
    has_valid_weight |=
        AddBoneTransform(bone_transform, bone_matrices, skinned_vertex.bond_id[i], skinned_vertex.weight[i]);
    has_valid_weight |=
        AddBoneTransform(bone_transform, bone_matrices, skinned_vertex.bond_id2[i], skinned_vertex.weight2[i]);
  }
  if (!has_valid_weight) {
    bone_transform = glm::mat4(1.0f);
  }

  vertex.position = glm::vec3(bone_transform * glm::vec4(skinned_vertex.position, 1.0f));
  vertex.normal =
      NormalizeOrFallback(glm::vec3(bone_transform * glm::vec4(skinned_vertex.normal, 0.0f)), skinned_vertex.normal);
  vertex.tangent =
      NormalizeOrFallback(glm::vec3(bone_transform * glm::vec4(skinned_vertex.tangent, 0.0f)), skinned_vertex.tangent);
  vertex.tangent =
      NormalizeOrFallback(vertex.tangent - glm::dot(vertex.tangent, vertex.normal) * vertex.normal, vertex.tangent);
  return vertex;
}

std::vector<Vertex> evo_engine::BuildSkinnedRayTracingVertices(const std::vector<SkinnedVertex>& skinned_vertices,
                                                               const std::vector<glm::mat4>& bone_matrices) {
  std::vector<Vertex> vertices;
  vertices.reserve(skinned_vertices.size());
  for (const auto& skinned_vertex : skinned_vertices) {
    vertices.emplace_back(BuildSkinnedRayTracingVertex(skinned_vertex, bone_matrices));
  }
  return vertices;
}

std::vector<Vertex> evo_engine::BuildSkinnedRayTracingVertices(const std::vector<SkinnedVertex>& skinned_vertices,
                                                               const std::vector<glm::mat4>& bone_matrices,
                                                               const std::vector<uint32_t>& source_vertex_indices) {
  std::vector<Vertex> vertices;
  vertices.reserve(source_vertex_indices.size());
  for (const auto source_vertex_index : source_vertex_indices) {
    vertices.emplace_back(BuildSkinnedRayTracingVertex(skinned_vertices.at(source_vertex_index), bone_matrices));
  }
  return vertices;
}

bool SkinnedMesh::SaveInternal(const std::filesystem::path& path) const {
  if (path.extension() == ".eveskinnedmesh") {
    return Serialization::SaveAssetAsYaml(*this, path);
  } else if (path.extension() == ".obj") {
    std::ofstream of;
    of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
    if (of.is_open()) {
      std::string start = "#Mesh exporter, by Bosheng Li";
      start += "\n";
      of.write(start.c_str(), start.size());
      of.flush();
      if (!skinned_triangles_.empty()) {
        unsigned start_index = 1;
        std::string header = "#Vertices: " + std::to_string(skinned_vertices_.size()) +
                             ", tris: " + std::to_string(skinned_triangles_.size());
        header += "\n";
        of.write(header.c_str(), header.size());
        of.flush();
        std::string data;
#pragma region Data collection
        for (const auto& skinned_vertex : skinned_vertices_) {
          auto& vertex_position = skinned_vertex.position;
          auto& color = skinned_vertex.color;
          data += "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                  std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " + std::to_string(color.y) +
                  " " + std::to_string(color.z) + "\n";
        }
        for (const auto& vertex : skinned_vertices_) {
          data += "vn " + std::to_string(vertex.normal.x) + " " + std::to_string(vertex.normal.y) + " " +
                  std::to_string(vertex.normal.z) + "\n";
        }

        for (const auto& vertex : skinned_vertices_) {
          data += "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
        }
        // data += "s off\n";
        data += "# List of indices for faces vertices, with (x, y, z).\n";
        auto& triangles = skinned_triangles_;
        for (auto i = 0; i < skinned_triangles_.size(); i++) {
          const auto triangle = triangles[i];
          const auto f1 = triangle.x + start_index;
          const auto f2 = triangle.y + start_index;
          const auto f3 = triangle.z + start_index;
          data += "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                  std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " + std::to_string(f3) +
                  "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
        }
        start_index += skinned_vertices_.size();
#pragma endregion
        of.write(data.c_str(), data.size());
        of.flush();
      }
      of.close();
      return true;
    } else {
      EVOENGINE_ERROR("Can't open file!");
      return false;
    }
  }
  return false;
}

bool SkinnedMesh::RegisterAssetIoHandlers(const std::string& owner_name, const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<SkinnedMesh>(
      [](const SkinnedMesh& asset, const std::filesystem::path& path) {
        return asset.SaveInternal(path);
      },
      {},
      [](const SkinnedMesh& asset, const std::filesystem::path&) {
        return asset.SupportsStagedLoading();
      },
      {}, {}, owner_name, type_name);
}

SkinnedMesh::~SkinnedMesh() {
  GeometryStorage::FreeSkinnedMesh(GetHandle());
  GeometryStorage::FreeMesh(GetHandle());
  skinned_triangle_range_.reset();
  skinned_meshlet_range_.reset();
  ray_tracing_triangle_range_.reset();
  ray_tracing_meshlet_range_.reset();
}

void SkinnedMesh::DrawIndexed(const VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                              const int instances_count) const {
  if (instances_count == 0)
    return;
  global_pipeline_state.ApplyAllStates(vk_command_buffer);
  Platform::DrawIndexed(vk_command_buffer, skinned_triangle_range_->prev_frame_index_count * 3, instances_count,
                        skinned_triangle_range_->prev_frame_offset * 3);
}

glm::vec3 SkinnedMesh::GetCenter() const {
  return bound_.Center();
}
Bound SkinnedMesh::GetBound() const {
  return bound_;
}

void SkinnedMesh::FetchIndices(const std::vector<std::shared_ptr<Bone>>& bones) {
  bone_animator_indices.resize(bones.size());
  for (int i = 0; i < bones.size(); i++) {
    bone_animator_indices[i] = bones[i]->index;
  }
}

void SkinnedMesh::OnCreate() {
  version_ = 0;
  bound_ = Bound();
  skinned_meshlet_range_ = std::make_shared<RangeDescriptor>();
  skinned_triangle_range_ = std::make_shared<RangeDescriptor>();
  ray_tracing_meshlet_range_ = std::make_shared<RangeDescriptor>();
  ray_tracing_triangle_range_ = std::make_shared<RangeDescriptor>();
}

void SkinnedMesh::SetVertices(const SkinnedVertexAttributes& skinned_vertex_attributes,
                              const std::vector<SkinnedVertex>& skinned_vertices,
                              const std::vector<unsigned>& indices) {
  if (indices.size() % 3 != 0) {
    EVOENGINE_ERROR("Triangle size wrong!");
    return;
  }
  std::vector<glm::uvec3> triangles;
  triangles.resize(indices.size() / 3);
  memcpy(triangles.data(), indices.data(), indices.size() * sizeof(unsigned));
  SetVertices(skinned_vertex_attributes, skinned_vertices, triangles);
}

void SkinnedMesh::SetVertices(const SkinnedVertexAttributes& skinned_vertex_attributes,
                              const std::vector<SkinnedVertex>& skinned_vertices,
                              const std::vector<glm::uvec3>& triangles) {
  if (skinned_vertices.empty() || triangles.empty()) {
    EVOENGINE_LOG("Skinned vertices or triangles empty!");
    return;
  }

  skinned_vertices_ = skinned_vertices;
  skinned_triangles_ = triangles;
#pragma region Bound
  glm::vec3 min_bound = skinned_vertices_.at(0).position;
  glm::vec3 max_bound = skinned_vertices_.at(0).position;
  for (const auto& skinned_vertex : skinned_vertices_) {
    min_bound = glm::vec3((glm::min)(min_bound.x, skinned_vertex.position.x),
                          (glm::min)(min_bound.y, skinned_vertex.position.y),
                          (glm::min)(min_bound.z, skinned_vertex.position.z));
    max_bound = glm::vec3((glm::max)(max_bound.x, skinned_vertex.position.x),
                          (glm::max)(max_bound.y, skinned_vertex.position.y),
                          (glm::max)(max_bound.z, skinned_vertex.position.z));
  }
  bound_.max = max_bound;
  bound_.min = min_bound;
#pragma endregion
  if (!skinned_vertex_attributes.normal)
    RecalculateNormal();
  if (!skinned_vertex_attributes.tangent)
    RecalculateTangent();

  skinned_vertex_attributes_ = skinned_vertex_attributes;
  skinned_vertex_attributes_.normal = true;
  skinned_vertex_attributes_.tangent = true;

  if (version_ != 0) {
    GeometryStorage::FreeSkinnedMesh(GetHandle());
    GeometryStorage::FreeMesh(GetHandle());
  }
  GeometryStorage::AllocateSkinnedMesh(GetHandle(), skinned_vertices_, skinned_triangles_, skinned_meshlet_range_,
                                       skinned_triangle_range_);

  version_++;
  if (Platform::RayAccelerationStructureEnabled()) {
    auto vertices = BuildSkinnedRayTracingVertices(skinned_vertices_, {});
    auto triangles = skinned_triangles_;
    GeometryStorage::AllocateMesh(GetHandle(), vertices, triangles, ray_tracing_meshlet_range_,
                                  ray_tracing_triangle_range_);
    blas_ = std::make_shared<BottomLevelAccelerationStructure>(vertices, triangles);
  }
  saved_ = false;
}

size_t SkinnedMesh::GetSkinnedVerticesAmount() const {
  return skinned_vertices_.size();
}

size_t SkinnedMesh::GetTriangleAmount() const {
  return skinned_triangles_.size();
}

void SkinnedMesh::RecalculateNormal() {
  auto normal_lists = std::vector<std::vector<glm::vec3>>();
  const auto size = skinned_vertices_.size();
  for (auto i = 0; i < size; i++) {
    normal_lists.emplace_back();
  }
  for (const auto& triangle : skinned_triangles_) {
    const auto i1 = triangle.x;
    const auto i2 = triangle.y;
    const auto i3 = triangle.z;
    auto v1 = skinned_vertices_[i1].position;
    auto v2 = skinned_vertices_[i2].position;
    auto v3 = skinned_vertices_[i3].position;
    auto normal = glm::normalize(glm::cross(v1 - v2, v1 - v3));
    normal_lists[i1].push_back(normal);
    normal_lists[i2].push_back(normal);
    normal_lists[i3].push_back(normal);
  }
  for (auto i = 0; i < size; i++) {
    auto normal = glm::vec3(0.0f);
    for (auto j : normal_lists[i]) {
      normal += j;
    }
    skinned_vertices_[i].normal = glm::normalize(normal);
  }
}

void SkinnedMesh::RecalculateTangent() {
  auto tangent_lists = std::vector<std::vector<glm::vec3>>();
  auto handedness_sums = std::vector<float>();
  auto size = skinned_vertices_.size();
  for (auto i = 0; i < size; i++) {
    tangent_lists.emplace_back();
    handedness_sums.emplace_back(0.0f);
  }
  for (auto& triangle : skinned_triangles_) {
    const auto i1 = triangle.x;
    const auto i2 = triangle.y;
    const auto i3 = triangle.z;
    auto p1 = skinned_vertices_[i1].position;
    auto p2 = skinned_vertices_[i2].position;
    auto p3 = skinned_vertices_[i3].position;
    auto uv1 = skinned_vertices_[i1].tex_coord;
    auto uv2 = skinned_vertices_[i2].tex_coord;
    auto uv3 = skinned_vertices_[i3].tex_coord;

    auto e21 = p2 - p1;
    auto d21 = uv2 - uv1;
    auto e31 = p3 - p1;
    auto d31 = uv3 - uv1;
    const float determinant = d21.x * d31.y - d31.x * d21.y;
    if (glm::abs(determinant) <= 1e-8f) {
      continue;
    }
    const float f = 1.0f / determinant;
    const auto tangent =
        f * glm::vec3(d31.y * e21.x - d21.y * e31.x, d31.y * e21.y - d21.y * e31.y, d31.y * e21.z - d21.y * e31.z);
    const auto bitangent =
        f * glm::vec3(d31.x * e21.x - d21.x * e31.x, d31.x * e21.y - d21.x * e31.y, d31.x * e21.z - d21.x * e31.z);
    tangent_lists[i1].push_back(tangent);
    tangent_lists[i2].push_back(tangent);
    tangent_lists[i3].push_back(tangent);
    handedness_sums[i1] += TangentHandedness(tangent, bitangent, skinned_vertices_[i1].normal);
    handedness_sums[i2] += TangentHandedness(tangent, bitangent, skinned_vertices_[i2].normal);
    handedness_sums[i3] += TangentHandedness(tangent, bitangent, skinned_vertices_[i3].normal);
  }
  for (auto i = 0; i < size; i++) {
    auto tangent = glm::vec3(0.0f);
    for (auto j : tangent_lists[i]) {
      tangent += j;
    }
    skinned_vertices_[i].tangent = NormalizeOrFallback(tangent, glm::vec3(1.0f, 0.0f, 0.0f));
    skinned_vertices_[i].vertex_info3 = handedness_sums[i] < 0.0f ? -1.0f : 1.0f;
  }
}

std::vector<glm::uvec3>& SkinnedMesh::UnsafeGetTriangles() {
  return skinned_triangles_;
}
std::vector<SkinnedVertex>& SkinnedMesh::UnsafeGetSkinnedVertices() {
  return skinned_vertices_;
}

const SkinnedVertexAttributes& SkinnedMesh::GetSkinnedVertexAttributes() const {
  return skinned_vertex_attributes_;
}

const std::vector<SkinnedVertex>& SkinnedMesh::PeekSkinnedVertices() const {
  return skinned_vertices_;
}

const std::vector<glm::uvec3>& SkinnedMesh::PeekTriangles() const {
  return skinned_triangles_;
}
