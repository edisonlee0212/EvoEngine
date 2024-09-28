#include "SkinnedMesh.hpp"
#include "Mesh.hpp"

#include "Application.hpp"
#include "GeometryStorage.hpp"
using namespace evo_engine;
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
    descriptor_set_.emplace_back(
        std::make_shared<DescriptorSet>(Platform::GetDescriptorSetLayout("BONE_MATRICES_LAYOUT")));
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

bool SkinnedMesh::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Text(("Vertices size: " + std::to_string(skinned_vertices_.size())).c_str());
  ImGui::Text(("Triangle amount: " + std::to_string(skinned_triangles_.size())).c_str());

  if (!skinned_vertices_.empty()) {
    FileUtils::SaveFile(
        "Export as OBJ", "Mesh", {".obj"},
        [&](const std::filesystem::path& path) {
          Export(path);
        },
        false);
  }
  return changed;
}
bool SkinnedMesh::SaveInternal(const std::filesystem::path& path) const {
  if (path.extension() == ".eveskinnedmesh") {
    return IAsset::SaveInternal(path);
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

SkinnedMesh::~SkinnedMesh() {
  GeometryStorage::FreeSkinnedMesh(GetHandle());
  skinned_triangle_range_.reset();
  skinned_meshlet_range_.reset();
}

void SkinnedMesh::DrawIndexed(const VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                              const int instances_count) const {
  if (instances_count == 0)
    return;
  global_pipeline_state.ApplyAllStates(vk_command_buffer);
  vkCmdDrawIndexed(vk_command_buffer, skinned_triangle_range_->prev_frame_index_count * 3, instances_count,
                   skinned_triangle_range_->prev_frame_offset * 3, 0, 0);
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

  if (version_ != 0)
    GeometryStorage::FreeSkinnedMesh(GetHandle());
  GeometryStorage::AllocateSkinnedMesh(GetHandle(), skinned_vertices_, skinned_triangles_, skinned_meshlet_range_,
                                       skinned_triangle_range_);

  version_++;
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
  auto size = skinned_vertices_.size();
  for (auto i = 0; i < size; i++) {
    tangent_lists.emplace_back();
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
    float f = 1.0f / (d21.x * d31.y - d31.x * d21.y);
    auto tangent =
        f * glm::vec3(d31.y * e21.x - d21.y * e31.x, d31.y * e21.y - d21.y * e31.y, d31.y * e21.z - d21.y * e31.z);
    tangent_lists[i1].push_back(tangent);
    tangent_lists[i2].push_back(tangent);
    tangent_lists[i3].push_back(tangent);
  }
  for (auto i = 0; i < size; i++) {
    auto tangent = glm::vec3(0.0f);
    for (auto j : tangent_lists[i]) {
      tangent += j;
    }
    skinned_vertices_[i].tangent = glm::normalize(tangent);
  }
}

std::vector<glm::uvec3>& SkinnedMesh::UnsafeGetTriangles() {
  return skinned_triangles_;
}
std::vector<SkinnedVertex>& SkinnedMesh::UnsafeGetSkinnedVertices() {
  return skinned_vertices_;
}

void SkinnedMesh::Serialize(YAML::Emitter& out) const {
  if (!bone_animator_indices.empty()) {
    out << YAML::Key << "bone_animator_indices" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(bone_animator_indices.data()),
                        bone_animator_indices.size() * sizeof(unsigned));
  }

  out << YAML::Key << "skinned_vertex_attributes_" << YAML::BeginMap;
  skinned_vertex_attributes_.Serialize(out);
  out << YAML::EndMap;

  if (!skinned_vertices_.empty() && !skinned_triangles_.empty()) {
    out << YAML::Key << "skinned_vertices_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(skinned_vertices_.data()),
                        skinned_vertices_.size() * sizeof(SkinnedVertex));
    out << YAML::Key << "skinned_triangles_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(skinned_triangles_.data()),
                        skinned_triangles_.size() * sizeof(glm::uvec3));
  }
}
void SkinnedMesh::Deserialize(const YAML::Node& in) {
  if (in["bone_animator_indices"]) {
    const YAML::Binary& bone_indices = in["bone_animator_indices"].as<YAML::Binary>();
    bone_animator_indices.resize(bone_indices.size() / sizeof(unsigned));
    std::memcpy(bone_animator_indices.data(), bone_indices.data(), bone_indices.size());
  }

  if (in["skinned_vertex_attributes_"]) {
    skinned_vertex_attributes_.Deserialize(in["skinned_vertex_attributes_"]);
  } else {
    skinned_vertex_attributes_ = {};
    skinned_vertex_attributes_.normal = true;
    skinned_vertex_attributes_.tangent = true;
    skinned_vertex_attributes_.tex_coord = true;
    skinned_vertex_attributes_.color = true;
  }

  if (in["skinned_vertices_"] && in["skinned_triangles_"]) {
    const YAML::Binary& skinned_vertex_data = in["skinned_vertices_"].as<YAML::Binary>();
    std::vector<SkinnedVertex> skinned_vertices;
    skinned_vertices.resize(skinned_vertex_data.size() / sizeof(SkinnedVertex));
    std::memcpy(skinned_vertices.data(), skinned_vertex_data.data(), skinned_vertex_data.size());

    const YAML::Binary& triangle_data = in["skinned_triangles_"].as<YAML::Binary>();
    std::vector<glm::uvec3> triangles;
    triangles.resize(triangle_data.size() / sizeof(glm::uvec3));
    std::memcpy(triangles.data(), triangle_data.data(), triangle_data.size());

    SetVertices(skinned_vertex_attributes_, skinned_vertices, triangles);
  }
}