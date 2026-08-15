#include "Prefab.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "GltfMaterialCache.hpp"
#include "Lights.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "Resources.hpp"
#include "Serialization.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "TransformGraph.hpp"
#include "UnknownPrivateComponent.hpp"
#include "Utilities.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <optional>
#include <unordered_set>

#include <stb_image.h>

using namespace evo_engine;
void Prefab::OnCreate() {
  instance_name = "New Prefab";
}

void CalculateBoundingBox(const std::shared_ptr<Prefab>& walker, Bound& bound) {
  for (const auto& child : walker->child_prefabs) {
    CalculateBoundingBox(child, bound);
  }
  GlobalTransform gt{};
  for (const auto& data_component : walker->data_components) {
    if (data_component.data_component_type == Typeof<GlobalTransform>()) {
      gt.value = std::reinterpret_pointer_cast<GlobalTransform>(data_component.data_component)->value;
      break;
    }
  }
  for (const auto& private_component : walker->private_components) {
    if (const auto mmr = std::dynamic_pointer_cast<MeshRenderer>(private_component.private_component)) {
      if (const auto mesh = mmr->mesh.Get<Mesh>()) {
        auto mesh_bound = mesh->GetBound();
        mesh_bound.ApplyTransform(gt.value);
        glm::vec3 center = mesh_bound.Center();

        glm::vec3 size = mesh_bound.Size();
        bound.min = glm::vec3((glm::min)(bound.min.x, center.x - size.x), (glm::min)(bound.min.y, center.y - size.y),
                              (glm::min)(bound.min.z, center.z - size.z));
        bound.max = glm::vec3((glm::max)(bound.max.x, center.x + size.x), (glm::max)(bound.max.y, center.y + size.y),
                              (glm::max)(bound.max.z, center.z + size.z));
      }
    }
    if (const auto smmr = std::dynamic_pointer_cast<SkinnedMeshRenderer>(private_component.private_component)) {
      if (const auto skinned_mesh = smmr->skinned_mesh.Get<SkinnedMesh>()) {
        auto mesh_bound = skinned_mesh->GetBound();
        mesh_bound.ApplyTransform(gt.value);
        glm::vec3 center = mesh_bound.Center();

        glm::vec3 size = mesh_bound.Size();
        bound.min = glm::vec3((glm::min)(bound.min.x, center.x - size.x), (glm::min)(bound.min.y, center.y - size.y),
                              (glm::min)(bound.min.z, center.z - size.z));
        bound.max = glm::vec3((glm::max)(bound.max.x, center.x + size.x), (glm::max)(bound.max.y, center.y + size.y),
                              (glm::max)(bound.max.z, center.z + size.z));
      }
    }
  }
}

void SetPrefabLocalTransform(Prefab* prefab, const glm::mat4& value) {
  for (const auto& data_component : prefab->data_components) {
    if (data_component.data_component_type == Typeof<Transform>()) {
      std::reinterpret_pointer_cast<Transform>(data_component.data_component)->value = value;
      return;
    }
  }
  auto transform = std::make_shared<Transform>();
  transform->value = value;
  DataComponentHolder holder;
  holder.data_component_type = Typeof<Transform>();
  holder.data_component = transform;
  prefab->data_components.push_back(holder);
}

Bound Prefab::GetBoundingBox() const {
  Bound bound{};
  for (const auto& child : child_prefabs) {
    CalculateBoundingBox(child, bound);
  }

  GlobalTransform gt{};
  for (const auto& data_component : data_components) {
    if (data_component.data_component_type == Typeof<GlobalTransform>()) {
      gt.value = std::reinterpret_pointer_cast<GlobalTransform>(data_component.data_component)->value;
      break;
    }
  }
  for (const auto& private_component : private_components) {
    if (const auto mmr = std::dynamic_pointer_cast<MeshRenderer>(private_component.private_component)) {
      if (const auto mesh = mmr->mesh.Get<Mesh>()) {
        auto mesh_bound = mesh->GetBound();
        mesh_bound.ApplyTransform(gt.value);
        glm::vec3 center = mesh_bound.Center();

        glm::vec3 size = mesh_bound.Size();
        bound.min = glm::vec3((glm::min)(bound.min.x, center.x - size.x), (glm::min)(bound.min.y, center.y - size.y),
                              (glm::min)(bound.min.z, center.z - size.z));
        bound.max = glm::vec3((glm::max)(bound.max.x, center.x + size.x), (glm::max)(bound.max.y, center.y + size.y),
                              (glm::max)(bound.max.z, center.z + size.z));
      }
    }
    if (const auto smmr = std::dynamic_pointer_cast<SkinnedMeshRenderer>(private_component.private_component)) {
      if (const auto skinned_mesh = smmr->skinned_mesh.Get<SkinnedMesh>()) {
        auto mesh_bound = skinned_mesh->GetBound();
        mesh_bound.ApplyTransform(gt.value);
        glm::vec3 center = mesh_bound.Center();

        glm::vec3 size = mesh_bound.Size();
        bound.min = glm::vec3((glm::min)(bound.min.x, center.x - size.x), (glm::min)(bound.min.y, center.y - size.y),
                              (glm::min)(bound.min.z, center.z - size.z));
        bound.max = glm::vec3((glm::max)(bound.max.x, center.x + size.x), (glm::max)(bound.max.y, center.y + size.y),
                              (glm::max)(bound.max.z, center.z + size.z));
      }
    }
  }

  return bound;
}

#pragma region Assimp Import
struct AssimpImportNode {
  aiNode* corresponding_node = nullptr;
  std::string name;
  Transform local_transform;
  AssimpImportNode(aiNode* node);
  std::shared_ptr<AssimpImportNode> parent_node;
  std::vector<std::shared_ptr<AssimpImportNode>> child_nodes;
  std::shared_ptr<Bone> bone;
  bool has_mesh;

  bool NecessaryWalker(std::unordered_map<std::string, std::shared_ptr<Bone>>& bone_map);
  void AttachToAnimator(const std::shared_ptr<Animation>& animation, size_t& index) const;
  void AttachChild(const std::shared_ptr<Bone>& parent, size_t& index) const;
};
glm::mat4 Mat4Cast(const aiMatrix4x4& m) {
  return glm::transpose(glm::make_mat4(&m.a1));
}
aiMatrix4x4 Mat4Cast(const glm::mat4& m) {
  aiMatrix4x4 ret_val;
  ret_val.a1 = m[0][0];
  ret_val.a2 = m[1][0];
  ret_val.a3 = m[2][0];
  ret_val.a4 = m[3][0];

  ret_val.b1 = m[0][1];
  ret_val.b2 = m[1][1];
  ret_val.b3 = m[2][1];
  ret_val.b4 = m[3][1];

  ret_val.c1 = m[0][2];
  ret_val.c2 = m[1][2];
  ret_val.c3 = m[2][2];
  ret_val.c4 = m[3][2];

  ret_val.d1 = m[0][3];
  ret_val.d2 = m[1][3];
  ret_val.d3 = m[2][3];
  ret_val.d4 = m[3][3];
  return ret_val;
}
glm::mat4 Mat4Cast(const aiMatrix3x3& m) {
  return glm::transpose(glm::make_mat3(&m.a1));
}
AssimpImportNode::AssimpImportNode(aiNode* node) {
  corresponding_node = node;
  if (node->mParent)
    local_transform.value = Mat4Cast(node->mTransformation);
  name = node->mName.C_Str();
}
void AssimpImportNode::AttachToAnimator(const std::shared_ptr<Animation>& animation, size_t& index) const {
  animation->root_bone = bone;
  animation->root_bone->index = index;
  for (auto& i : child_nodes) {
    index += 1;
    i->AttachChild(bone, index);
  }
}
void AssimpImportNode::AttachChild(const std::shared_ptr<Bone>& parent, size_t& index) const {
  bone->index = index;
  parent->children.push_back(bone);
  for (auto& i : child_nodes) {
    index += 1;
    i->AttachChild(bone, index);
  }
}
bool AssimpImportNode::NecessaryWalker(std::unordered_map<std::string, std::shared_ptr<Bone>>& bone_map) {
  bool necessary = false;
  for (int i = 0; i < child_nodes.size(); i++) {
    if (!child_nodes[i]->NecessaryWalker(bone_map)) {
      child_nodes.erase(child_nodes.begin() + i);
      i--;
    } else {
      necessary = true;
    }
  }
  if (const auto search = bone_map.find(name); search != bone_map.end()) {
    bone = search->second;
    necessary = true;
  } else if (necessary) {
    bone = std::make_shared<Bone>();
    bone->name = name;
  }

  return necessary;
}
void ReadKeyFrame(BoneKeyFrames& bone_animation, const aiNodeAnim* channel) {
  const auto num_positions = channel->mNumPositionKeys;
  bone_animation.positions.resize(num_positions);
  for (int position_index = 0; position_index < num_positions; ++position_index) {
    const aiVector3D ai_position = channel->mPositionKeys[position_index].mValue;
    const float time_stamp = channel->mPositionKeys[position_index].mTime;
    BonePosition data;
    data.value = glm::vec3(ai_position.x, ai_position.y, ai_position.z);
    data.time_stamp = time_stamp;
    bone_animation.positions.push_back(data);
    bone_animation.max_time_stamp = glm::max(bone_animation.max_time_stamp, time_stamp);
  }

  const auto num_rotations = channel->mNumRotationKeys;
  bone_animation.rotations.resize(num_rotations);
  for (int rotation_index = 0; rotation_index < num_rotations; ++rotation_index) {
    const aiQuaternion ai_orientation = channel->mRotationKeys[rotation_index].mValue;
    const float time_stamp = channel->mRotationKeys[rotation_index].mTime;
    BoneRotation data;
    data.value = glm::quat(ai_orientation.w, ai_orientation.x, ai_orientation.y, ai_orientation.z);
    data.time_stamp = time_stamp;
    bone_animation.rotations.push_back(data);
    bone_animation.max_time_stamp = glm::max(bone_animation.max_time_stamp, time_stamp);
  }

  const auto num_scales = channel->mNumScalingKeys;
  bone_animation.scales.resize(num_scales);
  for (int key_index = 0; key_index < num_scales; ++key_index) {
    const aiVector3D scale = channel->mScalingKeys[key_index].mValue;
    const float time_stamp = channel->mScalingKeys[key_index].mTime;
    BoneScale data;
    data.m_value = glm::vec3(scale.x, scale.y, scale.z);
    data.time_stamp = time_stamp;
    bone_animation.scales.push_back(data);
    bone_animation.max_time_stamp = glm::max(bone_animation.max_time_stamp, time_stamp);
  }
}
void ReadAnimations(const aiScene* importer_scene, const std::shared_ptr<Animation>& animator,
                    std::unordered_map<std::string, std::shared_ptr<Bone>>& bones_map) {
  for (int i = 0; i < importer_scene->mNumAnimations; i++) {
    const aiAnimation* importer_animation = importer_scene->mAnimations[i];
    const std::string animation_name = importer_animation->mName.C_Str();
    float max_animation_time_stamp = 0.0f;
    for (int j = 0; j < importer_animation->mNumChannels; j++) {
      const aiNodeAnim* importer_node_animation = importer_animation->mChannels[j];
      const std::string node_name = importer_node_animation->mNodeName.C_Str();
      if (const auto search = bones_map.find(node_name); search != bones_map.end()) {
        const auto& bone = search->second;
        bone->animations[animation_name] = BoneKeyFrames();
        ReadKeyFrame(bone->animations[animation_name], importer_node_animation);
        max_animation_time_stamp = glm::max(max_animation_time_stamp, bone->animations[animation_name].max_time_stamp);
      }
    }
    animator->animation_length[animation_name] = max_animation_time_stamp;
  }
}
void AddTextureImportCandidate(std::vector<std::filesystem::path>& candidates, std::unordered_set<std::string>& seen,
                               const std::filesystem::path& path, const bool include_dds_fallbacks) {
  const auto absolute_path = std::filesystem::absolute(path);
  auto extension = absolute_path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](const unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });
  const auto key = absolute_path.lexically_normal().string();
  if (seen.insert(key).second) {
    candidates.emplace_back(absolute_path);
  }
  if (include_dds_fallbacks && extension == ".dds") {
    for (const auto* fallback_extension : {".png", ".tga", ".jpg", ".jpeg"}) {
      auto fallback_path = absolute_path;
      fallback_path.replace_extension(fallback_extension);
      const auto fallback_key = fallback_path.lexically_normal().string();
      if (seen.insert(fallback_key).second) {
        candidates.emplace_back(fallback_path);
      }
    }
  }
}

std::vector<std::filesystem::path> CollectTextureImportCandidates(const std::string& directory, const std::string& path,
                                                                  const bool include_dds_fallbacks = true) {
  const auto base_dir = std::filesystem::absolute(directory);
  std::string decoded_path;
  decoded_path.reserve(path.size());
  const auto hex_value = [](const char value) -> int {
    if (value >= '0' && value <= '9')
      return value - '0';
    if (value >= 'a' && value <= 'f')
      return value - 'a' + 10;
    if (value >= 'A' && value <= 'F')
      return value - 'A' + 10;
    return -1;
  };
  for (size_t index = 0; index < path.size(); ++index) {
    if (path[index] == '%' && index + 2 < path.size()) {
      const int high = hex_value(path[index + 1]);
      const int low = hex_value(path[index + 2]);
      if (high >= 0 && low >= 0) {
        decoded_path.push_back(static_cast<char>((high << 4) | low));
        index += 2;
        continue;
      }
    }
    decoded_path.push_back(path[index]);
  }
  const auto texture_path = std::filesystem::path(decoded_path);
  const auto texture_filename = texture_path.filename();
  std::vector<std::filesystem::path> candidates;
  std::unordered_set<std::string> seen;

  AddTextureImportCandidate(candidates, seen, base_dir / texture_path, include_dds_fallbacks);
  AddTextureImportCandidate(candidates, seen, texture_path, include_dds_fallbacks);
  AddTextureImportCandidate(candidates, seen, base_dir / texture_filename, include_dds_fallbacks);
  AddTextureImportCandidate(candidates, seen, base_dir.parent_path() / texture_filename, include_dds_fallbacks);
  AddTextureImportCandidate(candidates, seen, base_dir.parent_path().parent_path() / "textures" / texture_filename,
                            include_dds_fallbacks);
  AddTextureImportCandidate(candidates, seen, base_dir.parent_path().parent_path() / "texture" / texture_filename,
                            include_dds_fallbacks);
  return candidates;
}

bool TexturePathNeedsYFlip(const std::filesystem::path& path) {
  auto extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](const unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });
  return extension != ".dds";
}

std::shared_ptr<Texture2D> CollectTexture(const std::string& directory, const std::string& path,
                                          std::unordered_map<std::string, std::shared_ptr<Texture2D>>& loaded_textures,
                                          bool* source_needs_y_flip = nullptr) {
  if (source_needs_y_flip) {
    *source_needs_y_flip = false;
  }
  for (const auto& full_path : CollectTextureImportCandidates(directory, path)) {
    if (!std::filesystem::exists(full_path)) {
      continue;
    }
    const auto full_path_string = full_path.string();
    if (const auto search = loaded_textures.find(full_path_string); search != loaded_textures.end()) {
      if (source_needs_y_flip) {
        *source_needs_y_flip = TexturePathNeedsYFlip(full_path);
      }
      return search->second;
    }

    std::shared_ptr<Texture2D> texture_2d;
    if (ProjectManager::IsInAssetsFolder(full_path)) {
      texture_2d = std::dynamic_pointer_cast<Texture2D>(
          ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(full_path)));
    } else {
      texture_2d = AssetManager::CreateTemporaryAsset<Texture2D>();
      if (!texture_2d->Import(full_path)) {
        continue;
      }
    }
    if (source_needs_y_flip) {
      *source_needs_y_flip = TexturePathNeedsYFlip(full_path);
    }
    loaded_textures[full_path_string] = texture_2d;
    return texture_2d;
  }
  return Resources::GetInstance().GetMissingTexture();
}
struct ImportedGltfMaterialData {
  GltfMaterialData material_data;
  std::vector<AssetRef> texture_refs;
};

std::string LowercaseExtension(const std::filesystem::path& path);

YAML::Node evo_engine::ReadGltfRootNode(const std::filesystem::path& path) {
  const auto extension = LowercaseExtension(path);
  if (extension == ".gltf") {
    std::ifstream stream(path);
    std::stringstream json;
    json << stream.rdbuf();
    return YAML::Load(json.str());
  }
  if (extension != ".glb") {
    return {};
  }

  std::ifstream stream(path, std::ios::binary);
  std::vector<unsigned char> bytes(std::filesystem::file_size(path));
  stream.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  const auto read_u32 = [&](const size_t offset) {
    uint32_t value = 0;
    if (offset + sizeof(value) <= bytes.size()) {
      std::memcpy(&value, bytes.data() + offset, sizeof(value));
    }
    return value;
  };
  constexpr uint32_t glb_magic = 0x46546c67;
  constexpr uint32_t json_chunk = 0x4e4f534a;
  if (bytes.size() < 20 || read_u32(0) != glb_magic || read_u32(4) != 2 || read_u32(8) > bytes.size() ||
      read_u32(16) != json_chunk || 20ull + read_u32(12) > bytes.size()) {
    throw std::runtime_error("invalid glTF binary header or JSON chunk");
  }
  std::string json(reinterpret_cast<const char*>(bytes.data() + 20), read_u32(12));
  while (!json.empty() && (json.back() == '\0' || std::isspace(static_cast<unsigned char>(json.back())))) {
    json.pop_back();
  }
  return YAML::Load(json);
}

std::vector<int32_t> GltfTextureImageIndices(const YAML::Node& gltf, const int32_t texture_index) {
  std::vector<int32_t> result;
  const auto textures = gltf["textures"];
  if (!textures || !textures.IsSequence() || texture_index < 0 || texture_index >= textures.size()) {
    return result;
  }
  const auto texture = textures[texture_index];
  const auto dds = texture["extensions"] && texture["extensions"]["MSFT_texture_dds"]
                       ? texture["extensions"]["MSFT_texture_dds"]
                       : YAML::Node{};
  if (dds && dds["source"]) {
    result.push_back(dds["source"].as<int32_t>());
  }
  if (texture["source"]) {
    const int32_t core_source = texture["source"].as<int32_t>();
    if (result.empty() || result.front() != core_source) {
      result.push_back(core_source);
    }
  }
  return result;
}

std::string GltfImageUri(const YAML::Node& gltf, const int32_t image_index) {
  const auto images = gltf["images"];
  if (!images || !images.IsSequence() || image_index < 0 || image_index >= images.size() ||
      !images[image_index]["uri"]) {
    return {};
  }
  return images[image_index]["uri"].as<std::string>();
}

const aiTexture* GltfEmbeddedTexture(const YAML::Node& gltf, const aiScene& scene, const int32_t image_index) {
  const auto images = gltf["images"];
  if (!images || !images.IsSequence() || image_index < 0 || image_index >= images.size()) {
    return nullptr;
  }
  int32_t embedded_index = -1;
  for (int32_t i = 0; i <= image_index; ++i) {
    const auto image = images[i];
    const auto uri = image["uri"] ? image["uri"].as<std::string>() : std::string{};
    if (image["bufferView"] || uri.rfind("data:", 0) == 0) {
      ++embedded_index;
    }
  }
  return embedded_index >= 0 && embedded_index < static_cast<int32_t>(scene.mNumTextures)
             ? scene.mTextures[embedded_index]
             : nullptr;
}

Texture2DSamplerSettings ToTextureSamplerSettings(const GltfSamplerInfo& source) {
  Texture2DSamplerSettings result;
  result.mag_filter = source.mag_filter;
  result.min_filter = source.min_filter;
  result.mipmap_mode = source.mipmap_mode;
  result.address_mode_u = source.address_mode_u;
  result.address_mode_v = source.address_mode_v;
  result.max_lod = source.max_lod;
  return result;
}

std::shared_ptr<Texture2D> LoadEmbeddedGltfTexture(const aiTexture& source, const bool srgb,
                                                   const Texture2DSamplerSettings& sampler) {
  auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  texture->srgb = srgb;
  texture->SetSamplerSettings(sampler);
  int width = 0;
  int height = 0;
  int components = 0;
  stbi_set_flip_vertically_on_load_thread(true);
  stbi_uc* decoded = nullptr;
  if (source.mHeight == 0 && source.pcData && source.mWidth > 0) {
    decoded = stbi_load_from_memory(reinterpret_cast<const stbi_uc*>(source.pcData), static_cast<int>(source.mWidth),
                                    &width, &height, &components, STBI_rgb_alpha);
  }
  std::vector<glm::vec4> pixels;
  if (decoded) {
    pixels.resize(static_cast<size_t>(width) * height);
    for (size_t i = 0; i < pixels.size(); ++i) {
      pixels[i] = glm::vec4(decoded[i * 4], decoded[i * 4 + 1], decoded[i * 4 + 2], decoded[i * 4 + 3]) / 255.0f;
    }
    stbi_image_free(decoded);
  } else if (source.pcData && source.mWidth > 0 && source.mHeight > 0) {
    width = static_cast<int>(source.mWidth);
    height = static_cast<int>(source.mHeight);
    components = 4;
    pixels.resize(static_cast<size_t>(width) * height);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const auto& texel = source.pcData[(height - 1 - y) * width + x];
        pixels[static_cast<size_t>(y) * width + x] = glm::vec4(texel.r, texel.g, texel.b, texel.a) / 255.0f;
      }
    }
  } else {
    return {};
  }
  texture->SetRgbaChannelData(pixels, glm::uvec2(width, height));
  texture->red_channel = components >= 1;
  texture->green_channel = components >= 2;
  texture->blue_channel = components >= 3;
  texture->alpha_channel = components >= 4;
  return texture;
}

std::vector<ImportedGltfMaterialData> ReadGltfMaterialData(
    const std::filesystem::path& path, const std::string& directory,
    std::unordered_map<std::string, std::shared_ptr<Texture2D>>& loaded_textures, const aiScene& scene,
    bool* parsed_gltf = nullptr) {
  if (parsed_gltf) {
    *parsed_gltf = false;
  }
  const auto extension = LowercaseExtension(path);
  if (extension != ".gltf" && extension != ".glb") {
    return {};
  }

  try {
    const auto gltf = ReadGltfRootNode(path);
    std::unordered_map<uint64_t, int32_t> resolved_texture_indices;
    std::unordered_map<uint64_t, bool> resolved_texture_source_needs_y_flip;
    std::unordered_map<int32_t, std::shared_ptr<Texture2D>> resolved_textures_by_storage_index;
    const auto cache_key = [](const int32_t texture_index, const bool srgb) {
      return (static_cast<uint64_t>(static_cast<uint32_t>(texture_index)) << 1u) | static_cast<uint64_t>(srgb);
    };
    const auto report_error = [&](const std::string& message) {
      EVOENGINE_ERROR(message)
    };
    auto material_data = BuildGltfMaterialDataFromGltfNode(
        gltf,
        [&](const int32_t texture_index, const bool srgb) {
          if (texture_index < 0) {
            return -1;
          }
          const auto key = cache_key(texture_index, srgb);
          if (const auto search = resolved_texture_indices.find(key); search != resolved_texture_indices.end()) {
            return search->second;
          }

          const auto sampler = ToTextureSamplerSettings(ReadGltfSamplerInfo(gltf, texture_index, report_error));
          std::shared_ptr<Texture2D> texture;
          bool source_needs_y_flip = false;
          for (const int32_t image_index : GltfTextureImageIndices(gltf, texture_index)) {
            if (const auto embedded = GltfEmbeddedTexture(gltf, scene, image_index)) {
              texture = LoadEmbeddedGltfTexture(*embedded, srgb, sampler);
              source_needs_y_flip = texture != nullptr;
            } else if (const auto texture_uri = GltfImageUri(gltf, image_index); !texture_uri.empty()) {
              for (const auto& full_path : CollectTextureImportCandidates(directory, texture_uri, false)) {
                if (!std::filesystem::exists(full_path)) {
                  continue;
                }
                if (!Platform::Initialized() && LowercaseExtension(full_path) == ".dds") {
                  continue;
                }
                std::shared_ptr<Texture2D> candidate;
                bool shared_project_image = false;
                if (ProjectManager::IsInAssetsFolder(full_path)) {
                  try {
                    const auto source = std::dynamic_pointer_cast<Texture2D>(
                        ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(full_path)));
                    auto shared_view = AssetManager::CreateTemporaryAsset<Texture2D>();
                    if (source && shared_view->ShareGpuImage(*source, srgb, sampler)) {
                      candidate = std::move(shared_view);
                      shared_project_image = true;
                    }
                  } catch (const std::exception& e) {
                    EVOENGINE_WARNING("Unable to reuse project texture " + full_path.filename().string() + ": " +
                                      e.what())
                  }
                }
                if (!candidate) {
                  candidate = AssetManager::CreateTemporaryAsset<Texture2D>();
                  candidate->SetSrgbImportOverride(srgb);
                  candidate->SetSamplerSettings(sampler);
                }
                if (shared_project_image || Serialization::LoadAsset(*candidate, full_path)) {
                  texture = std::move(candidate);
                  source_needs_y_flip = TexturePathNeedsYFlip(full_path);
                  loaded_textures[full_path.string() + "#gltf-" + std::to_string(texture_index) +
                                  (srgb ? "-srgb" : "-linear")] = texture;
                  break;
                }
              }
            }
            if (texture) {
              break;
            }
          }
          if (!texture) {
            report_error("glTF texture " + std::to_string(texture_index) + " could not be decoded; disabling binding.");
          }
          const int32_t storage_index = texture ? static_cast<int32_t>(texture->GetTextureStorageIndex()) : -1;
          resolved_texture_indices[key] = storage_index;
          resolved_texture_source_needs_y_flip[key] = source_needs_y_flip;
          if (texture && storage_index >= 0) {
            resolved_textures_by_storage_index[storage_index] = texture;
          }
          return storage_index;
        },
        [&](const int32_t texture_index, const bool srgb) {
          const auto resolved = resolved_texture_source_needs_y_flip.find(cache_key(texture_index, srgb));
          return resolved != resolved_texture_source_needs_y_flip.end() && resolved->second;
        },
        [&](const int32_t texture_index, const bool srgb) {
          const auto resolved_index = resolved_texture_indices.find(cache_key(texture_index, srgb));
          if (resolved_index == resolved_texture_indices.end()) {
            return false;
          }
          const auto resolved_texture = resolved_textures_by_storage_index.find(resolved_index->second);
          return resolved_texture != resolved_textures_by_storage_index.end() && resolved_texture->second &&
                 resolved_texture->second->SamplesLinearSrgb();
        },
        report_error);
    std::vector<ImportedGltfMaterialData> result;
    result.reserve(material_data.size());
    for (auto& data : material_data) {
      ImportedGltfMaterialData imported;
      imported.material_data = std::move(data);
      imported.texture_refs.resize(imported.material_data.texture_infos.size());
      for (size_t i = 1; i < imported.material_data.texture_infos.size(); ++i) {
        const auto storage_index = imported.material_data.texture_infos[i].index;
        if (const auto search = resolved_textures_by_storage_index.find(storage_index);
            search != resolved_textures_by_storage_index.end()) {
          imported.texture_refs[i] = search->second;
        }
      }
      result.emplace_back(std::move(imported));
    }
    if (parsed_gltf) {
      *parsed_gltf = true;
    }
    return result;
  } catch (const std::exception& e) {
    EVOENGINE_WARNING("Unable to read glTF material extension data from " + path.filename().string() + ": " + e.what())
    return {};
  }
}

int32_t ImportedTextureStorageIndex(const std::shared_ptr<Texture2D>& texture) {
  return texture ? static_cast<int32_t>(texture->GetTextureStorageIndex()) : -1;
}

bool ReadTexturePixelsForPacking(const std::shared_ptr<Texture2D>& texture, const glm::uvec2& resolution,
                                 std::vector<glm::vec4>& pixels) {
  if (!texture) {
    return false;
  }
  const auto pixel_count = static_cast<size_t>(resolution.x) * resolution.y;
  const auto& local_pixels = texture->PeekLocalData();
  if (texture->GetResolution() == resolution && local_pixels.size() == pixel_count) {
    pixels = local_pixels;
    return true;
  }
  texture->GetRgbaChannelData(pixels, static_cast<int>(resolution.x), static_cast<int>(resolution.y));
  return pixels.size() == pixel_count;
}

std::shared_ptr<Texture2D> BuildPackedMetallicRoughnessTexture(const std::shared_ptr<Texture2D>& roughness_texture,
                                                               const std::shared_ptr<Texture2D>& metallic_texture,
                                                               const float roughness_factor,
                                                               const float metallic_factor) {
  if (!roughness_texture && !metallic_texture) {
    return nullptr;
  }

  const glm::uvec2 resolution =
      roughness_texture ? roughness_texture->GetResolution() : metallic_texture->GetResolution();
  if (resolution.x == 0 || resolution.y == 0) {
    return nullptr;
  }

  const auto pixel_count = static_cast<size_t>(resolution.x) * resolution.y;
  std::vector<glm::vec3> packed_pixels(pixel_count, glm::vec3(1.0f, roughness_factor, metallic_factor));

  std::vector<glm::vec4> source_pixels;
  if (roughness_texture) {
    if (ReadTexturePixelsForPacking(roughness_texture, resolution, source_pixels)) {
      for (size_t i = 0; i < pixel_count; ++i) {
        packed_pixels[i].g = source_pixels[i].r;
      }
    }
  }
  if (metallic_texture) {
    if (ReadTexturePixelsForPacking(metallic_texture, resolution, source_pixels)) {
      for (size_t i = 0; i < pixel_count; ++i) {
        packed_pixels[i].b = source_pixels[i].r;
      }
    }
  }

  auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  texture->SetRgbChannelData(packed_pixels, resolution, true);
  return texture;
}

auto ReadMaterial(const std::string& directory,
                  std::unordered_map<std::string, std::shared_ptr<Texture2D>>& loaded_textures,
                  std::vector<std::pair<std::shared_ptr<Texture2D>, std::shared_ptr<Texture2D>>>& opacity_maps,
                  const aiMaterial* importer_material, const ImportedGltfMaterialData* imported_material_data)
    -> std::shared_ptr<Material> {
  auto target_material = AssetManager::CreateTemporaryAsset<Material>();
  if (imported_material_data) {
    target_material->SetGltfMaterialData(imported_material_data->material_data);
    target_material->RefTextureRefs() = imported_material_data->texture_refs;
  }
  std::shared_ptr<Texture2D> base_color_texture;
  std::shared_ptr<Texture2D> normal_texture;
  std::shared_ptr<Texture2D> metallic_texture;
  std::shared_ptr<Texture2D> roughness_texture;
  std::shared_ptr<Texture2D> occlusion_texture;
  if (importer_material && !imported_material_data) {
    // Direct glTF material data owns extension texture/factor semantics; Assimp is the fallback path for other imports.
    // PBR
    if (importer_material->GetTextureCount(aiTextureType_BASE_COLOR) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_BASE_COLOR, 0, &str);
      base_color_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
      target_material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, base_color_texture);
    }
    if (importer_material->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_DIFFUSE, 0, &str);
      base_color_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
      target_material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, base_color_texture);
    }
    if (importer_material->GetTextureCount(aiTextureType_NORMALS) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_NORMALS, 0, &str);
      normal_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
      target_material->SetTexture(&GltfShadeMaterial::normal_texture, normal_texture);
    } else if (importer_material->GetTextureCount(aiTextureType_HEIGHT) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_HEIGHT, 0, &str);
      normal_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
      target_material->SetTexture(&GltfShadeMaterial::normal_texture, normal_texture);
    } else if (importer_material->GetTextureCount(aiTextureType_NORMAL_CAMERA) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_NORMAL_CAMERA, 0, &str);
      normal_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
      target_material->SetTexture(&GltfShadeMaterial::normal_texture, normal_texture);
    }

    if (importer_material->GetTextureCount(aiTextureType_METALNESS) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_METALNESS, 0, &str);
      metallic_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
    }
    if (importer_material->GetTextureCount(aiTextureType_DIFFUSE_ROUGHNESS) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_DIFFUSE_ROUGHNESS, 0, &str);
      roughness_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
    }
    if (importer_material->GetTextureCount(aiTextureType_AMBIENT_OCCLUSION) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_AMBIENT_OCCLUSION, 0, &str);
      occlusion_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
      target_material->SetTexture(&GltfShadeMaterial::occlusion_texture, occlusion_texture);
    }
    if (importer_material->GetTextureCount(aiTextureType_OPACITY) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_OPACITY, 0, &str);
      const auto opacity_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
      opacity_maps.emplace_back(base_color_texture, opacity_texture);
    }
    if (importer_material->GetTextureCount(aiTextureType_TRANSMISSION) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_TRANSMISSION, 0, &str);
      const auto opacity_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
      opacity_maps.emplace_back(base_color_texture, opacity_texture);
    }

    int unknown_texture_size = 0;
    if (importer_material->GetTextureCount(aiTextureType_EMISSIVE) > 0) {
      unknown_texture_size++;
    }
    if (importer_material->GetTextureCount(aiTextureType_SHININESS) > 0) {
      unknown_texture_size++;
    }
    if (importer_material->GetTextureCount(aiTextureType_DISPLACEMENT) > 0) {
      unknown_texture_size++;
    }
    if (importer_material->GetTextureCount(aiTextureType_LIGHTMAP) > 0) {
      unknown_texture_size++;
    }
    if (importer_material->GetTextureCount(aiTextureType_REFLECTION) > 0) {
      unknown_texture_size++;
    }
    if (importer_material->GetTextureCount(aiTextureType_EMISSION_COLOR) > 0) {
      unknown_texture_size++;
    }
    if (importer_material->GetTextureCount(aiTextureType_SHEEN) > 0) {
      unknown_texture_size++;
    }
    if (importer_material->GetTextureCount(aiTextureType_CLEARCOAT) > 0) {
      unknown_texture_size++;
    }

    if (importer_material->GetTextureCount(aiTextureType_UNKNOWN) > 0) {
      unknown_texture_size++;
    }

    aiColor3D color;
    if (importer_material->Get(AI_MATKEY_COLOR_DIFFUSE, color) == aiReturn_SUCCESS) {
      target_material->material_data.shade_material.pbr_base_color_factor = glm::vec4(color.r, color.g, color.b, 1.0f);
    } else if (importer_material->Get(AI_MATKEY_BASE_COLOR, color) == aiReturn_SUCCESS) {
      target_material->material_data.shade_material.pbr_base_color_factor = glm::vec4(color.r, color.g, color.b, 1.0f);
    }
    ai_real factor;
    if (importer_material->Get(AI_MATKEY_METALLIC_FACTOR, factor) == aiReturn_SUCCESS) {
      target_material->material_data.shade_material.pbr_metallic_factor = factor;
    }
    if (importer_material->Get(AI_MATKEY_ROUGHNESS_FACTOR, factor) == aiReturn_SUCCESS) {
      target_material->material_data.shade_material.pbr_roughness_factor = factor;
    }
    if (importer_material->Get(AI_MATKEY_SPECULAR_FACTOR, factor) == aiReturn_SUCCESS) {
      target_material->material_data.shade_material.specular_factor = factor;
    }
  }
  if (!imported_material_data) {
    if (base_color_texture && !target_material->draw_settings.blending) {
      target_material->material_data.shade_material.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Mask);
      target_material->material_data.shade_material.alpha_cutoff = 0.5f;
    }
    target_material->SetTexture(
        &GltfShadeMaterial::pbr_metallic_roughness_texture,
        BuildPackedMetallicRoughnessTexture(roughness_texture, metallic_texture,
                                            target_material->material_data.shade_material.pbr_roughness_factor,
                                            target_material->material_data.shade_material.pbr_metallic_factor));
  }
  target_material->MarkDirty();
  return target_material;
}

float ImportedTangentHandedness(const aiMesh* mesh, int vertex_index);

std::pair<std::vector<MorphTarget>, std::vector<float>> ReadImportedMorphTargets(const aiMesh* mesh) {
  std::vector<MorphTarget> targets;
  std::vector<float> weights;
  if (!mesh) {
    return {targets, weights};
  }
  targets.reserve(mesh->mNumAnimMeshes);
  weights.reserve(mesh->mNumAnimMeshes);
  const auto cast = [](const aiVector3D& value) {
    return glm::vec3(value.x, value.y, value.z);
  };
  for (uint32_t target_index = 0; target_index < mesh->mNumAnimMeshes; target_index++) {
    const auto* source = mesh->mAnimMeshes[target_index];
    if (!source || source->mNumVertices != mesh->mNumVertices) {
      continue;
    }
    MorphTarget target;
    target.name = source->mName.length == 0 ? "target_" + std::to_string(target_index) : source->mName.C_Str();
    const auto read_deltas = [&](std::vector<glm::vec3>& deltas, const aiVector3D* values,
                                 const aiVector3D* base_values) {
      if (!values || !base_values) {
        return;
      }
      deltas.resize(mesh->mNumVertices);
      for (uint32_t vertex_index = 0; vertex_index < mesh->mNumVertices; vertex_index++) {
        deltas[vertex_index] = cast(values[vertex_index]) - cast(base_values[vertex_index]);
      }
    };
    read_deltas(target.position_deltas, source->mVertices, mesh->mVertices);
    read_deltas(target.normal_deltas, source->mNormals, mesh->mNormals);
    read_deltas(target.tangent_deltas, source->mTangents, mesh->mTangents);
    targets.emplace_back(std::move(target));
    weights.emplace_back(source->mWeight);
  }
  return {std::move(targets), std::move(weights)};
}

template <typename VertexType>
std::vector<VertexType> RemapMorphBaseVertices(const std::vector<VertexType>& source_vertices,
                                               const std::vector<uint32_t>& source_vertex_indices,
                                               const std::vector<MorphTarget>& morph_targets,
                                               const std::vector<VertexType>& evaluated_vertices) {
  auto result = evaluated_vertices;
  const auto has_stream = [&](const auto stream) {
    return std::any_of(morph_targets.begin(), morph_targets.end(), [&](const MorphTarget& target) {
      return !(target.*stream).empty();
    });
  };
  const bool positions = has_stream(&MorphTarget::position_deltas);
  const bool normals = has_stream(&MorphTarget::normal_deltas);
  const bool tangents = has_stream(&MorphTarget::tangent_deltas);
  for (size_t index = 0; index < result.size(); index++) {
    const auto& source = source_vertices.at(source_vertex_indices.at(index));
    if (positions) {
      result[index].position = source.position;
    }
    if (normals) {
      result[index].normal = source.normal;
    }
    if (tangents) {
      result[index].tangent = source.tangent;
    }
  }
  return result;
}

glm::vec2 ReadImportedTexCoord(const aiMesh* mesh, const int channel, const int vertex_index,
                               const bool restore_gltf_coordinates) {
  glm::vec2 tex_coord(mesh->mTextureCoords[channel][vertex_index].x, mesh->mTextureCoords[channel][vertex_index].y);
  if (restore_gltf_coordinates) {
    tex_coord.y = 1.0f - tex_coord.y;
  }
  return tex_coord;
}

std::shared_ptr<Mesh> ReadMesh(aiMesh* importer_mesh, const bool restore_gltf_coordinates,
                               const int tangent_tex_coord) {
  VertexAttributes attributes;
  std::vector<Vertex> vertices;
  std::vector<unsigned> indices;
  if (importer_mesh->mNumVertices == 0 || !importer_mesh->HasFaces())
    return nullptr;
  vertices.resize(importer_mesh->mNumVertices);
  // Walk through each of the mesh's vertices
  for (int i = 0; i < importer_mesh->mNumVertices; i++) {
    Vertex vertex;
    glm::vec3 v3;  // we declare a placeholder vector since assimp uses its own vector class that doesn't directly
    // convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
    // positions
    v3.x = importer_mesh->mVertices[i].x;
    v3.y = importer_mesh->mVertices[i].y;
    v3.z = importer_mesh->mVertices[i].z;
    vertex.position = v3;
    if (importer_mesh->HasNormals()) {
      v3.x = importer_mesh->mNormals[i].x;
      v3.y = importer_mesh->mNormals[i].y;
      v3.z = importer_mesh->mNormals[i].z;
      vertex.normal = v3;
      attributes.normal = true;
    } else {
      attributes.normal = false;
    }
    if (importer_mesh->HasTangentsAndBitangents()) {
      v3.x = importer_mesh->mTangents[i].x;
      v3.y = importer_mesh->mTangents[i].y;
      v3.z = importer_mesh->mTangents[i].z;
      vertex.tangent = v3;
      vertex.vertex_info3 = ImportedTangentHandedness(importer_mesh, i);
      attributes.tangent = true;
    } else {
      attributes.tangent = false;
    }
    if (importer_mesh->HasVertexColors(0)) {
      const auto& color = importer_mesh->mColors[0][i];
      vertex.color = glm::vec4(color.r, color.g, color.b, color.a);
      if (restore_gltf_coordinates) {
        vertex.color = glm::clamp(vertex.color, glm::vec4(0.0f), glm::vec4(1.0f));
      }
      attributes.color = true;
    } else {
      attributes.color = false;
    }
    if (importer_mesh->HasTextureCoords(0)) {
      vertex.tex_coord = ReadImportedTexCoord(importer_mesh, 0, i, restore_gltf_coordinates);
      attributes.tex_coord = true;
    } else {
      vertex.tex_coord = glm::vec2(0.0f, 0.0f);
      attributes.tex_coord = false;
    }
    if (importer_mesh->HasTextureCoords(1)) {
      vertex.tex_coord_1 = ReadImportedTexCoord(importer_mesh, 1, i, restore_gltf_coordinates);
      attributes.tex_coord_1 = true;
    } else {
      vertex.tex_coord_1 = glm::vec2(0.0f);
      attributes.tex_coord_1 = false;
    }
    if (importer_mesh->HasTextureCoords(2)) {
      vertex.tex_coord_2 = ReadImportedTexCoord(importer_mesh, 2, i, restore_gltf_coordinates);
      attributes.tex_coord_2 = true;
    }
    if (importer_mesh->HasTextureCoords(3)) {
      vertex.tex_coord_3 = ReadImportedTexCoord(importer_mesh, 3, i, restore_gltf_coordinates);
      attributes.tex_coord_3 = true;
    }
    vertices[i] = vertex;
  }
  auto [morph_targets, default_morph_weights] = restore_gltf_coordinates
                                                    ? ReadImportedMorphTargets(importer_mesh)
                                                    : std::pair<std::vector<MorphTarget>, std::vector<float>>{};
  const auto morph_base_vertices = vertices;
  if (!morph_targets.empty()) {
    vertices = BuildMorphedVertices(vertices, morph_targets, {}, default_morph_weights);
  }
  // now walk through each of the mesh's _Faces (a face is a mesh its triangle) and retrieve the corresponding vertex
  // indices.
  for (int i = 0; i < importer_mesh->mNumFaces; i++) {
    assert(importer_mesh->mFaces[i].mNumIndices == 3);
    // retrieve all indices of the face and store them in the indices vector
    for (int j = 0; j < 3; j++)
      indices.push_back(importer_mesh->mFaces[i].mIndices[j]);
  }
  auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  std::vector<uint32_t> source_vertex_indices;
  mesh->SetVertices(attributes, vertices, indices, tangent_tex_coord, &source_vertex_indices);
  if (!morph_targets.empty()) {
    RemapMorphTargets(morph_targets, source_vertex_indices);
    auto remapped_base =
        RemapMorphBaseVertices(morph_base_vertices, source_vertex_indices, morph_targets, mesh->PeekVertices());
    mesh->SetMorphTargets(std::move(morph_targets), std::move(default_morph_weights), std::move(remapped_base));
  }
  return mesh;
}
std::shared_ptr<SkinnedMesh> ReadSkinnedMesh(
    std::unordered_map<Handle, std::vector<std::shared_ptr<Bone>>>& bones_lists,
    std::unordered_map<std::string, std::shared_ptr<Bone>>& bones_map, aiMesh* importer_mesh,
    const bool restore_gltf_coordinates, const int tangent_tex_coord) {
  SkinnedVertexAttributes skinned_vertex_attributes{};
  std::vector<SkinnedVertex> vertices;
  std::vector<unsigned> indices;
  if (importer_mesh->mNumVertices == 0 || !importer_mesh->HasFaces())
    return nullptr;
  vertices.resize(importer_mesh->mNumVertices);
  // Walk through each of the mesh's vertices
  for (int i = 0; i < importer_mesh->mNumVertices; i++) {
    SkinnedVertex vertex;
    glm::vec3 v3;  // we declare a placeholder vector since assimp uses its own vector class that doesn't directly
    // convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
    // positions
    v3.x = importer_mesh->mVertices[i].x;
    v3.y = importer_mesh->mVertices[i].y;
    v3.z = importer_mesh->mVertices[i].z;
    vertex.position = v3;
    if (importer_mesh->HasNormals()) {
      v3.x = importer_mesh->mNormals[i].x;
      v3.y = importer_mesh->mNormals[i].y;
      v3.z = importer_mesh->mNormals[i].z;
      vertex.normal = v3;
      skinned_vertex_attributes.normal = true;
    }
    if (importer_mesh->HasTangentsAndBitangents()) {
      v3.x = importer_mesh->mTangents[i].x;
      v3.y = importer_mesh->mTangents[i].y;
      v3.z = importer_mesh->mTangents[i].z;
      vertex.tangent = v3;
      vertex.vertex_info3 = ImportedTangentHandedness(importer_mesh, i);
      skinned_vertex_attributes.tangent = true;
    }
    if (importer_mesh->HasVertexColors(0)) {
      const auto& color = importer_mesh->mColors[0][i];
      vertex.color = glm::vec4(color.r, color.g, color.b, color.a);
      if (restore_gltf_coordinates) {
        vertex.color = glm::clamp(vertex.color, glm::vec4(0.0f), glm::vec4(1.0f));
      }
      skinned_vertex_attributes.color = true;
    }
    if (importer_mesh->HasTextureCoords(0)) {
      vertex.tex_coord = ReadImportedTexCoord(importer_mesh, 0, i, restore_gltf_coordinates);
      skinned_vertex_attributes.tex_coord = true;
    } else {
      vertex.tex_coord = glm::vec2(0.0f, 0.0f);
      skinned_vertex_attributes.tex_coord = false;
    }
    if (importer_mesh->HasTextureCoords(1)) {
      vertex.tex_coord_1 = ReadImportedTexCoord(importer_mesh, 1, i, restore_gltf_coordinates);
      skinned_vertex_attributes.tex_coord_1 = true;
    } else {
      vertex.tex_coord_1 = glm::vec2(0.0f);
      skinned_vertex_attributes.tex_coord_1 = false;
    }
    if (importer_mesh->HasTextureCoords(2)) {
      vertex.tex_coord_2 = ReadImportedTexCoord(importer_mesh, 2, i, restore_gltf_coordinates);
      skinned_vertex_attributes.tex_coord_2 = true;
    }
    if (importer_mesh->HasTextureCoords(3)) {
      vertex.tex_coord_3 = ReadImportedTexCoord(importer_mesh, 3, i, restore_gltf_coordinates);
      skinned_vertex_attributes.tex_coord_3 = true;
    }
    vertices[i] = vertex;
  }
  auto [morph_targets, default_morph_weights] = restore_gltf_coordinates
                                                    ? ReadImportedMorphTargets(importer_mesh)
                                                    : std::pair<std::vector<MorphTarget>, std::vector<float>>{};
  const auto morph_base_vertices = vertices;
  if (!morph_targets.empty()) {
    vertices = BuildMorphedVertices(vertices, morph_targets, {}, default_morph_weights);
  }
  // now walk through each of the mesh's _Faces (a face is a mesh its triangle) and retrieve the corresponding vertex
  // indices.
  for (int i = 0; i < importer_mesh->mNumFaces; i++) {
    assert(importer_mesh->mFaces[i].mNumIndices == 3);
    // retrieve all indices of the face and store them in the indices vector
    for (int j = 0; j < 3; j++)
      indices.push_back(importer_mesh->mFaces[i].mIndices[j]);
  }
  auto skinned_mesh = AssetManager::CreateTemporaryAsset<SkinnedMesh>();
#pragma region Read bones
  std::vector<std::vector<std::pair<int, float>>> vertices_bone_id_weights;
  vertices_bone_id_weights.resize(vertices.size());
  for (unsigned i = 0; i < importer_mesh->mNumBones; i++) {
    aiBone* importer_bone = importer_mesh->mBones[i];
    auto name = importer_bone->mName.C_Str();
    if (const auto search = bones_map.find(name); search == bones_map.end())  // If we can't find this bone
    {
      auto bone = std::make_shared<Bone>();
      bone->name = name;
      bone->offset_matrix.value = Mat4Cast(importer_bone->mOffsetMatrix);
      bones_map[name] = bone;
      bones_lists[skinned_mesh->GetHandle()].push_back(bone);
    } else {
      bones_lists[skinned_mesh->GetHandle()].push_back(search->second);
    }

    for (int j = 0; j < importer_bone->mNumWeights; j++) {
      vertices_bone_id_weights[importer_bone->mWeights[j].mVertexId].emplace_back(i,
                                                                                  importer_bone->mWeights[j].mWeight);
    }
  }
  for (unsigned i = 0; i < vertices_bone_id_weights.size(); i++) {
    auto ids = glm::ivec4(-1);
    auto weights = glm::vec4(0.0f);
    auto& list = vertices_bone_id_weights[i];
    for (unsigned j = 0; j < 4; j++) {
      if (!list.empty()) {
        int extract = -1;
        float max = -1.0f;
        for (int k = 0; k < list.size(); k++) {
          if (list[k].second > max) {
            max = list[k].second;
            extract = k;
          }
        }
        ids[j] = list[extract].first;
        weights[j] = list[extract].second;
        list.erase(list.begin() + extract);
      } else
        break;
    }
    vertices[i].bond_id = ids;
    vertices[i].weight = weights;

    ids = glm::ivec4(-1);
    weights = glm::vec4(0.0f);
    for (unsigned j = 0; j < 4; j++) {
      if (!list.empty()) {
        int extract = -1;
        float max = -1.0f;
        for (int k = 0; k < list.size(); k++) {
          if (list[k].second > max) {
            max = list[k].second;
            extract = k;
          }
        }
        ids[j] = list[extract].first;
        weights[j] = list[extract].second;
        list.erase(list.begin() + extract);
      } else
        break;
    }
    vertices[i].bond_id2 = ids;
    vertices[i].weight2 = weights;
  }
#pragma endregion
  std::vector<uint32_t> source_vertex_indices;
  skinned_mesh->SetVertices(skinned_vertex_attributes, vertices, indices, tangent_tex_coord, &source_vertex_indices);
  if (!morph_targets.empty()) {
    RemapMorphTargets(morph_targets, source_vertex_indices);
    auto remapped_base = RemapMorphBaseVertices(morph_base_vertices, source_vertex_indices, morph_targets,
                                                skinned_mesh->PeekSkinnedVertices());
    skinned_mesh->SetMorphTargets(std::move(morph_targets), std::move(default_morph_weights), std::move(remapped_base));
  }
  return skinned_mesh;
}

std::unordered_map<std::string, const aiLight*> BuildImportedLightMap(const aiScene& scene) {
  std::unordered_map<std::string, const aiLight*> lights;
  for (unsigned int light_index = 0; light_index < scene.mNumLights; ++light_index) {
    const auto* light = scene.mLights[light_index];
    if (!light)
      continue;
    lights.try_emplace(light->mName.C_Str(), light);
  }
  return lights;
}

glm::vec3 Vec3Cast(const aiVector3D& value) {
  return {value.x, value.y, value.z};
}

glm::vec3 ColorCast(const aiColor3D& value) {
  return {value.r, value.g, value.b};
}

float ImportedTangentHandedness(const aiMesh* mesh, const int vertex_index) {
  if (!mesh || !mesh->HasTangentsAndBitangents() || !mesh->HasNormals()) {
    return 1.0f;
  }
  const auto tangent = Vec3Cast(mesh->mTangents[vertex_index]);
  const auto bitangent = Vec3Cast(mesh->mBitangents[vertex_index]);
  const auto normal = Vec3Cast(mesh->mNormals[vertex_index]);
  return glm::dot(glm::cross(tangent, bitangent), normal) < 0.0f ? -1.0f : 1.0f;
}

std::string ImportedLightTypeName(const aiLightSourceType type) {
  switch (type) {
    case aiLightSource_DIRECTIONAL:
      return "directional";
    case aiLightSource_POINT:
      return "point";
    case aiLightSource_SPOT:
      return "spot";
    default:
      return "unsupported";
  }
}

std::optional<float> ReadImportedLightRange(const aiNode* importer_node) {
  if (!importer_node || !importer_node->mMetaData) {
    return std::nullopt;
  }
  float range = 0.0f;
  if (!importer_node->mMetaData->Get("PBR_LightRange", range) || !std::isfinite(range) || range <= 0.0f) {
    return std::nullopt;
  }
  return range;
}

glm::quat SafeLookAt(const glm::vec3& front, const glm::vec3& requested_up) {
  if (glm::dot(front, front) <= 0.0f) {
    return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  }
  const glm::vec3 normalized_front = glm::normalize(front);
  glm::vec3 up =
      glm::dot(requested_up, requested_up) > 0.0f ? glm::normalize(requested_up) : glm::vec3(0.0f, 1.0f, 0.0f);
  if (const auto cross = glm::cross(normalized_front, up); glm::dot(cross, cross) <= 0.0001f) {
    up = glm::abs(normalized_front.y) < 0.99f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
  }
  return glm::quatLookAt(normalized_front, up);
}

void ApplyImportedLightColor(const aiLight& light, glm::vec3& diffuse, float& diffuse_brightness) {
  const glm::vec3 color_with_intensity = glm::max(ColorCast(light.mColorDiffuse), glm::vec3(0.0f));
  const float brightness = glm::max(glm::max(color_with_intensity.x, color_with_intensity.y), color_with_intensity.z);
  if (brightness > 0.0f) {
    diffuse = color_with_intensity / brightness;
    diffuse_brightness = brightness;
  } else {
    diffuse = glm::vec3(0.0f);
    diffuse_brightness = 0.0f;
  }
}

Transform CreateImportedLightLocalTransform(const aiLight& light) {
  Transform transform;
  const glm::vec3 position = Vec3Cast(light.mPosition);
  const glm::vec3 direction = Vec3Cast(light.mDirection);
  const glm::vec3 up = Vec3Cast(light.mUp);
  glm::quat rotation(1.0f, 0.0f, 0.0f, 0.0f);
  if (light.mType == aiLightSource_DIRECTIONAL) {
    rotation = SafeLookAt(-direction, up);
  } else if (light.mType == aiLightSource_SPOT) {
    rotation = SafeLookAt(direction, up);
  }
  transform.SetValue(position, rotation, glm::vec3(1.0f));
  return transform;
}

void PushImportedLightPrefab(Prefab* model_node, const aiNode* importer_node,
                             const std::shared_ptr<IPrivateComponent>& light_component,
                             const Transform& light_local_transform, const std::string& type_name) {
  auto light_node = AssetManager::CreateTemporaryAsset<Prefab>();
  light_node->instance_name = std::string(importer_node->mName.C_Str()) + " " + type_name + " light";
  SetPrefabLocalTransform(light_node.get(), light_local_transform.value);

  PrivateComponentHolder holder;
  holder.enabled = true;
  holder.private_component = light_component;
  light_node->private_components.push_back(holder);
  model_node->child_prefabs.push_back(std::move(light_node));
}

void ApplyImportedLightRange(PointLight& light, const std::optional<float>& range) {
  if (!range) {
    return;
  }
  light.range = *range;
  light.shadow_distance = *range;
}

void ApplyImportedLightRange(SpotLight& light, const std::optional<float>& range) {
  if (!range) {
    return;
  }
  light.range = *range;
  light.shadow_distance = *range;
}

bool AttachImportedPunctualLight(Prefab* model_node, const aiNode* importer_node,
                                 const std::unordered_map<std::string, const aiLight*>& imported_lights) {
  const auto search = imported_lights.find(importer_node->mName.C_Str());
  if (search == imported_lights.end()) {
    return false;
  }
  const aiLight& imported_light = *search->second;
  const auto type_name = ImportedLightTypeName(imported_light.mType);
  const auto range = ReadImportedLightRange(importer_node);
  std::shared_ptr<IPrivateComponent> component;
  switch (imported_light.mType) {
    case aiLightSource_DIRECTIONAL: {
      auto light = Serialization::ProduceSerializable<DirectionalLight>();
      ApplyImportedLightColor(imported_light, light->diffuse, light->diffuse_brightness);
      component = std::static_pointer_cast<IPrivateComponent>(light);
    } break;
    case aiLightSource_POINT: {
      auto light = Serialization::ProduceSerializable<PointLight>();
      ApplyImportedLightColor(imported_light, light->diffuse, light->diffuse_brightness);
      light->constant = imported_light.mAttenuationConstant;
      light->linear = imported_light.mAttenuationLinear;
      light->quadratic = imported_light.mAttenuationQuadratic;
      ApplyImportedLightRange(*light, range);
      component = std::static_pointer_cast<IPrivateComponent>(light);
    } break;
    case aiLightSource_SPOT: {
      auto light = Serialization::ProduceSerializable<SpotLight>();
      ApplyImportedLightColor(imported_light, light->diffuse, light->diffuse_brightness);
      light->constant = imported_light.mAttenuationConstant;
      light->linear = imported_light.mAttenuationLinear;
      light->quadratic = imported_light.mAttenuationQuadratic;
      light->inner_degrees = glm::degrees(imported_light.mAngleInnerCone);
      light->outer_degrees = glm::degrees(imported_light.mAngleOuterCone);
      ApplyImportedLightRange(*light, range);
      component = std::static_pointer_cast<IPrivateComponent>(light);
    } break;
    default:
      EVOENGINE_WARNING("Skipped unsupported imported punctual light '" + std::string(importer_node->mName.C_Str()) +
                        "' of type " + std::to_string(static_cast<int>(imported_light.mType)))
      return false;
  }

  PushImportedLightPrefab(model_node, importer_node, component, CreateImportedLightLocalTransform(imported_light),
                          type_name);
  return true;
}

auto ProcessNode(const std::string& directory, Prefab* model_node,
                 std::unordered_map<unsigned, std::shared_ptr<Material>>& loaded_materials,
                 std::unordered_map<std::string, std::shared_ptr<Texture2D>>& texture_2ds_loaded,
                 std::vector<std::pair<std::shared_ptr<Texture2D>, std::shared_ptr<Texture2D>>>& opacity_maps,
                 const std::vector<ImportedGltfMaterialData>& gltf_material_data, const bool restore_gltf_coordinates,
                 const std::unordered_map<std::string, const aiLight*>& imported_lights,
                 std::unordered_map<Handle, std::vector<std::shared_ptr<Bone>>>& bones_lists,
                 std::unordered_map<std::string, std::shared_ptr<Bone>>& bones_map, const aiNode* importer_node,
                 const std::shared_ptr<AssimpImportNode>& assimp_node, const aiScene* importer_scene,
                 const std::shared_ptr<Animation>& animation) -> bool {
  bool added_mesh_renderer = false;
  SetPrefabLocalTransform(model_node,
                          importer_node->mParent ? Mat4Cast(importer_node->mTransformation) : Transform().value);
  added_mesh_renderer = AttachImportedPunctualLight(model_node, importer_node, imported_lights);
  for (unsigned i = 0; i < importer_node->mNumMeshes; i++) {
    // the modelNode object only contains indices to index the actual objects in the scene.
    // the scene contains all the data, modelNode is just to keep stuff organized (like relations between nodes).
    aiMesh* importer_mesh = importer_scene->mMeshes[importer_node->mMeshes[i]];
    if (!importer_mesh)
      continue;
    auto child_node = AssetManager::CreateTemporaryAsset<Prefab>();
    child_node->instance_name = std::string(importer_mesh->mName.C_Str());
    const auto search = loaded_materials.find(importer_mesh->mMaterialIndex);
    const bool is_skinned_mesh = importer_mesh->mNumBones != 0xffffffff && importer_mesh->mBones;
    const auto imported_material_data = importer_mesh->mMaterialIndex < gltf_material_data.size()
                                            ? &gltf_material_data[importer_mesh->mMaterialIndex]
                                            : nullptr;
    int tangent_tex_coord = 0;
    if (imported_material_data) {
      const auto normal_texture = imported_material_data->material_data.shade_material.normal_texture;
      if (normal_texture < imported_material_data->material_data.texture_infos.size()) {
        tangent_tex_coord = imported_material_data->material_data.texture_infos[normal_texture].tex_coord;
      }
    }
    std::shared_ptr<Material> material;
    if (search == loaded_materials.end()) {
      const aiMaterial* importer_material = nullptr;
      if (importer_mesh->mMaterialIndex != 0xffffffff && importer_mesh->mMaterialIndex < importer_scene->mNumMaterials)
        importer_material = importer_scene->mMaterials[importer_mesh->mMaterialIndex];
      material = ReadMaterial(directory, texture_2ds_loaded, opacity_maps, importer_material, imported_material_data);
      loaded_materials[importer_mesh->mMaterialIndex] = material;
    } else {
      material = search->second;
    }

    if (is_skinned_mesh) {
      auto skinned_mesh_renderer = Serialization::ProduceSerializable<SkinnedMeshRenderer>();
      skinned_mesh_renderer->material.Set<Material>(material);
      skinned_mesh_renderer->skinned_mesh.Set<SkinnedMesh>(
          ReadSkinnedMesh(bones_lists, bones_map, importer_mesh, restore_gltf_coordinates, tangent_tex_coord));
      if (!skinned_mesh_renderer->skinned_mesh.Get())
        continue;
      added_mesh_renderer = true;
      PrivateComponentHolder holder;
      holder.enabled = true;
      holder.private_component = std::static_pointer_cast<IPrivateComponent>(skinned_mesh_renderer);
      child_node->private_components.push_back(holder);
    } else {
      auto mesh_renderer = Serialization::ProduceSerializable<MeshRenderer>();
      mesh_renderer->material.Set<Material>(material);
      mesh_renderer->mesh.Set<Mesh>(ReadMesh(importer_mesh, restore_gltf_coordinates, tangent_tex_coord));
      if (!mesh_renderer->mesh.Get())
        continue;
      added_mesh_renderer = true;
      PrivateComponentHolder holder;
      holder.enabled = true;
      holder.private_component = std::static_pointer_cast<IPrivateComponent>(mesh_renderer);
      child_node->private_components.push_back(holder);
    }
    auto transform = std::make_shared<Transform>();
    transform->value = Transform().value;

    DataComponentHolder holder;
    holder.data_component_type = Typeof<Transform>();
    holder.data_component = transform;
    child_node->data_components.push_back(holder);

    model_node->child_prefabs.push_back(std::move(child_node));
  }

  for (unsigned i = 0; i < importer_node->mNumChildren; i++) {
    auto child_node = AssetManager::CreateTemporaryAsset<Prefab>();
    child_node->instance_name = std::string(importer_node->mChildren[i]->mName.C_Str());
    auto child_assimp_node = std::make_shared<AssimpImportNode>(importer_node->mChildren[i]);
    child_assimp_node->parent_node = assimp_node;
    const bool child_add =
        ProcessNode(directory, child_node.get(), loaded_materials, texture_2ds_loaded, opacity_maps, gltf_material_data,
                    restore_gltf_coordinates, imported_lights, bones_lists, bones_map, importer_node->mChildren[i],
                    child_assimp_node, importer_scene, animation);
    if (child_add) {
      model_node->child_prefabs.push_back(std::move(child_node));
    }
    added_mesh_renderer = added_mesh_renderer | child_add;
    assimp_node->child_nodes.push_back(std::move(child_assimp_node));
  }
  return added_mesh_renderer;
}

std::string LowercaseExtension(const std::filesystem::path& path) {
  auto extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](const unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });
  return extension;
}

bool IsNativePrefabPath(const std::filesystem::path& path) {
  return LowercaseExtension(path) == ".eveprefab";
}

bool SupportsStagedModelImportPath(const std::filesystem::path& path) {
  const auto extension = LowercaseExtension(path);
  return extension == ".obj" || extension == ".gltf" || extension == ".glb" || extension == ".blend" ||
         extension == ".ply" || extension == ".fbx" || extension == ".dae" || extension == ".x3d";
}

using PrefabImportClock = std::chrono::steady_clock;

void LogPrefabImportPhaseDuration(const std::filesystem::path& path, const std::string& phase,
                                  const PrefabImportClock::time_point start) {
  const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(PrefabImportClock::now() - start);
  if (duration < std::chrono::milliseconds(16)) {
    return;
  }
  EVOENGINE_WARNING("Prefab import phase took " + std::to_string(duration.count()) + " ms: " + phase + " " +
                    path.filename().string())
}

bool Prefab::LoadModelSceneInternal(const std::filesystem::path& path, const aiScene& scene) {
  auto temp = path;
  const std::string directory = temp.remove_filename().string();
  instance_name = path.filename().string();
  std::unordered_map<unsigned, std::shared_ptr<Material>> loaded_materials;
  std::vector<std::pair<std::shared_ptr<Texture2D>, std::shared_ptr<Texture2D>>> opacity_maps;
  std::unordered_map<std::string, std::shared_ptr<Bone>> bones_map;
  std::shared_ptr<Animation> animation;
  std::shared_ptr<AssimpImportNode> root_assimp_node = std::make_shared<AssimpImportNode>(scene.mRootNode);

  std::unordered_map<Handle, std::vector<std::shared_ptr<Bone>>> bones_lists;
  const auto process_nodes_start = PrefabImportClock::now();
  std::unordered_map<std::string, std::shared_ptr<Texture2D>> loaded_textures;
  bool parsed_gltf = false;
  const auto gltf_material_data = ReadGltfMaterialData(path, directory, loaded_textures, scene, &parsed_gltf);
  const auto extension = LowercaseExtension(path);
  const bool restore_gltf_coordinates = (extension == ".gltf" || extension == ".glb") && parsed_gltf;
  const auto imported_lights = BuildImportedLightMap(scene);
  if (!ProcessNode(directory, this, loaded_materials, loaded_textures, opacity_maps, gltf_material_data,
                   restore_gltf_coordinates, imported_lights, bones_lists, bones_map, scene.mRootNode, root_assimp_node,
                   &scene, animation)) {
    EVOENGINE_ERROR("Model is empty!")
    return false;
  }
  LogPrefabImportPhaseDuration(path, "Process nodes", process_nodes_start);

  const auto opacity_maps_start = PrefabImportClock::now();
  for (auto& pair : opacity_maps) {
    std::vector<glm::vec4> color_data;
    const auto& albedo_texture = pair.first;
    const auto& opacity_texture = pair.second;
    if (!albedo_texture || !opacity_texture)
      continue;
    const auto resolution = albedo_texture->GetResolution();
    if (!ReadTexturePixelsForPacking(albedo_texture, resolution, color_data))
      continue;
    std::vector<glm::vec4> alpha_data;
    ReadTexturePixelsForPacking(opacity_texture, resolution, alpha_data);
    if (alpha_data.size() < color_data.size())
      continue;
    Jobs::RunParallelFor(color_data.size(), [&](size_t i) {
      color_data[i].a = alpha_data[i].r;
    });
    std::shared_ptr<Texture2D> replacement_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
    replacement_texture->SetRgbaChannelData(color_data, albedo_texture->GetResolution(), true);
    pair.second = replacement_texture;
  }
  LogPrefabImportPhaseDuration(path, "Apply opacity maps", opacity_maps_start);

  const auto material_relink_start = PrefabImportClock::now();
  for (const auto& material : loaded_materials) {
    const auto base_color_texture = material.second->GetTexture(&GltfShadeMaterial::pbr_base_color_texture);
    if (!base_color_texture)
      continue;
    for (const auto& pair : opacity_maps) {
      if (base_color_texture->GetHandle() == pair.first->GetHandle()) {
        material.second->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, pair.second);
      }
    }
  }
  LogPrefabImportPhaseDuration(path, "Relink materials", material_relink_start);

  if (!bones_map.empty()) {
    const auto animation_start = PrefabImportClock::now();
    animation = AssetManager::CreateTemporaryAsset<Animation>();
    root_assimp_node->NecessaryWalker(bones_map);
    size_t index = 0;
    root_assimp_node->AttachToAnimator(animation, index);
    animation->bone_size = index + 1;
    ReadAnimations(&scene, animation, bones_map);
    ApplyBoneIndices(bones_lists, this);

    auto animator = Serialization::ProduceSerializable<Animator>();
    animator->Setup(animation);
    AttachAnimator(this, entity_handle);
    PrivateComponentHolder holder;
    holder.enabled = true;
    holder.private_component = std::static_pointer_cast<IPrivateComponent>(animator);
    private_components.push_back(holder);
    LogPrefabImportPhaseDuration(path, "Build animation", animation_start);
  } else if (scene.HasAnimations()) {
    EVOENGINE_WARNING("Skipped non-skeletal animation channels while importing " + path.filename().string())
  }
  const auto gather_assets_start = PrefabImportClock::now();
  GatherAssets();
  LogPrefabImportPhaseDuration(path, "Gather assets", gather_assets_start);
  return true;
}
#pragma endregion
void Prefab::ApplyBoneIndices(const std::unordered_map<Handle, std::vector<std::shared_ptr<Bone>>>& bones_lists,
                              Prefab* node) {
  if (const auto skinned_mesh_renderer = node->GetPrivateComponent<SkinnedMeshRenderer>()) {
    const auto skinned_mesh = skinned_mesh_renderer->skinned_mesh.Get<SkinnedMesh>();
    skinned_mesh->FetchIndices(bones_lists.at(skinned_mesh->GetHandle()));
  }
  for (auto& i : node->child_prefabs) {
    ApplyBoneIndices(bones_lists, i.get());
  }
}
#pragma region Model Loading
void Prefab::AttachChildrenPrivateComponent(const std::shared_ptr<Scene>& scene,
                                            const std::shared_ptr<Prefab>& model_node, const Entity& parent_entity,
                                            const std::unordered_map<Handle, Handle>& map) const {
  Entity entity;
  auto children = scene->GetChildren(parent_entity);
  for (auto& i : children) {
    auto a = scene->GetEntityHandle(i).GetValue();
    auto b = map.at(model_node->entity_handle).GetValue();
    if (a == b)
      entity = i;
  }
  if (entity.GetIndex() == 0)
    return;
  for (auto& i : model_node->private_components) {
    size_t id;
    auto ptr = std::static_pointer_cast<IPrivateComponent>(
        Serialization::ProduceSerializable(i.private_component->GetTypeName(), id));
    Serialization::ClonePrivateComponent(ptr, i.private_component);
    ptr->scene_ = scene;
    scene->SetPrivateComponent(entity, ptr);
  }
  int index = 0;
  for (auto& i : model_node->child_prefabs) {
    AttachChildrenPrivateComponent(scene, i, entity, map);
    index++;
  }
}
void Prefab::AttachChildren(const std::shared_ptr<Scene>& scene, const std::shared_ptr<Prefab>& model_node,
                            Entity parent_entity, std::unordered_map<Handle, Handle>& map) {
  std::vector<DataComponentType> types;
  for (auto& i : model_node->data_components) {
    types.emplace_back(i.data_component_type);
  }
  auto archetype = Entities::CreateEntityArchetype("", types);
  auto entity = scene->CreateEntity(archetype, model_node->instance_name);
  map[model_node->entity_handle] = scene->GetEntityHandle(entity);
  scene->SetEnable(entity, model_node->enabled_);
  scene->SetParent(entity, parent_entity);
  for (auto& i : model_node->data_components) {
    scene->SetDataComponent(entity.GetIndex(), i.data_component_type.type_index, i.data_component_type.type_size,
                            i.data_component.get());
  }
  int index = 0;
  for (auto& i : model_node->child_prefabs) {
    AttachChildren(scene, i, entity, map);
    index++;
  }
}

void Prefab::AttachAnimator(Prefab* parent, const Handle& animator_entity_handle) {
  if (const auto skinned_mesh_renderer = parent->GetPrivateComponent<SkinnedMeshRenderer>()) {
    skinned_mesh_renderer->animator.entity_handle_ = animator_entity_handle;
    skinned_mesh_renderer->animator.private_component_type_name_ = "Animator";
  }
  for (auto& i : parent->child_prefabs) {
    AttachAnimator(i.get(), animator_entity_handle);
  }
}

void Prefab::FromEntity(const Entity& entity) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    EVOENGINE_ERROR("Scene not attached!");
    return;
  }
  entity_handle = scene->GetEntityHandle(entity);
  instance_name = scene->GetEntityName(entity);
  enabled_ = scene->IsEntityEnabled(entity);
  scene->UnsafeForEachDataComponent(entity, [&](const DataComponentType& type, const void* data) {
    DataComponentHolder holder;
    holder.data_component_type = type;
    size_t id;
    size_t size;
    holder.data_component =
        std::static_pointer_cast<IDataComponent>(Serialization::ProduceDataComponent(type.type_name, id, size));
    memcpy(holder.data_component.get(), data, type.type_size);
    data_components.push_back(std::move(holder));
  });

  const auto& elements =
      scene->scene_data_storage_.entity_metadata_list.at(entity.GetIndex()).private_component_elements;
  for (auto& element : elements) {
    size_t id;
    auto ptr = std::static_pointer_cast<IPrivateComponent>(
        Serialization::ProduceSerializable(element.private_component_data->GetTypeName(), id));
    ptr->OnCreate();
    Serialization::ClonePrivateComponent(ptr, element.private_component_data);
    PrivateComponentHolder holder;
    holder.enabled = element.private_component_data->enabled_;
    holder.private_component = ptr;
    private_components.push_back(holder);
  }

  const auto children = scene->GetChildren(entity);
  for (auto& i : children) {
    auto temp = AssetManager::CreateTemporaryAsset<Prefab>();
    temp->instance_name = scene->GetEntityName(i);
    child_prefabs.push_back(temp);
    child_prefabs.back()->FromEntity(i);
  }
}
bool Prefab::LoadInternal(const std::filesystem::path& path) {
  if (path.extension() == ".eveprefab") {
    std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    YAML::Node in = YAML::Load(string_stream.str());
#pragma region Assets
    if (const auto& in_local_assets = in["LocalAssets"]) {
      std::vector<std::shared_ptr<IAsset>> local_assets;
      for (const auto& i : in_local_assets) {
        Handle handle = i["Handle"].as<uint64_t>();
        local_assets.push_back(AssetManager::CreateTemporaryAssetImpl(i["TypeName"].as<std::string>(), handle));
      }
      int index = 0;
      for (const auto& i : in_local_assets) {
        Serialization::DeserializeObject(i, *local_assets[index++]);
      }
    }

#pragma endregion
    Serialization::DeserializeObject(in, *this);
    return true;
  }
  return LoadModelInternal(path);
}

namespace {
class PrefabStagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  enum class Kind { NativePrefab, ModelImport };

  Kind kind = Kind::NativePrefab;
  YAML::Node node;
  std::filesystem::path path;
  std::unique_ptr<Assimp::Importer> importer;
  const aiScene* scene = nullptr;
};
}  // namespace

bool Prefab::SupportsStagedLoading(const std::filesystem::path& path) const {
  return IsNativePrefabPath(path) || SupportsStagedModelImportPath(path);
}

std::shared_ptr<StagedAssetLoadPayload> Prefab::LoadStagedPayloadInternal(const std::filesystem::path& path) const {
  auto payload = std::make_shared<PrefabStagedLoadPayload>();
  payload->path = path;
  if (!IsNativePrefabPath(path)) {
    payload->kind = PrefabStagedLoadPayload::Kind::ModelImport;
    payload->importer = std::make_unique<Assimp::Importer>();
    auto flags = aiProcess_Triangulate | aiProcess_CalcTangentSpace | aiProcess_GenSmoothNormals;
    const auto extension = LowercaseExtension(path);
    if (extension == ".gltf" || extension == ".glb") {
      flags &= ~aiProcess_CalcTangentSpace;
    }
    payload->scene = payload->importer->ReadFile(path.string(), flags);
    if (!payload->scene || payload->scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !payload->scene->mRootNode) {
      EVOENGINE_ERROR("Assimp: " + std::string(payload->importer->GetErrorString()))
      return {};
    }
    return payload;
  }

  try {
    const std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    payload->node = YAML::Load(string_stream.str());
    return payload;
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to load staged prefab payload: " + std::string(e.what()))
    return {};
  }
}

bool Prefab::ApplyStagedPayloadInternal(const std::filesystem::path&,
                                        const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  const auto prefab_payload = std::dynamic_pointer_cast<PrefabStagedLoadPayload>(payload);
  if (!prefab_payload) {
    return false;
  }
  if (prefab_payload->kind == PrefabStagedLoadPayload::Kind::ModelImport) {
    if (!prefab_payload->scene) {
      return false;
    }
    return LoadModelSceneInternal(prefab_payload->path, *prefab_payload->scene);
  }

  try {
    const auto& in = prefab_payload->node;
    if (const auto& in_local_assets = in["LocalAssets"]) {
      std::vector<std::shared_ptr<IAsset>> local_assets;
      for (const auto& i : in_local_assets) {
        Handle handle = i["Handle"].as<uint64_t>();
        local_assets.push_back(AssetManager::CreateTemporaryAssetImpl(i["TypeName"].as<std::string>(), handle));
      }
      int index = 0;
      for (const auto& i : in_local_assets) {
        Serialization::DeserializeObject(i, *local_assets[index++]);
      }
    }
    Serialization::DeserializeObject(in, *this);
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to apply staged prefab payload: " + std::string(e.what()))
    return false;
  }
  return true;
}

bool Prefab::RegisterAssetIoHandlers(const std::string& owner_name, const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<Prefab>(
      [](const Prefab& asset, const std::filesystem::path& path) {
        return asset.SaveInternal(path);
      },
      [](Prefab& asset, const std::filesystem::path& path) {
        return asset.LoadInternal(path);
      },
      [](const Prefab& asset, const std::filesystem::path& path) {
        return asset.SupportsStagedLoading(path);
      },
      [](const Prefab& asset, const std::filesystem::path& path) {
        return asset.LoadStagedPayloadInternal(path);
      },
      [](Prefab& asset, const std::filesystem::path& path, const std::shared_ptr<StagedAssetLoadPayload>& payload) {
        return asset.ApplyStagedPayloadInternal(path, payload);
      },
      owner_name, type_name);
}

bool Prefab::IsPrefabEnabled() const {
  return enabled_;
}

void Prefab::SetPrefabEnabled(const bool value) {
  enabled_ = value;
}

bool Prefab::LoadModelInternal(const std::filesystem::path& path, bool optimize, unsigned int flags) {
  flags = flags | aiProcess_Triangulate;
  const auto extension = LowercaseExtension(path);
  if (extension == ".gltf" || extension == ".glb") {
    flags &= ~aiProcess_CalcTangentSpace;
  }
  if (optimize) {
    flags = flags | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes;
  }
  // read file via ASSIMP
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(path.string(), flags);
  // check for errors
  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)  // if is Not Zero
  {
    EVOENGINE_ERROR("Assimp: " + std::string(importer.GetErrorString()))
    return false;
  }
  return LoadModelSceneInternal(path, *scene);
}

#pragma endregion

#pragma region Assimp Export

struct AssimpExportNode {
  int mesh_index = -1;
  std::string name;
  aiMatrix4x4 transform;

  std::vector<AssimpExportNode> children;

  void Collect(const std::shared_ptr<Prefab>& current_prefab,
               std::vector<std::pair<std::shared_ptr<Mesh>, int>>& meshes, std::vector<std::string>& mesh_names,
               std::vector<std::shared_ptr<Material>>& materials);

  void Process(aiNode* exporter_node);
};

void AssimpExportNode::Collect(const std::shared_ptr<Prefab>& current_prefab,
                               std::vector<std::pair<std::shared_ptr<Mesh>, int>>& meshes,
                               std::vector<std::string>& mesh_names,
                               std::vector<std::shared_ptr<Material>>& materials) {
  mesh_index = -1;
  name = current_prefab->instance_name;
  for (const auto& data_component : current_prefab->data_components) {
    if (data_component.data_component_type == Typeof<Transform>()) {
      transform = Mat4Cast(std::reinterpret_pointer_cast<Transform>(data_component.data_component)->value);
    }
  }
  for (const auto& private_component : current_prefab->private_components) {
    if (const auto mesh_renderer = std::dynamic_pointer_cast<MeshRenderer>(private_component.private_component)) {
      auto mesh = mesh_renderer->mesh.Get<Mesh>();
      auto material = mesh_renderer->material.Get<Material>();
      if (mesh && material) {
        int target_material_index = -1;
        for (int material_index = 0; material_index < materials.size(); material_index++) {
          if (materials[material_index] == material) {
            target_material_index = material_index;
          }
        }
        if (target_material_index == -1) {
          target_material_index = materials.size();
          materials.emplace_back(material);
        }

        if (mesh_index == -1) {
          mesh_index = meshes.size();
          meshes.emplace_back(mesh, target_material_index);
          mesh_names.emplace_back(current_prefab->instance_name);
        }
      }
    }
  }

  for (const auto& child_prefab : current_prefab->child_prefabs) {
    children.emplace_back();
    auto& new_node = children.back();
    new_node.Collect(child_prefab, meshes, mesh_names, materials);
  }
}

void AssimpExportNode::Process(aiNode* exporter_node) {
  exporter_node->mName = name;
  exporter_node->mTransformation = transform;

  if (mesh_index != -1) {
    exporter_node->mNumMeshes = 1;
    exporter_node->mMeshes = new unsigned int[1];
    exporter_node->mMeshes[0] = mesh_index;
  }
  exporter_node->mNumChildren = children.size();
  if (children.empty()) {
    exporter_node->mChildren = nullptr;
  } else {
    exporter_node->mChildren = new aiNode*[children.size()];
  }
  for (int i = 0; i < children.size(); i++) {
    exporter_node->mChildren[i] = new aiNode();
    exporter_node->mChildren[i]->mParent = exporter_node;
    children.at(i).Process(exporter_node->mChildren[i]);
  }
}

bool Prefab::SaveModelInternal(const std::filesystem::path& path) const {
  Assimp::Exporter exporter;
  aiScene exporter_scene{};
  exporter_scene.mMetaData = new aiMetadata();
  std::vector<std::pair<std::shared_ptr<Mesh>, int>> meshes;
  std::vector<std::shared_ptr<Material>> materials;
  std::vector<std::string> mesh_names;
  AssimpExportNode root_node;
  root_node.Collect(std::dynamic_pointer_cast<Prefab>(GetSelf()), meshes, mesh_names, materials);

  exporter_scene.mRootNode = new aiNode();
  exporter_scene.mRootNode->mName = instance_name;
  exporter_scene.mNumMeshes = meshes.size();
  if (meshes.empty()) {
    exporter_scene.mMeshes = nullptr;
  } else {
    exporter_scene.mMeshes = new aiMesh*[meshes.size()];
  }
  for (int mesh_index = 0; mesh_index < meshes.size(); mesh_index++) {
    aiMesh* exporter_mesh = exporter_scene.mMeshes[mesh_index] = new aiMesh();

    exporter_mesh->mName = aiString(mesh_names[mesh_index]);
    auto& mesh = meshes.at(mesh_index);
    const auto& vertices = mesh.first->UnsafeGetVertices();
    const auto& triangles = mesh.first->UnsafeGetTriangles();
    exporter_mesh->mNumVertices = vertices.size();
    exporter_mesh->mVertices = new aiVector3D[vertices.size()];
    exporter_mesh->mNormals = new aiVector3D[vertices.size()];
    exporter_mesh->mNumUVComponents[0] = 2;
    exporter_mesh->mTextureCoords[0] = new aiVector3D[vertices.size()];
    exporter_mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
    for (int vertex_index = 0; vertex_index < vertices.size(); vertex_index++) {
      exporter_mesh->mVertices[vertex_index].x = vertices.at(vertex_index).position.x;
      exporter_mesh->mVertices[vertex_index].y = vertices.at(vertex_index).position.y;
      exporter_mesh->mVertices[vertex_index].z = vertices.at(vertex_index).position.z;

      exporter_mesh->mNormals[vertex_index].x = vertices.at(vertex_index).normal.x;
      exporter_mesh->mNormals[vertex_index].y = vertices.at(vertex_index).normal.y;
      exporter_mesh->mNormals[vertex_index].z = vertices.at(vertex_index).normal.z;

      exporter_mesh->mTextureCoords[0][vertex_index].x = vertices.at(vertex_index).tex_coord.x;
      exporter_mesh->mTextureCoords[0][vertex_index].y = vertices.at(vertex_index).tex_coord.y;
      exporter_mesh->mTextureCoords[0][vertex_index].z = 0.f;
    }

    exporter_mesh->mNumFaces = triangles.size();
    if (triangles.empty()) {
      exporter_mesh->mFaces = nullptr;
    } else {
      exporter_mesh->mFaces = new aiFace[triangles.size()];
    }
    for (int triangle_index = 0; triangle_index < triangles.size(); triangle_index++) {
      exporter_mesh->mFaces[triangle_index].mIndices = new unsigned int[3];
      exporter_mesh->mFaces[triangle_index].mNumIndices = 3;
      exporter_mesh->mFaces[triangle_index].mIndices[0] = triangles[triangle_index][0];
      exporter_mesh->mFaces[triangle_index].mIndices[1] = triangles[triangle_index][1];
      exporter_mesh->mFaces[triangle_index].mIndices[2] = triangles[triangle_index][2];
    }
    exporter_mesh->mMaterialIndex = mesh.second;
    exporter_mesh->mName = std::string("mesh_") + std::to_string(mesh_index);
  }

  exporter_scene.mNumMaterials = materials.size();
  if (materials.empty()) {
    exporter_scene.mMaterials = nullptr;
  } else {
    exporter_scene.mMaterials = new aiMaterial*[materials.size()];
  }

  const auto texture_folder_path = std::filesystem::absolute(path.parent_path() / "textures");
  std::filesystem::create_directories(texture_folder_path);

  struct SeparatedTexturePath {
    aiString color;
    bool has_opacity = false;
    aiString m_opacity;
  };

  std::unordered_map<std::shared_ptr<Texture2D>, SeparatedTexturePath> collected_texture;

  for (int material_index = 0; material_index < materials.size(); material_index++) {
    aiMaterial* exporter_material = exporter_scene.mMaterials[material_index] = new aiMaterial();
    auto& material = materials.at(material_index);
    exporter_material->mNumProperties = 0;
    auto material_name = aiString(std::string("material_") + std::to_string(material_index));

    exporter_material->AddProperty(&material_name, AI_MATKEY_NAME);

    auto export_texture = [&](const std::shared_ptr<Texture2D>& texture, const std::string& title,
                              const bool split_opacity) {
      const auto search = collected_texture.find(texture);
      if (search != collected_texture.end()) {
        return search->second;
      }
      SeparatedTexturePath info{};
      if (texture->IsTemporary()) {
        const auto file_name = std::to_string(material_index) + "_" + title + ".png";
        info.color = aiString((std::filesystem::path("textures") / file_name).string());
        const auto succeed = texture->Export(texture_folder_path / file_name);
      } else {
        info.color = aiString((std::filesystem::path("textures") / texture->GetAbsolutePath().filename()).string());
        std::filesystem::copy(texture->GetAbsolutePath(), texture_folder_path / texture->GetAbsolutePath().filename(),
                              std::filesystem::copy_options::overwrite_existing);
      }
      if (split_opacity && texture->alpha_channel) {
        info.has_opacity = true;
        const auto opacity_title = std::to_string(material_index) + "_opacity.png";
        info.m_opacity = aiString((std::filesystem::path("textures") / opacity_title).string());
        std::vector<glm::vec4> data;
        texture->GetRgbaChannelData(data);
        std::vector<float> src(data.size() * 4);
        Jobs::RunParallelFor(data.size(), [&](size_t i) {
          src[i * 4] = data[i].a;
          src[i * 4 + 1] = data[i].a;
          src[i * 4 + 2] = data[i].a;
          src[i * 4 + 3] = data[i].a;
        });
        const auto resolution = texture->GetResolution();
        Texture2D::StoreToPng(texture_folder_path / opacity_title, src, resolution.x, resolution.y, 4, 4);
      }
      collected_texture[texture] = info;
      return info;
    };

    if (const auto base_color_texture = material->GetTexture(&GltfShadeMaterial::pbr_base_color_texture)) {
      const auto info = export_texture(base_color_texture, "diffuse", true);
      exporter_material->AddProperty(&info.color, AI_MATKEY_TEXTURE_DIFFUSE(0));
      if (info.has_opacity) {
        exporter_material->AddProperty(&info.m_opacity, AI_MATKEY_TEXTURE_OPACITY(0));
      }
    }
    if (const auto normal_texture = material->GetTexture(&GltfShadeMaterial::normal_texture)) {
      const auto info = export_texture(normal_texture, "normal", false);
      exporter_material->AddProperty(&info.color, AI_MATKEY_TEXTURE_NORMALS(0));
    }
    if (const auto metallic_roughness_texture =
            material->GetTexture(&GltfShadeMaterial::pbr_metallic_roughness_texture)) {
      const auto info = export_texture(metallic_roughness_texture, "metallic_roughness", false);
      exporter_material->AddProperty(&info.color, AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE_ROUGHNESS, 0));
      exporter_material->AddProperty(&info.color, AI_MATKEY_TEXTURE(aiTextureType_METALNESS, 0));
    }
    if (const auto emissive_texture = material->GetTexture(&GltfShadeMaterial::emissive_texture)) {
      const auto info = export_texture(emissive_texture, "emissive", false);
      exporter_material->AddProperty(&info.color, AI_MATKEY_TEXTURE_EMISSIVE(0));
    }
    if (const auto ao_texture = material->GetTexture(&GltfShadeMaterial::occlusion_texture)) {
      const auto info = export_texture(ao_texture, "ao", false);
      exporter_material->AddProperty(&info.color, AI_MATKEY_TEXTURE(aiTextureType_AMBIENT_OCCLUSION, 0));
    }
  }

  std::string format_id;
  if (path.extension().string() == ".obj") {
    format_id = "obj";
  } else if (path.extension().string() == ".fbx") {
    format_id = "fbx";
  } else if (path.extension().string() == ".gltf") {
    format_id = "gltf";
  } else if (path.extension().string() == ".dae") {
    format_id = "dae";
  }

  root_node.Process(exporter_scene.mRootNode);
  exporter.Export(&exporter_scene, format_id.c_str(), path.string());

  return true;
}

std::shared_ptr<Texture2D> Prefab::GenerateThumbnailTexture() {
  return EditorLayer::FindIcon("Prefab");
}

#pragma endregion

Entity Prefab::ToEntity(const std::shared_ptr<Scene>& scene, bool rescale, bool recenter) const {
  std::unordered_map<Handle, Handle> entity_map;
  std::vector<DataComponentType> types;
  types.reserve(data_components.size());
  for (auto& i : data_components) {
    types.emplace_back(i.data_component_type);
  }
  auto archetype = Entities::CreateEntityArchetype("", types);
  const Entity entity = scene->CreateEntity(archetype, instance_name);
  entity_map[entity_handle] = scene->GetEntityHandle(entity);
  scene->SetEnable(entity, enabled_);
  for (auto& i : data_components) {
    scene->SetDataComponent(entity.GetIndex(), i.data_component_type.type_index, i.data_component_type.type_size,
                            i.data_component.get());
  }
  int index = 0;
  for (const auto& i : child_prefabs) {
    AttachChildren(scene, i, entity, entity_map);
    index++;
  }

  for (auto& i : private_components) {
    size_t id;
    auto ptr = std::static_pointer_cast<IPrivateComponent>(
        Serialization::ProduceSerializable(i.private_component->GetTypeName(), id));
    Serialization::ClonePrivateComponent(ptr, i.private_component);
    ptr->handle_ = Handle();
    ptr->scene_ = scene;
    scene->SetPrivateComponent(entity, ptr);
  }
  for (const auto& i : child_prefabs) {
    AttachChildrenPrivateComponent(scene, i, entity, entity_map);
    index++;
  }

  RelinkChildren(scene, entity, entity_map);

  TransformGraph::CalculateTransformGraphForDescendants(scene, entity);

  if (rescale || recenter) {
    const auto adjusted_transform = CalculateAdjustedTransform(rescale, recenter);
    scene->SetDataComponent(entity, adjusted_transform);
    TransformGraph::CalculateTransformGraphForDescendants(scene, entity);
  }
  return entity;
}
Transform Prefab::CalculateAdjustedTransform(const bool rescale, const bool recenter) const {
  Transform ret_val{};
  if (rescale || recenter) {
    const auto bound = GetBoundingBox();
    auto size = bound.Size();
    glm::vec3 scale = glm::vec3(1.f);
    if (rescale) {
      while (size.x > 10.f || size.y > 10.f || size.z > 10.f) {
        scale /= 2.f;
        size /= 2.f;
      }
    }
    if (recenter) {
      ret_val.SetPosition(-bound.Center() * scale);
    }
    if (rescale) {
      ret_val.SetScale(scale);
    }
  }
  return ret_val;
}

void DataComponentHolder::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "tn" << YAML::Value << data_component_type.type_name;
  out << YAML::Key << "dc" << YAML::Value
      << YAML::Binary((const unsigned char*)data_component.get(), data_component_type.type_size);
}
bool DataComponentHolder::Deserialize(const YAML::Node& in) {
  data_component_type.type_name = in["tn"].as<std::string>();
  if (!Serialization::HasComponentDataType(data_component_type.type_name))
    return false;
  data_component = Serialization::ProduceDataComponent(data_component_type.type_name, data_component_type.type_index,
                                                       data_component_type.type_size);
  if (in["dc"]) {
    YAML::Binary data = in["dc"].as<YAML::Binary>();
    std::memcpy(data_component.get(), data.data(), data.size());
  }
  return true;
}
void Prefab::CollectAssets(std::unordered_map<Handle, std::shared_ptr<IAsset>>& map) const {
  std::vector<AssetRef> list;
  for (auto& i : private_components) {
    Serialization::CollectAssetRefs(*i.private_component, list);
  }
  for (auto& i : list) {
    auto asset = i.Get<IAsset>();
    if (asset && !Resources::IsResource(asset) && asset->IsTemporary()) {
      map[asset->GetHandle()] = asset;
    }
  }
  bool list_check = true;
  while (list_check) {
    const size_t current_size = map.size();
    list.clear();
    for (auto& i : map) {
      Serialization::CollectAssetRefs(*i.second, list);
    }
    for (auto& i : list) {
      auto asset = i.Get<IAsset>();
      if (asset && !Resources::IsResource(asset) && asset->IsTemporary()) {
        map[asset->GetHandle()] = asset;
      }
    }
    if (map.size() == current_size)
      list_check = false;
  }
  for (auto& i : child_prefabs)
    i->CollectAssets(map);
}
bool Prefab::SaveInternal(const std::filesystem::path& path) const {
  if (path.extension() == ".eveprefab") {
    auto directory = path;
    directory.remove_filename();
    std::filesystem::create_directories(directory);
    YAML::Emitter out;
    out << YAML::BeginMap;
    Serialization::SerializeObject(out, *this);
    std::unordered_map<Handle, std::shared_ptr<IAsset>> asset_map;
    CollectAssets(asset_map);
    std::vector<AssetRef> list;
    bool list_check = true;
    while (list_check) {
      const size_t current_size = asset_map.size();
      list.clear();
      for (const auto& i : asset_map) {
        Serialization::CollectAssetRefs(*i.second, list);
      }
      for (auto& i : list) {
        if (const auto asset = i.Get<IAsset>(); asset && !Resources::IsResource(asset->GetHandle())) {
          if (asset->IsTemporary()) {
            asset_map[asset->GetHandle()] = asset;
          } else if (!asset->Saved()) {
            asset->Save();
          }
        }
      }
      if (asset_map.size() == current_size)
        list_check = false;
    }

    if (!asset_map.empty()) {
      out << YAML::Key << "LocalAssets" << YAML::Value << YAML::BeginSeq;
      for (auto& i : asset_map) {
        out << YAML::BeginMap;
        out << YAML::Key << "TypeName" << YAML::Value << i.second->GetTypeName();
        out << YAML::Key << "Handle" << YAML::Value << i.first.GetValue();
        Serialization::SerializeObject(out, *i.second);
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
    }
    out << YAML::EndMap;

    std::ofstream fout(path.string());
    fout << out.c_str();
    fout.flush();
    return true;
  }

  return SaveModelInternal(path);
}
void Prefab::RelinkChildren(const std::shared_ptr<Scene>& scene, const Entity& parent_entity,
                            const std::unordered_map<Handle, Handle>& map) {
  scene->ForEachPrivateComponent(parent_entity, [&](PrivateComponentElement& data) {
    Serialization::RelinkObject(*data.private_component_data, map, scene);
  });
  scene->ForEachChild(parent_entity, [&](Entity child) {
    RelinkChildren(scene, child, map);
  });
}

void Prefab::LoadModel(const std::filesystem::path& path, const bool optimize, const unsigned flags) {
  LoadModelInternal(ProjectManager::GetAssetsFolderPath() / path, optimize, flags);
}

void Prefab::GatherAssets() {
  collected_assets.clear();
  for (const auto& components : private_components) {
    std::vector<AssetRef> asset_refs;
    Serialization::CollectAssetRefs(*components.private_component, asset_refs);
    for (const auto& asset_ref : asset_refs)
      collected_assets[asset_ref.GetAssetHandle()] = asset_ref;
  }

  for (const auto& child : child_prefabs)
    GatherAssetsWalker(child, collected_assets);
}

void Prefab::GatherAssetsWalker(const std::shared_ptr<Prefab>& walker, std::unordered_map<Handle, AssetRef>& assets) {
  for (const auto& i : walker->private_components) {
    std::vector<AssetRef> asset_refs;
    Serialization::CollectAssetRefs(*i.private_component, asset_refs);
    for (const auto& asset_ref : asset_refs)
      assets[asset_ref.GetAssetHandle()] = asset_ref;
  }

  for (const auto& child : walker->child_prefabs)
    GatherAssetsWalker(child, assets);
}

void PrivateComponentHolder::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "e" << YAML::Value << enabled;
  if (const auto unknown_component = std::dynamic_pointer_cast<UnknownPrivateComponent>(private_component)) {
    out << YAML::Key << "tn" << YAML::Value << unknown_component->GetOriginalTypeName();
  } else {
    out << YAML::Key << "tn" << YAML::Value << private_component->GetTypeName();
  }
  out << YAML::Key << "h" << private_component->GetHandle().GetValue();
  out << YAML::Key << "pc" << YAML::BeginMap;
  Serialization::SerializeObject(out, *private_component);
  out << YAML::EndMap;
}
void PrivateComponentHolder::Deserialize(const YAML::Node& in) {
  enabled = in["e"].as<bool>();
  const auto type_name = in["tn"].as<std::string>();
  const auto handle = Handle(in["h"].as<uint64_t>());
  const auto& in_data = in["pc"];
  if (Serialization::HasSerializableType(type_name)) {
    size_t hash_code;
    private_component =
        std::dynamic_pointer_cast<IPrivateComponent>(Serialization::ProduceSerializable(type_name, hash_code, handle));
  } else {
    size_t hash_code;
    private_component = std::dynamic_pointer_cast<IPrivateComponent>(
        Serialization::ProduceSerializable("UnknownPrivateComponent", hash_code, handle));
    if (auto unknown_component = std::dynamic_pointer_cast<UnknownPrivateComponent>(private_component)) {
      unknown_component->SetOriginalTypeName(type_name);
      unknown_component->SetSerializedNode(in_data);
    }
  }
  private_component->OnCreate();
  Serialization::DeserializeObject(in_data, *private_component);
}
