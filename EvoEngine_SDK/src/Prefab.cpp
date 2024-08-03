#include "Prefab.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"
#include "ProjectManager.hpp"
#include "Resources.hpp"
#include "Serialization.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "TransformGraph.hpp"
#include "UnknownPrivateComponent.hpp"
#include "Utilities.hpp"
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
std::shared_ptr<Texture2D> CollectTexture(
    const std::string& directory, const std::string& path,
    std::unordered_map<std::string, std::shared_ptr<Texture2D>>& loaded_textures) {
  const auto full_path_str = directory + "\\" + path;
  std::string full_path = std::filesystem::absolute(std::filesystem::path(full_path_str)).string();
  if (!std::filesystem::exists(full_path)) {
    full_path = std::filesystem::absolute(path).string();
  }
  if (!std::filesystem::exists(full_path)) {
    full_path = std::filesystem::absolute(directory + "\\" + std::filesystem::path(path).filename().string()).string();
  }

  if (!std::filesystem::exists(full_path)) {
    const auto base_dir = std::filesystem::absolute(directory);
    full_path =
        std::filesystem::absolute(base_dir.parent_path() / std::filesystem::path(path).filename().string()).string();
  }
  if (!std::filesystem::exists(full_path)) {
    const auto base_dir = std::filesystem::absolute(directory);
    full_path = std::filesystem::absolute(base_dir.parent_path().parent_path() / "textures" /
                                          std::filesystem::path(path).filename().string())
                    .string();
  }
  if (!std::filesystem::exists(full_path)) {
    const auto base_dir = std::filesystem::absolute(directory);
    full_path = std::filesystem::absolute(base_dir.parent_path().parent_path() / "texture" /
                                          std::filesystem::path(path).filename().string())
                    .string();
  }
  if (!std::filesystem::exists(full_path)) {
    return Resources::GetResource<Texture2D>("TEXTURE_MISSING");
  }
  if (const auto search = loaded_textures.find(full_path); search != loaded_textures.end()) {
    return search->second;
  }
  std::shared_ptr<Texture2D> texture_2d;
  if (ProjectManager::IsInProjectFolder(full_path)) {
    texture_2d = std::dynamic_pointer_cast<Texture2D>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(full_path)));
  } else {
    texture_2d = ProjectManager::CreateTemporaryAsset<Texture2D>();
    texture_2d->Import(full_path);
  }
  loaded_textures[full_path] = texture_2d;
  return texture_2d;
}
auto ReadMaterial(const std::string& directory,
                  std::unordered_map<std::string, std::shared_ptr<Texture2D>>& loaded_textures,
                  std::vector<std::pair<std::shared_ptr<Texture2D>, std::shared_ptr<Texture2D>>>& opacity_maps,
                  const aiMaterial* importer_material) -> std::shared_ptr<Material> {
  auto target_material = ProjectManager::CreateTemporaryAsset<Material>();
  if (importer_material) {
    // PBR
    if (importer_material->GetTextureCount(aiTextureType_BASE_COLOR) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_BASE_COLOR, 0, &str);
      target_material->SetAlbedoTexture(CollectTexture(directory, str.C_Str(), loaded_textures));
    }
    if (importer_material->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_DIFFUSE, 0, &str);
      target_material->SetAlbedoTexture(CollectTexture(directory, str.C_Str(), loaded_textures));
    }
    if (importer_material->GetTextureCount(aiTextureType_NORMALS) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_NORMALS, 0, &str);
      target_material->SetNormalTexture(CollectTexture(directory, str.C_Str(), loaded_textures));
    } else if (importer_material->GetTextureCount(aiTextureType_HEIGHT) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_HEIGHT, 0, &str);
      target_material->SetNormalTexture(CollectTexture(directory, str.C_Str(), loaded_textures));
    } else if (importer_material->GetTextureCount(aiTextureType_NORMAL_CAMERA) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_NORMAL_CAMERA, 0, &str);
      target_material->SetNormalTexture(CollectTexture(directory, str.C_Str(), loaded_textures));
    }

    if (importer_material->GetTextureCount(aiTextureType_METALNESS) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_METALNESS, 0, &str);
      target_material->SetMetallicTexture(CollectTexture(directory, str.C_Str(), loaded_textures));
    }
    if (importer_material->GetTextureCount(aiTextureType_DIFFUSE_ROUGHNESS) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_DIFFUSE_ROUGHNESS, 0, &str);
      target_material->SetRoughnessTexture(CollectTexture(directory, str.C_Str(), loaded_textures));
    }
    if (importer_material->GetTextureCount(aiTextureType_AMBIENT_OCCLUSION) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_AMBIENT_OCCLUSION, 0, &str);
      target_material->SetAoTexture(CollectTexture(directory, str.C_Str(), loaded_textures));
    }
    if (importer_material->GetTextureCount(aiTextureType_OPACITY) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_OPACITY, 0, &str);
      const auto opacity_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
      const auto albedo_texture = target_material->GetAlbedoTexture();

      opacity_maps.emplace_back(albedo_texture, opacity_texture);
    }
    if (importer_material->GetTextureCount(aiTextureType_TRANSMISSION) > 0) {
      aiString str;
      importer_material->GetTexture(aiTextureType_TRANSMISSION, 0, &str);
      const auto opacity_texture = CollectTexture(directory, str.C_Str(), loaded_textures);
      const auto albedo_texture = target_material->GetAlbedoTexture();

      opacity_maps.emplace_back(albedo_texture, opacity_texture);
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
      target_material->material_properties.albedo_color = glm::vec3(color.r, color.g, color.b);
    } else if (importer_material->Get(AI_MATKEY_BASE_COLOR, color) == aiReturn_SUCCESS) {
      target_material->material_properties.albedo_color = glm::vec3(color.r, color.g, color.b);
    }
    ai_real factor;
    if (importer_material->Get(AI_MATKEY_METALLIC_FACTOR, factor) == aiReturn_SUCCESS) {
      target_material->material_properties.metallic = factor;
    }
    if (importer_material->Get(AI_MATKEY_ROUGHNESS_FACTOR, factor) == aiReturn_SUCCESS) {
      target_material->material_properties.roughness = factor;
    }
    if (importer_material->Get(AI_MATKEY_SPECULAR_FACTOR, factor) == aiReturn_SUCCESS) {
      target_material->material_properties.specular = factor;
    }
  }
  return target_material;
}
std::shared_ptr<Mesh> ReadMesh(aiMesh* importer_mesh) {
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
      attributes.tangent = true;
    } else {
      attributes.tangent = false;
    }
    if (importer_mesh->HasVertexColors(0)) {
      v3.x = importer_mesh->mColors[0][i].r;
      v3.y = importer_mesh->mColors[0][i].g;
      v3.z = importer_mesh->mColors[0][i].b;
      vertex.color = glm::vec4(v3, 1.0f);
      attributes.color = true;
    } else {
      attributes.color = false;
    }
    if (importer_mesh->HasTextureCoords(0)) {
      glm::vec2 v2;
      v2.x = importer_mesh->mTextureCoords[0][i].x;
      v2.y = importer_mesh->mTextureCoords[0][i].y;
      vertex.tex_coord = v2;
      attributes.tex_coord = true;
    } else {
      vertex.tex_coord = glm::vec2(0.0f, 0.0f);
      attributes.color = false;
    }
    vertices[i] = vertex;
  }
  // now walk through each of the mesh's _Faces (a face is a mesh its triangle) and retrieve the corresponding vertex
  // indices.
  for (int i = 0; i < importer_mesh->mNumFaces; i++) {
    assert(importer_mesh->mFaces[i].mNumIndices == 3);
    // retrieve all indices of the face and store them in the indices vector
    for (int j = 0; j < 3; j++)
      indices.push_back(importer_mesh->mFaces[i].mIndices[j]);
  }
  auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
  mesh->SetVertices(attributes, vertices, indices);
  return mesh;
}
std::shared_ptr<SkinnedMesh> ReadSkinnedMesh(
    std::unordered_map<Handle, std::vector<std::shared_ptr<Bone>>>& bones_lists,
    std::unordered_map<std::string, std::shared_ptr<Bone>>& bones_map, aiMesh* importer_mesh) {
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
      skinned_vertex_attributes.tangent = true;
    }
    if (importer_mesh->HasVertexColors(0)) {
      v3.x = importer_mesh->mColors[0][i].r;
      v3.y = importer_mesh->mColors[0][i].g;
      v3.z = importer_mesh->mColors[0][i].b;
      vertex.color = glm::vec4(v3, 1.0f);
      skinned_vertex_attributes.color = true;
    }
    glm::vec2 v2;
    if (importer_mesh->HasTextureCoords(0)) {
      v2.x = importer_mesh->mTextureCoords[0][i].x;
      v2.y = importer_mesh->mTextureCoords[0][i].y;
      vertex.tex_coord = v2;
      skinned_vertex_attributes.tex_coord = true;
    } else {
      vertex.tex_coord = glm::vec2(0.0f, 0.0f);
      skinned_vertex_attributes.tex_coord = true;
    }
    vertices[i] = vertex;
  }
  // now walk through each of the mesh's _Faces (a face is a mesh its triangle) and retrieve the corresponding vertex
  // indices.
  for (int i = 0; i < importer_mesh->mNumFaces; i++) {
    assert(importer_mesh->mFaces[i].mNumIndices == 3);
    // retrieve all indices of the face and store them in the indices vector
    for (int j = 0; j < 3; j++)
      indices.push_back(importer_mesh->mFaces[i].mIndices[j]);
  }
  auto skinned_mesh = ProjectManager::CreateTemporaryAsset<SkinnedMesh>();
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
  skinned_mesh->SetVertices(skinned_vertex_attributes, vertices, indices);
  return skinned_mesh;
}

auto ProcessNode(const std::string& directory, Prefab* model_node,
                 std::unordered_map<unsigned, std::shared_ptr<Material>>& loaded_materials,
                 std::unordered_map<std::string, std::shared_ptr<Texture2D>>& texture_2ds_loaded,
                 std::vector<std::pair<std::shared_ptr<Texture2D>, std::shared_ptr<Texture2D>>>& opacity_maps,
                 std::unordered_map<Handle, std::vector<std::shared_ptr<Bone>>>& bones_lists,
                 std::unordered_map<std::string, std::shared_ptr<Bone>>& bones_map, const aiNode* importer_node,
                 const std::shared_ptr<AssimpImportNode>& assimp_node, const aiScene* importer_scene,
                 const std::shared_ptr<Animation>& animation) -> bool {
  bool added_mesh_renderer = false;
  for (unsigned i = 0; i < importer_node->mNumMeshes; i++) {
    // the modelNode object only contains indices to index the actual objects in the scene.
    // the scene contains all the data, modelNode is just to keep stuff organized (like relations between nodes).
    aiMesh* importer_mesh = importer_scene->mMeshes[importer_node->mMeshes[i]];
    if (!importer_mesh)
      continue;
    auto child_node = ProjectManager::CreateTemporaryAsset<Prefab>();
    child_node->instance_name = std::string(importer_mesh->mName.C_Str());
    const auto search = loaded_materials.find(importer_mesh->mMaterialIndex);
    const bool is_skinned_mesh = importer_mesh->mNumBones != 0xffffffff && importer_mesh->mBones;
    std::shared_ptr<Material> material;
    if (search == loaded_materials.end()) {
      const aiMaterial* importer_material = nullptr;
      if (importer_mesh->mMaterialIndex != 0xffffffff && importer_mesh->mMaterialIndex < importer_scene->mNumMaterials)
        importer_material = importer_scene->mMaterials[importer_mesh->mMaterialIndex];
      material = ReadMaterial(directory, texture_2ds_loaded, opacity_maps, importer_material);
      loaded_materials[importer_mesh->mMaterialIndex] = material;
    } else {
      material = search->second;
    }

    if (is_skinned_mesh) {
      auto skinned_mesh_renderer = Serialization::ProduceSerializable<SkinnedMeshRenderer>();
      skinned_mesh_renderer->material.Set<Material>(material);
      skinned_mesh_renderer->skinned_mesh.Set<SkinnedMesh>(ReadSkinnedMesh(bones_lists, bones_map, importer_mesh));
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
      mesh_renderer->mesh.Set<Mesh>(ReadMesh(importer_mesh));
      if (!mesh_renderer->mesh.Get())
        continue;
      added_mesh_renderer = true;
      PrivateComponentHolder holder;
      holder.enabled = true;
      holder.private_component = std::static_pointer_cast<IPrivateComponent>(mesh_renderer);
      child_node->private_components.push_back(holder);
    }
    auto transform = std::make_shared<Transform>();
    transform->value = Mat4Cast(importer_node->mTransformation);
    if (!importer_node->mParent)
      transform->value = Transform().value;

    DataComponentHolder holder;
    holder.data_component_type = Typeof<Transform>();
    holder.data_component = transform;
    child_node->data_components.push_back(holder);

    model_node->child_prefabs.push_back(std::move(child_node));
  }

  for (unsigned i = 0; i < importer_node->mNumChildren; i++) {
    auto child_node = ProjectManager::CreateTemporaryAsset<Prefab>();
    child_node->instance_name = std::string(importer_node->mChildren[i]->mName.C_Str());
    auto child_assimp_node = std::make_shared<AssimpImportNode>(importer_node->mChildren[i]);
    child_assimp_node->parent_node = assimp_node;
    const bool child_add =
        ProcessNode(directory, child_node.get(), loaded_materials, texture_2ds_loaded, opacity_maps, bones_lists,
                    bones_map, importer_node->mChildren[i], child_assimp_node, importer_scene, animation);
    if (child_add) {
      model_node->child_prefabs.push_back(std::move(child_node));
    }
    added_mesh_renderer = added_mesh_renderer | child_add;
    assimp_node->child_nodes.push_back(std::move(child_assimp_node));
  }
  return added_mesh_renderer;
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
  scene->SetEnable(entity, enabled_);
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
  const auto scene = Application::GetActiveScene();
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
    auto temp = ProjectManager::CreateTemporaryAsset<Prefab>();
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
        local_assets.push_back(ProjectManager::CreateTemporaryAsset(i["TypeName"].as<std::string>(), handle));
      }
      int index = 0;
      for (const auto& i : in_local_assets) {
        local_assets[index++]->Deserialize(i);
      }
    }

#pragma endregion
    Deserialize(in);
    return true;
  }
  return LoadModelInternal(path);
}
bool Prefab::LoadModelInternal(const std::filesystem::path& path, bool optimize, unsigned int flags) {
  flags = flags | aiProcess_Triangulate;
  if (optimize) {
    flags = flags | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes;
  }
  // read file via ASSIMP
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(path.string(), flags);
  // check for errors
  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)  // if is Not Zero
  {
    EVOENGINE_LOG("Assimp: " + std::string(importer.GetErrorString()));
    return false;
  }
  // retrieve the directory path of the filepath
  auto temp = path;
  const std::string directory = temp.remove_filename().string();
  instance_name = path.filename().string();
  std::unordered_map<unsigned, std::shared_ptr<Material>> loaded_materials;
  std::vector<std::pair<std::shared_ptr<Texture2D>, std::shared_ptr<Texture2D>>> opacity_maps;
  std::unordered_map<std::string, std::shared_ptr<Bone>> bones_map;
  std::shared_ptr<Animation> animation;
  if (!bones_map.empty() || scene->HasAnimations()) {
    animation = ProjectManager::CreateTemporaryAsset<Animation>();
  }
  std::shared_ptr<AssimpImportNode> root_assimp_node = std::make_shared<AssimpImportNode>(scene->mRootNode);

  std::unordered_map<Handle, std::vector<std::shared_ptr<Bone>>> bones_lists;
  if (std::unordered_map<std::string, std::shared_ptr<Texture2D>> loaded_textures;
      !ProcessNode(directory, this, loaded_materials, loaded_textures, opacity_maps, bones_lists, bones_map,
                   scene->mRootNode, root_assimp_node, scene, animation)) {
    EVOENGINE_ERROR("Model is empty!")
    return false;
  }

  for (auto& pair : opacity_maps) {
    std::vector<glm::vec4> color_data;
    const auto& albedo_texture = pair.first;
    const auto& opacity_texture = pair.second;
    if (!albedo_texture || !opacity_texture)
      continue;
    albedo_texture->GetRgbaChannelData(color_data);
    std::vector<glm::vec4> alpha_data;
    const auto resolution = albedo_texture->GetResolution();
    opacity_texture->GetRgbaChannelData(alpha_data, resolution.x, resolution.y);
    Jobs::RunParallelFor(color_data.size(), [&](unsigned i) {
      color_data[i].a = alpha_data[i].r;
    });
    std::shared_ptr<Texture2D> replacement_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
    replacement_texture->SetRgbaChannelData(color_data, albedo_texture->GetResolution(), true);
    pair.second = replacement_texture;
  }

  for (const auto& material : loaded_materials) {
    const auto albedo_texture = material.second->GetAlbedoTexture();
    if (!albedo_texture)
      continue;
    for (const auto& pair : opacity_maps) {
      if (albedo_texture->GetHandle() == pair.first->GetHandle()) {
        material.second->SetAlbedoTexture(pair.second);
      }
    }
  }

  if (!bones_map.empty() || scene->HasAnimations()) {
    root_assimp_node->NecessaryWalker(bones_map);
    size_t index = 0;
    root_assimp_node->AttachToAnimator(animation, index);
    animation->bone_size = index + 1;
    ReadAnimations(scene, animation, bones_map);
    ApplyBoneIndices(bones_lists, this);

    auto animator = Serialization::ProduceSerializable<Animator>();
    animator->Setup(animation);
    AttachAnimator(this, entity_handle);
    PrivateComponentHolder holder;
    holder.enabled = true;
    holder.private_component = std::static_pointer_cast<IPrivateComponent>(animator);
    private_components.push_back(holder);
  }
  GatherAssets();
  return true;
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

    if (const auto albedo_texture = material->GetAlbedoTexture()) {
      const auto search = collected_texture.find(albedo_texture);
      SeparatedTexturePath info{};
      if (search != collected_texture.end()) {
        info = search->second;
      } else {
        if (albedo_texture->IsTemporary()) {
          std::string diffuse_title = std::to_string(material_index) + "_diffuse.png";
          info.color = aiString((std::filesystem::path("textures") / diffuse_title).string());
          const auto succeed = albedo_texture->Export(texture_folder_path / diffuse_title);
        } else {
          info.color =
              aiString((std::filesystem::path("textures") / albedo_texture->GetAbsolutePath().filename()).string());
          std::filesystem::copy(albedo_texture->GetAbsolutePath(),
                                texture_folder_path / albedo_texture->GetAbsolutePath().filename(),
                                std::filesystem::copy_options::overwrite_existing);
        }
        if (albedo_texture->alpha_channel) {
          info.has_opacity = true;
          std::string opacity_title = std::to_string(material_index) + "_opacity.png";
          info.m_opacity = aiString((std::filesystem::path("textures") / opacity_title).string());
          std::vector<glm::vec4> data;
          albedo_texture->GetRgbaChannelData(data);
          std::vector<float> src(data.size() * 4);
          Jobs::RunParallelFor(data.size(), [&](const unsigned i) {
            src[i * 4] = data[i].a;
            src[i * 4 + 1] = data[i].a;
            src[i * 4 + 2] = data[i].a;
            src[i * 4 + 3] = data[i].a;
          });
          auto resolution = albedo_texture->GetResolution();
          Texture2D::StoreToPng(texture_folder_path / opacity_title, src, resolution.x, resolution.y, 4, 4);
        }
      }

      exporter_material->AddProperty(&info.color, AI_MATKEY_TEXTURE_DIFFUSE(0));
      if (info.has_opacity) {
        exporter_material->AddProperty(&info.m_opacity, AI_MATKEY_TEXTURE_OPACITY(0));
      }
    }

    if (const auto normal_texture = material->GetNormalTexture()) {
      const auto search = collected_texture.find(normal_texture);
      SeparatedTexturePath info{};
      if (search != collected_texture.end()) {
        info = search->second;
      } else {
        if (normal_texture->IsTemporary()) {
          std::string title = std::to_string(material_index) + "_normal.png";
          info.color = aiString((std::filesystem::path("textures") / title).string());
          const auto succeed = normal_texture->Export(texture_folder_path / title);
        } else {
          info.color =
              aiString((std::filesystem::path("textures") / normal_texture->GetAbsolutePath().filename()).string());
          std::filesystem::copy(normal_texture->GetAbsolutePath(),
                                texture_folder_path / normal_texture->GetAbsolutePath().filename(),
                                std::filesystem::copy_options::overwrite_existing);
        }
      }

      exporter_material->AddProperty(&info.color, AI_MATKEY_TEXTURE_NORMALS(0));
    }
    if (const auto metallic_texture = material->GetMetallicTexture()) {
      const auto search = collected_texture.find(metallic_texture);
      SeparatedTexturePath info{};
      if (search != collected_texture.end()) {
        info = search->second;
      } else {
        if (metallic_texture->IsTemporary()) {
          std::string title = std::to_string(material_index) + "_metallic.png";
          info.color = aiString((std::filesystem::path("textures") / title).string());
          const auto succeed = metallic_texture->Export(texture_folder_path / title);
        } else {
          info.color =
              aiString((std::filesystem::path("textures") / metallic_texture->GetAbsolutePath().filename()).string());
          std::filesystem::copy(metallic_texture->GetAbsolutePath(),
                                texture_folder_path / metallic_texture->GetAbsolutePath().filename(),
                                std::filesystem::copy_options::overwrite_existing);
        }
      }
      exporter_material->AddProperty(&info.color, AI_MATKEY_TEXTURE_SHININESS(0));
    }
    if (const auto roughness_texture = material->GetRoughnessTexture()) {
      const auto search = collected_texture.find(roughness_texture);
      SeparatedTexturePath info{};
      if (search != collected_texture.end()) {
        info = search->second;
      } else {
        if (roughness_texture->IsTemporary()) {
          std::string title = std::to_string(material_index) + "_roughness.png";
          info.color = aiString((std::filesystem::path("textures") / title).string());
          const auto succeed = roughness_texture->Export(texture_folder_path / title);
        } else {
          info.color =
              aiString((std::filesystem::path("textures") / roughness_texture->GetAbsolutePath().filename()).string());
          std::filesystem::copy(roughness_texture->GetAbsolutePath(),
                                texture_folder_path / roughness_texture->GetAbsolutePath().filename(),
                                std::filesystem::copy_options::overwrite_existing);
        }
      }
      exporter_material->AddProperty(&info.color, AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE_ROUGHNESS, 0));
    }
    if (const auto ao_texture = material->GetAoTexture()) {
      const auto search = collected_texture.find(ao_texture);
      SeparatedTexturePath info{};
      if (search != collected_texture.end()) {
        info = search->second;
      } else {
        if (ao_texture->IsTemporary()) {
          std::string title = std::to_string(material_index) + "_ao.png";
          info.color = aiString((std::filesystem::path("textures") / title).string());
          const auto succeed = ao_texture->Export(texture_folder_path / title);
        } else {
          info.color =
              aiString((std::filesystem::path("textures") / ao_texture->GetAbsolutePath().filename()).string());
          std::filesystem::copy(ao_texture->GetAbsolutePath(),
                                texture_folder_path / ao_texture->GetAbsolutePath().filename(),
                                std::filesystem::copy_options::overwrite_existing);
        }
      }
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

  scene->SetEnable(entity, enabled_);

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
void Prefab::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "in" << YAML::Value << instance_name;
  out << YAML::Key << "e" << YAML::Value << enabled_;
  out << YAML::Key << "eh" << YAML::Value << entity_handle.GetValue();

  if (!data_components.empty()) {
    out << YAML::Key << "dc" << YAML::BeginSeq;
    for (auto& i : data_components) {
      out << YAML::BeginMap;
      i.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }

  if (!private_components.empty()) {
    out << YAML::Key << "pc" << YAML::BeginSeq;
    for (auto& i : private_components) {
      out << YAML::BeginMap;
      i.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }

  if (!child_prefabs.empty()) {
    out << YAML::Key << "c" << YAML::BeginSeq;
    for (auto& i : child_prefabs) {
      out << YAML::BeginMap;
      out << YAML::Key << "h" << i->GetHandle().GetValue();
      i->Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}
void Prefab::Deserialize(const YAML::Node& in) {
  instance_name = in["in"].as<std::string>();
  enabled_ = in["e"].as<bool>();
  entity_handle = Handle(in["eh"].as<uint64_t>());
  if (in["dc"]) {
    for (const auto& i : in["dc"]) {
      DataComponentHolder holder;
      if (holder.Deserialize(i)) {
        data_components.push_back(holder);
      }
    }
  }
  std::vector<std::pair<int, std::shared_ptr<IAsset>>> local_assets;
  if (const auto in_local_assets = in["LocalAssets"]) {
    int index = 0;
    for (const auto& i : in_local_assets) {
      // First, find the asset in asset registry
      if (const auto type_name = i["TypeName"].as<std::string>(); Serialization::HasSerializableType(type_name)) {
        auto asset = ProjectManager::CreateTemporaryAsset(type_name, i["Handle"].as<uint64_t>());
        local_assets.emplace_back(index, asset);
      }
      index++;
    }

    for (const auto& i : local_assets) {
      i.second->Deserialize(in_local_assets[i.first]);
    }
  }
#ifdef _DEBUG
  EVOENGINE_LOG(std::string("Prefab Deserialization: Loaded " + std::to_string(local_assets.size()) + " assets."))
#endif
  if (in["pc"]) {
    for (const auto& i : in["pc"]) {
      PrivateComponentHolder holder;
      holder.Deserialize(i);
      private_components.push_back(holder);
    }
  }

  if (in["c"]) {
    for (const auto& i : in["c"]) {
      auto child = ProjectManager::CreateTemporaryAsset<Prefab>();
      child->handle_ = i["h"].as<uint64_t>();
      child->Deserialize(i);
      child_prefabs.push_back(child);
    }
  }
}
void Prefab::CollectAssets(std::unordered_map<Handle, std::shared_ptr<IAsset>>& map) const {
  std::vector<AssetRef> list;
  for (auto& i : private_components) {
    i.private_component->CollectAssetRef(list);
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
      i.second->CollectAssetRef(list);
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
    Serialize(out);
    std::unordered_map<Handle, std::shared_ptr<IAsset>> asset_map;
    CollectAssets(asset_map);
    std::vector<AssetRef> list;
    bool list_check = true;
    while (list_check) {
      const size_t current_size = asset_map.size();
      list.clear();
      for (const auto& i : asset_map) {
        i.second->CollectAssetRef(list);
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
        i.second->Serialize(out);
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
    data.private_component_data->Relink(map, scene);
  });
  scene->ForEachChild(parent_entity, [&](Entity child) {
    RelinkChildren(scene, child, map);
  });
}

void Prefab::LoadModel(const std::filesystem::path& path, const bool optimize, const unsigned flags) {
  LoadModelInternal(ProjectManager::GetProjectPath().parent_path() / path, optimize, flags);
}

void Prefab::GatherAssets() {
  collected_assets.clear();
  for (const auto& components : private_components) {
    std::vector<AssetRef> asset_refs;
    components.private_component->CollectAssetRef(asset_refs);
    for (const auto& asset_ref : asset_refs)
      collected_assets[asset_ref.GetAssetHandle()] = asset_ref;
  }

  for (const auto& child : child_prefabs)
    GatherAssetsWalker(child, collected_assets);
}

bool Prefab::OnInspectWalker(const std::shared_ptr<Prefab>& walker) {
  bool changed = false;
  ImGui::Text(("Name: " + walker->instance_name).c_str());

  if (OnInspectComponents(walker))
    changed = true;

  if (!walker->child_prefabs.empty()) {
    if (ImGui::TreeNode("Children")) {
      for (const auto& child : walker->child_prefabs) {
        if (OnInspectWalker(child))
          changed = true;
      }
      ImGui::TreePop();
    }
  }
  return changed;
}

void Prefab::GatherAssetsWalker(const std::shared_ptr<Prefab>& walker, std::unordered_map<Handle, AssetRef>& assets) {
  for (const auto& i : walker->private_components) {
    std::vector<AssetRef> asset_refs;
    i.private_component->CollectAssetRef(asset_refs);
    for (const auto& asset_ref : asset_refs)
      assets[asset_ref.GetAssetHandle()] = asset_ref;
  }

  for (const auto& child : walker->child_prefabs)
    GatherAssetsWalker(child, assets);
}

bool Prefab::OnInspectComponents(const std::shared_ptr<Prefab>& walker) {
  bool changed = false;
  if (ImGui::TreeNode("Data Components")) {
    for (auto& i : walker->data_components) {
      ImGui::Text(("Type: " + i.data_component_type.type_name).c_str());
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Private Components")) {
    for (auto& i : walker->private_components) {
      ImGui::Text(("Type: " + i.private_component->GetTypeName()).c_str());
    }
    ImGui::TreePop();
  }
  return changed;
}
bool Prefab::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Button("Instantiate")) {
    ToEntity(Application::GetActiveScene());
  }
  if (collected_assets.empty() && ImGui::Button("Collect assets"))
    GatherAssets();
  if (!collected_assets.empty()) {
    if (ImGui::TreeNode("Assets")) {
      for (auto& i : collected_assets) {
        const auto ptr = i.second.Get<IAsset>();
        const std::string tag = "##" + ptr->GetTypeName() + std::to_string(ptr->GetHandle());
        ImGui::Button((ptr->GetTitle() + tag).c_str());
        EditorLayer::DraggableAsset(ptr);
        EditorLayer::Rename(i.second);
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
          ProjectManager::GetInstance().inspecting_asset = ptr;
        }
      }
      ImGui::TreePop();
    }
  }

  if (ImGui::TreeNode("Prefab Hierarchy")) {
    ImGui::Text((instance_name + " (root)").c_str());
    if (OnInspectComponents(std::dynamic_pointer_cast<Prefab>(GetSelf())))
      changed = true;

    if (!child_prefabs.empty()) {
      if (ImGui::TreeNode("Children")) {
        for (const auto& child : child_prefabs) {
          if (OnInspectWalker(child))
            changed = true;
        }
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }
  return changed;
}

void PrivateComponentHolder::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "e" << YAML::Value << enabled;
  out << YAML::Key << "tn" << YAML::Value << private_component->GetTypeName();
  out << YAML::Key << "h" << private_component->GetHandle().GetValue();
  out << YAML::Key << "pc" << YAML::BeginMap;
  private_component->Serialize(out);
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
    std::dynamic_pointer_cast<UnknownPrivateComponent>(private_component)->original_type_name_ = type_name;
  }
  private_component->OnCreate();
  private_component->Deserialize(in_data);
}
