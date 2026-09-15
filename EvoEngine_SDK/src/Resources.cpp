#include "Resources.hpp"

#include "Cubemap.hpp"
#include "EnvironmentalMap.hpp"
#include "GeometryStorage.hpp"
#include "GlobalReflectionProbe.hpp"
#include "GpuService.hpp"
#include "PathUtils.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "RuntimePaths.hpp"
#include "Serialization.hpp"
#include "Shader.hpp"
#include "TextureStorage.hpp"
#include "Utilities.hpp"

#include <vector>
using namespace evo_engine;

std::filesystem::path Resources::GetDefaultResourcesPath() {
  if (runtime_paths::IsStrict()) {
    const auto path = runtime_paths::Resolve("DefaultResources");
    if (!std::filesystem::is_directory(path)) {
      throw std::runtime_error("Required DefaultResources directory is missing.");
    }
    return path;
  }
  std::vector<std::filesystem::path> candidates;
  if (const auto executable_path = path_utils::CurrentExecutablePath(); !executable_path.empty()) {
    candidates.emplace_back(executable_path.parent_path() / "DefaultResources");
  }
  candidates.emplace_back(std::filesystem::current_path() / "DefaultResources");
  candidates.emplace_back(std::filesystem::current_path() / "EvoEngine_SDK/Internals/DefaultResources");
  if (const auto default_resources = path_utils::FindExistingPath(candidates); !default_resources.empty()) {
    return default_resources;
  }
  return path_utils::NormalizeAbsolutePath("DefaultResources");
}

std::filesystem::path Resources::GetDefaultResourcePath(const std::filesystem::path& relative_path) {
  return GetDefaultResourcesPath() / relative_path;
}

const std::shared_ptr<Texture2D>& Resources::GetMissingTexture() const {
  return missing_texture_;
}

const std::shared_ptr<Cubemap>& Resources::GetDefaultSkybox() const {
  return default_skybox_;
}

const std::shared_ptr<EnvironmentalMap>& Resources::GetDefaultEnvironmentalMap() const {
  return default_environmental_map_;
}

const std::shared_ptr<GlobalReflectionProbe>& Resources::GetDefaultGlobalReflectionProbe() const {
  return default_global_reflection_probe_;
}

const std::shared_ptr<Mesh>& Resources::GetTexturePassThroughQuad() const {
  return texture_pass_through_quad_;
}

const std::shared_ptr<Mesh>& Resources::GetRenderingCube() const {
  return rendering_cube_;
}

const Resources::Primitives& Resources::GetPrimitives() const {
  return primitives_;
}

Resources::Primitives& Resources::GetPrimitives() {
  return primitives_;
}

void Resources::ClearPrimitives() {
  primitives_.quad.reset();
  primitives_.sphere.reset();
  primitives_.cube.reset();
  primitives_.cone.reset();
  primitives_.cylinder.reset();
  primitives_.torus.reset();
  primitives_.monkey.reset();
  primitives_.capsule.reset();
}

void Resources::LoadPrimitives() {
  const auto default_resources = GetDefaultResourcesPath();
  auto load_primitive = [&](std::shared_ptr<Mesh>& primitive, const std::filesystem::path& path) {
    primitive = CreateResource<Mesh>();
    Serialization::LoadAsset(*primitive, path);
  };

  load_primitive(primitives_.quad, default_resources / "Primitives/quad.evemesh");
  load_primitive(primitives_.sphere, default_resources / "Primitives/sphere.evemesh");
  load_primitive(primitives_.cube, default_resources / "Primitives/cube.evemesh");
  load_primitive(primitives_.cone, default_resources / "Primitives/cone.evemesh");
  load_primitive(primitives_.cylinder, default_resources / "Primitives/cylinder.evemesh");
  load_primitive(primitives_.torus, default_resources / "Primitives/torus.evemesh");
  load_primitive(primitives_.monkey, default_resources / "Primitives/monkey.evemesh");
  load_primitive(primitives_.capsule, default_resources / "Primitives/capsule.evemesh");
  {
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    Vertex vertex{};
    std::vector<Vertex> vertices;

    vertex.position = {-1, 1, 0};
    vertex.tex_coord = {0, 1};
    vertices.emplace_back(vertex);

    vertex.position = {1, 1, 0};
    vertex.tex_coord = {1, 1};
    vertices.emplace_back(vertex);

    vertex.position = {-1, -1, 0};
    vertex.tex_coord = {0, 0};
    vertices.emplace_back(vertex);

    vertex.position = {1, -1, 0};
    vertex.tex_coord = {1, 0};
    vertices.emplace_back(vertex);

    std::vector<glm::uvec3> triangles = {{0, 2, 3}, {0, 3, 1}};
    texture_pass_through_quad_ = CreateResource<Mesh>();
    texture_pass_through_quad_->SetVertices(attributes, vertices, triangles);
  }
  {
    VertexAttributes attributes{};
    Vertex vertex{};
    std::vector<Vertex> vertices;

    vertex.position = {-1, -1, -1};
    vertices.emplace_back(vertex);  // 0:

    vertex.position = {1, 1, -1};
    vertices.emplace_back(vertex);  // 1:

    vertex.position = {1, -1, -1};
    vertices.emplace_back(vertex);  // 2:

    vertex.position = {-1, 1, -1};
    vertices.emplace_back(vertex);  // 3:

    vertex.position = {-1, -1, 1};
    vertices.emplace_back(vertex);  // 4:

    vertex.position = {1, -1, 1};
    vertices.emplace_back(vertex);  // 5:

    vertex.position = {1, 1, 1};
    vertices.emplace_back(vertex);  // 6:

    vertex.position = {-1, 1, 1};
    vertices.emplace_back(vertex);  // 7:

    std::vector<glm::uvec3> triangles = {
        {0, 1, 2}, {1, 0, 3}, {4, 5, 6}, {6, 7, 4}, {7, 3, 0}, {0, 4, 7},
        {6, 2, 1}, {2, 6, 5}, {0, 2, 5}, {5, 4, 0}, {3, 6, 1}, {6, 3, 7},
    };
    rendering_cube_ = CreateResource<Mesh>();
    rendering_cube_->SetVertices(attributes, vertices, triangles);
  }
}

void Resources::Initialize() {
  auto& resources = GetInstance();
  resources.typed_resources_.clear();
  resources.resources_.clear();
  resources.current_max_handle_ = Handle(1);
  resources.LoadPrimitives();

  GeometryStorage::DeviceSync();
  GeometryStorage::WaitForPendingUploads();
  TextureStorage::DeviceSync();
  resources.missing_texture_ = CreateResource<Texture2D>();
  const auto default_resources = GetDefaultResourcesPath();
  Serialization::LoadAsset(*resources.missing_texture_, default_resources / "Textures/texture-missing.png");

  resources.default_environmental_map_texture_ = CreateResource<Texture2D>();
  Serialization::LoadAsset(*resources.default_environmental_map_texture_,
                           default_resources / "Textures/Cubemaps/GrandCanyon/GCanyon_C_YumaPoint_3k.hdr");

  resources.default_skybox_texture_ = CreateResource<Texture2D>();
  Serialization::LoadAsset(*resources.default_skybox_texture_,
                           default_resources / "Textures/Cubemaps/GrandCanyon/GCanyon_C_YumaPoint_Env.hdr");

  TextureStorage::DeviceSync();

  resources.default_skybox_ = CreateResource<Cubemap>();
  resources.default_skybox_->Initialize(256);
  resources.default_skybox_->ConvertFromEquirectangularTexture(resources.default_skybox_texture_);

  resources.default_environmental_map_ = CreateResource<EnvironmentalMap>();
  resources.default_environmental_map_->ConstructFromTexture2D(resources.default_environmental_map_texture_);
  resources.default_global_reflection_probe_ = CreateResource<GlobalReflectionProbe>();
  if (auto cubemap_ref = resources.default_environmental_map_->environment_cubemap;
      const auto environment_cubemap = cubemap_ref.Get<Cubemap>()) {
    resources.default_global_reflection_probe_->ConstructFromCubemap(environment_cubemap);
  }
}

Handle Resources::GenerateNewHandle() {
  return current_max_handle_.value_++;
}

bool Resources::IsResource(const Handle& handle) {
  auto& resources = GetInstance();
  return resources.resources_.find(handle) != resources.resources_.end();
}

bool Resources::IsResource(const std::shared_ptr<IAsset>& target) {
  auto& resources = GetInstance();
  return resources.resources_.find(target->GetHandle()) != resources.resources_.end();
}

bool Resources::IsResource(const AssetRef& target) {
  auto& resources = GetInstance();
  return resources.resources_.find(target.GetAssetHandle()) != resources.resources_.end();
}

void Resources::OnDestroy() {
  auto& resources = GetInstance();
  Platform::DrainGpuResourceWork();

  resources.typed_resources_.clear();
  resources.resources_.clear();

  resources.missing_texture_.reset();
  resources.default_environmental_map_texture_.reset();
  resources.default_skybox_texture_.reset();

  resources.default_skybox_.reset();
  resources.default_environmental_map_.reset();
  resources.default_global_reflection_probe_.reset();

  resources.texture_pass_through_quad_.reset();
  resources.rendering_cube_.reset();

  resources.ClearPrimitives();
}
