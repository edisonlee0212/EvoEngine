#include "Resources.hpp"

#include "Cubemap.hpp"
#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "GpuService.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"
#include "TextureStorage.hpp"
#include "Utilities.hpp"
using namespace evo_engine;

const std::shared_ptr<Texture2D>& Resources::GetMissingTexture() const {
  return missing_texture_;
}

const std::shared_ptr<Cubemap>& Resources::GetDefaultSkybox() const {
  return default_skybox_;
}

const std::shared_ptr<EnvironmentalMap>& Resources::GetDefaultEnvironmentalMap() const {
  return default_environmental_map_;
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
  {
    primitives_.quad = CreateResource<Mesh>();
    primitives_.quad->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/quad.evemesh");
  }
  {
    primitives_.sphere = CreateResource<Mesh>();
    primitives_.sphere->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/sphere.evemesh");
  }
  {
    primitives_.cube = CreateResource<Mesh>();
    primitives_.cube->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/cube.evemesh");
  }
  {
    primitives_.cone = CreateResource<Mesh>();
    primitives_.cone->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/cone.evemesh");
  }
  {
    primitives_.cylinder = CreateResource<Mesh>();
    primitives_.cylinder->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/cylinder.evemesh");
  }
  {
    primitives_.torus = CreateResource<Mesh>();
    primitives_.torus->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/torus.evemesh");
  }
  {
    primitives_.monkey = CreateResource<Mesh>();
    primitives_.monkey->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/monkey.evemesh");
  }
  {
    primitives_.capsule = CreateResource<Mesh>();
    primitives_.capsule->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/capsule.evemesh");
  }
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
  resources.missing_texture_->LoadInternal(std::filesystem::path("./DefaultResources") /
                                           "Textures/texture-missing.png");

  resources.default_environmental_map_texture_ = CreateResource<Texture2D>();
  resources.default_environmental_map_texture_->LoadInternal(
      std::filesystem::path("./DefaultResources") / "Textures/Cubemaps/GrandCanyon/GCanyon_C_YumaPoint_3k.hdr");

  resources.default_skybox_texture_ = CreateResource<Texture2D>();
  resources.default_skybox_texture_->LoadInternal(std::filesystem::path("./DefaultResources") /
                                                  "Textures/Cubemaps/GrandCanyon/GCanyon_C_YumaPoint_Env.hdr");

  TextureStorage::DeviceSync();

  resources.default_skybox_ = CreateResource<Cubemap>();
  resources.default_skybox_->Initialize(256);
  resources.default_skybox_->ConvertFromEquirectangularTexture(resources.default_skybox_texture_);

  resources.default_environmental_map_ = CreateResource<EnvironmentalMap>();
  resources.default_environmental_map_->ConstructFromTexture2D(resources.default_environmental_map_texture_);
}

Handle Resources::GenerateNewHandle() {
  return current_max_handle_.value_++;
}

void Resources::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& resources = GetInstance();
  if (resources.show_resources_) {
    if (ImGui::Begin("Resources")) {
      if (ImGui::CollapsingHeader("Textures")) {
        ImGui::Button("Missing");
        editor_layer->DraggableAsset<Texture2D>(resources.missing_texture_);
      }
      if (ImGui::CollapsingHeader("Cubemap")) {
        ImGui::Button("Default Skybox");
        editor_layer->DraggableAsset<Cubemap>(resources.default_skybox_);
      }
      if (ImGui::CollapsingHeader("Environmental Map")) {
        ImGui::Button("Default Env map");
        editor_layer->DraggableAsset<EnvironmentalMap>(resources.default_environmental_map_);
      }
      if (ImGui::CollapsingHeader("Primitives")) {
        ImGui::Button("Quad");
        editor_layer->DraggableAsset<Mesh>(resources.primitives_.quad);
        ImGui::Button("Sphere");
        editor_layer->DraggableAsset<Mesh>(resources.primitives_.sphere);
        ImGui::Button("Cube");
        editor_layer->DraggableAsset<Mesh>(resources.primitives_.cube);
        ImGui::Button("Cone");
        editor_layer->DraggableAsset<Mesh>(resources.primitives_.cone);
        ImGui::Button("Cylinder");
        editor_layer->DraggableAsset<Mesh>(resources.primitives_.cylinder);
        ImGui::Button("Torus");
        editor_layer->DraggableAsset<Mesh>(resources.primitives_.torus);
        ImGui::Button("Monkey");
        editor_layer->DraggableAsset<Mesh>(resources.primitives_.monkey);
        ImGui::Button("Capsule");
        editor_layer->DraggableAsset<Mesh>(resources.primitives_.capsule);
      }
    }
    ImGui::End();
  }
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

  resources.texture_pass_through_quad_.reset();
  resources.rendering_cube_.reset();

  resources.ClearPrimitives();
}
