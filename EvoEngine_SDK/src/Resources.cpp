#include "Resources.hpp"

#include "Cubemap.hpp"
#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"
#include "TextureStorage.hpp"
#include "Utilities.hpp"
using namespace evo_engine;

std::shared_ptr<Texture2D> Resources::missing_texture{};
std::shared_ptr<Texture2D> Resources::default_environmental_map_texture{};
std::shared_ptr<Texture2D> Resources::default_skybox_texture{};

std::shared_ptr<Cubemap> Resources::default_skybox{};
std::shared_ptr<EnvironmentalMap> Resources::default_environmental_map{};

std::shared_ptr<Mesh> Resources::texture_pass_through_quad{};
std::shared_ptr<Mesh> Resources::rendering_cube{};

std::shared_ptr<Mesh> Resources::Primitives::quad{};
std::shared_ptr<Mesh> Resources::Primitives::sphere{};
std::shared_ptr<Mesh> Resources::Primitives::cube{};
std::shared_ptr<Mesh> Resources::Primitives::cone{};
std::shared_ptr<Mesh> Resources::Primitives::cylinder{};
std::shared_ptr<Mesh> Resources::Primitives::torus{};
std::shared_ptr<Mesh> Resources::Primitives::monkey{};
std::shared_ptr<Mesh> Resources::Primitives::capsule{};

void Resources::Primitives::Load() {
  {
    quad = CreateResource<Mesh>();
    quad->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/quad.evemesh");
  }
  {
    sphere = CreateResource<Mesh>();
    sphere->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/sphere.evemesh");
  }
  {
    cube = CreateResource<Mesh>();
    cube->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/cube.evemesh");
  }
  {
    cone = CreateResource<Mesh>();
    cone->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/cone.evemesh");
  }
  {
    cylinder = CreateResource<Mesh>();
    cylinder->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/cylinder.evemesh");
  }
  {
    torus = CreateResource<Mesh>();
    torus->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/torus.evemesh");
  }
  {
    monkey = CreateResource<Mesh>();
    monkey->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/monkey.evemesh");
  }
  {
    capsule = CreateResource<Mesh>();
    capsule->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/capsule.evemesh");
  }
}
void Resources::Primitives::OnDestroy() {
  quad.reset();
  sphere.reset();
  cube.reset();
  cone.reset();
  cylinder.reset();
  torus.reset();
  monkey.reset();
  capsule.reset();
}

void Resources::LoadPrimitives() {
  auto& resources = GetInstance();
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
    texture_pass_through_quad = CreateResource<Mesh>();
    texture_pass_through_quad->SetVertices(attributes, vertices, triangles);
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
    rendering_cube = CreateResource<Mesh>();
    rendering_cube->SetVertices(attributes, vertices, triangles);
  }
  Primitives::Load();
}

void Resources::Initialize() {
  auto& resources = GetInstance();
  resources.typed_resources_.clear();
  resources.resources_.clear();
  resources.current_max_handle_ = Handle(1);
  LoadPrimitives();

  GeometryStorage::DeviceSync();
  TextureStorage::DeviceSync();
  missing_texture = CreateResource<Texture2D>();
  missing_texture->LoadInternal(std::filesystem::path("./DefaultResources") / "Textures/texture-missing.png");

  default_environmental_map_texture = CreateResource<Texture2D>();
  default_environmental_map_texture->LoadInternal(std::filesystem::path("./DefaultResources") /
                                                  "Textures/Cubemaps/GrandCanyon/GCanyon_C_YumaPoint_3k.hdr");

  default_skybox_texture = CreateResource<Texture2D>();
  default_skybox_texture->LoadInternal(std::filesystem::path("./DefaultResources") /
                                       "Textures/Cubemaps/GrandCanyon/GCanyon_C_YumaPoint_Env.hdr");

  TextureStorage::DeviceSync();

  default_skybox = CreateResource<Cubemap>();
  default_skybox->Initialize(256);
  default_skybox->ConvertFromEquirectangularTexture(default_skybox_texture);

  default_environmental_map = CreateResource<EnvironmentalMap>();
  default_environmental_map->ConstructFromTexture2D(default_environmental_map_texture);
}

Handle Resources::GenerateNewHandle() {
  return current_max_handle_.value_++;
}

void Resources::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& resources = GetInstance();
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("View")) {
      ImGui::Checkbox("Resources", &resources.show_resources_);
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
  if (resources.show_resources_) {
    if (ImGui::Begin("Resources")) {
      if (ImGui::CollapsingHeader("Textures")) {
        ImGui::Button("Missing");
        editor_layer->DraggableAsset<Texture2D>(missing_texture);
      }
      if (ImGui::CollapsingHeader("Cubemap")) {
        ImGui::Button("Default Skybox");
        editor_layer->DraggableAsset<Cubemap>(default_skybox);
      }
      if (ImGui::CollapsingHeader("Environmental Map")) {
        ImGui::Button("Default Env map");
        editor_layer->DraggableAsset<EnvironmentalMap>(default_environmental_map);
      }
      if (ImGui::CollapsingHeader("Primitives")) {
        ImGui::Button("Quad");
        editor_layer->DraggableAsset<Mesh>(Primitives::quad);
        ImGui::Button("Sphere");
        editor_layer->DraggableAsset<Mesh>(Primitives::sphere);
        ImGui::Button("Cube");
        editor_layer->DraggableAsset<Mesh>(Primitives::cube);
        ImGui::Button("Cone");
        editor_layer->DraggableAsset<Mesh>(Primitives::cone);
        ImGui::Button("Cylinder");
        editor_layer->DraggableAsset<Mesh>(Primitives::cylinder);
        ImGui::Button("Torus");
        editor_layer->DraggableAsset<Mesh>(Primitives::torus);
        ImGui::Button("Monkey");
        editor_layer->DraggableAsset<Mesh>(Primitives::monkey);
        ImGui::Button("Capsule");
        editor_layer->DraggableAsset<Mesh>(Primitives::capsule);
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
  resources.typed_resources_.clear();
  resources.resources_.clear();

  missing_texture.reset();
  default_environmental_map_texture.reset();
  default_skybox_texture.reset();

  default_skybox.reset();
  default_environmental_map.reset();

  texture_pass_through_quad.reset();
  rendering_cube.reset();

  Primitives::OnDestroy();
}
