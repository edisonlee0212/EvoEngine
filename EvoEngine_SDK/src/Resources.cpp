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
void Resources::LoadPrimitives() {
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
    const auto tex_pass_through = CreateResource<Mesh>("PRIMITIVE_TEX_PASS_THROUGH");
    tex_pass_through->SetVertices(attributes, vertices, triangles);
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
    const auto rendering_cube = CreateResource<Mesh>("PRIMITIVE_RENDERING_CUBE");
    rendering_cube->SetVertices(attributes, vertices, triangles);
  }
  {
    const auto quad = CreateResource<Mesh>("PRIMITIVE_QUAD");
    quad->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/quad.evemesh");
  }
  {
    const auto sphere = CreateResource<Mesh>("PRIMITIVE_SPHERE");
    sphere->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/sphere.evemesh");
  }
  {
    const auto cube = CreateResource<Mesh>("PRIMITIVE_CUBE");
    cube->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/cube.evemesh");
  }
  {
    const auto cone = CreateResource<Mesh>("PRIMITIVE_CONE");
    cone->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/cone.evemesh");
  }
  {
    const auto cylinder = CreateResource<Mesh>("PRIMITIVE_CYLINDER");
    cylinder->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/cylinder.evemesh");
  }
  {
    const auto torus = CreateResource<Mesh>("PRIMITIVE_TORUS");
    torus->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/torus.evemesh");
  }
  {
    const auto monkey = CreateResource<Mesh>("PRIMITIVE_MONKEY");
    monkey->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/monkey.evemesh");
  }
  {
    const auto capsule = CreateResource<Mesh>("PRIMITIVE_CAPSULE");
    capsule->LoadInternal(std::filesystem::path("./DefaultResources") / "Primitives/capsule.evemesh");
  }
}

void Resources::Initialize() {
  auto& resources = GetInstance();
  resources.typed_resources_.clear();
  resources.named_resources_.clear();
  resources.resources_.clear();
  resources.current_max_handle_ = Handle(1);
  LoadPrimitives();

  GeometryStorage::DeviceSync();

  const auto missing_texture = CreateResource<Texture2D>("TEXTURE_MISSING");
  missing_texture->LoadInternal(std::filesystem::path("./DefaultResources") / "Textures/texture-missing.png");

  const auto default_environmental_map_texture = CreateResource<Texture2D>("DEFAULT_ENVIRONMENTAL_MAP_TEXTURE");
  default_environmental_map_texture->LoadInternal(std::filesystem::path("./DefaultResources") /
                                                  "Textures/Cubemaps/GrandCanyon/GCanyon_C_YumaPoint_3k.hdr");

  const auto default_skybox_texture = CreateResource<Texture2D>("DEFAULT_SKYBOX_TEXTURE");
  default_skybox_texture->LoadInternal(std::filesystem::path("./DefaultResources") /
                                       "Textures/Cubemaps/GrandCanyon/GCanyon_C_YumaPoint_Env.hdr");

  TextureStorage::DeviceSync();

  const auto default_skybox = CreateResource<Cubemap>("DEFAULT_SKYBOX");
  default_skybox->Initialize(256);
  default_skybox->ConvertFromEquirectangularTexture(default_skybox_texture);

  const auto default_environmental_map = CreateResource<EnvironmentalMap>("DEFAULT_ENVIRONMENTAL_MAP");
  default_environmental_map->ConstructFromTexture2D(default_environmental_map_texture);
}

Handle Resources::GenerateNewHandle() {
  return current_max_handle_.value_++;
}

void Resources::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& resources = GetInstance();
  const auto& project_manager = ProjectManager::GetInstance();
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("View")) {
      ImGui::Checkbox("Assets", &resources.show_assets_);
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
  if (resources.show_assets_) {
    ImGui::Begin("Assets");
    if (ImGui::BeginTabBar("##Assets", ImGuiTabBarFlags_NoCloseWithMiddleMouseButton)) {
      if (ImGui::BeginTabItem("Inspection")) {
        if (project_manager.inspecting_asset) {
          const auto& asset = project_manager.inspecting_asset;
          ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0.5f, 0, 1));
          ImGui::Button(asset->GetTitle().c_str());
          ImGui::PopStyleColor(1);
          editor_layer->DraggableAsset(asset);
          ImGui::SameLine();
          ImGui::Text("Type:");
          ImGui::SameLine();
          ImGui::Text(asset->GetTypeName().c_str());
          if (!asset->IsTemporary()) {
            if (ImGui::Button("Save")) {
              asset->Save();
            }
            ImGui::SameLine();
            if (ImGui::Button("Reload")) {
              asset->Load();
            }
          }
          ImGui::SameLine();
          FileUtils::SaveFile(
              "Export...", asset->GetTypeName(), Serialization::PeekAssetExtensions(asset->GetTypeName()),
              [&](const std::filesystem::path& path) {
                asset->Export(path);
              },
              false);
          ImGui::SameLine();
          FileUtils::OpenFile(
              "Import...", asset->GetTypeName(), Serialization::PeekAssetExtensions(asset->GetTypeName()),
              [&](const std::filesystem::path& path) {
                asset->Import(path);
              },
              false);

          ImGui::Separator();
          if (asset->OnInspect(editor_layer))
            asset->SetUnsaved();
        } else {
          ImGui::Text("None");
        }
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Resources")) {
        for (auto& collection : resources.typed_resources_) {
          if (ImGui::CollapsingHeader(collection.first.c_str())) {
            for (auto& i : collection.second) {
              ImGui::Button(resources.resource_names_[i.second->GetHandle()].c_str());
              editor_layer->DraggableAsset(i.second);
            }
          }
        }
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
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
  resources.named_resources_.clear();
  resources.resource_names_.clear();
  resources.resources_.clear();
}
