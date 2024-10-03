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

void Resources::LoadShaders() {
#pragma region Shader Includes
  Shader::RegisterShaderIncludePath(std::filesystem::path("./DefaultResources/Shaders/Includes"));
  const auto& add = Platform::Constants::shader_global_defines;
#pragma endregion
#pragma region Ray Tracing Shader
  {
    auto camera_raygen = CreateResource<Shader>("RAY_TRACING_CAMERA_RAYGEN");
    camera_raygen->Set(ShaderType::RayGen, add,
                       std::filesystem::path("./DefaultResources") / "Shaders/RayTracing/RayGen/Camera.rgen");

    auto camera_miss = CreateResource<Shader>("RAY_TRACING_CAMERA_MISS");
    camera_miss->Set(ShaderType::Miss, add,
                     std::filesystem::path("./DefaultResources") / "Shaders/RayTracing/Miss/Camera.rmiss");

    auto camera_chit = CreateResource<Shader>("RAY_TRACING_CAMERA_CHIT");
    camera_chit->Set(ShaderType::Miss, add,
                     std::filesystem::path("./DefaultResources") / "Shaders/RayTracing/ClosestHit/Camera.rchit");
  }
#pragma endregion

#pragma region Standard Shader
  {
    auto standard_vert = CreateResource<Shader>("STANDARD_VERT");
    standard_vert->Set(ShaderType::Vertex, add,
                       std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Standard/Standard.vert");

    standard_vert = CreateResource<Shader>("STANDARD_SKINNED_VERT");
    standard_vert->Set(
        ShaderType::Vertex, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Standard/StandardSkinned.vert");

    standard_vert = CreateResource<Shader>("STANDARD_INSTANCED_VERT");
    standard_vert->Set(
        ShaderType::Vertex, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Standard/StandardInstanced.vert");

    standard_vert = CreateResource<Shader>("STANDARD_STRANDS_VERT");
    standard_vert->Set(
        ShaderType::Vertex, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Standard/StandardStrands.vert");

    auto standard_tesc = CreateResource<Shader>("STANDARD_STRANDS_TESC");
    standard_tesc->Set(ShaderType::TessellationControl, add,
                       std::filesystem::path("./DefaultResources") /
                           "Shaders/Graphics/TessellationControl/Standard/StandardStrands.tesc");

    auto standard_tese = CreateResource<Shader>("STANDARD_STRANDS_TESE");
    standard_tese->Set(ShaderType::TessellationEvaluation, add,
                       std::filesystem::path("./DefaultResources") /
                           "Shaders/Graphics/TessellationEvaluation/Standard/StandardStrands.tese");

    auto standard_geom = CreateResource<Shader>("STANDARD_STRANDS_GEOM");
    standard_geom->Set(
        ShaderType::Geometry, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Geometry/Standard/StandardStrands.geom");

    auto standard_task = CreateResource<Shader>("STANDARD_TASK");
    standard_task->Set(ShaderType::Task, add,
                       std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Task/Standard/Standard.task");

    auto standard_mesh = CreateResource<Shader>("STANDARD_MESH");
    standard_mesh->Set(ShaderType::Mesh, add,
                       std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Mesh/Standard/Standard.mesh");

    standard_mesh = CreateResource<Shader>("STANDARD_MESHLET_COLORED_MESH");
    standard_mesh->Set(
        ShaderType::Mesh, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Mesh/Standard/StandardMeshletColored.mesh");
  }

#pragma endregion
  auto tex_pass_vert = CreateResource<Shader>("TEXTURE_PASS_THROUGH_VERT");
  tex_pass_vert->Set(ShaderType::Vertex,
                     std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/TexturePassThrough.vert");

  auto tex_pass_frag = CreateResource<Shader>("TEXTURE_PASS_THROUGH_FRAG");
  tex_pass_frag->Set(ShaderType::Fragment,
                     std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/TexturePassThrough.frag");

#pragma region GBuffer
  {
    auto frag_shader = CreateResource<Shader>("STANDARD_DEFERRED_LIGHTING_FRAG");
    frag_shader->Set(ShaderType::Fragment, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Fragment/Standard/StandardDeferredLighting.frag");

    frag_shader = CreateResource<Shader>("STANDARD_DEFERRED_LIGHTING_SCENE_CAMERA_FRAG");
    frag_shader->Set(ShaderType::Fragment, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Fragment/Standard/StandardDeferredLightingSceneCamera.frag");

    frag_shader = CreateResource<Shader>("STANDARD_DEFERRED_FRAG");
    frag_shader->Set(
        ShaderType::Fragment, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Standard/StandardDeferred.frag");
  }
#pragma endregion

#pragma region PostProcessing
  {
    auto frag_shader = CreateResource<Shader>("SSR_REFLECT_FRAG");
    frag_shader->Set(
        ShaderType::Fragment, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/PostProcessing/SSRReflect.frag");

    frag_shader = CreateResource<Shader>("SSR_BLUR_FRAG");
    frag_shader->Set(ShaderType::Fragment, std::filesystem::path("./DefaultResources") /
                                               "Shaders/Graphics/Fragment/PostProcessing/SSRBlur.frag");

    frag_shader = CreateResource<Shader>("SSR_COMBINE_FRAG");
    frag_shader->Set(ShaderType::Fragment, std::filesystem::path("./DefaultResources") /
                                               "Shaders/Graphics/Fragment/PostProcessing/SSRCombine.frag");
  }
#pragma endregion
#pragma region Shadow Maps
  {
    auto vert_shader = CreateResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMap.vert");

    vert_shader = CreateResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_SKINNED_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMapSkinned.vert");

    vert_shader = CreateResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_INSTANCED_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMapInstanced.vert");

    vert_shader = CreateResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_STRANDS_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMapStrands.vert");

    vert_shader = CreateResource<Shader>("POINT_LIGHT_SHADOW_MAP_VERT");
    vert_shader->Set(
        ShaderType::Vertex, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Lighting/PointLightShadowMap.vert");

    vert_shader = CreateResource<Shader>("POINT_LIGHT_SHADOW_MAP_SKINNED_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Vertex/Lighting/PointLightShadowMapSkinned.vert");

    vert_shader = CreateResource<Shader>("POINT_LIGHT_SHADOW_MAP_INSTANCED_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Vertex/Lighting/PointLightShadowMapInstanced.vert");

    vert_shader = CreateResource<Shader>("POINT_LIGHT_SHADOW_MAP_STRANDS_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Vertex/Lighting/PointLightShadowMapStrands.vert");

    vert_shader = CreateResource<Shader>("SPOT_LIGHT_SHADOW_MAP_VERT");
    vert_shader->Set(
        ShaderType::Vertex, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMap.vert");

    vert_shader = CreateResource<Shader>("SPOT_LIGHT_SHADOW_MAP_SKINNED_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMapSkinned.vert");

    vert_shader = CreateResource<Shader>("SPOT_LIGHT_SHADOW_MAP_INSTANCED_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMapInstanced.vert");

    vert_shader = CreateResource<Shader>("SPOT_LIGHT_SHADOW_MAP_STRANDS_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMapStrands.vert");

    auto tesc_shader = CreateResource<Shader>("SHADOW_MAP_STRANDS_TESC");
    tesc_shader->Set(ShaderType::TessellationControl, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/TessellationControl/Lighting/ShadowMapStrands.tesc");

    auto tese_shader = CreateResource<Shader>("SHADOW_MAP_STRANDS_TESE");
    tese_shader->Set(ShaderType::TessellationEvaluation, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/TessellationEvaluation/Lighting/ShadowMapStrands.tese");

    auto geom_shader = CreateResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_STRANDS_GEOM");
    geom_shader->Set(ShaderType::Geometry, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Geometry/Lighting/DirectionalLightShadowMapStrands.geom");

    geom_shader = CreateResource<Shader>("POINT_LIGHT_SHADOW_MAP_STRANDS_GEOM");
    geom_shader->Set(ShaderType::Geometry, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Geometry/Lighting/PointLightShadowMapStrands.geom");

    geom_shader = CreateResource<Shader>("SPOT_LIGHT_SHADOW_MAP_STRANDS_GEOM");
    geom_shader->Set(ShaderType::Geometry, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Geometry/Lighting/SpotLightShadowMapStrands.geom");

    auto task_shader = CreateResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_TASK");
    task_shader->Set(
        ShaderType::Task, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Task/Lighting/DirectionalLightShadowMap.task");

    task_shader = CreateResource<Shader>("POINT_LIGHT_SHADOW_MAP_TASK");
    task_shader->Set(
        ShaderType::Task, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Task/Lighting/PointLightShadowMap.task");

    task_shader = CreateResource<Shader>("SPOT_LIGHT_SHADOW_MAP_TASK");
    task_shader->Set(
        ShaderType::Task, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Task/Lighting/SpotLightShadowMap.task");

    auto mesh_shader = CreateResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_MESH");
    mesh_shader->Set(
        ShaderType::Mesh, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Mesh/Lighting/DirectionalLightShadowMap.mesh");

    mesh_shader = CreateResource<Shader>("POINT_LIGHT_SHADOW_MAP_MESH");
    mesh_shader->Set(
        ShaderType::Mesh, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Mesh/Lighting/PointLightShadowMap.mesh");

    mesh_shader = CreateResource<Shader>("SPOT_LIGHT_SHADOW_MAP_MESH");
    mesh_shader->Set(
        ShaderType::Mesh, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Mesh/Lighting/SpotLightShadowMap.mesh");

    auto frag_shader = CreateResource<Shader>("EMPTY_FRAG");
    frag_shader->Set(ShaderType::Fragment, add,
                     std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Empty.frag");

    frag_shader = CreateResource<Shader>("SHADOW_MAP_PASS_THROUGH_FRAG");
    frag_shader->Set(
        ShaderType::Fragment, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/ShadowMapPassThrough.frag");
  }
#pragma endregion
#pragma region Environmental
  {
    auto vert_shader = CreateResource<Shader>("EQUIRECTANGULAR_MAP_TO_CUBEMAP_VERT");
    vert_shader->Set(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                             "Shaders/Graphics/Vertex/Lighting/EquirectangularMapToCubemap.vert");
    auto frag_shader = CreateResource<Shader>("ENVIRONMENTAL_MAP_BRDF_FRAG");
    frag_shader->Set(ShaderType::Fragment, std::filesystem::path("./DefaultResources") /
                                               "Shaders/Graphics/Fragment/Lighting/EnvironmentalMapBrdf.frag");
    frag_shader = CreateResource<Shader>("EQUIRECTANGULAR_MAP_TO_CUBEMAP_FRAG");
    frag_shader->Set(ShaderType::Fragment, std::filesystem::path("./DefaultResources") /
                                               "Shaders/Graphics/Fragment/Lighting/EquirectangularMapToCubemap.frag");

    frag_shader = CreateResource<Shader>("IRRADIANCE_CONSTRUCT_FRAG");
    frag_shader->Set(ShaderType::Fragment,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/Fragment/Lighting/EnvironmentalMapIrradianceConvolution.frag");

    frag_shader = CreateResource<Shader>("PREFILTER_CONSTRUCT_FRAG");
    frag_shader->Set(ShaderType::Fragment, std::filesystem::path("./DefaultResources") /
                                               "Shaders/Graphics/Fragment/Lighting/EnvironmentalMapPrefilter.frag");
  }
#pragma endregion

#pragma region Gizmos
  {
    auto vert_shader = CreateResource<Shader>("GIZMOS_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/Gizmos.vert");

    vert_shader = CreateResource<Shader>("GIZMOS_STRANDS_VERT");
    vert_shader->Set(ShaderType::Vertex, add,
                     std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosStrands.vert");

    vert_shader = CreateResource<Shader>("GIZMOS_INSTANCED_COLORED_VERT");
    vert_shader->Set(
        ShaderType::Vertex, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosInstancedColored.vert");

    vert_shader = CreateResource<Shader>("GIZMOS_NORMAL_COLORED_VERT");
    vert_shader->Set(
        ShaderType::Vertex, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosNormalColored.vert");

    vert_shader = CreateResource<Shader>("GIZMOS_STRANDS_NORMAL_COLORED_VERT");
    vert_shader->Set(
        ShaderType::Vertex, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosStrandsNormalColored.vert");

    vert_shader = CreateResource<Shader>("GIZMOS_VERTEX_COLORED_VERT");
    vert_shader->Set(
        ShaderType::Vertex, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosVertexColored.vert");

    auto tesc_shader = CreateResource<Shader>("GIZMOS_STRANDS_TESC");
    tesc_shader->Set(
        ShaderType::TessellationControl, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/TessellationControl/Gizmos/GizmosStrands.tesc");

    tesc_shader = CreateResource<Shader>("GIZMOS_STRANDS_COLORED_TESC");
    tesc_shader->Set(ShaderType::TessellationControl, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/TessellationControl/Gizmos/GizmosStrandsColored.tesc");

    auto tese_shader = CreateResource<Shader>("GIZMOS_STRANDS_TESE");
    tese_shader->Set(ShaderType::TessellationEvaluation, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/TessellationEvaluation/Gizmos/GizmosStrands.tese");

    tese_shader = CreateResource<Shader>("GIZMOS_STRANDS_COLORED_TESE");
    tese_shader->Set(ShaderType::TessellationEvaluation, add,
                     std::filesystem::path("./DefaultResources") /
                         "Shaders/Graphics/TessellationEvaluation/Gizmos/GizmosStrandsColored.tese");

    auto geom_shader = CreateResource<Shader>("GIZMOS_STRANDS_GEOM");
    geom_shader->Set(
        ShaderType::Geometry, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Geometry/Gizmos/GizmosStrands.geom");

    geom_shader = CreateResource<Shader>("GIZMOS_STRANDS_COLORED_GEOM");
    geom_shader->Set(
        ShaderType::Geometry, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Geometry/Gizmos/GizmosStrandsColored.geom");

    vert_shader = CreateResource<Shader>("GIZMOS_STRANDS_VERTEX_COLORED_VERT");
    vert_shader->Set(
        ShaderType::Vertex, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosStrandsVertexColored.vert");

    auto frag_shader = CreateResource<Shader>("GIZMOS_FRAG");
    frag_shader->Set(ShaderType::Fragment, add,
                     std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Gizmos/Gizmos.frag");

    frag_shader = CreateResource<Shader>("GIZMOS_COLORED_FRAG");
    frag_shader->Set(
        ShaderType::Fragment, add,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.frag");
  }
#pragma endregion
}

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

  resources.LoadShaders();

  const auto missing_texture = CreateResource<Texture2D>("TEXTURE_MISSING");
  missing_texture->LoadInternal(std::filesystem::path("./DefaultResources") / "Textures/texture-missing.png");
}

void Resources::InitializeEnvironmentalMap() {
  auto& resources = GetInstance();
  resources.LoadPrimitives();
  GeometryStorage::DeviceSync();
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
  auto& project_manager = ProjectManager::GetInstance();
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
          auto& asset = project_manager.inspecting_asset;
          ImGui::Text("Type:");
          ImGui::SameLine();
          ImGui::Text(asset->GetTypeName().c_str());
          ImGui::Separator();
          ImGui::Text("Name:");
          ImGui::SameLine();
          ImGui::Button(asset->GetTitle().c_str());
          editor_layer->DraggableAsset(asset);
          if (!asset->IsTemporary()) {
            if (ImGui::Button("Save")) {
              asset->Save();
            }
          } else {
            FileUtils::SaveFile(
                "Allocate path & save", asset->GetTypeName(), Serialization::PeekAssetExtensions(asset->GetTypeName()),
                [&](const std::filesystem::path& path) {
                  asset->SetPathAndSave(std::filesystem::relative(path, project_manager.project_path_));
                },
                true);
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
      if (ImGui::BeginTabItem("Shared Assets")) {
        if (ImGui::BeginDragDropTarget()) {
          if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
            IM_ASSERT(payload->DataSize == sizeof(Handle));
            const Handle handle = *static_cast<Handle*>(payload->Data);

            if (const auto asset = ProjectManager::GetAsset(handle)) {
              AssetRef ref;
              ref.Set(asset);
              resources.shared_assets_[asset->GetTypeName()].emplace_back(ref);
            }
          }
          ImGui::EndDragDropTarget();
        }

        for (auto& collection : resources.shared_assets_) {
          if (ImGui::CollapsingHeader(collection.first.c_str())) {
            for (auto it = collection.second.begin(); it != collection.second.end(); ++it) {
              auto asset_ref = *it;
              const auto ptr = asset_ref.Get<IAsset>();
              const std::string tag = "##" + ptr->GetTypeName() + std::to_string(ptr->GetHandle());
              ImGui::Button((ptr->GetTitle() + tag).c_str());

              EditorLayer::Rename(asset_ref);
              if (EditorLayer::Remove(asset_ref)) {
                collection.second.erase(it);
                break;
              }
              if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
                project_manager.inspecting_asset = ptr;
              }
              EditorLayer::Draggable(asset_ref);
            }
          }
        }
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Loaded Assets")) {
        for (auto& asset : project_manager.loaded_assets_) {
          if (asset.second->IsTemporary())
            continue;
          ImGui::Button(asset.second->GetTitle().c_str());
          editor_layer->DraggableAsset(asset.second);
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