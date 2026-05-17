#include "RenderLayer.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "Jobs.hpp"
#include "LodGroup.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"
#include "TextureStorage.hpp"
#include "Utilities.hpp"
#include "WindowLayer.hpp"
#include <algorithm>
#include <cctype>
#include <limits>
#include <sstream>
using namespace evo_engine;

void RenderLayer::RenderToPointLightShadowMap(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const PointLightShadowMapView& shadow_map_view)>&& func) {
  point_light_shadow_map_external_functions.emplace_back(func);
}

void RenderLayer::RenderToSpotLightShadowMap(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>&& func) {
  spot_light_shadow_map_external_functions.emplace_back(func);
}

void RenderLayer::RenderToDirectionalLightShadowMap(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>&&
        func) {
  directional_light_shadow_map_external_functions.emplace_back(func);
}

void RenderLayer::DeferredRenderingAllCameras(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer,
                           const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                           const DeferredRenderingView& forward_rendering_view)>&& func) {
  deferred_rendering_external_functions.emplace_back(func);
}

void RenderLayer::ForwardRenderingAllCameras(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                           const ForwardRenderingView& forward_rendering_view)>&& func) {
  forward_rendering_external_functions.emplace_back(func);
}

void RenderLayer::OnCreate() {
#pragma region Graphics Pipelines
  if (!point_light_shadow_pipeline_normal) {
    point_light_shadow_pipeline_normal = std::make_shared<GraphicsPipeline>();
    point_light_shadow_pipeline_normal->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Lighting/PointLightShadowMap.vert");
    point_light_shadow_pipeline_normal->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/ShadowMapPassThrough.frag");
    point_light_shadow_pipeline_normal->geometry_type = GeometryType::Mesh;
    point_light_shadow_pipeline_normal->descriptor_set_layouts.emplace_back(per_frame_layout);
    point_light_shadow_pipeline_normal->depth_attachment_format = Platform::Constants::shadow_map;
    point_light_shadow_pipeline_normal->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = point_light_shadow_pipeline_normal->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    point_light_shadow_pipeline_normal->Initialize();
  }
  if (Platform::Constants::support_mesh_shader && !point_light_shadow_pipeline_mesh_shader) {
    point_light_shadow_pipeline_mesh_shader = std::make_shared<GraphicsPipeline>();
    point_light_shadow_pipeline_mesh_shader->task_shader = Shader::CreateTemporary(
        ShaderType::Task, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Task/Lighting/PointLightShadowMap.task");
    point_light_shadow_pipeline_mesh_shader->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Mesh/Lighting/PointLightShadowMap.mesh");
    point_light_shadow_pipeline_mesh_shader->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Empty.frag");
    point_light_shadow_pipeline_mesh_shader->geometry_type = GeometryType::Mesh;
    point_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(per_frame_layout);
    point_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(meshlet_layout);
    point_light_shadow_pipeline_mesh_shader->depth_attachment_format = Platform::Constants::shadow_map;
    point_light_shadow_pipeline_mesh_shader->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = point_light_shadow_pipeline_mesh_shader->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    point_light_shadow_pipeline_mesh_shader->Initialize();
  }
  if (!spot_light_shadow_pipeline_normal) {
    spot_light_shadow_pipeline_normal = std::make_shared<GraphicsPipeline>();
    spot_light_shadow_pipeline_normal->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMap.vert");
    spot_light_shadow_pipeline_normal->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/ShadowMapPassThrough.frag");
    spot_light_shadow_pipeline_normal->geometry_type = GeometryType::Mesh;
    spot_light_shadow_pipeline_normal->descriptor_set_layouts.emplace_back(per_frame_layout);
    spot_light_shadow_pipeline_normal->depth_attachment_format = Platform::Constants::shadow_map;
    spot_light_shadow_pipeline_normal->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = spot_light_shadow_pipeline_normal->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    spot_light_shadow_pipeline_normal->Initialize();
  }
  if (Platform::Constants::support_mesh_shader && !spot_light_shadow_pipeline_mesh_shader) {
    spot_light_shadow_pipeline_mesh_shader = std::make_shared<GraphicsPipeline>();
    spot_light_shadow_pipeline_mesh_shader->task_shader = Shader::CreateTemporary(
        ShaderType::Task, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Task/Lighting/SpotLightShadowMap.task");
    spot_light_shadow_pipeline_mesh_shader->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Mesh/Lighting/SpotLightShadowMap.mesh");
    spot_light_shadow_pipeline_mesh_shader->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Empty.frag");
    spot_light_shadow_pipeline_mesh_shader->geometry_type = GeometryType::Mesh;
    spot_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(per_frame_layout);
    spot_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(meshlet_layout);
    spot_light_shadow_pipeline_mesh_shader->depth_attachment_format = Platform::Constants::shadow_map;
    spot_light_shadow_pipeline_mesh_shader->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = spot_light_shadow_pipeline_mesh_shader->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    spot_light_shadow_pipeline_mesh_shader->Initialize();
  }
  if (!directional_light_shadow_pipeline_normal) {
    directional_light_shadow_pipeline_normal = std::make_shared<GraphicsPipeline>();
    directional_light_shadow_pipeline_normal->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMap.vert");
    directional_light_shadow_pipeline_normal->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/ShadowMapPassThrough.frag");
    directional_light_shadow_pipeline_normal->geometry_type = GeometryType::Mesh;
    directional_light_shadow_pipeline_normal->descriptor_set_layouts.emplace_back(per_frame_layout);
    directional_light_shadow_pipeline_normal->depth_attachment_format = Platform::Constants::shadow_map;
    directional_light_shadow_pipeline_normal->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = directional_light_shadow_pipeline_normal->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    directional_light_shadow_pipeline_normal->Initialize();
  }
  if (Platform::Constants::support_mesh_shader && !directional_light_shadow_pipeline_mesh_shader) {
    directional_light_shadow_pipeline_mesh_shader = std::make_shared<GraphicsPipeline>();
    directional_light_shadow_pipeline_mesh_shader->task_shader = Shader::CreateTemporary(
        ShaderType::Task, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Task/Lighting/DirectionalLightShadowMap.task");
    directional_light_shadow_pipeline_mesh_shader->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Mesh/Lighting/DirectionalLightShadowMap.mesh");
    directional_light_shadow_pipeline_mesh_shader->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Empty.frag");
    directional_light_shadow_pipeline_mesh_shader->geometry_type = GeometryType::Mesh;
    directional_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(per_frame_layout);
    directional_light_shadow_pipeline_mesh_shader->descriptor_set_layouts.emplace_back(meshlet_layout);
    directional_light_shadow_pipeline_mesh_shader->depth_attachment_format = Platform::Constants::shadow_map;
    directional_light_shadow_pipeline_mesh_shader->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = directional_light_shadow_pipeline_mesh_shader->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    directional_light_shadow_pipeline_mesh_shader->Initialize();
  }
  if (!instanced_point_light_shadow_pipeline) {
    instanced_point_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    instanced_point_light_shadow_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Vertex/Lighting/PointLightShadowMapInstanced.vert");
    instanced_point_light_shadow_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/ShadowMapPassThrough.frag");
    instanced_point_light_shadow_pipeline->geometry_type = GeometryType::Mesh;
    instanced_point_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    instanced_point_light_shadow_pipeline->descriptor_set_layouts.emplace_back(ParticleInfoList::instanced_data_layout);
    instanced_point_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    instanced_point_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = instanced_point_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    instanced_point_light_shadow_pipeline->Initialize();
  }
  if (!instanced_spot_light_shadow_pipeline) {
    instanced_spot_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    instanced_spot_light_shadow_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMapInstanced.vert");
    instanced_spot_light_shadow_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/ShadowMapPassThrough.frag");
    instanced_spot_light_shadow_pipeline->geometry_type = GeometryType::Mesh;
    instanced_spot_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    instanced_spot_light_shadow_pipeline->descriptor_set_layouts.emplace_back(ParticleInfoList::instanced_data_layout);
    instanced_spot_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    instanced_spot_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = instanced_spot_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    instanced_spot_light_shadow_pipeline->Initialize();
  }
  if (!instanced_directional_light_shadow_pipeline) {
    instanced_directional_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    instanced_directional_light_shadow_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMapInstanced.vert");
    instanced_directional_light_shadow_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/ShadowMapPassThrough.frag");
    instanced_directional_light_shadow_pipeline->geometry_type = GeometryType::Mesh;
    instanced_directional_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    instanced_directional_light_shadow_pipeline->descriptor_set_layouts.emplace_back(
        ParticleInfoList::instanced_data_layout);
    instanced_directional_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    instanced_directional_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = instanced_directional_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    instanced_directional_light_shadow_pipeline->Initialize();
  }
  if (!skinned_point_light_shadow_pipeline) {
    skinned_point_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    skinned_point_light_shadow_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Vertex/Lighting/PointLightShadowMapSkinned.vert");
    skinned_point_light_shadow_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/ShadowMapPassThrough.frag");
    skinned_point_light_shadow_pipeline->geometry_type = GeometryType::SkinnedMesh;
    skinned_point_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    skinned_point_light_shadow_pipeline->descriptor_set_layouts.emplace_back(BoneMatrices::bone_matrices_layout);
    skinned_point_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    skinned_point_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = skinned_point_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    skinned_point_light_shadow_pipeline->Initialize();
  }
  if (!skinned_spot_light_shadow_pipeline) {
    skinned_spot_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    skinned_spot_light_shadow_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMapSkinned.vert");
    skinned_spot_light_shadow_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/ShadowMapPassThrough.frag");
    skinned_spot_light_shadow_pipeline->geometry_type = GeometryType::SkinnedMesh;
    skinned_spot_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    skinned_spot_light_shadow_pipeline->descriptor_set_layouts.emplace_back(BoneMatrices::bone_matrices_layout);
    skinned_spot_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    skinned_spot_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = skinned_spot_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    skinned_spot_light_shadow_pipeline->Initialize();
  }
  if (!skinned_directional_light_shadow_pipeline) {
    skinned_directional_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    skinned_directional_light_shadow_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMapSkinned.vert");
    skinned_directional_light_shadow_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/ShadowMapPassThrough.frag");
    skinned_directional_light_shadow_pipeline->geometry_type = GeometryType::SkinnedMesh;
    skinned_directional_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    skinned_directional_light_shadow_pipeline->descriptor_set_layouts.emplace_back(BoneMatrices::bone_matrices_layout);
    skinned_directional_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    skinned_directional_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    auto& push_constant_range = skinned_directional_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    skinned_directional_light_shadow_pipeline->Initialize();
  }
#ifdef EVOENGINE_WINDOWS
  if (!strands_point_light_shadow_pipeline) {
    strands_point_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    strands_point_light_shadow_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Vertex/Lighting/PointLightShadowMapStrands.vert");
    strands_point_light_shadow_pipeline->tessellation_control_shader =
        Shader::CreateTemporary(ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationControl/Lighting/ShadowMapStrands.tesc");
    strands_point_light_shadow_pipeline->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationEvaluation/Lighting/ShadowMapStrands.tese");
    strands_point_light_shadow_pipeline->geometry_shader =
        Shader::CreateTemporary(ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Geometry/Lighting/PointLightShadowMapStrands.geom");
    strands_point_light_shadow_pipeline->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Empty.frag");
    strands_point_light_shadow_pipeline->geometry_type = GeometryType::Strands;
    strands_point_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    strands_point_light_shadow_pipeline->descriptor_set_layouts.emplace_back(ParticleInfoList::instanced_data_layout);
    strands_point_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    strands_point_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    strands_point_light_shadow_pipeline->tessellation_patch_control_points = 4;
    auto& push_constant_range = strands_point_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    strands_point_light_shadow_pipeline->Initialize();
  }
  if (!strands_spot_light_shadow_pipeline) {
    strands_spot_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    strands_spot_light_shadow_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Vertex/Lighting/SpotLightShadowMapStrands.vert");
    strands_spot_light_shadow_pipeline->tessellation_control_shader =
        Shader::CreateTemporary(ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationControl/Lighting/ShadowMapStrands.tesc");
    strands_spot_light_shadow_pipeline->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationEvaluation/Lighting/ShadowMapStrands.tese");
    strands_spot_light_shadow_pipeline->geometry_shader =
        Shader::CreateTemporary(ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Geometry/Lighting/SpotLightShadowMapStrands.geom");
    strands_spot_light_shadow_pipeline->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Empty.frag");
    strands_spot_light_shadow_pipeline->geometry_type = GeometryType::Strands;
    strands_spot_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    strands_spot_light_shadow_pipeline->descriptor_set_layouts.emplace_back(ParticleInfoList::instanced_data_layout);
    strands_spot_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    strands_spot_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    strands_spot_light_shadow_pipeline->tessellation_patch_control_points = 4;
    auto& push_constant_range = strands_spot_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    strands_spot_light_shadow_pipeline->Initialize();
  }
  if (!strands_directional_light_shadow_pipeline) {
    strands_directional_light_shadow_pipeline = std::make_shared<GraphicsPipeline>();
    strands_directional_light_shadow_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Vertex/Lighting/DirectionalLightShadowMapStrands.vert");
    strands_directional_light_shadow_pipeline->tessellation_control_shader =
        Shader::CreateTemporary(ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationControl/Lighting/ShadowMapStrands.tesc");
    strands_directional_light_shadow_pipeline->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationEvaluation/Lighting/ShadowMapStrands.tese");
    strands_directional_light_shadow_pipeline->geometry_shader =
        Shader::CreateTemporary(ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Geometry/Lighting/DirectionalLightShadowMapStrands.geom");
    strands_directional_light_shadow_pipeline->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Empty.frag");
    strands_directional_light_shadow_pipeline->geometry_type = GeometryType::Strands;
    strands_directional_light_shadow_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    strands_directional_light_shadow_pipeline->descriptor_set_layouts.emplace_back(
        ParticleInfoList::instanced_data_layout);
    strands_directional_light_shadow_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
    strands_directional_light_shadow_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    strands_directional_light_shadow_pipeline->tessellation_patch_control_points = 4;
    auto& push_constant_range = strands_directional_light_shadow_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    strands_directional_light_shadow_pipeline->Initialize();
  }
#endif
  if (!deferred_prepass_pipeline_normal) {
    deferred_prepass_pipeline_normal = std::make_shared<GraphicsPipeline>();
    deferred_prepass_pipeline_normal->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Standard/Standard.vert");
    deferred_prepass_pipeline_normal->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Standard/StandardDeferred.frag");
    deferred_prepass_pipeline_normal->geometry_type = GeometryType::Mesh;
    deferred_prepass_pipeline_normal->descriptor_set_layouts.emplace_back(per_frame_layout);
    deferred_prepass_pipeline_normal->depth_attachment_format = Platform::Constants::render_texture_depth;
    deferred_prepass_pipeline_normal->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    deferred_prepass_pipeline_normal->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
    auto& push_constant_range = deferred_prepass_pipeline_normal->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    deferred_prepass_pipeline_normal->Initialize();
  }
  if (Platform::Constants::support_mesh_shader && !deferred_prepass_pipeline_mesh) {
    deferred_prepass_pipeline_mesh = std::make_shared<GraphicsPipeline>();
    deferred_prepass_pipeline_mesh->task_shader = Shader::CreateTemporary(
        ShaderType::Task, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Task/Standard/Standard.task");
    deferred_prepass_pipeline_mesh->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Mesh/Standard/Standard.mesh");
    deferred_prepass_pipeline_mesh->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Standard/StandardDeferred.frag");
    deferred_prepass_pipeline_mesh->geometry_type = GeometryType::Mesh;
    deferred_prepass_pipeline_mesh->descriptor_set_layouts.emplace_back(per_frame_layout);
    deferred_prepass_pipeline_mesh->descriptor_set_layouts.emplace_back(meshlet_layout);
    deferred_prepass_pipeline_mesh->depth_attachment_format = Platform::Constants::render_texture_depth;
    deferred_prepass_pipeline_mesh->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    deferred_prepass_pipeline_mesh->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
    auto& push_constant_range = deferred_prepass_pipeline_mesh->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    deferred_prepass_pipeline_mesh->Initialize();
  }
  if (!instanced_deferred_prepass_pipeline) {
    instanced_deferred_prepass_pipeline = std::make_shared<GraphicsPipeline>();
    instanced_deferred_prepass_pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Standard/StandardInstanced.vert");
    instanced_deferred_prepass_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Standard/StandardDeferredInstanced.frag");
    instanced_deferred_prepass_pipeline->geometry_type = GeometryType::Mesh;
    instanced_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    instanced_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(ParticleInfoList::instanced_data_layout);
    instanced_deferred_prepass_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    instanced_deferred_prepass_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    instanced_deferred_prepass_pipeline->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
    auto& push_constant_range = instanced_deferred_prepass_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    instanced_deferred_prepass_pipeline->Initialize();
  }
  if (!skinned_deferred_prepass_pipeline) {
    skinned_deferred_prepass_pipeline = std::make_shared<GraphicsPipeline>();
    skinned_deferred_prepass_pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Standard/StandardSkinned.vert");
    skinned_deferred_prepass_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Standard/StandardDeferred.frag");
    skinned_deferred_prepass_pipeline->geometry_type = GeometryType::SkinnedMesh;
    skinned_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    skinned_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(BoneMatrices::bone_matrices_layout);
    skinned_deferred_prepass_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    skinned_deferred_prepass_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    skinned_deferred_prepass_pipeline->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
    auto& push_constant_range = skinned_deferred_prepass_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    skinned_deferred_prepass_pipeline->Initialize();
  }
#ifdef EVOENGINE_WINDOWS
  if (!strands_deferred_prepass_pipeline) {
    strands_deferred_prepass_pipeline = std::make_shared<GraphicsPipeline>();
    strands_deferred_prepass_pipeline->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Standard/StandardStrands.vert");
    strands_deferred_prepass_pipeline->tessellation_control_shader =
        Shader::CreateTemporary(ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationControl/Standard/StandardStrands.tesc");
    strands_deferred_prepass_pipeline->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationEvaluation/Standard/StandardStrands.tese");
    strands_deferred_prepass_pipeline->geometry_shader = Shader::CreateTemporary(
        ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Geometry/Standard/StandardStrands.geom");
    strands_deferred_prepass_pipeline->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") /
            "Shaders/Graphics/Fragment/Standard/StandardStrandsDeferred.frag");
    strands_deferred_prepass_pipeline->geometry_type = GeometryType::Strands;
    strands_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    strands_deferred_prepass_pipeline->descriptor_set_layouts.emplace_back(ParticleInfoList::instanced_data_layout);
    strands_deferred_prepass_pipeline->tessellation_patch_control_points = 4;
    strands_deferred_prepass_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    strands_deferred_prepass_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    strands_deferred_prepass_pipeline->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
    auto& push_constant_range = strands_deferred_prepass_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    strands_deferred_prepass_pipeline->Initialize();
  }
#endif
  if (!deferred_lighting_pass_pipeline) {
    deferred_lighting_pass_pipeline = std::make_shared<GraphicsPipeline>();
    deferred_lighting_pass_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                        "Shaders/Graphics/Vertex/TexturePassThrough.vert");
    deferred_lighting_pass_pipeline->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Fragment/Standard/StandardDeferredLighting.frag");
    deferred_lighting_pass_pipeline->geometry_type = GeometryType::Mesh;
    deferred_lighting_pass_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    deferred_lighting_pass_pipeline->descriptor_set_layouts.emplace_back(Camera::g_buffer_layout);
    deferred_lighting_pass_pipeline->descriptor_set_layouts.emplace_back(lighting_layout);
    deferred_lighting_pass_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    deferred_lighting_pass_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    deferred_lighting_pass_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    auto& push_constant_range = deferred_lighting_pass_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    deferred_lighting_pass_pipeline->Initialize();
  }
  if (!deferred_lighting_pass_pipeline_scene_camera) {
    deferred_lighting_pass_pipeline_scene_camera = std::make_shared<GraphicsPipeline>();
    deferred_lighting_pass_pipeline_scene_camera->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                        "Shaders/Graphics/Vertex/TexturePassThrough.vert");
    deferred_lighting_pass_pipeline_scene_camera->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/Fragment/Standard/StandardDeferredLightingSceneCamera.frag");
    deferred_lighting_pass_pipeline_scene_camera->geometry_type = GeometryType::Mesh;
    deferred_lighting_pass_pipeline_scene_camera->descriptor_set_layouts.emplace_back(per_frame_layout);
    deferred_lighting_pass_pipeline_scene_camera->descriptor_set_layouts.emplace_back(Camera::g_buffer_layout);
    deferred_lighting_pass_pipeline_scene_camera->descriptor_set_layouts.emplace_back(lighting_layout);
    deferred_lighting_pass_pipeline_scene_camera->depth_attachment_format = Platform::Constants::render_texture_depth;
    deferred_lighting_pass_pipeline_scene_camera->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    deferred_lighting_pass_pipeline_scene_camera->color_attachment_formats = {
        1, Platform::Constants::render_texture_color};
    auto& push_constant_range = deferred_lighting_pass_pipeline_scene_camera->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    deferred_lighting_pass_pipeline_scene_camera->Initialize();
  }
  if (!gizmos) {
    gizmos = std::make_shared<GraphicsPipeline>();
    gizmos->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/Gizmos.vert");
    gizmos->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Gizmos/Gizmos.frag");
    gizmos->geometry_type = GeometryType::Mesh;
    gizmos->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos->descriptor_set_layouts.emplace_back(per_frame_layout);
    auto& push_constant_range = gizmos->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos->Initialize();
  }
  if (!gizmos_normal_colored) {
    gizmos_normal_colored = std::make_shared<GraphicsPipeline>();
    gizmos_normal_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosNormalColored.vert");
    gizmos_normal_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.frag");
    gizmos_normal_colored->geometry_type = GeometryType::Mesh;
    gizmos_normal_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_normal_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_normal_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_normal_colored->descriptor_set_layouts.emplace_back(per_frame_layout);
    gizmos_normal_colored->tessellation_patch_control_points = 4;
    auto& push_constant_range = gizmos_normal_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    gizmos_normal_colored->Initialize();
  }
  if (!gizmos_vertex_colored) {
    gizmos_vertex_colored = std::make_shared<GraphicsPipeline>();
    gizmos_vertex_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosVertexColored.vert");
    gizmos_vertex_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.frag");
    gizmos_vertex_colored->geometry_type = GeometryType::Mesh;
    gizmos_vertex_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_vertex_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_vertex_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_vertex_colored->descriptor_set_layouts.emplace_back(per_frame_layout);
    auto& push_constant_range = gizmos_vertex_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    gizmos_vertex_colored->Initialize();
  }
  if (!gizmos_instanced_colored) {
    gizmos_instanced_colored = std::make_shared<GraphicsPipeline>();
    gizmos_instanced_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosInstancedColored.vert");
    gizmos_instanced_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.frag");
    gizmos_instanced_colored->geometry_type = GeometryType::Mesh;
    gizmos_instanced_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_instanced_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_instanced_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_instanced_colored->descriptor_set_layouts.emplace_back(per_frame_layout);
    gizmos_instanced_colored->descriptor_set_layouts.emplace_back(ParticleInfoList::instanced_data_layout);
    auto& push_constant_range = gizmos_instanced_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_instanced_colored->Initialize();
  }
#ifdef EVOENGINE_WINDOWS
  if (!gizmos_strands) {
    gizmos_strands = std::make_shared<GraphicsPipeline>();
    gizmos_strands->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosStrands.vert");
    gizmos_strands->tessellation_control_shader = Shader::CreateTemporary(
        ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/TessellationControl/Gizmos/GizmosStrands.tesc");
    gizmos_strands->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationEvaluation/Gizmos/GizmosStrands.tese");
    gizmos_strands->geometry_shader = Shader::CreateTemporary(
        ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Geometry/Gizmos/GizmosStrands.geom");
    gizmos_strands->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Gizmos/Gizmos.frag");
    gizmos_strands->geometry_type = GeometryType::Strands;
    gizmos_strands->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_strands->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_strands->tessellation_patch_control_points = 4;
    gizmos_strands->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_strands->descriptor_set_layouts.emplace_back(per_frame_layout);
    auto& push_constant_range = gizmos_strands->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_strands->Initialize();
  }
  if (!gizmos_strands_normal_colored) {
    gizmos_strands_normal_colored = std::make_shared<GraphicsPipeline>();
    gizmos_strands_normal_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosStrandsNormalColored.vert");
    gizmos_strands_normal_colored->tessellation_control_shader =
        Shader::CreateTemporary(ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationControl/Gizmos/GizmosStrandsColored.tesc");
    gizmos_strands_normal_colored->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationEvaluation/Gizmos/GizmosStrandsColored.tese");
    gizmos_strands_normal_colored->geometry_shader = Shader::CreateTemporary(
        ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Geometry/Gizmos/GizmosStrandsColored.geom");
    gizmos_strands_normal_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.frag");
    gizmos_strands_normal_colored->geometry_type = GeometryType::Strands;
    gizmos_strands_normal_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_strands_normal_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_strands_normal_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_strands_normal_colored->descriptor_set_layouts.emplace_back(per_frame_layout);
    gizmos_strands_normal_colored->tessellation_patch_control_points = 4;
    auto& push_constant_range = gizmos_strands_normal_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_strands_normal_colored->Initialize();
  }
  if (!gizmos_strands_vertex_colored) {
    gizmos_strands_vertex_colored = std::make_shared<GraphicsPipeline>();
    gizmos_strands_vertex_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Vertex/Gizmos/GizmosStrandsVertexColored.vert");
    gizmos_strands_vertex_colored->tessellation_control_shader =
        Shader::CreateTemporary(ShaderType::TessellationControl, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationControl/Gizmos/GizmosStrandsColored.tesc");
    gizmos_strands_vertex_colored->tessellation_evaluation_shader =
        Shader::CreateTemporary(ShaderType::TessellationEvaluation, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/Graphics/TessellationEvaluation/Gizmos/GizmosStrandsColored.tese");
    gizmos_strands_vertex_colored->geometry_shader = Shader::CreateTemporary(
        ShaderType::Geometry, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Geometry/Gizmos/GizmosStrandsColored.geom");
    gizmos_strands_vertex_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.frag");
    gizmos_strands_vertex_colored->geometry_type = GeometryType::Strands;
    gizmos_strands_vertex_colored->tessellation_patch_control_points = 4;
    gizmos_strands_vertex_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_strands_vertex_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_strands_vertex_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_strands_vertex_colored->descriptor_set_layouts.emplace_back(per_frame_layout);
    gizmos_strands_vertex_colored->tessellation_patch_control_points = 4;
    auto& push_constant_range = gizmos_strands_vertex_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    gizmos_strands_vertex_colored->Initialize();
  }

#endif
#pragma endregion
#pragma region Ray Tracing Pipelines
  if (Platform::RayTracingEnabled() && !ray_tracing_camera_pipeline) {
    ray_tracing_camera_pipeline = std::make_shared<RayTracingPipeline>();
    ray_tracing_camera_pipeline->raygen_shader =
        Shader::CreateTemporary(ShaderType::RayGen, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") / "Shaders/RayTracing/RayGen/Camera.rgen");
    ray_tracing_camera_pipeline->miss_shader =
        Shader::CreateTemporary(ShaderType::Miss, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") / "Shaders/RayTracing/Miss/Camera.rmiss");
    ray_tracing_camera_pipeline->closest_hit_shader = Shader::CreateTemporary(
        ShaderType::ClosestHit, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/RayTracing/ClosestHit/Camera.rchit");
    ray_tracing_camera_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    ray_tracing_camera_pipeline->descriptor_set_layouts.emplace_back(ray_tracing_layout);
    ray_tracing_camera_pipeline->descriptor_set_layouts.emplace_back(RenderTexture::render_texture_storage_layout);
    auto& push_constant_range = ray_tracing_camera_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RayTracingCameraPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    ray_tracing_camera_pipeline->Initialize();
  }
  if (Platform::RayTracingEnabled() && !ray_tracing_point_cloud_pipeline) {
    ray_tracing_point_cloud_pipeline = std::make_shared<RayTracingPipeline>();
    ray_tracing_point_cloud_pipeline->raygen_shader =
        Shader::CreateTemporary(ShaderType::RayGen, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/RayTracing/RayGen/PointCloud.rgen");
    ray_tracing_point_cloud_pipeline->miss_shader =
        Shader::CreateTemporary(ShaderType::Miss, Platform::GetShaderGlobalDefines(),
                                std::filesystem::path("./DefaultResources") /
                                    "Shaders/RayTracing/Miss/PointCloud.rmiss");
    ray_tracing_point_cloud_pipeline->closest_hit_shader = Shader::CreateTemporary(
        ShaderType::ClosestHit, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/RayTracing/ClosestHit/PointCloud.rchit");
    ray_tracing_point_cloud_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    ray_tracing_point_cloud_pipeline->descriptor_set_layouts.emplace_back(ray_tracing_layout);
    ray_tracing_point_cloud_pipeline->descriptor_set_layouts.emplace_back(ray_tracing_point_cloud_layout);
    auto& push_constant_range = ray_tracing_point_cloud_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RayTracingPointCloudPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    ray_tracing_point_cloud_pipeline->Initialize();
  }
#pragma endregion

  const auto max_frames_in_flight = Platform::GetMaxFramesInFlight();
  render_instances_list_.resize(max_frames_in_flight);
  for (auto& i : render_instances_list_) {
    i = std::make_shared<RenderInstanceStorage>();
  }
  kernel_descriptor_buffers_.clear();
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
  for (size_t i = 0; i < max_frame_in_flight; i++) {
    buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    buffer_create_info.size = sizeof(glm::vec4) * Platform::Constants::max_kernel_amount * 2;
    kernel_descriptor_buffers_.emplace_back(
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
  }
  per_frame_descriptor_sets_.clear();
  for (size_t i = 0; i < max_frames_in_flight; i++) {
    auto descriptor_set = std::make_shared<DescriptorSet>(per_frame_layout);
    per_frame_descriptor_sets_.emplace_back(descriptor_set);
  }

  meshlet_descriptor_sets_.clear();
  for (size_t i = 0; i < max_frames_in_flight; i++) {
    auto descriptor_set = std::make_shared<DescriptorSet>(meshlet_layout);
    meshlet_descriptor_sets_.emplace_back(descriptor_set);
  }

  ray_tracing_descriptor_sets_.clear();
  if (Platform::Constants::support_ray_tracing) {
    for (size_t i = 0; i < max_frames_in_flight; i++) {
      auto descriptor_set = std::make_shared<DescriptorSet>(ray_tracing_layout);
      ray_tracing_descriptor_sets_.emplace_back(descriptor_set);
    }
  }

  std::vector<glm::vec4> kernels;
  for (uint32_t i = 0; i < Platform::Constants::max_kernel_amount; i++) {
    kernels.emplace_back(glm::ballRand(1.0f), 1.0f);
  }
  for (uint32_t i = 0; i < Platform::Constants::max_kernel_amount; i++) {
    kernels.emplace_back(glm::gaussRand(0.0f, 1.0f), glm::gaussRand(0.0f, 1.0f), glm::gaussRand(0.0f, 1.0f),
                         glm::gaussRand(0.0f, 1.0f));
  }
  for (int i = 0; i < Platform::GetMaxFramesInFlight(); i++) {
    kernel_descriptor_buffers_[i]->UploadVector(kernels);
  }
  PrepareEnvironmentalBrdfLut();
  lighting_ = std::make_unique<Lighting>();
  lighting_->Initialize();
}

void RenderLayer::ClearAllEditorCameras() const {
  const auto scene = GetScene();
  if (!scene)
    return;
  std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>> cameras;
  RenderInstanceStorage::CollectEditorCameras(scene, cameras);

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (const auto& i : cameras) {
      if (i.second->prev_global_transform_ != i.first.value) {
        i.second->frame_count_ = 0;
        i.second->prev_global_transform_ = i.first.value;
      }
      if ((i.second->camera_render_mode == Camera::CameraRenderMode::Rasterization && i.second->rendered_) ||
          (i.second->camera_render_mode == Camera::CameraRenderMode::RayTracing && i.second->frame_count_ == 0)) {
        if (const auto render_texture = i.second->GetRenderTexture()) {
          render_texture->Clear(vk_command_buffer);
        }
      }
    }
  });
}

void RenderLayer::ClearAllCameras() const {
  const auto scene = GetScene();
  if (!scene)
    return;
  std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>> cameras;
  RenderInstanceStorage::CollectCameras(scene, cameras);

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (const auto& i : cameras) {
      if (i.second->prev_global_transform_ != i.first.value) {
        i.second->frame_count_ = 0;
        i.second->prev_global_transform_ = i.first.value;
      }
      if ((i.second->camera_render_mode == Camera::CameraRenderMode::Rasterization && i.second->rendered_) ||
          (i.second->camera_render_mode == Camera::CameraRenderMode::RayTracing && i.second->frame_count_ == 0)) {
        if (const auto render_texture = i.second->GetRenderTexture()) {
          render_texture->Clear(vk_command_buffer);
        }
      }
    }
  });
}

void RenderLayer::PrepareForRendering() {
  const auto scene = GetScene();
  if (!scene)
    return;

  // Sync mutable geometry/texture storages here so updates performed during
  // this frame's simulation/editor phase are visible to this frame's draw.
  GeometryStorage::DeviceSync();
  TextureStorage::DeviceSync();

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  auto& graphics = Platform::GetInstance();
  graphics.prim_count[current_frame_index] = 0;
  graphics.draw_call[current_frame_index] = 0;
  const auto current_render_instances = render_instances_list_[current_frame_index];
  ApplyAnimators();
  if (UpdateRenderInstanceStorage(scene, current_frame_index)) {
  }
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
      0, current_render_instances->render_info_descriptor_buffer);
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
      1, current_render_instances->environment_info_descriptor_buffer);
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
      2, current_render_instances->camera_info_descriptor_buffer);
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
      3, current_render_instances->material_info_descriptor_buffer);
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
      4, current_render_instances->instance_info_descriptor_buffer);
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
      5, kernel_descriptor_buffers_[current_frame_index]);
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
      6, current_render_instances->directional_light_info_descriptor_buffer);
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
      7, current_render_instances->point_light_info_descriptor_buffer);
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
      8, current_render_instances->spot_light_info_descriptor_buffer);

  meshlet_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(0, GeometryStorage::GetVertexBuffer());
  meshlet_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(1, GeometryStorage::GetMeshletBuffer());

  TextureStorage::BindTexture2DToDescriptorSet(per_frame_descriptor_sets_[current_frame_index], 9);
  TextureStorage::BindCubemapToDescriptorSet(per_frame_descriptor_sets_[current_frame_index], 10);

  if (Platform::RayTracingEnabled()) {
    bool needs_ray_tracing_tlas = false;
    for (const auto& camera_pair : current_render_instances->cameras) {
      const auto& camera = camera_pair.second;
      if (!camera || !camera->require_rendering_) {
        continue;
      }
      if (camera->camera_render_mode == Camera::CameraRenderMode::RayTracing) {
        needs_ray_tracing_tlas = true;
        break;
      }
    }

    if (needs_ray_tracing_tlas) {
      current_render_instances->UpdateTopLevelAccelerationStructure(scene);

      if (current_render_instances->mesh_top_level_acceleration_structure) {
        ray_tracing_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
            0, GeometryStorage::GetVertexBuffer());
        ray_tracing_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
            1, GeometryStorage::GetTriangleBuffer());
        ray_tracing_descriptor_sets_[current_frame_index]->UpdateAccelerationStructureDescriptorBinding(
            2, current_render_instances->mesh_top_level_acceleration_structure);
      }
    } else {
      current_render_instances->mesh_top_level_acceleration_structure.reset();
    }
  }
}

void RenderLayer::RenderAll() {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  PreparePointAndSpotLightShadowMap();
  for (const auto& [cameraGlobalTransform, camera] : current_render_instances->cameras) {
    camera->rendered_ = false;
    if (camera->require_rendering_) {
      RenderToCamera(cameraGlobalTransform, camera);
    }
  }

  point_light_shadow_map_external_functions.clear();
  spot_light_shadow_map_external_functions.clear();
  directional_light_shadow_map_external_functions.clear();

  deferred_rendering_external_functions.clear();
  forward_rendering_external_functions.clear();

  const bool can_ray_trace = Platform::RayTracingEnabled() && current_render_instances->mesh_top_level_acceleration_structure;
  for (const auto& [cameraGlobalTransform, camera] : current_render_instances->cameras) {
    if (!camera->require_rendering_ || camera->camera_render_mode != Camera::CameraRenderMode::RayTracing) {
      continue;
    }
    if (can_ray_trace) {
      RenderToCameraRayTracing(cameraGlobalTransform, camera);
    } else {
      // Keep scene view valid when RT mode is selected but no acceleration structure exists.
      RenderToCamera(cameraGlobalTransform, camera, true);
    }
  }
}

void RenderLayer::RenderGizmos() const {
  if (const auto scene = GetScene(); !scene)
    return;
  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    const auto current_frame_index = Platform::GetCurrentFrameIndex();
    const auto current_render_instances = render_instances_list_[Platform::GetCurrentFrameIndex()];
    for (const auto& i : editor_layer->gizmo_mesh_tasks_) {
      if (editor_layer->editor_cameras_.find(i.editor_camera_component->GetHandle()) ==
          editor_layer->editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!");
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
          std::shared_ptr<GraphicsPipeline> gizmos_pipeline;
          switch (i.gizmo_settings.color_mode) {
            case GizmoSettings::ColorMode::Default: {
              gizmos_pipeline = gizmos;
            } break;
            case GizmoSettings::ColorMode::VertexColor: {
              gizmos_pipeline = gizmos_vertex_colored;
            } break;
            case GizmoSettings::ColorMode::NormalColor: {
              gizmos_pipeline = gizmos_normal_colored;
            } break;
          }
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);

          gizmos_pipeline->Bind(vk_command_buffer);
          gizmos_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&]() {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = i.color;
                push_constant.size = i.size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                gizmos_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindVertices(vk_command_buffer);
                i.mesh->DrawIndexed(vk_command_buffer, gizmos_pipeline->states, 1);
              });
        });
      }
    }
    for (const auto& i : editor_layer->gizmo_instanced_mesh_tasks_) {
      if (editor_layer->editor_cameras_.find(i.editor_camera_component->GetHandle()) ==
          editor_layer->editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!")
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_instanced_colored->states);
          i.gizmo_settings.ApplySettings(gizmos_instanced_colored->states);

          gizmos_instanced_colored->Bind(vk_command_buffer);
          gizmos_instanced_colored->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          gizmos_instanced_colored->BindDescriptorSet(vk_command_buffer, 1,
                                                      i.particle_info_list->GetDescriptorSet()->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = glm::vec4(0.0f);
                push_constant.size = i.size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                gizmos_instanced_colored->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindVertices(vk_command_buffer);
                i.mesh->DrawIndexed(vk_command_buffer, gizmos_instanced_colored->states,
                                    i.particle_info_list->PeekParticleInfoList().size());
              });
        });
      }
    }
#ifdef EVOENGINE_WINDOWS
    for (const auto& i : editor_layer->gizmo_strands_tasks_) {
      if (editor_layer->editor_cameras_.find(i.editor_camera_component->GetHandle()) ==
          editor_layer->editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!");
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
          std::shared_ptr<GraphicsPipeline> gizmos_pipeline;
          switch (i.gizmo_settings.color_mode) {
            case GizmoSettings::ColorMode::Default: {
              gizmos_pipeline = gizmos_strands;
            } break;
            case GizmoSettings::ColorMode::VertexColor: {
              gizmos_pipeline = gizmos_strands_vertex_colored;
            } break;
            case GizmoSettings::ColorMode::NormalColor: {
              gizmos_pipeline = gizmos_strands_normal_colored;
            } break;
          }
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);

          gizmos_pipeline->Bind(vk_command_buffer);
          gizmos_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = i.color;
                push_constant.size = i.size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                gizmos_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindStrandPoints(vk_command_buffer);
                i.strands->DrawIndexed(vk_command_buffer, gizmos_pipeline->states, 1);
              });
        });
      }
    }
#endif
  }
}

void RenderLayer::ForEachCollectedCamera(
    const std::function<void(const std::shared_ptr<Camera>& camera)>& action) const {
  const auto current_render_instances = render_instances_list_[Platform::GetCurrentFrameIndex()];
  for (const auto& camera : current_render_instances->cameras)
    action(camera.second);
}

std::shared_ptr<RenderInstanceStorage> RenderLayer::GetCurrentRenderInstanceStorage() const {
  const auto index = Platform::GetCurrentFrameIndex();
  if (index >= render_instances_list_.size())
    return {};
  return render_instances_list_[index];
}

std::shared_ptr<RenderInstanceStorage> RenderLayer::GetPreviousRenderInstanceStorage() const {
  const auto index =
      (Platform::GetMaxFramesInFlight() + Platform::GetCurrentFrameIndex() - 1) % Platform::GetMaxFramesInFlight();
  if (index >= render_instances_list_.size())
    return {};
  return render_instances_list_[index];
}

void RenderLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Count shadows drawcalls", &count_shadow_rendering_draw_calls);
  ImGui::Checkbox("Wireframe", &wire_frame);
  if (Platform::MeshShaderEnabled())
    ImGui::Checkbox("Meshlet", &enable_meshlet);
  ImGui::Checkbox("Indirect Rendering", &enable_indirect_rendering);
  ImGui::Checkbox("Needle state audit", &debug_needle_state_audit);
  ImGui::Checkbox("Needle hard reset", &debug_needle_hard_reset);
  ImGui::DragInt("Needle max logs/frame", &debug_needle_max_logs_per_frame, 1.0f, 1, 256);
  debug_needle_max_logs_per_frame = std::max(1, debug_needle_max_logs_per_frame);
  ImGui::Checkbox("Stem state audit", &debug_stem_state_audit);
  ImGui::DragInt("Stem max logs/frame", &debug_stem_max_logs_per_frame, 1.0f, 1, 64);
  debug_stem_max_logs_per_frame = std::max(1, debug_stem_max_logs_per_frame);
  render_settings.OnInspect(editor_layer);
}

void RenderLayer::ApplyAnimators() const {
  const auto scene = GetScene();
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Animator>()) {
    Jobs::RunParallelFor(owners->size(), [&](unsigned i) {
      const auto entity = owners->at(i);
      if (!scene->IsEntityEnabled(entity))
        return;
      const auto animator = scene->GetOrSetPrivateComponent<Animator>(owners->at(i)).lock();
      if (!animator->IsEnabled())
        return;
      animator->Apply();
    });
  }
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SkinnedMeshRenderer>()) {
    Jobs::RunParallelFor(owners->size(), [&](unsigned i) {
      const auto entity = owners->at(i);
      if (!scene->IsEntityEnabled(entity))
        return;
      const auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(entity).lock();
      if (!skinned_mesh_renderer->IsEnabled())
        return;
      skinned_mesh_renderer->UpdateBoneMatrices();
    });
    for (const auto& i : *owners) {
      if (!scene->IsEntityEnabled(i))
        return;
      const auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(i).lock();
      if (!skinned_mesh_renderer->IsEnabled())
        return;
      skinned_mesh_renderer->UpdateBoneMatrices();
      skinned_mesh_renderer->bone_matrices->UploadData();
    }
  }
}

void RenderLayer::PreparePointAndSpotLightShadowMap() const {
  const bool count_draw_calls = count_shadow_rendering_draw_calls;
  const bool use_mesh_shader = Platform::MeshShaderEnabled() && enable_meshlet;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto& point_light_shadow_pipeline =
      use_mesh_shader ? point_light_shadow_pipeline_mesh_shader : point_light_shadow_pipeline_normal;
  const auto& spot_light_shadow_pipeline =
      use_mesh_shader ? spot_light_shadow_pipeline_mesh_shader : spot_light_shadow_pipeline_normal;
  auto& platform = Platform::GetInstance();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    const auto prepare_graphics_pipeline = [&](const std::shared_ptr<GraphicsPipeline>& target_pipeline,
                                               const glm::ivec4& view_port) {
      target_pipeline->states.ResetAllStates(0);
      target_pipeline->Bind(vk_command_buffer);
      target_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
      target_pipeline->states.SetViewportScissor(view_port);
    };

    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = lighting_->point_light_shadow_map_->GetExtent().width;
    render_area.extent.height = lighting_->point_light_shadow_map_->GetExtent().height;
    lighting_->point_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);

    for (int face = 0; face < 6; face++) {
      VkRenderingInfo render_info{};
      auto depth_attachment = lighting_->GetLayeredPointLightDepthAttachmentInfo(face, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                                                 VK_ATTACHMENT_STORE_OP_STORE);
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 0;
      render_info.pColorAttachments = nullptr;
      render_info.pDepthAttachment = &depth_attachment;
      Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
        for (int i = 0; i < current_render_instances->point_light_info_blocks_.size(); i++) {
          GeometryStorage::BindVertices(vk_command_buffer);
          {
            prepare_graphics_pipeline(point_light_shadow_pipeline,
                                      current_render_instances->point_light_info_blocks_[i].viewport);
            if (use_mesh_shader) {
              point_light_shadow_pipeline->BindDescriptorSet(
                  vk_command_buffer, 1, meshlet_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
            }
            if (enable_indirect_rendering && !current_render_instances->deferred_render_instances->Empty()) {
              RenderInstancePushConstant push_constant;
              push_constant.camera_index = i;
              push_constant.light_split_index = face;
              push_constant.instance_index = 0;
              point_light_shadow_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
              point_light_shadow_pipeline->states.ApplyAllStates(vk_command_buffer);
              if (count_draw_calls)
                platform.draw_call[current_frame_index]++;
              if (count_draw_calls)
                platform.prim_count[current_frame_index] += current_render_instances->total_mesh_triangles;
              if (use_mesh_shader) {
                vkCmdDrawMeshTasksIndirectEXT(
                    vk_command_buffer,
                    current_render_instances->mesh_draw_mesh_tasks_indirect_commands_buffer->GetVkBuffer(), 0,
                    current_render_instances->mesh_draw_mesh_tasks_indirect_commands.size(),
                    sizeof(VkDrawMeshTasksIndirectCommandEXT));
              } else {
                vkCmdDrawIndexedIndirect(
                    vk_command_buffer,
                    current_render_instances->mesh_draw_indexed_indirect_commands_buffer->GetVkBuffer(), 0,
                    current_render_instances->mesh_draw_indexed_indirect_commands.size(),
                    sizeof(VkDrawIndexedIndirectCommand));
              }
            } else {
              current_render_instances->deferred_render_instances->ForEachRenderInstance(
                  [&](const auto& render_instance) {
                    if (!render_instance->cast_shadow)
                      return;
                    RenderInstancePushConstant push_constant;
                    push_constant.camera_index = i;
                    push_constant.light_split_index = face;
                    push_constant.instance_index = render_instance->instance_index;
                    const auto prim_count =
                        render_instance->Render(vk_command_buffer, push_constant, point_light_shadow_pipeline);
                    if (count_draw_calls) {
                      platform.draw_call[current_frame_index]++;
                      platform.prim_count[current_frame_index] += prim_count;
                    }
                  });
            }
          }
          {
            prepare_graphics_pipeline(instanced_point_light_shadow_pipeline,
                                      current_render_instances->point_light_info_blocks_[i].viewport);
            current_render_instances->deferred_instanced_render_instances->ForEachRenderInstance(
                [&](const auto& render_instance) {
                  if (!render_instance->cast_shadow)
                    return;
                  RenderInstancePushConstant push_constant;
                  push_constant.camera_index = i;
                  push_constant.light_split_index = face;
                  push_constant.instance_index = render_instance->instance_index;
                  const auto prim_count =
                      render_instance->Render(vk_command_buffer, push_constant, instanced_point_light_shadow_pipeline);
                  if (count_draw_calls) {
                    platform.draw_call[current_frame_index]++;
                    platform.prim_count[current_frame_index] += prim_count;
                  }
                });
          }
          GeometryStorage::BindSkinnedVertices(vk_command_buffer);
          {
            prepare_graphics_pipeline(skinned_point_light_shadow_pipeline,
                                      current_render_instances->point_light_info_blocks_[i].viewport);
            current_render_instances->deferred_skinned_render_instances->ForEachRenderInstance(
                [&](const auto& render_instance) {
                  if (!render_instance->cast_shadow)
                    return;
                  RenderInstancePushConstant push_constant;
                  push_constant.camera_index = i;
                  push_constant.light_split_index = face;
                  push_constant.instance_index = render_instance->instance_index;
                  const auto prim_count =
                      render_instance->Render(vk_command_buffer, push_constant, skinned_point_light_shadow_pipeline);
                  if (count_draw_calls) {
                    platform.draw_call[current_frame_index]++;
                    platform.prim_count[current_frame_index] += prim_count;
                  }
                });
          }
#ifdef EVOENGINE_WINDOWS
          GeometryStorage::BindStrandPoints(vk_command_buffer);
          {
            prepare_graphics_pipeline(strands_point_light_shadow_pipeline,
                                      current_render_instances->point_light_info_blocks_[i].viewport);
            current_render_instances->deferred_strands_render_instances->ForEachRenderInstance(
                [&](const auto& render_instance) {
                  if (!render_instance->cast_shadow)
                    return;
                  RenderInstancePushConstant push_constant;
                  push_constant.camera_index = i;
                  push_constant.light_split_index = face;
                  push_constant.instance_index = render_instance->instance_index;
                  const auto prim_count =
                      render_instance->Render(vk_command_buffer, push_constant, strands_point_light_shadow_pipeline);
                  if (count_draw_calls) {
                    platform.draw_call[current_frame_index]++;
                    platform.prim_count[current_frame_index] += prim_count;
                  }
                });
          }
#endif
          for (const auto& func : point_light_shadow_map_external_functions) {
            const auto prim_count =
                func(vk_command_buffer, {i, face, current_render_instances->point_light_info_blocks_[i].viewport});
            if (count_draw_calls) {
              platform.draw_call[current_frame_index]++;
              platform.prim_count[current_frame_index] += prim_count;
            }
          }
        }
      });
    }
#pragma region Viewport and scissor

    render_area.offset = {0, 0};
    render_area.extent.width = lighting_->spot_light_shadow_map_->GetExtent().width;
    render_area.extent.height = lighting_->spot_light_shadow_map_->GetExtent().height;

#pragma endregion
    lighting_->spot_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
    VkRenderingInfo render_info{};
    const auto depth_attachment =
        lighting_->GetSpotLightDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
    render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info.renderArea = render_area;
    render_info.layerCount = 1;
    render_info.colorAttachmentCount = 0;
    render_info.pColorAttachments = nullptr;
    render_info.pDepthAttachment = &depth_attachment;
    Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
      for (int i = 0; i < current_render_instances->spot_light_info_blocks_.size(); i++) {
        GeometryStorage::BindVertices(vk_command_buffer);
        {
          prepare_graphics_pipeline(spot_light_shadow_pipeline,
                                    current_render_instances->spot_light_info_blocks_[i].viewport);
          if (use_mesh_shader) {
            spot_light_shadow_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, meshlet_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          }
          if (enable_indirect_rendering && !current_render_instances->deferred_render_instances->Empty()) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = i;
            push_constant.light_split_index = 0;
            push_constant.instance_index = 0;
            spot_light_shadow_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
            spot_light_shadow_pipeline->states.ApplyAllStates(vk_command_buffer);
            if (count_draw_calls)
              platform.draw_call[current_frame_index]++;
            if (count_draw_calls)
              platform.prim_count[current_frame_index] += current_render_instances->total_mesh_triangles;
            if (use_mesh_shader) {
              vkCmdDrawMeshTasksIndirectEXT(
                  vk_command_buffer,
                  current_render_instances->mesh_draw_mesh_tasks_indirect_commands_buffer->GetVkBuffer(), 0,
                  current_render_instances->mesh_draw_mesh_tasks_indirect_commands.size(),
                  sizeof(VkDrawMeshTasksIndirectCommandEXT));
            } else {
              vkCmdDrawIndexedIndirect(
                  vk_command_buffer,
                  current_render_instances->mesh_draw_indexed_indirect_commands_buffer->GetVkBuffer(), 0,
                  current_render_instances->mesh_draw_indexed_indirect_commands.size(),
                  sizeof(VkDrawIndexedIndirectCommand));
            }
          } else {
            current_render_instances->deferred_render_instances->ForEachRenderInstance(
                [&](const auto& render_instance) {
                  if (!render_instance->cast_shadow)
                    return;
                  RenderInstancePushConstant push_constant;
                  push_constant.camera_index = i;
                  push_constant.light_split_index = 0;
                  push_constant.instance_index = render_instance->instance_index;
                  const auto prim_count =
                      render_instance->Render(vk_command_buffer, push_constant, spot_light_shadow_pipeline);
                  if (count_draw_calls) {
                    platform.draw_call[current_frame_index]++;
                    platform.prim_count[current_frame_index] += prim_count;
                  }
                });
          }
        }
        {
          prepare_graphics_pipeline(instanced_spot_light_shadow_pipeline,
                                    current_render_instances->spot_light_info_blocks_[i].viewport);
          current_render_instances->deferred_instanced_render_instances->ForEachRenderInstance(
              [&](const auto& render_instance) {
                if (!render_instance->cast_shadow)
                  return;
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = i;
                push_constant.light_split_index = 0;
                push_constant.instance_index = render_instance->instance_index;
                const auto prim_count =
                    render_instance->Render(vk_command_buffer, push_constant, instanced_spot_light_shadow_pipeline);
                if (count_draw_calls) {
                  platform.draw_call[current_frame_index]++;
                  platform.prim_count[current_frame_index] += prim_count;
                }
              });
        }
        GeometryStorage::BindSkinnedVertices(vk_command_buffer);
        {
          prepare_graphics_pipeline(skinned_spot_light_shadow_pipeline,
                                    current_render_instances->spot_light_info_blocks_[i].viewport);
          current_render_instances->deferred_skinned_render_instances->ForEachRenderInstance(
              [&](const auto& render_instance) {
                if (!render_instance->cast_shadow)
                  return;
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = i;
                push_constant.light_split_index = 0;
                push_constant.instance_index = render_instance->instance_index;
                const auto prim_count =
                    render_instance->Render(vk_command_buffer, push_constant, skinned_spot_light_shadow_pipeline);
                if (count_draw_calls) {
                  platform.draw_call[current_frame_index]++;
                  platform.prim_count[current_frame_index] += prim_count;
                }
              });
        }
#ifdef EVOENGINE_WINDOWS
        GeometryStorage::BindStrandPoints(vk_command_buffer);
        {
          prepare_graphics_pipeline(strands_spot_light_shadow_pipeline,
                                    current_render_instances->spot_light_info_blocks_[i].viewport);
          current_render_instances->deferred_strands_render_instances->ForEachRenderInstance(
              [&](const auto& render_instance) {
                if (!render_instance->cast_shadow)
                  return;
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = i;
                push_constant.light_split_index = 0;
                push_constant.instance_index = render_instance->instance_index;
                const auto prim_count =
                    render_instance->Render(vk_command_buffer, push_constant, strands_spot_light_shadow_pipeline);
                if (count_draw_calls) {
                  platform.draw_call[current_frame_index]++;
                  platform.prim_count[current_frame_index] += prim_count;
                }
              });
        }
#endif
        for (const auto& func : spot_light_shadow_map_external_functions) {
          const auto prim_count =
              func(vk_command_buffer, {i, current_render_instances->spot_light_info_blocks_[i].viewport});
          if (count_draw_calls) {
            platform.draw_call[current_frame_index]++;
            platform.prim_count[current_frame_index] += prim_count;
          }
        }
      }
    });
    lighting_->point_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    lighting_->spot_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}

bool RenderLayer::UpdateRenderInstanceStorage(const std::shared_ptr<Scene>& scene, const uint32_t current_frame_index) {
  auto lod_center = glm::vec3(0.f);
  float lod_max_distance = FLT_MAX;
  bool lod_set = false;
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    if (const auto main_camera_owner = main_camera->GetOwner(); scene->IsEntityValid(main_camera_owner)) {
      lod_center = scene->GetDataComponent<GlobalTransform>(main_camera_owner).GetPosition();
      lod_max_distance = main_camera->camera_settings.far_distance;
      lod_set = true;
    }
  }
  if (!lod_set) {
    if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
      if (const auto scene_camera = editor_layer->GetSceneCamera()) {
        lod_center = editor_layer->GetSceneCameraPosition();
        lod_max_distance = scene_camera->camera_settings.far_distance;
      }
    }
  }
  RenderInstanceStorage::CalculateLodFactor(scene, lod_center, lod_max_distance);
  auto world_bound = scene->GetBound();
  need_fade_ = false;
  const auto current_render_instances = render_instances_list_[current_frame_index];
  current_render_instances->BuildFromScene(render_settings, scene, world_bound);
  const bool render_instance_updated =
      *current_render_instances !=
      *render_instances_list_[(current_frame_index + Platform::GetMaxFramesInFlight() - 1) %
                              Platform::GetMaxFramesInFlight()];
  // if (render_instance_updated) {
  current_render_instances->Upload();
  //}
  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    if (scene->IsEntityValid(editor_layer->GetSelectedEntity())) {
      for (const auto& i : current_render_instances->instance_info_blocks_) {
        if (i.info_index & 1) {
          need_fade_ = true;
        }
      }
    }
    editor_layer->MouseEntitySelection();
  }
  if (render_instance_updated) {
    world_bound.min -= glm::vec3(0.1f);
    world_bound.max += glm::vec3(0.1f);
    scene->SetBound(world_bound);
    for (const auto& [cameraGlobalTransform, camera] : current_render_instances->cameras) {
      camera->frame_count_ = 0;
    }
  }
  return render_instance_updated;
}

void RenderLayer::PrepareEnvironmentalBrdfLut() {
  environmental_brdf_lut_.reset();
  environmental_brdf_lut_ = AssetManager::CreateTemporaryAsset<Texture2D>();
  auto& environmental_brdf_lut_texture_storage = environmental_brdf_lut_->RefTexture2DStorage();
  constexpr auto brdf_lut_resolution = 512;
  {
    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.extent.width = brdf_lut_resolution;
    image_info.extent.height = brdf_lut_resolution;
    image_info.extent.depth = 1;
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.format = VK_FORMAT_R16G16_SFLOAT;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    environmental_brdf_lut_texture_storage.image = std::make_unique<Image>(image_info);

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = environmental_brdf_lut_->GetVkImage();
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = VK_FORMAT_R16G16_SFLOAT;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 1;

    environmental_brdf_lut_texture_storage.image_view = std::make_unique<ImageView>(view_info);

    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.anisotropyEnable = VK_TRUE;
    sampler_info.maxAnisotropy = Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;
    sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampler_info.unnormalizedCoordinates = VK_FALSE;
    sampler_info.compareEnable = VK_FALSE;
    sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

    environmental_brdf_lut_texture_storage.sampler = std::make_unique<Sampler>(sampler_info);
  }
  const auto environmental_brdf_pipeline = std::make_shared<GraphicsPipeline>();
  environmental_brdf_pipeline->vertex_shader =
      Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                      "Shaders/Graphics/Vertex/TexturePassThrough.vert");
  environmental_brdf_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, std::filesystem::path("./DefaultResources") /
                                                        "Shaders/Graphics/Fragment/Lighting/EnvironmentalMapBrdf.frag");
  environmental_brdf_pipeline->geometry_type = GeometryType::Mesh;
  environmental_brdf_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  environmental_brdf_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  environmental_brdf_pipeline->color_attachment_formats = {1, VK_FORMAT_R16G16_SFLOAT};
  environmental_brdf_pipeline->Initialize();

  Platform::ImmediateSubmit([&](VkCommandBuffer vk_command_buffer) {
    environmental_brdf_lut_texture_storage.image->TransitImageLayout(vk_command_buffer,
                                                                     VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = brdf_lut_resolution;
    render_area.extent.height = brdf_lut_resolution;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = brdf_lut_resolution;
    viewport.height = brdf_lut_resolution;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = brdf_lut_resolution;
    scissor.extent.height = brdf_lut_resolution;
    environmental_brdf_pipeline->states.view_port = viewport;
    environmental_brdf_pipeline->states.scissor = scissor;
#pragma endregion
#pragma region Lighting pass
    {
      VkRenderingAttachmentInfo attachment{};
      attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

      attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
      attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
      attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

      attachment.clearValue = {0, 0, 0, 1};
      attachment.imageView = environmental_brdf_lut_texture_storage.image_view->GetVkImageView();

      VkRenderingInfo render_info{};
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 1;
      render_info.pColorAttachments = &attachment;
      environmental_brdf_pipeline->states.depth_test = false;
      environmental_brdf_pipeline->states.color_blend_attachment_states.clear();
      environmental_brdf_pipeline->states.color_blend_attachment_states.resize(1);
      for (auto& i : environmental_brdf_pipeline->states.color_blend_attachment_states) {
        i.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT;
        i.blendEnable = VK_FALSE;
      }
      Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
        environmental_brdf_pipeline->Bind(vk_command_buffer);
        const auto mesh = Resources::texture_pass_through_quad;
        GeometryStorage::BindVertices(vk_command_buffer);
        mesh->DrawIndexed(vk_command_buffer, environmental_brdf_pipeline->states, 1);
      });
#pragma endregion
    }
    environmental_brdf_lut_texture_storage.image->TransitImageLayout(vk_command_buffer,
                                                                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}
void RenderLayer::RenderToCamera(const GlobalTransform& camera_global_transform,
                                 const std::shared_ptr<Camera>& camera,
                                 const bool force_rasterization_fallback) const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  const int camera_index = current_render_instances->GetCameraIndex(camera->GetHandle());
  const auto scene = Application::GetActiveScene();
  if (camera->camera_render_mode == Camera::CameraRenderMode::Rasterization || force_rasterization_fallback) {
    const auto& graphics_settings = Application::GetApplicationInfo().graphics_settings;

    const bool count_draw_calls = count_shadow_rendering_draw_calls;
    const bool use_mesh_shader = Platform::MeshShaderEnabled() && enable_meshlet;
#pragma region Directional Light Shadows
    const auto& directional_light_shadow_pipeline =
        use_mesh_shader ? directional_light_shadow_pipeline_mesh_shader : directional_light_shadow_pipeline_normal;
    auto& platform = Platform::GetInstance();
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
      VkRect2D render_area;
      render_area.offset = {0, 0};
      render_area.extent.width = lighting_->directional_light_shadow_map_->GetExtent().width;
      render_area.extent.height = lighting_->directional_light_shadow_map_->GetExtent().height;
#pragma endregion
      lighting_->directional_light_shadow_map_->TransitImageLayout(vk_command_buffer,
                                                                   VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
      for (int split = 0; split < 4; split++) {
        const auto depth_attachment = lighting_->GetLayeredDirectionalLightDepthAttachmentInfo(
            split, VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
        VkRenderingInfo render_info{};
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 0;
        render_info.pColorAttachments = nullptr;
        render_info.pDepthAttachment = &depth_attachment;
        Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
          if (use_mesh_shader) {
            directional_light_shadow_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, meshlet_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          }
          for (int i = 0; i < current_render_instances->render_info_block.directional_light_size; i++) {
            const auto& directional_light_info_block =
                current_render_instances
                    ->directional_light_info_blocks_[camera_index * graphics_settings.max_directional_light_size + i];
            const auto prepare_graphics_pipeline = [&](const std::shared_ptr<GraphicsPipeline>& target_pipeline) {
              target_pipeline->states.ResetAllStates(0);
              target_pipeline->Bind(vk_command_buffer);
              target_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                 per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
              target_pipeline->states.SetViewportScissor(directional_light_info_block.viewport);
            };
            GeometryStorage::BindVertices(vk_command_buffer);
            {
              prepare_graphics_pipeline(directional_light_shadow_pipeline);
              if (enable_indirect_rendering && !current_render_instances->deferred_render_instances->Empty()) {
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = camera_index * graphics_settings.max_directional_light_size + i;
                push_constant.light_split_index = split;
                push_constant.instance_index = 0;
                directional_light_shadow_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                directional_light_shadow_pipeline->states.ApplyAllStates(vk_command_buffer);
                if (count_draw_calls)
                  platform.draw_call[current_frame_index]++;
                if (count_draw_calls)
                  platform.prim_count[current_frame_index] += current_render_instances->total_mesh_triangles;
                if (use_mesh_shader) {
                  vkCmdDrawMeshTasksIndirectEXT(
                      vk_command_buffer,
                      current_render_instances->mesh_draw_mesh_tasks_indirect_commands_buffer->GetVkBuffer(), 0,
                      current_render_instances->mesh_draw_mesh_tasks_indirect_commands.size(),
                      sizeof(VkDrawMeshTasksIndirectCommandEXT));
                } else {
                  vkCmdDrawIndexedIndirect(
                      vk_command_buffer,
                      current_render_instances->mesh_draw_indexed_indirect_commands_buffer->GetVkBuffer(), 0,
                      current_render_instances->mesh_draw_indexed_indirect_commands.size(),
                      sizeof(VkDrawIndexedIndirectCommand));
                }
              } else {
                current_render_instances->deferred_render_instances->ForEachRenderInstance(
                    [&](const auto& render_instance) {
                      if (!render_instance->cast_shadow)
                        return;
                      RenderInstancePushConstant push_constant;
                      push_constant.camera_index = camera_index * graphics_settings.max_directional_light_size + i;
                      push_constant.light_split_index = split;
                      push_constant.instance_index = render_instance->instance_index;
                      const auto prim_count =
                          render_instance->Render(vk_command_buffer, push_constant, directional_light_shadow_pipeline);
                      if (count_draw_calls) {
                        platform.draw_call[current_frame_index]++;
                        platform.prim_count[current_frame_index] += prim_count;
                      }
                    });
              }
            }
            {
              prepare_graphics_pipeline(instanced_directional_light_shadow_pipeline);
              current_render_instances->deferred_instanced_render_instances->ForEachRenderInstance(
                  [&](const auto& render_instance) {
                    if (!render_instance->cast_shadow)
                      return;
                    RenderInstancePushConstant push_constant;
                    push_constant.camera_index = camera_index * graphics_settings.max_directional_light_size + i;
                    push_constant.light_split_index = split;
                    push_constant.instance_index = render_instance->instance_index;
                    const auto prim_count = render_instance->Render(vk_command_buffer, push_constant,
                                                                    instanced_directional_light_shadow_pipeline);
                    if (count_draw_calls) {
                      platform.draw_call[current_frame_index]++;
                      platform.prim_count[current_frame_index] += prim_count;
                    }
                  });
            }
            GeometryStorage::BindSkinnedVertices(vk_command_buffer);
            {
              prepare_graphics_pipeline(skinned_directional_light_shadow_pipeline);
              current_render_instances->deferred_skinned_render_instances->ForEachRenderInstance(
                  [&](const auto& render_instance) {
                    if (!render_instance->cast_shadow)
                      return;
                    RenderInstancePushConstant push_constant;
                    push_constant.camera_index = camera_index * graphics_settings.max_directional_light_size + i;
                    push_constant.light_split_index = split;
                    push_constant.instance_index = render_instance->instance_index;
                    const auto prim_count = render_instance->Render(vk_command_buffer, push_constant,
                                                                    skinned_directional_light_shadow_pipeline);
                    if (count_draw_calls) {
                      platform.draw_call[current_frame_index]++;
                      platform.prim_count[current_frame_index] += prim_count;
                    }
                  });
            }
#ifdef EVOENGINE_WINDOWS
            GeometryStorage::BindStrandPoints(vk_command_buffer);
            {
              prepare_graphics_pipeline(strands_directional_light_shadow_pipeline);
              current_render_instances->deferred_strands_render_instances->ForEachRenderInstance(
                  [&](const auto& render_instance) {
                    if (!render_instance->cast_shadow)
                      return;
                    RenderInstancePushConstant push_constant;
                    push_constant.camera_index = camera_index * graphics_settings.max_directional_light_size + i;
                    push_constant.light_split_index = split;
                    push_constant.instance_index = render_instance->instance_index;
                    const auto prim_count = render_instance->Render(vk_command_buffer, push_constant,
                                                                    strands_directional_light_shadow_pipeline);
                    if (count_draw_calls) {
                      platform.draw_call[current_frame_index]++;
                      platform.prim_count[current_frame_index] += prim_count;
                    }
                  });
            }
#endif
            for (const auto& func : directional_light_shadow_map_external_functions) {
              const auto prim_count = func(
                  vk_command_buffer, {i, split, current_render_instances->directional_light_info_blocks_[i].viewport});
              if (count_draw_calls) {
                platform.draw_call[current_frame_index]++;
                platform.prim_count[current_frame_index] += prim_count;
              }
            }
          }
        });
      }
    });

#pragma endregion
    const auto editor_layer = Application::GetLayer<EditorLayer>();
    bool is_scene_camera = false;
    bool need_fade = false;
    if (editor_layer) {
      if (camera.get() == editor_layer->GetSceneCamera().get())
        is_scene_camera = true;
      if (need_fade_ && editor_layer->highlight_selection_)
        need_fade = true;
    }

    Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
      VkRect2D render_area;
      render_area.offset = {0, 0};
      render_area.extent.width = camera->GetSize().x;
      render_area.extent.height = camera->GetSize().y;
      glm::ivec4 view_port;
      view_port.x = 0.0f;
      view_port.y = 0.0f;
      view_port.z = camera->GetSize().x;
      view_port.w = camera->GetSize().y;

      camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
      camera->render_texture_->GetDepthImage()->TransitImageLayout(vk_command_buffer,
                                                                   VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);

      VkRenderingInfo geometry_pass_render_info{};
      geometry_pass_render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      geometry_pass_render_info.renderArea = render_area;
      geometry_pass_render_info.layerCount = 1;
#pragma endregion
#pragma region Deferred Rendering
#pragma region Geometry pass

      const auto geometry_pass_depth_attachment =
          camera->render_texture_->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      geometry_pass_render_info.pDepthAttachment = &geometry_pass_depth_attachment;
      std::vector<VkRenderingAttachmentInfo> geometry_pass_color_attachment_infos;
      camera->AppendGBufferColorAttachmentInfos(geometry_pass_color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                VK_ATTACHMENT_STORE_OP_STORE);
      geometry_pass_render_info.colorAttachmentCount = geometry_pass_color_attachment_infos.size();
      geometry_pass_render_info.pColorAttachments = geometry_pass_color_attachment_infos.data();
      Platform::RecordRenderCommands(geometry_pass_render_info, vk_command_buffer, [&]() {
        GeometryStorage::BindVertices(vk_command_buffer);
        {
          const auto& deferred_prepass_pipeline =
              use_mesh_shader ? deferred_prepass_pipeline_mesh : deferred_prepass_pipeline_normal;
          deferred_prepass_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
          deferred_prepass_pipeline->states.SetViewportScissor(view_port);
          deferred_prepass_pipeline->states.polygon_mode = wire_frame ? VK_POLYGON_MODE_LINE : VK_POLYGON_MODE_FILL;
          deferred_prepass_pipeline->Bind(vk_command_buffer);
          deferred_prepass_pipeline->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          if (use_mesh_shader) {
            deferred_prepass_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, meshlet_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          }
          if (enable_indirect_rendering && !current_render_instances->deferred_render_instances->Empty()) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = camera_index;
            push_constant.instance_index = 0;
            deferred_prepass_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
            deferred_prepass_pipeline->states.ApplyAllStates(vk_command_buffer);
            if (count_draw_calls)
              platform.draw_call[current_frame_index]++;
            if (count_draw_calls)
              platform.prim_count[current_frame_index] += current_render_instances->total_mesh_triangles;
            if (use_mesh_shader) {
              vkCmdDrawMeshTasksIndirectEXT(
                  vk_command_buffer,
                  current_render_instances->mesh_draw_mesh_tasks_indirect_commands_buffer->GetVkBuffer(), 0,
                  current_render_instances->mesh_draw_mesh_tasks_indirect_commands.size(),
                  sizeof(VkDrawMeshTasksIndirectCommandEXT));
            } else {
              vkCmdDrawIndexedIndirect(
                  vk_command_buffer,
                  current_render_instances->mesh_draw_indexed_indirect_commands_buffer->GetVkBuffer(), 0,
                  current_render_instances->mesh_draw_indexed_indirect_commands.size(),
                  sizeof(VkDrawIndexedIndirectCommand));
            }
          } else {
            current_render_instances->deferred_render_instances->ForEachRenderInstance(
                [&](const auto& render_instance) {
                  RenderInstancePushConstant push_constant;
                  push_constant.camera_index = camera_index;
                  push_constant.instance_index = render_instance->instance_index;
                  deferred_prepass_pipeline->states.polygon_mode =
                      wire_frame ? VK_POLYGON_MODE_LINE : render_instance->polygon_mode;
                  deferred_prepass_pipeline->states.cull_mode = render_instance->cull_mode;
                  deferred_prepass_pipeline->states.line_width = render_instance->line_width;
                  const auto prim_count =
                      render_instance->Render(vk_command_buffer, push_constant, deferred_prepass_pipeline);
                  if (count_draw_calls) {
                    platform.draw_call[current_frame_index]++;
                    platform.prim_count[current_frame_index] += prim_count;
                  }
                });
          }
        }
        {
          instanced_deferred_prepass_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
          instanced_deferred_prepass_pipeline->states.SetViewportScissor(view_port);
          instanced_deferred_prepass_pipeline->Bind(vk_command_buffer);
          instanced_deferred_prepass_pipeline->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());

          const auto current_frame = Platform::GetFrameCount();
          static uint32_t last_debug_frame = std::numeric_limits<uint32_t>::max();
          static int debug_logs_this_frame = 0;
          static int debug_stem_logs_this_frame = 0;
          if (last_debug_frame != current_frame) {
            last_debug_frame = current_frame;
            debug_logs_this_frame = 0;
            debug_stem_logs_this_frame = 0;
          }

          const auto to_lower = [](std::string value) {
            std::transform(value.begin(), value.end(), value.begin(),
                           [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return value;
          };

          const auto maybe_get_needle_instance =
              [&](const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance)
              -> std::shared_ptr<RenderInstanceStorage::InstancedRenderInstance> {
            const auto instanced_instance =
                std::dynamic_pointer_cast<RenderInstanceStorage::InstancedRenderInstance>(render_instance);
            if (!instanced_instance || !instanced_instance->material) {
              return {};
            }
            // Audit needle geometry only so log budget captures all needle
            // instances in grid tests (internodes can otherwise consume slots).
            if (!scene || !scene->IsEntityValid(instanced_instance->owner)) {
              return {};
            }
            const auto entity_name_lower = to_lower(scene->GetEntityName(instanced_instance->owner));
            if (entity_name_lower.find("needle") != std::string::npos) {
              return instanced_instance;
            }
            // [deprecated] Catch-all return removed; was: return instanced_instance;
            return {};
          };

          const auto maybe_get_stem_instance =
              [&](const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance)
              -> std::shared_ptr<RenderInstanceStorage::InstancedRenderInstance> {
            const auto instanced_instance =
                std::dynamic_pointer_cast<RenderInstanceStorage::InstancedRenderInstance>(render_instance);
            if (!instanced_instance || !instanced_instance->material) {
              return {};
            }
            if (!scene || !scene->IsEntityValid(instanced_instance->owner)) {
              return {};
            }
            const auto entity_name_lower = to_lower(scene->GetEntityName(instanced_instance->owner));
            if (entity_name_lower.find("internode") != std::string::npos ||
                entity_name_lower.find("stem") != std::string::npos) {
              return instanced_instance;
            }
            return {};
          };

          const auto log_needle_draw_audit =
              [&](const std::shared_ptr<RenderInstanceStorage::InstancedRenderInstance>& instanced_instance,
                  const RenderInstancePushConstant& push_constant) {
            if (!debug_needle_state_audit || !instanced_instance ||
                debug_logs_this_frame >= debug_needle_max_logs_per_frame) {
              return;
            }
            std::string entity_name = "<api>";
            if (scene && scene->IsEntityValid(instanced_instance->owner)) {
              entity_name = scene->GetEntityName(instanced_instance->owner);
            }

            const auto& material_blocks = current_render_instances->GetMaterialInfoBlocks();
            const auto& instance_blocks = current_render_instances->GetInstanceInfoBlocks();
            const int debug_visualization_mode = current_render_instances->render_info_block.debug_visualization;

            const int material_index = instanced_instance->material_index;
            const bool valid_material_index = material_index >= 0 && material_index < static_cast<int>(material_blocks.size());
            const bool valid_instance_index =
                push_constant.instance_index >= 0 && push_constant.instance_index < static_cast<int>(instance_blocks.size());

            int block_material_index = -1;
            int block_vertex_color_only = -1;
            float block_transmission = -1.0f;
            int block_albedo_texture_index = -999;
            float block_albedo_alpha = -1.0f;
            float block_roughness = -1.0f;
            if (valid_material_index) {
              block_vertex_color_only = material_blocks[material_index].vertex_color_only;
              block_transmission = material_blocks[material_index].transmission;
              block_albedo_texture_index = material_blocks[material_index].albedo_texture_index;
              block_albedo_alpha = material_blocks[material_index].albedo_color_val.a;
              block_roughness = material_blocks[material_index].roughness_val;
            }
            if (valid_instance_index) {
              block_material_index = instance_blocks[push_constant.instance_index].material_index;
            }

            const auto particle_count =
                instanced_instance->particle_infos
                    ? static_cast<int>(instanced_instance->particle_infos->PeekParticleInfoList().size())
                    : 0;

            VkDescriptorSet set0 = VK_NULL_HANDLE;
            VkDescriptorSet set1 = VK_NULL_HANDLE;
            if (per_frame_descriptor_sets_[current_frame_index]) {
              set0 = per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet();
            }
            if (instanced_instance->particle_infos) {
              set1 = instanced_instance->particle_infos->GetDescriptorSet()->GetVkDescriptorSet();
            }

            glm::vec4 first_instance_color(0.0f);
            if (instanced_instance->particle_infos) {
              const auto& particle_infos = instanced_instance->particle_infos->PeekParticleInfoList();
              if (!particle_infos.empty()) {
                first_instance_color = particle_infos.front().instance_color;
              }
            }

            glm::vec4 first_vertex_color(0.0f);
            // Iteration 4: sample mid and last vertex colors as well, plus
            // per-channel min/max across a stride of samples. If a needle
            // mesh's color variance is near-zero we have a per-pine vertex
            // color assignment bug in BuildNeedleAggregateMesh; if it's
            // healthy, the bug is purely transform/material-side.
            glm::vec4 mid_vertex_color(0.0f);
            glm::vec4 last_vertex_color(0.0f);
            glm::vec4 vc_min(1.0e9f);
            glm::vec4 vc_max(-1.0e9f);
            uint32_t vertex_total = 0u;
            uint32_t sampled = 0u;
            if (instanced_instance->mesh) {
              const auto& vertices = instanced_instance->mesh->UnsafeGetVertices();
              vertex_total = static_cast<uint32_t>(vertices.size());
              if (!vertices.empty()) {
                first_vertex_color = vertices.front().color;
                mid_vertex_color = vertices[vertices.size() / 2].color;
                last_vertex_color = vertices.back().color;
                // Stride-sample up to 64 vertices to estimate color spread
                // without blowing log/CPU cost.
                const uint32_t stride = std::max(1u, vertex_total / 64u);
                for (uint32_t vi = 0u; vi < vertex_total; vi += stride) {
                  const glm::vec4& c = vertices[vi].color;
                  vc_min = glm::min(vc_min, c);
                  vc_max = glm::max(vc_max, c);
                  ++sampled;
                }
              }
            }
            const glm::vec4 vc_range = (sampled > 0u) ? (vc_max - vc_min) : glm::vec4(0.0f);

            // Per-tree distinguishing fields: asset handles, geometry/particle
            // versions, owner entity index, model translation. Used to detect
            // shared GPU asset reuse across multiple needle entities.
            const uint64_t mesh_handle =
                instanced_instance->mesh ? static_cast<uint64_t>(instanced_instance->mesh->GetHandle().GetValue()) : 0ull;
            const uint64_t material_handle =
                instanced_instance->material ? static_cast<uint64_t>(instanced_instance->material->GetHandle().GetValue()) : 0ull;
            const uint64_t particle_handle =
                instanced_instance->particle_infos
                    ? static_cast<uint64_t>(instanced_instance->particle_infos->GetHandle().GetValue())
                    : 0ull;
            const uint32_t mesh_version =
                instanced_instance->mesh ? instanced_instance->mesh->GetVersion() : 0u;
            const uint32_t particle_version =
                instanced_instance->particle_infos ? instanced_instance->particle_infos->GetVersion() : 0u;
            const uint32_t owner_index = instanced_instance->owner.GetIndex();
            const glm::vec3 model_translation =
                glm::vec3(instanced_instance->model.value[3]);
            const uint32_t triangle_offset =
                (instanced_instance->mesh && instanced_instance->mesh->triangle_range_)
                    ? instanced_instance->mesh->triangle_range_->offset
                    : 0u;
            const uint32_t triangle_range_size =
                (instanced_instance->mesh && instanced_instance->mesh->triangle_range_)
                    ? instanced_instance->mesh->triangle_range_->range
                    : 0u;
            // Capture the values that actually drive vkCmdDrawIndexed inside
            // Mesh::DrawIndexed (uses prev_frame_*). If these drift from the
            // current offset/index_count we have a stale-snapshot bug; if the
            // tri_range_ptr is shared between two needle entities we have an
            // aliasing bug.
            const uint32_t triangle_index_count =
                (instanced_instance->mesh && instanced_instance->mesh->triangle_range_)
                    ? instanced_instance->mesh->triangle_range_->index_count
                    : 0u;
            const uint32_t triangle_prev_frame_offset =
                (instanced_instance->mesh && instanced_instance->mesh->triangle_range_)
                    ? instanced_instance->mesh->triangle_range_->prev_frame_offset
                    : 0u;
            const uint32_t triangle_prev_frame_index_count =
                (instanced_instance->mesh && instanced_instance->mesh->triangle_range_)
                    ? instanced_instance->mesh->triangle_range_->prev_frame_index_count
                    : 0u;
            const void* tri_range_ptr =
                instanced_instance->mesh ? static_cast<const void*>(instanced_instance->mesh->triangle_range_.get()) : nullptr;
            const void* meshlet_range_ptr =
                instanced_instance->mesh ? static_cast<const void*>(instanced_instance->mesh->meshlet_range_.get()) : nullptr;
            glm::vec3 local_bound_center(0.0f);
            glm::vec3 local_bound_extent(0.0f);
            glm::vec3 world_bound_center(0.0f);
            if (instanced_instance->mesh) {
              const Bound local_bound = instanced_instance->mesh->GetBound();
              local_bound_center = (local_bound.min + local_bound.max) * 0.5f;
              local_bound_extent = local_bound.max - local_bound.min;
              world_bound_center = glm::vec3(instanced_instance->model.value * glm::vec4(local_bound_center, 1.0f));
            }

            std::ostringstream os;
            os << "NeedleDrawAudit frame=" << current_frame << " cam=" << camera_index
               << " drawInstance=" << push_constant.instance_index
               << " entity='" << entity_name << "'"
               << " ownerIdx=" << owner_index
               << " matIdx(render)=" << material_index
               << " matIdx(block)=" << block_material_index
               << " dbgVis=" << debug_visualization_mode
               << " vco(cpu)=" << static_cast<int>(instanced_instance->material->vertex_color_only)
               << " vco(block)=" << block_vertex_color_only
               << " transmission(block)=" << block_transmission
               << " albTex(block)=" << block_albedo_texture_index
               << " albA(block)=" << block_albedo_alpha
               << " rough(block)=" << block_roughness
               << " cull=" << static_cast<uint32_t>(instanced_instance->cull_mode)
               << " castShadow=" << static_cast<int>(instanced_instance->cast_shadow)
               << " particleCount=" << particle_count
               << " p0Color=(" << first_instance_color.x << "," << first_instance_color.y
               << "," << first_instance_color.z << "," << first_instance_color.w << ")"
               << " vtx0Color=(" << first_vertex_color.x << "," << first_vertex_color.y
               << "," << first_vertex_color.z << "," << first_vertex_color.w << ")"
               << " vtxMidColor=(" << mid_vertex_color.x << "," << mid_vertex_color.y
               << "," << mid_vertex_color.z << "," << mid_vertex_color.w << ")"
               << " vtxLastColor=(" << last_vertex_color.x << "," << last_vertex_color.y
               << "," << last_vertex_color.z << "," << last_vertex_color.w << ")"
               << " vtxColorRange=(" << vc_range.x << "," << vc_range.y
               << "," << vc_range.z << "," << vc_range.w << ")"
               << " vertexCount=" << vertex_total
               << " sampledCount=" << sampled
               << " meshHandle=" << mesh_handle
               << " materialHandle=" << material_handle
               << " particleHandle=" << particle_handle
               << " meshVersion=" << mesh_version
               << " particleVersion=" << particle_version
               << " triOffset=" << triangle_offset
               << " triRange=" << triangle_range_size
               << " triIdxCount=" << triangle_index_count
               << " triPrevOffset=" << triangle_prev_frame_offset
               << " triPrevIdxCount=" << triangle_prev_frame_index_count
               << " triRangePtr=" << tri_range_ptr
               << " meshletRangePtr=" << meshlet_range_ptr
               << " modelT=(" << model_translation.x << "," << model_translation.y << "," << model_translation.z << ")"
               << " localC=(" << local_bound_center.x << "," << local_bound_center.y << "," << local_bound_center.z << ")"
               << " worldC=(" << world_bound_center.x << "," << world_bound_center.y << "," << world_bound_center.z << ")"
               << " localE=(" << local_bound_extent.x << "," << local_bound_extent.y << "," << local_bound_extent.z << ")"
               << " set0=" << set0
               << " set1=" << set1
               << " meshAttr[n/t/uv/c]="
               << static_cast<int>(instanced_instance->mesh ? instanced_instance->mesh->vertex_attributes_.normal : false)
               << "/"
               << static_cast<int>(instanced_instance->mesh ? instanced_instance->mesh->vertex_attributes_.tangent : false)
               << "/"
               << static_cast<int>(instanced_instance->mesh ? instanced_instance->mesh->vertex_attributes_.tex_coord : false)
               << "/"
               << static_cast<int>(instanced_instance->mesh ? instanced_instance->mesh->vertex_attributes_.color : false);
            // Iteration 5b: append GPU global buffer sizes + this pine's
            // max-needed index so we can spot GPU buffer truncation per draw.
            // If thisPineMaxIdx > gpuTriIdxCap, this pine's draw reads past
            // the end of the global triangle buffer ==> CONFIRMED truncation.
            {
              const auto& gpu_vtx_buf = GeometryStorage::GetVertexBuffer();
              const auto& gpu_tri_buf = GeometryStorage::GetTriangleBuffer();
              const auto& gpu_meshlet_buf = GeometryStorage::GetMeshletBuffer();
              const VkDeviceSize gpu_vtx_bytes =
                  gpu_vtx_buf ? gpu_vtx_buf->GetVmaAllocationInfo().size : 0;
              const VkDeviceSize gpu_tri_bytes =
                  gpu_tri_buf ? gpu_tri_buf->GetVmaAllocationInfo().size : 0;
              const VkDeviceSize gpu_meshlet_bytes =
                  gpu_meshlet_buf ? gpu_meshlet_buf->GetVmaAllocationInfo().size : 0;
              const uint64_t this_pine_max_idx =
                  (static_cast<uint64_t>(triangle_prev_frame_offset)
                   + static_cast<uint64_t>(triangle_prev_frame_index_count)) * 3ull;
              const uint64_t gpu_tri_idx_cap =
                  gpu_tri_bytes / sizeof(uint32_t);
              os << " gpuVtxBytes=" << gpu_vtx_bytes
                 << " gpuTriBytes=" << gpu_tri_bytes
                 << " gpuMeshletBytes=" << gpu_meshlet_bytes
                 << " thisPineMaxIdx=" << this_pine_max_idx
                 << " gpuTriIdxCap=" << gpu_tri_idx_cap
                 << " truncated=" << ((this_pine_max_idx > gpu_tri_idx_cap) ? 1 : 0);
            }
            EVOENGINE_WARNING(os.str());
            debug_logs_this_frame++;
          };

          const auto log_stem_draw_audit =
              [&](const std::shared_ptr<RenderInstanceStorage::InstancedRenderInstance>& instanced_instance,
                  const RenderInstancePushConstant& push_constant) {
            if (!debug_stem_state_audit || !instanced_instance ||
                debug_stem_logs_this_frame >= debug_stem_max_logs_per_frame) {
              return;
            }
            std::string entity_name = "<api>";
            if (scene && scene->IsEntityValid(instanced_instance->owner)) {
              entity_name = scene->GetEntityName(instanced_instance->owner);
            }

            const auto& material_blocks = current_render_instances->GetMaterialInfoBlocks();
            const auto& instance_blocks = current_render_instances->GetInstanceInfoBlocks();
            const int debug_visualization_mode = current_render_instances->render_info_block.debug_visualization;

            const int material_index = instanced_instance->material_index;
            const bool valid_material_index =
                material_index >= 0 && material_index < static_cast<int>(material_blocks.size());
            const bool valid_instance_index =
                push_constant.instance_index >= 0 &&
                push_constant.instance_index < static_cast<int>(instance_blocks.size());

            int block_material_index = -1;
            int block_vertex_color_only = -1;
            float block_roughness = -1.0f;
            if (valid_material_index) {
              block_vertex_color_only = material_blocks[material_index].vertex_color_only;
              block_roughness = material_blocks[material_index].roughness_val;
            }
            if (valid_instance_index) {
              block_material_index = instance_blocks[push_constant.instance_index].material_index;
            }

            const auto particle_count =
                instanced_instance->particle_infos
                    ? static_cast<int>(instanced_instance->particle_infos->PeekParticleInfoList().size())
                    : 0;

            VkDescriptorSet set1 = VK_NULL_HANDLE;
            if (instanced_instance->particle_infos) {
              set1 = instanced_instance->particle_infos->GetDescriptorSet()->GetVkDescriptorSet();
            }

            glm::vec4 first_instance_color(0.0f);
            if (instanced_instance->particle_infos) {
              const auto& particle_infos = instanced_instance->particle_infos->PeekParticleInfoList();
              if (!particle_infos.empty()) {
                first_instance_color = particle_infos.front().instance_color;
              }
            }

            const uint32_t owner_index = instanced_instance->owner.GetIndex();
            const uint32_t triangle_prev_frame_offset =
                (instanced_instance->mesh && instanced_instance->mesh->triangle_range_)
                    ? instanced_instance->mesh->triangle_range_->prev_frame_offset
                    : 0u;
            const uint32_t triangle_prev_frame_index_count =
                (instanced_instance->mesh && instanced_instance->mesh->triangle_range_)
                    ? instanced_instance->mesh->triangle_range_->prev_frame_index_count
                    : 0u;

            std::ostringstream os;
            os << "StemDrawAudit frame=" << current_frame << " cam=" << camera_index
               << " drawInstance=" << push_constant.instance_index
               << " entity='" << entity_name << "'"
               << " ownerIdx=" << owner_index
               << " matIdx(render)=" << material_index
               << " matIdx(block)=" << block_material_index
               << " dbgVis=" << debug_visualization_mode
               << " vco(block)=" << block_vertex_color_only
               << " rough(block)=" << block_roughness
               << " particleCount=" << particle_count
               << " p0Color=(" << first_instance_color.x << "," << first_instance_color.y
               << "," << first_instance_color.z << "," << first_instance_color.w << ")"
               << " triPrevOffset=" << triangle_prev_frame_offset
               << " triPrevIdxCount=" << triangle_prev_frame_index_count
               << " set1=" << set1;

            EVOENGINE_WARNING(os.str());
            debug_stem_logs_this_frame++;
          };

          const auto hard_reset_instanced_state =
              [&](const std::shared_ptr<RenderInstanceStorage::InstancedRenderInstance>& instanced_instance) {
            if (!debug_needle_hard_reset || !instanced_instance || !instanced_instance->particle_infos) {
              return;
            }
            instanced_deferred_prepass_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
            instanced_deferred_prepass_pipeline->states.SetViewportScissor(view_port);
            instanced_deferred_prepass_pipeline->Bind(vk_command_buffer);
            instanced_deferred_prepass_pipeline->BindDescriptorSet(
                vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
            instanced_deferred_prepass_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, instanced_instance->particle_infos->GetDescriptorSet()->GetVkDescriptorSet());
            GeometryStorage::BindVertices(vk_command_buffer);
          };

          current_render_instances->deferred_instanced_render_instances->ForEachRenderInstance(
              [&](const auto& render_instance) {
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = camera_index;
                push_constant.instance_index = render_instance->instance_index;

                const auto needle_instance = maybe_get_needle_instance(render_instance);
                const auto stem_instance = maybe_get_stem_instance(render_instance);
                hard_reset_instanced_state(needle_instance);

                instanced_deferred_prepass_pipeline->states.polygon_mode =
                    wire_frame ? VK_POLYGON_MODE_LINE : render_instance->polygon_mode;
                instanced_deferred_prepass_pipeline->states.cull_mode = render_instance->cull_mode;
                instanced_deferred_prepass_pipeline->states.line_width = render_instance->line_width;

                log_needle_draw_audit(needle_instance, push_constant);
                log_stem_draw_audit(stem_instance, push_constant);

                const auto prim_count =
                    render_instance->Render(vk_command_buffer, push_constant, instanced_deferred_prepass_pipeline);
                if (count_draw_calls) {
                  platform.draw_call[current_frame_index]++;
                  platform.prim_count[current_frame_index] += prim_count;
                }
              });
        }
        GeometryStorage::BindSkinnedVertices(vk_command_buffer);
        {
          skinned_deferred_prepass_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
          skinned_deferred_prepass_pipeline->states.SetViewportScissor(view_port);
          skinned_deferred_prepass_pipeline->Bind(vk_command_buffer);
          skinned_deferred_prepass_pipeline->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          current_render_instances->deferred_skinned_render_instances->ForEachRenderInstance(
              [&](const auto& render_instance) {
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = camera_index;
                push_constant.instance_index = render_instance->instance_index;
                skinned_deferred_prepass_pipeline->states.polygon_mode =
                    wire_frame ? VK_POLYGON_MODE_LINE : render_instance->polygon_mode;
                skinned_deferred_prepass_pipeline->states.cull_mode = render_instance->cull_mode;
                skinned_deferred_prepass_pipeline->states.line_width = render_instance->line_width;
                const auto prim_count =
                    render_instance->Render(vk_command_buffer, push_constant, skinned_deferred_prepass_pipeline);
                if (count_draw_calls) {
                  platform.draw_call[current_frame_index]++;
                  platform.prim_count[current_frame_index] += prim_count;
                }
              });
        }
#ifdef EVOENGINE_WINDOWS
        GeometryStorage::BindStrandPoints(vk_command_buffer);
        {
          strands_deferred_prepass_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
          strands_deferred_prepass_pipeline->states.SetViewportScissor(view_port);
          strands_deferred_prepass_pipeline->Bind(vk_command_buffer);
          strands_deferred_prepass_pipeline->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          current_render_instances->deferred_strands_render_instances->ForEachRenderInstance(
              [&](const auto& render_instance) {
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = camera_index;
                push_constant.instance_index = render_instance->instance_index;
                strands_deferred_prepass_pipeline->states.polygon_mode =
                    wire_frame ? VK_POLYGON_MODE_LINE : render_instance->polygon_mode;
                strands_deferred_prepass_pipeline->states.cull_mode = render_instance->cull_mode;
                strands_deferred_prepass_pipeline->states.line_width = render_instance->line_width;
                const auto prim_count =
                    render_instance->Render(vk_command_buffer, push_constant, strands_deferred_prepass_pipeline);
                if (count_draw_calls) {
                  platform.draw_call[current_frame_index]++;
                  platform.prim_count[current_frame_index] += prim_count;
                }
              });
        }
#endif
#pragma region External Deferred Rendering
        for (const auto& func : deferred_rendering_external_functions) {
          const auto prim_count =
              func(vk_command_buffer, geometry_pass_color_attachment_infos, {camera_index, view_port});
          if (count_draw_calls) {
            platform.draw_call[current_frame_index]++;
            platform.prim_count[current_frame_index] += prim_count;
          }
        }

#pragma endregion
      });

#pragma endregion
#pragma region Lighting pass
      GeometryStorage::BindVertices(vk_command_buffer);
      {
        camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        camera->render_texture_->GetDepthImage()->TransitImageLayout(vk_command_buffer,
                                                                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
        camera->GetRenderTexture()->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                               VK_ATTACHMENT_STORE_OP_STORE);
        VkRenderingInfo render_info{};
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = color_attachment_infos.size();
        render_info.pColorAttachments = color_attachment_infos.data();
        render_info.pDepthAttachment = VK_NULL_HANDLE;
        lighting_->directional_light_shadow_map_->TransitImageLayout(vk_command_buffer,
                                                                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        const auto& deferred_lighting_pipeline =
            is_scene_camera ? deferred_lighting_pass_pipeline_scene_camera : deferred_lighting_pass_pipeline;
        Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
          deferred_lighting_pipeline->states.ResetAllStates(color_attachment_infos.size());
          deferred_lighting_pipeline->states.depth_test = false;
          deferred_lighting_pipeline->states.SetViewportScissor(view_port);

          deferred_lighting_pipeline->Bind(vk_command_buffer);
          deferred_lighting_pipeline->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          deferred_lighting_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                        camera->g_buffer_descriptor_set_->GetVkDescriptorSet());
          deferred_lighting_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                                        lighting_->lighting_descriptor_set->GetVkDescriptorSet());
          RenderInstancePushConstant push_constant;
          push_constant.camera_index = camera_index;
          push_constant.light_split_index = need_fade ? glm::max(128, 256 - editor_layer->selection_alpha_) : 256;
          push_constant.instance_index = need_fade ? 1 : 0;
          deferred_lighting_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          const auto mesh = Resources::texture_pass_through_quad;
          mesh->DrawIndexed(vk_command_buffer, deferred_lighting_pipeline->states, 1);
        });
      }
#pragma endregion
#pragma endregion

#pragma region External Forward Rendering
      for (const auto& func : forward_rendering_external_functions) {
        const auto prim_count = func(vk_command_buffer, camera, {camera_index, view_port});
        if (count_draw_calls) {
          platform.draw_call[current_frame_index]++;
          platform.prim_count[current_frame_index] += prim_count;
        }
      }
#pragma endregion
    });

    // Post-processing
    if (const auto post_processing_stack = camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
      post_processing_stack->Process(camera);
    }
    camera->rendered_ = true;
    camera->require_rendering_ = false;
  }
}

void RenderLayer::RenderToCameraRayTracing(const GlobalTransform& camera_global_transform,
                                           const std::shared_ptr<Camera>& camera) const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  const int camera_index = current_render_instances->GetCameraIndex(camera->GetHandle());
  const auto scene = Application::GetActiveScene();

  if (camera->camera_render_mode == Camera::CameraRenderMode::RayTracing) {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      Platform::EverythingBarrier(vk_command_buffer);
      camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      Platform::EverythingBarrier(vk_command_buffer);
      ray_tracing_camera_pipeline->Bind(vk_command_buffer);
      ray_tracing_camera_pipeline->BindDescriptorSet(
          vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
      ray_tracing_camera_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, ray_tracing_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
      ray_tracing_camera_pipeline->BindDescriptorSet(
          vk_command_buffer, 2, camera->GetRenderTexture()->storage_descriptor_set_->GetVkDescriptorSet());
      RayTracingCameraPushConstant push_constant;
      push_constant.camera_index = camera_index;
      push_constant.frame_id = camera->frame_count_;
      ray_tracing_camera_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      ray_tracing_camera_pipeline->Trace(vk_command_buffer, camera->render_texture_->GetExtent().width,
                                         camera->render_texture_->GetExtent().height, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
    camera->rendered_ = true;
    camera->require_rendering_ = false;
    camera->frame_count_++;
  }
}

void RenderLayer::PreUpdate() {
  const auto scene = GetScene();
  if (!scene)
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  current_render_instances->Clear();
  scene->SetBound({});
}

uint32_t RenderLayer::DrawMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                               const GlobalTransform& global_transform, const bool cast_shadow) const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  return current_render_instances->RegisterMeshDrawCommand(mesh, material, global_transform, cast_shadow);
}

uint32_t RenderLayer::DrawMeshInstanced(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                                        const GlobalTransform& global_transform,
                                        const std::shared_ptr<ParticleInfoList>& particle_info_list,
                                        const bool cast_shadow) const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  return current_render_instances->RegisterMeshDrawInstancedCommand(mesh, material, global_transform,
                                                                    particle_info_list, cast_shadow);
}

const std::shared_ptr<DescriptorSet>& RenderLayer::GetPerFrameDescriptorSet() {
  return Application::GetLayer<RenderLayer>()->per_frame_descriptor_sets_[Platform::GetCurrentFrameIndex()];
}

const std::shared_ptr<DescriptorSet>& RenderLayer::GetLightingDescriptorSet() {
  return Application::GetLayer<RenderLayer>()->lighting_->lighting_descriptor_set;
}
