#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
#include "Utilities.hpp"
#include "WindowLayer.hpp"
using namespace evo_engine;

void Platform::CreateGraphicsPipelines() const {
  auto per_frame_layout = GetDescriptorSetLayout("PER_FRAME_LAYOUT");
  auto meshlet_layout = GetDescriptorSetLayout("MESHLET_LAYOUT");
  auto ray_tracing_layout = GetDescriptorSetLayout("RAY_TRACING_LAYOUT");
  auto camera_g_buffer_layout = GetDescriptorSetLayout("CAMERA_GBUFFER_LAYOUT");
  auto lighting_layout = GetDescriptorSetLayout("LIGHTING_LAYOUT");

  auto bone_matrices_layout = GetDescriptorSetLayout("BONE_MATRICES_LAYOUT");
  auto instanced_data_layout = GetDescriptorSetLayout("INSTANCED_DATA_LAYOUT");

  if (const auto window_layer = Application::GetLayer<WindowLayer>()) {
    const auto render_texture_pass_through = std::make_shared<GraphicsPipeline>();
    render_texture_pass_through->vertex_shader = Resources::GetResource<Shader>("TEXTURE_PASS_THROUGH_VERT");
    render_texture_pass_through->fragment_shader = Resources::GetResource<Shader>("TEXTURE_PASS_THROUGH_FRAG");
    render_texture_pass_through->geometry_type = GeometryType::Mesh;
    render_texture_pass_through->descriptor_set_layouts.emplace_back(
        GetDescriptorSetLayout("RENDER_TEXTURE_PRESENT_LAYOUT"));
    
    render_texture_pass_through->depth_attachment_format = VK_FORMAT_UNDEFINED;
    render_texture_pass_through->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    render_texture_pass_through->color_attachment_formats = {1, swapchain_->GetImageFormat()};
    render_texture_pass_through->Initialize();
    RegisterGraphicsPipeline("RENDER_TEXTURE_PRESENT", render_texture_pass_through);
  }
  {
    const auto ray_tracing_camera = std::make_shared<RayTracingPipeline>();
    ray_tracing_camera->raygen_shader = Resources::GetResource<Shader>("RAY_TRACING_CAMERA_RAYGEN");
    ray_tracing_camera->miss_shader = Resources::GetResource<Shader>("RAY_TRACING_CAMERA_MISS");
    ray_tracing_camera->closest_hit_shader = Resources::GetResource<Shader>("RAY_TRACING_CAMERA_CHIT");

    ray_tracing_camera->descriptor_set_layouts.emplace_back(per_frame_layout);
    ray_tracing_camera->descriptor_set_layouts.emplace_back(ray_tracing_layout);
    ray_tracing_camera->descriptor_set_layouts.emplace_back(
        GetDescriptorSetLayout("RENDER_TEXTURE_STORAGE_LAYOUT"));

    auto& push_constant_range = ray_tracing_camera->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RayTracingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    ray_tracing_camera->Initialize();
    RegisterRayTracingPipeline("RAY_TRACING_CAMERA", ray_tracing_camera);
  }
  {
    const auto ssr_reflect = std::make_shared<GraphicsPipeline>();
    ssr_reflect->vertex_shader = Resources::GetResource<Shader>("TEXTURE_PASS_THROUGH_VERT");
    ssr_reflect->fragment_shader = Resources::GetResource<Shader>("SSR_REFLECT_FRAG");
    ssr_reflect->geometry_type = GeometryType::Mesh;
    ssr_reflect->descriptor_set_layouts.emplace_back(per_frame_layout);
    ssr_reflect->descriptor_set_layouts.emplace_back(GetDescriptorSetLayout("SSR_REFLECT_LAYOUT"));

    ssr_reflect->depth_attachment_format = VK_FORMAT_UNDEFINED;
    ssr_reflect->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    ssr_reflect->color_attachment_formats = {2, Constants::render_texture_color};

    auto& push_constant_range = ssr_reflect->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SsrPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    ssr_reflect->Initialize();
    RegisterGraphicsPipeline("SSR_REFLECT", ssr_reflect);
  }

  {
    const auto ssr_blur = std::make_shared<GraphicsPipeline>();
    ssr_blur->vertex_shader = Resources::GetResource<Shader>("TEXTURE_PASS_THROUGH_VERT");
    ssr_blur->fragment_shader = Resources::GetResource<Shader>("SSR_BLUR_FRAG");
    ssr_blur->geometry_type = GeometryType::Mesh;
    ssr_blur->descriptor_set_layouts.emplace_back(GetDescriptorSetLayout("SSR_BLUR_LAYOUT"));

    ssr_blur->depth_attachment_format = VK_FORMAT_UNDEFINED;
    ssr_blur->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    ssr_blur->color_attachment_formats = {1, Constants::render_texture_color};

    auto& push_constant_range = ssr_blur->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SsrPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    ssr_blur->Initialize();
    RegisterGraphicsPipeline("SSR_BLUR", ssr_blur);
  }

  {
    const auto ssr_combine = std::make_shared<GraphicsPipeline>();
    ssr_combine->vertex_shader = Resources::GetResource<Shader>("TEXTURE_PASS_THROUGH_VERT");
    ssr_combine->fragment_shader = Resources::GetResource<Shader>("SSR_COMBINE_FRAG");
    ssr_combine->geometry_type = GeometryType::Mesh;
    ssr_combine->descriptor_set_layouts.emplace_back(GetDescriptorSetLayout("SSR_COMBINE_LAYOUT"));

    ssr_combine->depth_attachment_format = VK_FORMAT_UNDEFINED;
    ssr_combine->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    ssr_combine->color_attachment_formats = {1, Constants::render_texture_color};

    auto& push_constant_range = ssr_combine->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SsrPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    ssr_combine->Initialize();

    RegisterGraphicsPipeline("SSR_COMBINE", ssr_combine);
  }

  {
    const auto standard_deferred_prepass = std::make_shared<GraphicsPipeline>();
    standard_deferred_prepass->vertex_shader = Resources::GetResource<Shader>("STANDARD_VERT");

    standard_deferred_prepass->fragment_shader = Resources::GetResource<Shader>("STANDARD_DEFERRED_FRAG");
    standard_deferred_prepass->geometry_type = GeometryType::Mesh;
    standard_deferred_prepass->descriptor_set_layouts.emplace_back(per_frame_layout);

    standard_deferred_prepass->depth_attachment_format = Constants::g_buffer_depth;
    standard_deferred_prepass->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    standard_deferred_prepass->color_attachment_formats = {2, Constants::g_buffer_color};

    auto& push_constant_range = standard_deferred_prepass->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    standard_deferred_prepass->Initialize();
    RegisterGraphicsPipeline("STANDARD_DEFERRED_PREPASS", standard_deferred_prepass);
  }
  if (Constants::support_mesh_shader) {
    const auto standard_deferred_prepass = std::make_shared<GraphicsPipeline>();
    standard_deferred_prepass->task_shader = Resources::GetResource<Shader>("STANDARD_TASK");
    standard_deferred_prepass->mesh_shader = Resources::GetResource<Shader>("STANDARD_MESH");

    standard_deferred_prepass->fragment_shader = Resources::GetResource<Shader>("STANDARD_DEFERRED_FRAG");
    standard_deferred_prepass->geometry_type = GeometryType::Mesh;
    standard_deferred_prepass->descriptor_set_layouts.emplace_back(per_frame_layout);
    standard_deferred_prepass->descriptor_set_layouts.emplace_back(meshlet_layout);
    standard_deferred_prepass->depth_attachment_format = Constants::g_buffer_depth;
    standard_deferred_prepass->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    standard_deferred_prepass->color_attachment_formats = {2, Constants::g_buffer_color};

    auto& push_constant_range = standard_deferred_prepass->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    standard_deferred_prepass->Initialize();
    RegisterGraphicsPipeline("STANDARD_DEFERRED_PREPASS_MESH", standard_deferred_prepass);
  }

  {
    const auto standard_skinned_deferred_prepass = std::make_shared<GraphicsPipeline>();
    standard_skinned_deferred_prepass->vertex_shader = Resources::GetResource<Shader>("STANDARD_SKINNED_VERT");
    standard_skinned_deferred_prepass->fragment_shader = Resources::GetResource<Shader>("STANDARD_DEFERRED_FRAG");
    standard_skinned_deferred_prepass->geometry_type = GeometryType::SkinnedMesh;
    standard_skinned_deferred_prepass->descriptor_set_layouts.emplace_back(per_frame_layout);
    standard_skinned_deferred_prepass->descriptor_set_layouts.emplace_back(bone_matrices_layout);

    standard_skinned_deferred_prepass->depth_attachment_format = Constants::g_buffer_depth;
    standard_skinned_deferred_prepass->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    standard_skinned_deferred_prepass->color_attachment_formats = {2, Constants::g_buffer_color};

    auto& push_constant_range = standard_skinned_deferred_prepass->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    standard_skinned_deferred_prepass->Initialize();
    RegisterGraphicsPipeline("STANDARD_SKINNED_DEFERRED_PREPASS", standard_skinned_deferred_prepass);
  }
  {
    const auto standard_instanced_deferred_prepass = std::make_shared<GraphicsPipeline>();
    standard_instanced_deferred_prepass->vertex_shader = Resources::GetResource<Shader>("STANDARD_INSTANCED_VERT");
    standard_instanced_deferred_prepass->fragment_shader = Resources::GetResource<Shader>("STANDARD_DEFERRED_FRAG");
    standard_instanced_deferred_prepass->geometry_type = GeometryType::Mesh;
    standard_instanced_deferred_prepass->descriptor_set_layouts.emplace_back(per_frame_layout);
    standard_instanced_deferred_prepass->descriptor_set_layouts.emplace_back(instanced_data_layout);

    standard_instanced_deferred_prepass->depth_attachment_format = Constants::g_buffer_depth;
    standard_instanced_deferred_prepass->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    standard_instanced_deferred_prepass->color_attachment_formats = {2, Constants::g_buffer_color};

    auto& push_constant_range = standard_instanced_deferred_prepass->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    standard_instanced_deferred_prepass->Initialize();
    RegisterGraphicsPipeline("STANDARD_INSTANCED_DEFERRED_PREPASS", standard_instanced_deferred_prepass);
  }
  {
    const auto standard_strands_deferred_prepass = std::make_shared<GraphicsPipeline>();
    standard_strands_deferred_prepass->vertex_shader = Resources::GetResource<Shader>("STANDARD_STRANDS_VERT");
    standard_strands_deferred_prepass->tessellation_control_shader =
        Resources::GetResource<Shader>("STANDARD_STRANDS_TESC");
    standard_strands_deferred_prepass->tessellation_evaluation_shader =
        Resources::GetResource<Shader>("STANDARD_STRANDS_TESE");
    standard_strands_deferred_prepass->geometry_shader = Resources::GetResource<Shader>("STANDARD_STRANDS_GEOM");
    standard_strands_deferred_prepass->fragment_shader = Resources::GetResource<Shader>("STANDARD_DEFERRED_FRAG");
    standard_strands_deferred_prepass->geometry_type = GeometryType::Strands;
    standard_strands_deferred_prepass->descriptor_set_layouts.emplace_back(per_frame_layout);
    standard_strands_deferred_prepass->descriptor_set_layouts.emplace_back(instanced_data_layout);

    standard_strands_deferred_prepass->tessellation_patch_control_points = 4;

    standard_strands_deferred_prepass->depth_attachment_format = Constants::g_buffer_depth;
    standard_strands_deferred_prepass->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    standard_strands_deferred_prepass->color_attachment_formats = {2, Constants::g_buffer_color};

    auto& push_constant_range = standard_strands_deferred_prepass->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    standard_strands_deferred_prepass->Initialize();
    RegisterGraphicsPipeline("STANDARD_STRANDS_DEFERRED_PREPASS", standard_strands_deferred_prepass);
  }
  {
    const auto standard_deferred_lighting = std::make_shared<GraphicsPipeline>();
    standard_deferred_lighting->vertex_shader = Resources::GetResource<Shader>("TEXTURE_PASS_THROUGH_VERT");
    standard_deferred_lighting->fragment_shader = Resources::GetResource<Shader>("STANDARD_DEFERRED_LIGHTING_FRAG");
    standard_deferred_lighting->geometry_type = GeometryType::Mesh;
    standard_deferred_lighting->descriptor_set_layouts.emplace_back(per_frame_layout);
    standard_deferred_lighting->descriptor_set_layouts.emplace_back(camera_g_buffer_layout);
    standard_deferred_lighting->descriptor_set_layouts.emplace_back(lighting_layout);

    standard_deferred_lighting->depth_attachment_format = Constants::render_texture_depth;
    standard_deferred_lighting->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    standard_deferred_lighting->color_attachment_formats = {1, Constants::render_texture_color};

    auto& push_constant_range = standard_deferred_lighting->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    standard_deferred_lighting->Initialize();
    RegisterGraphicsPipeline("STANDARD_DEFERRED_LIGHTING", standard_deferred_lighting);
  }
  {
    const auto standard_deferred_lighting_scene_camera = std::make_shared<GraphicsPipeline>();
    standard_deferred_lighting_scene_camera->vertex_shader =
        Resources::GetResource<Shader>("TEXTURE_PASS_THROUGH_VERT");
    standard_deferred_lighting_scene_camera->fragment_shader =
        Resources::GetResource<Shader>("STANDARD_DEFERRED_LIGHTING_SCENE_CAMERA_FRAG");
    standard_deferred_lighting_scene_camera->geometry_type = GeometryType::Mesh;
    standard_deferred_lighting_scene_camera->descriptor_set_layouts.emplace_back(per_frame_layout);
    standard_deferred_lighting_scene_camera->descriptor_set_layouts.emplace_back(camera_g_buffer_layout);
    standard_deferred_lighting_scene_camera->descriptor_set_layouts.emplace_back(lighting_layout);

    standard_deferred_lighting_scene_camera->depth_attachment_format = Constants::render_texture_depth;
    standard_deferred_lighting_scene_camera->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    standard_deferred_lighting_scene_camera->color_attachment_formats = {1, Constants::render_texture_color};

    auto& push_constant_range = standard_deferred_lighting_scene_camera->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    standard_deferred_lighting_scene_camera->Initialize();
    RegisterGraphicsPipeline("STANDARD_DEFERRED_LIGHTING_SCENE_CAMERA", standard_deferred_lighting_scene_camera);
  }
  {
    const auto directional_light_shadow_map = std::make_shared<GraphicsPipeline>();
    directional_light_shadow_map->vertex_shader = Resources::GetResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_VERT");
    directional_light_shadow_map->fragment_shader = Resources::GetResource<Shader>("SHADOW_MAP_PASS_THROUGH_FRAG");
    directional_light_shadow_map->geometry_type = GeometryType::Mesh;
    directional_light_shadow_map->descriptor_set_layouts.emplace_back(per_frame_layout);
    directional_light_shadow_map->depth_attachment_format = Constants::shadow_map;
    directional_light_shadow_map->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = directional_light_shadow_map->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    directional_light_shadow_map->Initialize();
    RegisterGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP", directional_light_shadow_map);
  }
  if (Constants::support_mesh_shader) {
    const auto directional_light_shadow_map = std::make_shared<GraphicsPipeline>();
    directional_light_shadow_map->task_shader = Resources::GetResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_TASK");
    directional_light_shadow_map->mesh_shader = Resources::GetResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_MESH");
    directional_light_shadow_map->fragment_shader = Resources::GetResource<Shader>("EMPTY_FRAG");
    directional_light_shadow_map->geometry_type = GeometryType::Mesh;
    directional_light_shadow_map->descriptor_set_layouts.emplace_back(per_frame_layout);
    directional_light_shadow_map->descriptor_set_layouts.emplace_back(meshlet_layout);
    directional_light_shadow_map->depth_attachment_format = Constants::shadow_map;
    directional_light_shadow_map->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = directional_light_shadow_map->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    directional_light_shadow_map->Initialize();
    RegisterGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_MESH", directional_light_shadow_map);
  }
  {
    const auto directional_light_shadow_map_skinned = std::make_shared<GraphicsPipeline>();
    directional_light_shadow_map_skinned->vertex_shader =
        Resources::GetResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_SKINNED_VERT");
    directional_light_shadow_map_skinned->fragment_shader =
        Resources::GetResource<Shader>("SHADOW_MAP_PASS_THROUGH_FRAG");
    directional_light_shadow_map_skinned->geometry_type = GeometryType::SkinnedMesh;
    directional_light_shadow_map_skinned->descriptor_set_layouts.emplace_back(per_frame_layout);
    directional_light_shadow_map_skinned->descriptor_set_layouts.emplace_back(bone_matrices_layout);
    directional_light_shadow_map_skinned->depth_attachment_format = Constants::shadow_map;
    directional_light_shadow_map_skinned->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = directional_light_shadow_map_skinned->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    directional_light_shadow_map_skinned->Initialize();
    RegisterGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_SKINNED", directional_light_shadow_map_skinned);
  }
  {
    const auto directional_light_shadow_map_instanced = std::make_shared<GraphicsPipeline>();
    directional_light_shadow_map_instanced->vertex_shader =
        Resources::GetResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_INSTANCED_VERT");
    directional_light_shadow_map_instanced->fragment_shader =
        Resources::GetResource<Shader>("SHADOW_MAP_PASS_THROUGH_FRAG");
    directional_light_shadow_map_instanced->geometry_type = GeometryType::Mesh;
    directional_light_shadow_map_instanced->descriptor_set_layouts.emplace_back(per_frame_layout);
    directional_light_shadow_map_instanced->descriptor_set_layouts.emplace_back(instanced_data_layout);
    directional_light_shadow_map_instanced->depth_attachment_format = Constants::shadow_map;
    directional_light_shadow_map_instanced->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = directional_light_shadow_map_instanced->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    directional_light_shadow_map_instanced->Initialize();
    RegisterGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_INSTANCED", directional_light_shadow_map_instanced);
  }
  {
    const auto directional_light_shadow_map_strand = std::make_shared<GraphicsPipeline>();
    directional_light_shadow_map_strand->vertex_shader =
        Resources::GetResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_STRANDS_VERT");
    directional_light_shadow_map_strand->tessellation_control_shader =
        Resources::GetResource<Shader>("SHADOW_MAP_STRANDS_TESC");
    directional_light_shadow_map_strand->tessellation_evaluation_shader =
        Resources::GetResource<Shader>("SHADOW_MAP_STRANDS_TESE");
    directional_light_shadow_map_strand->geometry_shader =
        Resources::GetResource<Shader>("DIRECTIONAL_LIGHT_SHADOW_MAP_STRANDS_GEOM");
    directional_light_shadow_map_strand->fragment_shader = Resources::GetResource<Shader>("EMPTY_FRAG");
    directional_light_shadow_map_strand->geometry_type = GeometryType::Mesh;
    directional_light_shadow_map_strand->descriptor_set_layouts.emplace_back(per_frame_layout);
    directional_light_shadow_map_strand->descriptor_set_layouts.emplace_back(instanced_data_layout);
    directional_light_shadow_map_strand->depth_attachment_format = Constants::shadow_map;
    directional_light_shadow_map_strand->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    directional_light_shadow_map_strand->tessellation_patch_control_points = 4;

    auto& push_constant_range = directional_light_shadow_map_strand->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    directional_light_shadow_map_strand->Initialize();
    RegisterGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_STRANDS", directional_light_shadow_map_strand);
  }
  {
    const auto point_light_shadow_map = std::make_shared<GraphicsPipeline>();
    point_light_shadow_map->vertex_shader = Resources::GetResource<Shader>("POINT_LIGHT_SHADOW_MAP_VERT");
    point_light_shadow_map->fragment_shader = Resources::GetResource<Shader>("SHADOW_MAP_PASS_THROUGH_FRAG");
    point_light_shadow_map->geometry_type = GeometryType::Mesh;
    point_light_shadow_map->descriptor_set_layouts.emplace_back(per_frame_layout);
    point_light_shadow_map->depth_attachment_format = Constants::shadow_map;
    point_light_shadow_map->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = point_light_shadow_map->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    point_light_shadow_map->Initialize();
    RegisterGraphicsPipeline("POINT_LIGHT_SHADOW_MAP", point_light_shadow_map);
  }
  if (Constants::support_mesh_shader) {
    const auto point_light_shadow_map = std::make_shared<GraphicsPipeline>();

    point_light_shadow_map->task_shader = Resources::GetResource<Shader>("POINT_LIGHT_SHADOW_MAP_TASK");
    point_light_shadow_map->mesh_shader = Resources::GetResource<Shader>("POINT_LIGHT_SHADOW_MAP_MESH");

    point_light_shadow_map->fragment_shader = Resources::GetResource<Shader>("EMPTY_FRAG");
    point_light_shadow_map->geometry_type = GeometryType::Mesh;
    point_light_shadow_map->descriptor_set_layouts.emplace_back(per_frame_layout);
    point_light_shadow_map->descriptor_set_layouts.emplace_back(meshlet_layout);
    point_light_shadow_map->depth_attachment_format = Constants::shadow_map;
    point_light_shadow_map->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = point_light_shadow_map->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    point_light_shadow_map->Initialize();
    RegisterGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_MESH", point_light_shadow_map);
  }
  {
    const auto point_light_shadow_map_skinned = std::make_shared<GraphicsPipeline>();
    point_light_shadow_map_skinned->vertex_shader =
        Resources::GetResource<Shader>("POINT_LIGHT_SHADOW_MAP_SKINNED_VERT");
    point_light_shadow_map_skinned->fragment_shader = Resources::GetResource<Shader>("SHADOW_MAP_PASS_THROUGH_FRAG");
    point_light_shadow_map_skinned->geometry_type = GeometryType::SkinnedMesh;
    point_light_shadow_map_skinned->descriptor_set_layouts.emplace_back(per_frame_layout);
    point_light_shadow_map_skinned->descriptor_set_layouts.emplace_back(bone_matrices_layout);
    point_light_shadow_map_skinned->depth_attachment_format = Constants::shadow_map;
    point_light_shadow_map_skinned->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = point_light_shadow_map_skinned->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    point_light_shadow_map_skinned->Initialize();
    RegisterGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_SKINNED", point_light_shadow_map_skinned);
  }
  {
    const auto point_light_shadow_map_instanced = std::make_shared<GraphicsPipeline>();
    point_light_shadow_map_instanced->vertex_shader =
        Resources::GetResource<Shader>("POINT_LIGHT_SHADOW_MAP_INSTANCED_VERT");
    point_light_shadow_map_instanced->fragment_shader = Resources::GetResource<Shader>("SHADOW_MAP_PASS_THROUGH_FRAG");
    point_light_shadow_map_instanced->geometry_type = GeometryType::Mesh;
    point_light_shadow_map_instanced->descriptor_set_layouts.emplace_back(per_frame_layout);
    point_light_shadow_map_instanced->descriptor_set_layouts.emplace_back(instanced_data_layout);
    point_light_shadow_map_instanced->depth_attachment_format = Constants::shadow_map;
    point_light_shadow_map_instanced->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = point_light_shadow_map_instanced->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    point_light_shadow_map_instanced->Initialize();
    RegisterGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_INSTANCED", point_light_shadow_map_instanced);
  }
  {
    const auto point_light_shadow_map_strand = std::make_shared<GraphicsPipeline>();
    point_light_shadow_map_strand->vertex_shader =
        Resources::GetResource<Shader>("POINT_LIGHT_SHADOW_MAP_STRANDS_VERT");
    point_light_shadow_map_strand->tessellation_control_shader =
        Resources::GetResource<Shader>("SHADOW_MAP_STRANDS_TESC");
    point_light_shadow_map_strand->tessellation_evaluation_shader =
        Resources::GetResource<Shader>("SHADOW_MAP_STRANDS_TESE");
    point_light_shadow_map_strand->geometry_shader =
        Resources::GetResource<Shader>("POINT_LIGHT_SHADOW_MAP_STRANDS_GEOM");
    point_light_shadow_map_strand->fragment_shader = Resources::GetResource<Shader>("EMPTY_FRAG");
    point_light_shadow_map_strand->geometry_type = GeometryType::Mesh;
    point_light_shadow_map_strand->descriptor_set_layouts.emplace_back(per_frame_layout);
    point_light_shadow_map_strand->descriptor_set_layouts.emplace_back(instanced_data_layout);
    point_light_shadow_map_strand->depth_attachment_format = Constants::shadow_map;
    point_light_shadow_map_strand->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    point_light_shadow_map_strand->tessellation_patch_control_points = 4;

    auto& push_constant_range = point_light_shadow_map_strand->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    point_light_shadow_map_strand->Initialize();
    RegisterGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_STRANDS", point_light_shadow_map_strand);
  }
  {
    const auto spot_light_shadow_map = std::make_shared<GraphicsPipeline>();
    spot_light_shadow_map->vertex_shader = Resources::GetResource<Shader>("SPOT_LIGHT_SHADOW_MAP_VERT");
    spot_light_shadow_map->fragment_shader = Resources::GetResource<Shader>("SHADOW_MAP_PASS_THROUGH_FRAG");
    spot_light_shadow_map->geometry_type = GeometryType::Mesh;
    spot_light_shadow_map->descriptor_set_layouts.emplace_back(per_frame_layout);
    spot_light_shadow_map->depth_attachment_format = Constants::shadow_map;
    spot_light_shadow_map->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = spot_light_shadow_map->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    spot_light_shadow_map->Initialize();
    RegisterGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP", spot_light_shadow_map);
  }
  if (Constants::support_mesh_shader) {
    const auto spot_light_shadow_map = std::make_shared<GraphicsPipeline>();

    spot_light_shadow_map->task_shader = Resources::GetResource<Shader>("SPOT_LIGHT_SHADOW_MAP_TASK");
    spot_light_shadow_map->mesh_shader = Resources::GetResource<Shader>("SPOT_LIGHT_SHADOW_MAP_MESH");

    spot_light_shadow_map->fragment_shader = Resources::GetResource<Shader>("EMPTY_FRAG");
    spot_light_shadow_map->geometry_type = GeometryType::Mesh;
    spot_light_shadow_map->descriptor_set_layouts.emplace_back(per_frame_layout);
    spot_light_shadow_map->descriptor_set_layouts.emplace_back(meshlet_layout);
    spot_light_shadow_map->depth_attachment_format = Constants::shadow_map;
    spot_light_shadow_map->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = spot_light_shadow_map->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    spot_light_shadow_map->Initialize();
    RegisterGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_MESH", spot_light_shadow_map);
  }
  {
    const auto spot_light_shadow_map = std::make_shared<GraphicsPipeline>();
    spot_light_shadow_map->vertex_shader = Resources::GetResource<Shader>("SPOT_LIGHT_SHADOW_MAP_SKINNED_VERT");
    spot_light_shadow_map->fragment_shader = Resources::GetResource<Shader>("SHADOW_MAP_PASS_THROUGH_FRAG");
    spot_light_shadow_map->geometry_type = GeometryType::SkinnedMesh;
    spot_light_shadow_map->descriptor_set_layouts.emplace_back(per_frame_layout);
    spot_light_shadow_map->descriptor_set_layouts.emplace_back(bone_matrices_layout);
    spot_light_shadow_map->depth_attachment_format = Constants::shadow_map;
    spot_light_shadow_map->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = spot_light_shadow_map->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    spot_light_shadow_map->Initialize();
    RegisterGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_SKINNED", spot_light_shadow_map);
  }
  {
    const auto spot_light_shadow_map = std::make_shared<GraphicsPipeline>();
    spot_light_shadow_map->vertex_shader = Resources::GetResource<Shader>("SPOT_LIGHT_SHADOW_MAP_INSTANCED_VERT");
    spot_light_shadow_map->fragment_shader = Resources::GetResource<Shader>("SHADOW_MAP_PASS_THROUGH_FRAG");
    spot_light_shadow_map->geometry_type = GeometryType::Mesh;
    spot_light_shadow_map->descriptor_set_layouts.emplace_back(per_frame_layout);
    spot_light_shadow_map->descriptor_set_layouts.emplace_back(instanced_data_layout);
    spot_light_shadow_map->depth_attachment_format = Constants::shadow_map;
    spot_light_shadow_map->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    auto& push_constant_range = spot_light_shadow_map->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    spot_light_shadow_map->Initialize();
    RegisterGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_INSTANCED", spot_light_shadow_map);
  }
  {
    const auto spot_light_shadow_map_strand = std::make_shared<GraphicsPipeline>();
    spot_light_shadow_map_strand->vertex_shader = Resources::GetResource<Shader>("SPOT_LIGHT_SHADOW_MAP_STRANDS_VERT");
    spot_light_shadow_map_strand->tessellation_control_shader =
        Resources::GetResource<Shader>("SHADOW_MAP_STRANDS_TESC");
    spot_light_shadow_map_strand->tessellation_evaluation_shader =
        Resources::GetResource<Shader>("SHADOW_MAP_STRANDS_TESE");
    spot_light_shadow_map_strand->geometry_shader =
        Resources::GetResource<Shader>("SPOT_LIGHT_SHADOW_MAP_STRANDS_GEOM");
    spot_light_shadow_map_strand->fragment_shader = Resources::GetResource<Shader>("EMPTY_FRAG");
    spot_light_shadow_map_strand->geometry_type = GeometryType::Mesh;
    spot_light_shadow_map_strand->descriptor_set_layouts.emplace_back(per_frame_layout);
    spot_light_shadow_map_strand->descriptor_set_layouts.emplace_back(instanced_data_layout);
    spot_light_shadow_map_strand->depth_attachment_format = Constants::shadow_map;
    spot_light_shadow_map_strand->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    spot_light_shadow_map_strand->tessellation_patch_control_points = 4;

    auto& push_constant_range = spot_light_shadow_map_strand->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderInstancePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    spot_light_shadow_map_strand->Initialize();
    RegisterGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_STRANDS", spot_light_shadow_map_strand);
  }
  {
    const auto brdf_lut = std::make_shared<GraphicsPipeline>();
    brdf_lut->vertex_shader = Resources::GetResource<Shader>("TEXTURE_PASS_THROUGH_VERT");
    brdf_lut->fragment_shader = Resources::GetResource<Shader>("ENVIRONMENTAL_MAP_BRDF_FRAG");
    brdf_lut->geometry_type = GeometryType::Mesh;

    brdf_lut->depth_attachment_format = Constants::render_texture_depth;
    brdf_lut->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    brdf_lut->color_attachment_formats = {1, VK_FORMAT_R16G16_SFLOAT};

    brdf_lut->Initialize();
    RegisterGraphicsPipeline("ENVIRONMENTAL_MAP_BRDF", brdf_lut);
  }
  {
    const auto equirectangular_to_cubemap = std::make_shared<GraphicsPipeline>();
    equirectangular_to_cubemap->vertex_shader = Resources::GetResource<Shader>("EQUIRECTANGULAR_MAP_TO_CUBEMAP_VERT");
    equirectangular_to_cubemap->fragment_shader = Resources::GetResource<Shader>("EQUIRECTANGULAR_MAP_TO_CUBEMAP_FRAG");
    equirectangular_to_cubemap->geometry_type = GeometryType::Mesh;

    equirectangular_to_cubemap->depth_attachment_format = Constants::shadow_map;
    equirectangular_to_cubemap->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    equirectangular_to_cubemap->color_attachment_formats = {1, Constants::texture_2d};
    equirectangular_to_cubemap->descriptor_set_layouts.emplace_back(
        GetDescriptorSetLayout("RENDER_TEXTURE_PRESENT_LAYOUT"));

    auto& push_constant_range = equirectangular_to_cubemap->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(glm::mat4) + sizeof(float);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    equirectangular_to_cubemap->Initialize();
    RegisterGraphicsPipeline("EQUIRECTANGULAR_TO_CUBEMAP", equirectangular_to_cubemap);
  }
  {
    const auto irradiance_construct = std::make_shared<GraphicsPipeline>();
    irradiance_construct->vertex_shader = Resources::GetResource<Shader>("EQUIRECTANGULAR_MAP_TO_CUBEMAP_VERT");
    irradiance_construct->fragment_shader = Resources::GetResource<Shader>("IRRADIANCE_CONSTRUCT_FRAG");
    irradiance_construct->geometry_type = GeometryType::Mesh;

    irradiance_construct->depth_attachment_format = Constants::shadow_map;
    irradiance_construct->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    irradiance_construct->color_attachment_formats = {1, Constants::texture_2d};
    irradiance_construct->descriptor_set_layouts.emplace_back(GetDescriptorSetLayout("RENDER_TEXTURE_PRESENT_LAYOUT"));

    auto& push_constant_range = irradiance_construct->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(glm::mat4) + sizeof(float);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    irradiance_construct->Initialize();
    RegisterGraphicsPipeline("IRRADIANCE_CONSTRUCT", irradiance_construct);
  }
  {
    const auto prefilter_construct = std::make_shared<GraphicsPipeline>();
    prefilter_construct->vertex_shader = Resources::GetResource<Shader>("EQUIRECTANGULAR_MAP_TO_CUBEMAP_VERT");
    prefilter_construct->fragment_shader = Resources::GetResource<Shader>("PREFILTER_CONSTRUCT_FRAG");
    prefilter_construct->geometry_type = GeometryType::Mesh;

    prefilter_construct->depth_attachment_format = Constants::shadow_map;
    prefilter_construct->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    prefilter_construct->color_attachment_formats = {1, Constants::texture_2d};
    prefilter_construct->descriptor_set_layouts.emplace_back(GetDescriptorSetLayout("RENDER_TEXTURE_PRESENT_LAYOUT"));

    auto& push_constant_range = prefilter_construct->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(glm::mat4) + sizeof(float);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    prefilter_construct->Initialize();
    RegisterGraphicsPipeline("PREFILTER_CONSTRUCT", prefilter_construct);
  }
  {
    const auto gizmos = std::make_shared<GraphicsPipeline>();
    gizmos->vertex_shader = Resources::GetResource<Shader>("GIZMOS_VERT");
    gizmos->fragment_shader = Resources::GetResource<Shader>("GIZMOS_FRAG");
    gizmos->geometry_type = GeometryType::Mesh;

    gizmos->depth_attachment_format = Constants::render_texture_depth;
    gizmos->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    gizmos->color_attachment_formats = {1, Constants::render_texture_color};
    gizmos->descriptor_set_layouts.emplace_back(per_frame_layout);

    auto& push_constant_range = gizmos->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos->Initialize();
    RegisterGraphicsPipeline("GIZMOS", gizmos);
  }
  {
    const auto gizmos_strands = std::make_shared<GraphicsPipeline>();
    gizmos_strands->vertex_shader = Resources::GetResource<Shader>("GIZMOS_STRANDS_VERT");
    gizmos_strands->tessellation_control_shader = Resources::GetResource<Shader>("GIZMOS_STRANDS_TESC");
    gizmos_strands->tessellation_evaluation_shader = Resources::GetResource<Shader>("GIZMOS_STRANDS_TESE");
    gizmos_strands->geometry_shader = Resources::GetResource<Shader>("GIZMOS_STRANDS_GEOM");
    gizmos_strands->fragment_shader = Resources::GetResource<Shader>("GIZMOS_FRAG");
    gizmos_strands->geometry_type = GeometryType::Strands;

    gizmos_strands->depth_attachment_format = Constants::render_texture_depth;
    gizmos_strands->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    gizmos_strands->tessellation_patch_control_points = 4;

    gizmos_strands->color_attachment_formats = {1, Constants::render_texture_color};
    gizmos_strands->descriptor_set_layouts.emplace_back(per_frame_layout);

    auto& push_constant_range = gizmos_strands->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_strands->Initialize();
    RegisterGraphicsPipeline("GIZMOS_STRANDS", gizmos_strands);
  }
  {
    const auto gizmos_normal_colored = std::make_shared<GraphicsPipeline>();
    gizmos_normal_colored->vertex_shader = Resources::GetResource<Shader>("GIZMOS_NORMAL_COLORED_VERT");
    gizmos_normal_colored->fragment_shader = Resources::GetResource<Shader>("GIZMOS_COLORED_FRAG");
    gizmos_normal_colored->geometry_type = GeometryType::Mesh;

    gizmos_normal_colored->depth_attachment_format = Constants::render_texture_depth;
    gizmos_normal_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    gizmos_normal_colored->color_attachment_formats = {1, Constants::render_texture_color};
    gizmos_normal_colored->descriptor_set_layouts.emplace_back(per_frame_layout);

    gizmos_normal_colored->tessellation_patch_control_points = 4;

    auto& push_constant_range = gizmos_normal_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_normal_colored->Initialize();
    RegisterGraphicsPipeline("GIZMOS_NORMAL_COLORED", gizmos_normal_colored);
  }
  {
    const auto gizmos_strands_normal_colored = std::make_shared<GraphicsPipeline>();
    gizmos_strands_normal_colored->vertex_shader = Resources::GetResource<Shader>("GIZMOS_STRANDS_NORMAL_COLORED_VERT");
    gizmos_strands_normal_colored->tessellation_control_shader =
        Resources::GetResource<Shader>("GIZMOS_STRANDS_COLORED_TESC");
    gizmos_strands_normal_colored->tessellation_evaluation_shader =
        Resources::GetResource<Shader>("GIZMOS_STRANDS_COLORED_TESE");
    gizmos_strands_normal_colored->geometry_shader = Resources::GetResource<Shader>("GIZMOS_STRANDS_COLORED_GEOM");
    gizmos_strands_normal_colored->fragment_shader = Resources::GetResource<Shader>("GIZMOS_COLORED_FRAG");
    gizmos_strands_normal_colored->geometry_type = GeometryType::Strands;

    gizmos_strands_normal_colored->depth_attachment_format = Constants::render_texture_depth;
    gizmos_strands_normal_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    gizmos_strands_normal_colored->color_attachment_formats = {1, Constants::render_texture_color};
    gizmos_strands_normal_colored->descriptor_set_layouts.emplace_back(per_frame_layout);

    gizmos_strands_normal_colored->tessellation_patch_control_points = 4;

    auto& push_constant_range = gizmos_strands_normal_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_strands_normal_colored->Initialize();
    RegisterGraphicsPipeline("GIZMOS_STRANDS_NORMAL_COLORED", gizmos_strands_normal_colored);
  }
  {
    const auto gizmos_vertex_colored = std::make_shared<GraphicsPipeline>();
    gizmos_vertex_colored->vertex_shader = Resources::GetResource<Shader>("GIZMOS_VERTEX_COLORED_VERT");
    gizmos_vertex_colored->fragment_shader = Resources::GetResource<Shader>("GIZMOS_COLORED_FRAG");
    gizmos_vertex_colored->geometry_type = GeometryType::Mesh;

    gizmos_vertex_colored->depth_attachment_format = Constants::render_texture_depth;
    gizmos_vertex_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    gizmos_vertex_colored->color_attachment_formats = {1, Constants::render_texture_color};
    gizmos_vertex_colored->descriptor_set_layouts.emplace_back(per_frame_layout);

    auto& push_constant_range = gizmos_vertex_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_vertex_colored->Initialize();
    RegisterGraphicsPipeline("GIZMOS_VERTEX_COLORED", gizmos_vertex_colored);
  }
  {
    const auto gizmos_strands_vertex_colored = std::make_shared<GraphicsPipeline>();
    gizmos_strands_vertex_colored->vertex_shader = Resources::GetResource<Shader>("GIZMOS_STRANDS_VERTEX_COLORED_VERT");
    gizmos_strands_vertex_colored->tessellation_control_shader =
        Resources::GetResource<Shader>("GIZMOS_STRANDS_COLORED_TESC");
    gizmos_strands_vertex_colored->tessellation_evaluation_shader =
        Resources::GetResource<Shader>("GIZMOS_STRANDS_COLORED_TESE");
    gizmos_strands_vertex_colored->geometry_shader = Resources::GetResource<Shader>("GIZMOS_STRANDS_COLORED_GEOM");
    gizmos_strands_vertex_colored->fragment_shader = Resources::GetResource<Shader>("GIZMOS_COLORED_FRAG");
    gizmos_strands_vertex_colored->geometry_type = GeometryType::Strands;

    gizmos_strands_vertex_colored->tessellation_patch_control_points = 4;

    gizmos_strands_vertex_colored->depth_attachment_format = Constants::render_texture_depth;
    gizmos_strands_vertex_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    gizmos_strands_vertex_colored->color_attachment_formats = {1, Constants::render_texture_color};
    gizmos_strands_vertex_colored->descriptor_set_layouts.emplace_back(per_frame_layout);

    gizmos_strands_vertex_colored->tessellation_patch_control_points = 4;

    auto& push_constant_range = gizmos_strands_vertex_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_strands_vertex_colored->Initialize();
    RegisterGraphicsPipeline("GIZMOS_STRANDS_VERTEX_COLORED", gizmos_strands_vertex_colored);
  }
  {
    const auto gizmos_instanced_colored = std::make_shared<GraphicsPipeline>();
    gizmos_instanced_colored->vertex_shader = Resources::GetResource<Shader>("GIZMOS_INSTANCED_COLORED_VERT");
    gizmos_instanced_colored->fragment_shader = Resources::GetResource<Shader>("GIZMOS_COLORED_FRAG");
    gizmos_instanced_colored->geometry_type = GeometryType::Mesh;

    gizmos_instanced_colored->depth_attachment_format = Constants::render_texture_depth;
    gizmos_instanced_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    gizmos_instanced_colored->color_attachment_formats = {1, Constants::render_texture_color};
    gizmos_instanced_colored->descriptor_set_layouts.emplace_back(per_frame_layout);
    gizmos_instanced_colored->descriptor_set_layouts.emplace_back(instanced_data_layout);

    auto& push_constant_range = gizmos_instanced_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_instanced_colored->Initialize();
    RegisterGraphicsPipeline("GIZMOS_INSTANCED_COLORED", gizmos_instanced_colored);
  }
}
