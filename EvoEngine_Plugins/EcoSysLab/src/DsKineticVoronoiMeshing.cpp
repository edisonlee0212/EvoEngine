#include "DsKineticVoronoiMeshing.hpp"
#include "ComputePipeline.hpp"
#include "DynamicStrands.hpp"
#include "KineticDelaunay.hpp"
#include "MeshIntersection.hpp"
#include "Platform/Platform.hpp"
#include "ProgressBar.hpp"
#include "SegmentBuilder.hpp"
#include "Shader.hpp"
#include "VoronoiMesh.hpp"

using namespace eco_sys_lab_plugin;

// helper functions

void DsKineticVoronoiMeshing::RunMeshingAlgorithm(std::vector<kinDS::CubicHermiteSpline<2>> strand_splines) {
  EVOENGINE_LOG("Starting Kinetic Delaunay Voronoi Meshing...");
  kinDS::KineticDelaunay kinetic_delaunay(strand_splines);

  kinetic_delaunay.init();
  kinDS::SegmentBuilder mesh_builder(kinetic_delaunay, strand_splines);
  mesh_builder.init();
  auto points = kinetic_delaunay.getPointsAt(0.0);

  size_t section_count = kinetic_delaunay.getSectionCount();

  evo_engine::ProgressBar section_progress_bar(0, section_count, "Computing Kinetic Voronoi Sections",
                                               evo_engine::ProgressBar::Display::Absolute);
  for (size_t i = 0; i < section_count; ++i) {
    section_progress_bar.Update(i);
    if (i != 0)
      mesh_builder.betweenSections(i);
    kinetic_delaunay.advanceOneSection(mesh_builder);

    points = kinetic_delaunay.getPointsAt(static_cast<double>(i + 1));
  }
  section_progress_bar.Finish();

  EVOENGINE_LOG("Finalizing Kinetic Delaunay Voronoi Meshing...");
  mesh_builder.finalize(section_count);

  auto meshes = mesh_builder.extractSegmentMeshlets();

  auto& boundary_mesh = mesh_builder.getBoundaryMesh();

  // intersect all meshes with the boundary mesh and save the result

  std::vector<kinDS::Mesh> intersections;
  intersections.reserve(meshes.size());

  ProgressBar intersection_progress_bar(0, meshes.size(), "Computing Mesh Intersections",
                                        ProgressBar::Display::Absolute, 50);
  for (size_t i = 0; i < meshes.size(); ++i) {
    intersection_progress_bar.Update(i);

    meshes[i].checkForDegenerateTriangles();
    auto intersection = kinDS::MeshIntersection::intersect(meshes[i], boundary_mesh);
    intersections.push_back(intersection);
  }
  intersection_progress_bar.Finish();

  // for now, just combine all meshes into one
  kinDS::Mesh combined_mesh;
  for (const auto& mesh : intersections) {
    combined_mesh += mesh;
  }

  // export combined mesh
  combined_mesh.mergeDuplicateVertices(0.0001);

  // convert to output format

  for (const auto& v : combined_mesh.getVertices()) {
    // v is a relative position in 2D, we need to convert it to 3D

    GpuSegmentMeshletVertex vertex;
    vertex.relative_position_x = v[0];
    vertex.relative_position_y = v[2];
    vertex.relative_position_z = v[1];
    vertex.segment_index = 0;  // Placeholder, set appropriately if segment information is available
    segment_meshlet_vertices.push_back(vertex);
  }

  const auto& vertex_indices = combined_mesh.getVertexIndices();
  for (size_t i = 0; i < vertex_indices.size(); i += 3) {
    GpuSegmentMeshletTriangle triangle;
    triangle.vertex_index0 = static_cast<unsigned int>(vertex_indices[i]);
    triangle.vertex_index1 = static_cast<unsigned int>(vertex_indices[i + 1]);
    triangle.vertex_index2 = static_cast<unsigned int>(vertex_indices[i + 2]);
    triangle.twin_triangle_index = -1;  // Placeholder, set appropriately if twin information is available
    segment_meshlet_triangles.push_back(triangle);
  }
}
// DsKineticVoronoiMeshing implementation
DsKineticVoronoiMeshing::RenderSettings DsKineticVoronoiMeshing::render_settings = {};

DsKineticVoronoiMeshing::DsKineticVoronoiMeshing() {
}

DsKineticVoronoiMeshing::~DsKineticVoronoiMeshing() {
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::InitBuffer(
    VkBufferCreateInfo& buffer_create_info, VmaAllocationCreateInfo& buffer_vma_allocation_create_info) {
  device_segment_meshlet_triangles_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_meshlet_vertices_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::InitData(
    const DynamicStrandsInitializeParameters& initialize_parameters, const StrandModelSkeleton& strand_model_skeleton,
    const StrandModelStrandGroup& strand_model_strand_group, DtsStrandGroup& randomly_subdivided_strand_group,
    DtsStrandGroup& uniformly_subdivided_strand_group) {
  // TODO: Compute Voronoi meshing here and initialize buffers accordingly.

  // Some test data:
  /*segment_meshlet_vertices = {
      // --- Meshlet 0 (tetrahedron A) ---
      {0.0f, 0.0f, 0.0f, 0},  // 0
      {1.0f, 0.0f, 0.0f, 0},  // 1
      {0.0f, 1.0f, 0.0f, 0},  // 2
      {0.0f, 0.0f, 1.0f, 0},  // 3

      // --- Meshlet 1 (tetrahedron B) ---
      {0.0f, 0.0f, 0.0f, 1},   // 4
      {1.0f, 0.0f, 0.0f, 1},   // 5
      {0.0f, 1.0f, 0.0f, 1},   // 6
      {0.0f, 0.0f, -1.0f, 1},  // 7
  };

  // Triangles are listed in global order. Twin indices refer to these positions.
  segment_meshlet_triangles = {
      // ===== Meshlet 0 triangles =====
      // Shared face (0,1,2)
      {0, 1, 2, 3},  // 0: twin with triangle 3 (meshlet 1 face)

      // Other faces
      {0, 1, 3, -1},  // 1
      {1, 2, 3, -1},  // 2
      {2, 0, 3, -1},  // 3

      // ===== Meshlet 1 triangles =====
      // Shared face (4,5,6), same geometry in meshlet-local space
      {4, 5, 6, 0},  // 4: twin with triangle 0

      // Other faces
      {4, 5, 7, -1},  // 5
      {5, 6, 7, -1},  // 6
      {6, 4, 7, -1},  // 7
  };*/
  // We just compute uniform particles as before:
  // First compute uniform particles
  const auto& target_strands = randomly_subdivided_strand_group.PeekStrands();

  strand_model_strand_group.UniformlySubdivide<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData>(
      uniformly_subdivided_strand_group, initialize_parameters.uniform_subdivision,
      [&](const StrandHandle src_handle, DtsStrandData& strand_data) {

      },
      [&](const float start_root_distance, const float end_root_distance, const StrandSegmentHandle src_handle,
          const uint32_t original_segment_index, const float segment_t, DtsStrandSegmentData& segment_data,
          const uint32_t sub_segment_index) {
        const auto& src_segment_data = strand_model_strand_group.PeekStrandSegmentData(src_handle);
        segment_data.node_handle = src_segment_data.node_handle;
        segment_data.original_segment_handle = src_handle;
        segment_data.original_segment_index = original_segment_index;
        segment_data.segment_index = sub_segment_index;
        segment_data.original_segment_t = segment_t;
        segment_data.start_root_distance = start_root_distance;
        segment_data.end_root_distance = end_root_distance;
        const auto& strand_segment = strand_model_strand_group.PeekStrandSegment(src_handle);
        const auto& strand = strand_model_strand_group.PeekStrand(strand_segment.GetStrandHandle());
        const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();

        glm::vec2 p0, p1, p3;
        const glm::vec2 p2 = src_segment_data.profile_position;
        float d0, d1, d3;
        const float d2 = src_segment_data.initial_distance_to_boundary;
        if (src_handle == strand_segment_handles.front()) {
          d1 = d2;
          d0 = d1 * 2.0f - d2;

          p1 = p2;
          p0 = p1 * 2.0f - p2;

        } else if (strand_segment.GetPrevHandle() == strand_segment_handles.front()) {
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          d0 = d2;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = p2;
          p1 = prev_segment_data.profile_position;

        } else {
          const auto& prev_segment = strand_model_strand_group.PeekStrandSegment(strand_segment.GetPrevHandle());
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          const auto& prev_prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(prev_segment.GetPrevHandle());
          d0 = prev_prev_segment_data.initial_distance_to_boundary;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = prev_prev_segment_data.profile_position;
          p1 = prev_segment_data.profile_position;
        }
        if (src_handle == strand_segment_handles.back()) {
          d3 = d2 * 2.0f - d1;

          p3 = p2 * 2.0f - p1;

        } else {
          const auto& next_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetNextHandle());
          d3 = next_segment_data.initial_distance_to_boundary;

          p3 = next_segment_data.profile_position;
        }
        segment_data.initial_distance_to_boundary = Strands::CubicInterpolation(d0, d1, d2, d3, segment_t);
        segment_data.profile_position = Strands::CubicInterpolation(p0, p1, p2, p3, segment_t);

        const auto calculate_polar_coordinates = [](const glm::vec2& profile_position) {
          const auto r = glm::length(profile_position);
          if (r <= glm::epsilon<float>()) {
            return glm::vec2(0.0f);
          }
          if (profile_position.y >= 0)
            return glm::vec2(r, glm::acos(profile_position.x / r));
          return glm::vec2(r, -glm::acos(profile_position.x / r));
        };

        segment_data.profile_polar_coordinate = calculate_polar_coordinates(segment_data.profile_position);
      },
      (initialize_parameters.min_segment_length + initialize_parameters.max_segment_length) * .5f * .01f);

  std::vector<std::vector<kinDS::Point<2>>> strand_guide_points(target_strands.size());
  std::vector<int> uniform_particle_offsets(target_strands.size());
  if (!uniform_particle_offsets.empty())
    uniform_particle_offsets[0] = 0;
  for (uint32_t strand_index = 1; strand_index < target_strands.size(); strand_index++) {
    uniform_particle_offsets[strand_index] =
        uniform_particle_offsets[strand_index - 1] +
        uniformly_subdivided_strand_group.PeekStrand(strand_index - 1).PeekStrandSegmentHandles().size() + 1;
  }

  Jobs::RunParallelFor(target_strands.size(), [&](const size_t strand_index) {
    auto& random_subdivided_strand = target_strands[strand_index];
    auto& uniformly_subdivided_strand = uniformly_subdivided_strand_group.PeekStrand(strand_index);
    const auto uniform_particle_offset = uniform_particle_offsets[strand_index];

    auto& first_uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(
        uniformly_subdivided_strand.PeekStrandSegmentHandles()[0]);

    // First 2 particles within same strand will always have same profile position/polar coordinate.
    kinDS::Point<2> profile_position{first_uniform_segment_data.profile_position.x,
                                     first_uniform_segment_data.profile_position.y};
    strand_guide_points[strand_index].push_back(profile_position);

    int last_index_with_new_node = 0;
    float previous_root_distance = 0.0f;
    for (int uniform_segment_index = 0;
         uniform_segment_index < uniformly_subdivided_strand.PeekStrandSegmentHandles().size();
         uniform_segment_index++) {
      const auto& uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(
          uniformly_subdivided_strand.PeekStrandSegmentHandles()[uniform_segment_index]);
      kinDS::Point<2> profile_position{uniform_segment_data.profile_position.x,
                                       uniform_segment_data.profile_position.y};
      strand_guide_points[strand_index].push_back(profile_position);
    }
  });

  // Proof of concept, just assume we have one trunk with no branches and all strands have the same length
  // construct cubic hermite spline for each strand
  std::vector<kinDS::CubicHermiteSpline<2>> strand_splines;
  for (const auto& guide_points : strand_guide_points) {
    strand_splines.push_back(kinDS::CubicHermiteSpline<2>(guide_points));
  }

  RunMeshingAlgorithm(strand_splines);
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::InitializationGraphicsPipeline(
    const DynamicStrandsInitializeParameters& initialize_parameters) {
  // Don't need this for now
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::BuildRenderComputePipelines() {
  // Don't need this for now
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::RenderCompute() const {
  // Don't need this for now
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::BuildRenderingPipelines() {
  BuildSegmentMeshletsRenderingPipelines();
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Download() {
  if (!segment_meshlet_vertices.empty()) {
    device_segment_meshlet_vertices_buffer->DownloadVector(segment_meshlet_vertices, segment_meshlet_vertices.size());
  }
  if (!segment_meshlet_triangles.empty()) {
    device_segment_meshlet_triangles_buffer->DownloadVector(segment_meshlet_triangles,
                                                            segment_meshlet_triangles.size());
  }
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Upload() {
  device_segment_meshlet_vertices_buffer->UploadVector(segment_meshlet_vertices);
  device_segment_meshlet_vertices_buffer->SetDebugName("Segment Meshlet Vertices Buffer");
  device_segment_meshlet_triangles_buffer->UploadVector(segment_meshlet_triangles);
  device_segment_meshlet_triangles_buffer->SetDebugName("Segment Meshlet Triangles Buffer");
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Clear() {
  segment_meshlet_vertices.clear();
  segment_meshlet_triangles.clear();
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::UpdateBindings() const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      8, device_segment_meshlet_vertices_buffer, 0);
  dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      9, device_segment_meshlet_triangles_buffer, 0);
}

bool eco_sys_lab_plugin::DsKineticVoronoiMeshing::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  // just output the size of the meshlets for now
  ImGui::Text((std::string("Segment Meshlets Vertices: ") + std::to_string(segment_meshlet_vertices.size())).c_str());
  ImGui::Text((std::string("Segment Meshlets Triangles: ") + std::to_string(segment_meshlet_triangles.size())).c_str());
  return false;
}

void DsKineticVoronoiMeshing::OnInspectRenderSettings(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Render Segment Meshlets", &render_settings.render_segment_meshlets);
  if (render_settings.render_segment_meshlets) {
    if (ImGui::Button("Rebuild segment meshlet pipelines")) {
      BuildSegmentMeshletsRenderingPipelines();
    }
  }
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::RegisterRenderInstances(Handle& rendering_instance_handle,
                                                                          std::shared_ptr<Scene> scene, Entity& owner) {
  // TODO: add settings
  RegisterSegmentMeshletsRenderInstance(rendering_instance_handle, scene, owner);
}

void DsKineticVoronoiMeshing::RegisterSegmentMeshletsRenderInstance(Handle& rendering_instance_handle,
                                                                    std::shared_ptr<Scene> scene, Entity& owner) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto inner_wood_material = dynamic_strands->materials.inner_wood_material_ref.Get<Material>();
  const auto snow_material = dynamic_strands->materials.snow_material_ref.Get<Material>();
  if (const auto bark_material = dynamic_strands->materials.bark_material_ref.Get<Material>();
      bark_material && inner_wood_material && snow_material) {
    if (!dynamic_strands->segments.empty()) {
      if (segment_meshlet_point_light_render_pipeline && segment_meshlet_point_light_render_pipeline->Initialized()) {
        render_layer->RenderToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToPointLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                            vk_command_buffer, view);
        });
      }
      if (segment_meshlet_spot_light_render_pipeline && segment_meshlet_spot_light_render_pipeline->Initialized()) {
        render_layer->RenderToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToSpotLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                           vk_command_buffer, view);
        });
      }
      if (segment_meshlet_directional_light_render_pipeline &&
          segment_meshlet_directional_light_render_pipeline->Initialized()) {
        render_layer->RenderToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToDirectionalLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                                  vk_command_buffer, view);
        });
      }
      if (segment_meshlet_render_pipeline && segment_meshlet_render_pipeline->Initialized()) {
        const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto renderer_handle = rendering_instance_handle;
        current_render_storage->RegisterRenderInstance(scene, owner, renderer_handle, bark_material);
        const auto inner_material_index = current_render_storage->RegisterMaterial(inner_wood_material);
        const auto snow_material_index = current_render_storage->RegisterMaterial(snow_material);
        render_layer->DeferredRenderingAllCameras(
            [=](const VkCommandBuffer vk_command_buffer,
                const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                const RenderLayer::DeferredRenderingView& view) {
              return RenderSegmentMeshletsToCameraDeferred(renderer_handle, inner_material_index, snow_material_index,
                                                           render_settings.segment_meshlet_render_parameters,
                                                           vk_command_buffer, geometry_pass_color_attachment_infos,
                                                           view, VK_POLYGON_MODE_FILL);
            });
      }
    }
  }
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Visualize(
    const std::shared_ptr<Camera>& target_camera, const DynamicStrandsInitializeParameters& initialize_parameters,
    const DynamicStrandsVisualizationParameters& visualization_parameters) {
  // TODO
}

void DsKineticVoronoiMeshing::BuildSegmentMeshletsRenderingPipelines() {
  segment_meshlet_point_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_point_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");
  segment_meshlet_point_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "PointLightShadowMap.mesh");
  segment_meshlet_point_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  segment_meshlet_point_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_point_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_point_light_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_point_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_point_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& point_light_push_constant_range =
      segment_meshlet_point_light_render_pipeline->push_constant_ranges.emplace_back();
  point_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  point_light_push_constant_range.offset = 0;
  point_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_point_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_spot_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_spot_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");

  // TODO: fix path
  segment_meshlet_spot_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "SpotLightShadowMap.mesh");
  segment_meshlet_spot_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  segment_meshlet_spot_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_spot_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_spot_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& spot_light_push_constant_range =
      segment_meshlet_spot_light_render_pipeline->push_constant_ranges.emplace_back();
  spot_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  spot_light_push_constant_range.offset = 0;
  spot_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_spot_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_directional_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_directional_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");

  // TODO: fix path
  segment_meshlet_directional_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "DirectionalLightShadowMap.mesh");
  segment_meshlet_directional_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  segment_meshlet_directional_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(
      DynamicStrands::strands_layout);
  segment_meshlet_directional_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_directional_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& directional_light_push_constant_range =
      segment_meshlet_directional_light_render_pipeline->push_constant_ranges.emplace_back();
  directional_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  directional_light_push_constant_range.offset = 0;
  directional_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_directional_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");
  segment_meshlet_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/Rendering.mesh");
  segment_meshlet_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Fragment/DynamicStrands/Rendering/Branches.frag");
  segment_meshlet_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::lighting_layout);
  segment_meshlet_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  segment_meshlet_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  segment_meshlet_render_pipeline->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
  auto& push_constant_range = segment_meshlet_render_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_render_pipeline->Initialize();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToPointLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, const VkCommandBuffer vk_command_buffer,
    const RenderLayer::PointLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = view.face_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_point_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_point_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_point_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_point_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_point_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToSpotLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const RenderLayer::SpotLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = 0;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_spot_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_spot_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_spot_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_spot_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_spot_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}
uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToDirectionalLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const RenderLayer::DirectionalLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = view.split_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_directional_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_directional_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_directional_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_directional_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_directional_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToCameraDeferred(
    const Handle& renderer_handle, int inner_wood_material_index, int snow_material_index,
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view, VkPolygonMode polygon_mode) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  if (!Platform::Constants::support_mesh_shader) {
    EVOENGINE_LOG("Failed to render! Mesh shader unsupported!")
    return 0;
  }

  // TODO: If we add any compute shaders, also check them here
  if (!segment_meshlet_render_pipeline || !segment_meshlet_render_pipeline->Initialized()) {
    return 0;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  SegmentMeshletPushConstant render_push_constant;
  render_push_constant.index1.instance_index =
      Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage()->GetRenderInstanceIndex(renderer_handle);
  render_push_constant.index2.camera_index = view.camera_index;
  render_push_constant.vertex_count = segment_meshlet_vertices.size();
  render_push_constant.triangle_count = segment_meshlet_triangles.size();
  segment_meshlet_render_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
  segment_meshlet_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_render_pipeline->states.polygon_mode = polygon_mode;
  segment_meshlet_render_pipeline->states.line_width = 2.0f;
  segment_meshlet_render_pipeline->states.ApplyAllStates(vk_command_buffer);

#ifdef USE_RENDERDOC
  if (rdoc_api) {
    rdoc_api->StartFrameCapture(NULL, NULL);
    EVOENGINE_LOG("RDOC API detected!");
  }
#endif  //  USERENDERDOC

  segment_meshlet_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                     RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_render_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                                     RenderLayer::GetLightingDescriptorSet()->GetVkDescriptorSet());

  segment_meshlet_render_pipeline->PushConstant(vk_command_buffer, 0, render_push_constant);

  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
#ifdef USE_RENDERDOC
  if (rdoc_api)
    rdoc_api->EndFrameCapture(NULL, NULL);
#endif
  return dynamic_strands->segments.size();
}

/* uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletVisualizationToCameraDeferred(
    const Handle& renderer_handle, const DynamicStrandsInitializeParameters& initialize_parameters,
    const SmallSegmentsVisualizationRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view) const {
}*/
