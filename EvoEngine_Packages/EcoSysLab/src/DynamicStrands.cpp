#include "DynamicStrands.hpp"
#include <cstring>
#include <functional>
#include <type_traits>
#include "Application.hpp"
#include "DsAlphaShapeUtils.hpp"
#include "DsColliders.hpp"
#include "DsConstraints.hpp"
#include "DsPhysics.hpp"
#include "DynamicStrandsProfiler.hpp"
#include "GpuProfiler.hpp"
#include "Shader.hpp"
#include "UVMapUtils.hpp"
#include "glm/gtx/quaternion.hpp"
using namespace eco_sys_lab_package;

#ifdef USE_CGAL
inline glm::vec3 cgal_to_glm(const Point_CGAL& p) {
  return {p.x(), p.y(), p.z()};
}
#endif

namespace {
constexpr size_t kGpuSegmentStride = offsetof(DynamicStrands::GpuSegment, particle0);
constexpr size_t kGpuSegmentDataStride = offsetof(DynamicStrands::GpuSegmentData, pair_handles);

static_assert(std::is_standard_layout_v<DynamicStrands::GpuSegment>);
static_assert(std::is_trivially_copyable_v<DynamicStrands::GpuSegment>);
static_assert(kGpuSegmentStride == 480);
static_assert(offsetof(DynamicStrands::GpuSegment, particle1) ==
              kGpuSegmentStride + sizeof(DynamicStrands::GpuParticle));
static_assert(sizeof(DynamicStrands::GpuSegment) == kGpuSegmentStride + 2 * sizeof(DynamicStrands::GpuParticle));
static_assert(std::is_standard_layout_v<DynamicStrands::GpuSegmentData>);
static_assert(std::is_trivially_copyable_v<DynamicStrands::GpuSegmentData>);
static_assert(kGpuSegmentDataStride == 48);
static_assert(sizeof(DynamicStrands::GpuSegmentConnectionHandles) == BUNDLE_MAX_CONNECTION * sizeof(int));
static_assert(sizeof(DynamicStrands::GpuSegmentData) ==
              kGpuSegmentDataStride + sizeof(DynamicStrands::GpuSegmentConnectionHandles));

struct PackedSegments {
  std::vector<std::byte> segments;
  std::vector<DynamicStrands::GpuParticle> particle0s;
  std::vector<DynamicStrands::GpuParticle> particle1s;
};

PackedSegments MakePackedSegments(const size_t count) {
  PackedSegments packed;
  packed.segments.resize(kGpuSegmentStride * count);
  packed.particle0s.resize(count);
  packed.particle1s.resize(count);
  return packed;
}

PackedSegments PackSegments(const std::vector<DynamicStrands::GpuSegment>& source) {
  auto packed = MakePackedSegments(source.size());
  for (size_t index = 0; index < source.size(); ++index) {
    std::memcpy(packed.segments.data() + index * kGpuSegmentStride, &source[index], kGpuSegmentStride);
    packed.particle0s[index] = source[index].particle0;
    packed.particle1s[index] = source[index].particle1;
  }
  return packed;
}

void UnpackSegments(std::vector<DynamicStrands::GpuSegment>& destination, const PackedSegments& packed) {
  for (size_t index = 0; index < destination.size(); ++index) {
    std::memcpy(&destination[index], packed.segments.data() + index * kGpuSegmentStride, kGpuSegmentStride);
    destination[index].particle0 = packed.particle0s[index];
    destination[index].particle1 = packed.particle1s[index];
  }
}

struct PackedSegmentData {
  std::vector<std::byte> data;
  std::vector<DynamicStrands::GpuSegmentConnectionHandles> connection_handles;
};

PackedSegmentData MakePackedSegmentData(const size_t count) {
  PackedSegmentData packed;
  packed.data.resize(kGpuSegmentDataStride * count);
  packed.connection_handles.resize(count);
  return packed;
}

PackedSegmentData PackSegmentData(const std::vector<DynamicStrands::GpuSegmentData>& source) {
  auto packed = MakePackedSegmentData(source.size());
  for (size_t index = 0; index < source.size(); ++index) {
    std::memcpy(packed.data.data() + index * kGpuSegmentDataStride, &source[index], kGpuSegmentDataStride);
    std::memcpy(packed.connection_handles[index].handles, source[index].pair_handles,
                sizeof(packed.connection_handles[index].handles));
  }
  return packed;
}

void UnpackSegmentData(std::vector<DynamicStrands::GpuSegmentData>& destination, const PackedSegmentData& packed) {
  for (size_t index = 0; index < destination.size(); ++index) {
    std::memcpy(&destination[index], packed.data.data() + index * kGpuSegmentDataStride, kGpuSegmentDataStride);
    std::memcpy(destination[index].pair_handles, packed.connection_handles[index].handles,
                sizeof(packed.connection_handles[index].handles));
  }
}
}  // namespace

void DynamicStrands::ReleaseStaticGpuResources() {
  foliage_visualization_render_pipeline.reset();
  segment_pairs_visualization_render_pipeline.reset();
  segment_visualization_render_pipeline.reset();

  foliage_render_pipeline.reset();
  foliage_directional_light_render_pipeline.reset();
  foliage_spot_light_render_pipeline.reset();
  foliage_point_light_render_pipeline.reset();

  strands_layout.reset();
}

void DynamicStrands::Physics(const PhysicsParameters& physics_parameters,
                             const std::function<void()>& pre_step_action) {
  const auto& profiler_items = dynamic_strands_profiler::GetItems();
  if (pre_step) {
    const RecordedGpuProfilerScope gpu_scope(profiler_items.pre_step);
    pre_step->Execute(physics_parameters, *this);
  }
  {
    const RecordedGpuProfilerScope gpu_scope(profiler_items.interaction);
    pre_step_action();
  }
  {
    const RecordedGpuProfilerScope gpu_scope(profiler_items.physics);
    for (int sub_step_index = 0; sub_step_index < physics_parameters.sub_step; sub_step_index++) {
      if (prediction) {
        prediction->Execute(physics_parameters, *this);
      }
      if (fungus && physics_parameters.enable_fungus) {
        fungus->Execute(physics_parameters, *this);
      }
      const auto scene = ApplicationContext::Get().GetActiveScene();
      const auto* box_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsBoxCollider>();
      const auto* sphere_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsSphereCollider>();
      const auto* cylinder_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsCylinderCollider>();
      const auto for_each_collider_entity =
          [&](const std::function<void(const std::shared_ptr<IDsCollider>& dts)>& action) {
            if (box_collider_entities && !box_collider_entities->empty()) {
              for (const auto& i : *box_collider_entities) {
                const auto box_collider = scene->GetOrSetPrivateComponent<DsBoxCollider>(i).lock();
                if (scene->IsEntityEnabled(i) && box_collider->IsEnabled())
                  action(std::dynamic_pointer_cast<IDsCollider>(box_collider));
              }
            }
            if (sphere_collider_entities && !sphere_collider_entities->empty()) {
              for (const auto& i : *sphere_collider_entities) {
                const auto sphere_collider = scene->GetOrSetPrivateComponent<DsSphereCollider>(i).lock();
                if (scene->IsEntityEnabled(i) && sphere_collider->IsEnabled())
                  action(std::dynamic_pointer_cast<IDsCollider>(sphere_collider));
              }
            }
            if (cylinder_collider_entities && !cylinder_collider_entities->empty()) {
              for (const auto& i : *cylinder_collider_entities) {
                const auto cylinder_collider = scene->GetOrSetPrivateComponent<DsCylinderCollider>(i).lock();
                if (scene->IsEntityEnabled(i) && cylinder_collider->IsEnabled())
                  action(std::dynamic_pointer_cast<IDsCollider>(cylinder_collider));
              }
            }
          };
      {
        for (int iteration_i = 0; iteration_i < physics_parameters.position_constraint_iteration; iteration_i++) {
          for (const auto& c : constraints) {
            if (c->enabled)
              c->ProjectPositionConstraint(physics_parameters, *this);
          }
        }
        for_each_collider_entity([&](const std::shared_ptr<IDsCollider>& dts) {
          dts->ProjectPositionConstraint(physics_parameters, *this);
        });
      }
      if (velocity_update) {
        velocity_update->Execute(physics_parameters, *this);
      }

      {
        for (int iteration_i = 0; iteration_i < physics_parameters.velocity_constraint_iteration; iteration_i++) {
          for (const auto& c : constraints) {
            if (c->enabled)
              c->ProjectVelocityConstraint(physics_parameters, *this);
          }
        }
        for_each_collider_entity([&](const std::shared_ptr<IDsCollider>& dts) {
          dts->ProjectVelocityConstraint(physics_parameters, *this);
        });
      }

      simulated_time += physics_parameters.time_step / static_cast<float>(physics_parameters.sub_step);
      if (physics_parameters.enable_structural_damage) {
        structural_damage->Execute(physics_parameters, *this);
      }
    }
  }

  if (physics_parameters.enable_segment_disconnection) {
    const RecordedGpuProfilerScope gpu_scope(profiler_items.dynamic_grouping);
    CalculateGroups(physics_parameters);
  }

  if (physics_parameters.enable_segment_collision) {
    const RecordedGpuProfilerScope gpu_scope(profiler_items.segment_collision);
    dynamic_hashed_grid->BuildGrid(physics_parameters, *this);
    segment_collision->Execute(physics_parameters, *this);
    // collision_post_step->Execute(physics_parameters, *this);
  }

  // for (const auto& c : constraints) {
  //   if (c->enabled) {
  //     c->ProjectPositionConstraint(physics_parameters, *this);
  //     c->ProjectVelocityConstraint(physics_parameters, *this);
  //   }
  // }

  frame_index++;
}

void DynamicStrands::InitMeshingAlgorithm(MeshingType meshing_type) {
  // create new meshing object

  switch (meshing_type) {
    case MeshingType::AlphaShape:
      meshing = std::make_shared<DsAlphaShapeMeshing>();
      break;
    case MeshingType::KineticVoronoi:
      meshing = std::make_shared<DsKineticVoronoiMeshing>();
      break;
    default:
      EVOENGINE_ERROR("Unsupported meshing type.");
      return;
  }

  meshing->dynamic_strands = this;
}

void DynamicStrands::Init(MeshingType meshing_type) {
#ifdef USE_RENDERDOC
  if (rdoc_api == nullptr) {
    if (HMODULE mod = GetModuleHandleA("renderdoc.dll")) {
      pRENDERDOC_GetAPI RENDERDOC_GetAPI = (pRENDERDOC_GetAPI)GetProcAddress(mod, "RENDERDOC_GetAPI");
      int ret = RENDERDOC_GetAPI(eRENDERDOC_API_Version_1_1_2, (void**)&rdoc_api);
      assert(ret == 1);
    }
  }
#endif

  InitMeshingAlgorithm(meshing_type);

  if (!strands_layout) {
    strands_layout = std::make_shared<DescriptorSetLayout>();
    strands_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(6, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(7, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(8, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(9, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(10, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(11, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(12, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->Initialize();
  }
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  device_strands_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_nodes_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segments_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_particle0_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_particle1_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_pairs_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_data_list_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_connection_handles_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  meshing->InitBuffer(buffer_create_info, buffer_vma_allocation_create_info);

  device_hashed_grid_elements_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_hashed_grid_cell_starts_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_foliage_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
  strands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : strands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(strands_layout);
  }
  pre_step = std::make_shared<DsPreStep>();
  prediction = std::make_shared<DsPrediction>();
  velocity_update = std::make_shared<DsVelocityUpdate>();
  dynamic_hashed_grid = std::make_shared<DsDynamicHashedGrid>();
  segment_collision = std::make_shared<DsSegmentCollision>();
  collision_post_step = std::make_shared<DsSegmentCollisionPostStep>();
  structural_damage = std::make_shared<DsStructuralDamage>();
  fungus = std::make_shared<DsFungus>();

  meshing->BuildRenderComputePipelines();
  meshing->BuildRenderingPipelines();
  BuildFoliageRenderingPipelines();
  BuildSegmentPairsRenderingPipeline();
}

void DynamicStrands::RenderCompute() const {
  meshing->RenderCompute();
}

uint32_t DynamicStrands::GetFrameIndex() const {
  return frame_index;
}

float DynamicStrands::GetSimulatedTime() const {
  return simulated_time;
}

bool DynamicStrands::PhysicsParameters::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("Preset Settings")) {
    if (ImGui::Button("Log Crack")) {
      bundle_strength_factor = 1.2f;
      crack_bd_shrinkage_offset = 0.1f;
      crack_R_scale = 1.0f;
      crack_T_scale = 1.0f;
      boundary_strength_decay_factor = 3.0f;
      internal_pattern = 1;
      changed = true;
    }
    if (ImGui::Button("Oak Trunk Crack Process")) {
      bundle_strength_factor = 1.0f;
      crack_bd_shrinkage_offset = 0.0f;
      crack_R_scale = 0.0f;
      crack_T_scale = 1.0f;
      boundary_strength_decay_factor = 6.0f;
      internal_pattern = 1;
      changed = true;
    }
    if (ImGui::Button("Oak Trunk Full Process")) {
      bundle_strength_factor = 1.0f;
      crack_bd_shrinkage_offset = 0.0f;
      crack_R_scale = 0.0f;
      crack_T_scale = 1.0f;
      boundary_strength_decay_factor = 6.0f;
      internal_pattern = 1;
      bd_offset = 0.06f;
      HL_threshold = 0.1f;
      matrixAb = glm::mat3(0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 2.0f);
      // time_step = 0.005f;
      bb = 0.5f;
      be = 0.5f;  // ZY: Test it!
      changed = true;
    }
    if (ImGui::Button("Elm")) {
      matrixAb = glm::mat3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 20.0f);
      bb = 0.5f;
      HC_threshold = 0.5f;
      HL_threshold = 0.1f;
      moisture_breaking_rod = 1;
      bd_offset = 0.02f;
      leaf_break_from_moisture = 1;
      leaf_break_threshold = 0.4f;
      changed = true;
    }
    if (ImGui::Button("Spruce")) {
      matrixAb = glm::mat3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 20.0f);
      matrixAm = glm::mat3(0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.5f);
      bb = 0.5f;
      HC_threshold = 0.5f;
      HL_threshold = 0.1f;
      moisture_breaking_rod = 1;
      bd_offset = 0.015f;
      leaf_break_from_moisture = 1;
      leaf_break_threshold = 0.4f;
      // rod_strength_factor = 0.7f;
      changed = true;
    }
    if (ImGui::Button("Oak")) {
      matrixAb = glm::mat3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 5.0f);
      matrixAm = glm::mat3(0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.5f);
      bb = 0.5f;
      HC_threshold = 0.5f;
      HL_threshold = 0.1f;
      moisture_breaking_rod = 1;
      bd_offset = 0.02f;
      leaf_break_from_moisture = 1;
      leaf_break_threshold = 0.4f;
      changed = true;
    }
    ImGui::TreePop();
  }
  if (ImGui::DragFloat("Time step", &time_step, 0.001f, 0.001f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Sub step", &sub_step, 1, 1, 100)) {
    changed = true;
  }
  if (ImGui::IsItemFocused()) {
    if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
      sub_step = ImMin(sub_step + 1, 100);
      changed = true;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
      sub_step = ImMax(sub_step - 1, 1);
      changed = true;
    }
  }
  if (ImGui::Checkbox("Enable Fungus", &enable_fungus)) {
    changed = true;
  }
  if (ImGui::Checkbox("Enable Collision", &enable_segment_collision)) {
    changed = true;
  }
  if (ImGui::DragFloat("Rod segment strength factor", &rod_strength_factor, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Bundle strength factor", &bundle_strength_factor, 0.01f, 0.0f, 2.0f))
    changed = true;

  bool pull_cubical_bool = (pull_cubical != 0);
  if (ImGui::Checkbox("Test on Pull Operators for cubical rotting?", &pull_cubical_bool)) {
    pull_cubical = pull_cubical_bool ? 1u : 0u;
    changed = true;
  }

  if (ImGui::TreeNode("Fungus propagation")) {
    if (ImGui::InputFloat("Time Step", &dt, 0.0f, 0.0f, "%.5f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Growth rate (white rot)", &aw, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Growth rate (brown rot)", &ab, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Chemical defense (white rot)", &bw, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Chemical defense (brown rot)", &bb, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Carbon tissue damage (white rot)", &ycw, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Carbon tissue damage (brown rot)", &ycb, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Lignin tissue damage (white rot)", &ylw, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Carbon regeneration", &pc, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Lignin regeneration", &pl, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Defense rate", &k, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Defence decay rate", &delta, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Lignin weight for white rot", &ll, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Carbon weight for white rot", &lc, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Boundary reaction for rot growth", &bo, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Boundary reaction for propogation", &be, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Adjustment from carbon to white rot growth", &kc, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Lignin threshold", &HL_threshold, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat("Carbon threshold", &HC_threshold, 0.0f, 0.0f, "%.2f")) {
      changed = true;
    }
    if (ImGui::TreeNode("Moisture Settings")) {
      if (ImGui::InputFloat("Base growth rate for white rot", &brw, 0.0f, 0.0f, "%.2f")) {
        changed = true;
      }
      if (ImGui::InputFloat("Base growth rate for brown rot", &brb, 0.0f, 0.0f, "%.2f")) {
        changed = true;
      }
      if (ImGui::InputFloat("Moisture spread rate", &msr, 0.0f, 0.0f, "%.2f")) {
        changed = true;
      }
      ImGui::TreePop();
    }

    bool global_bool = (global_parameter != 0);
    if (ImGui::Checkbox("Global parameter", &global_bool)) {
      global_parameter = global_bool ? 1u : 0u;
      changed = true;
    }

    // matrixAw: input by columns (glm is column-major)
    ImGui::Text("Diffusion obstruction matrix for white rot");
    float* pAw = glm::value_ptr(matrixAw);
    if (ImGui::InputFloat3("matrixAw col0", pAw + 0, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAw col1", pAw + 3, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAw col2", pAw + 6, "%.2f")) {
      changed = true;
    }

    // matrixAb: same pattern as matrixAw
    ImGui::Text("Diffusion obstruction matrix for brown rot");
    float* pAb = glm::value_ptr(matrixAb);
    if (ImGui::InputFloat3("matrixAb col0", pAb + 0, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAb col1", pAb + 3, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAb col2", pAb + 6, "%.2f")) {
      changed = true;
    }

    // matrixAc: same pattern as matrixAw
    ImGui::Text("Diffusion obstruction matrix for carbon");
    float* pAc = glm::value_ptr(matrixAc);
    if (ImGui::InputFloat3("matrixAc col0", pAc + 0, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAc col1", pAc + 3, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAc col2", pAc + 6, "%.2f")) {
      changed = true;
    }

    ImGui::Text("Diffusion obstruction matrix for carbon");
    float* pAm = glm::value_ptr(matrixAm);
    if (ImGui::InputFloat3("matrixAm col0", pAm + 0, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAm col1", pAm + 3, "%.2f")) {
      changed = true;
    }
    if (ImGui::InputFloat3("matrixAm col2", pAm + 6, "%.2f")) {
      changed = true;
    }

    ImGui::RadioButton("Tree space", &treespace, 1);
    ImGui::RadioButton("Global space", &treespace, 0);
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Crack Parameters:")) {
    if (ImGui::DragFloat("Crack Boundary Shrinkage Offset", &crack_bd_shrinkage_offset, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Crack R Scale", &crack_R_scale, 0.01f, 0.0f, 2.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Crack T Scale", &crack_T_scale, 0.01f, 0.0f, 2.0f)) {
      changed = true;
    }
    bool internal_pattern_bool = (internal_pattern != 0);
    if (ImGui::Checkbox("Simulate internal cracking?", &internal_pattern_bool)) {
      internal_pattern = internal_pattern_bool ? 1u : 0u;
      changed = true;
    }

    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Leaf Parameters:")) {
    if (ImGui::DragFloat("Leaf Break Threshold", &leaf_break_threshold, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    bool break_from_moisture = (leaf_break_from_moisture != 0);
    if (ImGui::Checkbox("Leaf Break from Moisture?", &break_from_moisture)) {
      leaf_break_from_moisture = break_from_moisture ? 1u : 0u;
      changed = true;
    }
    ImGui::TreePop();
  }

  if (ImGui::Checkbox("Structural damage", &enable_structural_damage)) {
    changed = true;
  }

  if (enable_structural_damage) {
    if (ImGui::Checkbox("Segment breaking", &enable_segment_breaking)) {
      changed = true;
    }
    if (ImGui::Checkbox("Segment disconnection", &enable_segment_disconnection)) {
      changed = true;
    }
    if (enable_segment_breaking) {
      if (ImGui::TreeNode("Segment breaking")) {
        if (ImGui::Checkbox("Segment positional breaking", &enable_positional_breaking)) {
          changed = true;
        }
        if (ImGui::Checkbox("Segment rotational breaking", &enable_rotational_breaking)) {
          changed = true;
        }
        ImGui::TreePop();
      }
    }
    if (enable_segment_disconnection) {
      if (ImGui::TreeNode("Segment disconnection")) {
        if (ImGui::Checkbox("Segment tensile disconnection", &enable_segment_tensile_disconnection)) {
          changed = true;
        }
        if (ImGui::Checkbox("Segment compression disconnection", &enable_segment_compression_disconnection)) {
          changed = true;
        }
        if (enable_segment_compression_disconnection) {
          ImGui::DragFloat("Compression strength factor", &compression_strength_factor, 0.1f, 0.1f, 1000.f);
        }
        ImGui::TreePop();
      }
    }
    if (ImGui::Checkbox("Foliage detachment", &enable_foliage_detachment)) {
      changed = true;
    }
  }

  if (ImGui::Checkbox("Dynamic Grouping", &dynamic_grouping)) {
    changed = true;
  }
  if (!dynamic_grouping) {
    if (ImGui::DragInt("Grouping iteration", &grouping_iteration, 1, 1, 500)) {
      grouping_iteration = glm::clamp(grouping_iteration, 1, 500);
      changed = true;
    }
  }
  if (ImGui::Checkbox("Segment collision", &enable_segment_collision)) {
    changed = true;
  }
  if (ImGui::DragInt("Position constraint iteration", &position_constraint_iteration, 1, 1, 50))
    changed = true;
  if (ImGui::DragInt("Velocity constraint iteration", &velocity_constraint_iteration, 1, 1, 50))
    changed = true;
  if (ImGui::DragFloat("Segment Velocity damping", &segment_velocity_damping, 0.01f, 0.f, 5.f, "%.2f"))
    changed = true;
  if (ImGui::DragFloat("Segment Angular velocity damping", &segment_angular_velocity_damping, 0.01f, 0.f, 5.f, "%.2f"))
    changed = true;
  if (ImGui::DragFloat("Leaf Velocity damping", &leaf_velocity_damping, 0.0001f, 0.f, 1.f, "%.4f"))
    changed = true;
  if (ImGui::DragFloat("Leaf Angular velocity damping", &leaf_angular_velocity_damping, 0.00001f, 0.f, 1.f, "%.5f"))
    changed = true;

  if (ImGui::DragFloat3("Gravity", &gravity.x, 1.f))
    changed = true;

  if (ImGui::DragFloat("Pivot ring radius", &pivot_ring_radius, 0.02f, 0.f, 10.f, "%.2f"))
    changed = true;

  return changed;
}

void DynamicStrands::UpdateBindings() const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto& descriptor_set = strands_descriptor_sets[current_frame_index];

  descriptor_set->UpdateBufferDescriptorBinding(0, device_strands_buffer, 0);
  descriptor_set->UpdateBufferDescriptorBinding(1, device_nodes_buffer, 0);
  descriptor_set->UpdateBufferDescriptorBinding(2, device_segments_buffer, 0);
  descriptor_set->UpdateBufferDescriptorBinding(3, device_segment_pairs_buffer, 0);
  descriptor_set->UpdateBufferDescriptorBinding(4, device_segment_data_list_buffer, 0);
  descriptor_set->UpdateBufferDescriptorBinding(5, device_hashed_grid_elements_buffer, 0);
  descriptor_set->UpdateBufferDescriptorBinding(6, device_hashed_grid_cell_starts_buffer, 0);
  descriptor_set->UpdateBufferDescriptorBinding(7, device_foliage_buffer, 0);
  descriptor_set->UpdateBufferDescriptorBinding(10, device_segment_particle0_buffer, 0);
  descriptor_set->UpdateBufferDescriptorBinding(11, device_segment_particle1_buffer, 0);
  descriptor_set->UpdateBufferDescriptorBinding(12, device_segment_connection_handles_buffer, 0);

  meshing->UpdateBindings();
  for (const auto& c : constraints) {
    c->UpdateBindings();
  }
}

glm::vec3 DynamicStrands::GpuSegment::GetCenterX0() const {
  return (particle0.x0 + particle1.x0) * .5f;
}

void DynamicStrands::Upload() {
  device_strands_buffer->UploadVector(strands);
  device_strands_buffer->SetDebugName("Strands Buffer");
  device_nodes_buffer->UploadVector(nodes);
  device_nodes_buffer->SetDebugName("Nodes Buffer");
  const auto packed_segments = PackSegments(segments);
  device_segments_buffer->UploadData(packed_segments.segments.size(), packed_segments.segments.data());
  device_segments_buffer->SetDebugName("Segments Buffer");
  device_segment_particle0_buffer->UploadVector(packed_segments.particle0s);
  device_segment_particle0_buffer->SetDebugName("Segment Particle 0 Buffer");
  device_segment_particle1_buffer->UploadVector(packed_segments.particle1s);
  device_segment_particle1_buffer->SetDebugName("Segment Particle 1 Buffer");
  device_segment_pairs_buffer->UploadVector(segment_pairs);
  device_segment_pairs_buffer->SetDebugName("Segment Pairs Buffer");
  const auto packed_segment_data = PackSegmentData(segment_data_list);
  device_segment_data_list_buffer->UploadData(packed_segment_data.data.size(), packed_segment_data.data.data());
  device_segment_data_list_buffer->SetDebugName("Segment Data List Buffer");
  device_segment_connection_handles_buffer->UploadVector(packed_segment_data.connection_handles);
  device_segment_connection_handles_buffer->SetDebugName("Segment Connection Handles Buffer");

  meshing->Upload();

  device_hashed_grid_elements_buffer->UploadVector(hashed_grid_elements);
  device_hashed_grid_elements_buffer->SetDebugName("Hashed Grid Elements Buffer");
  device_hashed_grid_cell_starts_buffer->UploadVector(hashed_grid_cell_starts);
  device_hashed_grid_cell_starts_buffer->SetDebugName("Hashed Grid Cell Starts Buffer");
  device_foliage_buffer->UploadVector(foliage);
  device_foliage_buffer->SetDebugName("Foliage Buffer");
  for (const auto& c : constraints) {
    c->UploadData();
  }
  UpdateBindings();
  frame_index = 0;
}

void DynamicStrands::Download() {
  if (!strands.empty())
    device_strands_buffer->DownloadVector(strands, strands.size());
  if (!nodes.empty())
    device_nodes_buffer->DownloadVector(nodes, nodes.size());
  if (!segments.empty()) {
    auto packed_segments = MakePackedSegments(segments.size());
    device_segments_buffer->DownloadData(packed_segments.segments.size(), packed_segments.segments.data());
    device_segment_particle0_buffer->DownloadVector(packed_segments.particle0s, packed_segments.particle0s.size());
    device_segment_particle1_buffer->DownloadVector(packed_segments.particle1s, packed_segments.particle1s.size());
    UnpackSegments(segments, packed_segments);
  }
  if (!segment_pairs.empty())
    device_segment_pairs_buffer->DownloadVector(segment_pairs, segment_pairs.size());
  if (!segment_data_list.empty()) {
    auto packed_segment_data = MakePackedSegmentData(segment_data_list.size());
    device_segment_data_list_buffer->DownloadData(packed_segment_data.data.size(), packed_segment_data.data.data());
    device_segment_connection_handles_buffer->DownloadVector(packed_segment_data.connection_handles,
                                                             packed_segment_data.connection_handles.size());
    UnpackSegmentData(segment_data_list, packed_segment_data);
  }

  meshing->Download();

  if (!hashed_grid_elements.empty())
    device_hashed_grid_elements_buffer->DownloadVector(hashed_grid_elements, hashed_grid_elements.size());
  if (!hashed_grid_cell_starts.empty())
    device_hashed_grid_cell_starts_buffer->DownloadVector(hashed_grid_cell_starts, hashed_grid_cell_starts.size());
  if (!foliage.empty())
    device_foliage_buffer->DownloadVector(foliage, foliage.size());
  for (const auto& c : constraints) {
    c->DownloadData();
  }
}

void DynamicStrands::CalculateGroups(const PhysicsParameters& physics_parameters) const {
  if (segments.empty())
    return;

  // Below will be executed at the start of next frame.
  struct GroupingPushConstant {
    uint32_t segment_size;
  };

  static std::shared_ptr<ComputePipeline> reset_pipeline, step_pipeline, dynamic_step_pipeline, apply_pipeline{};
  static std::shared_ptr<Buffer> feedback_buffer;
  static std::shared_ptr<Buffer> new_group_index_buffer;

  static std::shared_ptr<DescriptorSetLayout> grouping_layout{};
  static std::shared_ptr<DescriptorSetLayout> dynamic_grouping_layout{};
  static std::shared_ptr<DescriptorSet> grouping_descriptor_set{};
  static std::shared_ptr<DescriptorSet> dynamic_grouping_descriptor_set{};

  if (!reset_pipeline) {
    std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Grouping/Reset.slang");
    reset_pipeline = std::make_shared<ComputePipeline>();
    reset_pipeline->compute_shader = shader;
    reset_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    auto& push_constant_range = reset_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroupingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    reset_pipeline->Initialize();
  }

  if (!grouping_layout) {
    grouping_layout = std::make_shared<DescriptorSetLayout>();
    grouping_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    grouping_layout->Initialize();
  }

  if (!dynamic_grouping_layout) {
    dynamic_grouping_layout = std::make_shared<DescriptorSetLayout>();
    dynamic_grouping_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                   0);
    dynamic_grouping_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                   0);
    dynamic_grouping_layout->Initialize();
  }

  if (!step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Grouping/Step.slang");
    step_pipeline = std::make_shared<ComputePipeline>();
    step_pipeline->compute_shader = shader;
    step_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    step_pipeline->descriptor_set_layouts.emplace_back(grouping_layout);
    auto& push_constant_range = step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroupingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    step_pipeline->Initialize();
  }
  if (!dynamic_step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Grouping/DynamicStep.slang");
    dynamic_step_pipeline = std::make_shared<ComputePipeline>();
    dynamic_step_pipeline->compute_shader = shader;
    dynamic_step_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    dynamic_step_pipeline->descriptor_set_layouts.emplace_back(dynamic_grouping_layout);
    auto& push_constant_range = dynamic_step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroupingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    dynamic_step_pipeline->Initialize();
  }
  if (!apply_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Grouping/Apply.slang");
    apply_pipeline = std::make_shared<ComputePipeline>();
    apply_pipeline->compute_shader = shader;
    apply_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    apply_pipeline->descriptor_set_layouts.emplace_back(grouping_layout);
    auto& push_constant_range = apply_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroupingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    apply_pipeline->Initialize();
  }
  if (!new_group_index_buffer) {
    VkBufferCreateInfo buffer_create_info{};
    buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_create_info.usage =
        VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    buffer_create_info.size = 1;
    VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
    buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    feedback_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
    new_group_index_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  }
  if (!dynamic_grouping_descriptor_set) {
    dynamic_grouping_descriptor_set = std::make_shared<DescriptorSet>(dynamic_grouping_layout);
  }
  if (!grouping_descriptor_set) {
    grouping_descriptor_set = std::make_shared<DescriptorSet>(grouping_layout);
  }
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;

  GroupingPushConstant push_constant;
  push_constant.segment_size = segments.size();
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto group_size = Platform::DivUp(segments.size(), work_group_invocations);
  std::vector<uint32_t> feedback(group_size);

  new_group_index_buffer->Resize(sizeof(int) * segments.size());
  grouping_descriptor_set->UpdateBufferDescriptorBinding(0, new_group_index_buffer);

  if (physics_parameters.dynamic_grouping) {
    const auto start_time = ApplicationContext::Get().GetTimes().Now();
    feedback_buffer->Resize(sizeof(uint32_t) * group_size);
    dynamic_grouping_descriptor_set->UpdateBufferDescriptorBinding(0, new_group_index_buffer);
    dynamic_grouping_descriptor_set->UpdateBufferDescriptorBinding(1, feedback_buffer);
    feedback_buffer->SetDebugName("Feedback Buffer");

    new_group_index_buffer->SetDebugName("New Group Index Buffer");

    Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
      reset_pipeline->Bind(vk_command_buffer);
      reset_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                        strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      reset_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      reset_pipeline->Dispatch(vk_command_buffer, group_size, 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
    bool updated = true;
    const auto step = [&]() {
      Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
        feedback_buffer->Fill(vk_command_buffer, 0, VK_WHOLE_SIZE, 0);
        Platform::EverythingBarrier(vk_command_buffer);
        dynamic_step_pipeline->Bind(vk_command_buffer);
        dynamic_step_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                 strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        dynamic_step_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                 dynamic_grouping_descriptor_set->GetVkDescriptorSet());
        dynamic_step_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        dynamic_step_pipeline->Dispatch(vk_command_buffer, group_size, 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);

        apply_pipeline->Bind(vk_command_buffer);
        apply_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                          strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        apply_pipeline->BindDescriptorSet(vk_command_buffer, 1, grouping_descriptor_set->GetVkDescriptorSet());
        apply_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        apply_pipeline->Dispatch(vk_command_buffer, group_size, 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      });
      feedback_buffer->DownloadVector(feedback, feedback.size());
    };
    static int max_iterations = 0;
    int iterations = 0;
    while (updated) {
      updated = false;
      step();
      for (const auto& i : feedback) {
        if (i != 0) {
          updated = true;
          break;
        }
      }
      iterations++;
    }
    max_iterations = glm::max(iterations, max_iterations);
    // EVOENGINE_LOG("Iterations: " + std::to_string(iterations), + ", max: " + std::to_string(max_iterations));
    const auto method3_time = std::to_string(ApplicationContext::Get().GetTimes().Now() - start_time);
  } else {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      reset_pipeline->Bind(vk_command_buffer);
      reset_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                        strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      reset_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      reset_pipeline->Dispatch(vk_command_buffer, group_size, 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
      for (int iteration = 0; iteration < physics_parameters.grouping_iteration; iteration++) {
        step_pipeline->Bind(vk_command_buffer);
        step_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        step_pipeline->BindDescriptorSet(vk_command_buffer, 1, grouping_descriptor_set->GetVkDescriptorSet());
        step_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        step_pipeline->Dispatch(vk_command_buffer, group_size, 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);

        apply_pipeline->Bind(vk_command_buffer);
        apply_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                          strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        apply_pipeline->BindDescriptorSet(vk_command_buffer, 1, grouping_descriptor_set->GetVkDescriptorSet());
        apply_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        apply_pipeline->Dispatch(vk_command_buffer, group_size, 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      }
    });
  }
}

void DynamicStrands::Clear() {
  strands.clear();
  nodes.clear();
  segments.clear();
  segment_pairs.clear();
  segment_data_list.clear();

  meshing->Clear();

  hashed_grid_elements.clear();
  hashed_grid_cell_starts.clear();
  constraints.clear();
}

glm::vec3 DynamicStrands::ComputeInertiaTensorBox(const float mass, const float width, const float height,
                                                  const float depth) {
  return {
      mass / 12.f * (height * height + depth * depth),
      mass / 12.f * (width * width + depth * depth),
      mass / 12.f * (width * width + height * height),
  };
}

glm::vec3 DynamicStrands::ComputeInertiaTensorRod(const float mass, const float radius, const float length) {
  float factor = mass / 12.f * (3.f * radius * radius + length * length);
  return {factor, factor, mass / 2.f * radius * radius};
}
