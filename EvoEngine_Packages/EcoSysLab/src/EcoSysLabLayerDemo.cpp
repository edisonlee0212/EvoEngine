#include "EcoSysLabLayer.hpp"

#include "AssetManager.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BasicPruningDescriptor.hpp"
#include "BasicReproductionModuleDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "Blower.hpp"
#include "BranchRenderer.hpp"
#include "Climate.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabSystem.hpp"
#include "ForestDescriptor.hpp"
#include "GeometryStorage.hpp"
#include "GpuTreeMeshRenderer.hpp"
#include "GraphicsResources.hpp"
#include "Lights.hpp"
#include "MeshRenderer.hpp"
#include "Platform/Platform.hpp"
#include "ProceduralNoiseGenerators.hpp"
#include "ProceduralNoiseOperators.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "SoilDescriptor.hpp"
#include "TextureStorage.hpp"
#include "TreeDescriptor.hpp"

#include <algorithm>

using namespace eco_sys_lab_package;

namespace {
struct WindyTreeDemoSettings {
  enum class TreePreset { Acacia, Spruce };
  TreePreset tree_preset = TreePreset::Spruce;
  glm::ivec2 tree_grid{4, 4};
  float tree_spacing = 1.75f;
  uint32_t tree_seed = 42;
  float acacia_growth_years = 8.0f;
  float spruce_growth_years = 5.0f;
  float growth_step_days = 3.0f;
  float shoot_growth_rate = 0.2f;
  glm::vec2 leaf_size{0.03f, 0.06f};
  glm::vec4 bark_color{0.38f, 0.25f, 0.17f, 1.0f};
  glm::vec4 leaf_color{0.16f, 0.48f, 0.14f, 1.0f};
  glm::vec4 ground_color{0.32f, 0.37f, 0.25f, 1.0f};
  glm::vec3 ground_size{12.0f, 0.2f, 12.0f};
  glm::vec3 camera_position{0.0f, 5.0f, 14.0f};
  glm::vec3 camera_target{0.0f, 2.8f, 0.0f};
  glm::vec3 sun_direction{0.35f, -0.8f, -0.48f};
  float sun_brightness = 2.0f;
  glm::vec3 blower_position{-5.0f, 2.5f, 0.0f};
  glm::vec3 blower_size{8.0f, 6.0f, 1.0f};
  float wind_speed = 5.0f;
  float wind_particle_rate = 1500.0f;
  float wind_strength = 1.5f;
  float wind_particle_radius = 0.25f;
  float air_cell_size = 0.1f;
  bool show_wind_particles = false;
};

const WindyTreeDemoSettings windy_tree_settings{};

void SetNoiseAngle(procedural_noise::ProceduralNoise4D& noise, const float base, const float variance,
                   const float frequency) {
  auto& graph = noise.node_graph;
  const auto base_handle = graph.AllocateNode(0, 1);
  const auto variance_handle = graph.AllocateNode(0, 1);
  const auto perlin_handle = graph.AllocateNode(3, 1);
  const auto add_handle = graph.AllocateNode(2, 1);
  const auto multiply_handle = graph.AllocateNode(2, 1);
  const auto set_constant = [&](const auto handle, const float value) {
    auto& node = graph.RefNode(handle);
    node.data.type = procedural_noise::NodeType::Constant;
    auto constant = std::make_shared<procedural_noise::ConstantNode>();
    constant->value = value;
    node.data.node_impl = constant;
  };
  set_constant(base_handle, base);
  set_constant(variance_handle, variance);
  auto& perlin = graph.RefNode(perlin_handle);
  perlin.data.type = procedural_noise::NodeType::Perlin3D;
  auto perlin_node = std::make_shared<procedural_noise::Perlin3DNode>();
  perlin_node->frequency = frequency;
  perlin.data.node_impl = perlin_node;
  graph.RefNode(add_handle).data = {procedural_noise::NodeType::Add, std::make_shared<procedural_noise::AddNode>()};
  graph.RefNode(multiply_handle).data = {procedural_noise::NodeType::Multiply,
                                         std::make_shared<procedural_noise::MultiplyNode>()};
  const auto& input = graph.PeekNode(1);
  for (int axis = 0; axis < 3; ++axis)
    graph.AllocateLink(input.GetOutputPinHandles()[axis], perlin.GetInputPinHandles()[axis]);
  graph.AllocateLink(graph.PeekNode(base_handle).GetOutputPinHandles()[0],
                     graph.PeekNode(add_handle).GetInputPinHandles()[0]);
  graph.AllocateLink(perlin.GetOutputPinHandles()[0], graph.PeekNode(multiply_handle).GetInputPinHandles()[0]);
  graph.AllocateLink(graph.PeekNode(variance_handle).GetOutputPinHandles()[0],
                     graph.PeekNode(multiply_handle).GetInputPinHandles()[1]);
  graph.AllocateLink(graph.PeekNode(multiply_handle).GetOutputPinHandles()[0],
                     graph.PeekNode(add_handle).GetInputPinHandles()[1]);
  graph.AllocateLink(graph.PeekNode(add_handle).GetOutputPinHandles()[0], graph.PeekNode(0).GetInputPinHandles()[0]);
}

std::vector<Entity> BuildWindyTreeScene(const std::shared_ptr<Scene>& scene) {
  const auto& settings = windy_tree_settings;
  const bool spruce = settings.tree_preset == WindyTreeDemoSettings::TreePreset::Spruce;
  const auto root = scene->CreateEntity("Windy Tree Demo");
  auto system = scene->GetOrCreateSystem<EcoSysLabSystem>(0.0f);
  system->Enable();
  system->environment.climate_descriptor = AssetManager::CreateTemporaryAsset<ClimateDescriptor>();
  auto soil = AssetManager::CreateTemporaryAsset<SoilDescriptor>();
  soil->texture_resolution = {64, 64};
  soil->height_field = AssetManager::CreateTemporaryAsset<HeightField>();
  system->environment.soil_descriptor = soil;
  system->environment.air_cell_size = settings.air_cell_size;
  system->environment.initialized = false;
  system->growth.settings.delta_time = settings.growth_step_days;

  const auto material = [](const glm::vec4& color) {
    auto result = AssetManager::CreateTemporaryAsset<Material>();
    result->material_data.shade_material.pbr_base_color_factor = color;
    result->material_data.shade_material.pbr_roughness_factor = 1.0f;
    result->MarkDirty();
    return result;
  };
  const auto bark_material = material(settings.bark_color);
  const auto leaf_material = material(spruce ? glm::vec4{0.12f, 0.28f, 0.14f, 1.0f} : settings.leaf_color);
  const auto ground_material = material(settings.ground_color);

  const auto descriptor = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
  const auto shoot = AssetManager::CreateTemporaryAsset<BasicShootDescriptor>();
  shoot->growth_rate = settings.shoot_growth_rate;
  shoot->straight_trunk = 0.0f;
  shoot->phototropism = spruce ? 0.02f : 0.05f;
  shoot->horizontal_tropism = spruce ? 0.01f : 0.1f;
  if (spruce)
    shoot->gravity_bending_strength = 5.0f;
  shoot->internode_length = spruce ? 0.03f : 0.045f;
  shoot->internode_length_thickness_factor = spruce ? 0.2f : 0.25f;
  shoot->end_node_thickness = 0.002f;
  shoot->internode_shadow_factor = spruce ? 0.02f : 0.035f;
  shoot->lateral_bud_count = 2;
  shoot->apical_bud_extinction_rate = spruce ? 0.0f : 0.54f;
  shoot->lateral_bud_flushing_rate = 0.3f;
  shoot->apical_control = spruce ? 1.91f : 0.9f;
  shoot->apical_dominance = spruce ? 0.0f : 0.1f;
  shoot->apical_dominance_loss = spruce ? 0.0f : 0.2f;
  SetNoiseAngle(shoot->branching_angle_graph, spruce ? 60.0f : 45.0f, 10.0f, spruce ? 100.0f : 1.0f);
  SetNoiseAngle(shoot->roll_angle_graph, 90.0f, 360.0f, 100.0f);
  SetNoiseAngle(shoot->apical_angle_graph, spruce ? 0.0f : 20.0f, spruce ? 0.0f : 40.0f, 100.0f);
  descriptor->shoot_descriptor = shoot;
  const auto pruning = AssetManager::CreateTemporaryAsset<BasicPruningDescriptor>();
  pruning->light_pruning_factor = spruce ? 0.2f : 0.41f;
  descriptor->pruning_descriptor = pruning;
  descriptor->reproduction_module_descriptor = AssetManager::CreateTemporaryAsset<BasicReproductionModuleDescriptor>();
  const auto bark = AssetManager::CreateTemporaryAsset<BasicBarkDescriptor>();
  bark->bark_material_ref = bark_material;
  descriptor->bark_descriptor = bark;
  const auto foliage = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
  foliage->leaf_size = spruce ? glm::vec2{0.015f, 0.06f} : settings.leaf_size;
  foliage->rotation_variance = spruce ? 0.1f : 999.0f;
  foliage->gravitropism = 0.0f;
  foliage->leaf_material_ref = leaf_material;
  descriptor->foliage_descriptor = foliage;

  const auto ground = scene->CreateEntity("Ground");
  scene->SetParent(ground, root);
  GlobalTransform ground_transform;
  ground_transform.SetPosition({0.0f, -settings.ground_size.y * 0.5f, 0.0f});
  ground_transform.SetScale(settings.ground_size);
  scene->SetDataComponent(ground, ground_transform);
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(ground).lock();
  renderer->mesh = Resources::GetInstance().GetPrimitives().cube;
  renderer->material = ground_material;

  const auto sunlight = scene->CreateEntity("Sunlight");
  scene->SetParent(sunlight, root);
  auto light_transform = GlobalTransform{};
  light_transform.SetRotation(glm::quatLookAt(-glm::normalize(settings.sun_direction), glm::vec3(0, 1, 0)));
  scene->SetDataComponent(sunlight, light_transform);
  const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(sunlight).lock();
  light->diffuse_brightness = settings.sun_brightness;

  if (const auto camera = scene->main_camera.Get<Camera>()) {
    camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
    GlobalTransform camera_transform;
    camera_transform.SetPosition(settings.camera_position);
    camera_transform.SetRotation(
        glm::quatLookAt(glm::normalize(settings.camera_target - settings.camera_position), glm::vec3(0, 1, 0)));
    scene->SetDataComponent(camera->GetOwner(), camera_transform);
  }

  std::vector<Entity> trees;
  for (int x = 0; x < settings.tree_grid.x; ++x) {
    for (int z = 0; z < settings.tree_grid.y; ++z) {
      const auto entity = scene->CreateEntity("Tree " + std::to_string(trees.size() + 1));
      scene->SetParent(entity, root);
      GlobalTransform transform;
      transform.SetPosition({(x - (settings.tree_grid.x - 1) * 0.5f) * settings.tree_spacing, 0.0f,
                             (z - (settings.tree_grid.y - 1) * 0.5f) * settings.tree_spacing});
      scene->SetDataComponent(entity, transform);
      scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(entity);
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
      tree->tree_descriptor_ref = descriptor;
      tree->shoot_model.seed = settings.tree_seed + static_cast<uint32_t>(trees.size());
      tree->generate_mesh = false;
      scene->GetOrSetPrivateComponent<BranchRenderer>(entity).lock()->material_ref = bark_material;
      trees.push_back(entity);
    }
  }

  const auto blower_entity = scene->CreateEntity("Blower");
  scene->SetParent(blower_entity, root);
  GlobalTransform blower_transform;
  blower_transform.SetValue(settings.blower_position, glm::rotation(glm::vec3(0, 0, 1), glm::vec3(1, 0, 0)),
                            settings.blower_size);
  scene->SetDataComponent(blower_entity, blower_transform);
  const auto blower = scene->GetOrSetPrivateComponent<Blower>(blower_entity).lock();
  blower->speed = settings.wind_speed;
  blower->particle_rate = settings.wind_particle_rate;
  blower->wind_strength = settings.wind_strength;
  blower->particle_radius = settings.wind_particle_radius;
  blower->SetEnabled(false);
  EVOENGINE_LOG("Created script-owned Windy Tree demo scene with " + std::to_string(trees.size()) +
                " trees and a Blower.")
  return trees;
}
}  // namespace

namespace eco_sys_lab_package {
class EcoSysLabDemoBootstrap {
  std::vector<Entity> demo_tree_entities_;
  std::weak_ptr<Scene> demo_scene_;
  bool demo_tree_growth_finished_ = false;
  bool demo_tree_initialized_ = false;
  bool demo_growth_started_ = false;
  size_t demo_background_idle_frames_ = 0;
  bool windy_tree_ = false;

 public:
  void Update(EcoSysLabLayer& layer);
};
}  // namespace eco_sys_lab_package

void EcoSysLabLayer::UpdateDemoTreeGrowth() {
  if (!demo_bootstrap_)
    demo_bootstrap_ = std::make_shared<EcoSysLabDemoBootstrap>();
  demo_bootstrap_->Update(*this);
}

void EcoSysLabDemoBootstrap::Update(EcoSysLabLayer& layer) {
  const auto scene = layer.GetScene();
  if (demo_scene_.lock() != scene) {
    demo_scene_ = scene;
    demo_tree_entities_.clear();
    demo_tree_growth_finished_ = false;
    demo_tree_initialized_ = false;
    demo_growth_started_ = false;
    demo_background_idle_frames_ = 0;
    windy_tree_ = false;
  }
  if (layer.GetApplication().GetApplicationStatus() != Application::ExecutionStatus::Playing &&
      layer.GetApplication().GetApplicationStatus() != Application::ExecutionStatus::Step)
    return;
  if (demo_tree_growth_finished_)
    return;
  if (!ProjectManager::IsProjectIdle()) {
    demo_background_idle_frames_ = 0;
    return;
  }
  const auto project_path = ProjectManager::GetProjectPath();
  windy_tree_ = project_path.filename() == "WindyTree.eveproj" && project_path.parent_path().filename() == "WindyTree";
  if (!windy_tree_ &&
      (project_path.filename() != "test.eveproj" || project_path.parent_path().filename() != "EcoSysLabProject")) {
    demo_tree_growth_finished_ = true;
    return;
  }
  if (!scene) {
    return;
  }
  if (!demo_tree_initialized_) {
    if (!windy_tree_ && !layer.demo_growth_enabled_)
      return;
    if (Platform::MeshShaderEnabled()) {
      if (!layer.gpu_tree_mesh_renderer_)
        layer.gpu_tree_mesh_renderer_ = std::make_shared<GpuTreeMeshRenderer>();
      if (!layer.gpu_tree_mesh_renderer_->Prepare())
        return;
    }
    if (windy_tree_) {
      demo_tree_entities_ = BuildWindyTreeScene(scene);
      layer.visualize_wind_particles = windy_tree_settings.show_wind_particles;
      demo_tree_initialized_ = true;
      return;
    }
    const auto patch =
        std::dynamic_pointer_cast<ForestPatch>(ProjectManager::GetOrCreateAsset("AppleForest.forestpatch"));
    if (!patch || !patch->tree_descriptor.Get<TreeDescriptor>()) {
      EVOENGINE_ERROR("Failed to load the EcoSysLab demo AppleForest patch.")
      demo_tree_growth_finished_ = true;
      return;
    }
    const size_t previous_tree_count = scene->UnsafeGetPrivateComponentOwnersList<Tree>()
                                           ? scene->UnsafeGetPrivateComponentOwnersList<Tree>()->size()
                                           : 0;
    const auto forest = patch->InstantiatePatch(patch->grid_size, false);
    const auto* trees = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    if (!scene->IsEntityValid(forest) || !trees || trees->size() != previous_tree_count + 4) {
      EVOENGINE_ERROR("Failed to instantiate four EcoSysLab demo AppleForest trees.")
      demo_tree_growth_finished_ = true;
      return;
    }
    if (const auto* blowers = scene->UnsafeGetPrivateComponentOwnersList<Blower>(); !blowers || blowers->empty()) {
      const auto blower = scene->CreateEntity("Blower");
      GlobalTransform transform;
      transform.SetValue({-3.0f, 2.0f, 0.0f}, glm::rotation(glm::vec3(0, 0, 1), glm::vec3(1, 0, 0)),
                         {4.0f, 4.0f, 1.0f});
      scene->SetDataComponent(blower, transform);
      scene->GetOrSetPrivateComponent<Blower>(blower);
      EVOENGINE_LOG("Created EcoSysLab demo Blower.")
    }
    demo_tree_entities_.assign(trees->begin() + previous_tree_count, trees->end());
    demo_tree_initialized_ = true;
    EVOENGINE_LOG("Instantiated four EcoSysLab demo AppleForest trees.")
  }
  if (!demo_growth_started_) {
    if (!windy_tree_ && !layer.demo_growth_enabled_)
      return;
    const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
    const auto shader_progress =
        render_layer ? render_layer->GetShaderPreparationProgress() : ShaderPreparationProgress{};
    if (AssetManager::GetAssetLoadSnapshot().Active() || TextureStorage::HasPendingUploads() ||
        GeometryStorage::HasPendingUploads() || BottomLevelAccelerationStructure::HasPendingStaticBuilds() ||
        (render_layer && (!render_layer->IsSceneShaderPrewarmComplete(scene) ||
                          shader_progress.completed != shader_progress.total || !shader_progress.error.empty()))) {
      demo_background_idle_frames_ = 0;
      return;
    }
    if (++demo_background_idle_frames_ < 4)
      return;
    demo_growth_started_ = true;
    EVOENGINE_LOG((windy_tree_ ? "Started the script-owned Windy Tree growth animation."
                               : "Started the EcoSysLab demo AppleForest four-year growth animation."))
  }

  const float growth_years = windy_tree_ ? (windy_tree_settings.tree_preset == WindyTreeDemoSettings::TreePreset::Spruce
                                                ? windy_tree_settings.spruce_growth_years
                                                : windy_tree_settings.acacia_growth_years)
                                         : 4.0f;
  const float target_growth_time = growth_years * 365.0f;
  if (layer.GetSimulatedTime() < target_growth_time) {
    const auto system = scene->GetSystem<EcoSysLabSystem>();
    auto growth_settings = system->growth.settings;
    growth_settings.delta_time = std::min(growth_settings.delta_time, target_growth_time - layer.GetSimulatedTime());
    layer.Simulate(growth_settings, system->growth.stats);
  }
  if (layer.GetSimulatedTime() < target_growth_time) {
    return;
  }
  if (!Platform::MeshShaderEnabled()) {
    auto demo_mesh_generator_settings = layer.mesh_generator_settings;
    demo_mesh_generator_settings.foliage_instancing = false;
    if (windy_tree_)
      demo_mesh_generator_settings.enable_foliage = false;
    for (const auto entity : demo_tree_entities_)
      scene->GetOrSetPrivateComponent<Tree>(entity).lock()->GenerateGeometryEntities(demo_mesh_generator_settings);
  }
  demo_tree_growth_finished_ = true;
  if (windy_tree_) {
    if (const auto* blowers = scene->UnsafeGetPrivateComponentOwnersList<Blower>())
      for (const auto entity : *blowers)
        scene->GetOrSetPrivateComponent<Blower>(entity).lock()->SetEnabled(true);
    EVOENGINE_LOG("Enabled Windy Tree blower after growth.")
  }
  EVOENGINE_LOG((windy_tree_ ? "Finished the script-owned Windy Tree growth animation."
                             : "Finished the EcoSysLab demo AppleForest four-year growth animation."))
}
