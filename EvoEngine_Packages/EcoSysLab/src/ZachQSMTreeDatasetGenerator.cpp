#include <fstream>
#include <iomanip>
#include <iostream>
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "PackageManager.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;

namespace {
std::ofstream Output(const std::filesystem::path& path) {
  std::ofstream stream(path);
  stream.exceptions(std::ios::badbit | std::ios::failbit);
  stream << std::setprecision(9);
  return stream;
}

void Vector(std::ostream& stream, const glm::vec3& v) {
  stream << v.x << ',' << v.y << ',' << v.z;
}

void ExportTree(const Tree& tree, const BasicFoliageDescriptor& foliage, const std::filesystem::path& folder,
                bool leaves, int seed) {
  const auto& skeleton = tree.shoot_model.PeekShootSkeleton();
  if (skeleton.PeekSortedNodeList().size() < 2)
    throw std::runtime_error("Tree did not grow.");
  auto nodes = Output(folder / "skeleton.csv");
  nodes << "id,parent_id,flow_id,order,start_x,start_y,start_z,end_x,end_y,end_z,diameter_m,length_m\n";
  for (auto handle : skeleton.PeekSortedNodeList()) {
    const auto& node = skeleton.PeekNode(handle);
    nodes << handle << ',' << node.GetParentHandle() << ',' << node.GetFlowHandle() << ',' << node.info.order << ',';
    Vector(nodes, node.info.global_position);
    nodes << ',';
    Vector(nodes, node.info.GetGlobalEndPosition());
    nodes << ',' << node.info.thickness << ',' << node.info.length << '\n';
  }
  auto node_graph = Output(folder / "nodes.yaml");
  YAML::Emitter node_yaml;
  node_yaml.SetFloatPrecision(9);
  node_yaml << YAML::BeginMap;
  tree.ExportNodeGraph(node_yaml);
  node_yaml << YAML::EndMap;
  node_graph << node_yaml.c_str();
  auto flow_graph = Output(folder / "flows.yaml");
  YAML::Emitter flow_yaml;
  flow_yaml.SetFloatPrecision(9);
  flow_yaml << YAML::BeginMap;
  tree.ExportFlowGraph(flow_yaml);
  flow_yaml << YAML::EndMap;
  flow_graph << flow_yaml.c_str();
  if (leaves) {
    std::srand(seed);
    auto mesh = Output(folder / "leaves.obj");
    auto labels = Output(folder / "leaf_instances.csv");
    labels << "leaf_id,node_id,first_vertex\n";
    size_t count = 0;
    const glm::vec4 corners[] = {{-0.5f, 0, -0.5f, 1}, {0.5f, 0, -0.5f, 1}, {0.5f, 0, 0.5f, 1}, {-0.5f, 0, 0.5f, 1}};
    for (auto handle : skeleton.PeekSortedNodeList()) {
      std::vector<glm::mat4> matrices;
      foliage.GenerateFoliageMatrices(matrices, skeleton.PeekNode(handle).info, 1.f);
      for (const auto& matrix : matrices) {
        for (const auto& corner : corners) {
          const glm::vec3 p = matrix * corner;
          mesh << "v " << p.x << ' ' << p.y << ' ' << p.z << '\n';
        }
        const auto first = count * 4 + 1;
        mesh << "f " << first << ' ' << first + 1 << ' ' << first + 2 << '\n';
        mesh << "f " << first << ' ' << first + 2 << ' ' << first + 3 << '\n';
        labels << count << ',' << handle << ',' << first << '\n';
        ++count;
      }
    }
  }
}
}  // namespace

// Called only after the runtime package loader has registered EcoSysLab types.
EVOENGINE_PACKAGE_EXPORT int EvoEngineGenerateZachQSMTreeDataset(const char* config_path) {
  try {
    const auto config = YAML::LoadFile(config_path);
    const auto folder = std::filesystem::absolute(config["output"].as<std::string>());
    const int count = config["count"].as<int>(12);
    const int first_seed = config["seed"].as<int>(1001);
    const float years = config["years"].as<float>(5.f);
    const float step_days = config["step_days"].as<float>(30.f);
    if (count < 1 || count > 100000 || first_seed < 0 || first_seed > INT_MAX - count || !std::isfinite(years) ||
        years <= 0 || !std::isfinite(step_days) || step_days <= 0 || step_days > 365)
      throw std::runtime_error("Invalid count, seed, years or step_days.");
    if (std::filesystem::exists(folder / "effective_settings.yaml"))
      throw std::runtime_error("Native output already exists; choose a new output folder.");
    std::filesystem::create_directories(folder);
    Jobs::Initialize(1);
    const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
    ApplicationContext::Get().Attach(scene);
    const auto layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
    if (!layer)
      throw std::runtime_error("EcoSysLab layer is missing.");
    const auto soil_descriptor = AssetManager::CreateTemporaryAsset<SoilDescriptor>();
    const auto soil = scene->GetOrSetPrivateComponent<Soil>(scene->CreateEntity("Soil")).lock();
    soil->soil_descriptor_ref = soil_descriptor;
    const auto climate_descriptor = AssetManager::CreateTemporaryAsset<ClimateDescriptor>();
    const auto climate = scene->GetOrSetPrivateComponent<Climate>(scene->CreateEntity("Climate")).lock();
    climate->climate_descriptor_ref = climate_descriptor;
    climate->InitializeClimateModel();
    const auto shoot = AssetManager::CreateTemporaryAsset<BasicShootDescriptor>();
    DeserializeBasicShootDescriptor(YAML::LoadFile(config["shoot"].as<std::string>()), *shoot);
    if (!std::isfinite(shoot->internode_length) || shoot->internode_length <= 0 || !std::isfinite(shoot->growth_rate) ||
        shoot->growth_rate <= 0 || !std::isfinite(shoot->end_node_thickness) || shoot->end_node_thickness <= 0)
      throw std::runtime_error("Shoot length, growth rate and tip diameter must be finite and positive.");
    const auto foliage = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
    DeserializeBasicFoliageDescriptor(YAML::LoadFile(config["foliage"].as<std::string>()), *foliage);
    const auto pruning = AssetManager::CreateTemporaryAsset<BasicPruningDescriptor>();
    DeserializeBasicPruningDescriptor(YAML::LoadFile(config["pruning"].as<std::string>()), *pruning);
    const auto reproduction = AssetManager::CreateTemporaryAsset<BasicReproductionModuleDescriptor>();
    const auto descriptor = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
    descriptor->shoot_descriptor = shoot;
    descriptor->foliage_descriptor = foliage;
    descriptor->pruning_descriptor = pruning;
    descriptor->reproduction_module_descriptor = reproduction;
    layer->simulation_settings = {};
    if (config["simulation"])
      layer->simulation_settings.Deserialize(config["simulation"]);
    layer->simulation_settings.delta_time = step_days;
    layer->simulation_settings.soil_simulation = false;
    layer->simulation_settings.auto_clear_fruit_and_leaves = true;
    layer->simulation_settings.auto_generate_skeletal_graph = false;
    auto settings_file = Output(folder / "effective_settings.yaml");
    YAML::Emitter settings;
    settings.SetFloatPrecision(9);
    settings << YAML::BeginMap << YAML::Key << "simulation" << YAML::Value << YAML::BeginMap;
    layer->simulation_settings.Serialize(settings);
    settings << YAML::EndMap << YAML::Key << "shoot" << YAML::Value << YAML::BeginMap;
    SerializeBasicShootDescriptor(settings, *shoot);
    settings << YAML::EndMap << YAML::Key << "climate" << YAML::Value << YAML::BeginMap;
    SerializeClimateDescriptor(settings, *climate_descriptor);
    settings << YAML::EndMap << YAML::EndMap;
    settings_file << settings.c_str();
    settings_file.close();
    for (int index = 0; index < count; ++index) {
      std::ostringstream name;
      name << "tree_" << std::setw(3) << std::setfill('0') << index + 1;
      const auto tree_folder = folder / name.str();
      if (std::filesystem::exists(tree_folder))
        throw std::runtime_error("Tree output already exists: " + tree_folder.string());
      std::filesystem::create_directories(tree_folder);
      layer->ResetAllTrees(nullptr);
      std::srand(first_seed + index);
      // MSVC's C random state is per thread; foliage growth runs on the worker.
      Jobs::RunParallelFor(1, [&](size_t) {
        std::srand(first_seed + index);
      });
      const auto entity = scene->CreateEntity(name.str());
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
      tree->tree_descriptor_ref = descriptor;
      tree->shoot_model.seed = first_seed + index;
      tree->enable_history = false;
      tree->generate_mesh = false;
      auto history = Output(tree_folder / "growth.csv");
      history << "age_days,nodes,flows,height_m\n";
      SimulationStats stats;
      float day = 0;
      int iteration = 0;
      while (day < years * 365.f) {
        layer->simulation_settings.delta_time = std::min(step_days, years * 365.f - day);
        layer->Simulate(layer->simulation_settings, stats);
        day += layer->simulation_settings.delta_time;
        const auto& skeleton = tree->shoot_model.PeekShootSkeleton();
        float height = 0;
        for (auto handle : skeleton.PeekSortedNodeList())
          height = std::max(height, skeleton.PeekNode(handle).info.GetGlobalEndPosition().y);
        history << day << ',' << skeleton.PeekSortedNodeList().size() << ',' << skeleton.PeekSortedFlowList().size()
                << ',' << height << '\n';
        if (++iteration % 60 == 0 || day >= years * 365.f)
          std::cout << name.str() << " seed=" << first_seed + index << " age=" << day / 365.f
                    << " years nodes=" << skeleton.PeekSortedNodeList().size() << " height=" << height << std::endl;
      }
      ExportTree(*tree, *foliage, tree_folder, config["leaves"].as<bool>(false), first_seed + index);
      scene->DeleteEntity(entity);
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "EcoSysLab dataset error: " << error.what() << std::endl;
    return 1;
  }
}
