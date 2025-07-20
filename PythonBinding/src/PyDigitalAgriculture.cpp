#include "PyDigitalAgriculture.hpp"
#include "Serialization.hpp"

#ifdef DATASET_GENERATION_PACKAGE
#  include "DatasetGenerationSerializationAdapters.hpp"
#endif

#include "BtfMeshRenderer.hpp"
#include "PARSensorGroup.hpp"
#include "SorghumCoordinates.hpp"
#include "TriangleIlluminationEstimator.hpp"

#ifdef DIGITAL_AGRICULTURE_PLUGIN

namespace evo_engine {
class TriangleIlluminationEstimator;
}

namespace py = pybind11;
using namespace py_digital_agriculture_package;
namespace {
template <typename T>
void RegisterSerializationHandler(const std::string& type_name) {
  Serialization::RegisterSerializationHandler<T>(
      [](YAML::Emitter& out, const T& target) {
        target.Serialize(out);
      },
      [](const YAML::Node& in, T& target) {
        target.Deserialize(in);
      },
      {}, type_name);
}
}  // namespace
void PyDigitalAgriculture::PushSorghumLayer() {
  ApplicationContext::Get().PushLayer<SorghumLayer>("Sorghum Layer");
}
void PyDigitalAgriculture::RegisterClasses() {
  auto& application = PyEvoEngine::GetRuntime().GetApplication();
  application.RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
  RegisterSerializationHandler<ObjectRotator>("ObjectRotator");
#  ifdef DATASET_GENERATION_PACKAGE
  application.RegisterPrivateComponent<SorghumPointCloudScanner>("SorghumPointCloudScanner");
  Serialization::RegisterSerializationHandler<SorghumPointCloudScanner>(
      SerializeSorghumPointCloudScanner, DeserializeSorghumPointCloudScanner, {}, "SorghumPointCloudScanner");
#  endif
}

Entity PyDigitalAgriculture::CreateEntityFromSorghumState(const Handle& sorghum_handle) {
  const auto sorghum_asset = PyEvoEngine::GetAsset(sorghum_handle);
  if (sorghum_asset->GetTypeName() != "SorghumState") {
    EVOENGINE_ERROR("CreateEntityFromSorghumState failed: invalid asset type!")
    return {};
  }
  return DatasetGenerator::CreateSorghumEntity(sorghum_asset);
}

Entity PyDigitalAgriculture::CreateEntityFromSorghumDescriptor(const Handle& sorghum_handle) {
  const auto sorghum_asset = PyEvoEngine::GetAsset(sorghum_handle);
  if (sorghum_asset->GetTypeName() != "SorghumDescriptor") {
    EVOENGINE_ERROR("CreateEntityFromSorghumDescriptor failed: invalid asset type!")
    return {};
  }
  return DatasetGenerator::CreateSorghumEntity(sorghum_asset);
}

Entity PyDigitalAgriculture::CreateEntityFromSorghumGenerator(const Handle& sorghum_generator_handle, const int seed) {
  const auto sorghum_asset = PyEvoEngine::GetAsset(sorghum_generator_handle);
  if (sorghum_asset->GetTypeName() != "SorghumGenerator") {
    EVOENGINE_ERROR("CreateEntityFromSorghumGenerator failed: invalid asset type!")
    return {};
  }
  return DatasetGenerator::CreateSorghumEntity(sorghum_asset, seed);
}
Entity PyDigitalAgriculture::CreateEntityFromSorghumField(const Handle& sorghum_generator_handle, int seed) {
  const auto sorghum_asset = PyEvoEngine::GetAsset(sorghum_generator_handle);
  if (sorghum_asset->GetTypeName() != "SorghumField") {
    EVOENGINE_ERROR("CreateEntityFromSorghumField failed: invalid asset type!")
    return {};
  }
  return DatasetGenerator::CreateSorghumEntity(sorghum_asset, seed);
}

void PyDigitalAgriculture::ApplySorghumGrid(const Handle& sorghum_field_handle, const Handle& sorghum_generator_handle,
                                            const SorghumGrid& sorghum_grid) {
  const auto sorghum_field = PyEvoEngine::GetAsset(sorghum_field_handle);
  const auto sorghum_generator = PyEvoEngine::GetAsset(sorghum_generator_handle);
  DatasetGenerator::ApplySorghumGrid(sorghum_field, sorghum_generator, sorghum_grid);
}


void PyDigitalAgriculture::Initialize(pybind11::module& m) {
  PyEvoEngine::Initialize(m);
  m.def("RegisterClasses", &RegisterClasses);
  m.def("PushSorghumLayer", &PushSorghumLayer);
  m.def("CreateEntityFromSorghumState", &CreateEntityFromSorghumState);
  m.def("CreateEntityFromSorghumDescriptor", &CreateEntityFromSorghumDescriptor);
  m.def("CreateEntityFromSorghumGenerator", &CreateEntityFromSorghumGenerator);
  m.def("CreateEntityFromSorghumField", &CreateEntityFromSorghumField);
  m.def("ApplySorghumGrid", &ApplySorghumGrid);
  m.def("EnableBTF", &EnableBTF);
  m.def("CheckBTFComponentsExist", &CheckBTFComponentsExist);
  m.def("SetCBTFGroup", &SetCBTFGroup);
  m.def("SetSkyDome", &SetSkyDome);
  m.def("PushRayTracerLayer", &PushRayTracerLayer);
  m.def("SetSunDirection", &SetSunDirection);
  m.def("SetPARSensors", &SetPARSensors);
  m.def("IlluminationEstimationOnSensors", &IlluminationEstimationOnSensors);
  m.def("GetAllIlluminationEstimationResultsFromSensors", &GetAllIlluminationEstimationResultsFromSensors);
  m.def("IlluminationEstimationOnSorghum", &IlluminationEstimationOnSorghum);
  m.def("CheckTriangleEstimator", &CheckTriangleEstimator);
  m.def("GetAllIlluminationEstimationResultsOnSorghum", &GetAllIlluminationEstimationResultsOnSorghum);
  m.def("InstantiateSorghumField", &InstantiateSorghumField, 
      py::arg("sorghum_field_handle"),
      py::arg("sorghum_coordinates"), 
      py::arg("seed"), 
      py::arg("index") = 200, 
      py::arg("radius") = 2.5f);

  py::class_<SorghumMeshGeneratorSettings>(m, "SorghumMeshGeneratorSettings")
      .def(py::init<>())
      .def_readwrite("enable_panicle", &SorghumMeshGeneratorSettings::enable_panicle)
      .def_readwrite("enable_stem", &SorghumMeshGeneratorSettings::enable_stem)
      .def_readwrite("enable_leaves", &SorghumMeshGeneratorSettings::enable_leaves)
      .def_readwrite("enable_leaf_sheath", &SorghumMeshGeneratorSettings::enable_leaf_sheath)
      .def_readwrite("single_leaf_index", &SorghumMeshGeneratorSettings::single_leaf_index)
      .def_readwrite("bottom_face", &SorghumMeshGeneratorSettings::bottom_face)
      .def_readwrite("leaf_separated", &SorghumMeshGeneratorSettings::leaf_separated)
      .def_readwrite("leaf_thickness", &SorghumMeshGeneratorSettings::leaf_thickness);

  py::class_<SorghumPointCloudPointSettings>(m, "SorghumPointCloudPointSettings")
      .def(py::init<>())
      .def_readwrite("variance", &SorghumPointCloudPointSettings::variance)
      .def_readwrite("ball_rand_radius", &SorghumPointCloudPointSettings::ball_rand_radius)
      .def_readwrite("type_index", &SorghumPointCloudPointSettings::type_index)
      .def_readwrite("instance_index", &SorghumPointCloudPointSettings::instance_index)
      .def_readwrite("leaf_index", &SorghumPointCloudPointSettings::leaf_index)
      .def_readwrite("bounding_box_limit", &SorghumPointCloudPointSettings::bounding_box_limit);

  py::class_<DatasetGenerator::SorghumDataGenerationParameters>(m, "SorghumDataGenerationParameters")
      .def(py::init<>())
      .def_readwrite("export_point_cloud", &DatasetGenerator::SorghumDataGenerationParameters::export_point_cloud)
      .def_readwrite("export_mesh", &DatasetGenerator::SorghumDataGenerationParameters::export_mesh)

      .def_readwrite("generate_ground_mesh", &DatasetGenerator::SorghumDataGenerationParameters::generate_ground_mesh)
      .def_readwrite("avoid_occlusion", &DatasetGenerator::SorghumDataGenerationParameters::avoid_occlusion)

      .def_readwrite("sorghum_point_cloud_point_settings",
                     &DatasetGenerator::SorghumDataGenerationParameters::sorghum_point_cloud_point_settings)
      .def_readwrite("sorghum_mesh_generator_settings",
                     &DatasetGenerator::SorghumDataGenerationParameters::sorghum_mesh_generator_settings)
      .def_readwrite("output_folder", &DatasetGenerator::SorghumDataGenerationParameters::output_folder)
      .def_readwrite("output_file_name", &DatasetGenerator::SorghumDataGenerationParameters::output_file_name);

  py::class_<SorghumGantryCaptureSettings>(m, "SorghumGantryCaptureSettings")
      .def(py::init<>())
      .def_readwrite("bounding_box_size", &SorghumGantryCaptureSettings::bounding_box_size)
      .def_readwrite("grid_size", &SorghumGantryCaptureSettings::grid_size)
      .def_readwrite("grid_distance", &SorghumGantryCaptureSettings::grid_distance)
      .def_readwrite("step", &SorghumGantryCaptureSettings::step)
      .def_readwrite("output_spline_info", &SorghumGantryCaptureSettings::output_spline_info)
      .def_readwrite("sample_height", &SorghumGantryCaptureSettings::sample_height);

  py::class_<SorghumGrid>(m, "SorghumGrid")
      .def(py::init<>())
      .def_readwrite("grid_distance", &SorghumGrid::grid_distance)
      .def_readwrite("position_offset_mean", &SorghumGrid::position_offset_mean)
      .def_readwrite("position_offset_variance", &SorghumGrid::position_offset_variance)
      .def_readwrite("rotation_variance_xz", &SorghumGrid::rotation_variance_xz)
      .def_readwrite("rotation_variance_y", &SorghumGrid::rotation_variance_y)
      .def_readwrite("grid_size", &SorghumGrid::grid_size);
}

void PyDigitalAgriculture::EnableBTF() {
  auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  sorghum_layer->enable_compressed_btf = true;

  EVOENGINE_LOG("Sorghum layer enabled: " << sorghum_layer->enable_compressed_btf)
}

void PyDigitalAgriculture::SetCBTFGroup(const Handle& cbtf_group_handle) {
  const auto cbtf_group_asset = PyEvoEngine::GetAsset(cbtf_group_handle);
  if (cbtf_group_asset->GetTypeName() != "CBTFGroup") {
    EVOENGINE_ERROR("SetCBTFGroup failed: invalid asset type!")
  }
  auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  sorghum_layer->leaf_cbtf_group = cbtf_group_asset;

  EVOENGINE_LOG("Sorghum layer set leaf cbtf group: " << sorghum_layer->leaf_cbtf_group.GetAssetHandle())
}

bool PyDigitalAgriculture::CheckBTFComponentsExist() {
  const auto scene = Application::GetActiveScene();
  EVOENGINE_LOG("!scene " << !scene )
  const auto owners = scene->GetPrivateComponentOwnersList<BtfMeshRenderer>();

  EVOENGINE_LOG("btf material count: " << owners.size())

  return !owners.empty();

}

void PyDigitalAgriculture::SetSkyDome() {
  auto ray_tracer_layer = Application::GetLayer<RayTracerLayer>();
  ray_tracer_layer->environment_properties.environmental_lighting_type = EnvironmentalLightingType::Skydome;

  EVOENGINE_LOG("sky info: " << ray_tracer_layer->environment_properties.sun_direction.x << ","
                             << ray_tracer_layer->environment_properties.sun_direction.y << ","
                             << ray_tracer_layer->environment_properties.sun_direction.z )

}


void PyDigitalAgriculture::PushRayTracerLayer() {
  Application::PushLayer<RayTracerLayer>("Ray Tracer Layer");
}

void PyDigitalAgriculture::SetSunDirection(glm::vec3 angles) {
  auto ray_tracer_layer = Application::GetLayer<RayTracerLayer>();
  glm::vec3 sun_direction = glm::quat(glm::radians(angles)) * glm::vec3(0, 0, -1);
  ray_tracer_layer->environment_properties.sun_direction = sun_direction;
}

void PyDigitalAgriculture::IlluminationEstimationOnSorghum() {

  auto scene = Application::GetActiveScene();
  auto sorghum_layer = Application::GetLayer<SorghumLayer>();

  sorghum_layer->CalculateIllumination();

}




void PyDigitalAgriculture::CheckTriangleEstimator(const Entity& sorghum_entity) {
  auto scene = Application::GetActiveScene();
  auto triangle_illumination_estimator =
      scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(sorghum_entity).lock();

  triangle_illumination_estimator->PrepareLightProbeGroup();
  EVOENGINE_LOG("triangle illumination estimator total light probes: " << triangle_illumination_estimator->GetLightProbeGroup().light_probes.size())
}

Entity PyDigitalAgriculture::InstantiateSorghumField(const Handle& sorghum_field_handle,
    const Handle& sorghum_coordinates, const int seed, const int index, const float radius) {

  const auto field_asset = PyEvoEngine::GetAsset(sorghum_field_handle);
  if (field_asset->GetTypeName() != "SorghumField") {
    EVOENGINE_ERROR("InstantiateSorghumField failed: invalid asset type!")
    return {};
  }
  const auto coordinates_data = PyEvoEngine::GetAsset(sorghum_coordinates);

  const auto sorghum_field = std::dynamic_pointer_cast<SorghumField>(field_asset);

  const auto coordinates = std::dynamic_pointer_cast<SorghumCoordinates>(coordinates_data);
  glm::dvec2 offset;
  coordinates->Apply(sorghum_field, offset, index, radius);

  auto field_entity = DatasetGenerator::CreateSorghumEntity(field_asset, seed);

  return field_entity;
}

std::vector<std::vector<glm::vec3>> PyDigitalAgriculture::GetAllIlluminationEstimationResultsOnSorghum() {
  auto scene = Application::GetActiveScene();
  auto sorghum_layer = Application::GetLayer<SorghumLayer>();
    const std::vector<Entity>* sorghum_entities =
        scene->UnsafeGetPrivateComponentOwnersList<TriangleIlluminationEstimator>();
  std::vector<std::vector<glm::vec3>> results;

  // todo: maybe need to bind the result to a specific sorghum 

  for (const auto& sorghum : *sorghum_entities) {
    auto triangle_illumination_estimator =
        scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(sorghum).lock();
    auto transform = scene->GetDataComponent<Transform>(sorghum);

    std::vector<glm::vec3> result;
    // sorghum position
    result.emplace_back(transform.GetPosition());
    // sorghum rotation
    result.emplace_back(transform.GetEulerRotation());
    // estimator area
    result.emplace_back(triangle_illumination_estimator->total_area);
    // total flux
    result.emplace_back(triangle_illumination_estimator->total_flux);
    // average flux
    result.emplace_back(triangle_illumination_estimator->average_flux);

    results.emplace_back(result);

  }
  return results;
}

// todo: given the sorghum field prepare the PARSensor group according to the sorghums
// for now, it is just using a fixed grid
Handle PyDigitalAgriculture::SetPARSensors(const Entity& sorghum_field) {
  auto sensor_group_handle = PyEvoEngine::CreateRuntimeAsset("PARSensorGroup");
  
  auto sensor_group_asset = PyEvoEngine::GetAsset(sensor_group_handle);
  const auto sensors = std::dynamic_pointer_cast<PARSensorGroup>(sensor_group_asset);

  auto& samplers = sensors->samplers;


  glm::vec3 max_range(2, 1.1, 2);
  glm::vec3 min_range(-2, 0.5, -2);
  float step = 0.3f;

  const int sx = static_cast<int>((max_range.x - min_range.x + step) / step);
  const int sy = static_cast<int>((max_range.y - min_range.y + step) / step);
  const int sz = static_cast<int>((max_range.z - min_range.z + step) / step);
  const auto voxel_size = sx * sy * sz;
  samplers.resize(voxel_size);
  Jobs::RunParallelFor(voxel_size, [&](unsigned i) {
    float z = (i % sz) * step + min_range.z;
    float y = ((i / sz) % sy) * step + min_range.y;
    float x = ((i / sz / sy) % sx) * step + min_range.x;
    glm::vec3 start = {x, y, z};
    samplers[i].v_0.position = samplers[i].v_1.position = samplers[i].v_2.position = start;
    samplers[i].front_face = true;
    samplers[i].back_face = false;
    samplers[i].v_0.normal = samplers[i].v_1.normal = samplers[i].v_2.normal = glm::vec3(0, 1, 0);
  });
  EVOENGINE_LOG("sensor counts: " << sensors->samplers.size())
  return sensor_group_handle;
}

void PyDigitalAgriculture::IlluminationEstimationOnSensors(const Handle& sensor_group_handle) {

  const auto sensor_group_asset = PyEvoEngine::GetAsset(sensor_group_handle);

  const auto sensors = std::dynamic_pointer_cast<PARSensorGroup>(sensor_group_asset);

  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  sensors->CalculateIllumination(sorghum_layer->ray_properties, sorghum_layer->m_seed, sorghum_layer->push_distance);
}

std::vector<std::vector<glm::vec3>> PyDigitalAgriculture::GetAllIlluminationEstimationResultsFromSensors(
    const Handle& sensor_group_handle) {
  const auto sensor_group_asset = PyEvoEngine::GetAsset(sensor_group_handle);

  const auto sensors = std::dynamic_pointer_cast<PARSensorGroup>(sensor_group_asset);

  auto& samplers = sensors->samplers;

  std::vector<std::vector<glm::vec3>> results;

  // todo: maybe need to bind the result to a specific sorghum

  for (const auto& sampler : samplers) {


    std::vector<glm::vec3> result;
    // sampler position (3 vertices are at the same position for now)
    result.emplace_back(sampler.v_0.position);

    // energy
    result.emplace_back(sampler.energy);

    // energy dominant direction
    result.emplace_back(sampler.direction);


    results.emplace_back(result);
  }
  return results;

}


#endif
