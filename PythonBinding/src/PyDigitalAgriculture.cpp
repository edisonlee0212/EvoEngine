#include "PyDigitalAgriculture.hpp"
#include "Serialization.hpp"

#ifdef DATASET_GENERATION_PACKAGE
#  include "DatasetGenerationSerializationAdapters.hpp"
#endif

#ifdef DIGITAL_AGRICULTURE_PACKAGE
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

#endif
