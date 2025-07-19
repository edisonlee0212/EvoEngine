#include "PyDigitalAgriculture.hpp"

#ifdef DIGITAL_AGRICULTURE_PACKAGE
namespace py = pybind11;
using namespace py_digital_agriculture_package;



void GenerateDataForSorghum(const bool use_gpu, const Entity& sorghum_entity,
                            const SorghumGantryCaptureSettings& capture_settings,
                            DatasetGenerator::SorghumDataGenerationParameters data_generation_parameters) {
  const auto gantry_capture_settings = std::make_shared<SorghumGantryCaptureSettings>();
  *gantry_capture_settings = capture_settings;
  gantry_capture_settings->grid_distance = {2.f, 2.f};
  gantry_capture_settings->grid_size.x = gantry_capture_settings->grid_size.y = 1;
  data_generation_parameters.point_cloud_capture_settings = gantry_capture_settings;
  if (use_gpu) {
    gantry_capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::Gpu;
  } else {
    gantry_capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::Cpu;
  }
  DatasetGenerator::GenerateDataForSorghum(sorghum_entity, data_generation_parameters);
}

void InitiateSorghumEntity(
    const Entity& sorghum_entity, const DatasetGenerator::SorghumDataGenerationParameters &data_generation_parameters) {


  Application::GetLayer<SorghumLayer>()->GenerateMeshForAllSorghums(
      data_generation_parameters.sorghum_mesh_generator_settings);
  Application::Loop();
  Application::Loop();
  
}

void GenerateDataForAllSorghums(const bool use_gpu, const SorghumGantryCaptureSettings& capture_settings,
                                DatasetGenerator::SorghumDataGenerationParameters data_generation_parameters) {
  const auto gantry_capture_settings = std::make_shared<SorghumGantryCaptureSettings>();
  *gantry_capture_settings = capture_settings;
  data_generation_parameters.point_cloud_capture_settings = gantry_capture_settings;
  if (use_gpu) {
    gantry_capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::Gpu;
  } else {
    gantry_capture_settings->capture_mode = PointCloudCaptureSettings::CaptureMode::Cpu;
  }
  DatasetGenerator::GenerateDataForAllSorghums(data_generation_parameters);
}

PYBIND11_MODULE(PyDigitalAgriculture, m) {
  m.doc() = "PyDigitalAgriculture";  // optional module docstring
  PyDigitalAgriculture::Initialize(m);
  m.def("GenerateDataForSorghum", &GenerateDataForSorghum, "Generate data point for single sorghum");
  m.def("GenerateDataForAllSorghums", &GenerateDataForAllSorghums,
        "Generate data point for all existing sorghum(s) in current scene");
  m.def("InitiateSorghumEntity", &InitiateSorghumEntity,
        "Initialize a sorghum entity with proper components set in the sorghum layer in the scene");
}
#endif