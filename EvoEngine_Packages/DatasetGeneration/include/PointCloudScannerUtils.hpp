#pragma once
#include "PointCloudSample.hpp"

namespace dataset_generation_package {
using namespace evo_engine;
class PointCloudCaptureSettings {
 public:
  bool output_spline_info = false;
  uint32_t spline_subdivision_count = 8;
  enum class CaptureMode {
    Cpu,
    Gpu,
  };

  CaptureMode capture_mode = CaptureMode::Gpu;

  virtual bool OnInspect() = 0;
  virtual void Save(const std::string& name, YAML::Emitter& out) const {
  }
  virtual void Load(const std::string& name, const YAML::Node& in) {
  }
  virtual void GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) = 0;
  virtual bool SampleFilter(const PointCloudSample& sample) {
    return true;
  }
};
}  // namespace dataset_generation_package
