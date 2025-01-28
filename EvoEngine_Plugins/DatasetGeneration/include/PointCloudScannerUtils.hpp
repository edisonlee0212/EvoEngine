#pragma once
#include "PointCloudSample.hpp"
using namespace evo_engine;
namespace dataset_generation_plugin {
class PointCloudCaptureSettings {
 public:
  bool output_spline_info = false;
  uint32_t spline_subdivision_count = 8;
  bool use_gpu = true;
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
}  // namespace dataset_generation_plugin