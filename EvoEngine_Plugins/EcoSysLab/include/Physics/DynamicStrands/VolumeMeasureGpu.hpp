#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace eco_sys_lab_plugin {

/// GPU / CSV layout matching VolumeMeasure.glsl `VolumeMeasureResult`.
struct GpuVolumeMeasureResult {
  float cumulative_volume = 0.0f;
  float initial_cumulative_volume = 0.0f;
  uint32_t frame_index = 0;
  uint32_t padding0 = 0;
};

/// e.g. `Metadata/kinetic_volume_measure_20260930_112345.csv`
std::filesystem::path MakeTimestampedVolumeMeasureCsvPath(const std::string& stem);

/// Appends one row: frame, absolute cumulative, percent of initial.
class VolumeMeasureCsvLogger {
 public:
  bool Open(const std::filesystem::path& path, const std::string& label);
  void Close();
  bool IsOpen() const {
    return stream_.is_open();
  }
  void Append(uint32_t frame_index, double cumulative_volume, double initial_cumulative_volume);

 private:
  std::ofstream stream_;
  std::string label_;
};

}  // namespace eco_sys_lab_plugin
