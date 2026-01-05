#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace eco_sys_lab_package {

/// GPU / CSV layout matching VolumeMeasure.slang `VolumeMeasureResult`.
struct GpuVolumeMeasureResult {
  float cumulative_volume = 0.0f;
  float initial_cumulative_volume = 0.0f;
  uint32_t frame_index = 0;
  uint32_t padding0 = 0;
};

/// e.g. `Metadata/kinetic_volume_measure_20260930_112345.csv`
std::filesystem::path MakeTimestampedVolumeMeasureCsvPath(const std::string& stem);

/// Appends one row: frame, absolute cumulative, percent of initial.
/// When opened with @ref OpenWithSimulationTime, rows use @ref AppendAtTime instead.
class VolumeMeasureCsvLogger {
 public:
  bool Open(const std::filesystem::path& path, const std::string& label);
  /// Same as @ref Open but header/rows use simulation time instead of frame index.
  bool OpenWithSimulationTime(const std::filesystem::path& path, const std::string& label);
  void Close();
  bool IsOpen() const {
    return stream_.is_open();
  }
  void Append(uint32_t frame_index, double cumulative_volume, double initial_cumulative_volume);
  void AppendAtTime(double simulation_time, double cumulative_volume, double initial_cumulative_volume);

 private:
  bool OpenInternal(const std::filesystem::path& path, const std::string& label, bool use_simulation_time);
  std::ofstream stream_;
  std::string label_;
  bool use_simulation_time_ = false;
};

}  // namespace eco_sys_lab_package
