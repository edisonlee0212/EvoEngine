#include "VolumeMeasureGpu.hpp"

#include "Console.hpp"
#include "EcoSysLabPaths.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace eco_sys_lab_plugin {

std::filesystem::path MakeTimestampedVolumeMeasureCsvPath(const std::string& stem) {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
#ifdef _WIN32
  localtime_s(&local_tm, &time);
#else
  localtime_r(&time, &local_tm);
#endif
  std::ostringstream name;
  name << stem << '_' << std::put_time(&local_tm, "%Y%m%d_%H%M%S") << ".csv";
  return EcoSysLabMetadataPath(name.str());
}

bool VolumeMeasureCsvLogger::Open(const std::filesystem::path& path, const std::string& label) {
  Close();
  label_ = label;
  stream_.open(path, std::ios::out | std::ios::trunc);
  if (!stream_) {
    EVOENGINE_ERROR("VolumeMeasureCsvLogger: failed to open " << path.string());
    return false;
  }
  stream_ << "frame,label,cumulative_volume,initial_cumulative_volume,percent_of_initial\n";
  stream_.flush();
  EVOENGINE_LOG("VolumeMeasureCsvLogger: writing " << label << " to " << path.string());
  return true;
}

void VolumeMeasureCsvLogger::Close() {
  if (stream_.is_open()) {
    stream_.flush();
    stream_.close();
  }
}

void VolumeMeasureCsvLogger::Append(const uint32_t frame_index, const double cumulative_volume,
                                    const double initial_cumulative_volume) {
  if (!stream_.is_open()) {
    return;
  }
  double percent = 0.0;
  if (initial_cumulative_volume > 1e-18) {
    percent = 100.0 * cumulative_volume / initial_cumulative_volume;
  }
  stream_ << frame_index << ',' << label_ << ',' << std::setprecision(9) << cumulative_volume << ','
          << initial_cumulative_volume << ',' << percent << '\n';
  stream_.flush();
}

}  // namespace eco_sys_lab_plugin
