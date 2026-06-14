#pragma once

#include "JoeScanScanner.hpp"
#include "LogScan.hpp"

namespace log_scanning_package {
void SerializeLogScan(YAML::Emitter& out, const LogScan& target);
void DeserializeLogScan(const YAML::Node& in, LogScan& target);
void SerializeJoeScanScanner(YAML::Emitter& out, const JoeScanScanner& target);
void DeserializeJoeScanScanner(const YAML::Node& in, JoeScanScanner& target);
}  // namespace log_scanning_package
