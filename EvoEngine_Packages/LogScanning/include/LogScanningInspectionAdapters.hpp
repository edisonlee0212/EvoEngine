#pragma once

#include "InspectorRegistry.hpp"
#include "LogScanReconstruction.hpp"

namespace log_scanning_package {
class JoeScanScanner;
class LogScan;

bool InspectJoeScanScanner(evo_engine::InspectorContext& context, JoeScanScanner& scanner);
bool InspectLogScan(evo_engine::InspectorContext& context, LogScan& log_scan);
bool DrawLogScanReconstructionParameterGui(LogScanReconstruction::ReconstructionParameter& parameter);
}  // namespace log_scanning_package
