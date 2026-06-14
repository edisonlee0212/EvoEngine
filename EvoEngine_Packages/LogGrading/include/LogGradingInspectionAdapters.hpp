#pragma once

#include "InspectorRegistry.hpp"

namespace log_grading_package {
class LogGrader;
class ProceduralLogParameters;

bool DrawProceduralLogParametersGui(ProceduralLogParameters& parameters);
bool InspectLogGrader(evo_engine::InspectorContext& context, LogGrader& log_grader);
}  // namespace log_grading_package
