#pragma once

namespace l_system_package {

struct RenderPublishPolicy {
  bool defer_to_main_thread = true;
  bool deduplicate_identical_payloads = true;
  float min_republish_interval_seconds = 0.0f;
};

}  // namespace l_system_package
