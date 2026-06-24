#pragma once

namespace l_system_package {

/**
 * @brief Common publication policy shared by L-system render channels.
 *
 * Stage can be called from worker threads, while Flush must run on the main
 * thread. These knobs control how aggressively staged payloads are published.
 */
struct RenderPublishPolicy {
  /// True: defer publication until Flush(). False: attempt immediate publish.
  bool defer_to_main_thread = true;

  /// Skip publication when the staged payload matches the last published one.
  bool deduplicate_identical_payloads = true;

  /// Minimum seconds between successful publishes for this channel.
  float min_republish_interval_seconds = 0.0f;
};

}  // namespace l_system_package
