#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <random>
#include <string>
#include <vector>
#include <Plot2D.hpp>
#include "ILSystemExplorableDescriptor.hpp"

namespace l_system_plugin {

enum class ParamMotionMode : int {
  Stopped = 0,
  Lissajous = 1,
  RandomWalk = 2,
  LinearInterp = 3,
};

struct ParamSpaceAxis {
  std::string label;       // Full label for sliders.
  std::string short_label; // Compact axis label for parallel coords.
  float min_val = 0.0f;
  float max_val = 1.0f;
  std::function<float()> getter;
  std::function<void(float)> setter;
};

class ParamSpaceExplorer {
 public:
  static constexpr int kTrailCapacity = 200;

  ParamMotionMode mode = ParamMotionMode::Stopped;
  float speed = 1.0f;
  bool playing = false;

  /// Bind any descriptor that implements ILSystemExplorableDescriptor.
  /// Triggers RebuildAxes() which delegates axis registration back to the
  /// descriptor via RegisterExplorableAxes.
  void Bind(ILSystemExplorableDescriptor& desc);
  bool IsBound() const { return bound_desc_ != nullptr; }

  /// Draw controls + parallel-coords canvas. Returns true if any parameter changed.
  bool OnInspect();

  void Reset();
  void SaveSnapshotA();
  void SaveSnapshotB();

  // -- Axis registration helpers (called from ILSystemExplorableDescriptor
  //    ::RegisterExplorableAxes implementations). --

  void AddAxis(const std::string& label,
               const std::string& short_label,
               float min_val,
               float max_val,
               std::function<float()> getter,
               std::function<void(float)> setter);

  void AddSingle(const std::string& label_prefix,
                 const std::string& short_prefix,
                 evo_engine::SingleDistribution<float>& dist,
                 float mean_min,
                 float mean_max,
                 float deviation_max);

  void AddPlotted(const std::string& label_prefix,
                  const std::string& short_prefix,
                  evo_engine::PlottedDistribution<float>& dist);

  void AddCurve(const std::string& label_prefix,
                const std::string& short_prefix,
                evo_engine::Curve2D& curve);

 private:
  ILSystemExplorableDescriptor* bound_desc_ = nullptr;
  std::vector<ParamSpaceAxis> axes_;
  uint64_t schema_signature_ = 0;

  // Motion state.
  double time_ = 0.0;
  std::mt19937 rng_{42};

  // Trail ring buffer (normalized [0,1] per axis, all axes).
  std::vector<std::vector<float>> trail_;
  int trail_head_ = 0;
  int trail_count_ = 0;

  // Snapshots for LinearInterp (normalized).
  std::vector<float> snapshot_a_;
  std::vector<float> snapshot_b_;
  bool has_snapshot_a_ = false;
  bool has_snapshot_b_ = false;
  float interp_t_ = 0.0f;

  // UI state.
  int visible_axis_offset_ = 0;
  int visible_axis_count_ = 24;
  bool sliders_window_only_ = true;
  std::array<char, 128> axis_search_{};

  // Helpers.
  void RebuildAxes();
  uint64_t ComputeSchemaSignature() const;

  float GetNormalized(size_t i) const;
  void SetNormalized(size_t i, float t01);
  void GetNormalizedPoint(std::vector<float>& out) const;
  void SetNormalizedPoint(const std::vector<float>& pt);
  void PushTrailPoint();
  bool Tick(float dt);
  std::vector<int> GetVisibleAxisIndices() const;
  void DrawParallelCoords(float width, float height, const std::vector<int>& axis_indices);
};

}  // namespace l_system_plugin
