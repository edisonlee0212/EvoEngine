#include "ParamSpaceExplorer.hpp"
#include "MaizeTasselDescriptor.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <imgui.h>
#include <limits>
#include <string>

using namespace l_system_plugin;

namespace {
constexpr const char* kModeNames[] = {"Stopped", "Lissajous", "Random Walk", "Linear Interp"};
constexpr float kPi = 3.14159265358979f;
constexpr float kGoldenAngle = 2.39996323f;  // radians

std::string MakeCompactLabel(const std::string& s) {
  std::string compact;
  compact.reserve(4);
  for (const char c : s) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      compact.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
      if (compact.size() >= 4) {
        break;
      }
    }
  }
  if (compact.empty()) {
    compact = "AXIS";
  }
  return compact;
}

bool ContainsCaseInsensitive(const char* haystack, const char* needle) {
  if (!haystack || !needle) {
    return false;
  }
  if (*needle == '\0') {
    return true;
  }

  const std::string h(haystack);
  const std::string n(needle);
  auto to_lower = [](unsigned char c) { return static_cast<char>(std::tolower(c)); };

  std::string hl;
  std::string nl;
  hl.resize(h.size());
  nl.resize(n.size());
  std::transform(h.begin(), h.end(), hl.begin(), to_lower);
  std::transform(n.begin(), n.end(), nl.begin(), to_lower);
  return hl.find(nl) != std::string::npos;
}

uint64_t HashCombine(uint64_t seed, uint64_t value) {
  return seed ^ (value + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U));
}

void ExpandRange(float value, float& min_v, float& max_v) {
  min_v = std::min(min_v, value);
  max_v = std::max(max_v, value);
}

void SymmetricRange(float min_source, float max_source, float& min_out, float& max_out, float min_span) {
  float lo = std::min(min_source, max_source);
  float hi = std::max(min_source, max_source);
  const float span = std::max(min_span, hi - lo);
  min_out = lo - span;
  max_out = hi + span;
}
}  // namespace

// ---------------------------------------------------------------------------
// Bind
// ---------------------------------------------------------------------------

void ParamSpaceExplorer::Bind(MaizeTasselDescriptor& d) {
  bound_desc_ = &d;
  RebuildAxes();
}

// ---------------------------------------------------------------------------
// Normalized helpers
// ---------------------------------------------------------------------------

float ParamSpaceExplorer::GetNormalized(size_t i) const {
  if (i >= axes_.size()) {
    return 0.0f;
  }
  const auto& a = axes_[i];
  if (!a.getter) {
    return 0.0f;
  }
  if (a.max_val <= a.min_val) return 0.5f;
  return (a.getter() - a.min_val) / (a.max_val - a.min_val);
}

void ParamSpaceExplorer::SetNormalized(size_t i, float t01) {
  if (i >= axes_.size()) {
    return;
  }
  t01 = std::clamp(t01, 0.0f, 1.0f);
  const auto& a = axes_[i];
  if (!a.setter) {
    return;
  }
  a.setter(a.min_val + t01 * (a.max_val - a.min_val));
}

void ParamSpaceExplorer::GetNormalizedPoint(std::vector<float>& out) const {
  out.resize(axes_.size());
  for (size_t i = 0; i < axes_.size(); i++) {
    out[i] = GetNormalized(i);
  }
}

void ParamSpaceExplorer::SetNormalizedPoint(const std::vector<float>& pt) {
  const size_t n = std::min(pt.size(), axes_.size());
  for (size_t i = 0; i < n; i++) {
    SetNormalized(i, pt[i]);
  }
}

void ParamSpaceExplorer::PushTrailPoint() {
  if (axes_.empty() || trail_.empty()) {
    return;
  }

  std::vector<float> pt;
  GetNormalizedPoint(pt);
  trail_[trail_head_] = pt;
  trail_head_ = (trail_head_ + 1) % kTrailCapacity;
  if (trail_count_ < kTrailCapacity) trail_count_++;
}

// ---------------------------------------------------------------------------
// Axis schema
// ---------------------------------------------------------------------------

void ParamSpaceExplorer::AddAxis(const std::string& label,
                                 const std::string& short_label,
                                 float min_val,
                                 float max_val,
                                 std::function<float()> getter,
                                 std::function<void(float)> setter) {
  ParamSpaceAxis axis;
  axis.label = label;
  axis.short_label = short_label.empty() ? MakeCompactLabel(label) : short_label;
  axis.min_val = min_val;
  axis.max_val = max_val;
  axis.getter = std::move(getter);
  axis.setter = std::move(setter);
  axes_.emplace_back(std::move(axis));
}

void ParamSpaceExplorer::AddSingle(const std::string& label_prefix,
                                   const std::string& short_prefix,
                                   evo_engine::SingleDistribution<float>& dist,
                                   float mean_min,
                                   float mean_max,
                                   float deviation_max) {
  auto* d = &dist;
  AddAxis(label_prefix + ".mean", short_prefix + "M", mean_min, mean_max,
          [d]() { return d->mean; },
          [d](float v) { d->mean = v; });

  const float dev_hi = std::max(deviation_max, d->deviation * 2.0f + 0.01f);
  AddAxis(label_prefix + ".deviation", short_prefix + "D", 0.0f, dev_hi,
          [d]() { return d->deviation; },
          [d](float v) { d->deviation = std::max(0.0f, v); });
}

void ParamSpaceExplorer::AddCurve(const std::string& label_prefix,
                                  const std::string& short_prefix,
                                  evo_engine::Curve2D& curve) {
  auto* curve_ptr = &curve;
  auto& values = curve_ptr->UnsafeGetValues();
  if (values.empty()) {
    return;
  }

  float x_min = std::numeric_limits<float>::max();
  float x_max = -std::numeric_limits<float>::max();
  float y_min = std::numeric_limits<float>::max();
  float y_max = -std::numeric_limits<float>::max();
  for (const auto& p : values) {
    ExpandRange(p.x, x_min, x_max);
    ExpandRange(p.y, y_min, y_max);
  }

  float xr_min = 0.0f;
  float xr_max = 1.0f;
  float yr_min = 0.0f;
  float yr_max = 1.0f;
  SymmetricRange(x_min, x_max, xr_min, xr_max, 0.5f);
  SymmetricRange(y_min, y_max, yr_min, yr_max, 0.5f);

  for (size_t i = 0; i < values.size(); i++) {
    const std::string idx = "[" + std::to_string(i) + "]";
    AddAxis(label_prefix + idx + ".x", short_prefix + "X", xr_min, xr_max,
            [curve_ptr, i]() {
              auto& pts = curve_ptr->UnsafeGetValues();
              return i < pts.size() ? pts[i].x : 0.0f;
            },
            [curve_ptr, i](float v) {
              auto& pts = curve_ptr->UnsafeGetValues();
              if (i < pts.size()) {
                pts[i].x = v;
              }
            });

    AddAxis(label_prefix + idx + ".y", short_prefix + "Y", yr_min, yr_max,
            [curve_ptr, i]() {
              auto& pts = curve_ptr->UnsafeGetValues();
              return i < pts.size() ? pts[i].y : 0.0f;
            },
            [curve_ptr, i](float v) {
              auto& pts = curve_ptr->UnsafeGetValues();
              if (i < pts.size()) {
                pts[i].y = v;
              }
            });
  }
}

void ParamSpaceExplorer::AddPlotted(const std::string& label_prefix,
                                    const std::string& short_prefix,
                                    evo_engine::PlottedDistribution<float>& dist) {
  auto* pd = &dist;

  float mean_min_lo = 0.0f;
  float mean_min_hi = 1.0f;
  SymmetricRange(pd->mean.min_value, pd->mean.max_value, mean_min_lo, mean_min_hi, 0.25f);
  AddAxis(label_prefix + ".mean.min", short_prefix + "Mm", mean_min_lo, mean_min_hi,
          [pd]() { return pd->mean.min_value; },
          [pd](float v) { pd->mean.min_value = v; });
  AddAxis(label_prefix + ".mean.max", short_prefix + "Mx", mean_min_lo, mean_min_hi,
          [pd]() { return pd->mean.max_value; },
          [pd](float v) { pd->mean.max_value = v; });

  float dev_min_lo = 0.0f;
  float dev_min_hi = 1.0f;
  SymmetricRange(pd->deviation.min_value, pd->deviation.max_value, dev_min_lo, dev_min_hi, 0.1f);
  AddAxis(label_prefix + ".deviation.min", short_prefix + "Dm", dev_min_lo, dev_min_hi,
          [pd]() { return pd->deviation.min_value; },
          [pd](float v) { pd->deviation.min_value = v; });
  AddAxis(label_prefix + ".deviation.max", short_prefix + "Dx", dev_min_lo, dev_min_hi,
          [pd]() { return pd->deviation.max_value; },
          [pd](float v) { pd->deviation.max_value = v; });

  AddCurve(label_prefix + ".mean.curve", short_prefix + "C1", pd->mean.curve);
  AddCurve(label_prefix + ".deviation.curve", short_prefix + "C2", pd->deviation.curve);
}

uint64_t ParamSpaceExplorer::ComputeSchemaSignature() const {
  if (!bound_desc_) {
    return 0;
  }

  const auto& d = *bound_desc_;
  uint64_t h = 1469598103934665603ULL;

  auto add_curve = [&](const evo_engine::Curve2D& curve) {
    const size_t n = const_cast<evo_engine::Curve2D&>(curve).UnsafeGetValues().size();
    h = HashCombine(h, static_cast<uint64_t>(n));
  };
  auto add_plot = [&](const evo_engine::PlottedDistribution<float>& pd) {
    add_curve(pd.mean.curve);
    add_curve(pd.deviation.curve);
  };

  add_plot(d.branch_internode_length);
  add_plot(d.branch_internode_thickness);
  add_plot(d.lateral_insertion_angle);
  add_plot(d.lateral_internode_length);
  add_plot(d.lateral_node_count);
  add_plot(d.peduncle_branch_probability);
  add_plot(d.spike_internode_length);
  add_plot(d.spike_internode_thickness);
  add_plot(d.spike_zone_branch_probability);
  add_plot(d.main_pair_proximal_scale_x);
  add_plot(d.main_pair_proximal_scale_y);
  add_plot(d.main_pair_proximal_scale_z);
  add_plot(d.main_pair_proximal_angle);
  add_plot(d.main_pair_internode_length);
  add_plot(d.main_pair_internode_thickness);
  add_plot(d.main_pair_internode_angle);
  add_plot(d.main_pair_distal_scale_x);
  add_plot(d.main_pair_distal_scale_y);
  add_plot(d.main_pair_distal_scale_z);
  add_plot(d.main_pair_distal_angle);
  add_plot(d.branch_pair_proximal_scale_x);
  add_plot(d.branch_pair_proximal_scale_y);
  add_plot(d.branch_pair_proximal_scale_z);
  add_plot(d.branch_pair_proximal_angle);
  add_plot(d.branch_pair_internode_length);
  add_plot(d.branch_pair_internode_thickness);
  add_plot(d.branch_pair_internode_angle);
  add_plot(d.branch_pair_distal_scale_x);
  add_plot(d.branch_pair_distal_scale_y);
  add_plot(d.branch_pair_distal_scale_z);
  add_plot(d.branch_pair_distal_angle);
  add_plot(d.lateral_initiation_delay_gdd);
  add_plot(d.spike_anthesis_offset_gdd);
  add_plot(d.primary_lateral_branch_probability);
  add_plot(d.secondary_lateral_branch_probability);

  add_curve(d.rachis_elongation_curve);
  add_curve(d.rachis_thickness_curve);
  add_curve(d.lateral_elongation_curve);
  add_curve(d.lateral_thickness_curve);
  add_curve(d.lateral_angle_development_curve);
  add_curve(d.pair_proximal_scale_curve);
  add_curve(d.pair_proximal_angle_curve);
  add_curve(d.pair_internode_length_curve);
  add_curve(d.pair_internode_thickness_curve);
  add_curve(d.pair_internode_angle_curve);
  add_curve(d.pair_distal_scale_curve);
  add_curve(d.pair_distal_angle_curve);

  h = HashCombine(h, static_cast<uint64_t>(d.tropisms.size()));
  for (const auto& t : d.tropisms) {
    add_plot(t.order_response);
  }
  return h;
}

void ParamSpaceExplorer::RebuildAxes() {
  if (!bound_desc_) {
    axes_.clear();
    return;
  }

  auto& d = *bound_desc_;
  axes_.clear();

  // Single distributions (mean + deviation).
  AddSingle("branch_node_count", "BNC", d.branch_node_count, 0.0f, 80.0f, 20.0f);
  AddSingle("spike_node_count", "SNC", d.spike_node_count, 0.0f, 120.0f, 30.0f);
  AddSingle("phyllotaxis_angle", "PHY", d.phyllotaxis_angle, 0.0f, 360.0f, 180.0f);
  AddSingle("branch_azimuth_offset", "BAO", d.branch_azimuth_offset, -180.0f, 180.0f, 180.0f);
  AddSingle("lateral_thickness_ratio", "LTR", d.lateral_thickness_ratio, 0.0f, 2.0f, 1.0f);
  AddSingle("secondary_insertion_angle", "SIA", d.secondary_insertion_angle, 0.0f, 120.0f, 60.0f);
  AddSingle("secondary_internode_length", "SIL", d.secondary_internode_length, 0.0f, 20.0f, 10.0f);
  AddSingle("secondary_internode_thickness", "SIT", d.secondary_internode_thickness, 0.0f, 2.0f, 1.0f);
  AddSingle("secondary_node_count", "SNN", d.secondary_node_count, 0.0f, 20.0f, 10.0f);
  AddSingle("final_age_gdd", "FAG", d.final_age_gdd, 0.0f, 3000.0f, 1500.0f);

  // Global development distributions.
  AddSingle("base_temperature", "TMP", d.base_temperature, -10.0f, 60.0f, 30.0f);
  AddSingle("plastochron_gdd", "PGD", d.plastochron_gdd, 1.0f, 500.0f, 250.0f);
  AddSingle("anthesis_gdd", "AGD", d.anthesis_gdd, 0.0f, 3000.0f, 1500.0f);
  AddSingle("maturity_gdd", "MGD", d.maturity_gdd, 0.0f, 5000.0f, 2500.0f);

  // All plotted distributions: ranges + all curve control points.
  AddPlotted("branch_internode_length", "BIL", d.branch_internode_length);
  AddPlotted("branch_internode_thickness", "BIT", d.branch_internode_thickness);
  AddPlotted("lateral_insertion_angle", "LIA", d.lateral_insertion_angle);
  AddPlotted("lateral_internode_length", "LIL", d.lateral_internode_length);
  AddPlotted("lateral_node_count", "LNC", d.lateral_node_count);
  AddPlotted("peduncle_branch_probability", "PBP", d.peduncle_branch_probability);
  AddPlotted("spike_internode_length", "SIL", d.spike_internode_length);
  AddPlotted("spike_internode_thickness", "SIT", d.spike_internode_thickness);
  AddPlotted("spike_zone_branch_probability", "SZP", d.spike_zone_branch_probability);

  AddPlotted("main_pair_proximal_scale_x", "MPX", d.main_pair_proximal_scale_x);
  AddPlotted("main_pair_proximal_scale_y", "MPY", d.main_pair_proximal_scale_y);
  AddPlotted("main_pair_proximal_scale_z", "MPZ", d.main_pair_proximal_scale_z);
  AddPlotted("main_pair_proximal_angle", "MPA", d.main_pair_proximal_angle);
  AddPlotted("main_pair_internode_length", "MIL", d.main_pair_internode_length);
  AddPlotted("main_pair_internode_thickness", "MIT", d.main_pair_internode_thickness);
  AddPlotted("main_pair_internode_angle", "MIA", d.main_pair_internode_angle);
  AddPlotted("main_pair_distal_scale_x", "MDX", d.main_pair_distal_scale_x);
  AddPlotted("main_pair_distal_scale_y", "MDY", d.main_pair_distal_scale_y);
  AddPlotted("main_pair_distal_scale_z", "MDZ", d.main_pair_distal_scale_z);
  AddPlotted("main_pair_distal_angle", "MDA", d.main_pair_distal_angle);

  AddPlotted("branch_pair_proximal_scale_x", "BPX", d.branch_pair_proximal_scale_x);
  AddPlotted("branch_pair_proximal_scale_y", "BPY", d.branch_pair_proximal_scale_y);
  AddPlotted("branch_pair_proximal_scale_z", "BPZ", d.branch_pair_proximal_scale_z);
  AddPlotted("branch_pair_proximal_angle", "BPA", d.branch_pair_proximal_angle);
  AddPlotted("branch_pair_internode_length", "BIL", d.branch_pair_internode_length);
  AddPlotted("branch_pair_internode_thickness", "BIT", d.branch_pair_internode_thickness);
  AddPlotted("branch_pair_internode_angle", "BIA", d.branch_pair_internode_angle);
  AddPlotted("branch_pair_distal_scale_x", "BDX", d.branch_pair_distal_scale_x);
  AddPlotted("branch_pair_distal_scale_y", "BDY", d.branch_pair_distal_scale_y);
  AddPlotted("branch_pair_distal_scale_z", "BDZ", d.branch_pair_distal_scale_z);
  AddPlotted("branch_pair_distal_angle", "BDA", d.branch_pair_distal_angle);

  AddPlotted("lateral_initiation_delay_gdd", "LID", d.lateral_initiation_delay_gdd);
  AddPlotted("spike_anthesis_offset_gdd", "SAO", d.spike_anthesis_offset_gdd);
  AddPlotted("primary_lateral_branch_probability", "PLP", d.primary_lateral_branch_probability);
  AddPlotted("secondary_lateral_branch_probability", "SLP", d.secondary_lateral_branch_probability);

  // Direct curve axes.
  AddCurve("rachis_elongation_curve", "REC", d.rachis_elongation_curve);
  AddCurve("rachis_thickness_curve", "RTC", d.rachis_thickness_curve);
  AddCurve("lateral_elongation_curve", "LEC", d.lateral_elongation_curve);
  AddCurve("lateral_thickness_curve", "LTC", d.lateral_thickness_curve);
  AddCurve("lateral_angle_development_curve", "LAC", d.lateral_angle_development_curve);
  AddCurve("pair_proximal_scale_curve", "PPS", d.pair_proximal_scale_curve);
  AddCurve("pair_proximal_angle_curve", "PPA", d.pair_proximal_angle_curve);
  AddCurve("pair_internode_length_curve", "PIL", d.pair_internode_length_curve);
  AddCurve("pair_internode_thickness_curve", "PIT", d.pair_internode_thickness_curve);
  AddCurve("pair_internode_angle_curve", "PIA", d.pair_internode_angle_curve);
  AddCurve("pair_distal_scale_curve", "PDS", d.pair_distal_scale_curve);
  AddCurve("pair_distal_angle_curve", "PDA", d.pair_distal_angle_curve);

  // Dynamic tropism dimensions.
  for (size_t i = 0; i < d.tropisms.size(); i++) {
    auto& tropism = d.tropisms[i];
    const std::string p = "tropism[" + std::to_string(i) + "]";
    const std::string s = "T" + std::to_string(i);

    AddSingle(p + ".direction_x", s + "X", tropism.direction_x, -1.0f, 1.0f, 1.0f);
    AddSingle(p + ".direction_y", s + "Y", tropism.direction_y, -1.0f, 1.0f, 1.0f);
    AddSingle(p + ".direction_z", s + "Z", tropism.direction_z, -1.0f, 1.0f, 1.0f);
    AddSingle(p + ".strength", s + "S", tropism.strength, -5.0f, 5.0f, 5.0f);

    auto* tropism_ptr = &d.tropisms[i];
    AddAxis(p + ".usage_chance_percent", s + "U", 0.0f, 100.0f,
            [tropism_ptr]() { return tropism_ptr->usage_chance_percent; },
            [tropism_ptr](float v) { tropism_ptr->usage_chance_percent = std::clamp(v, 0.0f, 100.0f); });

    AddPlotted(p + ".order_response", s + "O", tropism.order_response);
  }

  schema_signature_ = ComputeSchemaSignature();

  // Resize runtime state.
  trail_.assign(kTrailCapacity, std::vector<float>(axes_.size(), 0.0f));
  trail_head_ = 0;
  trail_count_ = 0;

  snapshot_a_.assign(axes_.size(), 0.0f);
  snapshot_b_.assign(axes_.size(), 0.0f);
  has_snapshot_a_ = false;
  has_snapshot_b_ = false;
  interp_t_ = 0.0f;

  if (!axes_.empty()) {
    visible_axis_count_ = std::clamp(visible_axis_count_, 4, std::max(4, static_cast<int>(axes_.size())));
    visible_axis_offset_ = std::clamp(visible_axis_offset_, 0,
                                      std::max(0, static_cast<int>(axes_.size()) - visible_axis_count_));
    PushTrailPoint();
  }
}

// ---------------------------------------------------------------------------
// Tick — advance motion one frame
// ---------------------------------------------------------------------------

bool ParamSpaceExplorer::Tick(float dt) {
  if (!bound_desc_ || !playing || mode == ParamMotionMode::Stopped || axes_.empty()) return false;

  time_ += static_cast<double>(dt * speed);

  switch (mode) {
    case ParamMotionMode::Lissajous: {
      const float t = static_cast<float>(time_);
      for (size_t i = 0; i < axes_.size(); i++) {
        const float freq = 1.0f + std::fmod(static_cast<float>(i) * 0.6180339f, 17.0f);
        const float phase = static_cast<float>(i) * kGoldenAngle;
        const float val = 0.5f + 0.5f * std::sin(2.0f * kPi * freq * t + phase);
        SetNormalized(i, val);
      }
      PushTrailPoint();
      return true;
    }
    case ParamMotionMode::RandomWalk: {
      std::uniform_real_distribution<float> step_dist(-0.02f, 0.02f);
      for (size_t i = 0; i < axes_.size(); i++) {
        float cur = GetNormalized(i);
        float next = cur + step_dist(rng_) * speed;
        // Reflect at boundaries.
        if (next < 0.0f) next = -next;
        if (next > 1.0f) next = 2.0f - next;
        SetNormalized(i, std::clamp(next, 0.0f, 1.0f));
      }
      PushTrailPoint();
      return true;
    }
    case ParamMotionMode::LinearInterp: {
      if (!has_snapshot_a_ || !has_snapshot_b_) return false;
      interp_t_ += dt * speed * 0.5f;
      if (interp_t_ >= 1.0f) {
        interp_t_ = 0.0f;
        std::swap(snapshot_a_, snapshot_b_);
      }
      const size_t n = std::min(snapshot_a_.size(), axes_.size());
      for (size_t i = 0; i < n; i++) {
        const float v = snapshot_a_[i] + interp_t_ * (snapshot_b_[i] - snapshot_a_[i]);
        SetNormalized(i, v);
      }
      PushTrailPoint();
      return true;
    }
    default:
      return false;
  }
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

void ParamSpaceExplorer::SaveSnapshotA() {
  GetNormalizedPoint(snapshot_a_);
  has_snapshot_a_ = true;
}

void ParamSpaceExplorer::SaveSnapshotB() {
  GetNormalizedPoint(snapshot_b_);
  has_snapshot_b_ = true;
}

void ParamSpaceExplorer::Reset() {
  playing = false;
  time_ = 0.0;
  trail_head_ = 0;
  trail_count_ = 0;
  interp_t_ = 0.0f;
  rng_.seed(42);
}

// ---------------------------------------------------------------------------
// Parallel-coordinates canvas
// ---------------------------------------------------------------------------

std::vector<int> ParamSpaceExplorer::GetVisibleAxisIndices() const {
  std::vector<int> result;
  if (axes_.empty()) {
    return result;
  }

  const int count = std::clamp(visible_axis_count_, 1, static_cast<int>(axes_.size()));
  const int max_offset = std::max(0, static_cast<int>(axes_.size()) - count);
  const int offset = std::clamp(visible_axis_offset_, 0, max_offset);

  result.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; i++) {
    result.emplace_back(offset + i);
  }
  return result;
}

void ParamSpaceExplorer::DrawParallelCoords(float width, float height, const std::vector<int>& axis_indices) {
  if (axis_indices.size() < 2) {
    ImGui::TextUnformatted("Select at least 2 visible axes.");
    return;
  }

  const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
  const ImVec2 canvas_size(width, height);
  ImDrawList* dl = ImGui::GetWindowDrawList();

  // Background.
  dl->AddRectFilled(
      canvas_pos,
      ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
      IM_COL32(25, 25, 30, 255));

  // Reserve the space in the layout.
  ImGui::Dummy(canvas_size);

  constexpr float kMarginL = 14.0f;
  constexpr float kMarginR = 14.0f;
  constexpr float kMarginT = 6.0f;
  constexpr float kMarginB = 18.0f;
  const float plot_w = canvas_size.x - kMarginL - kMarginR;
  const float plot_h = canvas_size.y - kMarginT - kMarginB;
  if (plot_w < 10.0f || plot_h < 10.0f) return;

  const int axis_count = static_cast<int>(axis_indices.size());
  const float spacing = plot_w / static_cast<float>(axis_count - 1);

  // Draw vertical axes.
  for (int i = 0; i < axis_count; i++) {
    const int axis_idx = axis_indices[i];
    const float x = canvas_pos.x + kMarginL + static_cast<float>(i) * spacing;
    const float y_top = canvas_pos.y + kMarginT;
    const float y_bot = canvas_pos.y + canvas_size.y - kMarginB;
    dl->AddLine(ImVec2(x, y_top), ImVec2(x, y_bot), IM_COL32(70, 70, 80, 255));

    // Short label below axis.
    const ImVec2 ts = ImGui::CalcTextSize(axes_[axis_idx].short_label.c_str());
    dl->AddText(ImVec2(x - ts.x * 0.5f, y_bot + 2.0f),
                IM_COL32(140, 140, 150, 255), axes_[axis_idx].short_label.c_str());
  }

  // Helper lambda: axis index -> screen x/y from normalized value.
  auto axis_xy = [&](int visible_i, float norm) -> ImVec2 {
    const float x = canvas_pos.x + kMarginL + static_cast<float>(visible_i) * spacing;
    const float y = canvas_pos.y + kMarginT + (1.0f - norm) * plot_h;
    return ImVec2(x, y);
  };

  // Draw trail polylines (older = more transparent).
  for (int t = 0; t < trail_count_; t++) {
    const int idx = (trail_head_ - trail_count_ + t + kTrailCapacity) % kTrailCapacity;
    const float alpha = static_cast<float>(t + 1) / static_cast<float>(trail_count_) * 0.35f;
    const ImU32 color = IM_COL32(100, 170, 255, static_cast<int>(alpha * 255));
    for (int i = 0; i < axis_count - 1; i++) {
      const int a = axis_indices[i];
      const int b = axis_indices[i + 1];
      if (a >= static_cast<int>(trail_[idx].size()) || b >= static_cast<int>(trail_[idx].size())) {
        continue;
      }
      dl->AddLine(axis_xy(i, trail_[idx][a]),
                  axis_xy(i + 1, trail_[idx][b]),
                  color, 1.0f);
    }
  }

  // Draw current point (bright polyline + dots).
  {
    std::vector<float> cur;
    GetNormalizedPoint(cur);
    const ImU32 bright = IM_COL32(255, 220, 80, 255);
    for (int i = 0; i < axis_count - 1; i++) {
      const int a = axis_indices[i];
      const int b = axis_indices[i + 1];
      if (a >= static_cast<int>(cur.size()) || b >= static_cast<int>(cur.size())) {
        continue;
      }
      dl->AddLine(axis_xy(i, cur[a]), axis_xy(i + 1, cur[b]), bright, 2.0f);
    }
    for (int i = 0; i < axis_count; i++) {
      const int axis_idx = axis_indices[i];
      if (axis_idx >= static_cast<int>(cur.size())) {
        continue;
      }
      dl->AddCircleFilled(axis_xy(i, cur[axis_idx]), 3.0f, bright);
    }

    // Value annotations next to dots (on hover only to avoid clutter).
    const ImVec2 mouse = ImGui::GetMousePos();
    for (int i = 0; i < axis_count; i++) {
      const int axis_idx = axis_indices[i];
      if (axis_idx >= static_cast<int>(cur.size())) {
        continue;
      }
      const ImVec2 dot = axis_xy(i, cur[axis_idx]);
      const float dx = mouse.x - dot.x;
      const float dy = mouse.y - dot.y;
      if (dx * dx + dy * dy < 100.0f) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.4f", axes_[axis_idx].getter ? axes_[axis_idx].getter() : 0.0f);
        dl->AddText(ImVec2(dot.x + 6.0f, dot.y - 8.0f),
                    IM_COL32(255, 255, 255, 230), buf);
        // Also show full label as tooltip.
        ImGui::SetTooltip("%s: %s", axes_[axis_idx].label.c_str(), buf);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// OnInspect — full explorer UI
// ---------------------------------------------------------------------------

bool ParamSpaceExplorer::OnInspect() {
  if (!bound_desc_) return false;

  const uint64_t new_signature = ComputeSchemaSignature();
  if (new_signature != schema_signature_) {
    RebuildAxes();
  }

  bool changed = false;
  bool preferences_changed = false;

  ImGui::Text("Total dimensions: %d", static_cast<int>(axes_.size()));
  if (ImGui::SmallButton("Rebuild Axis Schema")) {
    RebuildAxes();
  }

  if (axes_.empty()) {
    ImGui::TextUnformatted("No axes available.");
    return false;
  }

  // Mode selector.
  int mode_int = static_cast<int>(mode);
  if (ImGui::Combo("Motion Mode", &mode_int, kModeNames, 4)) {
    mode = static_cast<ParamMotionMode>(mode_int);
    preferences_changed = true;
  }

  if (ImGui::DragFloat("Speed", &speed, 0.01f, 0.01f, 10.0f, "%.2f")) {
    preferences_changed = true;
  }

  if (ImGui::Button(playing ? "Pause" : "Play")) {
    playing = !playing;
    preferences_changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Reset")) {
    Reset();
    preferences_changed = true;
    PushTrailPoint();
  }

  // Snapshot controls for LinearInterp.
  if (mode == ParamMotionMode::LinearInterp) {
    if (ImGui::Button("Set A")) {
      SaveSnapshotA();
      preferences_changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Set B")) {
      SaveSnapshotB();
      preferences_changed = true;
    }
    ImGui::SameLine();
    ImGui::Text("A:%s B:%s",
                has_snapshot_a_ ? "set" : "---",
                has_snapshot_b_ ? "set" : "---");
    if (!has_snapshot_a_ || !has_snapshot_b_) {
      ImGui::TextColored(ImVec4(1, 1, 0, 1),
                         "Set both A and B snapshots before playing.");
    }
  }

  visible_axis_count_ = std::clamp(visible_axis_count_, 2, std::max(2, static_cast<int>(axes_.size())));
  const int max_offset = std::max(0, static_cast<int>(axes_.size()) - visible_axis_count_);
  visible_axis_offset_ = std::clamp(visible_axis_offset_, 0, max_offset);
  ImGui::DragInt("Visible Axis Count", &visible_axis_count_, 1.0f, 2,
                 std::max(2, static_cast<int>(axes_.size())));
  ImGui::DragInt("Visible Axis Offset", &visible_axis_offset_, 1.0f, 0, max_offset);

  const auto visible_axis_indices = GetVisibleAxisIndices();

  // Parallel-coordinates canvas.
  const float canvas_w = std::max(ImGui::GetContentRegionAvail().x, 200.0f);
  DrawParallelCoords(canvas_w, 180.0f, visible_axis_indices);

  // Per-axis sliders.
  if (ImGui::TreeNodeEx("Axis Sliders", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("Show Window Only", &sliders_window_only_);
    ImGui::InputText("Search", axis_search_.data(), axis_search_.size());

    int slider_count = 0;
    auto try_draw_axis = [&](int axis_idx) {
      if (axis_idx < 0 || axis_idx >= static_cast<int>(axes_.size())) {
        return;
      }
      auto& axis = axes_[axis_idx];
      if (!ContainsCaseInsensitive(axis.label.c_str(), axis_search_.data())) {
        return;
      }

      ImGui::PushID(axis_idx);
      float val = axis.getter ? axis.getter() : 0.0f;
      const float step = std::max(0.0001f, (axis.max_val - axis.min_val) * 0.005f);
      if (ImGui::DragFloat(axis.label.c_str(), &val, step,
                           axis.min_val, axis.max_val, "%.4f")) {
        if (axis.setter) {
          axis.setter(val);
        }
        changed = true;
      }
      ImGui::PopID();
      slider_count++;
    };

    if (sliders_window_only_) {
      for (const int axis_idx : visible_axis_indices) {
        try_draw_axis(axis_idx);
      }
    } else {
      for (int axis_idx = 0; axis_idx < static_cast<int>(axes_.size()); axis_idx++) {
        try_draw_axis(axis_idx);
      }
    }

    ImGui::Text("Displayed sliders: %d", slider_count);
    if (changed) PushTrailPoint();
    ImGui::TreePop();
  }

  // Tick if playing.
  if (playing) {
    changed |= Tick(ImGui::GetIO().DeltaTime);
  }

  return changed || preferences_changed;
}
