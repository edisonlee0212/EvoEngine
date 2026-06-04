#pragma once

#include <cstdint>

// ILSystemExplorableDescriptor
// ----------------------------
// Lightweight abstract interface that any L-System asset descriptor implements
// to opt-in to the generic ParamSpaceExplorer UI panel (random walk, Lissajous,
// linear interp, parallel coords, snapshots).
//
// C++ has no reflection; rather than hard-coding the explorer to a single
// descriptor type, descriptors enumerate their tunable distributions/curves
// once via RegisterExplorableAxes by calling explorer.AddSingle/AddPlotted/
// AddCurve/AddAxis. ParamSpaceExplorer itself stays descriptor-agnostic.
//
// To add a new L-System asset descriptor with full explorer support:
//   1. inherit publicly from ILSystemExplorableDescriptor;
//   2. override RegisterExplorableAxes to list every field you want exposed;
//   3. own a `ParamSpaceExplorer explorer_` member (or share one externally);
//   4. in OnInspect call:
//          if (!explorer_.IsBound()) explorer_.Bind(*this);
//          explorer_.OnInspect();

namespace l_system_package {

class ParamSpaceExplorer;

class ILSystemExplorableDescriptor {
 public:
  virtual ~ILSystemExplorableDescriptor() = default;

  /// Populate the explorer's axis list. Called by ParamSpaceExplorer::Bind /
  /// RebuildAxes after the explorer has cleared its axes_ container. The
  /// implementation should call explorer.AddSingle/AddPlotted/AddCurve/AddAxis
  /// for every parameter that should be tweakable from the panel.
  virtual void RegisterExplorableAxes(ParamSpaceExplorer& explorer) = 0;

  /// Optional hook fired whenever the explorer mutates a parameter. Default
  /// no-op; descriptors needing live re-instantiation can override.
  virtual void OnExplorerParameterChanged() {
  }

  /// Cheap fingerprint of the descriptor's *structural* shape (e.g. dynamic
  /// array sizes such as tropism count). Used by ParamSpaceExplorer::OnInspect
  /// to detect when axes_ became stale and an automatic RebuildAxes is needed.
  /// Implementations must change this value whenever RegisterExplorableAxes
  /// would emit a different number/order of axes. Default = 0 disables
  /// auto-detection (the user-facing "Rebuild Axis Schema" button still works).
  virtual uint64_t ExplorableSchemaFingerprint() const {
    return 0;
  }
};

}  // namespace l_system_package