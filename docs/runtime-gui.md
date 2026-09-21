# Runtime GUI

`RuntimeGui` draws GUI over the displayed main camera in the stopped editor, Play/Pause, and standalone builds. It never draws in the Scene Camera, previews, or headless applications.

## Setup and layout

1. Add `RuntimeGui` to any entity and assign its `Camera` reference to the scene's main camera. The component, camera, and their entities must be enabled.
2. Add GUI assets to its ordered list. `RuntimeDebugGui` (`.everuntimegui`) provides timing, a time-step control, texture/camera previews, and a resizable Images panel.
3. Arrange windows while stopped, then save the scene. Window positions, sizes, collapse state, and resizable child sizes belong to the component.

Play copies the authored layout; Play/Pause layout edits are discarded on Stop. Duplicates and prefab instances copy layouts and then remain independent. Shared GUI assets share their fields, but each component has its own windows. Layouts use camera-relative positions and readable logical sizes; they do not enter the editor's `imgui.ini`.

Standalone builds retain layout overrides in `UserData/RuntimeGuiLayouts.json`. Relaunching the same export restores them; a new export starts from its newly exported layouts. Automatic overrides currently cover authored startup-scene components. Dynamically created components remain session-only. Launches without `runtime_gui_layout_revision` use serialized layouts without cross-launch overrides.

## Writing a GUI asset

Derive from `IRuntimeGui`, register the concrete type/extension and its serialization handler, and implement `OnGui`:

```cpp
void DebugControls::OnGui(RuntimeGuiContext& context) {
  if (context.BeginWindow("Debug")) {
    ImGui::TextUnformatted("Ordinary ImGui widgets");
    if (context.BeginChild("Details", {0, 120},
                           ImGuiChildFlags_Borders | ImGuiChildFlags_ResizeY)) {
      ImGui::TextUnformatted("Resizable child content");
    }
    context.EndChild();
  }
  context.EndWindow();
}
```

- Always pair `BeginWindow`/`BeginChild` with their End methods, even when Begin returns false. Give children stable, distinct IDs; helper child IDs are independent of `ImGui::PushID`. A title's `###` suffix supplies its stable identity.
- Use the context's popup, modal, menu, and tooltip helpers. End those only when Begin succeeds. Ordinary widgets and `ImGui::CloseCurrentPopup` remain native. Shared-context modals can block editor interaction.
- `GetComponent()`, `GetScene()`, `GetCamera()`, `GetOrigin()`, and `GetSize()` describe the current callback. Do not retain the context. Callbacks also run while stopped; check execution status for simulation-only controls.
- Serialize persistent asset fields and collect asset dependencies through `CollectAssetRef`. Authors manage transient widget state. Do not create platform windows or dock into editor panels.

Components run by `draw_order`, then entity handle; assets run in list order. Duplicate asset references within one component are rejected. Missing targets and missing/wrong-type assets draw nothing without removing the references.

## Images, drawing, and input

Use `GuiTextureRegistry::GetTextureId(Texture2D)` or `GetColorTextureId(RenderTexture)` with ordinary `ImGui::Image`. Request IDs during the frame; resources remain retained through GPU submission. Engine images use UVs `{0,1}` to `{1,0}`. Specialized depth/cubemap inspectors are not included.

For camera annotations, use the context's background/foreground lists:

```cpp
const auto origin = context.GetOrigin();
context.GetForegroundDrawList()->AddText(
    {origin.x + 16, origin.y + 16}, IM_COL32_WHITE, "Camera annotation");
```

Background draws behind GUI windows; foreground draws above them. Both are clipped to the camera, rebuilt each frame, and provide no input capture or layout persistence. Coordinates are logical screen coordinates; add `GetOrigin()` for camera-relative offsets. Do not retain these lists or use native viewport background/foreground lists. Inside a GUI window, ordinary `ImGui::GetWindowDrawList()` is supported.

One shared ImGui context renders a separate transparent overlay. Camera texture captures exclude GUI; displayed screenshots include it. GUI capture filters gameplay input while preserving raw backend events, release cleanup, and application shortcuts.

Standard bootstraps add `ImGuiLayer` then `RuntimeGuiLayer`. Custom graphical hosts must add both. See [runtime builds](runtime-builds.md) for export and installation.
