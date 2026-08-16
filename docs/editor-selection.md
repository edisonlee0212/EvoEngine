# Editor entity selection

The editor keeps entity selection as transient, scene-scoped state. A selection contains an ordered set of entities, a
primary entity, and an optional range anchor. The primary is the most recently activated exact selection and remains the
compatibility target returned to integrations that use the single-entity API.

Selection membership, the primary entity, the range anchor, and the selection lock are not serialized. Loading a
project or replacing the active scene clears all of them. Existing integrations may continue to call
`EditorLayer::GetSelectedEntity` and `SetSelectedEntity`; those APIs read or replace the primary selection.

## Entity Explorer

- A plain click in Archetype mode, or a plain double-click in Hierarchy mode, replaces the selection.
- Ctrl+click toggles the clicked entity without clearing the other selected entities.
- Shift+click replaces the selection with the inclusive range between the anchor and the clicked row.
- Ctrl+Shift+click adds that inclusive range to the selection.
- Only rows visible in the current Explorer display mode participate in a range. Changing display mode clears the range
  anchor.
- Right-click opens the clicked entity's context menu without changing selection.

The last clicked entity becomes primary. If the primary is removed, the most recently activated remaining entity is
promoted. Lock rejects user selection changes; programmatic compatibility calls and lifecycle cleanup remain available.

## Viewports and deletion

A plain viewport click preserves the existing parent-cycling behavior. Ctrl+click instead adds the entity directly under
the pointer without cycling; an entity already in the selection is unchanged. Ctrl+clicking empty space preserves the
current selection. Escape clears the selection.

Delete removes every selected entity when either the Scene viewport or Entity Explorer is focused. Selected descendants
of another selected entity are omitted from the delete request because deleting their selected ancestor already removes
them. Delete is ignored while an ImGui text field is accepting input.

## Highlighting

`EntitySelectionHighlight` is owned by `EditorLayer` and supplies immutable presentation state to the renderer. The
scene-camera render graph runs `EntitySelectionHighlightPass` after ordinary post-processing. The pass reads the utility
G-buffer selection mask, draws a three-pixel orange outside outline, and applies the existing 50 percent background focus
fade. Selection roots and their descendants contribute to the mask through an explicit per-frame coverage set; selection
is not stored in ECS metadata and selection-only changes do not invalidate camera render history.

The Focus checkbox enables or disables this presentation component. The fade starts when an empty selection becomes
non-empty or when Focus is re-enabled; changing membership within a non-empty selection does not restart it. Clearing the
selection removes the effect immediately. Reflection-probe captures and non-scene cameras do not run the pass.

The highlight pass is skipped when no selected entity or selected descendant contributes a render instance. Transparent-only
and Gaussian-splat-only entities are not highlighted when they do not contribute identity to the raster
utility G-buffer. Supporting those cases requires a separate selected-geometry mask.

## Entity Inspector

The Inspector edits the exact selected entities; descendants covered only by highlighting are not Inspector targets.
Enabled/static state and local Transform fields support mixed values. Entity renaming remains available from the entity
context menu rather than the Inspector. With multiple targets, dragging a Transform axis applies a relative change from
the drag-start values: Translation and Rotation use additive deltas, while Scale uses a ratio (or an additive fallback
when the representative scale starts at zero). Directly typing a value remains absolute. Every edit preserves untouched
axes.

Single and multiple selection use the same component layout. The Data Components and Private Components sections and every
common component use the same full-width, default-open collapsing-header style. Their bodies do not add nested tree
indentation; expanded inspector content receives only a small visual inset and vertical spacing. The two section headers
have Gear menus with an `Add Component` submenu containing their available Add actions. Data Components uses the
four-square data icon, while Private Components uses the generic private-component icon. Registered icons override the
matching category fallback on individual component headers. The Data Components section uses a lighter blue header and its
entries use stronger blue headers; the Private Components section uses lighter brown and its entries use stronger brown.
Private-component reference slot buttons share the lighter brown palette. Each palette has light- and dark-theme variants
with matching hover/active states. Their Gear button and header right-click menu contain the
selection-aware removal action, and Transform remains protected. Private-component Enabled state also lives in the header
and supports mixed multi-selection values. Inspector content and private-component drag-and-drop are available only while
the header is expanded. With one target, the normal component inspector is preferred,
then a batch inspector may accept the one-element target set as a fallback. With multiple targets, only explicit batch
inspectors are used; unsupported common components remain visible with `Multi-object editing not supported`. Transform has
one batch inspector for every selection count. Its Hazel-style Translation, Rotation, and Scale rows use colored X/Y/Z
reset buttons and compact numeric fields. Axis resets write zero for Translation/Rotation and one for Scale. Resets and
typed numeric values remain absolute local per-axis writes for every exact target; mouse drags use the relative behavior
described above. `GlobalTransform` and `TransformUpdateFlag` are internal and hidden.

Only component types common to every exact target are expanded for multiple selection. A notice reports non-common types
that are hidden. Common private-component enabled state can be changed for all targets.

The Inspector labels structural operations with their scope. Add is opened from the Data Components or Private Components
section Gear menu's `Add Component` submenu, applies only to selected entities missing that component, and displays
`Add to N/M`; remove is available only for a common removable type and removes it from all exact targets.
Transform and internal transform bookkeeping cannot be removed. Unknown data/private placeholder types are never offered
by the Add menus. Entity Explorer context-menu operations remain scoped to the row that opened the menu.

## Multi-entity gizmos

The Scene viewport owns the transform-operation toolbar. Its 23-pixel Select, Translate, Rotate, and Scale buttons use
Hazel's editor icons; Translate is the default. Select is represented by all three legacy operation-selection layout keys
being false and suppresses entity and environmental transform manipulators. The Transform component no longer changes the
active viewport tool. The strip sits immediately left of the equally sized playback strip, while the primary
Play/Pause/Resume button remains exactly centered in the viewport.

The 96-pixel camera View gizmo is always visible at the Scene child's upper-left corner, including in Select mode and when
ordinary scene gizmos are disabled. The top-right Gear popup is the exclusive UI for Scene-camera information visibility,
navigation speed, transform/default actions, resolution, and Camera settings. It has independent padding, is right-aligned,
and scrolls only when its viewport-constrained height requires it. Scene and Main Camera information overlays have padded,
content-sized heights; the Scene overlay remains offset below the Gear control instead of covering it. The tool strip, View
gizmo, gear button, and popup consume their own mouse input and do not trigger entity picking.

Gizmos manipulate only top-level exact selections, filtering out an entity when one of its selected ancestors already
participates. This prevents a selected child from moving twice. If the exact primary is filtered, its nearest participating
selected ancestor supplies the handle; the exact primary used by APIs does not change. Handle placement uses the union of
enabled mesh, skinned-mesh, particle, and strand renderer bounds in each participant subtree. Meshless selections fall back
to participant world positions.

For multiple selections, Pivot and Center modes both place the handle at the shared renderable-bounds center. Pivot mode
preserves individual authored origins for rotation and scale, while Center mode rotates and scales the selection around that
point. A single selection uses its renderable-bounds center. Local uses the reference orientation for the displayed handle;
Global uses world axes. Center+Local is the default, and both mode preferences are stored in editor layout rather than scene
data. Existing layouts migrate to Center once, after which an explicit Pivot choice is preserved.
Focus, gizmo visibility, Pivot/Center, and Local/Global preferences are edited on the `Entity Inspector` tab of the
EditorLayer inspection window. Editor-camera input mappings are edited on its dedicated `Key Bindings` tab. Bindings display
readable keyboard and mouse-button names; click a binding and press the replacement input, or press Escape to cancel capture.
Selection Lock and Clear remain beside the entity enabled/static controls.

The Scene-camera `Focus Selection` binding defaults to F. While the Scene viewport is focused, it frames the same shared
renderable bounds used by the gizmo, retains the current viewing side, rotates toward the bounds center, and moves through
the editor camera's smooth transition. The framing distance accounts for the current projection and a small margin. The
shortcut is ignored while typing or dragging an entity gizmo, does not depend on Selection Lock, and has no effect on the
Main Camera viewport. Its target and interpolated rotations are rebuilt against world Y, matching regular Scene-camera
controls and preventing roll throughout the focus transition.

## Imported model origins

`PrefabModelImportOptions::center_mesh_renderer_origins` is enabled by default for model imports. For each static
`MeshRenderer`, import moves the generated renderer entity to the mesh AABB center and offsets its vertex positions by the
inverse amount. The composed world-space geometry therefore remains unchanged while object rotation and scaling use a
geometry-centered authored origin. Skinned meshes retain their imported vertices and bind poses. API callers can disable
the option to reproduce the legacy origin layout; existing saved prefabs change only when reimported.

A drag snapshots selection, hierarchy, parents, transforms, modes, and play state once, then evaluates each frame from that
snapshot while preserving ImGuizmo's live manipulated handle between frames. Translation applies one common world delta.
Rotation and scale follow the selected Pivot/Center and Local/Global
modes. Every candidate is converted back to local TRS and applied only if all participants are finite, nonsingular, and
representable without shear. A rejected mathematical update leaves the last valid result intact and shows a viewport
message so the same drag can recover. Selection, hierarchy, scene, or play-mode changes cancel the transient drag without
restoring over the external change. This phase does not add general undo/redo support.
