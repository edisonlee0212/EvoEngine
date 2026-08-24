# EvoEngine Profiler

[Back to README](../README.md)

The editor profiler presents frame-aligned CPU and GPU timing. Open it from **View > Profiler**. The window and capture
are disabled at startup, so ordinary editor runs do not record CPU events or write Vulkan timestamp queries.

## Capture controls

- **Capture** starts or stops CPU events and GPU timestamps together. Capture remains active if the profiler window is
  closed and continues until it is explicitly stopped.
- **Pause** freezes the panel selection while capture continues. **Pause on next frame** freezes after the next complete
  captured frame. Neither action stops recording.
- Turning **Capture** off stops both recorders and retains the current history for inspection.
- Turning it on again appends a new capture session. CPU and GPU plots leave a discontinuity between sessions.
- **Clear** removes retained CPU and GPU history without changing the capture toggle.
- **History** bounds the retained CPU window from 30 to 2000 frames. GPU history is independently capped at 2000
  originating application frames.

If GPU timestamp queries are unavailable, Capture still records CPU data and the panel reports the GPU limitation.

Global capture controls stay above five views. **Overview** contains aligned CPU, synchronization and GPU history;
**Breakdown** contains rolling CPU/GPU budgets; **CPU** and **GPU** contain detailed timing views; and **Diagnostics**
contains pinned-frame CPU/GPU timelines, stable detail tables and trace export.

## Frame overview

The Overview separates main-thread CPU activity from explicitly instrumented synchronization. Synchronization includes
frame/resource fence waits, GPU-service waits, swapchain image acquisition and presentation. It does not infer mutex
contention, asset-future waits or task-runtime condition waits. Overlapping or nested synchronization scopes are unioned.

For each frame, `CPU active = CPU wall - synchronization`, while `Total = max(CPU wall, GPU span)`. CPU active and
synchronization are stacked because they partition CPU wall time. GPU span and Total are lines because GPU execution
overlaps CPU execution. The average table uses only CPU frames with resolved, frame-aligned GPU snapshots and reports the
paired-frame count.

## CPU timing

The main-thread hierarchy starts at `Application Loop` and separates Pre Update, Fixed Update, Update, and Late Update.
Invoked layers appear beneath their lifecycle phase by stable layer name. Fixed Update is displayed as a lifecycle sibling
even though the runtime currently invokes it from the pre-update implementation. Worker-thread work remains outside
main-thread wall time. The UI groups physical lanes under stable Main Thread, Worker, Asset I/O, GPU Submission (CPU),
Render, Background and Other executor roots without inventing cross-thread parent relationships.

| Value | Meaning |
| --- | --- |
| Inclusive | Time in the scope, including synchronous child scopes on the same thread. |
| Self | Inclusive time minus synchronous child time. |
| Calls | Occurrences aggregated into this hierarchy node. |
| Parent % | Inclusive time divided by the parent node's inclusive time. |
| Frame % | Inclusive time divided by the selected application-loop duration. |
| Average / Max / P95 | Statistics for the matching hierarchy path over the selected history window. |

Threads and hierarchy paths are registered in first-seen order and remain in that position until **Clear**. A registered
row absent from the selected frame displays zero current values, while its historical statistics count missing
frames as zero. Rows that have aged completely out of the rolling history remain available but show zero history values.
Filtering, pausing, changing the selected frame and reducing history length do not reorder or discard registered rows.
Tree expansion uses logical executor and full hierarchy-path identities, so rolling refreshes do not collapse nodes.

Repeated sibling names are aggregated. Recursive scopes remain separate by hierarchy depth. A scope that begins in one
frame and finishes later remains associated with its originating frame. The panel reports dropped CPU events instead of
presenting a capacity-limited frame as complete. Chrome trace export contains CPU tracks only.

## GPU timing

GPU durations come from paired `vkCmdWriteTimestamp2` queries at `VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT`, converted with
the physical device's `timestampPeriod` and graphics-queue valid-bit width. Results are resolved when the originating
frame slot is recycled, so GPU data normally trails CPU capture by the frames-in-flight delay. The live panel selects the
newest CPU frame with a resolved matching GPU snapshot.

The table is grouped as render group, logical pass, then instance. A logical pass can contain repeated cameras, reflection
probe faces, shadow cascades or light faces. Expand it to inspect view, instance and queue identity. Logical-pass rows show
total duration, percentage of their group, percentage of summed additive work, call count, average, median, maximum and
p95. Non-additive summary scopes remain inspectable but are excluded from summed work and stacked composition.

GPU groups and logical passes use the same first-seen, retain-until-Clear behavior as CPU rows. Missing passes count as
zero only across frames whose timestamp results resolved successfully; delayed or unavailable GPU frames are excluded.
Per-view and per-instance children describe occurrences in the selected frame and therefore are not retained when that
logical pass is absent.

Two totals intentionally answer different questions:

- **GPU span** is device-clock time from the first recorded scope begin to the latest recorded scope end. It can include
  gaps, overlap and uninstrumented intervals between recorded scopes, but not work before the first or after the last
  timestamp.
- **Summed pass work** adds leaf scopes marked as additive. It can exceed GPU span when queues or nested work overlap and
  is not a substitute for end-to-end frame latency.

The line plot shows a filtered top-N set plus pinned passes, with 16.7 ms and 33.3 ms budget guides. Any captured pass can
be pinned or hidden. The stacked plot aggregates unselected additive passes into `Other` for display only; raw table rows
and samples are preserved. Series and legend ordering, labels and colors use the logical pass's first-seen identity until
Clear. Automatic top-N membership still follows rolling average cost, so an expensive pass can enter or leave the plots
without changing the identity or ordering of registered rows. Missing results are gaps, and capture sessions are never
connected by a continuous line.

The CPU tab provides the same top-N, filter, pin/hide and hover workflow for inclusive scope histories. Its stacked plot
is limited to the non-overlapping direct children of `Application::Loop`, loop self time and frame overhead. Nested scopes
and parallel worker work remain line series and are not added into the phase stack.

## Diagnostics

Diagnostics owns a selected-frame index independent of the live tabs. It defaults to a pinned frame and provides
Previous, Next, Latest and opt-in Follow latest controls, keeping its tables stable while capture continues.

The CPU timeline uses offsets relative to the CPU frame. The GPU timeline uses actual timestamp samples grouped into
Graphics, Compute, Transfer, Ray Tracing and Immediate queue lanes, with overlapping samples packed into sub-lanes. GPU
Submission (CPU) identifies the engine's CPU executor and is not a device lane. The timelines remain separate because
their clocks are not calibrated and their horizontal positions cannot be compared.

## Capacity and limitations

Each frame-slot query pool contains 256 queries, allowing at most 128 paired scope occurrences per originating frame.
The panel displays query use, skipped scopes and unresolved results. When capacity is exhausted, remaining scopes are
skipped and the frame is explicitly marked incomplete. Query pools are reset and timestamp writes are omitted when
capture is disabled.

Raster render-graph passes, raster cameras, shadows, dynamic reflection probes, AO/DDGI, post-processing, editor/UI work,
and existing ray-tracing timestamp scopes use the generic table. Complete ray-tracing instrumentation is not claimed.
External renderer callbacks are only visible when their owners add profiler metadata and timestamp scopes. GPU timestamps
are device-clock measurements and are not exported onto CPU Chrome-trace tracks because no calibrated CPU/GPU clock
mapping is currently recorded.

## Validation snapshot

On 2026-08-20, the installed Rendering/Sponza demo was measured on an AMD Ryzen 7 9800X3D and NVIDIA GeForce RTX 5070
after a fixed warmup, with the profiler panel hidden in both cases. Three overlay samples averaged 21.47 FPS with capture
disabled (21.4, 21.6, 21.4) and 21.33 FPS with unified CPU/GPU capture enabled (21.7, 21.3, 21.0), a 0.62% reduction. This
meets the less-than-2% target and is below the 10% acceptance ceiling; it is a representative interactive measurement,
not a portable hardware guarantee.

Timestamp conversion, valid-bit wrap, delayed availability, repeated instances, ordering, capacity metadata and malformed
results are covered by GPU timestamp unit tests. Render-graph tests verify timestamp scopes enclose their named command
recording boundaries. Nsight Graphics, PIX and RenderDoc were not installed on the validation machine, so no external
debugger cross-capture is claimed. The device-clock queries and deterministic pure-data conversion tests are the timing
evidence for this checkpoint; an external debugger comparison remains a useful follow-up when one is available.

The installed launcher opened the Rendering demo, both raster and ray-traced Sponza views remained active, and the editor
stayed responsive for more than 30 seconds without a new Windows application crash event. Capture persistence while the
window was hidden, stop/freeze, restart into a new session, and retained-history behavior were also exercised manually.

## Retain and follow up

- Retain startup-disabled unified capture; its disabled GPU-query cost is structurally zero.
- Retain the 256-query capacity and 2000-frame cap. Current validation produced no capacity warning; raise capacity only
  from a measured incomplete workload because every frame slot owns its query pool.
- Retain the top-N/pinned line plot and stacked composition; they expose regressions without hiding raw samples.
- Keep GPU Chrome-trace export disabled until calibrated CPU/GPU timestamps can place GPU events accurately.
- Re-run the fixed-warmup overhead sample and an external debugger comparison after major render-pass or queue changes.
