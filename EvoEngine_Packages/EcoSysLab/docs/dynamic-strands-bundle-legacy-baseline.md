# Dynamic-strands bundle legacy baseline

Captured on 2026-08-31 from `codex/ssr-stabilization` at pre-change HEAD `1ef5876d`, using the
RelWithDebInfo installed editor and `Resources/EcoSysLabProject/test.eveproj`. The EcoSysLab runtime package was loaded,
the profiler was enabled, and each report used the most recent 120 post-warmup frames.

The experiment controls retained their default values. The board used rod dimensions `20 x 40 x 20`; the log used rod
size `800` and `20` segments. Both cases used solver mode `Legacy` (`solver_mode: 0`).

| Metric | Board | Large log |
| --- | ---: | ---: |
| Segments | 18,215 | 18,215 |
| Bundle pairs | 552,601 | 180,707 |
| Constraint RMS | 1.7027181e-5 | 1.7000497e-5 |
| Dispatches per projection | 7 | 7 |
| Strand buffer bytes | 98,746,480 | 98,746,480 |
| Legacy bundle GPU median | 14.253184 ms | 5.817872 ms |
| Legacy bundle GPU p95 | 17.8242032 ms | 8.4891808 ms |

The board rigid-fit quaternion was `[1.7635954e-5, -6.137684e-6, -4.45751e-4, 1]` in XYZW order, with center
translation `[-2.6226044e-6, -6.105304e-4, 1.1408702e-6]` and far-side displacement
`[4.976988e-5, -5.686283e-4, 8.9779496e-7]`.

The log rigid-fit quaternion was `[-9.766796e-6, 2.6936866e-6, -4.550385e-4, 1]` in XYZW order, with center
translation `[-3.4570694e-6, -5.941391e-4, 4.656613e-10]` and far-side displacement
`[-3.5643578e-5, -6.507635e-4, -1.7043203e-7]`.

To reproduce the capture, build and install `EcoSysLabPackage`, open the project above, load EcoSysLab from the Runtime
Package Manager if it is not already loaded, enable View > Profiler > Capture, initialize the board or log experiment,
allow at least 120 frames after warmup, then click `Capture bundle diagnostics`. The raw YAML reports are written to
`Resources/EcoSysLabProject/Diagnostics/board-bundle-diagnostics.yaml` and
`Resources/EcoSysLabProject/Diagnostics/log-bundle-diagnostics.yaml`.
