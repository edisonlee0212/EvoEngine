# Ray-camera denoising validation notes

[Current input contract and controls](ray-camera-denoising.md)

These are historical measurements retained from earlier implementation checkpoints, not benchmarks rerun for the
documentation refresh. Accumulated-input behavior and subsequent shader changes can alter both quality and cost.
Keep the stated scene, device, sample budgets and limitations attached to each result; remeasure the current executable
before using these numbers to choose a rendering mode.

## Earlier fresh-input baseline

The earlier fresh-input baseline on a matched 1920×1080 Sponza setup against a 2048-frame Path Tracing reference measured denoised PT64 at 0.02730 RMSE versus raw PT79 at 0.05260; denoised Candidate64 at 0.02922 versus raw PT120 at 0.04228. Enhanced64 measured 0.03437 versus raw PT389 at 0.02175, missing the equal-GPU-time quality gate. These budgets include ray-camera and NRD passes; they do not measure the accumulated PT input now used by default. Bright material-edge residuals, long-history energy differences, and Enhanced's high ray cost remain open. Fresh unaccumulated frames are still noisy.

### Integration validation and limits

At 1920x1080 Sponza on an RTX 5070, regular REBLUR with Enhanced, 64 frames,
one sample per frame and temporal AA produced linear-HDR RMS error 0.05664
against a 4096-sample PT reference. Correcting squared roughness reduced RMS
error by 0.8%; absolute error was essentially unchanged. Translation and rotation
endpoint captures still show noisy reflective surfaces. These captures do not
establish sequence-level ghosting behavior or resolve the distant-noise root cause.

An approximately equal GPU budget comparison measured 41.59 seconds for Enhanced
and 41.96 seconds for raw PT (49 frames, 32 samples per frame). PT RMS error was
0.00945, substantially lower than Enhanced; this static quality gate does not pass.
The PT reference and candidate share a deterministic sample prefix, which favors
PT's reference error; this is a single-scene diagnostic, not a broad quality result.
GPU budgets sum camera pass timestamps and exclude shader compilation and other
frame work. Memory pressure and timing variability limit performance conclusions.

In the corrected Enhanced static run, median guide time was 27.73 ms, NRD was
2.12 ms, and temporal AA plus resolve was 0.24 ms. The receiving probes are
expensive, and preparing guides for every independent estimate duplicates work.
Reducing that cost and improving reflection signal correspondence remain follow-up
work. Both lobe distances are explicitly generated at each valid center guide hit;
per-pixel distance and history-rejection coverage still need dedicated readback.

### PT and ReSTIR with REBLUR

A subsequent 1920x1080 Sponza comparison used REBLUR and NRD Temporal AA in
all modes, 64 frames, and 4096-sample raw PT references at matching final poses.
PT samples per frame were selected separately to match each ReSTIR mode's
measured camera GPU budget (within 3.6%). Temporal Only used 2 samples per
frame; Enhanced used 1. PT used 18 for Temporal's budget, and 22–24 for Enhanced's.

| Camera path | PT RMS reduction versus Temporal Only | PT RMS reduction versus Enhanced |
| --- | ---: | ---: |
| Static | 4.1% | 18.4% |
| Translation, 0.02 units/frame | 17.9% | 35.0% |
| Rotation, 0.2 degrees/frame | 10.6% | 18.4% |

PT also had lower relative absolute error and sphere-grid crop error in every
matched comparison. Reflective mottling was lower with static PT, but all modes
still smeared reflective detail during motion. This favors PT + REBLUR as the
current implementation's baseline for these tested settings. It is one scene and
device, with deterministic sample overlap and endpoint captures; it does not
establish sequence ghosting or general algorithm superiority. Camera GPU cost was
about 478–658 ms/frame, so this is not a real-time quality gate. Denoising residual
error remains significant even with many PT samples.

An equal-sample follow-up used 1 and 2 samples per frame for every mode with the
same 64-frame camera paths and denoising controls. PT had lower full-image RMS
and sphere-grid crop RMS in all six cases. Static results were mixed: Temporal
Only had lower average absolute error, while PT's RMS advantage was only 1–2.4%.
Translation and rotation favored PT in both RMS and average absolute error.

| Camera | Samples/frame | PT RMS | Temporal Only RMS | Enhanced RMS |
| --- | ---: | ---: | ---: | ---: |
| Static | 1 | 0.04936 | 0.05055 | 0.05664 |
| Static | 2 | 0.04768 | 0.04824 | 0.05372 |
| Translation | 1 | 0.05495 | 0.06048 | 0.07596 |
| Translation | 2 | 0.05224 | 0.06035 | 0.07664 |
| Rotation | 1 | 0.05775 | 0.06174 | 0.06557 |
| Rotation | 2 | 0.05565 | 0.05997 | 0.06459 |

PT measured about 37/64 ms of camera GPU work per frame at 1/2 samples. Temporal
Only measured 167–175/478–496 ms; Enhanced measured 595–658/1486–1704 ms.
Equal samples are independent estimates, rather than equal traced-ray counts;
reuse/replay adds work. Nonlinear scaling and memory pressure limit general
performance conclusions. Reflective detail remains noisy or smeared in all modes;
the deterministic, single-scene endpoint comparison does not establish sequence
quality or statistical significance for the small static differences.

The camera backend matters in live comparisons: PT follows the Ray Tracing or
Ray Query selection, while every ReSTIR mode uses Ray Query. A 1080p Sponza
1-SPP/64-frame reproduction with NRD off measured PT at 73.0 ms with Ray Tracing,
31.4 ms with Ray Query, and Temporal Only at 56.2 ms. Thus Temporal can outrun PT
when PT uses the Ray Tracing backend; it did not beat PT Ray Query in that test.
With REBLUR on, PT Ray Tracing measured 77.3 ms versus Temporal's 130.4 ms.
These are camera GPU times, not the editor's rolling whole-frame FPS.

Temporal currently selects separate scene-specialized raw combine/resolve shaders
only with NRD off. NRD-on uses generic combine/resolve pipelines; in this run,
combine cost 52.4 ms versus 1.7 ms raw, while NRD dispatch itself cost 2.2 ms.
Reducing NRD Temporal shader overhead remains performance work. Scene variants
are queued after general pipelines become available, explaining compilation and
performance changes after a mode switch.

The SER selector is additionally restricted by the camera pass: SER is disabled
when linear swept sphere (LSS) support is enabled. LSS currently follows device
capability rather than scene strand presence. On the RTX 5070, requested SER
Disabled, Enabled, and Automatic therefore all execute without reordering,
even in triangle-only Sponza. The inspector does not currently explain this
restriction. Repeated raw PT captures measured Ray Tracing at 70–73 ms versus
Ray Query at about 31 ms; active SER cannot explain that gap. Shader register,
spill, occupancy, and scheduling attribution remains unmeasured.
