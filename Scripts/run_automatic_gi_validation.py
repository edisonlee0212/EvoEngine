"""Focused installed 1440p Sponza validation for automatic GI cascades."""

import argparse
import os
from pathlib import Path
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--provider", choices=("sdfgi", "ddgi"), required=True)
    parser.add_argument("--resources", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=root / "tasks/automatic-gi-validation")
    parser.add_argument("--install", type=Path, default=root / "out/install/vs2026-x64")
    args = parser.parse_args()
    resources, output = args.resources.resolve(), args.output.resolve()
    module_dir = args.install.resolve() / "python"
    output.mkdir(parents=True, exist_ok=True)
    os.chdir(module_dir)
    sys.path.insert(0, str(module_dir))
    import PyEvoEngine as engine

    rt = args.provider == "ddgi"

    def advance_ready():
        for _ in range(1200):
            assert engine.Loop()
            if not engine.IsCurrentSceneReadyForCapture():
                continue
            if rt:
                state = engine.GetCurrentSceneDdgiHistoryStatus()
                expected = engine.GetCurrentSceneGiProbeSettings().cascade_count
                if len(state) == expected and all(c["ready"] for c in state):
                    return state
            else:
                state = engine.GetCurrentSceneGiStatus()
                if state["accepted_contributor_count"] > 0 and state["published"] and state["transport_pass"] >= 35:
                    return state
        raise RuntimeError(f"GI did not become ready: {state}")

    def capture(label):
        assert engine.CaptureCurrentScene(2560, 1440, output / f"{args.provider}-{label}.png", 1)

    try:
        assert engine.RunDemoWindowless("Rendering", resources, True, rt)
        assert engine.ConfigureCurrentSceneCameraForCapture("Rasterization", 1, 1)
        engine.ResizeCurrentSceneCameraForCapture(2560, 1440)
        settings = engine.GetCurrentSceneGiSettings()
        settings.provider = (engine.IndirectGiProvider.AutomaticDdgi if rt
                             else engine.IndirectGiProvider.AutomaticSdfgi)
        engine.SetCurrentSceneGiSettings(settings)
        report = engine.SdfgiCapabilityReport()
        assert all(report[k] == rt for k in ("ray_tracing_enabled", "ray_query_enabled", "blas_enabled", "tlas_enabled"))
        initial = advance_ready()
        print("Initial:", initial, report, flush=True)
        capture("initial")

        # Reject a complete candidate without publishing any of its changed fields.
        invalid = engine.GetCurrentSceneGiSettings()
        invalid.probes.probe_count_x = 257
        invalid.probes.probe_count_y = 257
        invalid.probes.cascade_count = 8
        try:
            engine.SetCurrentSceneGiSettings(invalid)
            raise AssertionError("Unsupported allocation accepted")
        except ValueError:
            pass
        accepted = engine.GetCurrentSceneGiSettings()
        assert accepted.probes.probe_count_x == settings.probes.probe_count_x
        assert accepted.probes.probe_count_y == settings.probes.probe_count_y
        assert accepted.probes.cascade_count == settings.probes.cascade_count
        assert accepted.provider == settings.provider

        position = engine.GetCurrentSceneCameraPositionForCapture()
        identities = [c["cascade_id"] for c in initial] if rt else None
        for label, dx in (("positive", 0.9), ("negative", -0.9), ("teleport", 40.0), ("restored", 0.0)):
            engine.SetCurrentSceneCameraPositionForCapture(position[0] + dx, position[1], position[2])
            for _ in range(3):
                assert engine.Loop()
            state = advance_ready()
            if rt:
                assert [c["cascade_id"] for c in state] == identities
            print(label, state, flush=True)

        edited = engine.GetCurrentSceneGiSettings()
        edited.probes.probe_count_x = 25
        engine.SetCurrentSceneGiSettings(edited)
        advance_ready()
        engine.SetCurrentSceneGiSettings(settings)
        advance_ready()

        if rt:
            before = engine.GetCurrentSceneDdgiHistoryStatus()
            engine.SetCurrentSceneDdgiVisibilitySmoothing(0.5)
            assert engine.Loop()
            after = engine.GetCurrentSceneDdgiHistoryStatus()
            assert [c["resource_ids"] for c in before] == [c["resource_ids"] for c in after]
            engine.SetCurrentSceneDdgiVisibilitySmoothing(0.9)
            alternate = engine.GetCurrentSceneGiSettings()
            alternate.provider = engine.IndirectGiProvider.AutomaticSdfgi
            engine.SetCurrentSceneGiSettings(alternate)
            for _ in range(180):
                assert engine.Loop()
                alternate_state = engine.GetCurrentSceneGiStatus()
                if alternate_state["published"] and alternate_state["transport_pass"] >= 35:
                    break
            assert alternate_state["published"] and alternate_state["transport_pass"] >= 35
            assert not engine.GetCurrentSceneDdgiHistoryStatus()
            engine.SetCurrentSceneGiSettings(settings)
            advance_ready()
            print("Direct DDGI/SDFGI/DDGI switch passed", flush=True)
        else:
            edited = engine.GetCurrentSceneGiSettings()
            edited.sdfgi.probe_spacing_cells = 8
            engine.SetCurrentSceneGiSettings(edited)
            advance_ready()
            engine.SetCurrentSceneGiSettings(settings)
            advance_ready()
            rejected = engine.GetCurrentSceneGiSettings()
            rejected.provider = engine.IndirectGiProvider.AutomaticDdgi
            try:
                engine.SetCurrentSceneGiSettings(rejected)
                raise AssertionError("RT-disabled DDGI accepted")
            except ValueError:
                pass
            assert engine.GetCurrentSceneGiSettings().provider == settings.provider

        disabled = engine.GetCurrentSceneGiSettings()
        disabled.provider = engine.IndirectGiProvider.Environment
        engine.SetCurrentSceneGiSettings(disabled)
        assert engine.Loop()
        assert not engine.GetCurrentSceneDdgiHistoryStatus()
        engine.SetCurrentSceneGiSettings(settings)
        advance_ready()
        capture("restored")
        print("PASS: shared layout edits, signed scroll, teleport, atomic rejection, provider lifecycle and finite captures", flush=True)
    finally:
        engine.Terminate()


if __name__ == "__main__":
    main()
