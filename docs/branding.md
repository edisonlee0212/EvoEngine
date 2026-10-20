# EvoEngine logo assets

EvoEngine uses the rounded hexagon with a branching trunk and seven open interior shapes. The white logo has a transparent background and softly antialiased alpha edges.

- `EvoEngine_SDK/Internals/DefaultResources/Icons/EvoEngine.png` is the 1024px white master and default exported-runtime startup logo.
- `Icons/EvoEngineWhite<size>.png` provides white versions at 32, 48, 64, 96, 128, 192, 256, 384, 512, 768 and 1024px.
- `Icons/EvoEngine16.png`, `EvoEngine24.png`, `EvoEngine32.png` and `EvoEngine64.png` are black variants used for native window icons. The 64px variant is also used on light editor and launcher title bars.
- `EvoEngine_SDK/Editor/Internals/DefaultResources/Editor/TitleBar/EvoEngine64White.png` is the matching white title-bar variant for dark themes.
- `EvoEngine_App/resources/EvoEngineLauncher.ico` embeds white 16, 24, 32, 48, 64, 128 and 256px frames. The editor, launcher, runtime, exporter and registered application resource scripts all reference this shared icon. `EvoEngineEditor.ico` and `EvoEngineApp.ico` retain matching copies for direct use.

All variants preserve the same aspect ratio and transparent padding. Keep white/black counterparts' alpha channels identical when updating them. PNG transparency includes the seven interior openings; no background color is baked into the images.

New runtime exports include these default resources automatically. A project's explicitly selected custom startup logo continues to take precedence. Existing exported distributions retain their packaged assets until exported again. See [runtime builds](runtime-builds.md#startup-screen).
