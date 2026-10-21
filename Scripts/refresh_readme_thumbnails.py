"""Create README WebP previews from engine captures. Requires Pillow.

Run from any directory after refreshing the full-resolution captures under
Resources/GitHub. The original PNGs remain available through README links.
"""

from pathlib import Path

from PIL import Image, ImageOps


SOURCES = {
    "Editor": "GitHub/RenderingDemo.png",
    "Sponza": "GitHub/SponzaDemo.png",
    "SSS": "GitHub/SubsurfaceScatteringDemo.png",
    "Bistro": "GitHub/BistroDemo.png",
    "Bicycle": "Launcher/DemoPreviews/bicycle.png",
    "Galaxy": "GitHub/GalaxyDemo.png",
    "EcoSysLab": "GitHub/TreeFrameworkDemo.png",
    "Strands": "GitHub/StrandVisualization.png",
    "Fracture": "GitHub/TreeFracture.png",
    "CropLighting": "GitHub/SorghumEnvLighting.png",
    "Agriculture": "GitHub/IlluminationEstimationDemo.png",
}


def main():
    resources = Path(__file__).resolve().parents[1] / "Resources"
    output = resources / "GitHub/Thumbnails"
    output.mkdir(parents=True, exist_ok=True)
    for name, relative in SOURCES.items():
        size = (1280, 720) if name == "Editor" else (640, 360)
        with Image.open(resources / relative) as source:
            image = ImageOps.pad(source.convert("RGB"), size, method=Image.Resampling.LANCZOS, color="#151515")
            destination = output / f"{name}.webp"
            image.save(destination, quality=85, method=6)
            print(f"{destination.relative_to(resources.parent)}: {destination.stat().st_size} bytes")


if __name__ == "__main__":
    main()
