import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_sdk_shader_policy import collect_sources


class SdkShaderInventoryTest(unittest.TestCase):
    def test_inventory_includes_editor_and_runtime_resource_roots(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for base, relative in (("EvoEngine_SDK", "Modules/EvoEngine/Runtime.slang"),
                                   ("EvoEngine_SDK/Editor", "Graphics/Vertex/Gizmos/Widget.slang")):
                path = root / base / "Internals/DefaultResources/Shaders" / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("// shader fixture\n")
            records = collect_sources(root)
            self.assertEqual({record["module"] for record in records},
                             {"EvoEngine.Runtime", "EvoEngine.EntryPoints.Graphics.Vertex.Gizmos.Widget"})
            self.assertEqual(len(records), 2)
