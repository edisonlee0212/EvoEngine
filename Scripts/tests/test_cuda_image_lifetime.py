"""Source contracts for CUDA interop teardown; actual OptiX reference checks exercise the rebuilt service."""
from pathlib import Path
import unittest


class CudaImageLifetimeTest(unittest.TestCase):
    def test_cleanup_uses_created_handles_and_cannot_throw(self):
        root = Path(__file__).resolve().parents[2] / "EvoEngine_Services/CudaModule"
        source = (root / "src/VulkanInterlop.cpp").read_text(encoding="utf-8")
        header = (root / "include/RayTracer/VulkanInterlop.hpp").read_text(encoding="utf-8")
        self.assertIn("for (const auto surface : surface_objects)", source)
        self.assertNotIn("mipmap_levels", source + header)
        self.assertNotIn("CUDA_CHECK(", source)
        self.assertLess(source.index("DestroySurfaceObject"), source.index("FreeMipmappedArray"))
        self.assertLess(source.index("FreeMipmappedArray"), source.index("DestroyExternalMemory"))


if __name__ == "__main__":
    unittest.main()
