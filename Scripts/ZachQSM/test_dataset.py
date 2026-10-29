import csv
from pathlib import Path
import tempfile
import unittest
import subprocess
import sys

import numpy as np
import manifold3d as m3d
import yaml

from dataset import read_skeleton, solid_from_cylinders, closed_exterior


class DatasetTests(unittest.TestCase):
    def test_comparison_ignores_gallery_and_requires_native_files(self):
        script = Path(__file__).resolve().parent / "verify_dataset.py"
        with tempfile.TemporaryDirectory() as temporary:
            roots = [Path(temporary) / name for name in ["baseline", "repeat"]]
            for root in roots:
                (root / "tree_001").mkdir(parents=True)
                (root / "tree_gallery.png").write_bytes(b"gallery")
                for name in ["skeleton.csv", "nodes.yaml", "flows.yaml", "growth.csv"]:
                    (root / "tree_001" / name).write_text("same native content")
            command = [sys.executable, str(script), str(roots[0]), "--compare", str(roots[1])]
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            self.assertIn("4 native files across 1 trees", result.stdout)
            for root in roots:
                (root / "tree_001/growth.csv").unlink()
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)

    def test_variant_preserves_original_and_procedural_graphs(self):
        here = Path(__file__).resolve().parent
        source = here if (here / "Oak.shoot").exists() else here.parents[1] / "Resources/ZachQSM"
        original = (source / "Oak.shoot").read_bytes()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "variant"
            subprocess.run([sys.executable, str(here / "make_variant.py"), "--source", str(source),
                            "--output", str(output), "--set", "shoot.growth_rate=0.16"], check=True, capture_output=True)
            expected = yaml.safe_load(original)
            expected["growth_rate"] = .16
            self.assertEqual(yaml.safe_load((output / "Oak.shoot").read_text()), expected)
            self.assertEqual((source / "Oak.shoot").read_bytes(), original)
            for kind in ["foliage", "pruning"]:
                self.assertEqual(yaml.safe_load((output / f"Oak.{kind}").read_text()),
                                 yaml.safe_load((source / f"Oak.{kind}").read_text()))

    def test_fork_is_one_closed_surface(self):
        starts = np.array([[0,0,0], [0,1,0], [0,1,0]], dtype=float)
        ends = np.array([[0,1,0], [.6,1.8,0], [-.5,1.7,0]], dtype=float)
        mesh = solid_from_cylinders(starts, ends, np.array([.1,.07,.06]), 16)
        self.assertTrue(mesh.is_watertight)
        self.assertEqual(mesh.euler_number, 2)
        self.assertGreater(mesh.volume, .03)

    def test_disconnected_wood_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "disconnected"):
            solid_from_cylinders(np.array([[0,0,0],[5,0,0]], float),
                                 np.array([[0,1,0],[5,1,0]], float), np.array([.1,.1]), 12)

    def test_internal_void_is_filled_without_changing_exterior(self):
        outer = m3d.Manifold.sphere(1, 16)
        hollow = outer - m3d.Manifold.sphere(.1, 16)
        mesh = closed_exterior(hollow)
        self.assertTrue(mesh.is_watertight)
        self.assertEqual(mesh.metadata["filled_internal_cavities"], 1)
        self.assertAlmostEqual(mesh.volume, outer.volume(), places=6)
        self.assertAlmostEqual(mesh.metadata["filled_internal_cavity_volume_m3"],
                               outer.volume() - hollow.volume(), places=8)

    def write_nodes(self, folder, parents, gap=0):
        path = Path(folder) / "skeleton.csv"
        with path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow("id,parent_id,flow_id,order,start_x,start_y,start_z,end_x,end_y,end_z,diameter_m,length_m".split(","))
            w.writerow([0,parents[0],0,0,0,0,0,0,1,0,.2,1])
            w.writerow([1,parents[1],1,1,0,1+gap,0,0,2+gap,0,.1,1])
        return path

    def test_native_parent_and_gap_checks(self):
        with tempfile.TemporaryDirectory() as folder:
            self.assertEqual(read_skeleton(self.write_nodes(folder, [-1,0]))[-1], 0)
            self.assertAlmostEqual(read_skeleton(self.write_nodes(folder, [-1,0], .01))[-1], .01)
            with self.assertRaisesRegex(ValueError, "gap"):
                read_skeleton(self.write_nodes(folder, [-1,0], .3))
            with self.assertRaisesRegex(ValueError, "Missing parent"):
                read_skeleton(self.write_nodes(folder, [-1,99]))


if __name__ == "__main__":
    unittest.main()
