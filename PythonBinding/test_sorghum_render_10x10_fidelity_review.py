import sys
import unittest

import sorghum_render_10x10_fidelity_review as review


class WorkerCommandTest(unittest.TestCase):
    def test_keeps_drive_publication_in_parent_process(self) -> None:
        command = review.worker_command(["--samples", "8", "--publish-drive"])

        self.assertEqual(sys.executable, command[0])
        self.assertEqual(["--samples", "8", "--worker"], command[2:])


if __name__ == "__main__":
    unittest.main()
