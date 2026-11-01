"""Exercise environment rejection before setup can modify a workspace."""
import os
import shutil
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "scripts/ros_environment.sh"


class EnvironmentTests(unittest.TestCase):
    def run_helper(self, command, **overrides):
        env = os.environ.copy()
        for key in ("ROS_DISTRO", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH"):
            env.pop(key, None)
        env.update(overrides)
        return subprocess.run(
            ["bash", "-c", 'set -euo pipefail; source "$1"; ' + command, "bash", str(HELPER)],
            env=env, text=True, capture_output=True,
        )

    def test_conflicting_ros_distro_is_rejected(self):
        result = self.run_helper("require_lyrical_shell", ROS_DISTRO="jazzy")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("fresh terminal", result.stderr)

    def test_old_ros_prefix_is_rejected_even_without_ros_distro(self):
        result = self.run_helper("require_lyrical_shell", AMENT_PREFIX_PATH="/opt/ros/jazzy")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("fresh terminal", result.stderr)

    def test_clean_and_lyrical_shells_are_accepted(self):
        for env in ({}, {"ROS_DISTRO": "lyrical", "AMENT_PREFIX_PATH": "/opt/ros/lyrical"}):
            with self.subTest(env=env):
                result = self.run_helper("require_lyrical_shell", **env)
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_unmarked_build_is_rejected_without_modifying_files(self):
        with tempfile.TemporaryDirectory() as directory:
            build = Path(directory) / "build"
            build.mkdir()
            result = self.run_helper('check_workspace_platform "$TEST_WS" setup', TEST_WS=directory)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("move", result.stderr.lower())
            self.assertTrue(build.exists())

    def test_jazzy_platform_marker_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / ".platform"
            marker.write_text("jazzy:3.12\n")
            result = self.run_helper('check_workspace_platform "$TEST_WS" setup', TEST_WS=directory)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("incompatible", result.stderr.lower())
            self.assertEqual(marker.read_text(), "jazzy:3.12\n")

    def test_fresh_workspace_is_allowed_only_for_setup(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.run_helper('check_workspace_platform "$TEST_WS" setup', TEST_WS=directory)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = self.run_helper('check_workspace_platform "$TEST_WS" launch', TEST_WS=directory)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("setup_workspace.sh", result.stderr)

    def test_matching_platform_marker_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            version = subprocess.check_output(
                ["/usr/bin/python3", "-c", "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"],
                text=True,
            ).strip()
            (Path(directory) / ".platform").write_text(f"lyrical:{version}\n")
            result = self.run_helper('check_workspace_platform "$TEST_WS" setup', TEST_WS=directory)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_all_entry_points_reject_conflicting_ros_before_doing_work(self):
        for name in ("setup_workspace.sh", "launch_sim.sh", "launch_ps5_teleop.sh", "stop_car.sh", "smoke_test.sh"):
            with self.subTest(script=name):
                result = subprocess.run(
                    ["bash", str(ROOT / "scripts" / name)],
                    env={**os.environ, "ROS_DISTRO": "jazzy"},
                    text=True, capture_output=True,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("fresh terminal", result.stderr)

    def test_real_venv_requires_access_to_system_packages(self):
        with tempfile.TemporaryDirectory() as directory:
            venv = Path(directory) / ".venv"
            subprocess.run(["/usr/bin/python3", "-m", "venv", "--without-pip", str(venv)], check=True)
            result = self.run_helper('check_workspace_platform "$TEST_WS" setup', TEST_WS=directory)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("system ROS packages", result.stderr)
            subprocess.run(["/usr/bin/python3", "-m", "venv", "--without-pip", "--system-site-packages", str(venv)], check=True)
            result = self.run_helper('check_workspace_platform "$TEST_WS" setup', TEST_WS=directory)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_incomplete_virtual_environment_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / ".venv").mkdir()
            result = self.run_helper('check_workspace_platform "$TEST_WS" setup', TEST_WS=directory)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("Incomplete virtual environment", result.stderr)

    def test_copied_or_moved_venv_is_rejected(self):
        for move in (False, True):
            with self.subTest(move=move), tempfile.TemporaryDirectory() as directory:
                old = Path(directory) / "old"
                new = Path(directory) / "new"
                subprocess.run(["/usr/bin/python3", "-m", "venv", "--without-pip", "--system-site-packages", str(old / ".venv")], check=True)
                if move:
                    shutil.move(str(old), str(new))
                else:
                    shutil.copytree(old, new, symlinks=True)
                result = self.run_helper('check_workspace_platform "$TEST_WS" setup', TEST_WS=str(new))
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("relocated", result.stderr.lower())

    def test_venv_created_for_old_python_is_rejected_after_system_upgrade(self):
        with tempfile.TemporaryDirectory() as directory:
            venv = Path(directory) / ".venv"
            subprocess.run(["/usr/bin/python3", "-m", "venv", "--without-pip", "--system-site-packages", str(venv)], check=True)
            config = venv / "pyvenv.cfg"
            config.write_text("\n".join("version = 3.12.3" if line.startswith("version = ") else line for line in config.read_text().splitlines()) + "\n")
            result = self.run_helper('check_workspace_platform "$TEST_WS" setup', TEST_WS=directory)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("created for", result.stderr.lower())


if __name__ == "__main__":
    unittest.main()
