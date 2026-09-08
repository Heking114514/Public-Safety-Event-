#!/usr/bin/env python3

import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


HELPER = Path(__file__).resolve().parents[1] / "navigation_run_manifest.py"


class NavigationRunManifestTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name) / "workspace"
        self.root.mkdir()
        self.git("init", "-q")
        self.git("config", "user.email", "test@example.com")
        self.git("config", "user.name", "Navigation Test")
        (self.root / ".gitignore").write_text(
            "build/\ninstall/\nlog/\nnavigation_runs/\n"
            "latest_navigation_bag\nlatest_navigation_run\n",
            encoding="utf-8",
        )
        (self.root / "src").mkdir()
        (self.root / "src" / "main.cpp").write_text("int main() { return 0; }\n")
        (self.root / "scripts").mkdir()
        (self.root / "scripts" / "start.sh").write_text("#!/bin/sh\n")
        self.git("add", ".")
        self.git("commit", "-qm", "initial")
        self.artifact = self.root / "install" / "pkg" / "node"
        self.artifact.parent.mkdir(parents=True)
        self.artifact.write_bytes(b"runtime-v1")
        self.artifact.chmod(0o755)
        self.build_manifest = self.root / "build" / "runtime-build.json"

    def tearDown(self):
        self.temporary.cleanup()

    def git(self, *args, cwd=None):
        subprocess.run(
            ["git", "-C", str(cwd or self.root), *args],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def helper(self, *args, check=True):
        return subprocess.run(
            ["python3", str(HELPER), *map(str, args)],
            check=check,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

    def fingerprint(self):
        return self.helper("fingerprint", "--workspace", self.root).stdout.strip()

    def record_build(self):
        fingerprint = self.fingerprint()
        result = self.helper(
            "record-build",
            "--workspace", self.root,
            "--output", self.build_manifest,
            "--expected-fingerprint", fingerprint,
            "--artifact", f"node={self.artifact}",
            "--setting", "use_serial=false",
        )
        return result.stdout.strip()

    def create_run(self, build_id, *argv):
        arguments = [
            "create-run",
            "--workspace", self.root,
            "--runs-root", self.root / "navigation_runs",
            "--latest-bag", self.root / "latest_navigation_bag",
            "--latest-run", self.root / "latest_navigation_run",
            "--source-fingerprint", self.fingerprint(),
            "--build-manifest", self.build_manifest,
            "--build-id", build_id,
            "--setting", "camera_serial=serial with space",
            "--topic", "/odometry/fused",
            "--input-file", f"startup_script={self.root / 'scripts' / 'start.sh'}",
        ]
        for token in argv:
            arguments.append(f"--argv={token}")
        return Path(self.helper(*arguments).stdout.strip())

    def finalize_run(self, run, exit_code=0, metadata=True):
        bag = run / "bag"
        bag.mkdir()
        (bag / "data.db3").write_bytes(b"recorded data")
        if metadata:
            (bag / "metadata.yaml").write_text("valid: true\n")
        self.helper(
            "finalize-run", "--run-dir", run,
            "--latest-bag", self.root / "latest_navigation_bag",
            "--exit-code", str(exit_code),
            "--parameter-snapshots-complete", "true",
        )

    def test_tracked_and_runtime_untracked_changes_affect_fingerprint(self):
        initial = self.fingerprint()
        (self.root / "src" / "main.cpp").write_text("int main() { return 1; }\n")
        tracked_changed = self.fingerprint()
        self.assertNotEqual(initial, tracked_changed)

        self.git("add", "src/main.cpp")
        staged_changed = self.fingerprint()
        self.assertNotEqual(initial, staged_changed)

        self.git("restore", "--staged", "src/main.cpp")
        self.git("restore", "src/main.cpp")
        self.assertEqual(initial, self.fingerprint())
        (self.root / "src" / "new_config.yaml").write_text("gain: 1\n")
        self.assertNotEqual(initial, self.fingerprint())

    def test_generated_and_bag_content_do_not_affect_fingerprint(self):
        initial = self.fingerprint()
        generated = [
            self.root / "build" / "cache.txt",
            self.root / "install" / "binary",
            self.root / "log" / "build.log",
            self.root / "navigation_runs" / "run" / "bag" / "data.db3",
            self.root / "latest_navigation_bag" / "metadata.yaml",
            self.root / "scripts" / "__pycache__" / "helper.pyc",
        ]
        for index, path in enumerate(generated):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(f"generated-{index}".encode())
        self.assertEqual(initial, self.fingerprint())

    def test_recursive_submodule_changes_affect_fingerprint(self):
        child = Path(self.temporary.name) / "child"
        child.mkdir()
        self.git("init", "-q", cwd=child)
        self.git("config", "user.email", "test@example.com", cwd=child)
        self.git("config", "user.name", "Submodule Test", cwd=child)
        (child / "src").mkdir()
        (child / "src" / "core.cpp").write_text("int core = 1;\n")
        self.git("add", ".", cwd=child)
        self.git("commit", "-qm", "child", cwd=child)
        self.git("-c", "protocol.file.allow=always", "submodule", "add", "-q", str(child), "src/dependency")
        self.git("commit", "-qam", "add submodule")

        initial = self.fingerprint()
        (self.root / "src" / "dependency" / "src" / "core.cpp").write_text("int core = 2;\n")
        self.assertNotEqual(initial, self.fingerprint())
        source_info = self.root / "source-info.json"
        self.helper(
            "source-info", "--workspace", self.root, "--output", source_info
        )
        submodule = json.loads(source_info.read_text())["submodules"][0]
        self.assertTrue(submodule["dirty"])
        self.assertIn("repository_status_sha256", submodule)
        self.git("restore", "src/core.cpp", cwd=self.root / "src" / "dependency")
        (self.root / "src" / "dependency" / "src" / "new.hpp").write_text("#pragma once\n")
        self.assertNotEqual(initial, self.fingerprint())

    def test_build_verification_rejects_source_and_artifact_changes(self):
        self.record_build()
        verify = (
            "verify-build", "--workspace", self.root,
            "--manifest", self.build_manifest,
            "--artifact", f"node={self.artifact}",
        )
        self.helper(*verify)

        (self.root / "src" / "main.cpp").write_text("int main() { return 2; }\n")
        result = self.helper(*verify, check=False)
        self.assertNotEqual(0, result.returncode)
        self.assertIn("source fingerprint differs", result.stderr)

        self.git("restore", "src/main.cpp")
        self.artifact.write_bytes(b"runtime-v2")
        result = self.helper(*verify, check=False)
        self.assertNotEqual(0, result.returncode)
        self.assertIn("artifact 'node' hash differs", result.stderr)

    def test_build_manifest_records_runtime_and_resolved_symlink_paths(self):
        real_artifact = self.root / "build" / "pkg" / "node-real"
        real_artifact.parent.mkdir(parents=True, exist_ok=True)
        real_artifact.write_bytes(b"real runtime")
        real_artifact.chmod(0o755)
        self.artifact.unlink()
        self.artifact.symlink_to(real_artifact)

        self.record_build()
        manifest = json.loads(self.build_manifest.read_text())
        entry = manifest["artifacts"]["node"]
        self.assertEqual(str(self.artifact), entry["runtime_path"])
        self.assertEqual(str(real_artifact), entry["resolved_path"])

    def test_unique_runs_preserve_legacy_and_do_not_precreate_bag(self):
        build_id = self.record_build()
        legacy_bag = self.root / "latest_navigation_bag"
        legacy_bag.mkdir()
        legacy_bytes = b"legacy bag bytes\x00\x01"
        (legacy_bag / "data.db3").write_bytes(legacy_bytes)

        first = self.create_run(build_id, "--camera-serial", "serial with space")
        self.assertTrue(first.is_dir())
        self.assertFalse((first / "bag").exists())
        self.assertTrue(legacy_bag.is_symlink())
        self.assertEqual(legacy_bytes, (legacy_bag.resolve() / "data.db3").read_bytes())
        first_manifest = json.loads((first / "run_manifest.json").read_text())
        self.assertEqual(
            ["--camera-serial", "serial with space"], first_manifest["raw_argv"]
        )

        (first / "bag").mkdir()
        (first / "bag" / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
        self.helper(
            "finalize-run", "--run-dir", first,
            "--latest-bag", legacy_bag,
            "--exit-code", "0",
            "--parameter-snapshots-complete", "true",
        )
        self.assertEqual(first / "bag", legacy_bag.resolve())

        second = self.create_run(build_id, "--no-serial")
        self.assertNotEqual(first, second)
        self.assertTrue(first.exists())
        self.assertFalse((second / "bag").exists())
        self.assertEqual(first / "bag", legacy_bag.resolve())
        source_info = self.root / "source-info.json"
        self.helper(
            "source-info", "--workspace", self.root, "--output", source_info
        )
        self.assertFalse(json.loads(source_info.read_text())["dirty"])

    def test_parameter_entries_and_incomplete_finalize_are_explicit(self):
        build_id = self.record_build()
        run = self.create_run(build_id)
        snapshot = run / "parameters" / "fused_ekf.yaml"
        snapshot.write_text("/fused_ekf:\n  ros__parameters: {}\n")
        self.helper(
            "record-parameter", "--run-dir", run,
            "--node", "/fused_ekf", "--status", "success",
            "--snapshot", snapshot,
        )
        self.helper(
            "record-parameter", "--run-dir", run,
            "--node", "/waypoint_navigator", "--status", "failed",
            "--error", "timeout",
        )
        self.helper(
            "record-parameter", "--run-dir", run,
            "--node", "/imu_rpy_filter", "--status", "skipped",
            "--error", "disabled",
        )
        (run / "bag").mkdir()
        (run / "bag" / "metadata.yaml").write_text("valid: true\n")
        self.helper(
            "finalize-run", "--run-dir", run,
            "--latest-bag", self.root / "latest_navigation_bag",
            "--exit-code", "0",
            "--parameter-snapshots-complete", "false",
        )

        manifest = json.loads((run / "run_manifest.json").read_text())
        self.assertEqual("success", manifest["parameters"]["/fused_ekf"]["status"])
        self.assertTrue(manifest["parameters"]["/fused_ekf"]["sha256"])
        self.assertEqual("failed", manifest["parameters"]["/waypoint_navigator"]["status"])
        self.assertEqual("skipped", manifest["parameters"]["/imu_rpy_filter"]["status"])
        self.assertFalse(manifest["parameter_snapshots_complete"])
        self.assertEqual("failed", manifest["status"])

    def test_failed_run_without_metadata_keeps_previous_latest_bag(self):
        build_id = self.record_build()
        previous = self.root / "navigation_runs" / "previous" / "bag"
        previous.mkdir(parents=True)
        (previous / "metadata.yaml").write_text("valid: true\n")
        (previous.parent / "run_manifest.json").write_text(json.dumps({
            "schema_version": 1,
            "run_id": previous.parent.name,
            "status": "completed",
            "end_time": "2020-01-01T00:00:00+00:00",
            "bag": {"path": str(previous), "present": True},
        }))
        latest = self.root / "latest_navigation_bag"
        latest.symlink_to(previous)

        run = self.create_run(build_id)
        (run / "bag").mkdir()
        (run / "bag" / "partial.db3").write_bytes(b"partial")
        (run / "bag" / "metadata.yaml").write_text("valid: true\n")
        self.helper(
            "finalize-run", "--run-dir", run,
            "--latest-bag", latest,
            "--exit-code", "37",
            "--parameter-snapshots-complete", "true",
        )
        self.assertEqual(previous, latest.resolve())
        manifest = json.loads((run / "run_manifest.json").read_text())
        self.assertEqual(37, manifest["exit_code"])
        self.assertEqual("failed", manifest["status"])
        self.assertTrue(manifest["bag"]["metadata_present"])
        self.assertFalse(manifest["bag"]["latest_link_updated"])

    def test_retention_keeps_only_three_latest_completed_bags(self):
        build_id = self.record_build()
        runs = []
        for _ in range(5):
            run = self.create_run(build_id)
            self.finalize_run(run)
            runs.append(run)

        self.assertEqual(
            [False, False, True, True, True],
            [(run / "bag").is_dir() for run in runs],
        )
        self.assertEqual(
            runs[-1] / "bag",
            (self.root / "latest_navigation_bag").resolve(),
        )
        for run in runs[:2]:
            manifest = json.loads((run / "run_manifest.json").read_text())
            self.assertFalse(manifest["bag"]["present"])
            self.assertTrue(manifest["bag"]["pruned_at"])
            self.assertEqual("retention_limit_3", manifest["bag"]["pruned_reason"])

    def test_retention_is_strictly_the_three_most_recent_runs(self):
        build_id = self.record_build()
        completed = self.create_run(build_id)
        self.finalize_run(completed)
        failed = []
        for _ in range(3):
            run = self.create_run(build_id)
            self.finalize_run(run, exit_code=37)
            failed.append(run)

        latest = self.root / "latest_navigation_bag"
        self.assertFalse(latest.exists())
        self.assertFalse(latest.is_symlink())
        self.assertFalse((completed / "bag").exists())
        self.assertTrue(all((run / "bag").is_dir() for run in failed))

    def test_legacy_bag_counts_toward_retention_and_ages_out(self):
        build_id = self.record_build()
        latest = self.root / "latest_navigation_bag"
        latest.mkdir()
        (latest / "metadata.yaml").write_text("valid: true\n")
        (latest / "legacy.db3").write_bytes(b"legacy")

        runs = []
        for _ in range(3):
            run = self.create_run(build_id)
            self.finalize_run(run)
            runs.append(run)

        legacy_runs = [
            path for path in (self.root / "navigation_runs").iterdir()
            if path.name.startswith("legacy_")
        ]
        self.assertEqual(1, len(legacy_runs))
        legacy_manifest = json.loads(
            (legacy_runs[0] / "run_manifest.json").read_text()
        )
        self.assertFalse((legacy_runs[0] / "bag").exists())
        self.assertFalse(legacy_manifest["bag"]["present"])
        self.assertEqual(runs[-1] / "bag", latest.resolve())

    def test_retention_ignores_external_and_symlink_bag_paths(self):
        build_id = self.record_build()
        runs_root = self.root / "navigation_runs"
        runs_root.mkdir()
        outside = self.root / "outside_bag"
        outside.mkdir()
        (outside / "metadata.yaml").write_text("valid: true\n")
        (outside / "outside.db3").write_bytes(b"must survive")

        external_run = runs_root / "external"
        external_run.mkdir()
        (external_run / "run_manifest.json").write_text(json.dumps({
            "schema_version": 1,
            "run_id": external_run.name,
            "status": "failed",
            "end_time": "2020-01-01T00:00:00+00:00",
            "bag": {"path": str(outside)},
        }))
        symlink_run = runs_root / "symlink"
        symlink_run.mkdir()
        (symlink_run / "bag").symlink_to(outside, target_is_directory=True)
        (symlink_run / "run_manifest.json").write_text(json.dumps({
            "schema_version": 1,
            "run_id": symlink_run.name,
            "status": "failed",
            "end_time": "2020-01-02T00:00:00+00:00",
            "bag": {"path": str(symlink_run / "bag")},
        }))

        for _ in range(4):
            run = self.create_run(build_id)
            self.finalize_run(run)

        self.assertEqual(b"must survive", (outside / "outside.db3").read_bytes())
        self.assertTrue((symlink_run / "bag").is_symlink())
        self.assertTrue(external_run.is_dir())

    def test_create_run_marks_old_active_run_abandoned_and_retains_it_normally(self):
        build_id = self.record_build()
        runs_root = self.root / "navigation_runs"
        old_run = runs_root / "old_running"
        (old_run / "bag").mkdir(parents=True)
        (old_run / "bag" / "partial.db3").write_bytes(b"partial")
        (old_run / "run_manifest.json").write_text(json.dumps({
            "schema_version": 1,
            "run_id": old_run.name,
            "status": "running",
            "start_time": "2020-01-01T00:00:00+00:00",
            "bag": {"path": str(old_run / "bag")},
        }))

        runs = []
        for _ in range(3):
            run = self.create_run(build_id)
            self.finalize_run(run)
            runs.append(run)

        manifest = json.loads((old_run / "run_manifest.json").read_text())
        self.assertEqual("abandoned", manifest["status"])
        self.assertEqual("superseded_by_new_run", manifest["abandoned_reason"])
        self.assertFalse((old_run / "bag").exists())
        self.assertFalse(manifest["bag"]["present"])
        self.assertTrue(all((run / "bag").is_dir() for run in runs))


if __name__ == "__main__":
    unittest.main()
