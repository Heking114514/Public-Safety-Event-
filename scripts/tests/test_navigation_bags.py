#!/usr/bin/env python3

import os
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation_bags import (
    BagError,
    atomic_symlink,
    compatibility_link_target,
    is_compatibility_link,
    preserve_legacy_bag,
    stale_windows_link_target,
)


def write_pseudo_link(path, target):
    path.write_bytes(b"IntxLNK\x01" + str(target).encode("utf-16-le") + b"\x00\x00")


class StaleWindowsLinkTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def test_absolute_pseudo_link_is_decoded(self):
        target = self.root / "navigation_runs" / "run-1"
        target.mkdir(parents=True)
        link = self.root / "latest_navigation_run"
        write_pseudo_link(link, target)

        self.assertFalse(link.is_symlink())
        self.assertEqual(target, stale_windows_link_target(link))
        self.assertTrue(is_compatibility_link(link))
        self.assertEqual(target, compatibility_link_target(link))

    def test_relative_pseudo_link_is_resolved_against_its_directory(self):
        target = self.root / "navigation_runs" / "run-1"
        target.mkdir(parents=True)
        link = self.root / "latest_navigation_run"
        write_pseudo_link(link, "navigation_runs/run-1")

        self.assertEqual(target, stale_windows_link_target(link))
        self.assertEqual(target, compatibility_link_target(link))

    def test_pseudo_link_with_missing_target_has_no_resolved_target(self):
        link = self.root / "latest_navigation_run"
        write_pseudo_link(link, self.root / "navigation_runs" / "gone")

        self.assertTrue(is_compatibility_link(link))
        self.assertIsNone(compatibility_link_target(link))

    def test_plain_files_and_directories_are_not_links(self):
        regular = self.root / "latest_navigation_run"
        regular.write_text("ordinary file\n", encoding="utf-8")
        directory = self.root / "latest_navigation_bag"
        directory.mkdir()
        short = self.root / "short"
        short.write_bytes(b"Intx")
        truncated = self.root / "truncated"
        truncated.write_bytes(b"IntxLNK\x01\x41")

        for path in (regular, directory, short, truncated):
            self.assertIsNone(stale_windows_link_target(path), path)
            self.assertFalse(is_compatibility_link(path), path)
            self.assertIsNone(compatibility_link_target(path), path)

    def test_real_symlink_is_still_a_compatibility_link(self):
        target = self.root / "navigation_runs" / "run-1"
        target.mkdir(parents=True)
        link = self.root / "latest_navigation_run"
        os.symlink(os.path.relpath(target, self.root), link)

        self.assertIsNone(stale_windows_link_target(link))
        self.assertTrue(is_compatibility_link(link))
        self.assertEqual(target, compatibility_link_target(link))

    def test_atomic_symlink_replaces_pseudo_link_but_not_regular_file(self):
        target = self.root / "navigation_runs" / "run-1"
        target.mkdir(parents=True)
        link = self.root / "latest_navigation_run"
        write_pseudo_link(link, self.root / "navigation_runs" / "run-0")

        atomic_symlink(target, link)
        self.assertTrue(link.is_symlink())
        self.assertEqual(target, link.resolve())

        link.unlink()
        link.write_text("unrelated content\n", encoding="utf-8")
        with self.assertRaises(BagError):
            atomic_symlink(target, link)
        self.assertEqual("unrelated content\n", link.read_text())

    def test_preserve_legacy_bag_drops_stale_pointer_and_keeps_directory(self):
        runs_root = self.root / "navigation_runs"
        runs_root.mkdir()
        legacy_bag = self.root / "latest_navigation_bag"
        write_pseudo_link(legacy_bag, runs_root / "run-1")

        preserve_legacy_bag(legacy_bag, runs_root)

        self.assertFalse(legacy_bag.exists())
        self.assertFalse(legacy_bag.is_symlink())
        self.assertEqual([], list(runs_root.iterdir()))

    def test_preserve_legacy_bag_still_migrates_a_real_directory(self):
        runs_root = self.root / "navigation_runs"
        runs_root.mkdir()
        legacy_bag = self.root / "latest_navigation_bag"
        legacy_bag.mkdir()
        (legacy_bag / "data.db3").write_bytes(b"legacy")

        preserve_legacy_bag(legacy_bag, runs_root)

        self.assertTrue(legacy_bag.is_symlink())
        self.assertEqual(b"legacy", (legacy_bag.resolve() / "data.db3").read_bytes())


if __name__ == "__main__":
    unittest.main()
