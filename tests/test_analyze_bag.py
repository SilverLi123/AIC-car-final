"""Unit tests for src/scripts/analyze_bag.py (pure Python, no ROS 2 required)."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest

MODULE_PATH = Path(__file__).resolve().parent.parent / "src" / "scripts" / "analyze_bag.py"
SPEC = importlib.util.spec_from_file_location("analyze_bag", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
analyze_bag_mod = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analyze_bag_mod)


class TestAnalyzeBagHelpers(unittest.TestCase):
    def test_fps_and_max_gap(self) -> None:
        self.assertEqual(
            analyze_bag_mod.compute_fps_and_max_gap([])["count"],
            0,
        )
        # 31 frames at ~30 Hz (33 ms), with one 100 ms drop
        ts = [i * 33_000_000 for i in range(15)] + [
            14 * 33_000_000 + 100_000_000 + i * 33_000_000 for i in range(16)
        ]
        res = analyze_bag_mod.compute_fps_and_max_gap(ts)
        self.assertEqual(res["count"], 31)
        self.assertAlmostEqual(res["max_gap_ms"], 100.0, places=1)
        self.assertGreater(res["fps"], 25.0)
        self.assertLess(res["fps"], 31.0)

    def test_avoid_metrics_episodes_lat_accel_and_flips(self) -> None:
        avoid_samples = [
            # Episode 1: smooth positive turn at v=0.60, w=0.50 -> lat_a=0.30
            (1.00, 0.30, 0.40),
            (1.10, 0.30, 0.50),
            (1.20, 0.30, 0.45),
            # Gap > 0.35 s -> Episode 2 with yaw sign flip
            (2.00, 0.40, 0.60),
            (2.10, 0.40, -0.55),
            (2.20, 0.40, -0.20),
        ]
        odom_samples = [
            (1.00, 0.58),
            (1.10, 0.60),
            (1.20, 0.59),
            (2.00, 0.62),
            (2.10, 0.60),
            (2.20, 0.55),
        ]
        res = analyze_bag_mod.compute_avoid_metrics(avoid_samples, odom_samples)
        self.assertEqual(res["episode_count"], 2)
        self.assertAlmostEqual(res["max_abs_yaw"], 0.60, places=2)
        self.assertAlmostEqual(res["max_lat_accel"], 0.62 * 0.60, places=2)
        self.assertEqual(res["yaw_sign_flips"], 1)
        self.assertAlmostEqual(res["max_yaw_step"], 1.15, places=2)

    def test_braking_episodes_coast_and_reverse(self) -> None:
        stop_samples = [
            # Episode 1: coasting without reverse brake
            (5.00, 0.0),
            (5.10, 0.0),
            # Episode 2: reverse braking active
            (12.00, -0.40),
            (12.10, -0.40),
            (12.20, 0.0),
        ]
        odom_samples = [
            (4.95, 1.00, 0.50),
            (5.00, 1.02, 0.48),
            (5.20, 1.10, 0.25),
            (5.40, 1.16, 0.02),
            (11.95, 4.00, 0.58),
            (12.00, 4.03, 0.56),
            (12.15, 4.09, 0.22),
            (12.30, 4.11, 0.01),
        ]
        eps = analyze_bag_mod.compute_braking_episodes(stop_samples, odom_samples)
        self.assertEqual(len(eps), 2)
        self.assertFalse(eps[0]["reverse_engaged"])
        self.assertAlmostEqual(eps[0]["coast_cm"], 14.0, places=1)
        self.assertAlmostEqual(eps[0]["time_to_zero_s"], 0.40, places=2)

        self.assertTrue(eps[1]["reverse_engaged"])
        self.assertAlmostEqual(eps[1]["coast_cm"], 8.0, places=1)
        self.assertAlmostEqual(eps[1]["time_to_zero_s"], 0.30, places=2)

    def test_parse_rosout_events(self) -> None:
        logs = [
            (2.1, "Yellow stop line #2 detected, reverse brake engaged"),
            (3.4, "Pair decision chosen=6 conf=0.82 share=0.95"),
            (4.5, "[digit turn][robust:fork] best:8 votes:3 conf_sum:1.39 x:0.70 y:0.51 dur:0.48s lin:0.22 ang:-0.35"),
            (5.0, "[T-merge] Inner branch armed (yellow=2, det_w=-0.35, dist=2.60m)."),
            (7.0, "T-junction arm armed after yellow #2"),
            (8.2, "[T-merge] Triggered (wall): upper=0.01, wide=0.32 -> v=0.14 w=-1.45"),
            (9.1, "[T-merge] Completed (upper=0.85, wide=0.90). Returning to closed-loop PD."),
        ]
        ev = analyze_bag_mod.parse_rosout_events(logs)
        self.assertEqual(ev["yellow_stop_logs"], 1)
        self.assertEqual(len(ev["digit_turns"]), 2)
        self.assertEqual(ev["digit_turns"][0]["digit"], 6)
        self.assertAlmostEqual(ev["digit_turns"][0]["conf"], 0.82, places=2)
        self.assertEqual(ev["digit_turns"][1]["digit"], 8)
        self.assertAlmostEqual(ev["digit_turns"][1]["conf"], 1.39, places=2)
        self.assertEqual(ev["tjunction_armed"], 2)
        self.assertEqual(ev["tjunction_triggered"], 1)
        self.assertEqual(ev["tjunction_exited"], 1)

    def test_analyze_bag_offline_sqlite_and_markdown_table(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bag = root / "fork_f3_20261127_183000"
            bag.mkdir()
            (bag / "run_meta.json").write_text(
                json.dumps(
                    {
                        "map": "fork",
                        "tag": "f3",
                        "params_basename": "YOLO_FORK_OPT4_S044_PREVIEW_RB.yaml",
                        "yolo_model_basename": "best_modified_yolov8.bin",
                        "yolo_model_md5": "deadbeef1234",
                    }
                ),
                encoding="utf-8",
            )
            (bag / "metadata.yaml").write_text(
                "rosbag2_bagfile_information:\n  compression_format: zstd\n  compression_mode: message\n",
                encoding="utf-8",
            )

            db = bag / "fork_f3_20261127_183000_0.db3"
            conn = sqlite3.connect(db)
            conn.execute(
                "CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT, offered_qos_profiles TEXT)"
            )
            conn.execute(
                "CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)"
            )
            conn.execute(
                "INSERT INTO topics VALUES (1, '/image_raw', 'sensor_msgs/msg/Image', 'cdr', '')"
            )
            conn.execute(
                "INSERT INTO topics VALUES (2, '/odom', 'nav_msgs/msg/Odometry', 'cdr', '')"
            )
            base_ns = 1_700_000_000_000_000_000
            for idx in range(10):
                conn.execute(
                    "INSERT INTO messages (topic_id, timestamp, data) VALUES (1, ?, ?)",
                    (base_ns + idx * 33_333_333, b"fake_img"),
                )
                conn.execute(
                    "INSERT INTO messages (topic_id, timestamp, data) VALUES (2, ?, ?)",
                    (base_ns + idx * 20_000_000, b"fake_odom"),
                )
            conn.commit()
            conn.close()

            discovered = analyze_bag_mod.discover_bags([root])
            self.assertEqual(discovered, [bag.resolve()])

            report = analyze_bag_mod.analyze_bag(bag)
            self.assertEqual(report["bag_name"], bag.name)
            self.assertEqual(report["camera_fps"]["count"], 10)
            self.assertAlmostEqual(report["camera_fps"]["fps"], 30.0, places=0)
            self.assertEqual(report["topic_counts"]["/odom"], 10)

            md = analyze_bag_mod.format_comparison_markdown([report])
            self.assertIn("YOLO_FORK_OPT4_S044_PREVIEW_RB.yaml", md)
            self.assertIn("best_modified_yolov8.bin", md)
            summary = analyze_bag_mod.format_single_summary(report)
            self.assertIn("deadbeef", summary)


if __name__ == "__main__":
    unittest.main()
