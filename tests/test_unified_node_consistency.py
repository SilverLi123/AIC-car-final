"""Static consistency checks for unified dual-map ROS 2 C++ nodes, presets, and CLI."""

from __future__ import annotations

import json
import pathlib
import re
import subprocess
import unittest

try:
    import yaml  # type: ignore
except ModuleNotFoundError:  # pragma: no cover
    yaml = None  # type: ignore


def _load_yaml(path: pathlib.Path) -> dict:
    if yaml is not None:
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    raw = subprocess.check_output(
        ["ruby", "-ryaml", "-rjson", "-e", "puts JSON.generate(YAML.load_file(ARGV[0]))", str(path)],
        text=True,
    )
    return json.loads(raw)


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PKG_DIR = REPO_ROOT / "src" / "line_follow_ws" / "src" / "line_follower"
PRESETS_DIR = PKG_DIR / "config" / "parameter_presets"


def _strip_cpp_comments_and_strings(src: str) -> str:
    """Remove C/C++ comments and quoted literals before brace balance inspection."""
    pattern = re.compile(
        r'//.*?$|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        re.DOTALL | re.MULTILINE,
    )
    return pattern.sub(" ", src)


def _infer_declared_cpp_types(cpp_text: str) -> dict[str, str]:
    """Parse ROS 2 `declare_parameter` calls (both template and untyped overloads)."""
    declared: dict[str, str] = {}

    typed_pat = re.compile(
        r'declare_parameter<\s*([^>]+?)\s*>\(\s*"([^"]+)"\s*,'
    )
    for ctype, name in typed_pat.findall(cpp_text):
        if "bool" in ctype:
            declared[name] = "bool"
        elif "int" in ctype:
            declared[name] = "int"
        elif "double" in ctype or "float" in ctype:
            declared[name] = "double"
        else:
            declared[name] = "string"

    untyped_pat = re.compile(
        r'declare_parameter\(\s*"([^"]+)"\s*,\s*([^)]+?)\s*\)\s*;'
    )
    for name, default_expr in untyped_pat.findall(cpp_text):
        expr = default_expr.strip()
        if expr in {"true", "false"}:
            declared[name] = "bool"
        elif expr.startswith('"'):
            declared[name] = "string"
        elif re.fullmatch(r"-?\d+", expr):
            declared[name] = "int"
        elif re.fullmatch(r"-?\d+(?:\.\d+)?", expr):
            declared[name] = "double"
        else:
            declared[name] = "double"
    return declared


def _is_yaml_value_compatible(val: object, cpp_type: str) -> bool:
    if cpp_type == "bool":
        return isinstance(val, bool)
    if cpp_type == "int":
        return isinstance(val, int) and not isinstance(val, bool)
    if cpp_type == "double":
        return isinstance(val, (int, float)) and not isinstance(val, bool)
    if cpp_type == "string":
        return isinstance(val, str)
    return False


class UnifiedNodeConsistencyTests(unittest.TestCase):
    def test_cpp_files_have_balanced_delimiters(self) -> None:
        files = [
            PKG_DIR / "include" / "line_follower" / "line_follower_node.hpp",
            PKG_DIR / "include" / "line_follower" / "passable_gap_guidance.hpp",
            PKG_DIR / "include" / "line_follower" / "red_lat_accel_limiter.hpp",
            PKG_DIR / "src" / "line_follower_node.cpp",
            PKG_DIR / "src" / "detection_turn_controller.cpp",
            PKG_DIR / "src" / "cmd_mux_node.cpp",
        ]
        pairs = {"(": ")", "{": "}", "[": "]"}
        closing = {v: k for k, v in pairs.items()}
        for path in files:
            stripped = _strip_cpp_comments_and_strings(path.read_text(encoding="utf-8"))
            stack: list[str] = []
            for ch in stripped:
                if ch in pairs:
                    stack.append(ch)
                elif ch in closing:
                    self.assertTrue(stack, f"{path.name}: unmatched closing '{ch}'")
                    self.assertEqual(stack.pop(), closing[ch], f"{path.name}: mismatched '{ch}'")
            self.assertEqual(stack, [], f"{path.name}: unclosed delimiters {stack}")

    def test_cmake_and_launch_wiring(self) -> None:
        cmake = (PKG_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        m = re.search(
            r"ament_target_dependencies\(\s*detection_turn_controller(.*?)\)",
            cmake,
            re.DOTALL,
        )
        self.assertIsNotNone(m)
        self.assertIn("std_msgs", m.group(1))

        launch_txt = (
            PKG_DIR / "launch" / "line_follower_with_detection.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("avoid_cmd_topic", launch_txt)
        self.assertIn("avoid_topic", launch_txt)

    def test_line_follower_get_parameter_all_declared(self) -> None:
        lf_cpp = (PKG_DIR / "src" / "line_follower_node.cpp").read_text(encoding="utf-8")
        declared = _infer_declared_cpp_types(lf_cpp)
        used = set(re.findall(r'this->get_parameter\(\s*"([^"]+)"\s*\)', lf_cpp))
        missing = sorted(used - set(declared))
        self.assertEqual(missing, [], f"Undeclared get_parameter keys: {missing}")

        # Ensure regressive 5a6e8bb logic is permanently absent
        self.assertNotIn("hold_clipped_cone", lf_cpp)
        self.assertNotIn("edge_clipped", lf_cpp)

    def test_all_active_presets_match_declared_node_parameters(self) -> None:
        node_declared = {
            "line_follower_node": _infer_declared_cpp_types(
                (PKG_DIR / "src" / "line_follower_node.cpp").read_text(encoding="utf-8")
            ),
            "detection_turn_controller": _infer_declared_cpp_types(
                (PKG_DIR / "src" / "detection_turn_controller.cpp").read_text(encoding="utf-8")
            ),
            "cmd_mux_node": _infer_declared_cpp_types(
                (PKG_DIR / "src" / "cmd_mux_node.cpp").read_text(encoding="utf-8")
            ),
        }

        yaml_files = sorted(PRESETS_DIR.rglob("YOLO_*.yaml"))
        self.assertGreaterEqual(len(yaml_files), 18)

        for ypath in yaml_files:
            with self.subTest(preset=str(ypath.relative_to(PRESETS_DIR))):
                data = _load_yaml(ypath)
                self.assertIn("line_follower_node", data)
                self.assertIn("detection_turn_controller", data)
                self.assertIn("cmd_mux_node", data)

                for node_name, decl_map in node_declared.items():
                    params = data[node_name]["ros__parameters"]
                    for key, val in params.items():
                        self.assertIn(
                            key,
                            decl_map,
                            f"{ypath.name}: {node_name}.{key} is not declared in C++",
                        )
                        self.assertTrue(
                            _is_yaml_value_compatible(val, decl_map[key]),
                            f"{ypath.name}: {node_name}.{key}={val!r} incompatible with C++ {decl_map[key]}",
                        )

                lf = data["line_follower_node"]["ros__parameters"]
                if ypath.name.startswith("YOLO_RING_"):
                    self.assertFalse(lf.get("enable_tjunction_merge", False))
                    self.assertFalse(lf.get("enable_passable_gap_red_avoid", False))
                elif ypath.name.startswith("YOLO_FORK_"):
                    self.assertTrue(lf.get("enable_tjunction_merge", False))
                    self.assertTrue(lf.get("enable_passable_gap_red_avoid", False))

    def test_canonical_ring_and_fork_base_invariants(self) -> None:
        ring_base = _load_yaml(PRESETS_DIR / "YOLO_RING_BASE.yaml")
        lf_ring = ring_base["line_follower_node"]["ros__parameters"]
        self.assertAlmostEqual(lf_ring["target_speed"], 0.52)
        self.assertAlmostEqual(lf_ring["roi_top_ratio"], 0.58)
        self.assertAlmostEqual(lf_ring["red_roi_top_ratio"], 0.45)
        self.assertAlmostEqual(lf_ring["red_avoid_max_yaw_rate"], 1.25)
        self.assertTrue(lf_ring["enable_reverse_brake"])
        self.assertFalse(lf_ring["enable_tjunction_merge"])
        self.assertFalse(lf_ring["enable_passable_gap_red_avoid"])

        fork_base = _load_yaml(PRESETS_DIR / "YOLO_FORK_BASE.yaml")
        lf_fork = fork_base["line_follower_node"]["ros__parameters"]
        dtc_fork = fork_base["detection_turn_controller"]["ros__parameters"]
        self.assertAlmostEqual(lf_fork["target_speed"], 0.28)
        self.assertAlmostEqual(lf_fork["roi_top_ratio"], 0.65)
        self.assertAlmostEqual(lf_fork["tjunction_min_arm_dist"], 0.75)
        self.assertEqual(lf_fork["tjunction_min_full_rows"], 80)
        self.assertAlmostEqual(lf_fork["tjunction_min_duration"], 1.30)
        self.assertTrue(lf_fork["enable_tjunction_merge"])
        self.assertTrue(lf_fork["enable_passable_gap_red_avoid"])
        self.assertAlmostEqual(dtc_fork["min_winner_conf"], 0.55)
        self.assertFalse(dtc_fork.get("enable_inlane_centering", False))

    def test_launcher_lists_canonical_bases_first_and_discovers_models(self) -> None:
        ring_out = subprocess.check_output(
            ["bash", "src/scripts/start_line_following.sh", "--list-presets", "ring"],
            cwd=REPO_ROOT,
            text=True,
        )
        fork_out = subprocess.check_output(
            ["bash", "src/scripts/start_line_following.sh", "--list-presets", "fork"],
            cwd=REPO_ROOT,
            text=True,
        )
        models_out = subprocess.check_output(
            ["bash", "src/scripts/start_line_following.sh", "--list-models"],
            cwd=REPO_ROOT,
            text=True,
        )
        self.assertRegex(ring_out, r"1\)\s+\[默认推荐基线\]\s+YOLO_RING_BASE\.yaml")
        self.assertRegex(fork_out, r"1\)\s+\[默认推荐基线\]\s+YOLO_FORK_BASE\.yaml")
        self.assertIn("best_modified_yolov8.bin", models_out)
        self.assertIn("best_modified_yolov8_v3.bin", models_out)
        self.assertIn("best_modified_yolov8s_v5.bin", models_out)


if __name__ == "__main__":
    unittest.main()
