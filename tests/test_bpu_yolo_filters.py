import importlib.util
import pathlib
import sys
import types
import unittest


def _load_bpu_module():
    stub_names = [
        "cv2",
        "cv_bridge",
        "hobot_dnn",
        "rclpy",
        "rclpy.executors",
        "rclpy.node",
        "rcl_interfaces",
        "rcl_interfaces.msg",
        "sensor_msgs",
        "sensor_msgs.msg",
        "ai_msgs",
        "ai_msgs.msg",
    ]
    saved = {name: sys.modules.get(name) for name in stub_names}
    try:
        for name in stub_names:
            if name not in sys.modules:
                sys.modules[name] = types.ModuleType(name)
        sys.modules["cv_bridge"].CvBridge = object
        sys.modules["hobot_dnn"].pyeasy_dnn = types.ModuleType("pyeasy_dnn")
        sys.modules["rclpy.executors"].ExternalShutdownException = Exception
        sys.modules["rclpy.node"].Node = object
        sys.modules["rcl_interfaces.msg"].SetParametersResult = object
        sys.modules["sensor_msgs.msg"].Image = object
        sys.modules["sensor_msgs.msg"].RegionOfInterest = object
        sys.modules["ai_msgs.msg"].PerceptionTargets = object
        sys.modules["ai_msgs.msg"].Target = object
        sys.modules["ai_msgs.msg"].Roi = object

        mod_path = pathlib.Path(__file__).resolve().parents[1] / "src" / "yolov8" / "bpu_yolov8_node.py"
        spec = importlib.util.spec_from_file_location("bpu_yolov8_node", mod_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        for name, old in saved.items():
            if old is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old


_bpu_mod = _load_bpu_module()
filter_detection_box = _bpu_mod.filter_detection_box


class TestBpuYoloFilters(unittest.TestCase):
    def test_default_filter_preserves_legacy_min_box_behavior(self):
        # Even top-of-frame or wide boxes are kept when enable_roi_geo_filter=False
        self.assertTrue(
            filter_detection_box(10, 5, 200, 30, 640, 480, min_box_size=6, enable_roi_geo_filter=False)
        )
        self.assertFalse(
            filter_detection_box(10, 5, 5, 12, 640, 480, min_box_size=6, enable_roi_geo_filter=False)
        )

    def test_roi_geo_filter_rejects_top_out_of_track_and_oversized_and_skinny(self):
        # Normal on-track circular digit in lower half
        self.assertTrue(filter_detection_box(280, 250, 42, 36, 640, 480, enable_roi_geo_filter=True))
        # Top background false positive (cy=40 -> 0.083 < 0.18)
        self.assertFalse(filter_detection_box(280, 20, 40, 40, 640, 480, enable_roi_geo_filter=True))
        # Bottom edge clipped artifact (cy=476 -> 0.991 > 0.98)
        self.assertFalse(filter_detection_box(280, 466, 30, 20, 640, 480, enable_roi_geo_filter=True))
        # Oversized false positive (> 35% of frame width)
        self.assertFalse(filter_detection_box(100, 240, 260, 120, 640, 480, enable_roi_geo_filter=True))
        # Extreme skinny horizontal strip (aspect=5.0 > 3.0)
        self.assertFalse(filter_detection_box(200, 260, 100, 20, 640, 480, enable_roi_geo_filter=True))
        # Extreme vertical bar (aspect=0.2 < 0.35)
        self.assertFalse(filter_detection_box(200, 240, 10, 50, 640, 480, enable_roi_geo_filter=True))


if __name__ == "__main__":
    unittest.main()
