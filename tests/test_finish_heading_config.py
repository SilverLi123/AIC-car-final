import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'src/line_follow_ws/src/line_follower/scripts'
sys.path.insert(0, str(SCRIPTS))
from precision_parking.finish_heading_config import load_finish_heading_config


class FinishHeadingConfigTests(unittest.TestCase):
    def setUp(self):
        self.path = ROOT / 'src/line_follow_ws/src/line_follower/config/precision_parking/ground_checked_20261030.json'

    def test_surveyed_geometry_loads_for_heading_experiment(self):
        calibration, ground, options = load_finish_heading_config(self.path)
        self.assertEqual((calibration.width, calibration.height), (640, 480))
        self.assertTrue(ground.validated)
        self.assertEqual((ground.width, ground.height), (calibration.width, calibration.height))
        self.assertEqual(options, {'saturation_min': 35, 'value_min': 80, 'confirm_frames': 3})

    def test_unvalidated_geometry_is_rejected(self):
        data = json.loads(self.path.read_text())
        data['ground_projection']['validated'] = False
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', encoding='utf-8') as stream:
            json.dump(data, stream)
            stream.flush()
            with self.assertRaisesRegex(ValueError, 'surveyed'):
                load_finish_heading_config(stream.name)

    def test_disagreeing_bumper_offsets_are_rejected(self):
        data = json.loads(self.path.read_text())
        data['ground_projection']['front_offset_m'] += .01
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', encoding='utf-8') as stream:
            json.dump(data, stream)
            stream.flush()
            with self.assertRaisesRegex(ValueError, 'front_offset'):
                load_finish_heading_config(stream.name)


if __name__ == '__main__':
    unittest.main()
