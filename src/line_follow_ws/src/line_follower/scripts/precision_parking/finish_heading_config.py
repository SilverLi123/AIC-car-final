"""Load only the surveyed geometry needed by the optional finish-heading gate."""
import json
from pathlib import Path

from .controller import Calibration
from .geometry import GroundGeometry


def load_finish_heading_config(path):
    """Return row calibration, surveyed ground projection and detector options.

    The finish-heading experiment never performs parking-distance control, so it
    deliberately does not load or reinterpret the braking profile in this file.
    """
    with Path(path).open(encoding='utf-8') as stream:
        data = json.load(stream)
    if data.get('schema_version') != 1:
        raise ValueError('Unsupported finish-heading calibration schema')
    calibration_data = data.get('calibration')
    ground_data = data.get('ground_projection')
    if not isinstance(calibration_data, dict) or type(calibration_data.get('validated')) is not bool:
        raise ValueError('Finish-heading row calibration must have an explicit validation flag')
    if not isinstance(ground_data, dict) or type(ground_data.get('validated')) is not bool:
        raise ValueError('Finish-heading requires a surveyed ground projection')
    if not ground_data['validated']:
        raise ValueError('Finish-heading requires surveyed, validated ground geometry')

    calibration = Calibration(**calibration_data)
    ground = GroundGeometry(ground_data)
    if (ground.width, ground.height) != (calibration.width, calibration.height):
        raise ValueError('Ground projection and row calibration resolution mismatch')
    controller = data.get('controller')
    if not isinstance(controller, dict) or 'front_offset' not in controller:
        raise ValueError('Calibration must include the surveyed controller front_offset')
    if abs(ground.front_offset - float(controller['front_offset'])) > 1e-9:
        raise ValueError('Ground projection front_offset_m disagrees with controller front_offset')

    options = data.get('detector', {})
    allowed = {'saturation_min', 'value_min', 'confirm_frames'}
    if not isinstance(options, dict) or set(options) - allowed:
        raise ValueError('Unsupported finish-heading detector options')
    normalized = {
        'saturation_min': options.get('saturation_min', 35),
        'value_min': options.get('value_min', 80),
        'confirm_frames': options.get('confirm_frames', 3),
    }
    if (type(normalized['saturation_min']) is not int or not 0 <= normalized['saturation_min'] <= 255 or
            type(normalized['value_min']) is not int or not 0 <= normalized['value_min'] <= 255 or
            type(normalized['confirm_frames']) is not int or not 2 <= normalized['confirm_frames'] <= 10):
        raise ValueError('Invalid finish-heading detector settings')
    return calibration, ground, normalized
