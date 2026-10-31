"""Bounded, independent weak-yellow hints. No parking/count policy lives here.

All metric hints pass GroundGeometry and remain preview-only. Image-space matching
never claims an odometric prediction. Source-time SE(2) hints can transform only a
validated ground feature whose projection remains in the surveyed domain.
"""
from dataclasses import dataclass
import heapq
import math
import cv2
import numpy as np


WEAK_DEFAULTS = {
    'enabled': False, 'saturation_min': 12, 'value_min': 80,
    'min_row_fraction': .20, 'min_fragment_width_fraction': .025,
    'max_fragment_height_fraction': .20, 'min_aspect_ratio': 3.,
    'min_area_px': 12., 'max_components': 64, 'merge_gap_fraction': .12,
    'merge_normal_fraction': .015, 'merge_angle_deg': 12.,
    'min_total_width_fraction': .10,
}
ASSOCIATION_DEFAULTS = {
    'enabled': True, 'apply_to_strong': False, 'max_gap_s': .20, 'max_lateral_fraction': .12,
    'max_row_fraction': .10, 'max_angle_deg': 12., 'min_length_ratio': .55,
    'stable_frames': 3,
}


def _options(value, defaults, rules, label):
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError(label + ' must be a mapping')
    unknown = set(value) - set(defaults)
    if unknown:
        raise ValueError(label + ' unknown options: ' + ', '.join(sorted(map(str, unknown))))
    result = dict(defaults)
    result.update(value)
    for key, val in result.items():
        kind, lower, upper = rules[key]
        if kind is bool:
            valid = type(val) is bool
        elif kind is int:
            valid = type(val) is int and lower <= val <= upper
        else:
            valid = (type(val) in (int, float) and math.isfinite(val) and lower <= val <= upper)
            if valid:
                result[key] = float(val)
        if not valid:
            raise ValueError(label + '.' + key + ' has invalid type or range')
    return result


def validate_preview_options(weak_preview=None, association=None):
    """Validate and normalize nested options before constructing any ROS node.

    Unknown keys and bool-as-number values are errors; capped component counts
    bound pair checks to 8128 even at the maximum supported setting (128).
    """
    weak_rules = {
        'enabled': (bool, 0, 1), 'saturation_min': (int, 1, 254),
        'value_min': (int, 0, 255), 'min_row_fraction': (float, 0., .95),
        'min_fragment_width_fraction': (float, .005, .5),
        'max_fragment_height_fraction': (float, .005, .5),
        'min_aspect_ratio': (float, 2., 30.), 'min_area_px': (float, 1., 10000.),
        'max_components': (int, 1, 128), 'merge_gap_fraction': (float, 0., .3),
        'merge_normal_fraction': (float, .001, .05), 'merge_angle_deg': (float, 0., 30.),
        'min_total_width_fraction': (float, .01, .9),
    }
    association_rules = {
        'enabled': (bool, 0, 1), 'apply_to_strong': (bool, 0, 1), 'max_gap_s': (float, .001, 1.),
        'max_lateral_fraction': (float, .001, .5), 'max_row_fraction': (float, .001, .5),
        'max_angle_deg': (float, 0., 45.), 'min_length_ratio': (float, .01, 1.),
        'stable_frames': (int, 2, 30),
    }
    weak = _options(weak_preview, WEAK_DEFAULTS, weak_rules, 'weak_preview')
    association = _options(association, ASSOCIATION_DEFAULTS, association_rules, 'association')
    if weak['min_fragment_width_fraction'] > weak['min_total_width_fraction']:
        raise ValueError('weak_preview fragment width must not exceed total width')
    return weak, association


@dataclass
class _Candidate:
    points: object
    center: object
    direction: object
    length: float
    fragments: int = 1
    distance: object = None
    line: object = None
    validated: bool = False
    status: str = 'preview_only'


def _angle(a, b):
    return math.degrees(math.acos(float(np.clip(abs(np.dot(a, b)), 0., 1.))))


def _candidate(points, fragments=1):
    points = np.asarray(points, float)
    vx, vy, x, y = cv2.fitLine(points.astype(np.float32), cv2.DIST_L2, 0, .01, .01).flatten()
    direction = np.array([vx, vy], float)
    if direction[0] < 0:
        direction = -direction
    projections = points @ direction
    center = np.array([x, y], float) + direction * (
        .5 * (projections.min() + projections.max()) - np.dot([x, y], direction))
    return _Candidate(points, center, direction, float(np.ptp(projections)), fragments)


class WeakYellowPreview:
    """Single-worker-owned state; only detached scalar diagnostics leave it."""
    def __init__(self, weak, association, strong_saturation_min, ground):
        self.options, self.association = weak, association
        self.strong_saturation_min = strong_saturation_min
        self.ground = ground
        self.previous = None
        self.hits = 0
        self._mask_size = None
        self._domain = None
        self.diagnostics = {'weak_components': 0, 'weak_pair_checks': 0,
                            'weak_candidates': 0, 'motion_prediction': 'unused'}

    def _domain_mask(self, width, height):
        if self._mask_size == (width, height):
            return self._domain
        domain = np.zeros((height, width), np.uint8)
        ground = self.ground
        if ground is None:
            domain[int(height * self.options['min_row_fraction']):] = 255
        elif (width, height) == (ground.width, ground.height):
            # The polygon is the surveyed domain. Its bounding rectangle is never
            # substituted for missing ground calibration. Erode raster boundaries
            # so rounding cannot admit pixels just outside the actual polygon.
            hull_mask = np.zeros_like(domain)
            cv2.fillPoly(hull_mask, [np.round(ground.hull).astype(np.int32)], 255)
            hull_mask = cv2.erode(hull_mask, np.ones((3, 3), np.uint8))
            if ground.spec.get('pixel_space') == 'rectified':
                xx, yy = np.meshgrid(np.arange(width), np.arange(height))
                raw = np.column_stack([xx.ravel(), yy.ravel()]).astype(float)
                rectified = ground.rectified(raw).reshape(height, width, 2).astype(np.float32)
                domain = cv2.remap(hull_mask, rectified[:, :, 0], rectified[:, :, 1],
                                   cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
            else:
                domain = hull_mask
        self._mask_size, self._domain = (width, height), domain
        return domain

    def _fragments(self, hsv, domain):
        opt = self.options
        height, width = hsv.shape[:2]
        mask = cv2.inRange(hsv, (20, opt['saturation_min'], opt['value_min']), (40, 255, 255))
        mask = cv2.bitwise_and(mask, domain)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Keep only the largest bounded set before fitLine or pair processing.
        contours = heapq.nlargest(opt['max_components'], contours, key=cv2.contourArea)
        fragments = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if (w < width * opt['min_fragment_width_fraction'] or
                    h > height * opt['max_fragment_height_fraction'] or area < opt['min_area_px']):
                continue
            sides = cv2.minAreaRect(contour)[1]
            if min(sides) <= 0 or max(sides) / min(sides) < opt['min_aspect_ratio']:
                continue
            # Long, strong contours already belong to the established baseline.
            # Small saturated fragments may still be useful to this independent path.
            edge_fragment = x <= .03 * width or x + w >= .97 * width
            strong_width = width * (.10 if edge_fragment else .18)
            local = np.zeros((h, w), np.uint8)
            cv2.drawContours(local, [contour - np.array([[[x, y]]])], -1, 255, -1)
            if w >= strong_width:
                saturation = hsv[y:y+h, x:x+w, 1][local != 0]
                if np.median(saturation) >= self.strong_saturation_min:
                    continue
            columns = [col for col in range(0, w, 2) if np.any(local[:, col])]
            points = np.array([[x + col, y + np.flatnonzero(local[:, col])[-1]] for col in columns], float)
            if len(points) < 2:
                continue
            candidate = _candidate(points)
            # A wall edge parallel to travel is not a transverse yellow fragment.
            if abs(candidate.direction[0]) < .70:
                continue
            fragments.append(candidate)
        return sorted(fragments, key=lambda c: float(c.center[0]))

    def _merge(self, fragments, width, height):
        opt = self.options
        count = len(fragments)
        aligned = np.eye(count, dtype=bool)
        adjacent = np.eye(count, dtype=bool)
        pair_checks = 0
        for i, first in enumerate(fragments):
            for j in range(i + 1, count):
                second = fragments[j]
                pair_checks += 1
                delta = second.center - first.center
                direction = first.direction
                normal = np.array([-direction[1], direction[0]])
                compatible = (_angle(direction, second.direction) <= opt['merge_angle_deg'] and
                              abs(float(delta @ normal)) <= height * opt['merge_normal_fraction'])
                aligned[i, j] = aligned[j, i] = compatible
                gap = abs(float(delta @ direction)) - .5 * (first.length + second.length)
                adjacent[i, j] = adjacent[j, i] = compatible and gap <= width * opt['merge_gap_fraction']
        # Greedy bounded grouping: every member must be aligned with every other
        # member, and a new member must adjoin the group. This prevents drifting
        # transitive chains and never enumerates fragment subsets/combinations.
        groups = []
        for index in range(count):
            for group in groups:
                if all(aligned[index, other] for other in group) and any(adjacent[index, other] for other in group):
                    group.append(index)
                    break
            else:
                groups.append([index])
        candidates = [_candidate(np.concatenate([fragments[i].points for i in group]), len(group))
                      for group in groups]
        candidates = [c for c in candidates if c.length >= width * opt['min_total_width_fraction']]
        self.diagnostics.update(weak_components=count, weak_pair_checks=pair_checks,
                                weak_candidates=len(candidates))
        return candidates

    def _matches(self, old, new, width, height):
        opt = self.association
        return (abs(float(new.center[0] - old.center[0])) <= width * opt['max_lateral_fraction'] and
                abs(float(new.center[1] - old.center[1])) <= height * opt['max_row_fraction'] and
                _angle(new.direction, old.direction) <= opt['max_angle_deg'] and
                min(new.length, old.length) / max(new.length, old.length, 1.) >= opt['min_length_ratio'])

    def _pose_hint(self, hint, stamp):
        """Copy a source-time tracker pose, using its x-right/y-forward convention."""
        ground = self.ground
        if ground is None or not ground.validated or not isinstance(hint, dict):
            return None
        try:
            hint_stamp = hint['stamp']
            offset = hint['front_offset_m']
            values = hint['pose']
            if (type(hint_stamp) not in (int, float) or type(offset) not in (int, float) or
                    len(values) != 3 or any(type(v) not in (int, float) for v in values)):
                return None
            pose = np.asarray(values, float)
            if (not math.isfinite(hint_stamp) or not math.isfinite(offset) or not np.isfinite(pose).all() or
                    abs(hint_stamp - stamp) > 1e-6 or abs(offset - ground.front_offset) > 1e-6):
                return None
            valid_from = hint.get('valid_from_stamp')
            if valid_from is not None and (type(valid_from) not in (int, float) or
                    not math.isfinite(valid_from) or valid_from > stamp):
                return None
            return pose.copy(), float(hint_stamp), valid_from
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def _rotation(yaw):
        return np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])

    def _ground_prediction(self, previous, old_pose, new_pose, width, height):
        """Predict an observed feature, refusing unsupported geometry at every step.

        Both source-time poses come from the owner's odometry tracker. Projecting
        guessed rows or scalar row-table distances into the world is prohibited.
        This result is used for association only; it never creates a new observation.
        """
        ground = self.ground
        if (ground is None or not ground.validated or not previous.validated or
                old_pose is None or new_pose is None or
                (width, height) != (ground.width, ground.height)):
            return None
        old_pose, old_stamp, _ = old_pose
        new_pose, _, valid_from = new_pose
        # The tracker's world origin changes after an odometry gap/reset. Source
        # time alone cannot join coordinates across that reset. Legacy manually
        # supplied hints without this marker assert one continuous world frame.
        if valid_from is not None and valid_from > old_stamp:
            return None
        # A discontinuous odometry reset cannot be a short-horizon association.
        if (np.linalg.norm(new_pose[:2] - old_pose[:2]) > .5 or
                abs(math.remainder(float(new_pose[2] - old_pose[2]), 2 * math.pi)) > .5):
            return None
        p = ground.rectified(previous.points)
        homogeneous = np.column_stack([p, np.ones(len(p))]) @ ground.H.T
        if not np.isfinite(homogeneous).all() or np.any(np.abs(homogeneous[:, 2]) < 1e-8):
            return None
        bumper = homogeneous[:, :2] / homogeneous[:, 2, None]
        base = bumper + np.array([0., ground.front_offset])
        world = base @ self._rotation(old_pose[2]).T + old_pose[:2]
        current_base = (world - new_pose[:2]) @ self._rotation(new_pose[2])
        current_bumper = current_base - np.array([0., ground.front_offset])
        projected = np.column_stack([current_bumper, np.ones(len(p))]) @ np.linalg.inv(ground.H).T
        if not np.isfinite(projected).all() or np.any(np.abs(projected[:, 2]) < 1e-8):
            return None
        rectified = projected[:, :2] / projected[:, 2, None]
        if ground.spec.get('pixel_space') == 'rectified':
            rays = np.column_stack([rectified, np.ones(len(rectified))]) @ np.linalg.inv(ground.K).T
            if np.any(np.abs(rays[:, 2]) < 1e-8):
                return None
            rays = rays / rays[:, 2, None]
            raw, _ = cv2.projectPoints(rays, np.zeros(3), np.zeros(3), ground.K, ground.distortion)
            pixels = raw.reshape(-1, 2)
        else:
            pixels = rectified
        if (not np.isfinite(pixels).all() or np.any(pixels[:, 0] < 0.) or
                np.any(pixels[:, 0] > width - 1) or np.any(pixels[:, 1] < 0.) or
                np.any(pixels[:, 1] > height - 1)):
            return None
        indices = np.round(pixels).astype(int)
        if not np.all(self._domain_mask(width, height)[indices[:, 1], indices[:, 0]]):
            return None
        estimate = ground.measure(pixels, width, height)
        if not estimate.validated or estimate.distance_m is None:
            return None
        return _candidate(pixels, previous.fragments)

    def process(self, hsv, stamp, motion_hint=None):
        height, width = hsv.shape[:2]
        self.diagnostics['motion_prediction'] = 'unused'
        candidates = self._merge(self._fragments(hsv, self._domain_mask(width, height)), width, height)
        return self._associate_candidates(candidates, stamp, width, height, motion_hint)

    def associate_feature(self, points, stamp, width, height, motion_hint=None):
        """Associate a baseline-selected strong contour without changing extraction.

        Use a separate instance from weak preview, so weak hints cannot contribute
        hits or geometry to a strong contour's formal confirmation policy.
        """
        self.diagnostics['motion_prediction'] = 'unused'
        candidates = [_candidate(points)] if points is not None and len(points) >= 2 else []
        return self._associate_candidates(candidates, stamp, width, height, motion_hint)

    def _associate_candidates(self, candidates, stamp, width, height, motion_hint):
        for candidate in candidates:
            if self.ground is not None:
                estimate = self.ground.measure(candidate.points, width, height)
                candidate.status = estimate.status
                # Unvalidated projections and lower bounds never masquerade as a
                # metric preview. Formal evidence never sees even validated hints.
                if estimate.validated and estimate.distance_m is not None:
                    candidate.distance, candidate.line = estimate.distance_m, estimate.line
                    candidate.validated = True
        continuous = (self.previous is not None and
                      0 < stamp - self.previous[0] <= self.association['max_gap_s'] and
                      self.previous[2] == (width, height))
        pose = self._pose_hint(motion_hint, stamp)
        prediction = (self._ground_prediction(self.previous[1], self.previous[3], pose, width, height)
                      if continuous and self.association['enabled'] else None)
        if prediction is not None:
            self.diagnostics['motion_prediction'] = 'ground_supported'
        reference = prediction if prediction is not None else (self.previous[1] if continuous else None)
        matches = ([c for c in candidates if (prediction is None or c.validated) and
                    self._matches(reference, c, width, height)]
                   if continuous and self.association['enabled'] else [])
        selected = max(matches or candidates, key=lambda c: (c.center[1], c.length), default=None)
        matched = selected is not None and bool(matches)
        self.hits = self.hits + 1 if matched else int(selected is not None)
        self.previous = (stamp, selected, (width, height), pose) if selected is not None else None
        result = {'preview_visible': selected is not None,
                  'preview_stable': self.hits >= self.association['stable_frames'],
                  'preview_hits': self.hits, 'preview_match': ('ground_predicted' if prediction is not None else 'matched')
                  if matched else ('new' if selected else ''),
                  'preview_distance_m': None, 'preview_distance_validated': False,
                  'preview_line': None, 'preview_status': 'not_visible', 'preview_fragment_count': 0}
        if selected:
            result.update(preview_distance_m=selected.distance, preview_distance_validated=selected.validated,
                          preview_line=selected.line, preview_status=selected.status,
                          preview_fragment_count=selected.fragments)
        return result
