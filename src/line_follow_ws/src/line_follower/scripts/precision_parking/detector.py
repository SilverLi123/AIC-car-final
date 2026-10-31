"""Independent yellow preview. Never reads or changes the white-road mask."""
import math
import cv2
import numpy as np
from .controller import Observation
from .weak_preview import WeakYellowPreview, validate_preview_options


class YellowDetector:
    # 独立黄线感知：只生成观测，不修改白路掩码，也不发布速度。
    #
    # review重点：visible（可见）、confirmed（多帧确认）和distance_validated
    # （米制距离可信）是三件事；弱预警不能直接变成正式停车计数。
    def __init__(self, calibration, saturation_min=35, value_min=80, confirm_frames=3, ground=None,
                 weak_preview=None, association=None):
        self.calibration = calibration
        self.ground = ground
        self.saturation_min, self.value_min = saturation_min, value_min
        self.confirm_frames = confirm_frames
        self.previous = None
        self.hits = 0
        self.row = None
        self.metric_row = None
        self.width_fraction = 0.
        self.last_result = None
        self.ground_points = None
        weak, association = validate_preview_options(weak_preview, association)
        if weak['enabled'] and weak['saturation_min'] >= saturation_min:
            raise ValueError('weak_preview.saturation_min must be lower than strong saturation_min')
        self.weak_preview = (WeakYellowPreview(weak, association, saturation_min, ground)
                             if weak['enabled'] else None)
        self.strong_association = (WeakYellowPreview(weak, association, saturation_min, ground)
                                   if association['enabled'] and association['apply_to_strong'] else None)
        self._diagnostics = {'weak_enabled': weak['enabled'], 'weak_components': 0,
                             'weak_pair_checks': 0, 'weak_candidates': 0,
                             'motion_prediction': 'unused',
                             'strong_association_enabled': self.strong_association is not None,
                             'strong_match': '', 'strong_motion_prediction': 'unused', 'strong_hits': 0}

    def diagnostic_snapshot(self):
        """Return detached diagnostics; no worker-owned tracking state escapes."""
        return dict(self._diagnostics)

    def process(self, image, stamp, sequence, motion_hint=None):
        if self.previous is not None and sequence == self.previous[1]:
            return self.last_result
        if image is None or image.ndim != 3 or image.shape[2] != 3 or not math.isfinite(stamp):
            raise ValueError('Expected BGR image and finite timestamp')
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (20, self.saturation_min, self.value_min), (40, 255, 255))
        # Independent full road preview; deliberately unrelated to follower ROI.
        mask[:int(height * .20)] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 9), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            edge_fragment = x <= .03 * width or x + w >= .97 * width
            min_width = .10 * width if edge_fragment else .18 * width
            if w < min_width or h > .45 * height or w < 1.15 * h:
                continue
            if cv2.contourArea(contour) < .06 * w * h:
                continue
            sides=cv2.minAreaRect(contour)[1]
            if min(sides)<=0 or max(sides)/min(sides)<4.:
                continue
            # Median of each column's near edge resists a single dangling blob.
            local = np.zeros((h, w), np.uint8)
            shifted = contour - np.array([[[x, y]]])
            cv2.drawContours(local, [shifted], -1, 255, -1)
            columns = [y + np.flatnonzero(local[:, col])[-1]
                       for col in range(w) if np.any(local[:, col])]
            row = float(np.median(columns))
            vx, vy, _, _ = cv2.fitLine(contour, cv2.DIST_L2, 0, .01, .01).flatten()
            left, right = int(width * .475), int(width * .525)
            metric_row = None
            if x <= left and x + w > right and abs(float(vy)) <= .18 * abs(float(vx)):
                center_edges = [y + np.flatnonzero(local[:, col - x])[-1]
                                for col in range(left, right)
                                if np.any(local[:, col - x])]
                if len(center_edges) == right - left:
                    metric_row = float(np.median(center_edges))
            edge_points=np.array([[x+col,y+np.flatnonzero(local[:,col])[-1]]
                                  for col in range(0,w,3) if np.any(local[:,col])],float)
            candidates.append((row, w / width, metric_row,edge_points))
        supported=[]
        if self.ground is not None:
            for item in candidates:
                estimate=self.ground.measure(item[3],width,height)
                if estimate.distance_m is not None:supported.append((estimate.distance_m,item))
        selected=min(supported,key=lambda item:item[0])[1] if supported else max(
            candidates,default=(None,0.,None,None),key=lambda item:item[0])
        self.row, self.width_fraction, self.metric_row, self.ground_points = selected
        visible = self.row is not None
        continuous = (self.previous is not None and 0 < stamp - self.previous[0] <= .20 and
                      self.previous[2] is not None and visible and
                      abs(self.row - self.previous[2]) <= height * .15)
        if self.strong_association is None:
            self.hits = self.hits + 1 if visible and continuous else int(visible)
        else:
            associated = self.strong_association.associate_feature(
                self.ground_points, stamp, width, height, motion_hint)
            self.hits = associated['preview_hits']
            self._diagnostics = dict(self._diagnostics, strong_hits=self.hits,
                                     strong_match=associated['preview_match'],
                                     strong_motion_prediction=self.strong_association.diagnostics['motion_prediction'])
        # A multi-frame hit above the top quarter is still only a preview: wall
        # signs and edge reflections live there, not on the road. Keep visible
        # semantics unchanged, but never turn such a candidate into a formal line.
        on_road = self.row is not None and self.row >= height * .25
        confirmed = self.hits >= self.confirm_frames and on_road
        distance = self.calibration.distance(self.metric_row, width, height) if self.metric_row is not None else None
        # Rows above the table top are provably FARTHER than the farthest calibrated entry;
        # the table is monotonic in (row up -> distance down), so this is a measurement of a
        # lower bound rather than an extrapolated distance.
        lower = None
        if (distance is None and self.metric_row is not None and
                (width, height) == (self.calibration.width, self.calibration.height) and
                self.metric_row < self.calibration.points[0][0]):
            lower = max(0.,self.calibration.points[0][1]-self.calibration.error_m)
        correct_size = (width, height) == (self.calibration.width, self.calibration.height)
        near = visible and correct_size and self.row >= self.calibration.points[-1][0]
        error=self.calibration.error_m;validated=self.calibration.validated
        line=None;source='row';status='row' if distance is not None else (
            'beyond_row_range' if lower is not None else 'preview_only')
        if self.ground is not None and visible:
            estimate=self.ground.measure(self.ground_points,width,height)
            distance,line,error,validated=estimate.distance_m,estimate.line,estimate.error_m,estimate.validated
            source,status='ground',estimate.status
            lower=estimate.min_distance_m
            near=False  # A pixel row is not a calibrated vehicle-footprint distance.
        out = Observation(stamp, sequence, visible, confirmed, distance, bool(near),
                          error,validated,line,source,status,lower)
        if self.weak_preview is not None:
            for name, value in self.weak_preview.process(hsv, stamp, motion_hint).items():
                setattr(out, name, value)
            self._diagnostics = dict(self._diagnostics, **self.weak_preview.diagnostics)
        if visible:
            if confirmed and distance is None:
                out.evidence_kind = 'strong_confirmed_unmetric'
            elif distance is not None:
                out.evidence_kind = 'strong_metric_validated' if validated else 'strong_metric_unvalidated'
            else:
                out.evidence_kind = 'strong_unconfirmed'
        else:
            out.evidence_kind = 'weak_preview' if out.preview_visible else 'none'
        self.previous, self.last_result = (stamp, sequence, self.row), out
        return out
