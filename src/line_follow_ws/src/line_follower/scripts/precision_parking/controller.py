"""Deterministic parking logic. No ROS imports, wall-clock calls or publishers."""
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional


@dataclass
class Calibration:
    width: int
    height: int
    points: list
    error_m: float
    validated: bool

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0 or len(self.points) < 2:
            raise ValueError('Calibration needs dimensions and at least two measured points')
        if not math.isfinite(self.error_m) or self.error_m < 0:
            raise ValueError('Invalid calibration uncertainty')
        for i, (row, distance) in enumerate(self.points):
            if not all(math.isfinite(x) for x in (row, distance)) or not 0 <= row < self.height or distance <= 0:
                raise ValueError('Invalid calibration point')
            if i and not (row > self.points[i - 1][0] and distance < self.points[i - 1][1]):
                raise ValueError('Rows must increase and distances must decrease')

    def distance(self, row, width, height):
        if (width, height) != (self.width, self.height) or not math.isfinite(row):
            return None
        for (r0, d0), (r1, d1) in zip(self.points, self.points[1:]):
            if r0 <= row <= r1:
                return d0 + (d1 - d0) * (row - r0) / (r1 - r0)
        return None


@dataclass
class Config:
    max_speed: float = .30
    calibrated_speed: float = .30
    deceleration: float = .20
    reaction_time: float = .15
    stop_margin: float = .075
    preview_speed: float = .15
    depart_speed: float = .12
    acceleration: float = .25
    still_v: float = .01
    still_w: float = .03
    still_time: float = .30
    still_samples: int = 4
    still_sample_gap: float = .15
    stream_timeout: float = .30
    prediction_timeout: float = .25
    rearm_distance: float = .18
    clear_time: float = .30
    depart_timeout: float = 8.
    depart_distance_limit: float = .80
    front_offset: float = .0
    unreliable_creep_distance: float = .25
    unreliable_creep_time: float = 3.
    # Deprecated: accepted for compatibility with old JSON only. Integrated parking
    # never uses displacement/elapsed time to authorize departure or final stillness.
    hold_settle_time: float = 3.
    hold_creep_limit: float = .02

    def __post_init__(self):
        for key, value in vars(self).items():
            if not math.isfinite(value):
                raise ValueError('Invalid parameter: ' + key)
            # Only the measured bumper overhang may legitimately be zero.
            if value <= 0 and key != 'front_offset':
                raise ValueError('Invalid positive parameter: ' + key)
        if self.front_offset < 0:
            raise ValueError('Front offset cannot be negative')
        if self.max_speed > self.calibrated_speed or self.depart_speed > self.max_speed:
            raise ValueError('Speed exceeds validated test envelope')


@dataclass
class Feedback:
    stamp: float
    sequence: int
    v: float
    w: float


@dataclass
class Observation:
    stamp: float
    sequence: int
    visible: bool
    confirmed: bool
    distance_m: Optional[float]
    near_trigger: bool
    error_m: float = .02
    distance_validated: bool = True
    line: Optional[tuple] = None
    geometry_source: str = 'row'
    status: str = ''
    # Proven lower bound when the line is seen beyond the calibrated range. It is a
    # conservative bound including calibration error, not a fabricated exact distance.
    # Cruising additionally requires the actual-speed stopping distance to fit now.
    min_distance_m: Optional[float] = None
    # Independent weak visual hints are never formal count/trigger/speed evidence.
    preview_visible: bool = False
    preview_stable: bool = False
    preview_hits: int = 0
    preview_distance_m: Optional[float] = None
    preview_distance_validated: bool = False
    preview_line: Optional[tuple] = None
    preview_status: str = ''
    preview_fragment_count: int = 0
    preview_match: str = ''
    evidence_kind: str = ''


@dataclass
class Request:
    stamp: float
    v: float
    w: float


@dataclass
class Output:
    v: float = 0.
    w: float = 0.
    phase: str = 'HOLD'
    count: int = 0
    reason: str = ''


class ParkingController:
    def __init__(self, config, calibration, geometry_validated=False):
        if not calibration.validated and not geometry_validated:
            raise ValueError('Active parking requires physically validated calibration')
        self.cfg, self.calibration = config, calibration
        self.count, self.phase, self.enabled = 0, 'CRUISE', False
        self.obs = self.odom = None
        self.last_step = None
        self.travel = 0.
        self.travel_history = deque(maxlen=300)
        self.distance = self.distance_stamp = self.distance_travel = None
        self.depart_origin = self.depart_start = None
        self.clear_since = None
        self.low_since, self.low_samples, self.low_stamp = None, 0, None
        self.last_v, self.reason = 0., ''
        self.final_still = False

    def _reset_still(self):
        self.low_since, self.low_samples, self.low_stamp = None, 0, None

    def set_enabled(self, enabled, now):
        if not math.isfinite(now):
            self._fault('Invalid enable timestamp')
            return
        enabled = bool(enabled)
        if enabled == self.enabled:
            return
        if enabled:
            if self.phase == 'FINAL' and self.final_still:
                self.count, self.phase = 0, 'DEPART'
                self.depart_origin, self.depart_start = self.travel, now
                self.final_still = False
            elif self.phase == 'FAULT':
                # Preserve the event; never resume cruise from an uncertain stop.
                self.phase = 'FINAL' if self.count >= 4 else ('HOLD' if self.count else 'CRUISE')
            elif self.phase in ('APPROACH', 'DEPART'):
                self.phase = 'HOLD'
            self.reason = ''
        self.enabled = enabled
        self.last_step, self.last_v = now, 0.
        self._reset_still()
        # The old distance must never survive a pause/resume.
        self.distance = self.distance_stamp = None
        self.clear_since = None

    def _fault(self, reason):
        self.phase, self.reason = 'FAULT', reason
        self.last_v = 0.
        self.final_still = False

    def observe(self, observation):
        if not math.isfinite(observation.stamp) or (observation.distance_m is not None and
                (not math.isfinite(observation.distance_m) or observation.distance_m <= 0)):
            self._fault('Invalid image/distance')
            return
        if self.obs is not None:
            if observation.sequence == self.obs.sequence:
                return
            if observation.stamp <= self.obs.stamp:
                self._fault('Image timestamp regressed')
                return
        self.obs = observation

    def feedback(self, feedback):
        if not all(math.isfinite(x) for x in (feedback.stamp, feedback.v, feedback.w)):
            self._fault('Invalid odometry')
            return
        previous = self.odom
        if previous is not None:
            if feedback.sequence == previous.sequence:
                return
            dt = feedback.stamp - previous.stamp
            if dt <= 0:
                self._fault('Odometry timestamp regressed')
                return
            if dt <= self.cfg.stream_timeout:
                self.travel += max(0., (previous.v + feedback.v) * .5) * dt
            else:
                self._reset_still()
        self.odom = feedback
        self.travel_history.append((feedback.stamp, self.travel))
        if self.enabled and self.phase in ('HOLD', 'FINAL'):
            gap = feedback.stamp - self.low_stamp if self.low_stamp is not None else 0.
            if abs(feedback.v) <= self.cfg.still_v and abs(feedback.w) <= self.cfg.still_w:
                if self.low_since is None or gap > self.cfg.still_sample_gap:
                    self.low_since, self.low_samples = feedback.stamp, 0
                self.low_samples += 1
                self.low_stamp = feedback.stamp
            else:
                self._reset_still()

    def _out(self, v=0., w=0.):
        self.last_v = v
        return Output(v, w, self.phase, self.count, self.reason)

    def step(self, now, request):
        if not math.isfinite(now):
            self._fault('Invalid clock')
            return self._out()
        dt = now - self.last_step if self.last_step is not None else 0.
        self.last_step = now
        if dt < 0:
            self._fault('Clock regressed')
        if not self.enabled:
            return self._out()
        if self.phase == 'FAULT':
            return self._out()
        if (not all(math.isfinite(x) for x in (request.stamp, request.v, request.w)) or
                request.v < 0 or self.obs is None or self.odom is None):
            self._fault('Invalid request or missing streams')
            return self._out()
        if any(not 0 <= now - stamp <= self.cfg.stream_timeout
               for stamp in (request.stamp, self.obs.stamp, self.odom.stamp)):
            self._fault('Stale or future request/image/odom')
            return self._out()
        if abs(self.odom.v) > self.cfg.calibrated_speed + .05:
            self._fault('Actual speed outside braking calibration')
            return self._out()
        if self.count >= 4 and self.phase not in ('APPROACH', 'FINAL'):
            self.phase = 'FINAL'
        v = min(request.v, self.cfg.max_speed)
        obs = self.obs
        if self.phase == 'CRUISE' and obs.confirmed:
            if obs.distance_m is None and not obs.near_trigger:
                return self._out(min(v, self.cfg.preview_speed), request.w)
            self.count += 1
            self.phase = 'APPROACH'
            self.distance_stamp = None
        if self.phase == 'APPROACH':
            if obs.confirmed and obs.near_trigger:
                self.phase = 'FINAL' if self.count == 4 else 'HOLD'
                self._reset_still()
                return self._out()
            if obs.confirmed and obs.distance_m is not None:
                if self.distance_stamp != obs.stamp:
                    exposure_travel = self._travel_at(obs.stamp)
                    observed_now = obs.distance_m - max(0., self.travel - exposure_travel)
                    predicted = (self.distance - (self.travel - self.distance_travel)
                                 if self.distance is not None else observed_now)
                    self.distance = min(observed_now, predicted)
                    self.distance_stamp, self.distance_travel = obs.stamp, self.travel
            if self.distance_stamp is None or now - self.distance_stamp > self.cfg.prediction_timeout:
                self._fault('Yellow distance lost during approach')
                return self._out()
            # No fresh odom between its last timestamp and this timer tick:
            # deduct the full admissible speed rather than invent no motion.
            unobserved_motion = (self.cfg.calibrated_speed + .05) * max(0., now - self.odom.stamp)
            remaining = self.distance - (self.travel - self.distance_travel) - unobserved_motion
            available = remaining - self.cfg.stop_margin - self.calibration.error_m
            if obs.near_trigger or available <= 0:
                self.phase = 'FINAL' if self.count == 4 else 'HOLD'
                self._reset_still()
                return self._out()
            available -= abs(self.odom.v) * self.cfg.reaction_time
            v = min(v, math.sqrt(2 * self.cfg.deceleration * max(0., available)))
            if v <= 0:
                self.phase = 'FINAL' if self.count == 4 else 'HOLD'
                self._reset_still()
                return self._out()
        if self.phase in ('HOLD', 'FINAL'):
            still = (self.low_since is not None and self.low_samples >= self.cfg.still_samples and
                     self.odom.stamp - self.low_since >= self.cfg.still_time)
            if self.phase == 'FINAL':
                self.final_still = self.final_still or still
                return self._out()
            if still:
                self.phase = 'DEPART'
                self.depart_origin, self.depart_start = self.travel, now
                self.clear_since = None
            return self._out()
        if self.phase == 'DEPART':
            travelled = self.travel - self.depart_origin
            if now - self.depart_start > self.cfg.depart_timeout or travelled > self.cfg.depart_distance_limit:
                self._fault('Departure did not clear yellow within limits')
                return self._out()
            if obs.visible:
                self.clear_since = None
            elif self.clear_since is None:
                self.clear_since = now
            if (travelled >= self.cfg.rearm_distance and self.clear_since is not None and
                    now - self.clear_since >= self.cfg.clear_time):
                self.phase = 'CRUISE'
                self.distance = self.distance_stamp = None
            # Acceleration is bounded during departure including the rearm tick.
            v = min(v, self.cfg.depart_speed, self.last_v + self.cfg.acceleration * max(0., dt))
        return self._out(v, request.w)

    def _travel_at(self, stamp):
        """Interpolate odometry at exposure; before history, use a conservative bound."""
        for (t0, s0), (t1, s1) in zip(self.travel_history, list(self.travel_history)[1:]):
            if t0 <= stamp <= t1:
                return s0 + (s1 - s0) * (stamp - t0) / (t1 - t0)
        first_t, first_s = self.travel_history[0]
        if stamp < first_t:
            return first_s - (self.cfg.calibrated_speed + .05) * (first_t - stamp)
        return self.travel
