"""Bounded latest-input/latest-result image processing; no controller or ROS access."""
from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class FrameResult:
    stamp: float
    sequence: int
    value: object
    started: float
    completed: float
    error: str = ''


class LatestFrameWorker:
    def __init__(self, process, max_age=.30):
        self.process, self.max_age = process, max_age
        self._condition = threading.Condition(); self._stop = False
        self._pending = self._latest = None
        self._last_sequence = None; self._error = ''
        self._busy_stamp = None; self._started = None
        self._input_dropped = self._result_dropped = self._rejected = self._processed = 0
        self._stale_dropped = self._out_of_order = self._result_too_old = 0
        # Bounded latency evidence for recoverable drops; never a replacement
        # for the serious-error latch below.
        self._drop_stats = {name: {'count': 0, 'max_ms': 0., 'buckets': [0, 0, 0, 0]}
                            for name in ('stale_input', 'out_of_order', 'backlog', 'result_too_old')}
        self._thread = threading.Thread(target=self._run, name='parking-image-worker', daemon=True)

    def start(self):
        self._thread.start()
        return self

    def _drop(self, reason, age_s):
        """Record a recoverable drop; caller already holds the condition lock."""
        stats = self._drop_stats[reason]
        age_s = max(0., age_s)
        stats['count'] += 1
        stats['max_ms'] = max(stats['max_ms'], age_s * 1000.)
        bucket = 0 if age_s <= .05 else 1 if age_s <= .10 else 2 if age_s <= .20 else 3
        stats['buckets'][bucket] += 1

    def submit(self, message, stamp, sequence, motion_hint=None):
        with self._condition:
            if self._stop or self._error:
                self._rejected += 1
                return False
            if self._last_sequence is not None and sequence <= self._last_sequence:
                self._out_of_order += 1
                self._drop('out_of_order', 0.)
                return False
            self._last_sequence = sequence
            if self._pending is not None:
                self._input_dropped += 1
                self._drop('backlog', time.monotonic()-self._pending[1])
            # Detach the source-time pose from the controller owner's snapshot.
            hint = (dict(motion_hint, pose=tuple(motion_hint['pose'])) if motion_hint is not None else None)
            self._pending = (message, stamp, sequence, hint)
            self._condition.notify()
            return True

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stop or self._pending is not None)
                if self._stop: return
                message, stamp, sequence, hint = self._pending; self._pending = None
                started = time.monotonic(); self._busy_stamp = stamp; self._started = started
            error = ''; value = None
            if not 0 <= started-stamp <= self.max_age:
                # A queued frame that expired before processing is a dropped
                # frame, not a worker failure: one slow frame must not stop all
                # later perception.
                with self._condition:
                    self._busy_stamp = self._started = None
                    self._stale_dropped += 1
                    self._drop('stale_input', started-stamp)
                continue
            try:
                value = (self.process(message, stamp, sequence, hint) if hint is not None
                         else self.process(message, stamp, sequence))
            except Exception as exc:
                error = type(exc).__name__ + ': ' + str(exc)
            result = FrameResult(stamp, sequence, value, started, time.monotonic(), error)
            with self._condition:
                self._busy_stamp = self._started = None
                self._processed += 1
                if self._stop: return
                if not error and result.completed-result.stamp > self.max_age:
                    # A completed result whose source frame is already outside
                    # the freshness window is unusable: drop it, count it, and
                    # keep serving later frames. The controller still owns the
                    # stale-observation fault decision.
                    self._result_too_old += 1
                    self._drop('result_too_old', result.completed-result.stamp)
                    continue
                if self._latest is not None: self._result_dropped += 1
                self._latest = result
                if error:
                    self._error = error; self._pending = None
                    return

    def take(self):
        with self._condition:
            result, self._latest = self._latest, None
            return result

    def snapshot(self):
        with self._condition:
            now = time.monotonic()
            return {'alive': self._thread.is_alive(), 'error': self._error,
                    'input_pending': self._pending is not None, 'result_pending': self._latest is not None,
                    'input_dropped': self._input_dropped, 'result_dropped': self._result_dropped,
                    'rejected': self._rejected, 'processed': self._processed,
                    'stale_dropped': self._stale_dropped,
                    'dropped_stale': self._stale_dropped,
                    'dropped_out_of_order': self._out_of_order,
                    'dropped_backlog': self._input_dropped,
                    'dropped_result_replaced': self._result_dropped,
                    'result_too_old': self._result_too_old,
                    'drop_stats': {name: {'count': value['count'], 'max_ms': value['max_ms'],
                                          'buckets': list(value['buckets'])}
                                   for name, value in self._drop_stats.items()},
                    'processing_age_s': now-self._busy_stamp if self._busy_stamp is not None else None,
                    'processing_elapsed_s': now-self._started if self._started is not None else None}

    def close(self, timeout=.2):
        with self._condition:
            self._stop = True; self._pending = self._latest = None; self._condition.notify()
        if self._thread.ident is not None: self._thread.join(max(0., timeout))
        return not self._thread.is_alive()
