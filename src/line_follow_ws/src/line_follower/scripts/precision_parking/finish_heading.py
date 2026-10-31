"""终点航向实验：只收敛航向，不控制横向中心，不覆盖原版停车。

地面坐标x向右、y向前；ROS角速度向左为正。角度由已知地面直线
的法向推导，不能拿像素中的斜率直接当车身偏角。所有数值仍待实车验收。
"""
import math


def heading_error(line):
    """地面直线ax+by+c=0的朝前法向，转成ROS的有符号航向误差。"""
    a, b, c = map(float, line)
    norm = math.hypot(a, b)
    if not all(math.isfinite(x) for x in (a, b, c)) or norm < 1e-9:
        raise ValueError('Invalid ground line')
    if b < 0: a, b = -a, -b
    if b / norm < .35:
        raise ValueError('Line nearly parallel to vehicle travel')
    return -math.atan2(a, b)


class FinishHeading:
    """小门控，输入原版速度请求，输出(v,w,可观察原因)。

    count来自原巡线日志，不另做黄线计数。启动时必须从第一根开始观察；
    漏计数锁停，第四次永久锁停，重启本节点才解除。暂停不重置原计数。
    """
    terminal_speed = .12
    odom_timeout = .15
    image_timeout = .15
    departure_m = .25
    tolerance = math.radians(5.)
    max_curvature = 1.5
    max_original_yaw = .30
    # 以已有0.3m/s约0.17~0.18m停距为保守占位预算，不向低速外推v²模型。
    # 它不是本门控/当前底盘的物理停距承诺；本模块仍保留原停车触发。
    stop_budget_m = .18
    final_gap_m = .075
    extra_margin_m = .04

    def __init__(self):
        self.started = False
        self.count = 0
        self.fault = False
        self.finished = False
        self.cleared = False
        self.travel = 0.
        self.odom = None
        self.observation = None
        self.previous_observation = None
        self.hits = 0

    def set_started(self, enabled):
        self.started = bool(enabled)
        if not enabled:
            self.observation = self.previous_observation = None
            self.hits = 0
            self.odom = None

    def on_count(self, count, total):
        if count == 4:
            self.finished = True
        if total != 4 or count != self.count + 1:
            self.fault = True
            return
        self.count = count
        self.cleared = False
        self.travel = 0.
        self.odom = None
        self.observation = self.previous_observation = None
        self.hits = 0

    def on_cleared(self):
        if self.count == 3:
            self.cleared = True

    def feedback(self, stamp, speed):
        if not all(math.isfinite(x) for x in (stamp, speed)):
            self.odom = None
            return
        if self.odom is not None:
            old_t, old_v = self.odom
            if stamp <= old_t: return
            dt = stamp - old_t
            if self.count == 3 and self.cleared and dt <= .2:
                self.travel += .5 * (max(0., old_v) + max(0., speed)) * dt
        self.odom = (stamp, speed)

    def observe(self, stamp, sequence, line, distance, error, validated, confirmed):
        previous = self.previous_observation
        if previous is not None and (sequence == previous[1] or stamp <= previous[0]):
            return
        angle = None
        if (self.count == 3 and self.cleared and self.travel >= self.departure_m and
                line is not None and distance is not None and validated and confirmed and
                all(math.isfinite(x) for x in (stamp, distance, error)) and
                error >= 0 and distance > error):
            try: angle = heading_error(line)
            except (ValueError, TypeError): pass
        continuous = (previous is not None and 0 < stamp - previous[0] <= .15 and
                      previous[2] is not None and angle is not None and
                      abs(angle - previous[2]) <= math.radians(5.))
        self.hits = self.hits + 1 if continuous else int(angle is not None)
        self.previous_observation = (stamp, sequence, angle)
        self.observation = ((stamp, angle, distance, error)
                            if angle is not None and self.hits >= 3 else None)

    def step(self, now, v, w, tracking=True):
        if not all(math.isfinite(x) for x in (now, v, w)) or v < 0:
            return 0., 0., 'invalid_request'
        if not self.started or self.fault or self.finished:
            return 0., 0., 'paused_or_locked'
        # 停车是硬优先级：不保留yaw，不在原版已停之后自行前进或原地转向。
        if v == 0:
            return 0., 0., 'upstream_stop'
        if self.count != 3:
            return v, w, 'baseline'
        requested_v = v
        v = min(v, self.terminal_speed)
        # Twist的w是车身角速度。只降v不降w会缩小Ackermann转弯半径，
        # 所以未进入航向修正时等比例缩放，保留原请求w/v（轮角/路径曲率）。
        original_yaw = w * (v / requested_v)
        if self.odom is None or not 0 <= now - self.odom[0] <= self.odom_timeout:
            return v, original_yaw, 'terminal_cap_no_fresh_odom'
        if not self.cleared or self.travel < self.departure_m:
            return v, original_yaw, 'departing_third_line'
        # 实际尚未降下来或正在明显过弯，继续原P转向，避免切弯。
        speed = self.odom[1]
        if not tracking or not 0 < speed <= .18 or abs(w) > self.max_original_yaw:
            return v, original_yaw, 'preserve_curve_or_wait_slowdown'
        if self.observation is None:
            return v, original_yaw, 'no_supported_heading'
        stamp, angle, distance, error = self.observation
        age = now - stamp
        if not 0 <= age <= self.image_timeout:
            return v, original_yaw, 'stale_heading'
        # 保守可用距离：近车角距离减误差和图像后前进量。
        remaining = distance - error - max(speed, v) * age
        limit = min(.30, self.max_curvature * v)
        # 收敛尾段的P输出小于饱和上限，不能按最大曲率算出过短的所需距离。
        min_turn_rate = min(limit, 1.2 * self.tolerance)
        turn_distance = max(speed, v) * max(0., abs(angle) - self.tolerance) / min_turn_rate
        required = self.stop_budget_m + self.final_gap_m + self.extra_margin_m + turn_distance
        if remaining < required:
            return v, original_yaw, 'insufficient_alignment_room'
        if abs(angle) <= self.tolerance:
            return v, 0., 'heading_within_tolerance'
        return v, max(-limit, min(limit, 1.2 * angle)), 'aligning'
