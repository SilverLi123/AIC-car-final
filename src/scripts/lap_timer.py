#!/usr/bin/env python3
"""Read-only lap timing for the existing RDK X5 controller.

ROS imports are deliberately deferred so the timing rules can be tested
without ROS. This program never publishes velocity or start commands.
"""
import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import math
from pathlib import Path
import re
import sys
import time
from typing import List, Optional


@dataclass(frozen=True)
class TimerConfig:
    linear_threshold: float = 0.01
    angular_threshold: float = 0.03
    confirm_seconds: float = 0.3
    min_samples: int = 4
    max_age: float = 0.3
    finish_count: int = 4
    follower_logger: str = 'line_follower_node'
    toggle_logger: str = 'start_toggle_node'

    def __post_init__(self):
        for name in ('linear_threshold', 'angular_threshold', 'confirm_seconds', 'max_age'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        if self.min_samples < 2 or self.finish_count != 4:
            raise ValueError('min_samples must be >=2 and finish_count must be 4')


@dataclass(frozen=True)
class LapResult:
    label: str
    run_index: int
    status: str
    started_at: float
    finished_at: float
    confirmed_at: float
    elapsed_s: float
    yellow_count: int
    start_source: str
    reason: str


def source_time(stamp_ns: int, wall_ns: int, now: float, max_age: float) -> Optional[float]:
    """Map a fresh device wall/ROS timestamp to local monotonic seconds."""
    age = (wall_ns - stamp_ns) / 1e9
    if stamp_ns <= 0 or not math.isfinite(age) or age < -.05 or age > max_age:
        return None
    return now - max(0., age)


class LapTimer:
    COUNT = re.compile(r'^New yellow line detected! Count: (\d+)/(\d+)$')
    FINAL = 'Final yellow line detected. Permanent stop activated.'

    def __init__(self, config: TimerConfig, label: str = ''):
        self.config, self.label = config, label
        self.state = 'IDLE'
        self.reason = '等待先收到false，再收到true'
        self.last_enabled = None
        self.pending_key = None
        self.run_index = 0
        self.started_at = None
        self.start_received_at = None
        self.clock_offset = None
        self.start_source = 'start_cmd_receive'
        self.yellow_count = 0
        self.last_count_at = None
        self.last_count_source_stamp = None
        self.finish_requested_at = None
        self.command = None
        self.zero_since = None
        self.last_odom_time = None
        self.last_odom_source_stamp = None
        self.window_start = None
        self.window_last = None
        self.window_samples = 0
        self.results: List[LapResult] = []

    @property
    def active(self):
        return self.state in ('RUNNING', 'WAIT_STOP')

    def _reset_window(self):
        self.window_start = self.window_last = None
        self.window_samples = 0

    def _close(self, status, finish, confirmed, reason):
        if not self.active:
            return
        self.results.append(LapResult(self.label, self.run_index, status,
                                     self.started_at, finish, confirmed,
                                     max(0., finish-self.started_at), self.yellow_count,
                                     self.start_source, reason))
        self.state, self.reason = status, reason
        self._reset_window()

    def cancel(self, now, reason='manual_pause', invalid=False):
        self._close('INVALID' if invalid else 'ABORTED', now, now, reason)

    def on_clock(self, now, wall_offset):
        if self.clock_offset is not None and abs(wall_offset-self.clock_offset) > .1:
            self.cancel(now, 'system_clock_changed', invalid=True)
            self.pending_key = None
        self.clock_offset = wall_offset

    def on_start(self, enabled, now):
        previous = self.last_enabled
        self.last_enabled = bool(enabled)
        if not enabled:
            self.cancel(now, 'manual_pause')
            return
        if previous is not False:
            return
        self.run_index += 1
        self.state, self.reason = 'RUNNING', '计时中'
        self.started_at, self.start_source = now, 'start_cmd_receive'
        self.start_received_at = now
        if self.pending_key is not None and 0 <= now-self.pending_key <= .5:
            self.started_at, self.start_source = self.pending_key, 'start_toggle_log'
        self.pending_key = None
        self.yellow_count, self.last_count_at = 0, None
        self.last_count_source_stamp = None
        self.last_odom_time = self.last_odom_source_stamp = None
        self.finish_requested_at = self.zero_since = None
        self._reset_window()

    @staticmethod
    def _logger_matches(actual, expected):
        return actual == expected or actual.endswith('.'+expected)

    def on_log(self, logger, text, event_time, source_stamp_ns=None):
        text = text.strip()
        if self._logger_matches(logger, self.config.toggle_logger):
            if text == 'Start signal toggled to true':
                self.pending_key = event_time
                if self.active and self.start_source == 'start_cmd_receive' and -.5 <= event_time-self.started_at <= .05:
                    self.started_at, self.start_source = event_time, 'start_toggle_log'
                    self.pending_key = None
            elif text == 'Start signal toggled to false':
                self.pending_key = None
                if self.active and event_time >= self.started_at:
                    self.cancel(event_time, 'manual_pause')
            return
        if not self._logger_matches(logger, self.config.follower_logger) or not self.active:
            return
        if event_time < self.started_at:
            return
        if text.startswith('Previous lap finished. New lap reset:'):
            if self.yellow_count == 0 and event_time <= self.start_received_at+.5:
                return
            self.cancel(event_time, 'controller_restarted_or_reset', invalid=True)
            return
        if text.startswith('Line follower subscribing to '):
            self.cancel(event_time, 'controller_restarted_or_reset', invalid=True)
            return
        match = self.COUNT.fullmatch(text)
        if match:
            count, total = map(int, match.groups())
            if source_stamp_ns is not None and self.last_count_source_stamp is not None:
                if source_stamp_ns < self.last_count_source_stamp:
                    return
                if count == self.yellow_count and source_stamp_ns == self.last_count_source_stamp:
                    return
            else:
                if self.last_count_at is not None and event_time < self.last_count_at:
                    return
                if count == self.yellow_count and event_time == self.last_count_at:
                    return
            if total != self.config.finish_count or count != self.yellow_count+1:
                self.cancel(event_time, 'yellow_count_not_sequential_or_not_four', invalid=True)
                return
            self.yellow_count, self.last_count_at = count, event_time
            self.last_count_source_stamp = source_stamp_ns
        elif text == self.FINAL:
            if self.yellow_count != self.config.finish_count:
                self.cancel(event_time, 'finish_log_without_all_four_counts', invalid=True)
                return
            if self.finish_requested_at is not None:
                return
            self.state, self.reason = 'WAIT_STOP', '第四条黄线已触发，等待实际停稳估计'
            self.finish_requested_at = event_time
            self._reset_window()
            self.zero_since = None
            if self.command and self.command[0] >= event_time and self.command[1] <= 1e-6 and self.command[2] <= 1e-6:
                self.zero_since = self.command[0]

    def on_command(self, linear, angular, now):
        if not all(math.isfinite(v) for v in (linear, angular, now)):
            self.command = self.zero_since = None
            self._reset_window()
            return
        if self.command is not None and now < self.command[0]:
            return
        self.command = (now, abs(linear), abs(angular))
        if abs(linear) > 1e-6 or abs(angular) > 1e-6:
            self.zero_since = None
            self._reset_window()
        elif self.finish_requested_at is not None and now >= self.finish_requested_at and self.zero_since is None:
            self.zero_since = now

    def on_odometry(self, linear, angular, sample_time, received_at, sample_stamp_ns=None):
        if not all(math.isfinite(v) for v in (linear, angular, sample_time, received_at)):
            self._reset_window()
            return
        if received_at-sample_time > self.config.max_age or sample_time > received_at+.05:
            self._reset_window()
            return
        if sample_stamp_ns is not None and self.last_odom_source_stamp is not None:
            if sample_stamp_ns <= self.last_odom_source_stamp:
                return
        elif self.last_odom_time is not None and sample_time <= self.last_odom_time:
            return
        self.last_odom_time = sample_time
        self.last_odom_source_stamp = sample_stamp_ns
        if self.state != 'WAIT_STOP':
            return
        command_ok = (self.command is not None and 0 <= received_at-self.command[0] <= self.config.max_age
                      and self.command[1] <= 1e-6 and self.command[2] <= 1e-6)
        if (not command_ok or self.zero_since is None
                or sample_time < max(self.finish_requested_at, self.zero_since)
                or abs(linear) > self.config.linear_threshold
                or abs(angular) > self.config.angular_threshold):
            self._reset_window()
            return
        if self.window_last is not None and sample_time-self.window_last > self.config.max_age:
            self._reset_window()
        if self.window_start is None:
            self.window_start = sample_time
        self.window_last = sample_time
        self.window_samples += 1
        if self.window_samples >= self.config.min_samples and sample_time-self.window_start >= self.config.confirm_seconds-1e-9:
            self._close('FINISHED', self.window_start, sample_time, 'fourth_line_odometry_stop_estimate')

    def tick(self, now):
        if self.state == 'WAIT_STOP' and (self.last_odom_time is None or now-self.last_odom_time > self.config.max_age
                                         or self.command is None or now-self.command[0] > self.config.max_age):
            self._reset_window()


def append_result(path, result, config, wall_offset):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    china = timezone(timedelta(hours=8))
    def iso(stamp):
        return datetime.fromtimestamp(stamp+wall_offset, china).isoformat(timespec='milliseconds')
    row = dict(label=result.label, run_index=result.run_index, status=result.status,
               elapsed_s=f'{result.elapsed_s:.6f}', yellow_count=result.yellow_count,
               start_source=result.start_source, started_at=iso(result.started_at),
               stopped_at=iso(result.finished_at), confirmed_at=iso(result.confirmed_at),
               confirmation_delay_s=f'{result.confirmed_at-result.finished_at:.6f}',
               linear_threshold=config.linear_threshold, angular_threshold=config.angular_threshold,
               confirm_seconds=config.confirm_seconds, reason=result.reason)
    with path.open('a+', newline='', encoding='utf-8') as stream:
        stream.seek(0, 2)
        writer = csv.DictWriter(stream, fieldnames=list(row))
        if stream.tell() == 0:
            writer.writeheader()
        writer.writerow(row)
        stream.flush()


def argument_parser():
    parser = argparse.ArgumentParser(description='独立实车计时器：空格启动→第四条黄线停稳估计，不发送运动指令。')
    parser.add_argument('--label', default='test', help='本轮配置名称，例如 original030 或 recommended035')
    parser.add_argument('--csv', type=Path, default=Path.home()/'rdkx5_test_data/lap_times.csv')
    parser.add_argument('--linear-threshold', type=float, default=.01, help='静止线速度阈值 m/s')
    parser.add_argument('--angular-threshold', type=float, default=.03, help='静止角速度阈值 rad/s')
    parser.add_argument('--confirm-seconds', type=float, default=.3)
    parser.add_argument('--max-age', type=float, default=.3)
    parser.add_argument('--start-topic', default='/line_follower/start_cmd')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--follower-logger', default='line_follower_node')
    parser.add_argument('--toggle-logger', default='start_toggle_node')
    return parser


def main(argv=None):
    parser = argument_parser()
    args, ros_args = parser.parse_known_args(argv)
    try:
        config = TimerConfig(linear_threshold=args.linear_threshold, angular_threshold=args.angular_threshold,
                             confirm_seconds=args.confirm_seconds, max_age=args.max_age,
                             follower_logger=args.follower_logger, toggle_logger=args.toggle_logger)
    except ValueError as error:
        parser.error(str(error))
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.executors import ExternalShutdownException
        from std_msgs.msg import Bool
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rcl_interfaces.msg import Log
    except ImportError as error:
        print('缺少ROS环境。请在开发板先 source /opt/tros/humble/setup.bash。\n'+str(error), file=sys.stderr)
        return 2

    class TimerNode(Node):
        def __init__(self):
            super().__init__('lap_timer')
            if self.get_parameter('use_sim_time').value:
                raise ValueError('本计时器用于实时实车，不支持use_sim_time或历史录包时钟')
            self.engine = LapTimer(config, args.label)
            self.saved = 0
            self.last_print = 0.
            self.wall_offset = time.time()-time.monotonic()
            self.engine.on_clock(time.monotonic(), self.wall_offset)
            reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
            sensor = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Bool, args.start_topic, self.start_callback, reliable)
            self.create_subscription(Twist, args.cmd_topic, self.command_callback, reliable)
            self.create_subscription(Odometry, args.odom_topic, self.odom_callback, sensor)
            self.create_subscription(Log, '/rosout', self.log_callback, reliable)
            self.create_timer(.1, self.timer_callback)
            self.get_logger().info('计时器已就绪。保持车辆暂停，确认收到false后，在start_toggle终端按空格。')
            self.get_logger().info(f'配置标签={args.label}；CSV={args.csv.expanduser()}；停稳为里程计估计，车头位置需现场核对。')

        def start_callback(self, msg):
            now, _ = self.clock_sample()
            old = self.engine.run_index
            self.engine.on_start(msg.data, now)
            if self.engine.run_index != old:
                self.get_logger().info(f'第{self.engine.run_index}次计时开始，起点来源={self.engine.start_source}')

        def command_callback(self, msg):
            now, _ = self.clock_sample()
            v = math.hypot(msg.linear.x, msg.linear.y, msg.linear.z)
            w = math.hypot(msg.angular.x, msg.angular.y, msg.angular.z)
            self.engine.on_command(v, w, now)

        def odom_callback(self, msg):
            now, wall_ns = self.clock_sample()
            stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            sample = source_time(stamp, wall_ns, now, config.max_age)
            if sample is None:
                self.engine._reset_window()
                return
            twist = msg.twist.twist
            v = math.hypot(twist.linear.x, twist.linear.y, twist.linear.z)
            w = math.hypot(twist.angular.x, twist.angular.y, twist.angular.z)
            self.engine.on_odometry(v, w, sample, received_at=now, sample_stamp_ns=stamp)

        def log_callback(self, msg):
            now, wall_ns = self.clock_sample()
            stamp = msg.stamp.sec*1000000000+msg.stamp.nanosec
            sample = source_time(stamp, wall_ns, now, config.max_age)
            if sample is not None:
                self.engine.on_log(msg.name, msg.msg, sample, source_stamp_ns=stamp)

        def save_results(self):
            while self.saved < len(self.engine.results):
                result = self.engine.results[self.saved]
                try:
                    append_result(args.csv, result, config, self.wall_offset)
                except OSError as error:
                    self.get_logger().error(f'CSV保存失败：{error}；终端成绩仍有效，请手工记录。')
                if result.status == 'FINISHED':
                    self.get_logger().info(f'[计时完成] {result.elapsed_s:.3f} 秒；4/4；停稳确认延迟{result.confirmed_at-result.finished_at:.3f}秒；是否合格圈需现场核对。')
                else:
                    self.get_logger().warn(f'[{result.status}] {result.elapsed_s:.3f} 秒；黄线{result.yellow_count}/4；原因={result.reason}，不作为完成成绩。')
                self.saved += 1

        def clock_sample(self):
            now, wall_ns = time.monotonic(), time.time_ns()
            self.wall_offset = wall_ns/1e9-now
            self.engine.on_clock(now, self.wall_offset)
            return now, wall_ns

        def timer_callback(self):
            now, _ = self.clock_sample()
            self.engine.tick(now)
            self.save_results()
            if now-self.last_print >= 1.:
                if self.engine.active:
                    elapsed = max(0.,now-self.engine.started_at)
                    self.get_logger().info(f'[计时] {elapsed:.1f}s | {self.engine.state} | 黄线{self.engine.yellow_count}/4 | {self.engine.reason}')
                elif self.engine.state == 'IDLE':
                    self.get_logger().info('等待false→true启动；请在start_toggle所在终端操作空格。')
                self.last_print = now

    rclpy.init(args=ros_args)
    node = None
    try:
        node = TimerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.engine.cancel(time.monotonic(), 'timer_stopped')
            node.save_results()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
