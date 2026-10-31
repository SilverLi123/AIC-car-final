#!/usr/bin/env python3
"""默认入口不运行本节点；显式实验launch才把它接在原mux之后。"""
import json
import math
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import Log

from precision_parking.detector import YellowDetector
from precision_parking.frame_worker import LatestFrameWorker
from precision_parking.finish_heading import FinishHeading, heading_error
from precision_parking.finish_heading_config import load_finish_heading_config


class FinishHeadingNode(Node):
    def __init__(self):
        super().__init__('finish_heading_node')
        defaults = {'calibration_file': '', 'image_topic': '/image_raw',
                    'request_topic': '/finish_heading/request', 'output_topic': '/cmd_vel'}
        for key, value in defaults.items(): self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        if self.resolve_topic_name(p('request_topic')) == self.resolve_topic_name(p('output_topic')):
            raise ValueError('Heading request/output topics must differ')
        calibration, ground, detector_options = load_finish_heading_config(p('calibration_file'))
        self.detector = YellowDetector(calibration, ground=ground, **detector_options)
        self.bridge = CvBridge()
        self.gate = FinishHeading()
        self.request = None
        self.last_zero = None
        self.started_at = None
        self.last_log_ns = 0
        self.last_diag = 0.
        self.latest_geometry = {}
        self.last_reason = ''
        self.road = None
        self.output = self.create_publisher(Twist, p('output_topic'), 1)
        self.diagnostics = self.create_publisher(String, '/finish_heading/diagnostics', 10)
        # 复用现有最新帧worker：OpenCV不占用控制回调，图像队列最多一张。
        self.worker = LatestFrameWorker(self.process_image, max_age=.15).start()
        self.create_subscription(Twist, p('request_topic'), self.on_request, 1)
        self.create_subscription(Twist, '/cmd_vel_stop', self.on_stop, 1)
        self.create_subscription(Bool, '/line_follower/start_cmd', self.on_start, 10)
        self.create_subscription(String, '/line_follower/track_state', self.on_road, 1)
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Image, p('image_topic'), self.on_image, qos_profile_sensor_data)
        self.create_timer(.02, self.tick)
        self.get_logger().warn('终点航向实验：第三根后限速0.12；原零速/第四根最高优先。未实车验收。')

    def source_time(self, stamp, max_age=.15):
        ns = int(stamp.sec)*1000000000 + int(stamp.nanosec)
        age = (self.get_clock().now().nanoseconds - ns)/1e9
        if ns <= 0 or not 0 <= age <= max_age:
            raise ValueError('stale or future source timestamp')
        return time.monotonic() - age, ns

    def on_start(self, message):
        if message.data and not self.gate.started:
            self.started_at = time.monotonic()
        self.gate.set_started(message.data)
        if not message.data:
            self.request = None
            self.road = None
            self.output.publish(Twist())

    def on_request(self, message):
        self.request = (time.monotonic(), message.linear.x, message.angular.z)
        # 零指令不等待下个20ms控制周期。
        if message.linear.x == 0: self.output.publish(Twist())

    def on_road(self, message):
        self.road = (time.monotonic(), message.data)

    def on_stop(self, message):
        self.last_zero = time.monotonic()
        self.output.publish(Twist())

    def on_log(self, message):
        if message.name.split('.')[-1] != 'line_follower_node' or not self.gate.started:
            return
        try: stamp, ns = self.source_time(message.stamp, .3)
        except ValueError: return
        if ns <= self.last_log_ns or self.started_at is None or stamp < self.started_at:
            return
        self.last_log_ns = ns
        match = re.fullmatch(r'New yellow line detected! Count: (\d+)/(\d+)', message.msg)
        if match:
            self.gate.on_count(*map(int, match.groups()))
            if self.gate.finished or self.gate.fault: self.output.publish(Twist())
        elif message.msg == 'Yellow line cleared. Ready for next yellow line.':
            self.gate.on_cleared()
        elif self.gate.count and (message.msg.startswith('Previous lap finished. New lap reset:') or
                                 message.msg.startswith('Line follower subscribing to ')):
            self.gate.fault = True
            self.output.publish(Twist())

    def on_odom(self, message):
        try: stamp, _ = self.source_time(message.header.stamp)
        except ValueError:
            self.gate.odom = None
            return
        self.gate.feedback(stamp, message.twist.twist.linear.x)

    def on_image(self, message):
        try:
            stamp, sequence = self.source_time(message.header.stamp)
            self.worker.submit(message, stamp, sequence)
        except ValueError:
            self.gate.observation = None

    def process_image(self, message, stamp, sequence):
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        return self.detector.process(frame, stamp, sequence)

    def tick(self):
        now = time.monotonic()
        result = self.worker.take()
        worker_state = self.worker.snapshot()
        if worker_state['error']:
            self.gate.observation = None
        elif result is not None and not result.error and 0 <= now-result.stamp <= .15:
            o = result.value
            self.gate.observe(o.stamp, o.sequence, o.line, o.distance_m,
                              o.error_m, o.distance_validated, o.confirmed)
            self.latest_geometry = {'status': o.status, 'distance_m': o.distance_m,
                                    'source_monotonic_s': o.stamp,
                                    'heading_error_deg_model': None}
            if o.line is not None:
                self.latest_geometry['heading_error_deg_model'] = math.degrees(heading_error(o.line))
        if self.request is None or not 0 <= now-self.request[0] <= .15:
            v, w, reason = 0., 0., 'request_timeout'
        elif self.last_zero is not None and now-self.last_zero <= .5:
            v, w, reason = 0., 0., 'original_stop_priority'
        else:
            # 原C++只在切换到tracking时发一次，不能把它当150ms心跳。
            # 状态保持到lost/yellow/暂停；运动请求、图像和odom仍分别验新鲜度。
            tracking = self.road is not None and self.road[1] == 'tracking'
            v, w, reason = self.gate.step(now, self.request[1], self.request[2], tracking=tracking)
        output = Twist(); output.linear.x = float(v); output.angular.z = float(w)
        self.output.publish(output)
        if reason != self.last_reason or now-self.last_diag >= .2:
            payload = dict(self.latest_geometry, count=self.gate.count, reason=reason,
                           output_v=v, output_w=w, departure_m=self.gate.travel,
                           road_state=self.road[1] if self.road else None,
                           actual_speed_mps=self.gate.odom[1] if self.gate.odom else None,
                           worker=worker_state, field_validated=False)
            if 'source_monotonic_s' in payload:
                payload['geometry_age_s'] = now-payload['source_monotonic_s']
            message = String(); message.data = json.dumps(payload, allow_nan=False)
            self.diagnostics.publish(message)
            self.last_reason, self.last_diag = reason, now

    def destroy_node(self):
        self.output.publish(Twist())
        self.worker.close(.2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FinishHeadingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None: node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__': main()
