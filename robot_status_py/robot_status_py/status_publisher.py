"""Fake battery/temperature sensor.

Demonstrates:
  * a publisher on a timer      -> /robot_status       (robot_status_interfaces/msg/RobotStatus)
  * a service server            -> /reset_status       (robot_status_interfaces/srv/ResetStatus)
  * parameters, including one that can be changed at runtime (publish_rate_hz)
"""
import random

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from robot_status_interfaces.msg import RobotStatus
from robot_status_interfaces.srv import ResetStatus


class StatusPublisher(Node):

    def __init__(self):
        super().__init__('status_publisher')

        # Parameters: declare them with defaults; values can come from YAML, launch or CLI.
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('battery_drain_per_sec', 0.5)
        self.declare_parameter('low_battery_threshold', 20.0)

        self._battery = 100.0
        self._temperature = 30.0
        self._last_tick = self.get_clock().now()

        self._publisher = self.create_publisher(RobotStatus, 'robot_status', 10)
        self._reset_srv = self.create_service(ResetStatus, 'reset_status', self._handle_reset)

        self._timer = None
        self._start_timer(self.get_parameter('publish_rate_hz').value)

        # "on_set" validates a change before it is applied; "post_set" reacts once it is applied.
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(self._apply_parameters)

        self.get_logger().info('Status publisher started')

    def _start_timer(self, rate_hz):
        if self._timer is not None:
            self.destroy_timer(self._timer)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_status)
        self.get_logger().info(f'Publishing at {rate_hz:.2f} Hz')

    def _validate_parameters(self, params):
        for param in params:
            if param.name == 'publish_rate_hz' and param.value <= 0.0:
                return SetParametersResult(successful=False, reason='publish_rate_hz must be > 0')
            if param.name == 'battery_drain_per_sec' and param.value < 0.0:
                return SetParametersResult(successful=False, reason='battery_drain_per_sec must be >= 0')
        return SetParametersResult(successful=True)

    def _apply_parameters(self, params):
        for param in params:
            if param.name == 'publish_rate_hz':
                self._start_timer(param.value)

    def _publish_status(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds / 1e9
        self._last_tick = now

        # Simulate: battery drains linearly, temperature creeps up as the battery empties, plus noise.
        drain = self.get_parameter('battery_drain_per_sec').value
        self._battery = max(0.0, self._battery - drain * dt)
        target_temp = 30.0 + 0.2 * (100.0 - self._battery)
        self._temperature += 0.1 * (target_temp - self._temperature) + random.gauss(0.0, 0.2)

        msg = RobotStatus()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.battery_percentage = self._battery
        msg.battery_voltage = 10.5 + 2.1 * (self._battery / 100.0)
        msg.temperature_celsius = self._temperature
        msg.level, msg.message = self._classify()

        self._publisher.publish(msg)

        if msg.level != RobotStatus.LEVEL_OK:
            # Throttle so a persistent warning does not flood the console.
            self.get_logger().warn(msg.message, throttle_duration_sec=5.0)

    def _classify(self):
        low = self.get_parameter('low_battery_threshold').value
        if self._battery <= 5.0:
            return RobotStatus.LEVEL_ERROR, f'Battery critical: {self._battery:.1f}%'
        if self._temperature > 50.0:
            return RobotStatus.LEVEL_ERROR, f'Overheating: {self._temperature:.1f} C'
        if self._battery < low:
            return RobotStatus.LEVEL_WARN, f'Battery low: {self._battery:.1f}%'
        return RobotStatus.LEVEL_OK, 'All systems nominal'

    def _handle_reset(self, request, response):
        if not 0.0 < request.battery_percentage <= 100.0:
            response.success = False
            response.message = 'battery_percentage must be in (0, 100]'
            return response

        self._battery = float(request.battery_percentage)
        self._temperature = 30.0
        response.success = True
        response.message = f'Battery reset to {self._battery:.1f}%'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
