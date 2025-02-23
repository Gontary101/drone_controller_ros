#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.wrench_pub = self.create_publisher(Wrench, '/drone/wrench', 10)
        self.get_logger().info('Wrench publisher created.')
        self.takeoff_duration = 5.0
        self.forward_duration = 5.0
        self.sequence_executed = False
        # Schedule the control sequence to start after 1 second
        self.timer = self.create_timer(1.0, self.execute_sequence)

    def execute_sequence(self):
        if self.sequence_executed:
            return
        self.sequence_executed = True
        self.get_logger().info('Starting control sequence...')

        # Takeoff: Apply upward force
        self.get_logger().info('Taking off...')
        upward_force = Vector3(x=0.0, y=0.0, z=15.0)
        zero_torque = Vector3(x=0.0, y=0.0, z=0.0)
        self.apply_wrench(upward_force, zero_torque, self.takeoff_duration)

        # Move forward: Apply forward force
        self.get_logger().info('Moving forward...')
        forward_force = Vector3(x=10.0, y=0.0, z=0.0)
        self.apply_wrench(forward_force, zero_torque, self.forward_duration)

        self.get_logger().info('Control sequence complete. Shutting down.')
        # Cancel the timer and shutdown
        self.timer.cancel()
        rclpy.shutdown()

    def apply_wrench(self, force: Vector3, torque: Vector3, duration_sec: float):
        wrench = Wrench()
        wrench.force = force
        wrench.torque = torque
        start_time = time.time()
        rate_interval = 1.0 / 100.0  # 100 Hz loop interval
        while time.time() - start_time < duration_sec:
            self.wrench_pub.publish(wrench)
            time.sleep(rate_interval)
        self.get_logger().info(f'Applied wrench for {duration_sec} seconds.')

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
