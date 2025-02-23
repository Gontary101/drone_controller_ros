#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        # Create a publisher for the wrench topic
        self.wrench_pub = self.create_publisher(Wrench, '/drone/wrench', 10)
        self.get_logger().info('Wrench publisher created. Starting control sequence...')
        
        self.takeoff_duration = 5.0  # Duration for takeoff in seconds
        self.forward_duration = 5.0  # Duration for forward movement in seconds
        self.execute_sequence()

    def apply_wrench(self, force: Vector3, torque: Vector3, duration_sec: float):
        """Publish wrench messages to apply force and torque for a specified duration."""
        wrench = Wrench()
        wrench.force = force
        wrench.torque = torque
        start_time = time.time()
        rate = self.create_rate(100)  # Publish at 100 Hz
        while time.time() - start_time < duration_sec:
            self.wrench_pub.publish(wrench)
            rate.sleep()
        self.get_logger().info(f'Applied wrench for {duration_sec} seconds')

    def execute_sequence(self):
        """Execute a sequence of movements: takeoff, then move forward."""
        # Takeoff: Apply upward force
        self.get_logger().info('Taking off...')
        upward_force = Vector3(x=0.0, y=0.0, z=15.0)  # Force in Newtons
        zero_torque = Vector3(x=0.0, y=0.0, z=0.0)
        self.apply_wrench(upward_force, zero_torque, self.takeoff_duration)
        
        # Move forward: Apply forward force
        self.get_logger().info('Moving forward...')
        forward_force = Vector3(x=10.0, y=0.0, z=0.0)
        self.apply_wrench(forward_force, zero_torque, self.forward_duration)
        
        self.get_logger().info('Control sequence complete. Shutting down.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)  # Spin to keep the node alive until shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()