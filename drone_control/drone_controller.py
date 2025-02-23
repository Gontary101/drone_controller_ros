#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Vector3
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        # Create a service client for applying a wrench to a body in Gazebo
        self.client = self.create_client(ApplyBodyWrench, '/apply_body_wrench')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /apply_body_wrench service...')
        self.get_logger().info('Service available. Starting control sequence...')
        
        # Define durations (in seconds) for each phase
        self.takeoff_duration = 5.0  
        self.forward_duration = 5.0  
        
        # Run the sequence (this is a simple synchronous example)
        self.execute_sequence()

    def apply_wrench(self, body_name, force: Vector3, torque: Vector3, duration_sec: float):
        req = ApplyBodyWrench.Request()
        req.body_name = body_name  # e.g., "advanced_quadrotor::base_link"
        req.reference_frame = "world"  # apply forces in world frame
        req.wrench.force = force
        req.wrench.torque = torque
        # Start immediately
        req.start_time.sec = 0  
        req.start_time.nanosec = 0
        # Set duration
        req.duration.sec = int(duration_sec)
        req.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Applied wrench on {body_name}')
        else:
            self.get_logger().error('Failed to apply wrench')

    def execute_sequence(self):
        # Phase 1: Take off (apply upward force)
        self.get_logger().info('Taking off...')
        upward_force = Vector3(x=0.0, y=0.0, z=15.0)  # adjust force as needed
        zero_torque = Vector3(x=0.0, y=0.0, z=0.0)
        self.apply_wrench("advanced_quadrotor::base_link", upward_force, zero_torque, self.takeoff_duration)
        time.sleep(self.takeoff_duration)
        
        # Phase 2: Move forward (apply force in the x direction)
        self.get_logger().info('Moving forward...')
        forward_force = Vector3(x=10.0, y=0.0, z=0.0)  # adjust force as needed
        self.apply_wrench("advanced_quadrotor::base_link", forward_force, zero_torque, self.forward_duration)
        time.sleep(self.forward_duration)
        
        self.get_logger().info('Control sequence complete. Shutting down.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
