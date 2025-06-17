#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import serial
import time

class ButtonNavigationNode(Node):
    def __init__(self):
        super().__init__('button_navigation_node')

        # Serial bağlantı (Arduino portunu kontrol et!)
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            time.sleep(2)  # Arduino'nun başlaması için bekle
            self.get_logger().info('Connected to Arduino for button input')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            return

        # Initial pose publisher
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10)

        # Action client
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose')
        
        self.get_logger().info('Waiting for navigation action server...')
        self.nav_to_pose_client.wait_for_server()

        # Hedef pozisyonlar
        self.poses = {
            'BTN2': self.create_pose(0.0, -2.0, 0.0),
            'BTN3': self.create_pose(2.0, -3.0, 0.0),
            'BTN4': self.create_pose(1.0, -0.5, 0.0),
        }

        # Button state tracking to avoid multiple triggers
        self.last_button_states = {'BTN1': 0, 'BTN2': 0, 'BTN3': 0, 'BTN4': 0}
        
        self.timer = self.create_timer(0.1, self.read_buttons)  # 10 Hz

        self.get_logger().info('Button Navigation Node Started')
        self.get_logger().info('BTN1: Set initial pose')
        self.get_logger().info('BTN2: Navigate to (0, -2)')
        self.get_logger().info('BTN3: Navigate to (2, -3)')
        self.get_logger().info('BTN4: Navigate to (1, -0.5)')

    def create_pose(self, x, y, theta):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # 90 degrees clockwise rotation
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = -0.7071
        pose.pose.orientation.w = 0.7071
        return pose

    def set_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = -0.7071
        initial_pose.pose.pose.orientation.w = 0.7071
        
        # Set covariance
        for i in range(36):
            initial_pose.pose.covariance[i] = 0.1 if i % 7 == 0 else 0.0
            
        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info("Initial pose set to (0, 0)!")

    def navigate_to_pose(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(f'Sending navigation goal to ({pose.pose.position.x:.1f}, {pose.pose.position.y:.1f})')
        
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return
        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info("Goal succeeded!")
        else:
            self.get_logger().error(f"Goal failed with status {status}")

    def read_buttons(self):
        try:
            # Read all available lines
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Only process button lines
                if not line.startswith('BTN'):
                    continue

                # Parse button states
                states = {}
                for part in line.split(';'):
                    if ':' in part:
                        key, val = part.split(':')
                        states[key.strip()] = int(val.strip())

                # Check for button presses (transition from 0 to 1)
                for btn in ['BTN1', 'BTN2', 'BTN3', 'BTN4']:
                    current_state = states.get(btn, 0)
                    last_state = self.last_button_states.get(btn, 0)
                    
                    # Detect button press (0 -> 1 transition)
                    if current_state == 1 and last_state == 0:
                        self.handle_button_press(btn)
                    
                    self.last_button_states[btn] = current_state

        except Exception as e:
            self.get_logger().debug(f"Button read error: {e}")

    def handle_button_press(self, button):
        self.get_logger().info(f"{button} pressed!")
        
        if button == 'BTN1':
            self.set_initial_pose()
        elif button in self.poses:
            self.navigate_to_pose(self.poses[button])

def main(args=None):
    rclpy.init(args=args)
    node = ButtonNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser'):
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

