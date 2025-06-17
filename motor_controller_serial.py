#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster

class MotorControllerSerial(Node):
    def __init__(self):
        super().__init__('motor_controller_serial')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.3)
        self.declare_parameter('max_speed', 255)
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().integer_value
        
        
        
        
        # Initialize serial connection
        self.serial_available = False
        try:
            self.get_logger().info(f'Attempting to connect to {serial_port} at {baud_rate} baud')
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            
            # Test the connection
            self.get_logger().info('Testing serial connection...')
            self.serial_conn.write(b'L0,R0\n')
            time.sleep(0.5)
            response = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
            self.get_logger().info(f'Arduino response: {response}')
            
            self.get_logger().info(f'Connected to Arduino on {serial_port}')
            self.serial_available = True
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.serial_available = False
            return
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Safety timer to stop motors if no commands
        self.last_cmd_time = time.time()
        self.safety_timer = self.create_timer(0.1, self.safety_check)
       
        
        self.get_logger().info('Motor Controller Serial Node Started - SUCCESS!')
    
    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to motor commands and send via serial"""
        try:
            linear_vel = msg.linear.x
            angular_vel = msg.angular.z
            
            # Store current velocities for odometry
            self.current_vx = linear_vel
            self.current_vth = angular_vel
            
            self.get_logger().info(f'Received cmd_vel: linear={linear_vel:.2f}, angular={angular_vel:.2f}')
            
            # Thresholds for movement
            linear_threshold = 0.05
            angular_threshold = 0.05
            
            # Determine motor commands based on velocities
            if abs(angular_vel) > angular_threshold:
                # Turning - prioritize rotation
                linear_vel = 0.0
                angular_vel = 1.0 * (angular_vel / abs(angular_vel))  # Normalize to ±1
                left_pwm = -self.max_speed if angular_vel > 0 else self.max_speed
                right_pwm = self.max_speed if angular_vel > 0 else -self.max_speed
            elif abs(linear_vel) > linear_threshold:
                # Moving forward/backward
                angular_vel = 0.0
                left_speed = linear_vel
                right_speed = linear_vel
                left_pwm = self.velocity_to_pwm(left_speed)
                right_pwm = self.velocity_to_pwm(right_speed)
            else: 
                # Stop
                linear_vel = 0.0
                angular_vel = 0.0
                left_pwm = 0
                right_pwm = 0

            # Send command to Arduino
            if self.serial_available:
                command = f"L{left_pwm},R{right_pwm}\n"
                self.get_logger().info(f'Sending command: {command.strip()}')
                
                try:
                    self.serial_conn.write(command.encode())
                    self.last_cmd_time = time.time()
                    time.sleep(0.05)  # Shorter delay
                    
                    # Read response
                    if self.serial_conn.in_waiting > 0:
                        response = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        self.get_logger().debug(f'Arduino response: {response}')
                except Exception as e:
                    self.get_logger().error(f'Serial write error: {e}')
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {e}')
    
    
    
    def velocity_to_pwm(self, velocity):
        """Convert velocity (m/s) to PWM value (-255 to 255)"""
        max_velocity = 1.0  # Maximum velocity in m/s
        min_pwm = 190  # Minimum PWM to overcome motor friction
        
        # Clamp velocity
        velocity = max(-max_velocity, min(max_velocity, velocity))
        
        # Convert to PWM
        pwm = int((velocity / max_velocity) * self.max_speed)
        
        # Apply minimum PWM threshold
        if pwm > 0:
            pwm = max(pwm, min_pwm)
        elif pwm < 0:
            pwm = min(pwm, -min_pwm)
        
        return pwm
    
    def safety_check(self):
        """Stop motors if no commands received recently"""
        if self.serial_available and time.time() - self.last_cmd_time > 1.0:  # 1 second timeout
            try:
                self.serial_conn.write(b"L0,R0\n")
                # Reset velocities
                self.current_vx = 0.0
                self.current_vth = 0.0
            except:
                pass
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'serial_conn') and self.serial_available:
            try:
                self.serial_conn.write(b"L0,R0\n")  # Stop motors
                self.serial_conn.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorControllerSerial()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    

