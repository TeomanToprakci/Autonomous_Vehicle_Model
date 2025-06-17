#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from nav_msgs.msg import Odometry
import math
import time

class TFPublisherFixed(Node):
    def __init__(self):
        super().__init__('tf_publisher_fixed')
        
        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Publish static transforms once
        self.publish_static_transforms()
        
        # Timer for dynamic transforms
        self.timer = self.create_timer(0.05, self.publish_dynamic_transforms)
        
        self.get_logger().info('TF Publisher Fixed Node Started')
    
    def publish_static_transforms(self):
        """Publish static transforms that don't change"""
        now = self.get_clock().now().to_msg()
        
        static_transforms = []
        
        # base_footprint -> base_link (static)
        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.stamp = now
        base_footprint_to_base_link.header.frame_id = 'base_footprint'
        base_footprint_to_base_link.child_frame_id = 'base_link'
        base_footprint_to_base_link.transform.translation.x = 0.0
        base_footprint_to_base_link.transform.translation.y = 0.0
        base_footprint_to_base_link.transform.translation.z = 0.05
        base_footprint_to_base_link.transform.rotation.x = 0.0
        base_footprint_to_base_link.transform.rotation.y = 0.0
        base_footprint_to_base_link.transform.rotation.z = 0.0
        base_footprint_to_base_link.transform.rotation.w = 1.0
        static_transforms.append(base_footprint_to_base_link)
        
        # base_link -> laser_frame (static)
        base_link_to_laser = TransformStamped()
        base_link_to_laser.header.stamp = now
        base_link_to_laser.header.frame_id = 'base_link'
        base_link_to_laser.child_frame_id = 'laser_frame'
        base_link_to_laser.transform.translation.x = 0.0
        base_link_to_laser.transform.translation.y = 0.0
        base_link_to_laser.transform.translation.z = 0.02
        base_link_to_laser.transform.rotation.x = 0.0
        base_link_to_laser.transform.rotation.y = 0.0
        base_link_to_laser.transform.rotation.z = 0.0
        base_link_to_laser.transform.rotation.w = 1.0
        static_transforms.append(base_link_to_laser)
        
        # Publish all static transforms
        self.static_tf_broadcaster.sendTransform(static_transforms)
        self.get_logger().info('Published static transforms')
    
    def cmd_vel_callback(self, msg):
        """Update current velocities from cmd_vel"""
        self.current_linear_vel = msg.linear.x
        self.current_angular_vel = msg.angular.z
    
    def publish_dynamic_transforms(self):
        """Publish dynamic transforms that change over time"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Update robot pose based on cmd_vel
        delta_x = self.current_linear_vel * math.cos(self.theta) * dt
        delta_y = self.current_linear_vel * math.sin(self.theta) * dt
        delta_theta = self.current_angular_vel * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        now = self.get_clock().now().to_msg()
        
        # odom -> base_footprint (dynamic)
        odom_to_base_footprint = TransformStamped()
        odom_to_base_footprint.header.stamp = now
        odom_to_base_footprint.header.frame_id = 'odom'
        odom_to_base_footprint.child_frame_id = 'base_footprint'
        odom_to_base_footprint.transform.translation.x = self.x
        odom_to_base_footprint.transform.translation.y = self.y
        odom_to_base_footprint.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        odom_to_base_footprint.transform.rotation.x = 0.0
        odom_to_base_footprint.transform.rotation.y = 0.0
        odom_to_base_footprint.transform.rotation.z = math.sin(self.theta / 2.0)
        odom_to_base_footprint.transform.rotation.w = math.cos(self.theta / 2.0)
        
        # Publish dynamic transform
        self.tf_broadcaster.sendTransform(odom_to_base_footprint)
        
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = odom_to_base_footprint.transform.rotation
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.current_linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.current_angular_vel
        
        # Covariance
        odom_msg.pose.covariance[0] = 0.1   # x
        odom_msg.pose.covariance[7] = 0.1   # y
        odom_msg.pose.covariance[35] = 0.1  # theta
        odom_msg.twist.covariance[0] = 0.1  # vx
        odom_msg.twist.covariance[35] = 0.1 # vtheta
        
        self.odom_pub.publish(odom_msg)
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TFPublisherFixed()
    
    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tf_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
