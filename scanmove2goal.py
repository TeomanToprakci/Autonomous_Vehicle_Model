#!/usr/bin/env python3

#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/scanmove2goal.py

import rospy
import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry  # Odometry mesajını da import ediyoruz


class Turtlebot:

    def __init__(self):
        rospy.init_node('turtletogoal', anonymous=True)
        self.vel_publisher = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
        # To take the current position of the robot, subscribe to odom
        self.vel_subscriber = rospy.Subscriber('/tb3_0/odom', Odometry, self.update_pose)  # Odometry tipi kullanıyoruz
        self.laser_subscriber = rospy.Subscriber('/tb3_0/scan', LaserScan, self.receiveSensorMsgs)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.goal_pose = Pose()
        self.obstacle = False
        
        self.goal_pose.x = -5.5
        self.goal_pose.y = -2.5

    def update_pose(self, data):
        # Odometry mesajını alarak robotun konumunu güncelliyoruz
        self.pose.x = data.pose.pose.position.x
        self.pose.y = data.pose.pose.position.y
        # Yönelim bilgisi (yaw) kullanılarak robotun yönü güncelleniyor
        orientation_q = data.pose.pose.orientation
        _, _, self.pose.theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def receiveSensorMsgs(self, msg):
        # Engel tespiti mantığı
        limit = 2.0  # Engel tespiti için mesafe limiti
        obstacle = False
        size = len(msg.ranges)

        # Ön bölgeyi kontrol et
        for i in range(size // 8):  # Sağ ön 
            if msg.ranges[i] < limit:
                obstacle = True
                rospy.loginfo("Obstacle detected on the front right!")
        for i in range(7 * size // 8, size):  # Sol ön 
            if msg.ranges[i] < limit:
                obstacle = True
                rospy.loginfo("Obstacle detected on the front left!")

        self.obstacle = obstacle

    def euclidean_distance(self, goal_pose):
        # Hedefe olan mesafeyi hesaplıyoruz
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.5):
        # Hedefe olan mesafeye göre doğrusal hız hesaplıyoruz
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        # Yönelim açısını hesaplıyoruz
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        # Yönelim açısını kullanarak dönüş hızını hesaplıyoruz
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def rotate_robot(self, dir):
        # Robotu engel bulunduğunda sağa ya da sola döndürüyoruz
        vel_msg = Twist()
        vel_msg.linear.x = 0.06  # Hafif ileri hareket
        vel_msg.angular.z = 0.2 * dir  # Dönüş hızını ayarlıyoruz
        self.vel_publisher.publish(vel_msg)

    def move2goal(self):
        dist_tolerance = 0.1
        vel_msg = Twist()

        while self.euclidean_distance(self.goal_pose) >= dist_tolerance:
            if self.obstacle:
                # Engel tespit edilirse robotu döndürüyoruz
                rospy.loginfo("Obstacle detected! Avoiding obstacle.")
                if self.pose.x < self.goal_pose.x:
                    self.rotate_robot(-1)  # Sağ tarafa dön
                else:
                    self.rotate_robot(1)   # Sol tarafa dön
                self.obstacle = False  # Engel durumu sıfırlanıyor
            else:
                # Hedefe doğru ilerliyoruz
                vel_msg.linear.x = self.linear_vel(self.goal_pose)
                vel_msg.angular.z = self.angular_vel(self.goal_pose)
                self.vel_publisher.publish(vel_msg)
                rospy.loginfo(f"Moving towards goal: ({self.goal_pose.x}, {self.goal_pose.y})")
            
            self.rate.sleep()

        # Hedefe ulaştığında robotu durduruyoruz
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_publisher.publish(vel_msg)
        rospy.spin()

if __name__ == "__main__":
    try:
        x = Turtlebot()
        x.move2goal()

    except rospy.ROSInterruptException:
        pass
