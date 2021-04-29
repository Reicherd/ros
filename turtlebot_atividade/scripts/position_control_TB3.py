#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import time

class TurtleControl:
    def __init__(self):
        rospy.init_node("tb3control_node", anonymous=True)
        self.vel_publisher = rospy.Publisher("tb3_1/cmd_vel", Twist, queue_size=10)
        self.vel_publisher_master = rospy.Publisher("tb3_0/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("tb3_1/odom", Odometry, self.update_pose)
        rospy.Subscriber("tb3_0/odom", Odometry, self.update_ref)
        self.pose = Pose()
        self.ref_pose = Pose()
        self.rate = rospy.Rate(10)
        self.max_vel = 0.22
        self.max_ang = 2.84

    def update_pose(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta =  yaw

    def update_ref(self, msg):
        self.ref_pose.x = msg.pose.pose.position.x
        self.ref_pose.y = msg.pose.pose.position.y

    def ref_distance(self):
        return np.sqrt(  (self.ref_pose.x - self.pose.x)**2 + (self.ref_pose.y - self.pose.y)**2)

    def linear_vel_control(self, kp = 1.5):
        distance = self.ref_distance()
        control = kp* distance
        if abs(control) > self.max_vel:
            control = self.max_vel*np.sign(control)
        return control

    def angular_vel_control(self, kp=6):
        angle_r = np.arctan2(self.ref_pose.y - self.pose.y,  self.ref_pose.x - self.pose.x )        
        control = kp*(angle_r - self.pose.theta)
        if abs(control) > self.max_ang:
            control = self.max_ang*np.sign(control)
        return control

    def move2ref(self):
        ref_tol = 0.5
        vel_msg = Twist()
        while not rospy.is_shutdown():
            while self.ref_distance() >= ref_tol:
                vel_msg.linear.x = self.linear_vel_control()
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel_control()

                self.vel_publisher.publish(vel_msg)
                self.rate.sleep()

            # stop
            vel_msg.linear.x = 0
            vel_msg.angular.z= 0
            self.vel_publisher.publish(vel_msg)
            
            #caso tenha batido
            if self.ref_distance() <= 0.1:
                self.vel_publisher_master.publish(vel_msg)
                rospy.loginfo("BATEU")

if __name__ == '__main__':
    bot = TurtleControl()
    time.sleep(5)
    bot.move2ref()
