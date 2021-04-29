#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class swarm_control:

    def __init__(self):
        rospy.init_node('swarm_control_node',  anonymous=False)
        self.pub1 = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
        #self.pub2 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.update)

        self.new_pos = Twist()
        self.cmd = Twist()
        self.rate = rospy.Rate(10) # Hz

        rospy.loginfo("/cmd_vel >> /tb3_0/cmd_vel")
        #rospy.loginfo("/cmd_vel >> /tb3_1/cmd_vel")


    def update(self, msg):
        self.cmd = msg

    def run(self):
        while not rospy.is_shutdown():
            self.new_pos = self.cmd
            self.pub1.publish(self.new_pos)
            #self.pub2.publish(self.new_pos)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        swarm_tb3 = swarm_control()
        swarm_tb3.run()
    except rospy.ROSInterruptException:
        pass
