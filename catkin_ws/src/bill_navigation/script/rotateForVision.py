#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import pi

class RotateForVision():
    def __init__(self):
        # Give the node a name
        rospy.init_node('rotateforvision', anonymous=False)
        # Set the rotation speed to 1.0 radians per second
        self.angular_speed = 2.0
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('bnRotation', String, self.rotationCB)

    def rotationCB(self, msg):
        if(msg.data == 'turn'):
            self.rotation()

    def rotation(self):
        # We don't have to run the script very fast
        self.rate = 5
        r = rospy.Rate(self.rate)
        self.timer = 0
        # A flag to determine whether or not voice control is paused
        # self.paused = False
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()
        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        rospy.loginfo("Ready to receive rotate")
        #180 = 21 direita
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
	#turn aroun 180 degres
	while self.timer < 45:
            self.cmd_vel.linear.x= 0.2
            self.cmd_vel.angular.z= 2
            self.cmd_vel_pub.publish(self.cmd_vel)
            r.sleep()
            self.timer = self.timer + 1
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        RotateForVision()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Rotate-for-vision node terminated.")
