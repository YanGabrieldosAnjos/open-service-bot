#!/usr/bin/env python

"""
  voice_nav.py - Version 1.1 2013-12-20
  
  Allows controlling a mobile base using simple speech commands.
  
  Based on the voice_cmd_vel.py script by Michael Ferguson in
  the pocketsphinx ROS package.
  
  See http://www.ros.org/wiki/pocketsphinx
"""
import time
import os
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import thread
from datetime import datetime
import threading
from threading import Thread
from math import copysign
from sound_play.libsoundplay import SoundClient
import sys
import subprocess
from subprocess import Popen
from sound_play.msg import SoundRequest

class VoiceNav:
    def __init__(self):
        rospy.init_node('voice_nav')
        
        rospy.on_shutdown(self.cleanup)
        
        # Set a number of parameters affecting the robot's speed
		# cont_erro = 0        

#        self.timer

        # We don't have to run the script very fast
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)
        
        self.timer = 0
        # A flag to determine whether or not voice control is paused
#        self.paused = False
        
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
#        rospy.Subscriber('/recognizer/output', String, self.speech_callback)
#        rospy.Subscriber('/wheel_encoder', Vector3, self.odom_callback)

        # A mapping from keywords or phrases to commands
#        self.keywords_to_command = {'stop': ['stop','kill'],
#                                    'start_nav': ['start navigation']}
       
        rospy.loginfo("Ready to receive rotate")
        #180 = 22 direita
#90 = 
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
	#turn aroun 180 degres        
	while self.timer < 21:
            self.cmd_vel.linear.x= 0
            self.cmd_vel.angular.z= 2
            self.cmd_vel_pub.publish(self.cmd_vel)
            r.sleep()                      
            self.timer = self.timer + 1
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    def cleanup(self):
       # When shutting down be sure to stop the robot!
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

if __name__=="__main__":
    try:
        VoiceNav()
        rospy.spin()		
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice navigation terminated.")

