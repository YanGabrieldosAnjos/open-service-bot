#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    regions = {
        min(min(msg.ranges[0:143]),12),
        min(min(msg.ranges[144:287]),12),
        min(min(msg.ranges[288:431]),12),
        min(min(msg.ranges[432:573]),12),
        min(min(msg.ranges[573:713]),12),
    }
    rospy.loginfo(regions)

def main():
    rospy.init_node("reading_laser")
    sub = rospy.Subscriber("scan",LaserScan, clbk_laser)
    rospy.spin()

if __name__ == '__main__':
    main()