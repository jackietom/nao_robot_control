#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('nao_control')
import math 
import tf
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('/nao_dcm/LHipRoll_position_controller/command', Float64, queue_size=10)
    rospy.init_node('jointPublisher')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(2)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
