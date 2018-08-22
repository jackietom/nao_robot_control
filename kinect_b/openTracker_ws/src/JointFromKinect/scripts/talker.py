#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('JointFromKinect')
import math 
import tf
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('jointPublisher')
    rate = rospy.Rate(10) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    while not rospy.is_shutdown():
        hello_str.header.stamp = rospy.Time.now()
        hello_str.name =  ['HeadYaw', 'HeadPitch', 'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand', 'RFinger23', 'RFinger13', 'RFinger12', 'LFinger21', 'LFinger13', 'LFinger11', 'RFinger22', 'LFinger22', 'RFinger21', 'LFinger12', 'RFinger11', 'LFinger23', 'LThumb1', 'RThumb1', 'RThumb2', 'LThumb2']
        hello_str.position = [1, 1, -0.00010594240000005861, 0.15590895200000005, -0.00038482599999989375, -0.00016400378000000493, -0.00016097489999999937, -1.490230000000814e-05, -0.00010594240000005861, -0.0001959274999999705, -0.00038482599999989375, -0.00016400378000000493, -4.8639999999711137e-05, -0.0001017730000000272, 0.0, -0.00014643740000003236, 0.0, -0.7897633000000001, 0.0, 0.0, 0.0, -1.7623500000008008e-05, 0.0, 0.7897633000000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        hello_str.velocity = []
        hello_str.effort = []
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
