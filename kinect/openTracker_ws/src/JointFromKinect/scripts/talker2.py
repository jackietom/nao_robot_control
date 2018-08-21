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
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        hello_str.header.stamp = rospy.Time.now()
        (transHead, rotHead) = listener.lookupTransform('/openni_tracker/head_1', '/openni_tracker/torso_1', rospy.Time(0))
        eulerHead = tf.transformations.euler_from_quaternion(rotHead)
        (transLHip, rotLHip) = listener.lookupTransform('/openni_tracker/left_hip_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerLHip = tf.transformations.euler_from_quaternion(rotLHip)
        (transLKnee, rotLKnee) = listener.lookupTransform('/openni_tracker/left_knee_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerLKnee = tf.transformations.euler_from_quaternion(rotLKnee)
        (transLAnkle, rotLAnkle) = listener.lookupTransform('/openni_tracker/left_foot_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerLAnkle = tf.transformations.euler_from_quaternion(rotLAnkle)
        (transRHip, rotRHip) = listener.lookupTransform('/openni_tracker/right_hip_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerRHip = tf.transformations.euler_from_quaternion(rotRHip)
        (transRKnee, rotRKnee) = listener.lookupTransform('/openni_tracker/right_knee_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerRKnee = tf.transformations.euler_from_quaternion(rotRKnee)
        (transRAnkle, rotRAnkle) = listener.lookupTransform('/openni_tracker/right_foot_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerRAnkle = tf.transformations.euler_from_quaternion(rotRAnkle)
        (transLShoulder, rotLShoulder) = listener.lookupTransform('/openni_tracker/left_shoulder_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerLShoulder = tf.transformations.euler_from_quaternion(rotLshoulder)
        (transLElbow, rotLElbow) = listener.lookupTransform('/openni_tracker/left_elbow_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerLElbow = tf.transformations.euler_from_quaternion(rotLElbow)
        (transLWrist, rotLWrist) = listener.lookupTransform('/openni_tracker/left_hand_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerLWrist = tf.transformations.euler_from_quaternion(rotLWrist)
        (transRShoulder, rotRShoulder) = listener.lookupTransform('/openni_tracker/right_shoulder_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerRShoulder = tf.transformations.euler_from_quaternion(rotRShoulder)
        (transRElbow, rotRElbow) = listener.lookupTransform('/openni_tracker/right_elbow_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerRElbow = tf.transformations.euler_from_quaternion(rotRElbow)
        (transRWrist, rotRWrist) = listener.lookupTransform('/openni_tracker/right_hand_1','/openni_tracker/torso_1',rospy.Time(0))
        eulerRWrist = tf.transformations.euler_from_quaternion(rotRWrist)
        hello_str.name =  ['HeadYaw', 'HeadPitch', 'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand', 'RFinger23', 'RFinger13', 'RFinger12', 'LFinger21', 'LFinger13', 'LFinger11', 'RFinger22', 'LFinger22', 'RFinger21', 'LFinger12', 'RFinger11', 'LFinger23', 'LThumb1', 'RThumb1', 'RThumb2', 'LThumb2']
        hello_str.position = [eulerHead[2], eulerHead[1], -0.00010594240000005861, 0.15590895200000005, -0.00038482599999989375, -0.00016400378000000493, -0.00016097489999999937, -1.490230000000814e-05, -0.00010594240000005861, -0.0001959274999999705, -0.00038482599999989375, -0.00016400378000000493, -4.8639999999711137e-05, -0.0001017730000000272, 0.0, -0.00014643740000003236, 0.0, -0.7897633000000001, 0.0, 0.0, 0.0, -1.7623500000008008e-05, 0.0, 0.7897633000000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        hello_str.velocity = []
        hello_str.effort = []
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
