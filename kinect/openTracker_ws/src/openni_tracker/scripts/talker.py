#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('openni_tracker')
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
    listener = tf.TransformListener()
    listener.waitForTransform("/head_1","/torso_1",rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform("/left_hip_1","/torso_1",rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform("/left_knee_1","/left_hip_1",rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform("/left_foot_1","/left_knee_1",rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform("/right_hip_1","/torso_1",rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform("/right_knee_1","/right_hip_1",rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform("/right_foot_1","/right_knee_1",rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform("/left_shoulder_1","/torso_1",rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform("/left_elbow_1","/left_shoulder_1",rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform("/left_hand_1","/left_elbow_1",rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform("/right_hand_1","/right_elbow_1",rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        hello_str.header.stamp = rospy.Time.now()
        now = rospy.Time.now()
        listener.waitForTransform("/head_1","/torso_1",now,rospy.Duration(4.0))
        (transHead, rotHead) = listener.lookupTransform("/head_1", "/torso_1", now)
        eulerHead = tf.transformations.euler_from_quaternion(rotHead)
        listener.waitForTransform("/left_hip_1","/torso_1",now,rospy.Duration(4.0))
        (transLHip, rotLHip) = listener.lookupTransform('/left_hip_1','/torso_1',now)
        eulerLHip = tf.transformations.euler_from_quaternion(rotLHip)
        listener.waitForTransform("/left_knee_1","/left_hip_1",now,rospy.Duration(4.0))
        (transLKnee, rotLKnee) = listener.lookupTransform('/left_knee_1','/left_hip_1',now)
        eulerLKnee = tf.transformations.euler_from_quaternion(rotLKnee)
        listener.waitForTransform("/left_foot_1","/left_knee_1",now,rospy.Duration(4.0))
        (transLAnkle, rotLAnkle) = listener.lookupTransform('/left_foot_1','/left_knee_1',now)
        eulerLAnkle = tf.transformations.euler_from_quaternion(rotLAnkle)
        listener.waitForTransform("/right_hip_1","/torso_1",now,rospy.Duration(4.0))
        (transRHip, rotRHip) = listener.lookupTransform('/right_hip_1','/torso_1',now)
        eulerRHip = tf.transformations.euler_from_quaternion(rotRHip)
        listener.waitForTransform("/right_knee_1","/right_hip_1",now,rospy.Duration(4.0))
        (transRKnee, rotRKnee) = listener.lookupTransform('/right_knee_1','/right_hip_1',now)
        eulerRKnee = tf.transformations.euler_from_quaternion(rotRKnee)
        listener.waitForTransform("/right_foot_1","/right_knee_1",now,rospy.Duration(4.0))
        (transRAnkle, rotRAnkle) = listener.lookupTransform('/right_foot_1','/right_knee_1',now)
        eulerRAnkle = tf.transformations.euler_from_quaternion(rotRAnkle)
        listener.waitForTransform("/left_shoulder_1","/torso_1",now,rospy.Duration(4.0))
        (transLShoulder, rotLShoulder) = listener.lookupTransform('/left_shoulder_1','/torso_1',now)
        eulerLShoulder = tf.transformations.euler_from_quaternion(rotLShoulder)
        listener.waitForTransform("/left_elbow_1","/left_shoulder_1",now,rospy.Duration(4.0))
        (transLElbow, rotLElbow) = listener.lookupTransform('/left_elbow_1','/left_shoulder_1',now)
        eulerLElbow = tf.transformations.euler_from_quaternion(rotLElbow)
        listener.waitForTransform("/left_hand_1","/left_elbow_1",now,rospy.Duration(4.0))
        (transLWrist, rotLWrist) = listener.lookupTransform('/left_hand_1','/left_elbow_1',now)
        eulerLWrist = tf.transformations.euler_from_quaternion(rotLWrist)
        listener.waitForTransform("/right_shoulder_1","/torso_1",now,rospy.Duration(4.0))
        (transRShoulder, rotRShoulder) = listener.lookupTransform('/right_shoulder_1','/torso_1',now)
        eulerRShoulder = tf.transformations.euler_from_quaternion(rotRShoulder)
        listener.waitForTransform("/right_elbow_1","/right_shoulder_1",now,rospy.Duration(4.0))
        (transRElbow, rotRElbow) = listener.lookupTransform('/right_elbow_1','/right_shoulder_1',now)
        eulerRElbow = tf.transformations.euler_from_quaternion(rotRElbow)
        listener.waitForTransform("/right_hand_1","/right_elbow_1",now,rospy.Duration(4.0))
        (transRWrist, rotRWrist) = listener.lookupTransform('/right_hand_1','/right_elbow_1',now)
        eulerRWrist = tf.transformations.euler_from_quaternion(rotRWrist)
        hello_str.name =  ['HeadYaw', 'HeadPitch', 'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand', 'RFinger23', 'RFinger13', 'RFinger12', 'LFinger21', 'LFinger13', 'LFinger11', 'RFinger22', 'LFinger22', 'RFinger21', 'LFinger12', 'RFinger11', 'LFinger23', 'LThumb1', 'RThumb1', 'RThumb2', 'LThumb2']
        hello_str.position = [eulerHead[2] , eulerHead[1],0.0,-eulerLHip[2], eulerLHip[0], eulerLKnee[0],eulerLAnkle[1] , eulerLAnkle[0], 0.0, -eulerRHip[2], eulerRHip[0], eulerRKnee[0], eulerRAnkle[1], eulerRAnkle[0], eulerLShoulder[0] + math.pi/2, -eulerLShoulder[2] + math.pi/2,eulerLElbow[0]-math.pi/2,  eulerLElbow[1], eulerLWrist[2], 0.0, eulerRShoulder[0] - math.pi/2, +eulerRShoulder[2] - math.pi/2,eulerRElbow[0]-math.pi/2, eulerRElbow[1], eulerRWrist[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        hello_str.velocity = []
        hello_str.effort = []
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
