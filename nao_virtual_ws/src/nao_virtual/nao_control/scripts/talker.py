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
from LowerHalfTrans import *
from calcAnkleLine import *
from judge import *
import numpy as np

def talker():
    pubHeadPitch = rospy.Publisher('/nao_dcm/HeadPitch_position_controller/command', Float64, queue_size=10)
    pubHeadYaw = rospy.Publisher('/nao_dcm/HeadYaw_position_controller/command', Float64, queue_size=10)
    pubLAnklePitch = rospy.Publisher('/nao_dcm/LAnklePitch_position_controller/command', Float64, queue_size=10)
    pubLAnkleRoll = rospy.Publisher('/nao_dcm/LAnkleRoll_position_controller/command', Float64, queue_size=10)
    pubLElbowRoll = rospy.Publisher('/nao_dcm/LElbowRoll_position_controller/command', Float64, queue_size=10)
    pubLElbowYaw = rospy.Publisher('/nao_dcm/LElbowYaw_position_controller/command', Float64, queue_size=10)
    pubLHand = rospy.Publisher('/nao_dcm/LHand_position_controller/command', Float64, queue_size=10)
    pubLHipPitch = rospy.Publisher('/nao_dcm/LHipPitch_position_controller/command', Float64, queue_size=10)
    pubLHipRoll = rospy.Publisher('/nao_dcm/LHipRoll_position_controller/command', Float64, queue_size=10)
    pubLHipYawPitch = rospy.Publisher('/nao_dcm/LHipYawPitch_position_controller/command', Float64, queue_size=10)
    pubLKneePitch = rospy.Publisher('/nao_dcm/LKneePitch_position_controller/command', Float64, queue_size=10)
    pubLShoulderPitch = rospy.Publisher('/nao_dcm/LShoulderPitch_position_controller/command', Float64, queue_size=10)
    pubLShoulderRoll = rospy.Publisher('/nao_dcm/LShoulderRoll_position_controller/command', Float64, queue_size=10)
    pubLWristYaw = rospy.Publisher('/nao_dcm/LWristYaw_position_controller/command', Float64, queue_size=10)
    pubRAnklePitch = rospy.Publisher('/nao_dcm/RAnklePitch_position_controller/command', Float64, queue_size=10)
    pubRAnkleRoll = rospy.Publisher('/nao_dcm/RAnkleRoll_position_controller/command', Float64, queue_size=10)
    pubRElbowRoll = rospy.Publisher('/nao_dcm/RElbowRoll_position_controller/command', Float64, queue_size=10)
    pubRElbowYaw = rospy.Publisher('/nao_dcm/RElbowYaw_position_controller/command', Float64, queue_size=10)
    pubRHand = rospy.Publisher('/nao_dcm/RHand_position_controller/command', Float64, queue_size=10)
    pubRHipPitch = rospy.Publisher('/nao_dcm/RHipPitch_position_controller/command', Float64, queue_size=10)
    pubRHipRoll = rospy.Publisher('/nao_dcm/RHipRoll_position_controller/command', Float64, queue_size=10)
    pubRHipYawPitch = rospy.Publisher('/nao_dcm/RHipYawPitch_position_controller/command', Float64, queue_size=10)
    pubRKneePitch = rospy.Publisher('/nao_dcm/RKneePitch_position_controller/command', Float64, queue_size=10)
    pubRShoulderPitch = rospy.Publisher('/nao_dcm/RShoulderPitch_position_controller/command', Float64, queue_size=10)
    pubRShoulderRoll = rospy.Publisher('/nao_dcm/RShoulderRoll_position_controller/command', Float64, queue_size=10)
    pubRWristYaw = rospy.Publisher('/nao_dcm/RWristYaw_position_controller/command', Float64, queue_size=10)
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
    thetaLPitchO = thetaLRollO = thetaRPitchO = thetaRRollO = 0
    thetaLHipPitchO = thetaRHipPitchO = 0
    thetaLHipRollO = thetaRHipRollO = 0
    thetaLKneeO = thetaRKneeO = 0
    thetaLAnklePitchO = thetaRAnklePitchO = 0
    thetaLAnkleRollO = thetaRAnkleRollO = 0
    HipPitchMin = -1.535889
    HipPitchMax = 0.48409
    HipRollMin = -0.379472
    HipRollMax = 0.790477
    KneeMin = -0.092346
    KneeMax =  2.112528
    AnklePitchMin = -1.189516
    AnklePitchMax = 0.922747
    AnkleRollMin = -0.397880
    AnkleRollMax = 0.769001
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
        hello_str.position = [eulerHead[2] , eulerHead[1],0.0,-eulerLHip[2], eulerLHip[0], eulerLKnee[0],eulerLAnkle[1] , eulerLAnkle[0], 0.0, -eulerRHip[2], eulerRHip[0], eulerRKnee[0], eulerRAnkle[1], eulerRAnkle[0], eulerLShoulder[0] + math.pi/2, -eulerLShoulder[2] + math.pi/2,eulerLElbow[0]-math.pi/2,  eulerLElbow[1], eulerLWrist[2], 0.0, eulerRShoulder[0] + math.pi/2, -eulerRShoulder[2] - math.pi/2,-eulerRElbow[0]+math.pi/2, eulerRElbow[1], eulerRWrist[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        hello_str.velocity = []
        hello_str.effort = []

        #parameters
        roLHipPitch = hello_str.position[4]
        roLHipRoll = hello_str.position[3]
        roRHipPitch = hello_str.position[10]
        roRHipRoll = hello_str.position[9]
        roLKnee = hello_str.position[5]
        roRKnee = hello_str.position[11]
        roLShoulderPitch = hello_str.position[14]
        roLShoulderRoll = hello_str.position[15]
        roRShoulderPitch = hello_str.position[20]
        roRShoulderRoll = hello_str.position[21]
        roLElbowYaw = hello_str.position[16]
        roLElbowRoll = hello_str.position[17]
        roRElbowYaw = hello_str.position[22]
        roRElbowRoll = hello_str.position[23]


        ThetaL = np.array([roLHipPitch,roLHipRoll,roRHipPitch,roRHipRoll,roLKnee,roRKnee])
        ThetaU = np.array([roLShoulderPitch, roLShoulderRoll, roRShoulderPitch, roRShoulderRoll, 
                          roLElbowYaw, roLElbowRoll, roRElbowYaw, roRElbowRoll])
        result = calcAnkleLine(ThetaL, ThetaU)
        #result = judge(result,ThetaL, ThetaU)

        thetaLPitch = result[3]
        thetaLRoll = result[4]
        thetaRPitch = result[5]
        thetaRRoll = result[6]
        #define restrict for joint angles
        #http://doc.aldebaran.com/1-14/family/robots/joints_robot.html#robot-joints-v4-left-leg-joints
        if roLHipPitch < HipPitchMin or roLHipPitch > HipPitchMax:
            roLHipPitch = thetaLHipPitchO
        else:
            thetaLHipPitchO = roLHipPitch

        if roRHipPitch < HipPitchMin or roRHipPitch > HipPitchMax:
            roRHipPitch = thetaRHipPitchO
        else:
            thetaRHipPitchO = roRHipPitch

        if roLHipRoll < HipRollMin or roLHipRoll > HipRollMax:
            roLHipRoll = thetaLHipRollO
        else:
            thetaLHipRollO = roLHipRoll

        if roRHipRoll < HipRollMin or roRHipRoll > HipRollMax:
            roRHipRoll = thetaRHipRollO
        else:
            thetaRHipRollO = roRHipRoll

        if roLKnee < KneeMin or roLKnee > KneeMax:
            roLKnee = thetaLKneeO
        else:
            thetaLKneeO = roLKnee

        if roRKnee < KneeMin or roRKnee > KneeMax:
            roRKnee = thetaRKneeO
        else:
            thetaRKneeO = roRKnee
        #if ankle angle exceeds the limit, restore all angles
        if thetaLPitch < AnklePitchMin or thetaLPitch > AnklePitchMax \
            or thetaRPitch < AnklePitchMin or thetaRPitch > AnklePitchMax \
            or thetaLRoll < AnkleRollMin or thetaLRoll > AnkleRollMax \
            or thetaRRoll < AnkleRollMin or thetaRRoll > AnkleRollMax:
            thetaLPitch = thetaLAnklePitchO
            thetaRPitch = thetaRAnklePitchO
            thetaLRoll = thetaLAnkleRollO
            thetaRRoll = thetaRAnkleRollO
            roLHipPitch = thetaLHipPitchO
            roRHipPitch = thetaRHipPitchO
            roLHipRoll = thetaLHipRollO
            roRHipRoll = thetaRHipRollO
            roLKnee = thetaLKneeO
            roRKnee = thetaRKneeO
        else:
            thetaLAnklePitchO = thetaLPitch
            thetaRAnklePitchO = thetaRPitch
            thetaLAnkleRollO = thetaLRoll
            thetaRAnkleRollO = thetaRRoll

        print("thetaLPitch, thetaLRoll, thetaPitch, thetaRRoll: ",thetaLPitch, thetaLRoll, thetaRPitch, thetaRRoll)

        #pubHeadPitch.publish(hello_str.position[1])
        #pubHeadYaw.publish(hello_str.position[0])
        pubLAnklePitch.publish(thetaLPitch) 
        pubLAnkleRoll.publish(thetaLRoll) 
        pubLElbowRoll.publish(hello_str.position[17])
        pubLElbowYaw.publish(hello_str.position[16])
        #pubLHand.publish(hello_str.position[19])
        pubLHipPitch.publish(roLHipPitch)
        pubLHipRoll.publish(roLHipRoll)
        #pubLHipYawPitch.publish(hello_str.position[2])
        pubLKneePitch.publish(roLKnee)
        pubLShoulderPitch.publish(hello_str.position[14])
        pubLShoulderRoll.publish(hello_str.position[15])
        #pubLWristYaw.publish(hello_str.position[18])
        pubRAnklePitch.publish(thetaRPitch)
        pubRAnkleRoll.publish(thetaRRoll)
        pubRElbowRoll.publish(hello_str.position[23])
        pubRElbowYaw.publish(hello_str.position[22])
        #pubRHand.publish(hello_str.position[25])
        pubRHipPitch.publish(roRHipPitch)
        pubRHipRoll.publish(roRHipRoll)
        #pubRHipYawPitch.publish(hello_str.position[8])
        pubRKneePitch.publish(roRKnee)
        pubRShoulderPitch.publish(hello_str.position[20])  
        pubRShoulderRoll.publish(hello_str.position[21])
        #pubRWristYaw.publish(hello_str.position[24])

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
