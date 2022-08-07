#!/usr/bin/env python 

import rospy
import math
import numpy as np
import tf
from tf.transformations import *
from geometry_msgs.msg import PoseStamped
from rosflight_msgs.msg import Attitude
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped

rospy.init_node('pose2quat')

quat_stamped_pub = rospy.Publisher('/rosflight/attitude_stamped', QuaternionStamped, queue_size=10)
quat_correction_pub = rospy.Publisher('/rosflight/attitude_correction', Quaternion, queue_size=10)
br = tf.TransformBroadcaster()


def stamp_quat(attitude_msg):
  global quat_stamped_pub  

  quat_msg = QuaternionStamped()
  quat_msg.quaternion = attitude_msg.attitude
  quat_msg.header.stamp = rospy.Time.now()
  
  quat_stamped_pub.publish(quat_msg)

def convert_pose2quat(pose_msg):
  global quat_correction_pub
  global br

  quat_correction_msg = Quaternion()

  q = (pose_msg.pose.orientation.x,     \
    pose_msg.pose.orientation.y,        \
    pose_msg.pose.orientation.z,        \
    pose_msg.pose.orientation.w) 

  R = np.matrix(quaternion_matrix(q))

  q_rot = quaternion_from_euler(math.pi, 0, 0)
  flip = np.matrix(quaternion_matrix(q_rot))

  R_correction = flip * R * flip
  q_final = quaternion_from_matrix(R_correction)

  q_final /= np.linalg.norm(q_final)
  # q_rot = quaternion_from_euler(math.pi, 0, 0)
  # q_new = quaternion_multiply(q, q_rot)
  # q_final = quaternion_multiply(q_rot, q_new)

  if q_final[3] < 0:
    quat_correction_msg.x = -q_final[0]
    quat_correction_msg.y = -q_final[1]
    quat_correction_msg.z = -q_final[2]
    quat_correction_msg.w = -q_final[3]
  else:
    quat_correction_msg.x = q_final[0]
    quat_correction_msg.y = q_final[1]
    quat_correction_msg.z = q_final[2]
    quat_correction_msg.w = q_final[3]

  br.sendTransform((0, 0, 0),
                   q_final,
                   rospy.Time.now(),
                   "rosflight_correction",
                   "rosflight_world")

  # quat_correction_msg = pose_msg.pose.orientation

  quat_correction_pub.publish(quat_correction_msg)

rospy.Subscriber('/true_pose', PoseStamped, convert_pose2quat)
rospy.Subscriber('/attitude', Attitude, stamp_quat)

rospy.spin()
