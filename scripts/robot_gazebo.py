# This module keeps robot state and sends commands to joints

import rospy
from std_msgs.msg import Float64
from joint_states_listener.srv import ReturnJointStates

from leg_joints import tf_leg_joints

import time
import sys
import os

class RobotGateway:

  def __init__(self):
    self.ns = 'ar600/'
    self.leg_joints = [ 'hip_r', 'hip_s', 'hip_f', 'knee', 'ankle_f', 'ankle_s' ] # r - yaw, f - pitch, s - roll
    self.joint_pubs = {}

  def start(self):
    self.create_publishers()
    # rospy.init_node('robot_gateway', anonymous=True)

  def get_joint_fullname(self, joint, side):
    return joint + '_joint_' + side

  def create_leg_publishers(self, side):
    for joint in self.leg_joints:
      full_joint_name = self.get_joint_fullname(joint, side)
      topic = self.ns + full_joint_name + '_position_controller/command'
      pub = rospy.Publisher(topic, Float64, queue_size=10)

      self.joint_pubs[full_joint_name] = pub

  def create_publishers(self):
    self.create_leg_publishers('left')
    self.create_leg_publishers('right')

  def sendLegJointsPosition(self, side, positions):
    positions = tf_leg_joints(*positions, side)
    
    self.joint_pubs[self.get_joint_fullname('hip_r', side)].publish(positions[0]) # yaw
    self.joint_pubs[self.get_joint_fullname('hip_s', side)].publish(positions[1]) # roll
    self.joint_pubs[self.get_joint_fullname('hip_f', side)].publish(positions[2]) # pitch
    self.joint_pubs[self.get_joint_fullname('knee', side)].publish(positions[3]) # pitch
    self.joint_pubs[self.get_joint_fullname('ankle_f', side)].publish(positions[4]) # pitch
    self.joint_pubs[self.get_joint_fullname('ankle_s', side)].publish(positions[5]) # roll
  
  def getJointNames(self, side):
    joint_names = []
    for joint in self.leg_joints:
      joint_names.append(self.get_joint_fullname(joint, side))
    return joint_names

  def getLegJointsPosition(self, side) -> tuple:
    joint_names = self.getJointNames(side)
    (position, velocity, effort) = self.call_return_joint_states(joint_names)

    return position

  def call_return_joint_states(self, joint_names):
    rospy.wait_for_service("/ar600/return_joint_states")
    try:
      s = rospy.ServiceProxy("/ar600/return_joint_states", ReturnJointStates)
      resp = s(joint_names)
    except rospy.ServiceException:
      print ('error when calling return_joint_states')
      sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
      if(not resp.found[ind]):
        print (")joint %s not found!"%joint_name)
    return (resp.position, resp.velocity, resp.effort)