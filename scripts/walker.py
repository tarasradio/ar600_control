#!/usr/bin/env python

from position_transition import PositionController
from robot_description import RobotDesc

from lipm import lipm
from legs_controller import find_legs_joints
from plotter import plot_trajectories

from robot_gazebo import RobotGateway
import rospy

robot = RobotGateway()
positionController = PositionController(robot)

def walker():
  robot.start()

  rospy.init_node('walker', anonymous=True)
  rate = rospy.Rate(50)

  # footstep planner's result
  
  # side walking from right foot
  # Kukhno virsion
  # sx = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  # sy = [0.35, 0.18, 0.35, 0.18, 0.35, 0.18, 0.35, 0.18, 0.35]
  # sz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  # sx = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  # sy = [0.35, 0.18, 0.35, 0.18, 0.35, 0.18, 0.35, 0.18, 0.35]
  # sz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  # forward walk with acceleration and deceleration
  sx = [0.0, 0.1, 0.15, 0.2, 0.25, 0.25, 0.25, 0.2, 0.15, 0.1, 0.0]
  sy = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
  sz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  # walking in place
  # sx = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  # sy = [0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18]

  # walking stairs
  # sx = [0.0, 0.25, 0.25, 0.25, 0.0]
  # sy = [0.3, 0.3, 0.3, 0.3, 0.3]
  # sz = [0, 0.0, 0.0, 0.0, 0]

  CoM_height = RobotDesc.CoM_height()
  CoM_z = CoM_height - 0.08

  Tsup = 0.8 * 4

  step_height = 0.15

  CoM_t, l_foot_t, r_foot_t = lipm(CoM_z, step_height, Tsup, sx, sy, sz, 0, 0, 0, 0, 0, 0, 10, 1)

  plot_trajectories(CoM_t, l_foot_t, r_foot_t)

  leg_l_joints, leg_r_joints = find_legs_joints(CoM_t, l_foot_t, r_foot_t)

  print('Walker started.')

  # Before starting to walk, the robot must take the initial position 
  positionController.take_position(Tsup, 0.01, rate, leg_l_joints['joints'][0], leg_r_joints['joints'][0])

  rospy.loginfo('Walk started.')

  i = 0
  while not rospy.is_shutdown():
    while i < len(leg_l_joints['t']):
      robot.sendLegJointsPosition('left', leg_l_joints['joints'][i])
      robot.sendLegJointsPosition('right', leg_r_joints['joints'][i])
      i += 1

      rate.sleep()
    else:
      rospy.loginfo('Walk stopped.')
      break

if __name__ == '__main__':
  try:
    walker()
  except rospy.ROSInterruptException:
    pass