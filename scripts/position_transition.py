from robot_gazebo import RobotGateway
from leg_joints import tf_leg_joints

import rospy

class PositionController:

  def __init__(self, robot:RobotGateway) -> None:
      self.robot = robot
  
  def calc_deltas(self, target, initial, ticks) -> tuple:
    deltas = []
    if len(target) != len(initial):
      raise Exception('The target and initial must be the same length! ')
    for i in range(0, len(target)):
      delta = (target[i] - initial[i]) / ticks
      deltas.append(delta)
    return tuple(deltas)

  def next_position(self, position:list, deltas):
    if len(position) != len(deltas):
      raise Exception('The position and deltas must be the same length! ')

    for i in range(0, len(position)):
      position[i] += deltas[i]

  def take_position(self, T, dt, rate:rospy.Rate, l_target:tuple, r_target:tuple):
    print('Transition started.')
    
    l_initial = self.robot.getLegJointsPosition('left')
    r_initial = self.robot.getLegJointsPosition('right')

    l_initial = tf_leg_joints(*l_initial, 'left')
    r_initial = tf_leg_joints(*r_initial, 'right')

    l_position = list(l_initial)
    r_position = list(r_initial)

    n = T / dt # Number of ticks

    l_deltas = self.calc_deltas(l_target, l_initial, n)
    r_deltas = self.calc_deltas(r_target, r_initial, n)

    t = 0
    while t < T:
      self.robot.sendLegJointsPosition('left', l_position)
      self.robot.sendLegJointsPosition('right', r_position)

      self.next_position(l_position, l_deltas)
      self.next_position(r_position, r_deltas)

      t += dt
      rate.sleep()

    print('transition completed.')