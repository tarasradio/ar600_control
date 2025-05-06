from robot_description import RobotDesc

from leg_ik import find_leg_joints, check_leg_joints

import math
import numpy as np
import matplotlib.pyplot as plt

from typing import Dict

from plotter import plot_leg_joints_trajectories

pelvis_to_hip = RobotDesc.pelvis_to_hip()
hip_length = RobotDesc.hip_length
shin_length = RobotDesc.shin_length

def print_leg_joints(joints):
  print('- hip yaw (vertical-):', '{:.2f}'.format(math.degrees(joints[0])))
  print('- hip roll (forward+):', '{:.2f}'.format(math.degrees(joints[1])))
  print('- hip pitch (right+):', '{:.2f}'.format(math.degrees(joints[2])))
  print('- knee (right+):', '{:.2f}'.format(math.degrees(joints[3])))
  print('- ankle pitch (right+):', '{:.2f}'.format(math.degrees(joints[4])))
  print('- ankle roll (forward+):', '{:.2f}'.format(math.degrees(joints[5])))

def find_legs_joints(CoM_path:Dict, l_foot_path:Dict, r_foot_path:Dict):
  E = np.eye(3)

  body = { 'p': [0, 0, 0] , 'R': E }
  l_foot = { 'p': [0, 0, 0], 'R': E }
  r_foot = { 'p': [0, 0, 0], 'R': E }

  l_joints = { 't': [], 'joints': [] }
  r_joints = { 't': [], 'joints': [] }

  n = min(len(CoM_path['t']), len(l_foot_path['t']), len(r_foot_path['t']))

  i = 0; t = 0; dt = 0.01

  while i < n:
    body['p'][0] = CoM_path['x'][i]
    body['p'][1] = CoM_path['y'][i]
    body['p'][2] = CoM_path['z'][i] - (RobotDesc.hip_fork_length + RobotDesc.foot_height)

    l_foot['p'][0] = l_foot_path['x'][i]
    l_foot['p'][1] = l_foot_path['y'][i]
    l_foot['p'][2] = l_foot_path['z'][i]

    r_foot['p'][0] = r_foot_path['x'][i]
    r_foot['p'][1] = r_foot_path['y'][i]
    r_foot['p'][2] = r_foot_path['z'][i]

    l_joints['t'].append(t)
    r_joints['t'].append(t)

    l_joints['joints'].append(find_leg_joints(body, pelvis_to_hip, hip_length, shin_length, l_foot))
    r_joints['joints'].append(find_leg_joints(body, -pelvis_to_hip, hip_length, shin_length, r_foot))

    t += dt; i += 1

  plot_leg_joints_trajectories(l_joints, r_joints)

  return l_joints, r_joints

def calc_body_turn(body_z, body_x_d = 0, body_y_d = 0, body_z_d = 0):
  E = np.eye(3)

  body = { 'p':[0, 0, body_z - (RobotDesc.hip_fork_length + RobotDesc.foot_height)], 'R': E }
  l_foot = { 'p':[0, pelvis_to_hip, 0], 'R': E }
  r_foot = { 'p':[0, -pelvis_to_hip, 0], 'R': E }

  t = 0
  T = 10; P = 2

  body_path = {'t':[], 'x':[], 'y':[], 'z':[]}

  l_joints = { 't':[], 'joints':[]}
  r_joints = { 't':[], 'joints':[]}

  while t < T:
    phase = (t % P) / P * math.pi * 2

    z = body_z - math.sin(phase) * body_z_d - (RobotDesc.hip_fork_length + RobotDesc.foot_height)

    x = math.sin(phase) * body_x_d
    y = math.cos(phase) * body_y_d

    body['p'][0] = x
    body['p'][1] = y
    body['p'][2] = z

    body_path['t'].append(t)
    body_path['x'].append(x)
    body_path['y'].append(y)
    body_path['z'].append(z)

    j_l = find_leg_joints(body, pelvis_to_hip, hip_length, shin_length, l_foot)
    j_r = find_leg_joints(body, -pelvis_to_hip, hip_length, shin_length, r_foot)

    check_leg_joints(j_l)
    check_leg_joints(j_r)
    
    l_joints['t'].append(t)
    l_joints['joints'].append(j_l)

    r_joints['t'].append(t)
    r_joints['joints'].append(j_r)

    t += 0.01

  plt.plot(body_path['x'], body_path['y'])
  plt.show()

  return l_joints, r_joints