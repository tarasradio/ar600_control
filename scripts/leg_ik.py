import math
from math import pi
import numpy as np
import numpy.linalg

from scipy.spatial.transform import Rotation

# D - distance between the body origin and the hip joint.
# A - hip (upper leg) length
# B - shin (lower leg) length

def find_leg_joints(body, D, A, B, foot):
  body_p = np.array(body['p'])
  body_R = np.array(body['R'])
  foot_p = np.array(foot['p'])
  foot_R = np.array(foot['R'])

  D_mult = np.array([0, D, 0])
  
  # crotch from ankle
  p2 = body_p + np.dot(body_R, D_mult.T)
  r = foot_R.T.dot(p2 - foot_p)
  C = numpy.linalg.norm(r)

  c5 = (C**2 - A**2 - B**2) / (2.0*A*B)

  if c5 >= 1:
    knee_pitch = 0.0
  elif c5 <= -1:
    knee_pitch = pi
  else:
    knee_pitch = math.acos(c5)

  # ankle pitch sub
  q6a = math.asin((A / C) * math.sin(pi - knee_pitch))

  # -pi/2 < q7 < pi/2
  ankle_roll = math.atan2(r[1], r[2]) 
  if ankle_roll > pi/2:
    ankle_roll -= pi 
  elif ankle_roll < -pi/2:
    ankle_roll += pi
  
  ankle_pitch = -math.atan2(r[0], np.sign(r[2]) * math.sqrt(r[1]**2 + r[2]**2)) - q6a

  Rx = Rotation.from_euler('x', -ankle_roll).as_matrix()
  Ry = Rotation.from_euler('y', -ankle_pitch - knee_pitch).as_matrix()

  R = body_R.T.dot(foot_R).dot(Rx).dot(Ry) ## hipZ * hipX * hipY
  
  hip_yaw = math.atan2(-R[0, 1], R[1, 1]); 

  cz = math.cos(hip_yaw); sz = math.sin(hip_yaw)

  hip_roll = math.atan2(R[2, 1], -R[0, 1] * sz + R[1, 1] * cz)
  hip_pitch = math.atan2(-R[2, 0], R[2, 2]); 

  return hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll

def check_leg_joints(joints):
  roll_sum = joints[1] + joints[5]
  pitch_sum = joints[2] + joints[3] + joints[4]

  if math.isclose(roll_sum, 0, abs_tol=0.001) != True:
    print("Roll joints are incorrect.")
  if math.isclose(pitch_sum, 0, abs_tol=0.001) != True:
      print("Pitch joints are incorrect.")