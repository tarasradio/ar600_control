import numpy as np

import math
import matplotlib.pyplot as plt

from foot_trajectories import foot_swing_trajectory, foot_support_trajectory, foot_swing_trajectory_magid, foot_swing_trajectory_stairs
from footsteps_generator import calc_footsteps

def lipm(zc, step_height, Tsup, sx, sy, sz, xi, yi, vxi, vyi, px, py, a, b):
  # zc, Tsup: z-axis intersection; support time
  # sx, sy: walk parameters(vectors)
  # xi, yi: initial CoM coordinates
  # px, py: initial foot placement
  # vxi, vyi: initial CoM velocity
  # a, b: params of cost-func

  ## Initialization
  g = 9.8
  
  # Calc steps for all legs
  footsteps = calc_footsteps(sx, sy, sz, 0, 0, 0, Tsup)

  # desired foot placement
  px0 = px
  py0 = py

  Tc = math.sqrt(zc / g)
  C = math.cosh(Tsup / Tc)
  S = math.sinh(Tsup / Tc)
  D = a * (C - 1)**2 + b * (S / Tc)**2

  CoM_trajectory = { 't':[], 'x':[], 'vx':[], 'y':[], 'vy':[], 'z':[] }
  l_foot_trajectory = { 't':[], 'x':[], 'y':[], 'z':[] }
  r_foot_trajectory = { 't':[], 'x':[], 'y':[], 'z':[] }

  n = 0

  def calc_foot_place(px0, py0, px, py):
    px0 = px0 + sx[n - 1]
    py0 = py0 - (-1)**n * sy[n-1]
    
    ## calculate the coordinate (xbar, ybar)
    xbar = sx[n] / 2
    ybar = (-1)**n * sy[n] / 2

    vxbar = (C + 1) / (Tc * S) * xbar
    vybar = (C - 1) / (Tc * S) * ybar
    ## target state of CoM, of the n-th step
    xd = px0 + xbar
    yd = py0 + ybar
    vxd = vxbar
    vyd = vybar
    ## update px, py to be the real foot place in step n
    # a, b are parameters for cost func
    px = - a * (C - 1) / D * (xd - C * xi - Tc * S * vxi) \
        - b * S / (Tc * D) * (vxd - S / Tc * xi - C * vxi)
    py = - a * (C - 1) / D * (yd - C * yi - Tc * S * vyi) \
        - b * S / (Tc * D) * (vyd - S / Tc * yi - C * vyi)

    # plt.plot(px0, py0, 'x')
    # plt.plot(px, py, 'o')

    return px0, py0, px, py

  def calc_foots_trajectories(r_foot_xi, r_foot_yi, r_foot_zi, l_foot_xi, l_foot_yi, l_foot_zi):
    step = 0
    supported_foot = 'r' # first supported foot

    def extend(trajectory, next_part):
      trajectory['t'].extend(next_part['t'])
      trajectory['x'].extend(next_part['x'])
      trajectory['y'].extend(next_part['y'])
      trajectory['z'].extend(next_part['z'])

    lst = foot_support_trajectory(0, l_foot_xi, l_foot_yi, l_foot_zi, Tsup)
    rst = foot_support_trajectory(0, r_foot_xi, r_foot_yi, r_foot_zi, Tsup)

    extend(r_foot_trajectory, rst)
    extend(l_foot_trajectory, lst)

    t_start = 0

    while step < len(footsteps['t']) - 2:
      t_start = footsteps['t'][step] + Tsup

      px_start = footsteps['px'][step]
      py_start = footsteps['py'][step]
      pz_start = footsteps['pz'][step]

      px_end = footsteps['px'][step + 2]
      py_end = footsteps['py'][step + 2]
      pz_end = footsteps['pz'][step + 2]
      
      swing_trajectory = foot_swing_trajectory_magid(t_start, px_start, py_start, pz_start, px_end, py_end, pz_end, Tsup, step_height)

      s_xi = footsteps['px'][step + 1]
      s_yi = footsteps['py'][step + 1]
      s_zi = footsteps['pz'][step + 1]

      support_trajectory = foot_support_trajectory(t_start, s_xi, s_yi, s_zi, Tsup)
      
      if supported_foot == 'r':
        extend(r_foot_trajectory, swing_trajectory)
        extend(l_foot_trajectory, support_trajectory)
        
        supported_foot = 'l'
      elif supported_foot == 'l':
        extend(l_foot_trajectory, swing_trajectory)
        extend(r_foot_trajectory, support_trajectory)

        supported_foot = 'r'

      step += 1
    
    t_start = footsteps['t'][step] + Tsup

    l_xi = footsteps['px'][step + 1]
    l_yi = footsteps['py'][step + 1]
    l_zi = footsteps['pz'][step + 1]

    r_xi = footsteps['px'][step]
    r_yi = footsteps['py'][step]
    r_zi = footsteps['pz'][step]
     

    lst = foot_support_trajectory(t_start, l_xi, l_yi, l_zi, Tsup)
    rst = foot_support_trajectory(t_start, r_xi, r_yi, r_zi, Tsup)

    extend(l_foot_trajectory, lst)
    extend(r_foot_trajectory, rst)
    
  def calc_walk_primitive(ti, dt, xi, vxi, yi, vyi, zi, zd):
    walk_t = { 't':[], 'x':[], 'vx':[], 'y':[], 'vy':[], 'z':[] }

    z_diff = zd - zi

    def next_CoM_trajectory(t):
      _t = t / Tc

      phaze = t / Tsup

      def calc_position(p, v, fp):
        return (p - fp) * math.cosh(_t) + Tc * v * math.sinh(_t) + fp
      
      def calc_velocity(p, v, fp):
        return (p - fp) / Tc * math.sinh(_t) + v * math.cosh(_t)

      return  calc_position(xi, vxi, px), \
              calc_velocity(xi, vxi, px), \
              calc_position(yi, vyi, py), \
              calc_velocity(yi, vyi, py), \
              zi + z_diff * phaze, \
              t + ti

    def append_CoM_trajectory(x, vx, y, vy, z, t):
      walk_t['x'].append(x)
      walk_t['vx'].append(vx)
      walk_t['y'].append(y)
      walk_t['vy'].append(vy)
      walk_t['z'].append(z)
      walk_t['t'].append(t)

    for t in np.arange(0, Tsup, dt):
      trajectory = next_CoM_trajectory(t)
      append_CoM_trajectory(*trajectory)
    
    final_conditions = next_CoM_trajectory(Tsup)

    return walk_t, final_conditions

  T = 0

  px0, py0, px, py = calc_foot_place(px0, py0, px, py) # set the initial placement of foot

  zi = zc
  zd = zi + sz[n]
  ## carry out the steps indicated by walk parameters sx, sy
  while n < len(sx): # sx was expanded by one element previously
    walk_primitive, final_conditions = calc_walk_primitive(T, 0.01, xi, vxi, yi, vyi, zi, zd)

    CoM_trajectory['t'].extend(walk_primitive['t'])
    CoM_trajectory['x'].extend(walk_primitive['x'])
    CoM_trajectory['vx'].extend(walk_primitive['vx'])
    CoM_trajectory['y'].extend(walk_primitive['y'])
    CoM_trajectory['vy'].extend(walk_primitive['vy'])
    CoM_trajectory['z'].extend(walk_primitive['z'])

    xi = final_conditions[0]
    vxi = final_conditions[1]
    yi = final_conditions[2]
    vyi = final_conditions[3]
    
    n += 1; 

    if n < len(sx):
      px0, py0, px, py = calc_foot_place(px0, py0, px, py); # calculate the actual foot place for step n
      zi += sz[n-1]
      zd = zi + sz[n]
    T += Tsup
  
  calc_foots_trajectories(footsteps['px'][0], footsteps['py'][0], footsteps['pz'][0], footsteps['px'][1], footsteps['py'][1], footsteps['pz'][1])

  return CoM_trajectory, l_foot_trajectory, r_foot_trajectory