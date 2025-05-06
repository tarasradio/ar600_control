import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

def plot_trajectories(CoM_t, l_foot_t, r_foot_t):
  fig, ax = plt.subplots()
  
  plt.title("Траектории ц.м. (x) и стоп робота (x, z).")

  plt.xlabel("t, sec") # ось абсцисс
  plt.ylabel("x, meters") # ось ординат

  ax.plot(CoM_t['t'], CoM_t['z'], 'r--', label="CoM (z)")
  ax.plot(l_foot_t['t'], l_foot_t['x'], label="Left foot (x)")
  ax.plot(r_foot_t['t'], r_foot_t['x'], label="Right foot (x)")
  ax.plot(l_foot_t['t'], l_foot_t['z'], label="Left foot (z)")
  ax.plot(r_foot_t['t'], r_foot_t['z'], label="Right foot (z)")

  ax.legend()

  ax.xaxis.set_major_formatter(ticker.FormatStrFormatter('%0.1f'))

  plt.show()

def plot_leg_joints_trajectories(l_joints, r_joints):
  plt.title("Траектории приводов ног робота.")

  plt.xlabel("t, sec") # ось абсцисс
  plt.ylabel("angle, radians") # ось ординат

  plt.plot(l_joints['t'], l_joints['joints'])
  plt.plot(r_joints['t'], r_joints['joints'])
  plt.show()