
import matplotlib.pyplot as plt

def calc_footsteps(sx, sy, sz, xi, yi, zi, Tsup):
  # the step after the last step should be zero
  sx.append(0)
  sy.append(0)
  sz.append(0)

  steps = { 't':[], 'px':[], 'py':[], 'pz':[] }
  n_steps = len(sx)
  n = 0; t = 0.
  px = xi; py = yi; pz = zi

  while n < n_steps:
    steps['t'].append(t)
    ## calculate the desired foot place during the n-th step
    if n != 0:
      px = px + sx[n-1]
      py = py - (-1)**n * sy[n-1]
      pz = pz + sz[n-1]
    steps['px'].append(px)
    steps['py'].append(py)
    steps['pz'].append(pz)

    print('px:', px, 'py:', py, 'pz:', pz)

    t += Tsup
    n += 1
  
  plt.title("Координаты точек шагов (стоп) робота.")
  plt.plot(steps['px'], steps['py'], 'x')
  plt.show()

  plt.title("Координаты точек шагов (стоп) робота.")
  plt.plot(steps['px'], steps['pz'], 'x')
  plt.show()
    
  return steps