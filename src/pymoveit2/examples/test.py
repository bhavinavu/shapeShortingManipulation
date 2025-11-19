import numpy as np






a = np.array([[0.000, -1.000,  0.020,  0.000],
  [0.000,  0.000,  1.000,  0.200],
 [-1.000,  0.000,  0.000,  0.700],
  [0.000,  0.000,  0.000,  1.000]])

t_base = np.array([0.128, 0.072, 0, 1])

t_cam = t_base @ np.linalg.inv(a)

print(t_cam)
