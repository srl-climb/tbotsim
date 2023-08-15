import numpy as np
import matplotlib.pyplot as plt
from tbotlib import *

points = np.array([[1,1],[1,2],[2,2],[3,2],[3,3]])

# smooth path
points_smoothed = BsplineSmoother(ds = 0.1).smooth(points.T).T
points_smoothed_coarse = BsplineSmoother(ds = 0.5).smooth(points.T).T

""" # orginal path
plt.plot(points[:,0], points[:,1],ls='--')
plt.scatter(points[:,0], points[:,1])

# smoothed path
plt.plot(points_smoothed[:,0], points_smoothed[:,1],ls='--')
plt.scatter(points_smoothed_coarse[:,0], points_smoothed_coarse[:,1]) """

# simulate some inverse kinematics
q = np.sqrt(points_smoothed_coarse[:,0]**2 + points_smoothed_coarse[:,1]**2)
q = np.sin(0.7*points_smoothed_coarse[:,1]/points_smoothed_coarse[:,0])
t = np.linspace(0, 1, len(q))

t_spline, q_spline, _, _, _ = polysplinefit(q, t[1]-t[0], 0.01)
t_spline_coarse, q_spline_coarse, _, _, _ = polysplinefit(q, t[1]-t[0], 0.05)

""" plt.scatter(t, q)
plt.plot(t, q, ls='--')
plt.scatter(t_spline_coarse, q_spline_coarse, marker='o')
plt.plot(t_spline, q_spline)
plt.show() """


d=','
""" np.savetxt('trajectory_path.csv', points, delimiter=d)
np.savetxt('trajectory_smoothedpath.csv', points_smoothed, delimiter=d)
np.savetxt('trajectory_coarsesmoothedpath.csv', points_smoothed_coarse, delimiter=d)
np.savetxt('trajectory_q.csv', np.vstack((t.T,q)).T, delimiter=d)
np.savetxt('trajectory_spline.csv', np.vstack((t_spline,q_spline)).T, delimiter=d)
np.savetxt('trajectory_coarsespline.csv', np.vstack((t_spline_coarse,q_spline_coarse)).T, delimiter=d) """

q = np.array([1,1,2,1.5])
t = np.array([0,1,2,3])

t_spline, q_spline, _, _, _ = polysplinefit(q, t[1]-t[0], 0.01)
""" plt.scatter(t, q)
plt.plot(t, q, ls='--')
plt.plot(t_spline, q_spline)
plt.show() """
np.savetxt('spline_q.csv', np.vstack((t.T,q)).T, delimiter=d)
np.savetxt('spline_spline.csv', np.vstack((t_spline,q_spline)).T, delimiter=d)