from tbotlib import * 
import os
import numpy as np

tbot1: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))
tbot2: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))
tbot3: TbTetherbot = TbTetherbot.load(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'))
tbots = [tbot1, tbot2, tbot3]

w = np.array([1,1,1,0.1,0.1,0.1])
W1 = TbElliptoidWrenchSet(w)
W2 = TbRectangleWrenchSet(-w, w)
W3 = TbRectangleWrenchSet(-w, w)
tbot1.W = W1
tbot2.W = W2
tbot3.W = W3

tbot1.W.parent = tbot1.platform
tbot2.W.parent = tbot2.platform
tbot3.W.parent = tbot3

# First test: start position

for tbot in tbots:
    tbot.platform.T_local = TransformMatrix([ 1.67824518e+00 , 1.60000000e+00, -2.42591111e-01, -1.93541049e-15, 2.29183118e+01 ,-4.97000342e-15])
    tbot.place_all([34 ,35 ,33 ,23, 21])


print('First test: start position')
print('elliptoid:                      ', tbot1.stability())
print('rectangle, platform referenced: ', tbot2.stability())
print('rectangle, world referenced:    ', tbot3.stability())


# Second test: rotating world

for tbot in tbots:
    tbot.T_local = TransformMatrix().rotate(2,54,34)

print('Second test: rotating world')
print('elliptoid:                      ', tbot1.stability())
print('rectangle, platform referenced: ', tbot2.stability())
print('rectangle, world referenced:    ', tbot3.stability())


# Third test: equivalent stance

for tbot in tbots:
    tbot.platform.T_local = TransformMatrix([ 4.92898610e-01,  4.00000000e-01,  8.55451796e-02, -6.83302773e-16,  5.72957795e+00, -6.84442941e-15])
    tbot.place_all([13 ,14 ,12 ,2, 0])

print('Third test: equivalent stance')
print('elliptoid:                      ', tbot1.stability())
print('rectangle, platform referenced: ', tbot2.stability())
print('rectangle, world referenced:    ', tbot3.stability())

# Third test: equivalent stance

for tbot in tbots:
    tbot.place_all([13 ,14 ,12 ,2, -1])

print('Fourth test: equivalent stance, 4 gripper removed')
print('elliptoid:                      ', tbot1.stability())
print('rectangle, platform referenced: ', tbot2.stability())
print('rectangle, world referenced:    ', tbot3.stability())

print('Fith test: qhull vs hyperplane shifting')
print('rectangle, platform referenced, hyperplane shifting: ', tbot2.stability())
tbot2._cwsolver = QuickHull()
print('rectangle, platform referenced, quick hull: ', tbot2.stability())

for tbot in tbots:
    tbot.place_all([13 ,14 ,12 ,2, 0])

print('Sixth test: gripper forceset')
tbot2.F = TbTetherForceSet(0, 100, parent = tbot2)
print('rectangle, platform referenced, tether force set: ', tbot2.stability())
tbot2.F = TbTetherGripperForceSet(0, 100, PanorelGripperFoceModel(), parent = tbot2)
print('rectangle, platform referenced, tether gripper force set: ', tbot2.stability())

for tbot in tbots:
    tbot.place_all([-1 ,14 ,12 ,2, 0])

print('Seventh test: gripper forceset one gripper removed')
tbot2.F = TbTetherForceSet(0, 100, parent = tbot2)
print('rectangle, platform referenced, tether force set: ', tbot2.stability())
tbot2.F = TbTetherGripperForceSet(0, 100, PanorelGripperFoceModel(), parent = tbot2)
print('rectangle, platform referenced, tether gripper force set: ', tbot2.stability())

print('Eigth test: finding and checking force distribution')
solver = QuadraticProgram()
f = solver.eval(np.array([1,0,0,0,0,0]), tbot2.F)
if f[0] is not None:
    print(f[1])
    print(np.round(f[0], 3))
    print(tbot2.F.in_set(f[0]))
print('done')

