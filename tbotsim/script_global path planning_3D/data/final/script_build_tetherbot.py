from tbotlib import *
from math import pi
import numpy as np
import os

# Create assets

#
tethers  = [TbTether.create() for _ in range(10)]
for tether in tethers:
    tether.add_collidable(TbTetherCollidable(radius = 0.05))
    tether.add_geometry(TbTethergeometry(radius = 0.005))

#
link_1 = TbRevoluteLink(q0=0, alpha=-pi/2, a=0, d=0.105, qlim=[-pi, pi])
link_1.add_geometry(TbCylinder(radius=0.050, height=0.05, T_local=[0,0,0,90,0,0]))
link_1.add_geometry(TbCylinder(radius=0.016, height=0.30, T_local=[0,0,0.15,0,0,0]))
link_2 = TbPrismaticLink(phi=0, alpha=-pi/2, a=0, q0=0.314, qlim=[0.314,1.414])
link_2.add_geometry(TbCylinder(radius=0.014, height=1.200, T_local=[0,0.616,0,90,0,0]))
link_2.add_geometry(TbCylinder(radius=0.014, height=0.016, T_local=[0,0.007,0,90,0,0]))
link_2.add_geometry(TbCylinder(radius=0.016, height=0.10, T_local=[0,0,-0.025,0,0,0])) #height=0.050, T_local=[0,0,0,0,0,0
link_3 = TbPrismaticLink(phi=0, alpha=0, a=0, q0=0.045, qlim=[-0.05, 0.3]) #0.045
link_3.add_geometry(TbCylinder(radius=0.014, height=0.2, T_local=[0,0,-0.1,0,0,0]))
link_3.add_collidable(TbBoxCollidable([0.04,0.04,0.2], T_local = [-0.02,-0.02,-0.2]))
arm = TbRPPArm(T_local=[0,0,0.05], links=[link_1, link_2, link_3])

#
B = np.array([[0.2, 0, 0.05], [0.2, 0, -0.05], [0.2, 0.15, 0.05], [0.2, 0.15,-0.05], [-0.2, 0.15, 0.05], 
              [-0.2, 0.15,-0.05], [-0.2,-0.15, 0.05],[-0.2,-0.15,-0.05],[ 0.2,-0.15, 0.05], [0.2,-0.15,-0.05]])
#B = np.array([[0.2, 0.025, 0.05], [0.2, -0.025, -0.05], [0.15, 0.15, 0.05], [0.2, 0.15,-0.05], [-0.15, 0.15, 0.05], 
#              [-0.2, 0.15,-0.05], [-0.15,-0.15, 0.05],[-0.2,-0.15,-0.05],[ 0.15,-0.15, 0.05], [0.2,-0.15,-0.05]])
platform = TbPlatform.create(B, arm)
platform.add_geometry(TbCylinder(radius=0.1, height=0.08, T_local=[0,0,0.05+0.08/2]))
platform.add_geometry(TbBox([0.4,0.3,0.1], T_local = TransformMatrix([-0.2,-0.15,-0.05])))
platform.add_collidable(TbBoxCollidable([0.45,0.35,0.11], T_local = TransformMatrix([-0.225,-0.175,-0.055])))

#
C = cylindricalgrid([0,0.5,1,1.5,2,2.5], [0,0.5,1,1.5,2,2.5], 4)
C = cylindricalgrid([0,0.45,0.9,1.35,1.8,2.25], [0,0.45,0.9,1.35,1.8,2.25], 4)
#C = cylindricalgrid([0,0.4,0.8,1.2,1.6,2], [0,0.4,0.8,1.2,1.6,2], 4)
C = np.column_stack((C[0:3,:].T, np.zeros((C.shape[1],1)), C[3,:], np.zeros((C.shape[1],1))))
holds = TbHold.batch(C, hoverpoint = [0,0,0.065],  grippoint = [0,0,0.015]) 
for hold in holds:
    hold.add_geometry(TbCylinder(radius = 0.05, height = 0.03, T_local = TransformMatrix([0,0,0])))

#
grippers = []
for _ in range(5):
    geometries = [TbCylinder(T_local = [0,0,0.075], radius = 0.015, height = 0.15), TbSphere(T_local = [0,0,0.15], radius=0.02)]
    collidables = [TbBoxCollidable([0.05,0.05,0.17], T_local = [-0.025,-0.025,0.0])]
    hoverpoint  = TbHoverPoint(T_local = [0,0,0.25])
    grippoint   = TbGripPoint(T_local = [0,0,0])
    anchorpoint = TbAnchorPoint(T_local = [0,0,0.12])
    dockpoint   = TbDockPoint(T_local = [0,0,0.15])
    grippers.append(TbGripper(hoverpoint, grippoint, anchorpoint, dockpoint, geometries=geometries, collidables = collidables))
    
#
wall = TbWall(holds=holds)
wall.add_geometry(TbCylinder(3.985, 3, 1, 100, T_local = TransformMatrix([0,1.25,-4,90,0,0])))
wall.add_collidable(TbCylinderCollidable(4.015, 3, 1, 50, T_local = TransformMatrix([0,1.25,-4,90,0,0])))

#
F = TbTetherForceSet(f_min=0, f_max=250)
#F = TbTetherForceSet(f_min=0, f_max=125)
#F = TbTetherGripperForceSet(f_min=0, f_max=150, gfm=PanorelGripperForceModel(250,300))
W = TbElliptoidWrenchSet(np.array([1,1,1,0.1,0.1,0.1])*1)
mapping = [[0,0],[0,1],[1,2],[1,3],[3,4],[3,5],[4,6],[4,7],[2,8],[2,9]]
aorder  = Ring([0,1,3,4,2])
tbot = TbTetherbot(platform=platform, grippers=grippers, tethers=tethers, wall=wall, W=W, F=F, mapping=mapping, aorder=aorder, l_min = 0.07)

pose = [0.4, 0.4, 0.05, C[6,4], 0, 90]
start = [8,2,14,0,12]

tbot.platform.T_local= TransformMatrix(pose)
tbot.place_all(start, correct_pose=True)
print(tbot.stability())

tbot.save(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'), overwrite=True)

vi = TetherbotVisualizer(tbot)
vi.run()

