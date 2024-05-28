from tbotlib import *
import numpy as np
import os

grid = cylindricalgrid([0,0.5,1,1.5,2,2.5], [0,0.5,1,1.5,2,2.5], 4)
grid = np.column_stack((grid[0:3,:].T, np.zeros((grid.shape[1],1)), grid[3,:], np.zeros((grid.shape[1],1))))
pose = [0.5,0.5,0.05,grid[6,4],0,90]
start    = [8,2,14,0,12]
""" pose = [2,2,-0.5,0,grid[27,4],0]
print(pose)
start    = [34, 35, 33, 23, 21] """

print(grid[6,4])

# Create assets
W       = hyperRectangle(np.array([1,1,1,0.1,0.1,0.1])*0.5, np.array([-1,-1,-1,-0.1,-0.1,-0.1])*0.5)
mapping = [[0,0],[0,1],[1,2],[1,3],[3,4],[3,5],[4,6],[4,7],[2,8],[2,9]]
aorder  = Ring([0,2,4,3,1]) #indices of the grippers clockwise (seen from above)
aorder  = Ring([0,1,3,4,2])
tethers  = [TbTether.example() for _ in range(10)]
platform = TbPlatform.example()
platform.remove_geometries()
platform.add_geometry(TbCylinder(radius=0.05, height=0.07, T_local=[0,0,0.085]))
platform.arm.T_local.translate([0,0,0.05])
platform.arm._workspace_radius = 0.9

platform.add_geometry(TbBox([0.4,0.3,0.1], T_local = TransformMatrix([-0.2,-0.15,-0.05])))
platform.add_collidable(TbBoxCollidable([0.5,0.4,0.11], T_local = TransformMatrix([-0.25,-0.2,-0.055])))
holds    = TbHold.batch(grid, hoverpoint = [0,0,0.1],  grippoint = [0,0,0.05])

for hold, g in zip(holds, grid):
    hold.remove_geometries()
    hold.add_geometry(TbCylinder(radius = 0.05, height = 0.03, T_local = TransformMatrix([0,0,0*-0.05])))

grippers = []
for _ in range(5):
    geometries = [TbCylinder(T_local = [0,0,-0.0015], radius = 0.015, height = 0.075), TbSphere(T_local = [0,0,0.05], radius=0.02)]
    hoverpoint  = TbHoverPoint(T_local = [0,0,0.12])
    grippoint   = TbGripPoint(T_local = [0,0,0])
    anchorpoint = TbAnchorPoint(T_local = [0,0,0.05])
    dockpoint   = TbDockPoint(T_local = [0,0,0.07])
    grippers.append(TbGripper(hoverpoint, grippoint, anchorpoint, dockpoint, geometries=geometries))
    
for tether in tethers:
    tether.f_max = 200
    tether.add_geometry(TbTethergeometry(radius = 0.008))

wall = TbWall(holds=holds)
wall.add_geometry(TbCylinder(3.985, 3, 1, 100, T_local = TransformMatrix([0,1.25,-4,90,0,0])))
wall.add_collidable(TbCylinderCollidable(4, 3, 1, 20, T_local = TransformMatrix([0,1.25,-4,90,0,0])))
tbot = TbTetherbot(platform=platform, grippers=grippers, tethers=tethers, wall=wall, W=W, mapping=mapping, aorder=aorder)
tbot._tether_collision_margin = 0.05
tbot._l_min = 0.05
tbot.platform.arm.links[-1].qlim = [0,0.2]
tbot.platform.arm.links[-1].q = -0.1
tbot.platform.arm.qs = tbot.platform.arm.qlims[:,0]
tbot.platform.T_local= TransformMatrix(pose)
tbot.place_all(start, correct_pose=True)

print(tbot.stability())
tbot.save(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'), overwrite=True)


""" tbot.platform.T_local = TransformMatrix([0.55,0.5,-0.0,grid[12,4],0,90])
qs =tbot.platform.arm.ivk(tbot.grippers[2].hoverpoint.T_world)
print(tbot.platform.arm.valid(qs))
tbot.platform.arm.qs = qs
 """

""" vi = TetherbotVisualizer(tbot)
vi.run() """

