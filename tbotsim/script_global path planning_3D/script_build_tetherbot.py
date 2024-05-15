from tbotlib import *
import numpy as np
import os

grid = cylindricalgrid([0,0.5,1,1.5,2,2.5], [0,0.5,1,1.5,2,2.5], 4)
grid = np.column_stack((grid[0:3,:].T, np.zeros((grid.shape[1],1)), grid[3,:], np.zeros((grid.shape[1],1))))
pose = [0.5,0.5,0.0,grid[6,4],0,90]
start    = [8,2,14,0,12]
#start    = [9,3,15,1,13]

print(grid[6,4])

# Create assets
W       = hyperRectangle(np.array([1,1,1,0.1,0.1,0.1])*0.5, np.array([-1,-1,-1,-0.1,-0.1,-0.1])*0.5)
mapping = [[0,0],[0,1],[1,2],[1,3],[3,4],[3,5],[4,6],[4,7],[2,8],[2,9]]
aorder  = Ring([0,2,4,3,1]) #indices of the grippers clockwise (seen from above)
aorder  = Ring([0,1,3,4,2])
tethers  = [TbTether.example() for _ in range(10)]
grippers = [TbGripper.example() for _ in range(5)]
platform = TbPlatform.example()
platform.remove_geometries()
platform.add_geometry(TbCylinder(radius=0.05, height=0.02, T_local=[0,0,0.06]))

platform.add_geometry(TbBox([0.4,0.3,0.1], T_local = TransformMatrix([-0.2,-0.15,-0.05])))
holds    = TbHold.batch(grid, hoverpoint = [0,0,0.05],  grippoint = [0,0,0])

for hold, g in zip(holds, grid):
    hold.remove_geometries()
    hold.add_geometry(TbCylinder(radius = 0.05, height = 0.03, T_local = TransformMatrix([0,0,-0.05])))

for gripper in grippers:
    gripper.remove_geometries()
    gripper.add_geometry(TbCylinder(T_local = [0,0,-0.0025], radius = 0.015, height = 0.065))
    gripper.add_geometry(TbSphere(T_local = [0,0,0.03], radius=0.02))

for tether in tethers:
    tether.f_max = 200
    tether.add_geometry(TbTethergeometry(radius = 0.008))

wall = TbWall(holds=holds)
tbot = TbTetherbot(platform=platform, grippers=grippers, tethers=tethers, wall=wall, W=W, mapping=mapping, aorder=aorder)
tbot._tether_collision_margin = 0.05
tbot._l_min = 0.1
tbot.platform.arm.links[-1].qlim = [-0.1,0.1]
tbot.platform.arm.links[-1].q = -0.1
tbot.platform.T_local= TransformMatrix(pose)
tbot.place_all(start)
print(tbot.stability())

tbot.save(os.path.join(os.path.dirname(__file__), 'data/tetherbot.pkl'), overwrite=True)

tbot.tension(1, False)
tbot.platform.T_local = TransformMatrix([0.55,0.5,-0.0,0,0,90])
print(tbot.stability())
qs =tbot.platform.arm.ivk(tbot.grippers[1].hoverpoint.T_world)
print(tbot.platform.arm.valid(qs))
print(qs)
tbot.platform.arm.qs = qs

qs =tbot.platform.arm.ivk(tbot.grippers[1].dockpoint.T_world)
print(tbot.platform.arm.valid(qs))
print(qs)
tbot.platform.arm.qs = qs

#vi = TetherbotVisualizer(tbot)
#vi.run()

print(tbot.filter_holds(2))