import tbotlib as tb
import numpy as np


# ==== CREATE ROBOT ====

W       = tb.hyperRectangle(np.array([5,5,0,0,0,0.5]), np.array([-5,-5,-0,-0,-0,-0.5]))
mapping = [[0,0],[0,1],[1,2],[1,3],[3,4],[3,5],[4,6],[4,7],[2,8],[2,9]]
aorder  = tb.Ring([0,1,3,4,2])
tethers  = [tb.TbTether.example() for _ in range(10)]
grippers = [tb.TbGripper.example() for _ in range(5)]
platform = tb.TbPlatform.example()
holds    = tb.TbHold.batch(tb.circulargrid([1,1.5,2], [0,20,40,60], 2.15).T, hoverpoint = [0,0,0.05],  grippoint = [0,0,0])

for hold in holds:
    hold.remove_geometries()
    hold.add_geometry(tb.TbCylinder(radius = 0.05, height = 0.03, T_local = tb.TransformMatrix([0,0,-0.05])))

for gripper in grippers:
    gripper.remove_geometries()
    gripper.add_geometry(tb.TbCylinder(T_local = [0,0,-0.0025], radius = 0.015, height = 0.065))
    gripper.add_geometry(tb.TbSphere(T_local = [0,0,0.03], radius=0.02))

for tether in tethers:
    tether.f_max = 200
    tether.add_geometry(tb.TbTethergeometry(radius = 0.008))

wall = tb.TbWall(holds=holds)
tbot = tb.TbTetherbot(platform=platform, grippers=grippers, tethers=tethers, wall=wall, W=W, mapping=mapping, aorder=aorder)
tbot.l_min = 0.1


tbot.platform.T_local= tb.TransformMatrix([1.4,0.45,2.18,0,0,110])
tbot.place_all([7,6,8,0,2])

#  just show me the robot!
vi = tb.TetherbotVisualizer(tbot)
vi.run()
exit()



# ==== PREPARATIONS ====

# position robot at origin, raise z a little
tbot.platform.T_local= tb.TransformMatrix([0,0,0.03,0,0,0])

# make gripper children of platform for relative positioning
for gripper in tbot.grippers:
    gripper.parent = tbot.platform
    
# remove wall
tbot._remove_child(tbot.wall)


# ==== ML-FUNCTION ====

def function_to_be_replaced_by_ml(tbot: tb.TbTetherbot, positions: np.ndarray) -> bool:
    
    # NOTE: tbot is only necessary for the calculation of stability(), the actual ml function will only receive positions

    for position, gripper in zip(positions, tbot.grippers):
        gripper.T_local = tb.TransformMatrix(position)
    
    return tbot.stability()[1] 


# ==== DEMO ====

# create relative position limits for each gripper
start = np.array([[0.30,-0.25,0],[0.30,0.30,0],[0.30,-0.30,0],[-0.30,0.30,0],[-0.30,-0.30,0]])
stop = np.array([ [0.50, 0.25,0],[0.50,0.50,0],[0.50,-0.50,0],[-0.50,0.50,0],[-0.50,-0.50,0]])
step = np.array([ [0.10, 0.25,1],[0.10,0.10,1],[0.10,-0.10,1],[-0.10,0.10,1],[-0.10,-0.10,1]])


i = 0
profiler = tb.Profiler()
profiler.on()
tbot.remove_all_geometries()
for positions in tb.ndrange(start, stop, step):
    
    i = i + 1
    result = function_to_be_replaced_by_ml(tbot, positions)

    if i == 10000:
        break

profiler.off()
profiler.print()