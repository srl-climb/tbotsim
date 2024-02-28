from tbotlib import * 
from sys import exit
import numpy as np
import csv
import open3d as o3d
import cv2 as cv2
from time import sleep, time

### USER INPUTS ######################################################################
######################################################################################

grid     = circulargrid([1,1.5,2], [0,20,40,60], 2.15)
position = [1.4,0.45,2.18,0,0,110]
start    = [7,6,8,0,2]
goal     = [10,9,11,3,5]
fileprefix = 'data/data'

### PREPARATIONS #####################################################################
######################################################################################

# Simulation timestep
dt = 0.0167

# Smoothing algorithm
platformsmoother = BsplineSmoother(0.003, k=3)
armsmoother = BsplineSmoother(0.005, k=3)


# Profile generators
platformprofiler = SlicedProfile6(a_t = [0.05,10], 
                                  d_t = [0.05,10], 
                                  v_t = [0.05,1], 
                                  dt = dt, 
                                  smoother = platformsmoother)
""" platformprofiler = ProfileQPlatform(platformsmoother, dt, qlim=[[-0,2],[-0.01,0.01],[-0.2,0.2]], iter_max=500)  """

armprofiler = Profile3(a_t = 0.05, 
                       d_t = 0.05,
                       v_t = 0.1, 
                       dt = dt, 
                       smoother = armsmoother)
""" armprofiler = ProfileQArm(armsmoother, dt, qlim=[[[-3.141592,3.141592],[-0.872665,0.872665],[-0.872665,0.872665]],
                                                 [[0.314,1.414],[-0.01,0.01],[-0.1,0.1]],
                                                 [[0,0.3],[-0.01,0.01],[-0.1,0.1]]], iter_max=100) """

# Local path planner objects
iter = 50000

platform2configuration = PlanPlatform2Configuration(graph = TbPlatformPoseGraph(goal_dist = 0.025, 
                                                                                goal_skew = 1, 
                                                                                directions = [0.02,0.02,0,2,0,0], #
                                                                                iter_max = iter),
                                                    profiler = platformprofiler,
                                                    workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-90],
                                                                            mode_2d = True,
                                                                            scale = [0.1,0.1,0.1,45,45,10], 
                                                                            mode = 'max'))

platform2gripper = PlanPlatform2Gripper(graph = TbPlatformAlignGraph(goal_skew = 1, 
                                                                     directions = [0.01,0.01,0,1,0,0], 
                                                                     iter_max = iter),
                                        profiler = platformprofiler)

platform2hold = PlanPlatform2Hold(graph = TbPlatformAlignGraph(goal_skew = 1, 
                                                               directions = [0.01,0.01,0,1,0,0], 
                                                               iter_max = iter),
                                  profiler = platformprofiler)

arm2pose = PlanArm2Pose(graph = TbArmPoseGraph(goal_dist = 0.05,
                                               directions = [0.05,0.05,0.01], 
                                               iter_max = iter),
                        profiler = armprofiler)

localplanner = PlanPickAndPlace2(
                    platform2configuration = platform2configuration,
                    platform2gripper       = platform2gripper, 
                    platform2hold          = platform2hold, 
                    arm2pose               = arm2pose)


# Fast path planner objects
iter = 25

platform2gripper = PlanPlatform2Gripper(graph = TbPlatformAlignGraph(goal_skew = 1, 
                                                                     directions = [0.01,0.01,0,1,0,0], 
                                                                     iter_max = iter),
                                        profiler = platformprofiler)

platform2hold = PlanPlatform2Hold(graph = TbPlatformAlignGraph(goal_skew = 1, 
                                                               directions = [0.01,0.01,0,1,0,0], 
                                                               iter_max = iter),
                                  profiler = platformprofiler)

# Global path planner objects
globalplanner = GlobalPlanner(graph = TbGlobalGraph2(goal_dist = 0.01,
                                                     cost = 0.05, 
                                                     platform2hold = platform2hold,
                                                     platform2gripper = platform2gripper,
                                                     workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-90], #45
                                                                             mode_2d = True,
                                                                             scale = [0.1,0.1,0.1,45,45,10], #0.2, 45
                                                                             mode = 'first'),
                                                     iter_max = 5000),
                             localplanner = localplanner)

# Create assets

W       = hyperRectangle(np.array([5,5,0,0,0,0.5]), np.array([-5,-5,-0,-0,-0,-0.5]))
mapping = [[0,0],[0,1],[1,2],[1,3],[3,4],[3,5],[4,6],[4,7],[2,8],[2,9]]
aorder  = Ring([0,2,4,3,1]) #indices of the grippers clockwise (seen from above)
aorder  = Ring([0,1,3,4,2])
tethers  = [TbTether.example() for _ in range(10)]
grippers = [TbGripper.example() for _ in range(5)]
platform = TbPlatform.example()
platform.remove_geometries()
platform.add_geometry(TbCylinder(radius=0.05, height=0.02, T_local=[0,0,0.06]))
platform.add_geometry(TbBox([0.4,0.3,0.1], T_local = TransformMatrix([-0.2,-0.15,-0.05])))
holds    = TbHold.batch(grid.T, hoverpoint = [0,0,0.05],  grippoint = [0,0,0])

for hold in holds:
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
tbot.platform.arm.links[-1].qlim = [0,0.1]

tbot.platform.T_local= TransformMatrix(position)
tbot.place_all(start)


""" # uncomment here to create new json file
vi = TetherbotVisualizer(tbot)
vi._vi.get_render_option().background_color = [1, 1, 1]
vi._vi.get_view_control().convert_from_pinhole_camera_parameters(o3d.io.read_pinhole_camera_parameters('json//camera_parameters.json'))

while vi.opened:
    vi.update()
    parameters = vi._vi.get_view_control().convert_to_pinhole_camera_parameters()
    sleep(0.1)

#o3d.io.write_pinhole_camera_parameters('json//camera_parameters.json', parameters)
exit()
 """
""" print(tbot.stability())
exit() """

""" localplanner.plan(tbot, 4, 5, None)
exit() """

### ACTUAL PATH PLANNING #############################################################
######################################################################################
profiler = Profiler()
profiler.on()
tic()
_, commands, exitflag = globalplanner.plan(tbot, start, goal, CommandList()) #CommandList())
toc()
profiler.off()
profiler.print()

if exitflag == False:
    exit()


### VISUALIZATION ####################################################################
######################################################################################

tbot.platform.T_local= TransformMatrix(position)
tbot.place_all(start)
tbot.platform.arm.qs = tbot.platform.arm.q0s

vi = TetherbotVisualizer(tbot)
vi._vi.get_view_control().convert_from_pinhole_camera_parameters(o3d.io.read_pinhole_camera_parameters('json//camera_parameters.json'))
vi._vi.get_render_option().background_color = [1, 1, 1]

start = time()
while time()-start < 20:
    vi.update()
    sleep(0.02)

done = True
stabilities = []
anchorpoints = []
timestamps = []
i = 0
j = 0

while commands: # and vi.opened:
    vi.update()

    if len(commands)%13 == 0 and done:
        # every twelfth command, we want to save the anchor points of all tethers
        anchorpoints.append([])
        for tether in tbot.tethers:
            r1 = list(tether.anchorpoints[0].r_world)
            r2 = list(tether.anchorpoints[1].r_world)
            anchorpoints[-1].extend(r1 + r2) #first anchorpoint, second anchorpoint, command counter
        vi._vi.capture_screen_image(fileprefix + '_image_' + str(i) + '.png', do_render=True)
        i = i + 1
        timestamps.append([len(stabilities)])

    if done:
        if commands:
            command = commands.pop(0)
            command.print()
        else:
            break

    done = command.do(tetherbot=tbot)

    stabilities.append([tbot.stability()[0]])

# also save last state 
anchorpoints.append([])
for tether in tbot.tethers:
    r1 = list(tether.anchorpoints[0].r_world)
    r2 = list(tether.anchorpoints[1].r_world)
    anchorpoints[-1].extend(r1 + r2)    
vi._vi.capture_screen_image(fileprefix + '_image_' + str(i) + '.png', do_render=True)
stabilities.append([tbot.stability()[0]])
timestamps.append([len(stabilities)])

### SAVE DATA ########################################################################
######################################################################################

with open(fileprefix + '_stability.csv', 'w') as file:
    writer = csv.writer(file, delimiter=',')

    for stability in stabilities:
        writer.writerow(stability)

with open(fileprefix + '_anchorpoints.csv', 'w') as file:
    writer = csv.writer(file, delimiter=',')

    for anchorpoint in anchorpoints:
        writer.writerow(anchorpoint)

with open(fileprefix + '_timestamps.csv', 'w') as file:
    writer = csv.writer(file, delimiter=',')

    for timestamp in timestamps:
        writer.writerow(timestamp)
