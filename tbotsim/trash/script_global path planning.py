from tbotlib import *  
from sys import exit
import numpy as np

### USER INPUTS ######################################################################
######################################################################################

start = [0,1,2,3,4]
goal  = [5,6,7,8,9]

### PREPARATIONS #####################################################################
######################################################################################

# Simulation timestep
dt = 0.0167

# Smoothing algorithm
smoother = BsplineSmoother(0.001)

# Profile generators
platformprofiler = SlicedProfile6(a_t = [0.05,1], 
                                  d_t = [0.05,1], 
                                  v_t = [0.75,0.5], 
                                  dt = dt, 
                                  smoother = smoother)

armprofiler = Profile3(a_t = 0.05, 
                       d_t = 0.05,
                       v_t = 0.75, 
                       dt = dt, 
                       smoother = smoother)

# Local path planner objects
iter = 50000

platform2configuration = PlanPlatform2Configuration(graph = TbPlatformPoseGraph(goal_dist = 0.025, 
                                                                                goal_skew = 1, 
                                                                                directions = [0.02,0.02,0.02,1,0,0],
                                                                                iter_max = iter),
                                                    profiler = platformprofiler,
                                                    workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-135], 
                                                                            scale = [0.1,0.1,0.1,45,45,45], 
                                                                            mode = 'max'))

platform2gripper = PlanPlatform2Gripper(graph = TbPlatformAlignGraph(goal_skew = 1, 
                                                                     directions = [0.01,0.01,0.01,1,0,0], 
                                                                     iter_max = iter),
                                        profiler = platformprofiler)

platform2hold = PlanPlatform2Hold(graph = TbPlatformAlignGraph(goal_skew = 1, 
                                                               directions = [0.01,0.01,0.01,1,0,0], 
                                                               iter_max = iter),
                                  profiler = platformprofiler)

arm2pose = PlanArm2Pose(graph = TbArmPoseGraph(goal_dist = 0.05,
                                               directions = [0.025,0.025,0.025], 
                                               iter_max = iter),
                        profiler = armprofiler)

localplanner = PlanPickAndPlace(
                    platform2configuration = platform2configuration,
                    platform2gripper       = platform2gripper, 
                    platform2hold          = platform2hold, 
                    arm2pose               = arm2pose)


# Fast path planner objects
iter = 50

platform2configuration = FastPlanPlatform2Configuration(workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-135], 
                                                                                scale = [0.1,0.1,0.1,45,45,45], 
                                                                                mode = 'first')) #max

platform2gripper = PlanPlatform2Gripper(graph = TbPlatformAlignGraph(goal_skew = 1, 
                                                                     directions = [0.01,0.01,0.0,1,0,0], 
                                                                     iter_max = iter),
                                        profiler = platformprofiler)

platform2hold = PlanPlatform2Hold(graph = TbPlatformAlignGraph(goal_skew = 1, 
                                                               directions = [0.01,0.01,0.0,1,0,0], 
                                                               iter_max = iter),
                                  profiler = platformprofiler)

fastplanner = FastPlanPickAndPlace(platform2configuration = platform2configuration,
                                   platform2gripper = platform2gripper,
                                   platform2hold = platform2hold)

# Global path planner objects
globalplanner = GlobalPlanner(graph = TbGlobalGraph(goal_dist = 0.001, 
                                                    planner = fastplanner, 
                                                    workspace = TbWorkspace(padding = [-0.1,-0.1,0,-180,-180,-135],
                                                                            scale = [0.3,0.3,0.1,45,45,45], 
                                                                            mode = 'first'),
                                                    iter_max = 500),
                             localplanner = localplanner)

# Load assets
tbot = TbTetherbot.example()


### ACTUAL PATH PLANNING #############################################################
######################################################################################

tic()
#profiler = Profiler()
#profiler.on()
_, commands, exitflag = globalplanner.plan(tbot, start, goal, CommandList()) #CommandList())
toc()
print(exitflag)
#profiler.off()
#profiler.print()

if exitflag == False:
    exit()


### VISUALIZATION #######################################################################
######################################################################################

#tbtetherbot = simscene.tetherbot.toTbTetherbot()
tbtetherbot = TbTetherbot.example()
vi = TetherbotVisualizer(tbtetherbot)

while commands and vi.opened:
    vi.update()
    commands.pop(0).do(tbtetherbot)


### SIMULATION #######################################################################
######################################################################################

# Set up simulation
""" config      = serializer.loadConfig('2022_11_21\\config.vxc')
application = SimApplication(config)

application.add(simscene.scene)
application.start()

# Run simulation
while application.running and commands:

    application.run()

    commands.pop(0).do(simscene.tetherbot) """