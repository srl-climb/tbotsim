from tbotlib import yaml2planner, TbTetherbot, TetherbotVisualizer, CommandList, hyperRectangle, tic, toc, TransformMatrix
import numpy as np

tbot_desc_file = '/home/climb/ros2_ws/libs/tbotsim/tbotsim/pickle/tetherbot.pkl' #'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/pickle/tetherbot.pkl'
tbot_light_desc_file = '/home/climb/ros2_ws/libs/tbotsim/tbotsim/pickle/tetherbot_light.pkl' #'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/pickle/tetherbot_light.pkl'
planner_config_file = '/home/climb/ros2_ws/libs/tbotsim/tbotsim/planner.yaml' #'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/planner.yaml'

simulation_dt, platform2pose, platform2configuration, arm2pose, local_planner, global_planner  = yaml2planner(planner_config_file)
tbot: TbTetherbot = TbTetherbot.load(tbot_desc_file)
tbot_light : TbTetherbot = TbTetherbot.load(tbot_light_desc_file)

#tbot.debug_print()
#tbot_light.debug_print()

T = TransformMatrix()
T.r = [0.47, 0.652, 0.068]
T.q = [0.735, -0.001, -0.001, 0.678]
tbot_light.platform.T_world = T
tbot.platform.T_world = T

print(tbot_light.stability())

commands = CommandList()

_, commands, exitflag = local_planner.plan(tbot_light, grip_idx = 3, hold_idx =  8, commands = commands)
if exitflag == False: exit()
_, commands, exitflag = local_planner.plan(tbot_light, grip_idx = 0, hold_idx =  5, commands = commands)
if exitflag == False: exit()
_, commands, exitflag = local_planner.plan(tbot_light, grip_idx = 2, hold_idx = 7, commands = commands)
if exitflag == False: exit()
_, commands, exitflag = local_planner.plan(tbot_light, grip_idx = 4, hold_idx =  9, commands = commands)
if exitflag == False: exit()
_, commands, exitflag = local_planner.plan(tbot_light, grip_idx = 1, hold_idx =  6, commands = commands)
if exitflag == False: exit()

""" _, commands, exitflag = global_planner.plan(tbot_light, [0,1,2,3,4],[5,6,7,8,9], commands = commands)
if exitflag == False: exit() """

vi = TetherbotVisualizer(tbot)
done = True
while vi.opened:
    vi.update()
    if done:
        if commands:
            command = commands.pop(0)
        else:
            break
        command.print()
    done = command.do(tetherbot=tbot, step = 200)
    print(tbot.l)
vi.run()

""" print('Path found, saving commands')
commands.save('C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/commands2.yaml', overwrite=True) """