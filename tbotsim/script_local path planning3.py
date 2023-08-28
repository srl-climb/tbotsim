from tbotlib import yaml2planner, TbTetherbot, TetherbotVisualizer, CommandList, hyperRectangle, tic, toc
import numpy as np

tbot_desc_file = 'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/pickle/tetherbot.pkl'
tbot_light_desc_file = 'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/pickle/tetherbot_light.pkl'
planner_config_file = 'C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/planner.yaml'

simulation_dt, platform2pose, platform2configuration, arm2pose, local_planner, global_planner  = yaml2planner(planner_config_file)
tbot: TbTetherbot = TbTetherbot.load(tbot_desc_file)
tbot_light : TbTetherbot = TbTetherbot.load(tbot_light_desc_file)
tbot_light.W = hyperRectangle(np.array([3,3,0,0,0,0.3]), np.array([-3,-3,-0,-0,-0,-0.3]))
tbot.debug_print()
tbot_light.debug_print()

commands = CommandList()
_, commands, exitflag = local_planner.plan(tbot_light, grip_idx = 0, hold_idx = 5, commands = commands)
if exitflag == False: exit()
_, commands, exitflag = local_planner.plan(tbot_light, grip_idx = 1, hold_idx =  6, commands = commands)
if exitflag == False: exit()
_, commands, exitflag = local_planner.plan(tbot_light, grip_idx = 2, hold_idx =  7, commands = commands)
if exitflag == False: exit()
_, commands, exitflag = local_planner.plan(tbot_light, grip_idx = 4, hold_idx =  9, commands = commands)
if exitflag == False: exit()
_, commands, exitflag = local_planner.plan(tbot_light, grip_idx = 3, hold_idx =  8, commands = commands)
if exitflag == False: exit()

print('Path found, saving commands')
commands.save('C:/Users/ngkla/Desktop/Git/tbotsim/tbotsim/commands.yaml', overwrite=True)