import os
from tbotlib import *
import csv
from time import sleep

file_prefix = os.path.dirname(__file__)

tbot: TbTetherbot = TbTetherbot.load(os.path.join(file_prefix, 'data/tetherbot.pkl'))
tbot.wall.geometries[0].geometry.paint_uniform_color([1,1,1])
commands = CommandList.load(os.path.join(file_prefix, 'data/commands.pkl'))

vi = TetherbotVisualizer(tbot)
vi.load_camera_parameters(os.path.join(file_prefix, 'json//camera_parameters.json'))
vi.set_background_color([1,1,1])
#vi.save_camera_parameters(os.path.join(file_prefix, 'json//camera_parameters.json'))

feasibility = FeasibilityContainer()
#feasibility.add(TbJointLimitFeasibility()) #
#feasibility.add(TbTetherArmCollisionFeasibility(distance = 0.02))
#feasibility.add(TbWallPlatformCollisionFeasibility(distance = -0.01))
feasibility.add(TbGripperPlatformCollisionFeasibility(distance = -0.01))
#feasibility.add(TbWrenchFeasibility(threshold = 0.00))

done = True
counter0 = 0 # number of iterations
counter1 = 0 # 
counter2 = 0
step = 10
stabilities = []
timestamps = []
dt = 0.0167
sleep(10)
while commands and vi.opened:
    
    vi.update()

    if len(commands)%13 == 0 and done:
        vi.capture_screen_image(os.path.join(file_prefix, 'data//sim2_data_image_' + str(counter1) + '.png'))
        timestamps.append([counter0*step*dt])
        counter1 += 1
        
    stabilities.append(tbot.stability())
    counter0 += 1

    if counter0>=2:
        if stabilities[-1][0] < 0 and stabilities[-2][0] > 0:
            print(stabilities[-1][0])
            vi.capture_screen_image(os.path.join(file_prefix, 'data//singularities//' + str(counter2) + '.png'))
            counter2 += 1

    if done:
        if commands:
            command = commands.pop(0)
            command.print()
            
        else:
            break
    
    if not feasibility.eval(tbot):
        print('failed')
        exit()

    done = command.do(tetherbot = tbot, step = step)

""" vi.capture_screen_image(os.path.join(file_prefix, 'data//sim2_data_image_' + str(counter1) + '.png'))
timestamps.append([len(stabilities)*step*dt])

with open(os.path.join(file_prefix, 'data//sim2_data_stability.csv'), 'w') as file:
    writer = csv.writer(file, delimiter=',')

    for stability in stabilities:
        writer.writerow(stability)

with open(os.path.join(file_prefix, 'data//sim2_data_timestamps.csv'), 'w') as file:
    writer = csv.writer(file, delimiter=',')

    for timestamp in timestamps:
        writer.writerow(timestamp) """