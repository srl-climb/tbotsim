import os
from tbotlib import *
import csv

file_prefix = os.path.dirname(__file__)

tbot: TbTetherbot = TbTetherbot.load(os.path.join(file_prefix, 'data/tetherbot.pkl'))
commands = CommandList.load(os.path.join(file_prefix, 'data/commands.pkl'))

#tbot.place_all([22, 23, 21, 11 , 8])

vi = TetherbotVisualizer(tbot)
vi.load_camera_parameters(os.path.join(file_prefix, 'json//camera_parameters.json'))
vi.set_background_color([1,1,1])
#vi.save_camera_parameters(os.path.join(file_prefix, 'json//camera_parameters.json'))
#exit()

done = True
counter = 0
step = 50
stabilities = []
timestamps = []
while commands and vi.opened:
    
    vi.update()

    if len(commands)%13 == 0 and done:
        vi.capture_screen_image(os.path.join(file_prefix, 'data//sim2_data_image_' + str(counter) + '.png'))
        timestamps.append([len(stabilities)*50])
        counter += 1
        
    stabilities.append(tbot.stability())
    
    if done:
        if commands:
            command = commands.pop(0)
            command.print()
            
        else:
            break

    done = command.do(tetherbot = tbot, step = step)

vi.capture_screen_image(os.path.join(file_prefix, 'data//sim2_data_image_' + str(counter) + '.png'))
timestamps.append([len(stabilities)])

with open(os.path.join(file_prefix, 'data//sim2_data_stability.csv'), 'w') as file:
    writer = csv.writer(file, delimiter=',')

    for stability in stabilities:
        writer.writerow(stability)

with open(os.path.join(file_prefix, 'data//sim2_data_timestamps.csv'), 'w') as file:
    writer = csv.writer(file, delimiter=',')

    for timestamp in timestamps:
        writer.writerow(timestamp)