from tbotlib import * 
from sys import exit
import numpy as np
import os
import csv

pathprefix = 'D:/T7 Back-Up/2023_10_18_Tethered Climbing Robot Test 6/'
csv_path = pathprefix + 'Rosbag/20231018-141144/csv/'
tbot_path = pathprefix + 'Description/tetherbot/tetherbot_light.pkl'
result_path = pathprefix + 'Other/platform0__target_stability.csv'
step = 50
start = 0
stop = float('inf')


def read_columns_between_stamps(file: str, start: float = 0, stop: float = float('inf'), timestamp_name: str = 'message_timestamp', format: str = 'float'):

    with open(file, 'r') as stream:
        reader = csv.DictReader(stream)
        columns: dict[str, list] = {name: [] for name in reader.fieldnames}
        for row in reader: 
            timestamp = float(row[timestamp_name])
            if timestamp >= start and timestamp <= stop:
                for (k,v) in row.items(): 
                    if format == 'float':
                        v = float(v)
                    columns[k].append(v)
            elif timestamp > stop:
                break
        stamps = np.array(columns[timestamp_name], dtype=float) - start

    return columns, stamps


tbot: TbTetherbot = TbTetherbot.load(tbot_path)
platform_pose_data, timestamps = read_columns_between_stamps(csv_path + 'platform0__platform_controller__target_pose.csv', start, stop)

platform_pose_data['x'] = np.array(platform_pose_data['x'])[::step]
platform_pose_data['y'] = np.array(platform_pose_data['y'])[::step]
platform_pose_data['z'] = np.array(platform_pose_data['z'])[::step]
platform_pose_data['theta_x'] = np.array(platform_pose_data['theta_x'])[::step]
platform_pose_data['theta_y'] = np.array(platform_pose_data['theta_y'])[::step]
platform_pose_data['theta_z'] = np.array(platform_pose_data['theta_z'])[::step]
timestamps = timestamps[::step]

gripper_hold_names = []
for i in range(5):
    d, t = read_columns_between_stamps(csv_path + 'gripper' + str(i) + '__gripper_state_publisher__hold_name.csv', start, stop+10, 'rosbag_timestamp', '')
    j = np.searchsorted(t, timestamps, side='left')
    gripper_hold_names.append(np.array(d['data'])[j])
gripper_hold_names = np.array(gripper_hold_names).T

tether_tension = []
d, t = read_columns_between_stamps(csv_path + 'platform0__platform_controller__tether_tension.csv', start, stop+10, 'rosbag_timestamp')
j = np.searchsorted(t, timestamps, side='left')
for i in range(10):
        tether_tension.append(d['data_' + str(i)])
tether_tension = np.array(tether_tension).T[j]

hold_names = [hold.name for hold in tbot.wall.holds]
stability = []

for x, y, z, theta_x, theta_y, theta_z, tension, gripper_hold_names, timestamp in zip(platform_pose_data['x'], platform_pose_data['y'], platform_pose_data['z'], platform_pose_data['theta_x'], platform_pose_data['theta_y'], platform_pose_data['theta_z'], tether_tension, gripper_hold_names, timestamps):
    tbot.platform.T_world = TransformMatrix((x, y, z, theta_x, theta_y, theta_z))
    tbot._tensioned = tension.astype(bool)
    
    for name, grip_idx in zip(gripper_hold_names, range(5)):
        hold_idx = hold_names.index(name)
        tbot.place(grip_idx, hold_idx, correct_pose=True)
        tbot._update_transforms()
    stability.append((timestamp, tbot.stability()[0]))

with open(result_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(stability)

print(stability)