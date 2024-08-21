from tbotlib import CommandList, TetherbotVisualizer, TbTetherbot
import csv

commands_path = 'D:/T7 Back-Up/2023_10_18_Tethered Climbing Robot Test 6/Commands/commands0.pkl'
tbot_path = 'D:/T7 Back-Up/2023_10_18_Tethered Climbing Robot Test 6/Description/tetherbot/tetherbot.pkl'
csv_path = 'D:/T7 Back-Up/2023_10_18_Tethered Climbing Robot Test 6/Other/platform0__target_stability.csv'

commands = CommandList.load(commands_path)
tbot: TbTetherbot = TbTetherbot.load(tbot_path)

vi = TetherbotVisualizer(tbot)
done = True

stabilities = []
step = 50
dt = 50 # Frequency of command data here: 50Hz
counter = 0

while vi.opened:
    vi.update()
    if done:
        if commands:
            command = commands.pop(0)
        else:
            break
        command.print()
    done = command.do(tetherbot=tbot, step = 50)
    stabilities.append((step/dt*counter, tbot.stability()[0]))

    counter += 1

with open(csv_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(stabilities)

print(stabilities)

for command in commands: 
    command.print()