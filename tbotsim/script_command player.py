from tbotlib import CommandList, TetherbotVisualizer, TbTetherbot

commands_path = '/home/climb/ros2_ws/commands/commands0.pkl'
tbot_path = '/home/climb/ros2_ws/src/tbotros_description/tbotros_description/desc/tetherbot/tetherbot.pkl'

commands = CommandList.load(commands_path)
tbot: TbTetherbot = TbTetherbot.load(tbot_path)

print(tbot.W)
vi = TetherbotVisualizer(tbot)
done = True
while commands and vi.opened:
    vi.update()
    if done:
        command = commands.pop(0)
        command.print()
    done = command.do(tetherbot=tbot, step = 200)
    print(tbot.l)
vi.run()

for command in commands: 
    command.print()