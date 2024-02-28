import tbotlib as tb
import os

file = os.path.join(os.path.dirname(__file__), 'pickle', 'tetherbot.pkl')

tbot: tb.TbTetherbot = tb.TbTetherbot.load(file)

print(tbot.platform.arm.T_local.T)

for link in tbot.platform.arm.links:
    print(link.name)
    print(link.dhp._modified)
    print(link.dhp.a)
    print(link.dhp.alpha)
    print(link.dhp.d)
    print(link.dhp.phi)
    print(link.dhp.T)
    print(link.T_local.T)