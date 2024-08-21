import os
from tbotlib import *
import msvcrt

file_prefix = os.path.dirname(__file__)

tbot: TbTetherbot = TbTetherbot.load(os.path.join(file_prefix, 'data/tetherbot.pkl'))
tbot.platform.T_world = TransformMatrix(np.hstack(((tbot.grippers[2].anchorpoint.T_world.r + tbot.grippers[3].anchorpoint.T_world.r)/2, (0,0,90)))).decompose()

vi = TetherbotVisualizer(tbot)
vi.set_background_color([1,1,1])
vi.save_camera_parameters(os.path.join(file_prefix, 'json//camera_parameters2.json'))
#vi.load_camera_parameters(os.path.join(file_prefix, 'data//camera_parameters2.json'))
vi.capture_screen_image(os.path.join(file_prefix, 'data//image.png'))

