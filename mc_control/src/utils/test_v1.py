import fr_utils as fr
import numpy as np
from mc_legs import *
from mc_robot import *

q = np.array([0,0,0])
T = fk_robot(q,'RL')
print(np.round(T,3))

# Z = 0.4424

q_0 = np.array([0., 0., 0.4424, 
                1., 0., 0., 0.,
                0,0,0, 0,0,0, 0,0,0, 0,0,0])

robot = mini_cheetah()
robot.step_update(q_0)

