import fr_utils as fr 
import numpy as np
from mc_legs import *


q = np.array([np.pi/3 ,0,0])

T = fk_RL(q,joint=3,group=True)
print(np.round(T,5))