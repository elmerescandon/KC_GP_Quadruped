import fr_utils as fr 
import numpy as np
from mc_legs import *


q = np.array([0,0,0])
T = fk_FR(q)
print(np.round(T,2))
