import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot
mat = "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\EvenDistribution\\100_41_0.6121118946772709_139.npy"
dredged = np.load(mat)
plt.imshow(dredged)
plt.colorbar()
plt.title("Dredged Locations")
plt.show()