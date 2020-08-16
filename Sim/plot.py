from skopt.space import Real, Integer
import skopt
from skopt import gp_minimize
from skopt.utils import use_named_args
from skopt import callbacks
from skopt.callbacks import CheckpointSaver
from skopt import load


checkpoint_saver = CheckpointSaver("C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\check\\checkpoint.pkl",)
