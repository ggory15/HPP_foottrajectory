import numpy as np

class variable(object):
    def __init__(self, nvar):
        self.nvar = nvar;
        self.x_L = np.ones((nvar), dtype=np.float_) * -np.inf
        self.x_U = np.ones((nvar), dtype=np.float_) * np.inf  # no bound
