import numpy as np

class cost(object):
    def __init__(self, nBoxes, initPos, nvar):
        self.nBoxes = nBoxes;
        self.initPos = initPos
        self.nvar = nvar;

    def eval_f(self, x, user_data= None):
        self.out = 0.0;
        pos = np.squeeze(np.asarray(self.initPos))
        posNext = x[0:3]
        dist = posNext - pos
        out = dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2]

        for i in range(0, self.nBoxes-1):
            pos = x[3*i: 3*i+3]
            posNext = x[3*(i+1): 3*i+6]
            dist = posNext - pos
            out += dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2]

        return out

    def eval_grad_f(self, x, user_data =None):
        grad_f = np.array(np.zeros(self.nvar), np.float_)
        grad_f[0:3] = 2.0 * (x[0:3] - np.squeeze(np.asarray(self.initPos)))

        for i in range(0, self.nBoxes-1):
            pos = x[3*i: 3*i+3]
            posNext = x[3*(i+1): 3*i+6]
            dist = posNext - pos
            grad_f[3 * i:3*i+3] -= 2.0 * dist
            grad_f[3 * (i+1) : 3 * i + 6] += 2.0 * dist

        return grad_f;
