import numpy as np

class constraints(object):
    def __init__(self):
        self.index = 0;
        self.g_L = []
        self.g_U = []
        self.nnzj = 0;
        self.ncon = 0;
        self.ncon_prev = [];
        self.g = [];
        self.jac_g = [];
        self.constraint = []

    def pushback_constraint(self, constraint):
        self.ncon_prev.append(self.ncon)
        self.constraint.append(constraint)
        self.g_L.extend(constraint.g_L)
        self.g_U.extend(constraint.g_U)
        self.nnzj += constraint.nnzj
        self.ncon += constraint.ncon

    def getG_L(self):
        #print("g_L", np.squeeze(self.g_L))
        gL = np.squeeze(np.reshape(self.g_L, (1, np.size(self.g_L))))
        #print("newgl", gL)
        return gL

    def getG_U(self):
        gU = np.squeeze(np.reshape(self.g_U, (1, np.size(self.g_U))))
        return gU

    def eval_g(self, x, user_data=None):
        self.g = []
        for i in range(0, len(self.constraint)):
            self.g.extend(self.constraint[i].eval_g(x))
        g = np.squeeze(np.reshape(self.g, (1, self.ncon)))
        return g

    def eval_jac_g(self, x, flag, user_data=None):
        self.jac_g = []
        jac_g1 = []
        jac_g2 = []
        jac_g = []

        if flag :
            for i in range(0, len(self.constraint)):
                self.jac_g.append(self.constraint[i].eval_jac_g(x, flag))

            for i in range(0, len(self.constraint)):
                jac_g1.extend(self.jac_g[i][0] + np.array(np.ones(self.constraint[i].nnzj) * self.ncon_prev[i]))
                jac_g2.extend(self.jac_g[i][1])

            jac_g1 = np.squeeze(np.reshape(jac_g1, (1, self.nnzj)))
            jac_g2 = np.squeeze(np.reshape(jac_g2, (1, self.nnzj)))

            return (np.array(jac_g1, np.int), np.array(jac_g2, np.int))
        else :
            for i in range(0, len(self.constraint)):
                self.jac_g.extend(self.constraint[i].eval_jac_g(x, flag))

            return  np.squeeze(np.reshape(self.jac_g, (1, self.nnzj)))