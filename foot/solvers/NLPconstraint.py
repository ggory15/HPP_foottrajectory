import numpy as np

class constraint(object):
    def __init__(self, index, BoxesHullTrajProble):
        self.index = index;
        self.NLP_framework = BoxesHullTrajProble
        self.ncon = self.getConstNum()

        if self.index < self.NLP_framework.nFixedPlanCstr:
            self.plane = self.NLP_framework.boxAboveFixedPlanFcts[self.index].Plane;
            self.box = self.NLP_framework.boxAboveFixedPlanFcts[self.index].Box;
        elif self.index == self.NLP_framework.nFixedPlanCstr:
            self.box = self.NLP_framework.fixedFinalBox;
            self.plane = self.NLP_framework.boxAboveFixedPlanFcts[self.index-1].Plane;
        elif self.index < self.NLP_framework.nCstr - self.NLP_framework.nMobilePlanCstr:
            self.iBox0Above = self.NLP_framework.plan[self.index - self.NLP_framework.config.nBoxes - 1].box0Above
            self.iBox1Above = self.NLP_framework.plan[self.index - self.NLP_framework.config.nBoxes - 1].box1Above
            self.iBoxBelow = self.NLP_framework.plan[self.index - self.NLP_framework.config.nBoxes - 1].boxBelow


        self.g_L = self.getLB()
        self.g_U = self.getUB()
        self.nnzj = 0;
        if self.index < self.NLP_framework.nFixedPlanCstr:
            self.nnzj = self.ncon
        elif self.index == self.NLP_framework.nFixedPlanCstr:
            self.nnzj = self.ncon
        elif self.index < self.NLP_framework.nCstr - self.NLP_framework.nMobilePlanCstr:
            if (self.iBox0Above != -1 and self.iBox1Above != self.NLP_framework.config.nBoxes):
                self.nnzj = 144
            else:
                self.nnzj = 120
        else:
            self.nnzj = self.ncon * 3

    def getConstNum(self):
        assert self.index < self.NLP_framework.nCstr
        if self.index < self.NLP_framework.nFixedPlanCstr:
            return 8;
        elif self.index == self.NLP_framework.nFixedPlanCstr:
            return 3;
        elif self.index < self.NLP_framework.nCstr - self.NLP_framework.nMobilePlanCstr:
            return 24;
        else:
            return 1;

    def getLB(self):
        g_L = np.array(np.zeros(self.ncon), np.float_)
        if self.index < self.NLP_framework.nFixedPlanCstr:
            for i in range(0, self.ncon):
                g_L[i] = self.plane.d
        elif self.index == self.NLP_framework.nFixedPlanCstr:
            for i in range(0, self.ncon):
                g_L[i] = np.float_(self.box[0, i])
        elif self.index < self.NLP_framework.nCstr - self.NLP_framework.nMobilePlanCstr:
            for i in range(0, 8):
                g_L[i] = self.NLP_framework.config.securityDistance
                g_L[i+8] = self.NLP_framework.config.securityDistance
                g_L[i+16] = self.NLP_framework.config.securityDistance
        else:
           g_L[0] = 1.0

        return g_L

    def getUB(self):
        g_U = np.array(np.zeros(self.ncon), np.float_)
        if self.index < self.NLP_framework.nFixedPlanCstr:
            g_U = np.array(np.ones(self.ncon), np.float_) * np.inf
        elif self.index == self.NLP_framework.nFixedPlanCstr:
            for i in range(0, self.ncon):
                g_U[i] = np.float_(self.box[0, i])
        elif self.index < self.NLP_framework.nCstr - self.NLP_framework.nMobilePlanCstr:
            g_U = np.array(np.ones(self.ncon), np.float_) * np.inf
        else:
            g_U[0] = 1.0
        return g_U

    def eval_g(self, x, user_data=None):
        self.g = np.array(np.zeros(self.ncon), np.float_)
        if self.index < self.NLP_framework.nFixedPlanCstr:
            iC = self.box.index
            trans = x[3*iC:3*iC+3] #np.squeeze(np.asarray(self.initPos))
            for i in range(0, self.ncon):
                self.g[i] = (trans + self.box.vertex[i]).dot(np.squeeze(np.asarray(self.plane.normal))) - np.squeeze(np.asarray(self.plane.d))
        elif self.index == self.NLP_framework.nFixedPlanCstr:
            trans = x[3*self.NLP_framework.config.nBoxes-3: 3*self.NLP_framework.config.nBoxes]
            trans = x[33:36]
            for i in range(0, self.ncon):
                self.g[i] = np.float_(trans[i])
        elif self.index < self.NLP_framework.nCstr - self.NLP_framework.nMobilePlanCstr:
            transBelow = np.array(self.NLP_framework.config.nObstacles_center)
            normal = x[-1*self.NLP_framework.config.nBoxes +4*self.index -3: -1*self.NLP_framework.config.nBoxes +4*self.index]
            d = x[-1*self.NLP_framework.config.nBoxes +4*self.index -4]

            if (self.iBox0Above != -1 and self.iBox1Above != self.NLP_framework.config.nBoxes):
                trans0Above = x[3*self.iBox0Above: 3+3*self.iBox0Above]
                trans1Above = x[3*self.iBox1Above: 3+3*self.iBox1Above]
                box0AbovePlanFct = self.NLP_framework.boxAbovePlanFcts[self.iBox0Above]
                box1AbovePlanFct = self.NLP_framework.boxAbovePlanFcts[self.iBox1Above]
            elif (self.iBox0Above == -1 and self.iBox1Above != self.NLP_framework.config.nBoxes):
                trans0Above = np.squeeze(np.asarray(self.NLP_framework.config.initPos))
                trans1Above = x[3 * self.iBox1Above: 3 + 3 * self.iBox1Above]
                box0AbovePlanFct = self.NLP_framework.initBoxAbovePlanFct
                box1AbovePlanFct = self.NLP_framework.boxAbovePlanFcts[self.iBox1Above]
            for i in range(0, 8):
                self.g[i] = (trans0Above + box0AbovePlanFct.vertex[i]).dot(normal) - d
                self.g[i+8] = (trans1Above + box0AbovePlanFct.vertex[i]).dot(normal) - d
                self.g[i+16] = (transBelow + self.NLP_framework.obstacleAbovePlanFcts[self.iBoxBelow].vertex[i]).dot(-normal) + d
        else:
            x_now = x[-5*self.NLP_framework.config.nBoxes +4*self.index -3: -5*self.NLP_framework.config.nBoxes +4*self.index]
            self.g[0] = x_now[0] * x_now[0] + x_now[1] * x_now[1] + x_now[2] * x_now[2]

        return self.g


    def eval_jac_g(self, x, flag, user_data=None):

        if self.index < self.NLP_framework.nFixedPlanCstr:
            if flag:
                iC = 3*self.box.index
                self.jac_g = (np.array([0, 1, 2, 3, 4, 5, 6, 7]),
                        np.array([iC+2, iC+2, iC+2, iC+2, iC+2, iC+2, iC+2, iC+2]))
                return self.jac_g

            else:
                iC = self.box.index
                trans = x[3 * iC:3 * iC + 3]
                normal = np.squeeze(np.asarray(self.plane.normal))
                self.jac_g = np.array([normal[2],
                                 normal[2],
                                 normal[2], #1
                                 normal[2],
                                 normal[2],
                                 normal[2], #2
                                 normal[2],
                                 normal[2] ], np.float_)
                return self.jac_g
        elif self.index == self.NLP_framework.nFixedPlanCstr:
            if flag:
                iC = 3*(self.NLP_framework.config.nBoxes - 1);
                self.jac_g = (np.array([0, 1, 2]),
                              np.array([iC, iC + 1, iC + 2]))
                return self.jac_g
            else:
                normal = np.squeeze(np.asarray(self.plane.normal))
                self.jac_g = np.array([1.0, 1.0, 1.0], np.float_)
                return self.jac_g
        elif self.index < self.NLP_framework.nCstr - self.NLP_framework.nMobilePlanCstr:
            if flag:
                iC = -1*self.NLP_framework.config.nBoxes +4*self.index -4
                if (self.iBox0Above != -1 and self.iBox1Above != self.NLP_framework.config.nBoxes):
                    row_index = np.array(np.zeros(3), np.int)
                    for i in range(1, 8):
                        row_index2 = np.array(np.ones(3) * i, np.int)
                        row_index = np.append(row_index, row_index2)  # for box 1 (3 * 8)
                    for i in range(8, 16):
                        row_index = np.append(row_index, np.ones(3) * i)  # for box 1 (3 * 8
                    for i in range(0, 8):
                        row_index = np.append(row_index, np.ones(1) * i)  # for plane d1 (1 * 8)
                    for i in range(0, 8):
                        row_index = np.append(row_index, np.array(np.ones(3) * i))  # for plane normal1 (3*8)
                    for i in range(8, 16):
                        row_index = np.append(row_index, np.array(np.ones(1) * i))  # for plane d2 (1*8)
                    for i in range(8, 16):
                        row_index = np.append(row_index, np.array(np.ones(3) * i))  # for plane normal2 (3*8)
                    for i in range(16, 24):
                        row_index = np.append(row_index, np.array(np.ones(1) * i))  # for plane obstalce d (1*8)
                    for i in range(16, 24):
                        row_index = np.append(row_index, np.array(np.ones(3) * i))  # for plane obstalce normal (3*8)

                    ####### Column ###########
                    col_index = np.array([3 * self.iBox0Above, 3 * self.iBox0Above + 1, 3 * self.iBox0Above + 2], np.int)
                    for i in range(1, 8):
                        col_index2 = np.array([3 * self.iBox0Above, 3 * self.iBox0Above + 1, 3 * self.iBox0Above + 2], np.int)
                        col_index = np.append(col_index, col_index2)  # for box 1 ( 8 * 3)
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([3 * self.iBox1Above, 3 * self.iBox1Above + 1, 3 * self.iBox1Above + 2], np.int))
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC]))  # for plane d (8 * 1)
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC + 1, iC + 2, iC + 3]))  # for plane normal ( 8 * 3)
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC]))  # for plane d (8 * 1)
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC + 1, iC + 2, iC + 3]))  # for plane normal ( 8 *
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC]))  # for plane d (8 * 1)
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC + 1, iC + 2, iC + 3]))  # for plane normal ( 8 *
                elif (self.iBox0Above == -1 and self.iBox1Above != self.NLP_framework.config.nBoxes):
                    row_index = np.array(np.ones(3) * 8, np.int)
                    for i in range(9, 16):
                        row_index2 = np.array(np.ones(3)*i, np.int)
                        row_index = np.append(row_index, row_index2) # for box 1 (3 * 8)
                    for i in range(0, 8):
                        row_index = np.append(row_index, np.ones(1)*i) # for plane d1 (1 * 8)
                    for i in range(0, 8):
                        row_index = np.append(row_index, np.array(np.ones(3)*i)) # for plane normal1 (3*8)
                    for i in range(8, 16):
                        row_index = np.append(row_index, np.array(np.ones(1)*i)) # for plane d2 (1*8)
                    for i in range(8, 16):
                        row_index = np.append(row_index, np.array(np.ones(3) * i))  # for plane normal2 (3*8)
                    for i in range(16, 24):
                        row_index = np.append(row_index, np.array(np.ones(1)*i)) # for plane obstalce d (1*8)
                    for i in range(16, 24):
                        row_index = np.append(row_index, np.array(np.ones(3) * i))  # for plane obstalce normal (3*8)

                    ####### Column ###########
                    col_index = np.array([3 * self.iBox1Above, 3 * self.iBox1Above+1, 3 * self.iBox1Above+2], np.int)
                    for i in range(1, 8):
                        col_index2 = np.array([3 * self.iBox1Above, 3 * self.iBox1Above+1, 3 * self.iBox1Above+2], np.int)
                        col_index = np.append(col_index, col_index2) # for box 1 ( 8 * 3)
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC])) # for plane d (8 * 1)
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC+1, iC+2, iC+3])) # for plane normal ( 8 * 3)
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC])) # for plane d (8 * 1)
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC+1, iC+2, iC+3])) # for plane normal ( 8 *
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC])) # for plane d (8 * 1)
                    for i in range(0, 8):
                        col_index = np.append(col_index, np.array([iC+1, iC+2, iC+3])) # for plane normal ( 8 *
                self.jac_g = (row_index, col_index)
                print(self.jac_g)
                return self.jac_g
            else:
                transBelow = np.array(self.NLP_framework.config.nObstacles_center)
                normal = x[-1*self.NLP_framework.config.nBoxes +4*self.index -3: -1*self.NLP_framework.config.nBoxes +4*self.index]
                d = x[-1*self.NLP_framework.config.nBoxes +4*self.index -4]

                if (self.iBox0Above != -1 and self.iBox1Above != self.NLP_framework.config.nBoxes):
                    self.jac_g = np.array(np.zeros(self.nnzj), np.float_)
                    trans0Above = x[3*self.iBox0Above: 3+3*self.iBox0Above]
                    trans1Above = x[3*self.iBox1Above: 3+3*self.iBox1Above]
                    box0AbovePlanFct = self.NLP_framework.boxAbovePlanFcts[self.iBox0Above]
                    box1AbovePlanFct = self.NLP_framework.boxAbovePlanFcts[self.iBox1Above]
                    for j in range(0, 16):
                        for i in range(0, 3):
                            self.jac_g[i + 3 * j] = normal[i]
                    for i in range(0, 8):
                        self.jac_g[48 + i] = -1.0
                    tmpDiff = self.getDiffNormal(trans0Above, np.array([0, 0, 0, 1]), box0AbovePlanFct)
                    for i in range(0, 24):
                        self.jac_g[56 + i] = tmpDiff[i] # 56
                    for i in range(0, 8):
                        self.jac_g[80 + i] = -1.0 # 64
                    tmpDiff = self.getDiffNormal(trans1Above, np.array([0, 0, 0, 1]), box1AbovePlanFct)
                    for i in range(0, 24):
                        self.jac_g[88 + i] = tmpDiff[i] # 56
                    for i in range(0, 8):
                        self.jac_g[112 + i] = 1.0 # 64
                    tmpDiff = self.getDiffNormal(transBelow, np.array([0, 0, 0, 1]), self.NLP_framework.obstacleAbovePlanFcts[self.iBoxBelow])
                    for i in range(0, 24):
                        self.jac_g[120 + i] = -tmpDiff[i] # 56

                elif (self.iBox0Above == -1 and self.iBox1Above != self.NLP_framework.config.nBoxes):
                    self.jac_g = np.array(np.zeros(self.nnzj), np.float_)
                    trans0Above = np.squeeze(np.asarray(self.NLP_framework.config.initPos))
                    trans1Above = x[3 * self.iBox1Above: 3 + 3 * self.iBox1Above]
                    box0AbovePlanFct = self.NLP_framework.initBoxAbovePlanFct
                    box1AbovePlanFct = self.NLP_framework.boxAbovePlanFcts[self.iBox1Above]
                    for j in range(0, 8):
                        for i in range(0, 3):
                            self.jac_g[i + 3 * j] = normal[i]
                    for i in range(0, 8):
                        self.jac_g[24 + i] = -1.0
                    tmpDiff = self.getDiffNormal(trans0Above, np.array([0, 0, 0, 1]), box0AbovePlanFct)
                    for i in range(0, 24):
                        self.jac_g[32 + i] = tmpDiff[i] # 56
                    for i in range(0, 8):
                        self.jac_g[56 + i] = -1.0 # 64
                    tmpDiff = self.getDiffNormal(trans1Above, np.array([0, 0, 0, 1]), box1AbovePlanFct)
                    for i in range(0, 24):
                        self.jac_g[64 + i] = tmpDiff[i] # 56
                    for i in range(0, 8):
                        self.jac_g[88 + i] = 1.0 # 64
                    tmpDiff = self.getDiffNormal(transBelow, np.array([0, 0, 0, 1]), self.NLP_framework.obstacleAbovePlanFcts[self.iBoxBelow])
                    for i in range(0, 24):
                        self.jac_g[96 + i] = -tmpDiff[i] # 56
                return self.jac_g
        else:
            if flag:
                iC = -5*self.NLP_framework.config.nBoxes +4*self.index -3
                self.jac_g = (np.array([0, 0, 0]),
                              np.array([iC, iC + 1, iC + 2]))
                return self.jac_g
            else:
                x_now = x[-5*self.NLP_framework.config.nBoxes +4*self.index -3: -5*self.NLP_framework.config.nBoxes +4*self.index]
                self.jac_g = np.array([2.0*x_now[0], 2.0*x_now[1], 2.0*x_now[2]], np.float_)
                return self.jac_g

    def getDiffNormal(self, t, q, box):
        diff = np.array(np.zeros(24))
        for i in range(0, 8):
            v = box.vertex[i]
            diff[i * 3] = t[0] - v[0] * (2* q[1] * q[1] + 2*q[2] * q[2] - 1.0) - v[1] * (2*q[3] * q[2] - 2*q[0] *q[1]) + v[2] * (2 *q[3] *q[1] + 2*q[0]*q[2])
            diff[i * 3 + 1] = t[1] - v[1] * (2 * q[0] * q[0] + 2 * q[2] * q[2] - 1.0) + v[0] * (
            2 * q[3] * q[2] + 2 * q[0] * q[1]) - v[2] * (2 * q[3] * q[0] - 2 * q[1] * q[2])
            diff[i * 3 + 2] = t[2] - v[2] * (2 * q[0] * q[0] + 2 * q[1] * q[1] - 1.0) - v[0] * (
            2 * q[3] * q[1] - 2 * q[0] * q[2]) + v[1] * (2 * q[3] * q[0] + 2 * q[1] * q[2])
        return diff
'''
0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00 -1.00 -1.10 -0.05 -0.37  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00

void BoxAboveFixedPlan::diffTrans(Eigen::Ref<Eigen::Matrix<double, 1, 3>> res,
                                  const Eigen::Ref<const Eigen::Vector3d> trans,
                                  const Eigen::Ref<const Eigen::Vector4d> quat,
                                  const long& index) const
{
  BoxAbovePlan::diffTrans(res, trans, quat, d_, normal_, index);
}
void BoxAbovePlan::diffTrans(Eigen::Ref<Eigen::Matrix<double, 1, 3>> res,
                             const Eigen::Ref<const Eigen::Vector3d>,
                             const Eigen::Ref<const Eigen::Vector4d>,
                             const double&,
                             const Eigen::Ref<const Eigen::Vector3d> n,
                             const long&) const
{
  res(0, 0) = n.x();
  res(0, 1) = n.y();
  res(0, 2) = n.z();
}

'''