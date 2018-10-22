import numpy as np

class constraint(object):
    def __init__(self, index, BoxesHullTrajProble):
        self.index = index;
        self.NLP_framework = BoxesHullTrajProble
        self.ncon = self.getConstNum()
        self.box = self.NLP_framework.boxAboveFixedPlanFcts[self.index].Box;
        self.plane = self.NLP_framework.boxAboveFixedPlanFcts[self.index].Plane;
        self.g_L = self.getLB()
        self.g_U = self.getUB()
        self.nnzj = 0;
        if self.index < self.NLP_framework.nFixedPlanCstr:
            self.nnzj = self.ncon * 3


    def getConstNum(self):
        assert self.index < self.NLP_framework.nCstr
        if self.index < self.NLP_framework.nFixedPlanCstr:
            return 0;
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
        return g_L

    def getUB(self):
        g_U = np.array(np.zeros(self.ncon), np.float_)
        if self.index < self.NLP_framework.nFixedPlanCstr:
            g_U = np.array(np.ones(self.ncon), np.float_) * np.inf

        return g_U

    def eval_g(self, x, user_data=None):
        g = np.array(np.zeros(self.ncon), np.float_)
        if self.index < self.NLP_framework.nFixedPlanCstr:
            iC = self.box.index
            trans = x[3*iC:3*iC+3] #np.squeeze(np.asarray(self.initPos))

            for i in range(0, self.ncon):
                g[i] = (trans + self.box.vertex[i]).dot(np.squeeze(np.asarray(self.plane.normal))) - np.squeeze(np.asarray(self.plane.d))

        return g


    def eval_jac_g(self, x, flag, user_data=None):
        if self.index < self.NLP_framework.nFixedPlanCstr:
            if flag:
                iC = self.box.index
                return (np.array([0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7]),
                        np.array([iC, iC+1, iC+2, iC, iC+1, iC+2, iC, iC+1, iC+2, iC, iC+1, iC+2, iC, iC+1, iC+2, iC, iC+1, iC+2, iC, iC+1, iC+2, iC, iC+1, iC+2]))
            else:
                iC = self.box.index
                trans = x[3 * iC:3 * iC + 3]
                normal = np.squeeze(np.asarray(self.plane.normal))
                return np.array([normal[0],
                                 normal[1],
                                 normal[2], #1
                                 normal[0],
                                 normal[1],
                                 normal[2], #2
                                 normal[0],
                                 normal[1],
                                 normal[2], #3
                                 normal[0],
                                 normal[1],
                                 normal[2], #4
                                 normal[0],
                                 normal[1],
                                 normal[2], #5
                                 normal[0],
                                 normal[1],
                                 normal[2], #6
                                 normal[0],
                                 normal[1],
                                 normal[2], #7
                                 normal[0],
                                 normal[1],
                                 normal[2] ])



'''
void BoxAboveFixedPlan::diffTrans(Eigen::Ref<Eigen::Matrix<double, 1, 3>> res,
                                  const Eigen::Ref<const Eigen::Vector3d> trans,
                                  const Eigen::Ref<const Eigen::Vector4d> quat,
                                  const long& index) const
{
  BoxAbovePlan::diffTrans(res, trans, quat, d_, normal_, index);
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


ncon = 2

g_L = array([25.0, 40.0])
g_U = array([2.0*pow(10.0, 19), 40.0])

def eval_g(x, user_data= None):
    assert len(x) == 4
    return array([
        x[0] * x[1] * x[2] * x[3],
        x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]
        ], float_)

nnzj = 8
def eval_jac_g(x, flag, user_data = None):
    if flag:
        return (array([0, 0, 0, 0, 1, 1, 1, 1]),
                array([0, 1, 2, 3, 0, 1, 2, 3]))
    else:
        assert len(x) == 4
        return array([ x[1]*x[2]*x[3],
            x[0]*x[2]*x[3],
            x[0]*x[1]*x[3],
            x[0]*x[1]*x[2],
            2.0*x[0],
            2.0*x[1],
            2.0*x[2],
            2.0*x[3] ])
'''