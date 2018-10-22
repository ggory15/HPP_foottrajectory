import numpy as np
from numpy import linalg
from ..functions import *

class BoxesHullTrajProble(object):
    def __init__(self, config):
        self.config = config
        self.nPlanes = self.config.nBoxes * self.config.nObstacles
        self.nFixedPlanes = self.config.nfixedPlane

        self.boxAboveFixedPlanFcts = [];
        self.boxAbovePlanFcts = [];
        self.fixedPlane = [];
        self.obstacleAbovePlanFcts = [];
        self.plan = [];
        self.obstalce = [];

        self.nMobilePlanCstr = self.config.nObstacles * self.config.nBoxes
        self.nFixedPlanCstr = self.nFixedPlanes * self.config.nBoxes
        self.nCstr = self.nFixedPlanCstr +1 *2 * self.nMobilePlanCstr
        self.manifold_size = self.config.manifold_size

        for i in range(0, self.nPlanes):
            self.boxAbovePlanFcts.append(Box(i, np.squeeze(np.asarray(self.config.boxSize))))

        self.fixedFinalBox = np.matrix([self.config.finalPos])
        self.fixedPlane.append(Plane(np.matrix([self.config.fixedPlane_normal]), np.matrix([self.config.fixedPlane_d])))

        for i in range(0, len(self.fixedPlane)):
            for j in range(0, len(self.boxAbovePlanFcts)):
                self.boxAboveFixedPlanFcts.append(BoxWithPlane(self.boxAbovePlanFcts[j], self.fixedPlane[i]))

        for i in range(0, self.config.nObstacles):
            self.obstacleAbovePlanFcts.append(Box(i, self.config.nObstacles_size, np.matrix([self.config.nObstacles_center])))

        for i in range(0, len(self.boxAboveFixedPlanFcts)):
            print("Box", self.boxAboveFixedPlanFcts[i].Box.index, "above fixed plane: normal", self.boxAboveFixedPlanFcts[i].Plane.normal, "d", self.boxAboveFixedPlanFcts[i].Plane.d)

        for i in range(-1, self.config.nBoxes-1):
            for j in range(0, self.config.nObstacles):
                self.plan.append(PlaneForHull(j, i, i+1))
        for i in range(0, self.nPlanes):
            print("Plan", i, "between Hull Boxes", self.plan[i].box0Above, "and", self.plan[i].box1Above, "and obstacles", self.plan[i].boxBelow)

    def findInitPoint(self):

        xInit = np.matrix(np.zeros(self.manifold_size))

        pi = np.pi

        for i in range(0, self.config.nBoxes):
            xInit[0, 3*i:3+3*i] = np.matrix([self.config.initPos]) + (i+1.0) * (np.matrix([self.config.finalPos])- np.matrix([self.config.initPos])) / self.config.nBoxes
            xInit[0, 2+3*i] += self.config.maxStepHeight * np.sin( (i+1.0) / self.config.nBoxes * pi)

        for i in range(0, len(self.plan)):
            box0Above = self.getBoxPositionFromX(self.plan[i].box0Above, xInit)
            box1Above = self.getBoxPositionFromX(self.plan[i].box1Above, xInit)
            boxBelow = self.obstacleAbovePlanFcts[self.plan[i].boxBelow].pos
            n = (box0Above + box1Above)/ 2.0 - boxBelow;

            center = boxBelow + n/2.0;
            n = n/linalg.norm(n, 2)
            d = center * n.transpose()
            xInit[0, 3 * self.config.nBoxes + 4 * i] = d
            xInit[0, 3 * self.config.nBoxes + 4 * i + 1: 3 * self.config.nBoxes + 4 * i + 4] = n

        return xInit

    def getBoxPositionFromX(self, i, x):
        if i == -1:
            return np.matrix([self.config.initPos])
        else:
            return x[0, 3*i:3+3*i]


