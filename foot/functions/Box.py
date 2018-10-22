import numpy as np

class Box(object):
    def __init__(self, index, size, position = np.matrix(np.zeros(3))):
        self.index = index;
        self.size = size;
        self.pos = position;
        self.vertex = []
        self.getvertex()

    def getvertex(self):
        l1 = np.array([-self.size[0] / 2.0, -self.size[1] / 2.0, -self.size[2] / 2.0], np.float_)
        l2 = np.array([-self.size[0] / 2.0, -self.size[1] / 2.0,  self.size[2] / 2.0], np.float_)
        l3 = np.array([-self.size[0] / 2.0,  self.size[1] / 2.0, -self.size[2] / 2.0], np.float_)
        l4 = np.array([-self.size[0] / 2.0,  self.size[1] / 2.0,  self.size[2] / 2.0], np.float_)
        l5 = np.array([ self.size[0] / 2.0, -self.size[1] / 2.0, -self.size[2] / 2.0], np.float_)
        l6 = np.array([ self.size[0] / 2.0, -self.size[1] / 2.0,  self.size[2] / 2.0], np.float_)
        l7 = np.array([ self.size[0] / 2.0,  self.size[1] / 2.0, -self.size[2] / 2.0], np.float_)
        l8 = np.array([ self.size[0] / 2.0,  self.size[1] / 2.0,  self.size[2] / 2.0], np.float_)

        self.vertex.append(l1)
        self.vertex.append(l2)
        self.vertex.append(l3)
        self.vertex.append(l4)
        self.vertex.append(l5)
        self.vertex.append(l6)
        self.vertex.append(l7)
        self.vertex.append(l8)

#class boxAbovePlanFcts(object):
#    def __init__(self):
