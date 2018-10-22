import yaml

class Config(object):
    def __init__(self):
        self.nBoxes = [];

    def setFromConfigFile(self, config_file):
        with open(config_file, 'r') as stream:
            data = yaml.load(stream)
            self.nBoxes = int(data['nBoxes'])
            self.nObstacles = (data['nobstacles'])
            self.nObstacles_size = (data['obstacles_size'])
            self.nObstacles_center = (data['obstalces_center'])
            self.manifold_size = 6 * self.nBoxes + 3 * self.nObstacles * self.nBoxes
            self.initPos = data['initPos']
            self.finalPos = data['finalPos']
            self.securityDistance = data['securityDistance']
            self.maxStepHeight = data['maxStepHeight']
            self.boxSize = data['BoxSize']
            self.nfixedPlane = data['nfixedPlane']
            self.fixedPlane_normal = data['fixedPlane_normal']
            self.fixedPlane_d = data['fixedPlane_d']
