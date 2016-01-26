from square_grid import *

class GridWithWeights(SquareGrid):
    def __init__(self, height, width):
        super().__init__(height, width)
        self.weights = {}

    def cost(self, from_node, to_node):
        # if from_node[0] != to_node[0] and from_node[1] != to_node[1]:
        #     return self.weights.get(to_node, 1.41)
        # else:
        return self.weights.get(to_node, 1)