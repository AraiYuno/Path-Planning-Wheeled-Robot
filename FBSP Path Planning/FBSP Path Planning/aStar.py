from pathplanning import PathPlanningProblem, Rectangle

class AStarSearch:
    def __init__(self, root, initial_rectangle, goal_rectangle):
        self.root = root
        self.initial_rectangle = initial_rectangle
        self.goal_rectangle = goal_rectangle
        self.find_node(self.root, self.initial_rectangle)


    #=================================================================================================
    # find_node( self, node, rectangle)
    #  - Finds a node that contains the rectangle and returns that specific node.
    #  - It is specifically to be used in order to find the node that contains initial or goal rectangle.
    #=================================================================================================
    def find_node(self, node, rectangle):
        # if the node is free space and it contains the rectangle, this is the node that we are looking for.
        if node[1] == 'free' and node[0].CalculateOverlap(rectangle) > 0.0:
            return node

        children_nodes = node[2]
        for i in range(0, len(children_nodes)):
            if children_nodes[i][0].CalculateOverlap(rectangle) > 0.0:
                to_return = self.find_node(children_nodes[i], rectangle)
        print("Initial Node's Rectangle: ", to_return[0].x, ", ", to_return[0].y, " width: ", to_return[0].width, ", height: "
              , to_return[0].height)
        return to_return