from pathplanning import PathPlanningProblem, Rectangle

class AStarSearch:
    def __init__(self, root, initial_rectangle, goal_rectangle):
        self.root = root
        self.initial_rectangle = initial_rectangle
        self.goal_rectangle = goal_rectangle
        self.initial_node = self.find_node(self.root, self.initial_rectangle)


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


    # =================================================================================================
    # find_path( self, node, rectangle)
    #  - A* Algorithm
    #  - finds the best path
    # =================================================================================================
    def find_path(self, root, goal_node):
        open_nodes = []
        closed_nodes = []
        open_nodes.append(root)
        while len(open_nodes) > 0:
            curr = open_nodes.pop()
            if curr == goal_node:
                return self.reconstruct_path(curr)

            neighbour_nodes = self.generate_neighour_nodes(curr)
            for i in range(0, len(neighbour_nodes)):
                # if neighbour is obstacle, we just ignore. Nothing to care about it.
                if neighbour_nodes[i][1] == "obstacle":
                    continue
                # if neighbour node is not in the open_nodes nor in closed_nodes, we calculate new G and H cost for the
                # neighbour node and add it as a children of the curr node.
                if neighbour_nodes[i][1] == "free" and self.exists(neighbour_nodes[i], open_nodes) == -9999 \
                        and self.exists(neighbour_nodes[i], closed_nodes) == -9999:
                    neighbour_nodes[i].append(curr[3] + self.get_distance(curr, neighbour_nodes[i]))  # node[3] = G cost
                    neighbour_nodes[i].append(self.get_distance(neighbour_nodes[i], goal_node))       # node[4] = H cost
                    neighbour_nodes[i].append(curr)          # node[5] = parent node
                    curr[2].append(neighbour_nodes[i])
                    open_nodes.append(neighbour_nodes[i])

                # if the neighbour already exists in the open_nodes list, then we update the cost
                if self.exists(open_nodes, neighbour_nodes[i]) != -9999:
                    new_cost = curr[3] + self.get_distance(curr, neighbour_nodes[i])
                    if new_cost < neighbour_nodes[3]:
                        neighbour_nodes[i][3] = new_cost
                        neighbour_nodes[i][4] = self.get_distance(neighbour_nodes[i], goal_node)
                        neighbour_nodes[i][5] = curr

            closed_nodes.append(curr)




    # =================================================================================================
    #  exists( curr, nodes_list )
    #  - Checks if input "curr" node exists in the list "nodes_list"
    #  - If curr exists in the list, it returns the index in the list, otherwise -9999
    # =================================================================================================
    def exists(self, curr, nodes_list):
        try:
            to_return = nodes_list.index(curr)
        except ValueError:
            to_return = -9999
        return to_return


    # =================================================================================================
    #  reconstruct_path( self, curr )
    #  - Takes in a goal node.
    #  - If curr exists in the list, it returns the index in the list, otherwise -9999
    # =================================================================================================
    def reconstruct_path(self, curr):