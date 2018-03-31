# Author: Kyle Ahn

import math
from pathplanning import PathPlanningProblem, Rectangle

class AStarSearch:
    def __init__(self, domain, root, initial_rectangle, goal_rectangle):
        self.domain = domain
        self.root = root
        self.initial_rectangle = initial_rectangle
        self.goal_rectangle = goal_rectangle
        self.initial_node = self.find_node(self.root, self.initial_rectangle)
        self.goal_node = self.find_node(self.root, self.goal_rectangle)

        self.initial_node.append(0)
        self.initial_node.append(self.calculate_h_cost(self.root, self.goal_node))
        self.initial_node.append(None)
        self.leaf_nodes = []
        self.generate_leaf_nodes(self.root)
        self.path = self.find_path(self.initial_node, self.goal_node)
        self.path_length = self.calculate_path_length(self.path)



    #=================================================================================================
    # find_node( self, node, rectangle)
    #  - Finds a node that contains the rectangle and returns that specific node.
    #  - It is specifically to be used in order to find the node that contains initial or goal rectangle.
    #=================================================================================================
    def find_node(self, node, rectangle):
        # if the node is free space and it contains the rectangle, this is the node that we are looking for.
        if node[1] == 'free' and node[0].CalculateOverlap(rectangle) > 0.0:
            return node
        to_return = None
        children_nodes = node[2]
        for i in range(0, len(children_nodes)):
            if children_nodes[i][0].CalculateOverlap(rectangle) > 0.0:
                to_return = self.find_node(children_nodes[i], rectangle)
        return to_return


    # =================================================================================================
    # find_path( self, node, rectangle)
    #  - A* Algorithm
    #  - finds the optimal path
    # =================================================================================================
    def find_path(self, root, goal_node):
        open_nodes = []                                         # list of not visited nodes
        closed_nodes = []                                       # list of visited nodes
        open_nodes.append(root)
        while len(open_nodes) > 0:
            curr = self.priority_remove(open_nodes)             # we get a node with the most promising f cost
            open_nodes.remove(curr)
            #print("X: ", curr[0].x, ", Y: ", curr[0].y, ", W: ", curr[0].width, ", H: ", curr[0].height)
            if goal_node[0].CalculateOverlap(curr[0]) or curr[0].CalculateOverlap(goal_node[0]):                               # base case.
                print("A* Found the Goal")
                return self.reconstruct_path(curr)
                                                                # 2 cases for generating neighbour nodes.                                             # Case 2: if cur is not mixed, then we just retrieve
            neighbour_nodes = self.generate_neighbours(curr)   # the neighbour nodes from curr position.
            for i in range(0, len(neighbour_nodes)):
                # if neighbour node is not in the open_nodes nor in closed_nodes, we calculate new G and H cost for the
                # neighbour node and add it as a children of the curr node.
                if self.exists(neighbour_nodes[i], open_nodes) == -9999 and self.exists(neighbour_nodes[i], closed_nodes) == -9999:
                    neighbour_nodes[i].append(curr[3] + self.calculate_g_cost(curr, neighbour_nodes[i]))  # node[3] = G cost
                    neighbour_nodes[i].append(self.calculate_h_cost(neighbour_nodes[i], goal_node))       # node[4] = H cost
                    neighbour_nodes[i].append(curr)          # node[5] = parent node
                    curr[2].append(neighbour_nodes[i])
                    open_nodes.append(neighbour_nodes[i])

                # if the neighbour already exists in the open_nodes list, then we update the cost
                if self.exists(open_nodes, neighbour_nodes[i]) != -9999:
                    new_cost = curr[3] + self.calculate_g_cost(curr, neighbour_nodes[i])
                    if new_cost < neighbour_nodes[i][3]:
                        neighbour_nodes[i][3] = new_cost
                        neighbour_nodes[i][4] = self.calculate_h_cost(neighbour_nodes[i], goal_node)
                        neighbour_nodes[i][5] = curr
            closed_nodes.append(curr)


    # =================================================================================================
    # calculate_g_cost(src, dest)
    #  - calculates the cost to move from the source to destination
    #     - CASE 1: When the destination node is not within the source node, we simply calculate
    #               the euclidean distance
    #     - CASE 2: If the destination node is inside the source node, then we need to calculate
    #               the g cost in the way that it does not exceed the the source node cell's g cost
    # =================================================================================================
    def calculate_g_cost(self, src, dest):
        # if the destination's x and y are within the source node, we consider the case 2.
        src_x_center = src[0].x+(src[0].width)/2
        src_y_center = src[0].y+(src[0].height)/2
        dest_x_center = dest[0].x + (dest[0].width) / 2
        dest_y_center = dest[0].y + (dest[0].height) / 2

        g_cost = math.sqrt(math.pow(dest_x_center-src_x_center, 2) + math.pow(dest_y_center-src_y_center, 2) )
        return g_cost


    # =================================================================================================
    # calculate_h_cost(curr, goal)
    #  - calculates the distance between curr node and goal node.
    #  - to be used as a heuristic value.
    # =================================================================================================
    def calculate_h_cost(self, curr, goal):
        return math.sqrt((goal[0].x - curr[0].x)*(goal[0].x - curr[0].x) + (goal[0].y - curr[0].y)*(goal[0].y - curr[0].y))


    # =================================================================================================
    #  Priority_remove(node_list)
    #  - returns the node with the most promising(lowest) f cost.
    # =================================================================================================
    def priority_remove(self, node_list):
        to_return = node_list[0]
        min_f_cost = node_list[0][3] + node_list[0][4]  # F = G + H
        for i in range(1, len(node_list)):
            if min_f_cost > node_list[i][3] + node_list[i][4]:
                min_f_cost = node_list[i][3] + node_list[i][4]
                to_return = node_list[i]
        return to_return


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
    #  - Returns a list of [x,y] with the path to the goal.
    # =================================================================================================
    def reconstruct_path(self, curr):
        to_return = []
        to_return.append([self.goal_rectangle.x + self.goal_rectangle.width/2, self.goal_rectangle.y + self.goal_rectangle.height/2])
        while curr[5] is not None:
            to_return.append([curr[0].x + curr[0].width/2, curr[0].y + curr[0].height/2])
            curr = curr[5]
        to_return.append([self.initial_rectangle.x + self.initial_rectangle.width/2, self.initial_rectangle.y + self.initial_rectangle.height/2])
        return to_return


    # =================================================================================================
    #  calculate_path_length
    #  - returns the length of the path from the initial point to the goal point
    # =================================================================================================
    def calculate_path_length(self, path):
        length = 0
        for i in range(1, len(path)):
            length += math.sqrt(math.pow(path[i][0] - path[i-1][0], 2) + math.pow(path[i][1] - path[i-1][1], 2))
        return length


    # =================================================================================================
    #  generate_leaf_nodes
    #  - takes in the root node as a parameter and stores all the free leaf nodes in "leaf_nodes" list
    # =================================================================================================
    def generate_leaf_nodes(self, node):
        if node[1] == "free":
            self.leaf_nodes.append(node)
        elif len(node[2]) != 0:
            self.generate_leaf_nodes(node[2][0])
            self.generate_leaf_nodes(node[2][1])
            self.generate_leaf_nodes(node[2][2])
            self.generate_leaf_nodes(node[2][3])


    # =================================================================================================
    #  generate_neighbours
    #  - Using AABB algorithm, we are detecting the neighbour rectangles that are interacting with
    #    the current rectangle.
    # =================================================================================================
    def generate_neighbours(self, curr):
        neighbours = []
        minX1 = curr[0].x - 0.1
        minY1 = curr[0].y - 0.1
        maxX1 = curr[0].x + curr[0].width + 0.1
        maxY1 = curr[0].y + curr[0].height + 0.1
        for i in range(0, len(self.leaf_nodes)):
            minX2 = self.leaf_nodes[i][0].x
            minY2 = self.leaf_nodes[i][0].y
            maxX2 = self.leaf_nodes[i][0].x + self.leaf_nodes[i][0].width
            maxY2 = self.leaf_nodes[i][0].y + self.leaf_nodes[i][0].height
            if maxX1 > minX2 and minX1 < maxX2 and maxY1 > minY2 and minY1 < maxY2:
                neighbours.append(self.leaf_nodes[i])

        return neighbours


