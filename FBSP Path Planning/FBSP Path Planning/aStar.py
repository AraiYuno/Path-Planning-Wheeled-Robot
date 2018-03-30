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
        self.find_path(self.initial_node, self.goal_node)
        self.leaf_nodes = []


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
        #print("Initial Node's Rectangle: ", to_return[0].x, ", ", to_return[0].y, " width: ", to_return[0].width,
        #      ", height: "
        #      , to_return[0].height)
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
            print("X: ", curr[0].x, ", Y: ", curr[0].y, ", W: ", curr[0].width, ", H: ", curr[0].height)
            if goal_node[0].CalculateOverlap(curr[0]) or curr[0].CalculateOverlap(goal_node[0]):                               # base case.
                print("GOAL?")
                return self.reconstruct_path(curr)
                                                                # 2 cases for generating neighbour nodes.
            if curr[1] == "mixed":                              # Case 1: if curr is mixed, we divide the curr node
                print("isMixed")
                neighbour_nodes = self.divide_node(curr)        #         so that we can identify the free space
            else:                                               # Case 2: if cur is not mixed, then we just retrieve
                neighbour_nodes = self.generate_neighbour_nodes(curr)   # the neighbour nodes from curr position.
                print("Not Mixed")

            print("Length of neighour_nodes: ", len(neighbour_nodes))
            for i in range(0, len(neighbour_nodes)):
                # if neighbour is obstacle, we just ignore. Nothing to care about it.
                if neighbour_nodes[i][1] == "obstacle":
                    continue
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
                    if new_cost < neighbour_nodes[3]:
                        neighbour_nodes[i][3] = new_cost
                        neighbour_nodes[i][4] = self.calculate_h_cost(neighbour_nodes[i], goal_node)
                        neighbour_nodes[i][5] = curr
            closed_nodes.append(curr)


    def check_neighbour_cost(self, neighbours):
        for i in range(0, len(neighbours)):
            x = neighbours[i]
            print("Neighbour ", i, "- F: ", neighbours[i][3]+neighbours[i][4])


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


    # # =================================================================================================
    # #  generate_neighour_nodes(curr)
    # #  - Generates the neighbour nodes and returns the list of the neighbours to the current cell.
    # #  - it returns up to 8 neighbours.
    # # =================================================================================================
    # def generate_neighbour_nodes(self, curr):
    #     to_return = []
    #     x = curr[0].x
    #     y = curr[0].y
    #     width = curr[0].width
    #     height = curr[0].height
    #     north_neighbour = self.find_neighbour_node(self.root, Rectangle(x, y + height, width, height)) # north
    #     if north_neighbour is not None:
    #         to_return.append(north_neighbour)
    #     northeast_neighbour = self.find_neighbour_node(self.root, Rectangle(x + width, y + height, width, height)) # northeast
    #     if northeast_neighbour is not None:
    #         to_return.append(northeast_neighbour)
    #     east_neighbour = self.find_neighbour_node(self.root, Rectangle(x + width, y, width, height)) # east
    #     if east_neighbour is not None:
    #         to_return.append(east_neighbour)
    #     southeast_neighbour = self.find_neighbour_node(self.root, Rectangle(x + width, y - height, width, height)) #southeast
    #     if southeast_neighbour is not None:
    #         to_return.append(southeast_neighbour)
    #     south_neighbour = self.find_neighbour_node(self.root, Rectangle(x, y - height, width, height)) #south
    #     if south_neighbour is not None:
    #         to_return.append(south_neighbour)
    #     southwest_neighbour = self.find_neighbour_node(self.root, Rectangle(x - width, y - height, width, height)) #southwest
    #     if southwest_neighbour is not None:
    #         to_return.append(southwest_neighbour)
    #     west_neighbour = self.find_neighbour_node(self.root, Rectangle(x - width, y, width, height)) #west
    #     if west_neighbour is not None:
    #         to_return.append(west_neighbour)
    #     northwest_neighbour = self.find_neighbour_node(self.root, Rectangle(x - width, y + height, width, height)) #northwest
    #     if northwest_neighbour is not None:
    #         to_return.append(northwest_neighbour)
    #     return to_return


    # =================================================================================================
    #  find_neighbour_nodes(node, rectangle)
    #  - finds the node of given rectangle
    #  - It uses root node to find that specific node.
    # =================================================================================================
    def find_neighbour_node(self, node, rectangle):
        to_return = None
        if (node[0].x == rectangle.x and node[0].y == rectangle.y and node[0].width == rectangle.width and\
                node[0].height == rectangle.height) or (node[0].x + node[0].width == rectangle.x):
            return node
        children_nodes = node[2]
        for i in range(0, len(children_nodes)):
            if children_nodes[i][0].CalculateOverlap(rectangle) > 0.0:
                to_return = self.find_neighbour_node(children_nodes[i], rectangle)
        # if to_return is not None:
        # print("Top Node's Rectangle: ", to_return[0].x, ", ", to_return[0].y, " width: ", to_return[0].width, ", height: "
        #       , to_return[0].height)
        return to_return


    # =================================================================================================
    #  divie_node(node)
    #  - Divides the input node in 4 cells like a quadtree.
    # =================================================================================================
    def divide_node(self, node):
        # First we divide the current node's rectangle into 4.
        to_return = []
        rectangle = node[0]
        x = rectangle.x
        y = rectangle.y
        w = rectangle.width
        h = rectangle.height
        # Create 4 smaller rectangles.
        topleft = [Rectangle(x, y + h/2, w/2, h/2), 'free', []]
        botleft = [Rectangle(x, y, w/2, h/2), 'free', []]
        botright = [Rectangle(x + w/2, y, w/2, h/2), 'free', []]
        topright = [Rectangle(x + w/2, y + h/2, w/2, h/2), 'free', []]
        to_return.append(topleft)
        to_return.append(botleft)
        to_return.append(botright)
        to_return.append(topright)
        for o in self.domain.obstacles:
            if o.CalculateOverlap(topleft[0]) >= (h/2) * (w/2):
                to_return[0][1] = 'obstacle'
                break
            elif o.CalculateOverlap(topleft[0]) > 0.0:
                to_return[0][1] = 'mixed'
                break
        for o in self.domain.obstacles:
            if o.CalculateOverlap(botleft[0]) >= (h/2) * (w/2):
                to_return[1][1] = 'obstacle'
                break
            elif o.CalculateOverlap(botleft[0]) > 0.0:
                to_return[1][1] = 'mixed'
                break
        for o in self.domain.obstacles:
            if o.CalculateOverlap(botright[0]) >= (h/2) * (w/2):
                to_return[2][1] = 'obstacle'
                break
            elif o.CalculateOverlap(botright[0]) > 0.0:
                to_return[2][1] = 'mixed'
                break
        for o in self.domain.obstacles:
            if o.CalculateOverlap(topright[0]) >= (h/2) * (w/2):
                to_return[3][1] = 'obstacle'
                break
            elif o.CalculateOverlap(topright[0]) > 0.0:
                to_return[3][1] = 'mixed'
                break
        return to_return


    # =================================================================================================
    #  riority_remove(node_list)
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
    #  - Returns a list of nodes with the path to the goal.
    # =================================================================================================
    def reconstruct_path(self, curr):
        to_return = []
        while curr[5] is not None:
            to_return.append(curr)
            curr = curr[5]
        return to_return


    def generate_leaf_nodes(self, node):
        if(len(node)==2):
            if(node[1]=="free"):
                self.leaf_nodes.append(node)

        else:
            self.generate_leaf_nodes(self, node[3][0])
            self.generate_leaf_nodes(self, node[3][1])
            self.generate_leaf_nodes(self, node[3][2])
            self.generate_leaf_nodes(self, node[3][3])


    def generate_neighbours(self, curr):
        neighbours = []
        minX1 = curr.x - 0.1
        minY1 = curr.y - 0.1
        maxX1 = curr.x +curr.width + 0.1
        maxY1 = curr.y + curr.height + 0.1
        for i in len(self.leaf_nodes):
            minX2 = self.leaf_nodes[i].x
            minY2 = self.leaf_nodes[i].y
            maxX2 = self.leaf_nodes[i].x + self.leaf_nodes[i].width
            maxY2 = self.leaf_nodes[i].y + self.leaf_nodes[i].height
            if( maxX1 > minX2 and minX1 < maxX2 and maxY1 > minY2 and minY1 < maxY2):
                neighbours.append(self.leaf_nodes[i])

        return neighbours









