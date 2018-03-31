__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import copy
import random
from pathplanning import PathPlanningProblem, Rectangle


class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:
    def __init__(self, v):
        self.v = None

    #this function will return the step length depending on the number of obstacles
    def get_step_len(self, domain):
        dd = 1
        if len(domain.obstacles) >0 and len(domain.obstacles) <=12:
            dd = 5
        elif len(domain.obstacles) >12 and len(domain.obstacles) <=24:
            dd = 4
        elif len(domain.obstacles) >24 and len(domain.obstacles) <=36:
            dd = 3
        elif len(domain.obstacles) >36 and len(domain.obstacles) <=48:
            dd = 2
        elif len(domain.obstacles) >48:
            dd = 1

        return dd

    def getNearNeighbour(self, nodeList, rnd):
        index = 0
        min = 10000000
        i = 0
        for node in nodeList:
            if math.sqrt((math.pow(node.x - rnd[0], 2)) + (math.pow(node.y - rnd[1], 2))) < min:
                min = math.sqrt((math.pow(node.x - rnd[0], 2)) + (math.pow(node.y - rnd[1], 2)))
                index = i
            i = i + 1
        return index

    def ExploreDomain(self, domain, initial, goal, steps):
        init = Node(initial[0], initial[1])
        g = Node(goal[0], goal[1])
        dd = self.get_step_len(domain)
        newpos = None
        nodeList = [init]
        while True:
            if steps == 0:
                break
            if random.randint(0, 100) > 5:
                rnd = [random.uniform(0, 100), random.uniform(0, 100)]
            else:
                rnd = [g.x, g.y]

            nind = self.getNearNeighbour(nodeList, rnd)
            nearestNode = nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            newNode = Node((nearestNode.x), (nearestNode.y))
            newNode.parent = nearestNode
            newpos = Node((newNode.x + dd * math.cos(theta)), (newNode.y + dd * math.sin(theta)))
            newpos.parent = newNode
            xp = newNode.x
            yp = newNode.y
            over = False
            for i in range(dd):
                xp = xp + math.cos(theta)
                yp = yp + math.sin(theta)
                rec = Rectangle(xp, yp, 1, 1)
                if domain.CheckOverlap(rec):
                    over = True
                    break

            r = Rectangle(newpos.x, newpos.y, 1, 1)

            if ( newpos.x >= 0.0 ) and ( newpos.x < domain.width ) and ( newpos.y >= 0.0 ) and ( newpos.y < domain.height ):

                if (not over and not domain.CheckOverlap( r )):
                    steps = steps - 1
                    dx = newpos.x - g.x
                    dy = newpos.y - g.y
                    d = math.sqrt(dx * dx + dy * dy)
                    if d <= dd:
                        newpos = Node(g.x, g.y)
                        newpos.parent = newNode
                        nodeList.append(newpos)
                        break
                    nodeList.append(newpos)

        finalpath = []
        lastIndex = len(nodeList) - 1
        node = nodeList[lastIndex]
        if node.x == g.x and node.y == g.y:
            while node.parent is not None:
                finalpath.append(node)
                node = node.parent
            finalpath.append(init)

        result = [nodeList, finalpath]
        return result


    def draw(self, plt, path, final):
        s = [4 for n in range(len(path))]
        lent = len(path) - 1

        while lent >= 0:
            node = path[lent]
            if node.parent is not None:
                par = node.parent
                all = plt.plot([node.x, par.x], [node.y, par.y])
                plt.setp(all, color='b', linewidth=0.75)
            else:
                plt.scatter([node.x], [node.y], color='b', s=s)
            lent = lent - 1

        lent = len(final) - 1

        while lent >= 0:
            node = final[lent]
            if node.parent is not None:
                par = node.parent
                fin = plt.plot([node.x, par.x], [node.y, par.y])
                plt.setp(fin, color='r', linewidth=0.75)
            else:
                plt.scatter([node.x], [node.y], color='r', s=s)
            lent = lent - 1

    def pathLen(self, path):
        leng = 0
        plen = len(path)-1
        node = path[plen]
        while plen >= 0:
            node = path[plen]
            if node.parent is not None:
                par = node.parent
                leng = leng + math.sqrt(math.pow((node.x-par.x), 2) + math.pow((node.y-par.y), 2))
            plen = plen - 1
        return leng