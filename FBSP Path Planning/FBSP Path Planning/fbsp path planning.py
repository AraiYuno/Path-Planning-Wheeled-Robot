import sys
import matplotlib.pyplot as plt
import math
import copy
from pathplanning import PathPlanningProblem, Rectangle
from aStar import AStarSearch

class CellDecomposition:
    def __init__(self, domain, minimumSize):
        self.domain = domain
        self.minimumSize = minimumSize
        self.root = [Rectangle(0.0, 0.0, domain.width, domain.height), 'unknown', []]

    def Draw(self, ax, node = None):
            if node == None:
                node = self.root
            r = plt.Rectangle((node[0].x, node[0].y), node[0].width, node[0].height, fill=False, facecolor=None, alpha=0.5)
            if node[1] == 'mixed':
                color = '#5080ff'
                if node[2] == []:
                    r.set_fill(True)
                    r.set_facecolor(color)
            elif node[1] == 'free':
                color = '#ffff00'
                r.set_fill(True)
                r.set_facecolor(color)
            elif node[1] == 'obstacle':
                color = '#5050ff'
                r.set_fill(True)
                r.set_facecolor(color)
            #elif node[1] == 'path':        # Fill the path with a certain colour
            #   color = 'ffffff'
            #   r.set_fill(True)
            #   r.set_facecolor(color)
            else:
                print("Error: don't know how to draw cell of type", node[1])
            #print('Draw node', node)
            ax.add_patch(r)
            for c in node[2]:
                self.Draw(ax, c)

    def CountCells(self, node = None ):
        if ( node is None ):
            node = self.root
        sum = 0
        if ( node[2] != [] ):
            sum = 0
            for c in node[2]:
                sum = sum + self.CountCells(c)
        else:
            sum = 1
        return sum

class QuadTreeDecomposition(CellDecomposition):
    def __init__(self, domain, minimumSize, initial, goals):
        self.initialCell = initial
        self.goalsCell = goals
        super().__init__(domain, minimumSize)
        self.root = self.Decompose(self.root)
        self.astar = AStarSearch(self.root, Rectangle(self.initialCell[0], self.initialCell[1], 0.1, 0.1),
                                 Rectangle(self.goalsCell[0][0], self.goalsCell[0][1], 0.1, 0.1))


    def Decompose(self, node):
        cell = 'free'
        r = node[0]
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height

        initialCell = Rectangle(self.initialCell[0], self.initialCell[1], 0.1, 0.1)
        goalCell = Rectangle(self.goalsCell[0][0], self.goalsCell[0][1], 0.1, 0.1)
        for o in self.domain.obstacles:
            if ( o.CalculateOverlap(r) >= rwidth * rheight ):
                cell = 'obstacle'
                break
            elif ( o.CalculateOverlap(r) > 0.0 ):
                cell = 'mixed'
                break
        if ( cell == 'mixed'):
            if (((rwidth / 2.0 > self.minimumSize) and (rheight / 2.0 > self.minimumSize))
                    or goalCell.CalculateOverlap(r) > 0.0 or initialCell.CalculateOverlap(r) > 0.0):
                childt1 = [Rectangle(rx, ry, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild1 = self.Decompose( childt1 )
                childt2 = [Rectangle(rx + rwidth/2.0, ry, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild2 = self.Decompose( childt2 )
                childt3 = [Rectangle(rx, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild3 = self.Decompose( childt3 )
                childt4 = [Rectangle(rx + rwidth/2.0, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild4 = self.Decompose( childt4 )
                children = [ qchild1, qchild2, qchild3, qchild4 ]
                node[2] = children
            else:
                cell = 'obstacle'
        node[1] = cell
        return node


class BinarySpacePartitioning(CellDecomposition):
    def __init__(self, domain, minimumSize):
        super().__init__(domain, minimumSize)
        self.root = self.Decompose(self.root)

    def Entropy(self, p):
        e = 0.0
        if ( ( p > 0 ) and ( p < 1.0 ) ):
            e = -p * math.log(p,2) - (1-p) * math.log(1-p,2)
        return e

    def CalcEntropy(self, rect):
        area = rect.width * rect.height
        a = 0.0
        for o in self.domain.obstacles:
            a = a + rect.CalculateOverlap(o)
        p = a / area
        return self.Entropy(p)

    def Decompose(self, node):
        cell = 'free'
        r = node[0]
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height
        area = rwidth * rheight

        for o in self.domain.obstacles:
            if ( o.CalculateOverlap(r) >= rwidth * rheight ):
                cell = 'obstacle'
                break
            elif ( o.CalculateOverlap(r) > 0.0 ):
                cell = 'mixed'
                break

        if ( cell == 'mixed'):
            entropy = self.CalcEntropy(r)
            igH = 0.0
            hSplitTop = None
            hSplitBottom = None
            vSplitLeft = None
            vSplitRight = None
            if ( r.height / 2.0 > self.minimumSize):
                hSplitTop = Rectangle(rx, ry + rheight/2.0, rwidth, rheight/2.0)
                entHSplitTop = self.CalcEntropy(hSplitTop)
                hSplitBottom = Rectangle(rx, ry, rwidth, rheight/2.0)
                entHSplitBottom = self.CalcEntropy( hSplitBottom )

                igH = entropy - ( r.width * r.height / 2.0 ) / area * entHSplitTop \
                      - ( r.width * r.height / 2.0 ) / area * entHSplitBottom
            igV = 0.0
            if ( r.width / 2.0 > self.minimumSize ):
                vSplitLeft = Rectangle(rx, ry, rwidth/2.0, rheight )
                entVSplitLeft = self.CalcEntropy( vSplitLeft )
                vSplitRight = Rectangle( rx + rwidth/2.0, ry, rwidth/2.0, rheight)
                entVSplitRight = self.CalcEntropy( vSplitRight)
                igV = entropy - ( r.width/2.0 * r.height ) / area * entVSplitLeft \
                      - ( r.width/2.0 * r.height ) / area * entVSplitRight
            children = []
            if ( igH > igV ):
                if ( igH > 0.0 ):
                    if ( hSplitTop is not None ) and ( hSplitBottom is not None ):
                        childTop = [ hSplitTop, 'unknown', [] ]
                        childBottom = [hSplitBottom, 'unknown', [] ]
                        children = [ childTop, childBottom]
            else:
                if ( igV > 0.0 ):
                    if ( vSplitLeft is not None ) and ( vSplitRight is not None ):
                        childLeft = [vSplitLeft, 'unknown', [] ]
                        childRight = [ vSplitRight, 'unknown', [] ]
                        children = [ childLeft, childRight ]
            for c in children:
                self.Decompose(c)
            node[2] = children
        node[1] = cell
        return node

def main( argv = None ):
    if ( argv == None ):
        argv = sys.argv[1:]

    width = 10.0
    height = 10.0

    pp = PathPlanningProblem( width, height, 15, 3.0, 3.0)
    #pp.obstacles = [ Obstacle(0.0, 0.0, pp.width, pp.height / 2.2, '#555555' ) ]
    initial, goals = pp.CreateProblemInstance()

    fig = plt.figure()
    ax = fig.add_subplot(1, 2, 1, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    for o in pp.obstacles:
        ax.add_patch(copy.copy(o.patch) )
    ip = plt.Rectangle((initial[0], initial[1]), 0.1, 0.1, facecolor='#ff0000')
    ax.add_patch(ip)

    for g in goals:
        g = plt.Rectangle((g[0], g[1]), 0.1, 0.1, facecolor='#00ff00')
        ax.add_patch(g)

    qtd = QuadTreeDecomposition(pp, 0.2, initial, goals)
    qtd.Draw(ax)
    n = qtd.CountCells()
    ax.set_title('Quadtree Decomposition\n{0} cells'.format(n))

    ax = fig.add_subplot(1, 2, 2, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    for o in pp.obstacles:
        ax.add_patch(copy.copy(o.patch))
    ip = plt.Rectangle((initial[0],initial[1]), 0.1, 0.1, facecolor='#ff0000')
    ax.add_patch(ip)

    for g in goals:
        g = plt.Rectangle((g[0],g[1]), 0.1, 0.1, facecolor='#00ff00')
        ax.add_patch(g)

    bsp = BinarySpacePartitioning(pp, 0.2)
    bsp.Draw(ax)
    n = bsp.CountCells()
    ax.set_title('BSP Decomposition\n{0} cells'.format(n))

    plt.show()

if ( __name__ == '__main__' ):
    main()


