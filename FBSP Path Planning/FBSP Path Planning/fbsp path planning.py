import sys
import matplotlib.pyplot as plt
import copy
import timeit
from pathplanning import PathPlanningProblem, Rectangle
from aStar import AStarSearch
from rrt import RRT


# =================================================================================================
#  cCellDecomposition( domain, minimumSize )
#  - Super class CellDecomposition for QuadTree and BST
# =================================================================================================
class CellDecomposition:
    def __init__(self, domain, minimumSize):
        self.domain = domain
        self.minimumSize = minimumSize
        self.root = [Rectangle(0.0, 0.0, domain.width, domain.height), 'unknown', []]

    def Draw(self, ax, node=None):
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
        # elif node[1] == 'path':        # Fill the path with a certain colour
        #   color = 'ffffff'
        #   r.set_fill(True)
        #   r.set_facecolor(color)
        else:
            print("Error: don't know how to draw cell of type", node[1])
        # print('Draw node', node)
        ax.add_patch(r)
        for c in node[2]:
            self.Draw(ax, c)

    def CountCells(self, node=None):
        if node is None:
            node = self.root
        sum = 0
        if node[2] != []:
            sum = 0
            for c in node[2]:
                sum = sum + self.CountCells(c)
        else:
            sum = 1
        return sum


# =================================================================================================
#  QuadTreeDecomposition( domain, minimumSize, initial, goals )
#  - Basic quadtree that divides every node quarterly if is is a mixed cell.
#  - Initial and goal cells are also divided quarterly if they exist in the mixed cell.
# =================================================================================================
class QuadTreeDecomposition(CellDecomposition):
    def __init__(self, domain, minimumSize, initial, goals):
        self.initialCell = initial
        self.goalsCell = goals
        super().__init__(domain, minimumSize)
        self.root = self.Decompose(self.root)


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
            if o.CalculateOverlap(r) >= rwidth * rheight:
                cell = 'obstacle'
                break
            elif o.CalculateOverlap(r) > 0.0:
                cell = 'mixed'
                break
        if cell == 'mixed':
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


# =================================================================================================
#  main()
#   - A main function that starts and draws the results of A* and RRT path finding algorithm.
# =================================================================================================
def main( argv = None ):
    if ( argv == None ):
        argv = sys.argv[1:]

    width = 100.0
    height = 100.0

    pp = PathPlanningProblem( width, height, 60, 40, 40)
    initial, goals = pp.CreateProblemInstance()

    fig = plt.figure()
    ax = fig.add_subplot(1, 2, 1, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    for o in pp.obstacles:
        ax.add_patch(copy.copy(o.patch) )
    ip = plt.Rectangle((initial[0], initial[1]), 1.0, 1.0, facecolor='#ff0000')
    ax.add_patch(ip)

    for g in goals:
        g = plt.Rectangle((g[0], g[1]), 1.0, 1.0, facecolor='#00ff00')
        ax.add_patch(g)

    qtd = QuadTreeDecomposition(pp, 0.2, initial, goals)
    qtd.Draw(ax)
    n = qtd.CountCells()

    start = timeit.default_timer()
    astar = AStarSearch(qtd.domain, qtd.root, Rectangle(qtd.initialCell[0], qtd.initialCell[1], 0.1, 0.1),
                        Rectangle(qtd.goalsCell[0][0], qtd.goalsCell[0][1], 0.1, 0.1))
    stop = timeit.default_timer()
    print("A* Running Time: ", stop - start)
    print("A* Path Length : ", astar.path_length)
    plt.plot([x for (x, y) in astar.path], [y for (x, y) in astar.path], '-')
    ax.set_title('Quadtree Decomposition\n{0} cells'.format(n))

    ax = fig.add_subplot(1, 2, 2, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)


    for o in pp.obstacles:
        ax.add_patch(copy.copy(o.patch))
    ip = plt.Rectangle((initial[0],initial[1]), 1, 1, facecolor='#ff0000')
    ax.add_patch(ip)

    goal= None
    for g in goals:
        goal = g
        g = plt.Rectangle((g[0],g[1]), 1, 1, facecolor='#00ff00')
        ax.add_patch(g)
    start = timeit.default_timer()
    spath = RRT.ExploreDomain(RRT(8),pp, initial, goal, 5000)
    path = spath[0]
    final = spath[1]
    if len(final) == 0:
        print("No path found")
    RRT.draw(RRT(0), plt, path, final)
    stop = timeit.default_timer()
    print("RRT Running Time: ", stop - start)
    if len(final) > 0:
        print("RRT Path Length : ", RRT.pathLen(RRT(0), final))
    ax.set_title('RRT')
    plt.show()

if ( __name__ == '__main__' ):
    main()


