import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import math
import random
from pathplanning import PathPlanningProblem, Rectangle

def ExploreDomain( domain, initial, steps ):
    log = np.zeros((steps,2))
    pos = np.array(initial)
    dd = 0.1
    theta = 0.00/180.0 * math.pi

    for i in range(steps):
        newpos = pos + dd * np.array([dd * math.cos(theta), dd * math.sin(theta)])
        r = Rectangle(newpos[0], newpos[1], 0.1, 0.1)
        if ( newpos[0] >= 0.0 ) and ( newpos[0] < domain.width ) and ( newpos[1] >= 0.0 ) and ( newpos[1] < domain.height ):
            if ( not domain.CheckOverlap( r ) ):
                pos = newpos

        theta = theta + random.uniform(-180.0/180.0 * math.pi, 180.0/180.0 * math.pi)
        while( theta >= math.pi ):
            theta = theta - 2 * math.pi
        while( theta < - math.pi ):
            theta = theta + 2 * math.pi
        log[i,:] = pos
    return log

def main( argv = None ):
    if ( argv == None ):
        argv = sys.argv[1:]

    width = 10.0
    height = 10.0

    pp = PathPlanningProblem( width, height, 5, 4.0, 4.0)
    #pp.obstacles = [ Obstacle(0.0, 0.0, pp.width, pp.height / 2.2, '#555555' ) ]
    pp.obstacles = []
    initial, goals = pp.CreateProblemInstance()

    fig = plt.figure()
    ax = fig.add_subplot(1,2,1, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    for o in pp.obstacles:
        ax.add_patch(o.patch)

#    ip = plt.Rectangle((initial[0],initial[1]), 0.1, 0.1, facecolor='#ff0000')
#    ax.add_patch(ip)

    for g in goals:
        g = plt.Rectangle((g[0],g[1]), 0.1, 0.1, facecolor='#00ff00')
#        ax.add_patch(g)

    path = ExploreDomain( pp, initial, 50000 )
    ax.set_title('Vacuuming Domain')

    plt.plot(path[:,0], path[:,1], 'b-')

    ax = fig.add_subplot(1,2,2)
#    x,y,z = pp.CalculateCoverage(path, 0.5)

#    X,Y = np.meshgrid(x,y)
#    Z = z
#    ax.plot_surface(X,Y,Z, rstride=1, cstride=1, cmap=cm.coolwarm)

    heatmap, x, y = np.histogram2d(path[:,0], path[:,1], bins = 50, range=[[0.0, pp.width], [0.0, pp.height]])
    coverage = float( np.count_nonzero(heatmap) ) / float( len(heatmap) * len(heatmap[0]))
    extent = [ x[0], x[-1], y[0], y[-1]]
    ax.set_title('Random Walk\nCoverage {0}'.format(coverage))
    plt.imshow(np.rot90(heatmap))
    plt.colorbar()

    plt.show()

if ( __name__ == '__main__' ):
    main()

