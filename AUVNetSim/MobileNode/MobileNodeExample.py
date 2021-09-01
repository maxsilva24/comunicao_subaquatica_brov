#!/usr/bin/env python

###########################################################################
#    Copyright (C) 2007 by Justin Eskesen                                      
#    <jge@mit.edu>                                                             
#
# Copyright: See COPYING file that comes with this distribution
#
###########################################################################

import src.Simulation as AUVSim
import sys

def ReorderPositions(PositionContainer):
    """Reorder an array (list or tuple) of positions tuples into 3 lists:
    a list of x's,y's, and z's for plotting.
    """
    x=[]
    y=[]
    z=[]
    for pos in PositionContainer:
        x.append(pos[0])
        y.append(pos[1])
        z.append(pos[2])
    
    return (x,y,z)
    
if(len(sys.argv) < 2):
    print ("usage: ", sys.argv[0], "ConfigFile")
    exit(1)

config = AUVSim.ReadConfigFromFile(sys.argv[1])

#the results we will plot... later
offered_load = []
throughput = []

print ("Running simulation..." )   
nodes=AUVSim.RunSimulation(config)
print ("Done")

print ("Plotting Results...")
# import pylab
from matplotlib import pylab
import matplotlib.axes3d as p3

fig=pylab.figure()
ax = p3.Axes3D(fig)

positions=[]

for node in nodes.values():
    pos=node.GetCurrentPosition()
    positions.append(pos)
    for log_line in node.app_layer.log:
        route = [i[1] for i in log_line["route"]]
        route.append(pos)
        rx,ry,rz = ReorderPositions(route)
        ax.plot3d(rx,ry,rz)
##    ax.text(node.GetCurrentPosition()[0],node.GetCurrentPosition()[1],node.name)        

nx,ny,nz = ReorderPositions(positions)
ax.scatter3d(nx,ny,nz)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.axis("equal")
pylab.show()
print ("Done")
