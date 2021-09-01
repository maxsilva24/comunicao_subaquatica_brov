# -*- coding: cp1252 -*-
###########################################################################
#
#    Copyright (C) 2007 by Justin Eskesen and Josep Miquel Jornet Montana                                     
#    <jge@mit.edu> <jmjornet@mit.edu>                                      
#
# Copyright: See COPYING file that comes with this distribution
#
# This file is part of AUVNetSim, a library for simulating acoustic
# networks of fixed and mobile underwater nodes, written in Python.
#
# AUVNetSim is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# AUVNetSim is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with AUVNetSim.  If not, see <http://www.gnu.org/licenses/>.
#
###########################################################################

import SimPy.Simulation as Sim
from AcousticNode import AcousticNode

def RunSimulation(config_dict):
    Sim.initialize()
    
    # Signal all nodes that a messge has been transmitted
    # no physical analog, as it is immediate.
    AcousticEvent = Sim.SimEvent("AcousticEvent")
    
    nodes = SetupNodesForSimulation(config_dict, AcousticEvent)

    Sim.simulate(until=config_dict["SimulationDuration"])

    return nodes


def SetupNodesForSimulation(config_dict, acoustic_event):
    nodes = {}

    # static nodes
    if "Nodes" in config_dict.keys():
        for n in config_dict["Nodes"]:
            cn = [config_dict,] + n
            nodes[n[0]]=AcousticNode(acoustic_event, config_dict, *n) 
    
    if "NodeField" in config_dict.keys():
        nodes.update(CreateRandomNodeField(acoustic_event, config_dict,*config_dict["NodeField"]))
    
    return nodes


def CreateRandomNodeField(acoustic_event, config, box_len, nWide, nHigh, bottom_left_position=(0,0,0), Prefix="StaticNode"):
    import random
    random.seed()
    node_positions=[]

    # Nodes can be randomly located within a virtual grid and moving randomly
    if not config["RandomGrid"] and not config["Moving"]:
        [node_positions.append((bottom_left_position[0]+box_len*x,
                                bottom_left_position[1]+box_len*y,
                                bottom_left_position[2])) for x in range(nWide) for y in range(nHigh)]
    elif config["RandomGrid"] and not config["Moving"]:
        [node_positions.append((bottom_left_position[0]+box_len*random.random()+box_len*x,
                                bottom_left_position[1]+box_len*random.random()+box_len*y,
                                bottom_left_position[2]+config["Height"]*random.random())) for x in range(nWide) for y in range(nHigh)]
    else:
        [node_positions.append(([(bottom_left_position[0]+box_len*random.random()+box_len*x,
                                bottom_left_position[1]+box_len*random.random()+box_len*y,
                                bottom_left_position[2]+config["Height"]*random.random()),
                               (bottom_left_position[0]+box_len*random.random()+box_len*x,
                                bottom_left_position[1]+box_len*random.random()+box_len*y,
                                bottom_left_position[2]+config["Height"]*random.random()),
                               (bottom_left_position[0]+box_len*random.random()+box_len*x,
                                bottom_left_position[1]+box_len*random.random()+box_len*y,
                                bottom_left_position[2]+config["Height"]*random.random()),
                               (bottom_left_position[0]+box_len*random.random()+box_len*x,
                                bottom_left_position[1]+box_len*random.random()+box_len*y,
                                bottom_left_position[2]+config["Height"]*random.random())], config["Speed"]) ) for x in range(nWide) for y in range(nHigh)]

    nodes={}
    j = enumerate(node_positions)
    for num, pos in j:
        name = "%s%03d" % (Prefix,num+1)
        nodes[name]=AcousticNode(acoustic_event, config, name, pos, config["Period"], None, config["Moving"])
    
    return nodes


def ReadConfigFromFile(ConfigFileName):    
    #Reads the configuration file
    config_dict={}
    execfile(ConfigFileName,config_dict)

    return config_dict
