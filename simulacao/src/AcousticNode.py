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
import MAC
from PhysicalLayer import PhysicalLayer
from RoutingLayer import SetupRouting
from MAC import SetupMAC
from ApplicationLayer import ApplicationLayer
import random

import random, math

class Position(tuple):
    """A simple class to encapsulate a cartesian coordinate system.
    """
    
    # Computes distance between two positions
    def distanceto(self, OtherPosition):
        assert len(OtherPosition) == 3
        return math.sqrt((self[0]-OtherPosition[0])**2 + (self[1]-OtherPosition[1])**2 + (self[2]-+OtherPosition[2])**2)


    # Only valid for 2D scenarios - Computes angle between two positions
    def anglewith(self, OtherPosition):
        assert len(OtherPosition) == 3
        angle = math.degrees(math.asin((OtherPosition[1]-self[1])/self.distanceto(OtherPosition)))
        if OtherPosition[0]-self[0] < 0:
            if OtherPosition[1]-self[1] > 0:
                angle = 180 - angle
            else:
                angle = -180 - angle
            
        return angle
            
    # Find a point within the path followed by the AUV
    def find_point_between(self, OtherPosition, fraction):
        x = self[0]*(1-fraction) + OtherPosition[0]*fraction
        y = self[1]*(1-fraction) + OtherPosition[1]*fraction
        z = self[2]*(1-fraction) + OtherPosition[2]*fraction
        pos_tuple = (x,y,z)
        return Position(pos_tuple)


    def __str__(self):
        return "(%5.1f,%5.1f,%5.1f)" % self

class Path():
    """Simulates mobile node (auv) motion between a series of points
    """
    
    def __init__(self, waypoint_arr, speed=0.0):
        self.current=0
        self.speed = speed
    
        positions = []
        for p in waypoint_arr:
            positions.append(Position(p))
        
        if speed == 0.0:
            self.waypoints = [(positions[0], 0.0)]
            return
        
        times = [0.0]
        for i in range(1,len(positions)):
            times.append((positions[i].distanceto(positions[i-1])/self.speed)+times[i-1])
        
        self.waypoints = zip(positions, times)
        

    def get_initial_position(self):
        return self.waypoints[0][0]
        

    def get_position_at_time(self, t):

        if t >= self.waypoints[-1][1]:
            current_pos = self.waypoints[-1][0]
            return Position(current_pos)
        
        #Advance to the next point?
        while self.waypoints[self.current+1][1] < t:
        #If the end of the path has been reached, stop
            if self.current == len(self.waypoints)-1:
                return self.waypoints[self.current][0]
            else:
                self.current += 1
        
        #Computes how far along on the path to the next point we are
        progress = (t-self.waypoints[self.current][1]) / (self.waypoints[self.current+1][1]-self.waypoints[self.current][1])
        return self.waypoints[self.current][0].find_point_between(self.waypoints[self.current+1][0], progress)


#####################################################################
# Acoustic Node class
#####################################################################

class AcousticNode():
        
    def __init__(self, event, config, label, position_or_path, period=None, destination=None, involuntary_moving=False):
        """Initialization of the acoustic node.
        """
        self.config = config       
        self.name = label
        self.SetupPath(position_or_path)
        self.involuntary_moving = involuntary_moving
        
        #Physical layer
        self.physical_layer = PhysicalLayer(self, config["PHY"], event)
        
        #MAC Layer
        self.MACProtocol = SetupMAC(self, config["MAC"])
        
        #Routing Layer
        self.routing_layer = SetupRouting(self, config["Routing"])
        
        #Application Layer
        self.app_layer = ApplicationLayer(self)
        if(period is not None):
            self.SetUpPeriodicTransmission(period, destination)            


    def SetupPath(self, pos_or_path):
        """SetupPath: parse the input as a position or a path,and setup the appropriate path structure.
        """
        assert(isinstance(pos_or_path, (tuple,list)))
        if len(pos_or_path)==2 and isinstance(pos_or_path[0], (tuple,list)):
            # This is a mobile node's path
            self.path = Path(pos_or_path[0], pos_or_path[1])
        elif(len(pos_or_path)==3):
            # This is a static node's position
            self.path = Path((pos_or_path,), 0.0)
        else:
            raise ValueError("Input type is not recoginized as a path or position")
        
        
    def SetPath(self, waypoints, speed):
        #This function is used to set a mobile node 
        self.path = Path(waypoints, speed)  
    

    def GetCurrentPosition(self):
        if not self.involuntary_moving:
            # Returns the real position of the nodes, which are not involuntary moving
            return self.path.get_position_at_time(Sim.now())
        else:
            # Returns the initial position, the only one that nodes may know if they are involuntary moving
            return self.path.get_initial_position()


    def GetRealCurrentPosition(self):
        # Returns the real position, which may show the effect of involuntary drifting if that's the case
        return self.path.get_position_at_time(Sim.now())
        

    def SetPosition(self, pos):
        #A path which is only one point, not moving. This is a static node.
        self.path = Path((pos,), 0.0)
    

    def SetUpPeriodicTransmission(self, period, destination):
        random_delay = random.random()*period
        Sim.activate(self.app_layer, self.app_layer.PeriodicTransmission(period, destination), delay=random_delay)


    def PrintStats(self):
        print self.app_layer
        print self.physical_layer
