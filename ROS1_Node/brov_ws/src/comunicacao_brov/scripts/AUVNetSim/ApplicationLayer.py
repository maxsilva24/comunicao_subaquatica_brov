###########################################################################
#    Copyright (C) 2007 by Justin Eskesen and Josep Miquel Jornet Montana                                      
#    <jge@mit.edu> <jmjornet@mit.edu>                                                             
#
# Copyright: See COPYING file that comes with this distribution
#
###########################################################################
import SimPy.Simulation as Sim
from numpy import *
from numpy.random import *

print_log_console = True #
class ApplicationLayer(Sim.Process):
    def __init__(self, node):
        Sim.Process.__init__(self)
        self.node = node
        self.packets_sent = 0
        self.packets_received = {}
        self.packets_time = []
        self.packets_hops = []
        self.packets_dhops = []        
        self.log = []
        
    def PeriodicTransmission(self, period, destination):
        while True:
            self.packets_sent+=1
            packet_ID =  self.node.name+str(self.packets_sent)

            if destination==None:
                destination = "AnySink"

            packet = {"ID": packet_ID, "dest": destination, "source": self.node.name, "route": [], "type": "DATA", "initial_time": Sim.now(), "length": self.node.config["DataPacketLength"]}
            self.node.routing_layer.SendPacket(packet)
            next = poisson(period)
            yield Sim.hold, self, next

            
    def OnPacketReception(self, packet):
        self.log.append(packet)
        origin = packet["route"][0][0]
        if origin in self.packets_received.keys():
            self.packets_received[origin]+=1
        else:
            self.packets_received[origin]=1

        delay = Sim.now()-packet["initial_time"]
        hops = len(packet["route"])

        self.PrintMessage("Packet "+packet["ID"]+" received over "+str(hops)+" hops with a delay of "+str(delay)+"s (delay/hop="+str(delay/hops)+").")

        self.packets_time.append(delay)
        self.packets_hops.append(hops)
        self.packets_dhops.append(delay/hops)
        
        
    def __str__(self):
        return "Packets Sent: %d\n" % (self.packets_sent)


    def PrintMessage(self, msg):
        if print_log_console:
            print ("APP (%s): %s" % (self.node.name, msg))        
