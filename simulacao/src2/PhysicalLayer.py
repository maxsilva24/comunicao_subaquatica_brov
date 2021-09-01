# -*- coding: cp1252 -*-
###########################################################################
#
#    Copyright (C) 2007 by Justin Eskesen and Josep Miquel Jornet Montana                                     
#    <jge@mit.edu> <jmjornet@mit.edu>
#
# Copyright: See COPYING file that comes with this distribution
#
# This file is part of AUVNetSim, a library for simulating acoustic
# networks of static and mobile underwater nodes, written in Python.
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
import math

from copy import deepcopy

class PhysicalLayer():

    # Initialization of the layer
    def __init__(self, node, config, channel_event):
        self.node = node

        ## Parameters initialization

        # Frequency Specifications
        self.freq = node.config["Frequency"]                    #In kHZ
        self.bandwidth = node.config["BandWidth"]               #In kHz
        self.band2bit = node.config["BandwidthBitrateRelation"] #In bps/Hz
        self.variable_bandwidth = config["variableBandwidth"]   #True or False

        # Power Specifications
        self.transmit_power = node.config["TransmitPower"]      #In dB re uPa - The one when power control is not used
        self.max_output_power = node.config["TransmitPower"]    #In dB re uPa - To prevent unreal power values if power control does not work properly
        self.receive_power = node.config["ReceivePower"]        #In dBW
        self.listen_power = node.config["ListenPower"]          #In dBW
        self.variable_power = config["variablePower"]           #True or False (if false = only one power level)
        self.level2distance = config["level2distance"]          #Levels defined in terms of distance (m)
        
        # Detection Specifications
        self.SIR_threshold = config["SIRThreshold"]             #Minimum signal to interference ratio to properly receive a packet
        self.SNR_threshold = config["SNRThreshold"]             #Minimum signal to noise ratio to properly receove a packet
        self.LIS_threshold = config["LISThreshold"]             #Minimum signal to noise ratio to properly detect a packet

        self.receiving_threshold = DB2Linear(ReceivingThreshold(self.freq, self.bandwidth, self.SNR_threshold))
        self.listening_threshold = DB2Linear(ListeningThreshold(self.freq, self.bandwidth, self.LIS_threshold))

        ## Definition of system parameters
        self.ambient_noise = DB2Linear( Noise(self.freq) + 10*math.log10(self.bandwidth*1e3) ) #In linear scale
        self.interference = self.ambient_noise                  #Initially, interference and noise are the same
        self.collision = False                                  #Emulates collision detection

        self.modem = Sim.Resource(name="a_modem")               #To emulate the half-duplex operation of the modem

        self.transducer = Transducer(self, self.ambient_noise, channel_event, self.node.GetRealCurrentPosition, self.SIR_threshold, self.OnSuccessfulReceipt)

        self.messages = []
        self.event = Sim.SimEvent("TransducerEvent" + self.node.name)

        ## Values used to take statistics
        self.max_output_power_used = 0                          #In dB - Monitors the maximum transmission power used
        self.distance2power = {}                                #Levels defined in terms of power (dB re uPa)
        for level, distance in self.level2distance.iteritems():
            self.distance2power[level] = DB2Linear(distance2Intensity(self.bandwidth, self.freq, distance, self.SNR_threshold))
        
        self.tx_energy = 0                                      #Energy consumed so far when tx (Joules) 
        self.rx_energy = 0                                      #Energy consumed so far when rx (Joules)


    # Before transmissting, we should check if the system is idle or not
    def IsIdle(self):
        if len(self.transducer.activeQ)>0:
            self.PrintMessage("The channel is not idle. Currently receiving: "+str(len(self.transducer.activeQ))+" packet(s).")            
            return False

        return True


    # Funtion called from the layers above. The MAC layer needs to transmit a packet
    def TransmitPacket(self, packet):

        if self.IsIdle()==False:
            self.PrintMessage("I should not do this... the channel is not idle!") # The MAC protocol is the one that should check this before transmitting

        self.collision = False

        if self.variable_power:
            distance = self.level2distance[packet["level"]]
            power = distance2Intensity(self.bandwidth, self.freq, distance, self.SNR_threshold)
        else:
            power = self.transmit_power

        if power > self.max_output_power_used:
            self.max_output_power_used = power

        if power > self.max_output_power:
            power = self.max_output_power

        new_transmission = OutgoingPacket(self)
        Sim.activate(new_transmission, new_transmission.transmit(packet, power))


    # Checks if there has been a collision
    def CollisionDetected(self):
        return self.collision


    # When a packet has been received, we should pass it to the MAC layer
    def OnSuccessfulReceipt(self, packet):
        self.node.MACProtocol.OnNewPacket(packet)
        

    # Shows information on screen
    def PrintMessage(self, msg):
        pass
        #print ("PHY (%s): %s" % (self.node.name, msg))

    def __str__(self):
        return "I'm "+ self.node.name    
        

class Transducer(Sim.Resource):
    """Transducer:  Uses a SimPy Resource model.
    --Incoming packets (from the channel): can request the resource
    and are allowed to interfere with one another.
    
    --Outgoing packets (coming from the modem): get queued by the modem until
    any outgoing transmission is completed.
    """
    
    def __init__(self, physical_layer, ambient_noise, channel_event, position_query, SIR_thresh, on_success, name="a_transducer"):
        # A resource with large capacity, because we don't want incoming messages to queue,
        # We want them to interfere.

        Sim.Resource.__init__(self, name=name, capacity=1000)
        
        self.physical_layer = physical_layer
        
        # Interference is initialized as ambient noise
        self.interference = ambient_noise
        
        # Setup our event listener
        self.channel_event = channel_event
        self.listener = AcousticEventListener(self)
        Sim.activate(self.listener, self.listener.listen(self.channel_event, position_query))
        
        # Function to call when we've received a packet
        self.on_success = on_success

        # SIR threshold
        self.SIR_thresh = SIR_thresh

        # Controls the half-duplex behavior
        self.transmitting = False

        # Takes statistics about the collisions
        self.collisions = []

    
    # Override SimPy Resource's "_request" function to update SIR for all incoming messages.
    def _request(self, arg):        
        # We should update the activeQ
        Sim.Resource._request(self, arg)
        
        # "arg[1] is a reference to the IncomingPacket instance that has been just created
        new_packet = arg[1]

        # Doom any newly incoming packets to failure if the transducer is transmitting
        if self.transmitting:
            new_packet.Doom()

        # Update all incoming packets' SIR
        self.interference += new_packet.power

        [i.UpdateInterference(self.interference, new_packet.packet) for i in self.activeQ]
   
    
    # Override SimPy Resource's "_release" function to update SIR for all incoming messages.
    def _release(self, arg):
        # "arg[1] is a reference to the IncomingPacket instance that just completed
        doomed = arg[1].doomed
        minSIR = arg[1].GetMinSIR()
        new_packet = deepcopy(arg[1].packet)

        # Reduce the overall interference by this message's power
        self.interference -= arg[1].power
        # Prevent rounding errors
        if self.interference<=0:
            self.interference = self.physical_layer.ambient_noise

        # Delete this from the transducer queue by calling the Parent form of "_release"
        Sim.Resource._release(self, arg)        

        # If it isn't doomed due to transmission & it is not interfered
        if minSIR>0:
            if not doomed and Linear2DB(minSIR) >= self.SIR_thresh and arg[1].power >= self.physical_layer.receiving_threshold:
                # Properly received: enough power, not enough interference
                self.collision = False                
                self.on_success(new_packet)

            elif arg[1].power >= self.physical_layer.receiving_threshold:
                # Too much interference but enough power to receive it: it suffered a collision                
                if self.physical_layer.node.name == new_packet["through"] or self.physical_layer.node.name == new_packet["dest"]:
                     self.collision = True                    
                     self.collisions.append(new_packet)
                     self.physical_layer.PrintMessage("A "+new_packet["type"]+" packet to "+new_packet["through"]
                                                                 +" was discarded due to interference.")               
            else:
                # Not enough power to be properly received: just heard.
                self.physical_layer.PrintMessage("This packet was not addressed to me.")

        else:
            # This should never appear, and in fact, it doesn't, but just to detect bugs (we cannot have a negative SIR in lineal scale).
            print (new_packet["type"], new_packet["source"], new_packet["dest"], new_packet["through"], self.physical_layer.node.name)


    def OnTransmitBegin(self):
        self.transmitting = True
        # Doom all currently incoming packets to failure.
        [i.Doom() for i in self.activeQ]
        
    
    def OnTransmitComplete(self):
        self.transmitting = False


class IncomingPacket(Sim.Process):
    """IncomingPacket: A small class to represent a message being received
        by the transducer.  It keeps track of signal power as well as the
        maximum interference that occurs over its lifetime (worst case scenario).
        Powers are linear, not dB.
    """
    
    def __init__(self, power, packet, physical_layer):
        Sim.Process.__init__(self, name="ReceiveMessage: "+str(packet))
        self.power = power
        self.packet = packet
        self.physical_layer = physical_layer
        self.doomed = False
        self.MaxInterference = 1


    def UpdateInterference(self, interference, packet):
        if(interference > self.MaxInterference):               
            self.MaxInterference = interference


    def Receive(self, duration):
        if self.power >= self.physical_layer.listening_threshold:
            # Otherwise I will not even notice that there are packets in the network
            yield Sim.request, self, self.physical_layer.transducer
            yield Sim.hold, self, duration
            yield Sim.release, self, self.physical_layer.transducer

            # Even if a packet is not received properly, we have consumed power
            self.physical_layer.rx_energy += DB2Linear(self.physical_layer.receive_power) * duration
                

    def GetMinSIR(self):
        return self.power/(self.MaxInterference-self.power+1)


    def Doom(self):
        self.doomed = True


class OutgoingPacket(Sim.Process):
    """OutgoingPacket: A small class to represent a message being transmitted
        by the transducer.  It establishes the packet duration according to
        the bandwidth. Powers are in linear scale, not in dB.
    """
    
    def __init__(self, physical_layer):
        Sim.Process.__init__(self)
        self.physical_layer = physical_layer
        

    def transmit(self, packet, power):
        yield Sim.request, self, self.physical_layer.modem

        # Create the acoustic event
        if self.physical_layer.variable_bandwidth:
            distance = self.physical_layer.level2distance[packet["level"]]
            bandwidth = distance2Bandwidth(power, self.physical_layer.freq, distance, self.physical_layer.SNR_threshold)
        else:
            bandwidth = self.physical_layer.bandwidth

        # Real bit-rate
        bitrate = bandwidth*1e3*self.physical_layer.band2bit
        duration = packet["length"]/bitrate
           
        self.physical_layer.transducer.channel_event.signal({"pos": self.physical_layer.node.GetRealCurrentPosition(),
                                                             "power": power, "duration": duration, "frequency": self.physical_layer.freq,
                                                             "packet": packet})

        # Hold onto the transducer for the duration of the transmission
        self.physical_layer.transducer.OnTransmitBegin()

        yield Sim.hold, self, duration
        self.physical_layer.transducer.OnTransmitComplete()
        
        # Release the modem when done
        yield Sim.release, self, self.physical_layer.modem

        power_w = DB2Linear(AcousticPower(power))
        self.physical_layer.tx_energy += (power_w * duration)
        
    
#####################################################################
# Acoustic Event Listener
#####################################################################

class AcousticEventListener(Sim.Process):
    """AcousticEventListener: No physical analog.
    Waits for another node to send something and then activates
    an Arrival Scheduler instance.
    """
    def __init__(self, transducer):
        Sim.Process.__init__(self)
        self.transducer = transducer
        
    def listen(self, channel_event, position_query):
        while(True):
            yield Sim.waitevent, self, channel_event
            params = channel_event.signalparam
            sched = ArrivalScheduler(name="ArrivalScheduler"+self.name[-1])
            Sim.activate(sched, sched.schedule_arrival(self.transducer, params, position_query()))


#####################################################################
# Arrival Scheduler
#####################################################################

class ArrivalScheduler(Sim.Process):
    """ArrivalScheduler class: simulates the transit time of a message
    """
    
    def schedule_arrival(self, transducer, params, pos):
        distance = pos.distanceto(params["pos"])
        
        if distance > 0.01:  # I should not receive my own transmissions
            receive_power = params["power"] - Attenuation(params["frequency"], distance)
            travel_time = distance/1482.0  # Speed of sound in water = 1482.0 m/s

            yield Sim.hold, self, travel_time

            new_incoming_packet = IncomingPacket(DB2Linear(receive_power), params["packet"], transducer.physical_layer)
            Sim.activate(new_incoming_packet,new_incoming_packet.Receive(params["duration"]))

#####################################################################
# Propagation functions
#####################################################################

def Attenuation(f, d):
    """Attenuation(P0,f,d)
    
    Calculates the acoustic signal path loss
    as a function of frequency & distance
    
    f - Frequency in kHz
    d - Distance in m
    """
    
    f2 = f**2
    k = 1.5
    DistanceInKm = d/1000
    
    # Thorp's formula for attenuation rate (in dB/km) -> Changes depending on the frequency
    if f > 1:
        absorption_coeff = 0.11*(f2/(1+f2)) + 44.0*(f2/(4100+f2)) + 0.000275*f2 + 0.003
    else:
        absorption_coeff = 0.002+0.11*(f2/(1+f2)) + 0.011*f2

    return k*Linear2DB(d) + DistanceInKm*absorption_coeff


def Noise(f):
    """Noise(f)

    Calculates the ambient noise at current frequency

    f - Frequency in kHz
    """

    # Noise approximation valid from a few kHz
    return 50 - 18*math.log10(f)


def distance2Bandwidth(I0, f, d, SNR):
    """distance2Bandwidth(P0, A, N, SNR)

    Calculates the available bandwidth for the acoustic signal as
    a function of acoustic intensity, frequency and distance

    I0 - Transmit power in dB
    f - Frequency in kHz
    d - Distance to travel in m
    SNR - Signal to noise ratio in dB
    """

    A = Attenuation(f,d)
    N = Noise(f)

    return DB2Linear(I0-SNR-N-A-30) #In kHz


def distance2Intensity(B, f, d, SNR):
    """distance2Power(B, A, N, SNR)

    Calculates the acoustic intensity at the source as
    a function of bandwidth, frequency and distance

    B - Bandwidth in kHz
    f - Frequency in kHz
    d - Distance to travel in m
    SNR - Signal to noise ratio in dB
    """

    A = Attenuation(f,d)
    N = Noise(f)
    B = Linear2DB(B*1.0e3)

    return SNR + A + N + B


def AcousticPower(I):
    """AcousticPower(P, dist)

    Calculates the acoustic power needed to create an acoustic intensity at a distance dist

    I - Created acoustic pressure
    dist - Distance in m
    """
    # 170dB re uPa is the sound intensity created over a sphere of 1m by a radiated acoustic power of 1 Watt with the source in the center
    I_ref = 172.0
    return I-I_ref


def ListeningThreshold(f, B, minSNR):
    """ReceivingThreshold(f, B)

    Calculates the minimum acoustic intensity that a node may be able to hear

    B - Bandwidth in kHz
    f - Frequency in kHz
    minSNR - Signal to noise ratio in dB
    """

    N = Noise(f)
    B = Linear2DB(B*1.0e3)

    return minSNR + N + B


def ReceivingThreshold(f, B, SNR):
    """ReceivingThreshold(f, B)

    Calculates the minimum acoustic intensity that a packet should have to be properly received

    B - Bandwidth in kHz
    f - Frequency in kHz
    SNR - Signal to noise ratio in dB
    """

    N = Noise(f)
    B = Linear2DB(B*1.0e3)

    return SNR + N + B


def DB2Linear(dB):
    return 10.0**(dB/10.0)


def Linear2DB(Linear):
    return 10.0*math.log10(Linear+0.0)

if __name__== '__main__':
    print('teste')