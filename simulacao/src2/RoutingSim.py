#!/usr/bin/env python

###########################################################################
#    Copyright (C) 2007 by Justin Eskesen and Josep Miquel Jornet Montana                                     
#    <jge@mit.edu> <jmjornet@mit.edu>                                                            
#
# Copyright: See COPYING file that comes with this distribution
#
###########################################################################

import Simulation  as AUVSim
# import Simulation as AUVSim
import sys, random
from matplotlib import pylab

def RoutingSimulation():
    random.seed()
    
    def GenerateRandomPeriodicTransmit(min, max):
        return random.random()*(max-min) + min
    
    if(len(sys.argv) < 2):
        print ("usage: ", sys.argv[0], "ConfigFile")
        exit(1)
    
    config = AUVSim.ReadConfigFromFile(sys.argv[1])

    nodes=[]
    for j in range(0,1):
        print ("Running simulation")
        nodess = AUVSim.RunSimulation(config)
        for x in nodess:
            nodes.append(nodess[x])
        print ("Done" )   

    energy, packets, delay, col = GetStat(nodes)

    print ("Energy consumption: ",energy)
    print( "Total number of packets transmitted: ",packets)
    print ("Average end to end delay: ",delay)
    print ("Collisions: ",col)


    PlotScenario3D(nodes, config)
    ##    PlotConsumption(nodes)
    ##    PlotDelay(nodes)
    ##    PlotDelayHist(nodes)    
    pylab.show()

    
def ROU():
    random.seed()
    
    def GenerateRandomPeriodicTransmit(min, max):
        return random.random()*(max-min) + min
    
    if(len(sys.argv) < 2):
        print ("usage: ", sys.argv[0], "ConfigFile")
        exit(1)
    
    config = AUVSim.ReadConfigFromFile(sys.argv[1])

    period = []
    energy = []
    packets = []
    delay = []
    offered_load = []
    throughput = []
    to = []
    col = []
    output_power = []
    t_data = []

    for j in range(0,1):
        filename = 'sim_N_'+str(config["NodeField"][1])+'_f_'+str(config["Frequency"])+'_b_'+str(config["BandWidth"])+'_'+config["MAC"]["protocol"]+'_'+config["Routing"]["Algorithm"]+'.txt'
        f = open(filename,'w')

        period.append([])
        energy.append([])
        packets.append([])
        delay.append([])
        offered_load.append([])
        throughput.append([])
        to.append([])
        col.append([])
        output_power.append([])
        t_data.append([])

        for i in range(0,1):
            period[j].append(config["Period"])
            energy_s = []
            packets_s = []
            delay_s = []            
            col_s = []
            offered_load_s = []
            throughput_s = []
            to_s = []

            output_power_s = []
            t_data_s = []
            
            for z in range(0,10):
                nodess = AUVSim.RunSimulation(config)
                nodes=[]
                for x in nodess:
                    nodes.append(nodess[x])                

                energy_ss, packets_ss, delay_ss, col_ss = GetStat(nodes)
                offered_load_ss, throughput_ss = Throughput(nodes, config["SimulationDuration"])
                energy_s.append(energy_ss)
                packets_s.append(packets_ss)
                delay_s.append(delay_ss)
                col_s.append(col_ss)
                offered_load_s.append(offered_load_ss)
                throughput_s.append(throughput_ss)
                to_s.append(throughput_ss/offered_load_ss)

                output_power_ss = []
                for x in nodes:
                    if x.physical_layer.max_output_power!=0:
                        output_power_ss.append(x.physical_layer.max_output_power_used)

                output_power_s.append(max(output_power_ss))
                t_data_s.append(x.MACProtocol.t_data)

            energy[j].append(Average(energy_s))
            packets[j].append(Average(packets_s))
            delay[j].append(Average(delay_s))
            col[j].append(Average(col_s))
            throughput[j].append(Average(throughput_s))
            offered_load[j].append(Average(offered_load_s))
            to[j].append(Average(to_s))
            output_power[j].append(max(output_power_s))
            t_data[j].append(Average(t_data_s))

            # File recordings
            record = str(config["Period"])+', '+str(delay[j][i])+', '+str(energy[j][i])+', '+str(offered_load[j][i])
            record = record+', '+str(throughput[j][i])+', '+str(col[j][i])+', '+str(packets[j][i])+', '+str(output_power[j][i])+'\n'
            f.write(record)

        print (config["PHY"]["level2distance"], config["Frequency"], config["BandWidth"])
        f.close() 


# Returns the average tx level used
def GetLevel(nodes):
    lev = []
    for x in nodes:
        if len(x.routing_layer.levels_used):
            lev.append(Average(x.routing_layer.levels_used))

    #print ("Average of the output power level covering:", Average(lev), "m.")
    return Average(lev)


# Obtains the average delay and energy consumption per consumption
def GetStat(nodes):
    packets = 0
    delay = []
    energy_vec = []
    collisions = 0
    
    for x in nodes:
        if x.physical_layer.tx_energy+x.physical_layer.rx_energy!=0:
            energy_vec.append(x.physical_layer.tx_energy+x.physical_layer.rx_energy)

        if len(x.app_layer.packets_time)!=0:
            packets+=len(x.app_layer.packets_time)
            delay.append(Average(x.app_layer.packets_time))

        if len(x.physical_layer.transducer.collisions)!=0:
            collisions+=len(x.physical_layer.transducer.collisions)

    energy = sum(energy_vec)
    delay = Average(delay)

    return energy, packets, delay, collisions

    
# Computes the network throughput
def Throughput(nodes, simtime):
    generated_packets = 0
    received_packets = 0
    delay = []
    hops = []
    
    for x in nodes:
        generated_packets+=x.app_layer.packets_sent
        received_packets+=len(x.app_layer.packets_time)

        if len(x.app_layer.packets_time)!=0:
            delay.append(Average(x.app_layer.packets_time))
            hops.append(Average(x.app_layer.packets_hops))

    t_data = nodes[0].MACProtocol.t_data

    throughput = received_packets*t_data/simtime*Average(hops)
    offeredload = generated_packets*t_data/simtime*Average(hops)

    ##    print "The throughput achieved for an offered load of", offeredload, "is", throughput
    return offeredload, throughput

# Plots the scenario where the nodes are currently located. Size is proportional to transmited energy, color changes with reception power
def PlotScenario(nodes):
    tx_energy_vec = []
    rx_energy_vec = []
    xpos = []
    ypos = []
    
    pylab.figure()
    pylab.hold(True)

    for x in nodes:
        xpos.append(x.GetCurrentPosition()[0])
        ypos.append(x.GetCurrentPosition()[1])

        tx_energy_vec.append(x.physical_layer.tx_energy)
        rx_energy_vec.append(x.physical_layer.rx_energy)

        if x.physical_layer.tx_energy > 1.0 or x.name[0:4] == "Sink":
            pylab.text(x.GetCurrentPosition()[0],x.GetCurrentPosition()[1],x.name)

    Normalize(tx_energy_vec, max(tx_energy_vec))
    Normalize(rx_energy_vec, max(rx_energy_vec))
    
    pylab.scatter(xpos, ypos, s=tx_energy_vec, c=rx_energy_vec)
    pylab.xlabel('X(m)')
    pylab.ylabel('Y(m)')
    pylab.title('Scenario')


# Plots the scenario in 3D axis
def PlotScenario3D(nodes, config):
    import matplotlib.axes3d as p3

    fig=pylab.figure()
    ax = p3.Axes3D(fig)

    filename = 'Positions_'+config["MAC"]["protocol"]+'_'+str(config["Routing"]["Algorithm"])+'.txt'
    f = open(filename,'w')
    filename = 'Routes_'+config["MAC"]["protocol"]+'_'+str(config["Routing"]["Algorithm"])+'.txt'
    ff = open(filename,'w')
    ff.write('-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n')

    positions=[]
    tx_energy_vec = []
    rx_energy_vec = []    
    
    for node in nodes:
        pos=node.GetCurrentPosition()
        positions.append(pos)       
        tx_energy_vec.append(node.physical_layer.tx_energy)
        rx_energy_vec.append(node.physical_layer.rx_energy)
        record = str(pos[0])+', '+str(pos[1])+', '+str(pos[2])+', '+str(node.physical_layer.tx_energy)+', '+str(node.physical_layer.rx_energy)+'\n'
        f.write(record)

        for log_line in node.app_layer.log:
            for i in log_line["route"]:
                route = i[1]
                ff.write(str(route[0])+', '+str(route[1])+', '+str(route[2])+', ')
            ff.write(str(pos[0])+', '+str(pos[1])+', '+str(pos[2])+', -1\n')

            route = [i[1] for i in log_line["route"]]
            route.append(pos)
            rx,ry,rz = ReorderPositions(route)
            ax.plot3d(rx,ry,rz)

        pylab.text(node.GetCurrentPosition()[0],node.GetCurrentPosition()[1],node.name)

    f.close()
    Normalize(tx_energy_vec, max(tx_energy_vec))
    Normalize(rx_energy_vec, max(rx_energy_vec))

    nx,ny,nz = ReorderPositions(positions)
    ax.scatter3d(nx,ny,nz,s=tx_energy_vec, c=rx_energy_vec)

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.axis("equal")
    

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


# Plots the energy consumption per node both in reception and in transmission
def PlotConsumption(nodes):
    tx_energy_vec = []
    rx_energy_vec = []

    pylab.figure()

    for x in nodes:
        tx_energy_vec.append(x.physical_layer.tx_energy)
        rx_energy_vec.append(x.physical_layer.rx_energy)

    N = len(tx_energy_vec)
    ind = pylab.arange(N)   # the x locations for the groups
    width = 1               # the width of the bars
    p1 = pylab.bar(ind, tx_energy_vec, width, color='r')
    p2 = pylab.bar(ind, rx_energy_vec, width, color='g', bottom=tx_energy_vec)

    pylab.ylabel('Energy (J)')
    pylab.title('Power Consumption')

    pylab.legend( (p1[0], p2[0]), ('Tx', 'Rx') )


# Plots the histogram of the consumed energy all over the network
def PlotConsumptionHist(nodes):
    tx_energy_vec = []
    rx_energy_vec = []
    energy_vec = []

    for x in nodes:
        if x.physical_layer.tx_energy!=0:
            tx_energy_vec.append(x.physical_layer.tx_energy)        
        rx_energy_vec.append(x.physical_layer.rx_energy)
        energy_vec.append(x.physical_layer.tx_energy+x.physical_layer.rx_energy)

    energy = Average(energy_vec)
    print ("Network total energy consumption:", sum(tx_energy_vec), "and average consumption per active node:", energy, "J." )
    return energy

    pylab.figure()    
    pylab.hist(tx_energy_vec,20);
    pylab.xlabel("Energy (J)");
    pylab.ylabel("Number of nodes");
    title = "Transmitting energy consumption"
    pylab.title(title)

    pylab.figure()
    pylab.hist(rx_energy_vec,20);    
    pylab.xlabel("Energy (J)");
    pylab.ylabel("Number of nodes");
    title = "Receiving energy consumption"
    pylab.title(title)


# Plot the total delay and the number of hops for each packet in each node
def PlotDelay(nodes):
    av = []
    name = []
    pylab.figure()
    width = 0.5
    
    for x in nodes:
        if len(x.app_layer.packets_time)!=0:
            av.append(Average(x.app_layer.packets_time))
            name.append(x.name)

    ind = pylab.arange(len(av))
    pylab.bar(ind, av, width)
    pylab.xticks(ind+width/2, name)
    

# Plot an histogram for each sink showing packets' delay
def PlotDelayHist(nodes):
    packets = 0
    delay = []
    
    for x in nodes:
        if len(x.app_layer.packets_time)!=0:
            print ("Total number of packets received at", x.name, ":", len(x.app_layer.packets_time))
            packets+=len(x.app_layer.packets_time)
            delay.append(Average(x.app_layer.packets_time))

            print (x.app_layer.packets_time)
            pylab.figure()
            pylab.hist(x.app_layer.packets_time,100);
            pylab.xlabel("End to End delay (s)");
            pylab.ylabel("Number of Packets");
            title = "End to End Delay for packets received at "+x.name
            pylab.title(title)

            pylab.figure()
            pylab.hist(x.app_layer.packets_dhops,10);
            pylab.xlabel("Delay per hop (s)");
            pylab.ylabel("Number of Packets");
            title = "Delay per hop for packets received at "+x.name
            pylab.title(title)

    print ("Average delay per packet in the network:", Average(delay), "s.")
    return packets, delay
    
# Normalizes energy in a scale from 10 to 1000 to achieve proper plots    
def Normalize(x, y):
    for i in range(0,len(x)):
        x[i]=x[i]/y*1000+50

# Computes the average value of vector x
def Average(x):
    av = 0
    for i in range(0,len(x)):
        av+=x[i]

    return av/len(x)

if __name__ == "__main__":
    RoutingSimulation()
##    ROU()
