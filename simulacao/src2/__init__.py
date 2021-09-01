###########################################################################
#    Copyright (C) 2007 by Justin Eskesen and Josep Miquel Jornet Montana                                      
#    <jge@mit.edu><jmjornet@mit.edu>                                                             
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

"""
AUVNetSim: a discrete event network simulator for underwater acoustic
environments.  Useful for testing new algorithms for stationary & mobile 
(AUVs) acoustic nodes.

Written and maintained by: Justin Eskesen <jge@mit.edu>
"""

__all__ = ["Simulation", "AcousticNode", "PhysicalLayer", "MAC", "ApplicationLayer", "FSM"]
