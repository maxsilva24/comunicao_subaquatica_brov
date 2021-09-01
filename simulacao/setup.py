###########################################################################
#    Copyright (C) 2007 by Justin Eskesen                                      
#    <jge@mit.edu>                                                             
#
# Copyright: See COPYING file that comes with this distribution
#
###########################################################################

# from distutils.core import setup, find
from setuptools import setup, find_packages

setup(name='AUVNetSim2',
      version='0.1',
      description='Discrete Event Simulator for Acoustic Networks of Stationary & Mobile Nodes',
      author='Justin Eskesen',
      author_email='jge@mit.edu',
      url='http://scripts.mit.edu/~jge/auvwiki/index.php?title=AUVNetSim',
      license='GNU GPL',
      # packages=['AUVNetSim'],
      packages=find_packages(),
      package_dir={'AUVNetSim' : 'src'}
     )

