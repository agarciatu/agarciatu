#
# Copyright (C) 2016 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, Ice, traceback, time
from PySide import *
from genericworker import *
import matplotlib.pyplot as plt

import networkx as nx

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"GotoPoint.ice")
from RoboCompGotoPoint import *
Ice.loadSlice(preStr+"AprilTags.ice")
from RoboCompAprilTags import *
Ice.loadSlice(preStr+"DifferentialRobot.ice")
from RoboCompDifferentialRobot import *



class SpecificWorker(GenericWorker):
	posiciones = {}
	camino = [10,61]
	estado = 'init'
  
	def __init__(self, proxy_map):
	  super(SpecificWorker, self).__init__(proxy_map)
	  self.timer.timeout.connect(self.compute)
	  self.Period = 2000
	  self.timer.start(self.Period)
	  self.fnodos()
	  self.state =  {
	    'init': self.ini, 
	    'Ti': self.ti, 
	    'Pi': self.pi, 
	    'Go': self.go, 
	    } 


	def setParams(self, params):
	  return True
	      
	@QtCore.Slot()
	def compute(self):
	  print 'SpecificWorker.compute...'
	  self.state[self.estado]()


	 def fnodos(self):
	   fil = open('puntos.txt', 'r')
	   with fil as f:
	     self.g = nx.Graph()
	     for line in f:
	       l = line.split()
	       if l[0]=="N":
		 self.g.add_node(l[1], x = float(l[2]), z = float(l[3]), tipo = l[4])
		 self.posiciones[l[1]] = (float(l[2]), float (l[3]))
		elif line[0] == "E":
		   self.g.add_edge(l[1], l[2])   
	    fil.close()
	    print self.posiciones
	    
	    
	    
	 def nodoCercano(self):
	    bState = RoboCompDifferentialRobot.Bstate()
	    bState = self.differentialrobot_proxy.getBaseState()
	    r = (bState.x , bState.z)
	    dist = lambda r,n: (r[0]-n[0])**2+(r[1]-n[1])**2
	    #funcion que devuele el nodo mas cercano al robot
	    return  sorted(list (( n[0] ,dist(n[1],r)) for n in self.posiciones.items() ), key=lambda s: s[1])[0][0]



	def ini(self):
	  self.estado = 'Ti'
	  
	def ti(self):
	  if len(self.camino)== 0:
	    self.estado = 'init'
	    return
	  
	  self.lNodCercano = nx.shortest_path(self.g, source=str(self.nodoCercano()), target=str(self.camino[0]))
	  self.camino.pop(0)
	  print self.lNodCercano
	  self.estado = 'Pi'
	  
	def pi(self):
	  if len(self.lNodCercano)== 0:
	    self.estado = 'Ti'
	    return
	  self.nodoA = self.lNodCercano[0]
	  self.lNodCercano.pop(0)
	  try:
	    print "Posicion target: ", self.posiciones[self.nodoA][0], self.posiciones[self.nodoA][1]
	    self.gotopoint_proxy.go("",self.posiciones[self.nodoA][0], self.posiciones[self.nodoA][1], 0.3)
	  except Ice.Exception as e:
	    print e
	  self.estado = "Go"
	  
	def go(self):
	  try:
	    if self.gotopoint_proxy.atTarget():
		self.estado = 'Pi'
		return
	  except Ice.Exception as e:
	      print e
	      
	
