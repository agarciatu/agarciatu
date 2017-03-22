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
import networkx as nx
import matplotlib.pyplot as plt
from PySide import *
from genericworker import *

# Variable global
#estado

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
	ruta = [10,61]
	estado = 'init'
	
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.fichero()
		self.state =  {
		  'init': self.initState, 
		  'Ti': self.ti, 
		  'Pi': self.pi, 
		  'Go': self.go, 
		} 

	def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True
	
	def fichero(self):
		fich = open('puntos.txt', 'r')
		with fich as f:
		  self.g=nx.Graph()
		  for line in f:
		    l=line.split()
		    if l[0] == "N":
		      self.g.add_node(l[1], x=float(l[2]), z=float(l[3]), tipo=l[4])
		      self.posiciones[l[1]] = (float(l[2]), float (l[3]))
		    elif line[0] == "E":  
		      self.g.add_edge(l[1], l[2])
		fich.close()
		print self.posiciones
		#img  = plt.imread("plano.png")
		#plt.imshow(img, extent = ([-12284, 25600, -3840, 9023]))
		#nx.draw_networkx_nodes(g, posiciones)
		#nx.draw_networkx_edges(g, posiciones)
		#nx.draw_networkx_labels(g, posiciones)

		#print g.nodes()
		#print g.number_of_nodes()
		#print nx.shortest_path(g, source="3", target="12")
		#nx.draw_networkx(g, self.posiciones)
		#plt.show()

        

	def nodoCercano(self):
	    bState = TBaseState()
	    bState= self.differentialrobot_proxy.getBaseState()
	    r = (bState.x , bState.z)
	    dist = lambda r,n: (r[0]-n[0])**2+(r[1]-n[1])**2
	    #funcion que devuele el nodo mas cercano al robot
	    return  sorted(list (( n[0] ,dist(n[1],r)) for n in self.posiciones.items() ), key=lambda s: s[1])[0][0]

	  
	def initState(self):
	  print "Init"
	  self.estado = 'Ti'
	    
	def ti(self):
	  if len(self.ruta)== 0:
	    self.estado = 'init'
	    return
	  
	  print "Ti --> Nodo cercano."
	  self.listan = nx.shortest_path(self.g, source=str(self.nodoCercano()), target=str(self.ruta[0]))
	  self.ruta.pop(0)
	  print self.listan
	  self.estado = 'Pi'
	  
	def pi(self):
	  print "Pi"
	  if len(self.listan)== 0:
	    self.estado = 'Ti'
	    return
	  self.nodoA = self.listan[0]
	  print "Nodo actual" + str(self.nodoA) 
	  self.listan.pop(0)
	  try:
	    print "Posicion target: ", self.posiciones[self.nodoA][0], self.posiciones[self.nodoA][1]
	    self.gotopoint_proxy.go("",self.posiciones[self.nodoA][0], self.posiciones[self.nodoA][1], 0.3)
	    
	  except Ice.Exception as e:
	    print e
	    
	  self.estado = "Go"
	  
	def go(self):
	  print "Go"
	  try:
	    if self.gotopoint_proxy.atTarget():
		self.estado = 'Pi'
		print "Nodo alcanzado", self.nodoA
		return
	      
	  except Ice.Exception as e:
	      print e
		    
	@QtCore.Slot()
	def compute(self):
	  self.state[self.estado]()