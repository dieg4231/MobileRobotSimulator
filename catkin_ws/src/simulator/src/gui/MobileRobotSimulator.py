#################################################
#						#
#	MobileRobotSimulator.py			#
#						#
#		Diego Cordero			#
#						#
#	Biorobotics Laboratory			#
# 		UNAM-2019			#
#						#
################################################# 

import rospkg
from Tkinter import *
from tkFont import Font
import threading
import ttk
import time
import math
from PIL import Image
from PIL import ImageDraw
import tkMessageBox
import os
import numpy as np
import subprocess

 
class MobileRobotSimulator(threading.Thread):
	
	def __init__(self):
		
		threading.Thread.__init__(self)
		self.rospack = rospkg.RosPack()
		self.stopped = False 
		# map size in meters
		self.mapX = 0 
		self.mapY = 0
		# canvas size in pixels
		self.canvasX= 600
		self.canvasY= 600
		# robot position and angle
		self.robot_theta=0
		self.robotX=-100
		self.robotY=-100

		self.p_giro=0
		self.p_distance=0
		
		self.polygonMap = []
		self.nodes_image = None
		self.light=-1
		self.robot=-1

		self.flagOnce=False

		self.light_x = 0
		self.light_y = 0
		self.startFlag = False

		self.lasers = []
		self.sensors_value = [None]*512;
		self.sensors_values = [None]*512;
		self.sensors_values_aux = [None]*512;
		self.sensors_values_aux_old = [None]*512;
		
		#for i in range(512):
			#self.sensors_value.append(0)
		#	self.sensors_values.append(0)
			#self.sensors_values_aux.append(0)


		self.graph_list = [200]
		for i in range(200):
			self.graph_list.append(0)

		self.rewind=[]
		self.trace_route= []
		self.varShowNodes   = False
		self.grid =[]
		self.contador = 0;
		self.contador_ = 0;
		self.bandera = True

		self.X_pose = 0
		self.Y_pose = 0

		self.num_polygons = 0  #How many polygons exist in the field.
		self.polygons = []	   #Stors polygons vertexes
		self.polygons_mm = []  #Stors 2 vertexses  for each polygon the maximum and minimum  x and y  points.

		self.objects_data = []
		self.grasp_id = False
		self.current_object = -1;
		self.current_object_name = -1;

		self.initX = 0
		self.initY = 0
		self.initR = 0

		self.steps_aux = 0
		self.posible_collision = [None] * 512;
		self.sensors_value = [None] * 512 

		self.start()

	def kill(self):  # When press (x) window
		self.stopped = True
		self.varTurtleBot.set(0)
		self.startFlag=False
		self.s_t_simulation(False)
		self.clear_topological_map()
		self.startFlag=False
		self.s_t_simulation(False)
		time.sleep(2)
		self.root.quit()
		self.root.destroy()



	def get_parameters(self): # It returns the parameters of simulation to be publish by a ROS topic
		parameters = []

		try:
			parameters.append(self.robotX*self.mapX / self.canvasX  )
		except ValueError:
			parameters.append(0.0)
		try:
			parameters.append( self.mapY  - (self.robotY)*self.mapX / self.canvasY )
		except ValueError:
			parameters.append(0.0)
		try:
			parameters.append(self.robot_theta)
		except ValueError:
			parameters.append(0.0)
		try:
			parameters.append(float(self.entryRadio.get()))
		except ValueError:
			parameters.append(0.0)
		try:
			parameters.append( float(self.entryAdvance.get()))
		except ValueError:
			parameters.append(0.0)
		try:
			parameters.append( float(self.entryTurnAngle.get()))
		except ValueError:
			parameters.append(0.0)
		try:
			parameters.append( int(self.entryNumSensors.get()))
		except ValueError:
			parameters.append(0)
		try:
			parameters.append( float(self.entryOrigin.get()))
		except ValueError:
			parameters.append(0.0)
		try:
			parameters.append( float(self.entryRange.get()))
		except ValueError:
			parameters.append(0.0)
		try:
			parameters.append( float(self.entryValue.get()))
		except ValueError:
			parameters.append(0.0)
		try:
			parameters.append( str(self.entryFile.get()))
		except ValueError:
			parameters.append("NOT FOUND")
		try:
			parameters.append( bool(self.varAddNoise.get() ))
		except ValueError:
			parameters.append(False)
		try:
			parameters.append( float(self.light_x))
		except ValueError:
			parameters.append(0.0)
		try:
			parameters.append( float(self.light_y))
		except ValueError:
			parameters.append(0.0)
		try:
			parameters.append( bool(self.startFlag ))
		except ValueError:
			parameters.append(False)
		try:
			parameters.append( int(self.entryBehavior.get() ))
		except ValueError:
			parameters.append(-1)
		try:
			parameters.append( bool(self.varTurtleBot.get()) )
		except ValueError:
			if self.varTurtleBot.get()==1:
				parameters.append( True )
			else:
				parameters.append( False )

		return parameters

	

##########################################
##########################################
#
#	Environment
#
##########################################
##########################################

	def print_graph(self,*args): # Plots graphs such as Dijkstra, DFS, A* and more (The graph is passed by a ROS service from motion_planner node)
			flagOnce=True;
			numNode_=1
			self.w.delete(self.nodes_image)	
			nodes_coords = []
			image = Image.new('RGBA', (self.canvasX, self.canvasY))
			draw = ImageDraw.Draw(image)
			map_file = open(self.rospack.get_path('simulator')+'/src/data/'+self.entryFile.get()+'/'+self.entryFile.get()+'.top','r')  #Open file
			lines = map_file.readlines()                          #Split the file in lines
			for line in lines: 									  #To read line by line
				words = line.split()	                          #To split  words 
				if words:										  #To avoid empty lines							
					if words[0] == "(":							  #To avoid coments
						if words[1] == "num":			  #To get world dimensions
							numNode = float (words[3])	
							if numNode == 0:
								numNode_=0
								map_file.close()
								break
						elif words[1] == "node":				  #to get polygons vertex
							#numNode = words[2]
							nodeXm = float (words[3]) * self.canvasX / self.mapX
							nodeYm = self.canvasY - ( float (words[4]) * self.canvasY) / self.mapY
	
							nodes_coords.append([nodeXm,nodeYm,numNode])
		
			if numNode_ != 0:
				secuence = 0
				for x in self.graph_list:
					if x != -1:
						if flagOnce:
							c1 = nodes_coords[x][0]
							c2 = nodes_coords[x][1]
							flagOnce = False
									
						draw.ellipse((nodes_coords[x][0] - 3 ,nodes_coords[x][1] - 3 ,nodes_coords[x][0] + 3 ,nodes_coords[x][1] + 3), outline = '#9C4FDB', fill = '#9C4FDB')
						draw.text( (nodes_coords[x][0],nodes_coords[x][1] + 2) ,fill = "darkblue" ,text = str(secuence) )
						secuence = secuence + 1
							#( connection 0 1 0.121195 )
						draw.line( (c1,c2,nodes_coords[x][0],nodes_coords[x][1]), fill = '#9C4FDB')
						c1 = nodes_coords[x][0]
						c2 = nodes_coords[x][1]

				map_file.close()

				image.save(self.rospack.get_path('simulator')+'/src/gui/nodes.png')
				self.gif1 = PhotoImage( file = self.rospack.get_path('simulator')+'/src/gui/nodes.png')
				self.nodes_image = self.w.create_image(self.canvasX / 2, self.canvasY / 2, image = self.gif1)

#####################
#####################
#
# Topological map
#
#####################
#####################
	def print_topological_map(self): # It plots  the topological map of the current map  and  show  "please wait" message
		wait_bg=self.w.create_rectangle(self.canvasX/2-30-120 ,self.canvasY/2-50 ,self.canvasX/2-30+120 ,self.canvasY/2+50 ,fill="white")
		wait = self.w.create_text(self.canvasX/2-30,self.canvasY/2,fill="darkblue",font="Calibri 20 bold",
                        text="PLEASE WAIT ...")
		self.w.update()
		self.print_topological_map_lines()
		self.w.delete(wait)
		self.w.delete(wait_bg)
		self.plot_robot()

	def clear_topological_map(self): # It removes topological_map
		if  self.varShowNodes :
			self.w.delete(self.nodes_image)	
			self.nodes=None
			self.w.update()
			self.varShowNodes = False

	def print_topological_map_lines(self): # It plots  the topological map of the current map  

		self.clear_topological_map();
		self.varShowNodes = True
		#self.w.delete(self.nodes_image)	
		nodes_coords = []
		image = Image.new('RGBA', (self.canvasX, self.canvasY))
		draw = ImageDraw.Draw(image)
		map_file = open(self.rospack.get_path('simulator')+'/src/data/'+self.entryFile.get()+'/'+self.entryFile.get()+'.top','r')                  #Open file
		lines = map_file.readlines()                          #Split the file in lines
		for line in lines: 									  #To read line by line
			words = line.split()	                          #To separate  words 
			if words:										  #To avoid empty lines							
				if words[0] == "(":							  #To avoid coments
					if words[1] == "num":			  #To get world dimensions
						numNode = float (words[3])	
					elif words[1] == "node":				  #to get polygons vertex
						numNode = words[2]
						nodeXm = float (words[3]) * self.canvasX / self.mapX
						nodeYm = self.canvasY - ( float (words[4]) * self.canvasY) / self.mapY
						nodes_coords.append([nodeXm,nodeYm])
						draw.ellipse((nodeXm - 3 ,nodeYm - 3 ,nodeXm + 3 ,nodeYm + 3), outline = '#9C4FDB', fill = '#9C4FDB')
						draw.text( (nodeXm,nodeYm + 2) ,fill = "darkblue" ,text = str(numNode) )
					elif words[1] == "connection":				  #to get polygons vertex
						c1 = int(words[2])
						c2 = int(words[3])
						draw.line( (nodes_coords[c1][0],nodes_coords[c1][1] ,nodes_coords[c2][0] ,nodes_coords[c2][1] ) , fill = '#9C4FDB')
						
		map_file.close()
		image.save(self.rospack.get_path('simulator')+'/src/gui/nodes.png')
		self.gif1 = PhotoImage( file = self.rospack.get_path('simulator')+'/src/gui/nodes.png')
		self.nodes_image = self.w.create_image(self.canvasX / 2, self.canvasY / 2, image = self.gif1)

##################################
##################################
# 
#   MAP
#
##################################
##################################
	def map_change(self,event): 
		self.w.delete(self.nodes_image)	
		self.read_map()
		if self.varShowNodes:
			self.clear_topological_map()
		self.delete_robot()
		for i in self.lasers:
			self.w.delete(i)
		self.lasers = []
		for trace in self.trace_route :
			self.w.delete(trace)
		self.trace_route = []
		self.w.delete(self.light)

	def read_map(self):  # It reads maps from  src/data/[map].wrl folder 
		self.num_polygons=0
		for polygon in self.polygonMap :
			self.w.delete(polygon)
		self.polygonMap = []
		self.polygons = []	
		self.polygons_mm = []
		try:
			#self.w.delete("all")
			map_file = open(self.rospack.get_path('simulator')+'/src/data/'+self.entryFile.get()+'/'+self.entryFile.get()+'.wrl','r') #Open file
			lines = map_file.readlines()                          #Split the file in lines
			for line in lines: 									  #To read line by line
				words = line.split()	                          #To separate  words 
				if words:										  #To avoid empty lines							
					if words[0] == "(":							  #To avoid coments
						if words[1] == "dimensions":			  #To get world dimensions
							self.mapX = float (words[3])	
							self.mapY = float (words[4])
							self.print_grid()
						elif words[1] == "polygon":				  #to get polygons vertex

							vertex_x = [ ( ( self.canvasX * float(x) ) / self.mapX ) for x in words[4:len(words)-1:2]	]
							vertex_y = [ ( self.canvasY -  ( self.canvasY * float(y) ) / self.mapY ) for y in words[5:len(words)-1:2]	]
							vertex_y_calculus = [ (( self.canvasY * float(y) ) / self.mapY ) for y in words[5:len(words)-1:2]	]
							
							vx = [ float(x)for x in words[4:len(words)-1:2] ]	
							vy = [ float(y)for y in words[5:len(words)-1:2] ]
						
							vertexs = ( zip( vertex_x , vertex_y) )
							self.polygons.append( zip( vertex_x , vertex_y_calculus))
							self.polygonMap.append(self.w.create_polygon(vertexs, outline=self.obstaclesOutlineColor, fill=self.obstacleInnerColor, width=1))	
							#self.w.create_text( self.canvasX * float(words[4]) / self.mapX,  self.canvasY -  ( self.canvasY * float(words[5]) ) / self.mapY, text=str(pp))
							max_x = 0;
							max_y = 0;
							min_x = 999;
							min_y = 999;

							for i in vertexs:
								if max_x < i[0]:
									max_x = i[0]
								if min_x > i[0]:
									min_x = i[0]

							for i in vertexs:
								if max_y < i[1]:
									max_y = i[1]
								if max_y > i[1]:
									min_y = i[1]
							self.polygons_mm.append( [[max_x,max_y],[min_x,min_y] ] )
					
							self.num_polygons = self.num_polygons+1
			for p in self.polygons:
				p.append(p[0])
		except IOError:
			tkMessageBox.showerror("World erros ", "World  '"+self.entryFile.get()+"' doesn' t exist \n Provide another file name ")





#####################################
#####################################
#
# Objects on the map
#
#####################################
#####################################
	def read_objects(self): 
		
		for polygon in self.objects_data:
			self.w.delete(polygon[3])
			self.w.delete(polygon[4])

		self.objects_data = []
		self.grasp_id = False
		if  self.varLoadObjects.get():
			try:
				map_file = open(self.rospack.get_path('simulator')+'/src/data/objects/objects.txt','r') #Open file
				lines = map_file.readlines()                          #Split the file in lines
				for line in lines: 									  #To read line by line
					words = line.split()	                          #To separate  words 
					if words:										  #To avoid empty lines							
						self.objects_data.append([words[0],float(words[1]),float(words[2]) ,-1,-1] )

				for i,objects_datas in enumerate(self.objects_data):
					objects_datas[3] = self.w.create_rectangle( (objects_datas[1]*self.canvasX)/self.mapX -10, ((1-objects_datas[2])*self.canvasY)/self.mapY -10 , (objects_datas[1]*self.canvasX)/self.mapX +10, ((1-objects_datas[2])*self.canvasY)/self.mapY +10  ,fill="#9FFF3D",outline="#9FFF3D")
					objects_datas[4] = self.w.create_text((objects_datas[1]*self.canvasX)/self.mapX ,((1-objects_datas[2])*self.canvasY)/self.mapY , fill="#9E4124",font="Calibri 10 bold",text=objects_datas[0])
			except IOError:
				tkMessageBox.showerror("Objects erros ", "Ups! an error occurred. \n Please check objects.txt syntax ")
		else:
			pass
	def exist_object(self,name):
		for obj in self.objects_data:
			if name == obj[0]:
				return True
		return False

	def get_object(self,name):
		for obj in self.objects_data:
			if name == obj[0]:
				return obj
		return False

	def grasp_object(self,name):
		distancia_min = 0.05

		if self.grasp_id==False :
			if self.exist_object(name) :
				obj = self.get_object(name)
				if  math.sqrt(((self.robotX*self.mapX)/self.canvasX-obj[1])**2+(((self.canvasY-self.robotY)*self.mapY)/self.canvasY-obj[2])**2 ) < distancia_min:
					self.grasp_id = name
					self.w.delete(obj[3])
					self.w.delete(obj[4])
					return True
				else:
					print("Distance to object "+name+" is more than "+str(distancia_min))
			else:
				print("Object " + name + " does not exist ")
		else:
			print("I can not grasp more than one object")

		return False

	def release_object(self):
		if self.grasp_id != False:
			
			for obj in self.objects_data:
				if self.grasp_id == obj[0]:
					x=(self.robotX*self.mapX)/self.canvasX + (( (float(self.entryRadio.get()))*math.cos(float(self.entryAngle.get()))))
					y=((self.canvasY -self.robotY)*self.mapY)/self.canvasY + (((float(self.entryRadio.get()))*math.sin(   float(self.entryAngle.get()) )))
					
					obj[1]= x
					obj[2]= y

					obj[3] = self.w.create_rectangle( (obj[1]*self.canvasX)/self.mapX -10, ((self.mapY-obj[2])*self.canvasY)/self.mapY -10 , (obj[1]*self.canvasX)/self.mapX +10, ((self.mapY-obj[2])*self.canvasY)/self.mapY +10  ,fill="#9FFF3D",outline="#9FFF3D")
					obj[4] = self.w.create_text((obj[1]*self.canvasX)/self.mapX ,((self.mapY-obj[2])*self.canvasY)/self.mapY , fill="#9E4124",font="Calibri 10 bold",text=obj[0])
			self.grasp_id = False
			return True
		else:
			print("Robot does not have an object")
			return False






#####################################
#####################################
#
#	Simulacion
#
#######################################
#######################################

	def s_t_simulation(self,star_stop): # Button start simulation
		
		if star_stop :
			self.w.delete(self.nodes_image)	
			self.denable('disabled')
			if not self.varTurtleBot.get:
				self.read_map()
			self.clear_topological_map() # To clear topological map
			self.startFlag=True
			self.steps_ = 0 ;
			self.steps_aux = int(self.entrySteps.get()) ;
			self.entrySteps.delete ( 0, END )
			self.entrySteps.insert ( 0, str(0)  )
			
			self.rewind_x = self.robotX
			self.rewind_y = self.robotY
			self.rewind_angle = self.robot_theta
			
			self.rewind=[]
			for trace in self.trace_route :
				self.w.delete(trace)
			self.trace_route = []
			
		else: 
			self.denable('normal')
			self.startFlag=False
			self.entrySteps.delete ( 0, END )
			self.entrySteps.insert ( 0, str(self.steps_aux)  )
       

	def rewindF(self): # When  the "Last simulation" button is pressed
		self.denable('disabled')
		self.buttonStop.configure(state='disabled')
		self.robotX = self.rewind_x
		self.robotY = self.rewind_y
		self.robot_theta=self.rewind_angle
		cta=1
		for i in self.rewind:
			self.p_giro  = i[0]
			self.p_distance = i[1]
			self.entryStepsExcec.config(text=str(cta))
			cta = cta+1;
			self.move_robot(0)
		self.denable('normal')
		self.buttonStop.configure(state='normal')

	def set_light_position(self,x,y): # Another way to start simulations, by plot the light ( goal point ).
		
		if self.light >0:
			self.w.delete(self.light)
		y1 = self.mapY - y 
		self.light = self.w.create_image(x/self.mapX*self.canvasX, y1/self.mapY*self.canvasY, image = self.gif2)
		self.light_x = x 
		self.light_y = y 
		self.entryLightX.config(text=str(self.light_x)[:4])
		self.entryLightY.config(text=str(self.light_y)[:4])


	def right_click(self,event): # Another way to start simulations, by plot the light ( goal point ).
		if not self.startFlag and not self.varTurtleBot.get():
			if self.light >0:
				self.w.delete(self.light)
			self.light = self.w.create_image(event.x, event.y, image = self.gif2)
			self.light_x = self.mapX*event.x / self.canvasX
			self.light_y = self.mapY -  ( self.mapY * event.y ) / self.canvasY
			self.entryLightX.config(text=str(self.light_x)[:4])
			self.entryLightY.config(text=str(self.light_y)[:4])
			self.s_t_simulation(True)

	def left_click(self,event): # It plot the robot in the field
		if not self.varTurtleBot.get(): 
			if self.robot > 0:
				self.delete_robot()
			self.robotX = event.x
			self.robotY = event.y
			self.plot_robot()




	def object_interaction(self,  *args):
		pass


##################################################
##################################################
#
# ROS
#
##################################################
##################################################
	def handle_simulator_object_interaction(self,grasp,name):
		if grasp == 1:
			return self.grasp_object(name)
		else :
			
			return self.release_object()
		self.d.set(1)


	def handle_service(self,theta,distance):
		if not self.varTurtleBot.get():
			self.p_giro = theta
			self.p_distance = distance * self.canvasX 
		else:
			return
			self.p_giro = 0
			self.p_distance = 0

		self.steps_= self.steps_+1;
		self.entrySteps.delete ( 0, END )
		 
		if self.startFlag:
			self.entrySteps.insert ( 0, str(self.steps_)  )
		else:
			self.entrySteps.insert ( 0, str(self.steps_aux)  )

		if self.steps_ == self.steps_aux:
			self.s_t_simulation(False)
		#elif( ( float(self.entryPoseX.get()) -self.light_x )**2 + (  float(self.entryPoseY.get())  - self.light_y )**2) < .05**2:
			#self.s_t_simulation(False)
		else:
			self.entryStepsExcec.config(text=str(self.steps_)[:4])
			self.rewind.append( [self.p_giro,self.p_distance])
			self.a.set(1)

	def handle_print_graph(self,graph_list):
		self.graph_list = graph_list 
		self.b.set(1)

	def handle_hokuyo(self,sensors_values):
		self.sensors_values = sensors_values 		
		self.c.set(1)

	def handle_turtle(self,x,y,r):

		if self.varTurtleBot.get():
			self.robotX = self.convert_from_m_to_pixel(  x * math.cos(self.initR) + y * math.sin(self.initR) - self.initX + 2.5)
			self.robotY = self.canvasY-self.convert_from_m_to_pixel( y * math.cos(self.initR) - x * math.sin(self.initR) - self.initY + 2.5 )

			self.robot_theta = r - self.initR
			self.d.set(1)
			#print(self.robotX,self.robotY,self.robot_theta)
		else:
			self.initX = x * math.cos(self.initR) + y * math.sin(self.initR)
			self.initY = y * math.cos(self.initR) - x * math.sin(self.initR)
			self.initR = r
			#print(x,y,r)


####################################################
####################################################
#real robot
####################################################
####################################################


			
	def print_real(self,*args):
		self.delete_robot()
		self.plot_robot2()

	def print_hokuyo_values(self,*args):

		"""if self.startFlag:

			originSensor = float( self.entryOrigin.get())   # -1.5707
			rangeSensor  = float( self.entryRange.get() )    #  240#3.1415
			numSensor    = int(self.entryNumSensors.get())
			rx = self.robotX
			ry = self.robotY
			color = '#FF1008'
			x =  300#( float( self.entryValue.get() ) * self.canvasX ) / self.mapY
			y = ry
			angle =self.robot_theta
			f = angle + originSensor
			step = float( float( rangeSensor ) / float( numSensor - 1 ) )

			for i in self.lasers:
				self.w.delete(i)
			self.lasers = []

			for i in range(0, numSensor):	
				#if self.sensors_values[i] == float("inf"):
				#	continue
				q,w =self.get_ray(f ,rx ,ry ,(self.sensors_value[i]/4  * self.canvasX ) / self.mapY)
				#self.lasers.append(self.w.create_line(rx ,ry ,q ,w ,fill = self.laserColor) )

				if float(self.sensors_value[i]) < float(self.entryValue.get()) :
					self.lasers.append(self.w.create_oval(q-1 ,w-1,q+1 ,w+1 ,fill = color, outline =color  ) )	
				f = f + step

		else:	"""
		if self.varTurtleBot.get() and not self.startFlag :
			originSensor = -1.5707#-2.0944 #float( self.entryOrigin.get())   # -1.5707
			rangeSensor  = 3.1415#4.18879#float( self.entryRange.get() )    #  240#3.1415
			numSensor    = 512 #int(self.entryNumSensors.get())
			rx = self.robotX
			ry = self.robotY
			color = '#FF1008'
			x =  300#( float( self.entryValue.get() ) * self.canvasX ) / self.mapY
			y = ry
			angle = self.robot_theta
			f = angle + originSensor
			step = float( float( rangeSensor ) / float( numSensor - 1 ) )

			for i in self.lasers:
				self.w.delete(i)
			self.lasers = []

			for i in range(0, 512,1):	
				#if self.sensors_values[i] == float("inf"):
				#	continue
				q,w =self.get_ray(f ,rx ,ry ,(self.sensors_values[i]  * self.canvasX ) / self.mapY)
				#self.lasers.append(self.w.create_line(rx ,ry ,q ,w ,fill = self.laserColor) )

				#if float(self.sensors_values[i]/4) < float(self.entryValue.get()) :

				self.lasers.append(self.w.create_oval(q-1 ,w-1,q+1 ,w+1 ,fill = color, outline =color  ) )	
				f = f + step

	def convert_from_m_to_pixel(self,m):
		return m * self.canvasX / self.mapX

	def use_real_robot(self):

		if self.varTurtleBot.get() :
			subprocess.Popen([self.rospack.get_path('simulator')+'/src/turtlebot/start_rviz_turtlebot.sh'])
			self.w.delete(self.nodes_image)
			state='disabled'
			if self.flagOnce :
				self.delete_robot()
			self.flagOnce=True

			for i in self.lasers:
				self.w.delete(i)
			self.clear_topological_map() # To clear topological map
			self.steps_ = 0 ;
			self.steps_aux = int(self.entrySteps.get()) ;
			self.entrySteps.delete ( 0, END )
			self.entrySteps.insert ( 0, str(100)  )
			
			for trace in self.trace_route :
				self.w.delete(trace)
			self.trace_route = []

			for polygon in self.polygonMap :
				self.w.delete(polygon)
			self.polygonMap = []
			self.polygons = []	
			self.polygons_mm = []
			self.mapX=5
			self.mapY=5
			self.entryRadio
			self.entryRadio.delete ( 0, END )
			self.entryRadio.insert ( 0, str(0.16)  )

			self.buttonMapLess.configure(state="normal")
			self.buttonMapMore.configure(state="normal")
			self.print_grid(1)
			#self.robotX=self.canvasX/2
			#self.robotY=self.canvasY/2
			#self.robot_theta=0
			#self.delete_robot()
			#self.plot_robot2()
			
		else: 
			self.buttonMapLess.configure(state="disabled")
			self.buttonMapMore.configure(state="disabled")
			state='normal'
			self.entryRadio         .configure(state=state)
			self.entryRadio.delete ( 0, END )
			self.entryRadio.insert ( 0, str(0.03)  )
			self.entrySteps.delete ( 0, END )
			self.entrySteps.insert ( 0, str(self.steps_aux)  )
			self.read_map()
			for i in self.lasers:
				self.w.delete(i)
			self.lasers = []

			self.delete_robot()

		self.buttonRviz .configure(state=state)  
		self.entryFile          .configure(state=state)     
		#self.entrySteps         .configure(state=state) 
		#self.buttonBehaviorLess .configure(state=state)         
		#self.entryBehavior    	.configure(state=state)        
		#self.buttonBehaviorMore .configure(state=state)       
		self.checkFaster      .configure(state=state)  
		self.checkShowSensors   .configure(state=state)  
		self.checkAddNoise      .configure(state=state)                
		self.entryRobot         .configure(state=state)   
		self.entryPoseX         .configure(state=state)   
		self.entryPoseY         .configure(state=state)   
		self.entryAngle         .configure(state=state)   
		self.entryRadio         .configure(state=state)   
		#self.entryAdvance       .configure(state=state)   
		#self.entryTurnAngle     .configure(state=state)   
		#self.entryNumSensors    .configure(state=state)    
		#self.entryOrigin        .configure(state=state)    
		#self.entryRange         .configure(state=state)    
		#self.entryValue         .configure(state=state)
		self.buttonLastSimulation.configure(state=state)    
		#self.buttonRunSimulation.configure(state=state)  
		self.buttonSetZero.configure(state=state) 
		self.buttonPlotTopological.configure(state=state)




	#####################################################################
	#####################################################################
	#####################################################################
	#              Window Utilities
	# The following functions are for define buttons, texboxes, check boxes
	# And others functions to manage configurations
	#####################################################################
	#####################################################################
	#####################################################################

	###lidar utilities

	def rotate_point(self,theta,ox,oy, x, y): 

		# It rotates a point (x,y) from another point (ox,oy)
		rotate = -theta
		nx = ( x - ox ) * math.cos( rotate ) - ( y - oy ) * math.sin(rotate) + ox
		ny = ( x - ox ) * math.sin( rotate ) + ( y - oy ) * math.cos(rotate) + oy
		return nx,ny


	def line_intersection(self, p1, p2, p3 , p4, laser_value): 

		#this function calculates the intersection  point between two segments of lines.
		# p1,p2 are the points of the first segment
		# p1,p2 are the points of the second segment

		denominadorTa = (p4[0]-p3[0])*(p1[1]-p2[1]) - (p1[0]-p2[0])*(p4[1]-p3[1])
		denominadorTb = (p4[0]-p3[0])*(p1[1]-p2[1]) - (p1[0]-p2[0])*(p4[1]-p3[1])
		
		if denominadorTa == 0 or denominadorTb ==0 :
			return laser_value

		ta = ( (p3[1]-p4[1])*(p1[0]-p3[0]) + (p4[0]-p3[0])*(p1[1]-p3[1]) ) / float( denominadorTa )
		tb = ( (p1[1]-p2[1])*(p1[0]-p3[0]) + (p2[0]-p1[0])*(p1[1]-p3[1]) ) / float( denominadorTb )
	    
		if  0 <= ta  and ta <= 1 and 0 <= tb and tb <= 1  :
			xi = p1[0]  + ta * ( p2[0] - p1[0] ) 
			yi = p1[1]  + ta * ( p2[1] - p1[1] ) 
			return math.sqrt( (p1[0] - xi)**2 + (p1[1] - yi)**2 ) 	
		else:
			return laser_value;	


	def calculate_ray_traicing(self): 

		#It Calculates laser values Raycasting
		j=0;
		ffl = False
		p1 = [None] * 2
		p2 = [None] * 2

		p3 = [None] * 2
		p4 = [None] * 2

		r_max = [None] * 2
		r_min = [None] * 2

		four_lines = [None] * 4

		p1[0] = self.robotX;
		p1[1] = (self.canvasY-self.robotY);

		value = float(self.entryValue.get() ) * self.canvasX / self.mapX

		r_max[0] = p1[0] + float(value); 
		r_max[1] = p1[1] + float(value);
		r_min[0] = p1[0] - float(value); 
		r_min[1] = p1[1] - float(value);

		#self.w.create_line( r_max[0], r_max[1], r_max[0], r_min[1], fill = 'red')
		#self.w.create_line( r_max[0], r_min[1], r_min[0], r_min[1], fill = 'red')
		#self.w.create_line( r_min[0], r_min[1], r_min[0], r_max[1], fill = 'red')
		#self.w.create_line( r_min[0], r_max[1], r_max[0], r_max[1], fill = 'red')
		#self.w.create_oval( r_max[0]+1, r_max[1]+1, r_max[0]-1, r_max[1]-1 ,outline='green', fill='green', width=1)
		#self.w.create_oval( r_min[0]+1, r_min[1]+1, r_min[0]-1, r_min[1]-1 ,outline='black', fill='black', width=1)

		four_lines[0] = [r_max[0], r_max[1], r_max[0], r_min[1]]
		four_lines[1] = [r_max[0], r_min[1], r_min[0], r_min[1]]
		four_lines[2] = [r_min[0], r_min[1], r_min[0], r_max[1]]
		four_lines[3] = [r_min[0], r_max[1], r_max[0], r_max[1]]


		for h in range(0,len(self.sensors_value)):
			self.sensors_value[h] = value

		
		for p in range(0,4):
			self.posible_collision[p] = p
			j = j + 1
		for i in range(0, self.num_polygons):
			ffl = False
			for k in four_lines:
				for m in range(0, len(self.polygons[i] ) ):
					if ffl == True:
						continue
					else:
						if self.polygons[i][m][0] <= r_max[0] and self.polygons[i][m][0] >= r_min[0] and self.polygons[i][m][1] <= r_max[1] and self.polygons[i][m][1] >= r_min[1] :

						#if self.line_intersection([k[0],k[1]],[k[2],k[3]],self.polygons[i][m],self.polygons[i][m+1],value) != value:
							self.posible_collision[j] = i
							j = j + 1
							ffl = True

		#print(str(j))
		#print(str(self.num_polygons))

		f = self.robot_theta + float(self.entryOrigin.get())

		step = float(self.entryRange.get()) / ( float(self.entryNumSensors.get()) - 1 )

		for k in range(0,int(self.entryNumSensors.get())):
			for i in range(0,j):
				for m in range(0, len(self.polygons[self.posible_collision[i]] )-1 ):
					p2[0] = float(value) * math.cos( f ) + float(p1[0]);
					p2[1] = float(value) * math.sin( f ) + float(p1[1]);
						
					if True:#self.ccw(p1,self.polygons[self.posible_collision[i]][m],self.polygons[self.posible_collision[i]][m+1]) != self.ccw(p2,self.polygons[ self.posible_collision[i] ][m],self.polygons[ self.posible_collision[i] ][m+1]) and self.ccw(p1,p2,self.polygons[ self.posible_collision[i] ][m]) != self.ccw(p1,p2,self.polygons[ self.posible_collision[i] ][m+1]):
						aux = self.line_intersection(p1,p2,self.polygons[self.posible_collision[i]][m],self.polygons[ self.posible_collision[i] ][m+1],value);
						if self.sensors_value[k] > aux*self.mapX/ self.canvasX:
							self.sensors_value[k] = aux*self.mapX	/ self.canvasX		
			f = f + step

	def ccw(self,A,B,C):
		return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])


	def plot_robot_values(self,color): 
		#self.sensors_value = self.sensors_values_aux
		x = self.robotX
		y = self.robotY
		angle=self.robot_theta

		try:
			self.entryPoseX.delete ( 0, END )
			self.entryPoseY.delete ( 0, END )
			self.entryAngle.delete ( 0, END )
			self.entryPoseX.insert ( 0, str(float(x)*self.mapX / self.canvasX) )
			self.entryPoseY.insert ( 0, str(self.mapY  - (float(y)*self.mapX / self.canvasY )  ))  
			if angle > math.pi*2 :  
				angle = angle  % (math.pi*2)
			if angle < 0 :
				angle = math.pi*2 - ( (angle * -1 ) % (math.pi*2) )
			self.robot_theta = angle
			self.entryAngle.insert ( 0, str( angle ) )

		except ValueError:
			pass

		if self.flagOnce :
			self.delete_robot()
		self.flagOnce=True

		if self.varShowSensors.get():
			self.plot_sensors(angle,x,y,color)	

		radio = ( float(self.entryRadio.get() ) * self.canvasX ) / self.mapX
		self.robot=self.w.create_oval(x-radio,y-radio, x+radio,y+radio   , outline=self.robotColor, fill=self.robotColor, width=1)
		self.hokuyo=self.w.create_oval(x-radio/5,y-radio/5, x+radio/5,y+radio/5 ,outline=self.hokuyoColor, fill=self.hokuyoColor, width=1)

		wheel1x1 = x - ( radio / 2 )
		wheel1y1 = y - ( 5 * radio /6 )
		wheel1x2 = x + radio / 2
		wheel1y2 = y - ( 3 * radio / 6 )
		wheel2y1 = y + ( 3 * radio / 6 )
		wheel2y2 = y + ( 5 * radio / 6 )
		wh1= []
		wh2= []
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel1y1))
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel1y1))
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel1y2))
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel1y2))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel2y1))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel2y1))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel2y2))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel2y2))
		self.wheelL=self.w.create_polygon( wh1 ,outline = self.wheelColor, fill = self.wheelColor, width=1)
		self.wheelR=self.w.create_polygon( wh2 ,outline = self.wheelColor, fill = self.wheelColor, width=1)
		head = []
		head.append( self.rotate_point( angle ,x ,y ,x + ( 2 * radio / 3 ) ,y - ( radio / 3 ) ) )
		head.append( self.rotate_point( angle ,x ,y ,x + ( 2 * radio / 3 ) ,y + ( radio / 3 ) ) )
		head.append( self.rotate_point( angle ,x ,y ,x + ( 5 * radio / 6 ) ,y ))
		self.arrow=self.w.create_polygon( head , outline = self.arrowColor , fill = self.arrowColor , width = 1 )
		if self.grasp_id != False:
			self.current_object = self.w.create_rectangle(x-10, y -10 , x +10, y +10  ,fill="#9FFF3D",outline="#9FFF3D")
			self.current_object_name = self.w.create_text(x ,y , fill="#9E4124",font="Calibri 10 bold",text=self.grasp_id)
		self.w.update()

	def plot_robot2(self):

		x = self.robotX
		y = self.robotY
		angle=self.robot_theta

		radio = ( float(self.entryRadio.get() ) * self.canvasX ) / self.mapX
		self.robot=self.w.create_oval(x-radio,y-radio, x+radio,y+radio   , outline=self.robotColor, fill=self.robotColor, width=1)
		self.hokuyo=self.w.create_oval(x-radio/5,y-radio/5, x+radio/5,y+radio/5 ,outline=self.hokuyoColor, fill=self.hokuyoColor, width=1)

		wheel1x1 = x - ( radio / 2 )
		wheel1y1 = y - ( 5 * radio /6 )
		wheel1x2 = x + radio / 2
		wheel1y2 = y - ( 3 * radio / 6 )
		wheel2y1 = y + ( 3 * radio / 6 )
		wheel2y2 = y + ( 5 * radio / 6 )
		wh1= []
		wh2= []
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel1y1))
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel1y1))
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel1y2))
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel1y2))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel2y1))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel2y1))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel2y2))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel2y2))
		self.wheelL=self.w.create_polygon( wh1 ,outline = self.wheelColor, fill = self.wheelColor, width=1)
		self.wheelR=self.w.create_polygon( wh2 ,outline = self.wheelColor, fill = self.wheelColor, width=1)
		head = []
		head.append( self.rotate_point( angle ,x ,y ,x + ( 2 * radio / 3 ) ,y - ( radio / 3 ) ) )
		head.append( self.rotate_point( angle ,x ,y ,x + ( 2 * radio / 3 ) ,y + ( radio / 3 ) ) )
		head.append( self.rotate_point( angle ,x ,y ,x + ( 5 * radio / 6 ) ,y ))
		self.arrow=self.w.create_polygon( head , outline = self.arrowColor ,fill = self.arrowColor, width = 1 )
		if self.grasp_id != False:
			self.current_object = self.w.create_rectangle(x-10, y -10 , x +10, y +10  ,fill="#9FFF3D",outline="#9FFF3D")
			self.current_object_name = self.w.create_text(x ,y , fill="#9E4124",font="Calibri 10 bold",text=self.grasp_id)
		self.w.update()
		time.sleep(.1)
		#self.laserColor = aux_color
	

	def plot_robot(self):
		# It plots the robot on the field
		# The position and angle depend on the variables:
		#  self.robotX, self.robotY and self.robot_theta
		self.calculate_ray_traicing()
		x = self.robotX
		y = self.robotY
		angle=self.robot_theta

		try:
			self.entryPoseX.delete ( 0, END )
			self.entryPoseY.delete ( 0, END )
			self.entryAngle.delete ( 0, END )
			self.entryPoseX.insert ( 0, str(float(x)*self.mapX / self.canvasX) )
			self.entryPoseY.insert ( 0, str(self.mapY  - (float(y)*self.mapX / self.canvasY )  ))  
			if angle > math.pi*2 :  
				angle = angle  % (math.pi*2)
			if angle < 0 :
				angle = math.pi*2 - ( (angle * -1 ) % (math.pi*2) )
			self.robot_theta = angle
			self.entryAngle.insert ( 0, str( angle ) )

		except ValueError:
			pass

		if self.flagOnce :
			self.delete_robot()
		self.flagOnce=True

		for i in self.lasers:
				self.w.delete(i)

		if self.varShowSensors.get():
			if  bool(self.varAddNoise.get() ) == True :
				self.plot_sensors(angle,x,y,"#1dff0d")	
			else:
				self.plot_sensors(angle,x,y)	

		radio = ( float(self.entryRadio.get() ) * self.canvasX ) / self.mapX
		self.robot=self.w.create_oval(x-radio,y-radio, x+radio,y+radio   , outline=self.robotColor, fill=self.robotColor, width=1)
		self.hokuyo=self.w.create_oval(x-radio/5,y-radio/5, x+radio/5,y+radio/5 ,outline=self.hokuyoColor, fill=self.hokuyoColor, width=1)

		wheel1x1 = x - ( radio / 2 )
		wheel1y1 = y - ( 5 * radio /6 )
		wheel1x2 = x + radio / 2
		wheel1y2 = y - ( 3 * radio / 6 )
		wheel2y1 = y + ( 3 * radio / 6 )
		wheel2y2 = y + ( 5 * radio / 6 )
		wh1= []
		wh2= []
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel1y1))
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel1y1))
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel1y2))
		wh1.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel1y2))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel2y1))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel2y1))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x2 ,wheel2y2))
		wh2.append(self.rotate_point (angle ,x ,y ,wheel1x1 ,wheel2y2))
		self.wheelL=self.w.create_polygon( wh1 ,outline = self.wheelColor, fill = self.wheelColor, width=1)
		self.wheelR=self.w.create_polygon( wh2 ,outline = self.wheelColor, fill = self.wheelColor, width=1)
		head = []
		head.append( self.rotate_point( angle ,x ,y ,x + ( 2 * radio / 3 ) ,y - ( radio / 3 ) ) )
		head.append( self.rotate_point( angle ,x ,y ,x + ( 2 * radio / 3 ) ,y + ( radio / 3 ) ) )
		head.append( self.rotate_point( angle ,x ,y ,x + ( 5 * radio / 6 ) ,y ))
		if self.grasp_id != False:
			self.current_object = self.w.create_rectangle(x-10, y -10 , x +10, y +10  ,fill="#9FFF3D",outline="#9FFF3D")
			self.current_object_name = self.w.create_text(x ,y , fill="#9E4124",font="Calibri 10 bold",text=self.grasp_id)

		self.arrow=self.w.create_polygon( head , outline = self.arrowColor , fill = self.arrowColor , width = 1 )
		self.w.update()

	

	def get_ray(self,angle,x,y,r):
		return r * math.cos( angle ) + x ,r * ( - math.sin(angle) ) + y

	
	def plot_sensors(self,angle,rx,ry,color = "#FF0D0D"):
		
		originSensor = float( self.entryOrigin.get())   # -1.5707
		rangeSensor  = float( self.entryRange.get() )    #  240#3.1415
		numSensor    = int(self.entryNumSensors.get())
		x = rx
		y = ry
		f = angle + originSensor
		step = float( float( rangeSensor ) / float( numSensor - 1 ) )

		for i in self.lasers:
			self.w.delete(i)
		self.lasers = []

		for i in range(0, numSensor):	
			q,w =self.get_ray(f ,rx ,ry ,(self.sensors_value[i] * self.canvasX ) / self.mapX)
			self.lasers.append(self.w.create_line(rx ,ry ,q ,w ,fill = color) )

			if float(self.sensors_value[i]) < float(self.entryValue.get()) :

				self.lasers.append(self.w.create_oval(q-1 ,w-1,q+1 ,w+1 ,fill = color, outline =color  ) )
			f = f + step
		
	def delete_robot(self):

		self.w.delete(self.robot)
		self.w.delete(self.wheelL)
		self.w.delete(self.wheelR)
		self.w.delete(self.arrow)
		self.w.delete(self.hokuyo)
		self.w.delete(self.current_object)
		self.w.delete(self.current_object_name)
		
	def move_robot(self,*args):

		theta = float(self.p_giro)
		distance = float(self.p_distance) 
		
		init_robotX = self.robotX
		init_robotY = self.robotY
		init_robotAngle = self.robot_theta
		i = self.robot_theta

		if self.varFaster.get() or self.varTurtleBot.get():
			self.plot_robot();			
			self.robot_theta = init_robotAngle + theta
			self.robotX=distance * math.cos(self.robot_theta) + self.robotX
			self.robotY=-( distance * math.sin(self.robot_theta) )+ self.robotY
			xf = self.robotX
			yf = self.robotY

		else:
			self.plot_robot();
			
			if theta ==0:
				pass
			else:
				if theta > 0 :
					while i < init_robotAngle + theta:
						i = i + (0.0174533*2)*self.sliderVelocity.get()
						self.robot_theta = i
						self.plot_robot()
						
				else:
					while i > init_robotAngle + theta:
						i = i - (0.0174533*2)*self.sliderVelocity.get()
						self.robot_theta = i
						self.plot_robot()
						
				self.robot_theta = init_robotAngle + theta
				self.plot_robot()

			xf = distance * math.cos(self.robot_theta) + self.robotX

			yf = -( distance * math.sin(self.robot_theta) )+ self.robotY


			x = auxX = init_robotX
			y = auxY = init_robotY
			m = -math.tan(self.robot_theta)

			if xf==auxX :
				if yf <= auxY:
					while y < yf:
						y = y - self.sliderVelocity.get()
						if(y > yf): 
							break
						self.robotY=y
						self.plot_robot()
				else:
					while y > yf:
						y = y + self.sliderVelocity.get()
						if(y < yf): 
							break
						self.robotY=y
						self.plot_robot()
				self.robotY = yf
				self.plot_robot()
			else:    # Calculos con punto pendiente  y2 -y1= m ( x2 - x1)
				if  m < -1 or  m > 1 : # Si los angulos caen en este rango en vez de evaluar y  evaluaremos x

					if yf > auxY:
						while y < yf:
							x = -( (y - auxY) / math.tan(self.robot_theta) -auxX	)
							y = y + self.sliderVelocity.get()
							if(y > yf): 
								break
							self.robotX=x
							self.robotY=y
							self.plot_robot()	
					else:
						while y > yf:
							x = -( (y - auxY) / math.tan(self.robot_theta) -auxX)   
							y = y - self.sliderVelocity.get()
							if(y < yf): 
								break
							self.robotX=x
							self.robotY=y
							self.plot_robot()	
				else:

					if xf > auxX:
						while x < xf:
							y = -math.tan(self.robot_theta) * (x - auxX) + auxY
							#print(y)
							x = x + self.sliderVelocity.get()
							if(x > xf): 
								break
							self.robotX=x
							self.robotY=y
							self.plot_robot()	
 
					else:
						while x > xf:
							y = -math.tan(self.robot_theta) * (x - auxX) + auxY
							x = x - self.sliderVelocity.get()
							if(x < xf): 
								break
							self.robotX=x
							self.robotY=y
							self.plot_robot()	
				self.robotX = xf
				self.robotY = yf

				self.plot_robot()



		self.trace_route.append(self.w.create_line(init_robotX ,init_robotY ,xf,yf,dash=(4, 4),   fill="#AB1111"))
	

	def print_grid(self,line_per_m = 10):
		for i in self.grid :
			self.w.delete(i)
		self.grid =[]

		for i in range(0, int(self.mapX)*line_per_m):
			self.grid.append(self.w.create_line( i * self.canvasX/(self.mapX*line_per_m),0, i*self.canvasX/(self.mapX*line_per_m), self.canvasY,  dash=(4, 4), fill=self.gridColor))
		for i in range(0, int(self.mapY)*line_per_m):
			self.grid.append(self.w.create_line( 0, i*self.canvasY/(self.mapY*line_per_m),self.canvasX, i*self.canvasY/(self.mapY*line_per_m),   dash=(4, 4), fill=self.gridColor))

	def behavioLess(self): #Button behavior <
		try:
			newbehavior=int(self.entryBehavior.get())
			self.entryBehavior.delete ( 0, END )
			self.entryBehavior.insert ( 0, str(newbehavior-1) )
		except ValueError:
			self.entryBehavior.delete ( 0, END )
			self.entryBehavior.insert ( 0, '1' )
	
	def behavioMore(self): #Button behavior >
		try:
			newbehavior=int(self.entryBehavior.get())
			self.entryBehavior.delete ( 0, END )
			self.entryBehavior.insert ( 0, str(newbehavior+1) )
		except ValueError:
			self.entryBehavior.delete ( 0, END )
			self.entryBehavior.insert ( 0, '1' )
	def set_angle(self,foo): #
		self.robot_theta = float(self.entryAngle.get())
		self.plot_robot()


	def set_zero_angle(self): #
		self.robot_theta = 0.0
		self.plot_robot()	

	def denable(self,state): # It disables some widgets when  a simulation is running
		self.buttonPlotTopological.configure(state=state)  
		self.entryFile          .configure(state=state)     
		#self.entrySteps         .configure(state=state) 
		self.buttonBehaviorLess .configure(state=state)         
		self.entryBehavior    	.configure(state=state)        
		self.buttonBehaviorMore .configure(state=state)       
		self.checkFaster      .configure(state=state)  
		self.checkShowSensors   .configure(state=state)  
		self.checkAddNoise      .configure(state=state)                
		self.entryRobot         .configure(state=state)   
		#self.entryPoseX         .configure(state=state)   
		#self.entryPoseY         .configure(state=state)   
		#self.entryAngle         .configure(state=state)   
		self.entryRadio         .configure(state=state)   
		self.entryAdvance       .configure(state=state)   
		self.entryTurnAngle     .configure(state=state)   
		self.entryNumSensors    .configure(state=state)    
		self.entryOrigin        .configure(state=state)    
		self.entryRange         .configure(state=state)    
		self.entryValue         .configure(state=state)
		self.buttonLastSimulation.configure(state=state)    
		self.buttonRunSimulation.configure(state=state)  
		#self.buttonStop 

	def mapMore(self):
		self.mapX = self.mapX-1 
		self.mapY = self.mapY-1 
		self.print_grid(1)
	def mapLess(self):
		self.mapX = self.mapX+1 
		self.mapY = self.mapY+1
		self.print_grid(1)

	def start_rviz(self):
		subprocess.Popen([self.rospack.get_path('simulator')+'/src/gui/start_rviz.sh'])
		

	def NewFile():
		print ("")
	def OpenFile():
		print ("")
	def About():
		print ("")
	

	def resizeCanvas(self,x,y):
		self.robotX = self.robotX * x / self.canvasX
		self.robotY = self.robotY * y / self.canvasY
		self.canvasX =x
		self.canvasY =y
		self.w.configure(width = self.canvasX, height = self.canvasY)
		self.changeTheme()

	def changeTheme(self):
		self.w.configure(bg = self.canvasColor)
		self.print_grid()
		self.read_map()
		self.buttonBehaviorLess.configure(background=self.buttonColor, foreground = self.buttonFontColor)
		self.buttonBehaviorMore.configure(background=self.buttonColor, foreground = self.buttonFontColor)
		self.buttonRunSimulation.configure(background=self.buttonColor, foreground = self.buttonFontColor)
		self.buttonLastSimulation.configure(background=self.buttonColor, foreground = self.buttonFontColor)
		self.buttonPlotTopological.configure(background=self.buttonColor, foreground = self.buttonFontColor)
		self.buttonStop.configure(background=self.buttonColor, foreground = self.buttonFontColor)
		self.plot_robot()

	def whiteTheme(self):
		self.obstacleInnerColor = '#447CFF'
		self.obstaclesOutlineColor="#216E7D"#'#002B7A'
		self.buttonColor = "#1373E6"
		self.buttonFontColor = "#FFFFFF"
		self.canvasColor = "#FFFFFF"
		self.gridColor = "#D1D2D4"
		self.wheelColor  = '#404000'  
		self.robotColor  = '#F7CE3F'  
		self.hokuyoColor = '#4F58DB' 
		self.arrowColor  = '#1AAB4A' 
		self.laserColor  = "#00DD41" 
		self.changeTheme()

	def darkTheme(self):
		self.obstacleInnerColor = '#003B00'
		self.obstaclesOutlineColor="#00FF41"#'#002B7A'
		self.buttonColor = "#3F4242"
		self.buttonFontColor = "#FFFFFF"
		self.canvasColor = "#0D0208"
		self.gridColor = "#333333"
		self.wheelColor  = '#404000'  
		self.robotColor  = '#FF1008'  
		self.hokuyoColor = '#006BFF' 
		self.arrowColor  = '#006BFF' 
		self.laserColor  = "#08FFFB" 
		self.changeTheme()

	def gui_init(self):

		self.backgroundColor = '#EDEDED';#"#FCFCFC";
		self.entrybackgroudColor = "#FBFBFB";##1A3A6D";
		self.entryforegroundColor = '#37363A';
		self.titlesColor = "#303133"
		self.menuColor = "#ECECEC"
		self.menuButonColor = "#375ACC"
		self.menuButonFontColor = "#FFFFFF"
		self.obstacleInnerColor = '#447CFF'
		self.obstaclesOutlineColor="#216E7D"#'#002B7A'
		self.buttonColor = "#1373E6"
		self.buttonFontColor = "#FFFFFF"
		self.canvasColor = "#FFFFFF"
		self.gridColor = "#D1D2D4"
		self.wheelColor  = '#404000'  
		self.robotColor  = '#F7CE3F'  
		self.hokuyoColor = '#4F58DB' 
		self.arrowColor  = '#1AAB4A' 
		self.laserColor  = "#00DD41" 


		
		self.root = Tk()
		self.root.protocol("WM_DELETE_WINDOW", self.kill)
		self.root.title("Mobile Robot Simulator")

		self.barMenu = Menu(self.root)
		self.settingsMenu = Menu(self.barMenu, tearoff=0)
		self.submenuTheme = Menu(self.settingsMenu, tearoff=0)
		self.submenuCanvas = Menu(self.settingsMenu, tearoff=0)
		self.root.config(menu=self.barMenu )
		self.barMenu.config(background = self.menuColor)

		
		self.barMenu.add_cascade(label=" Settings ", menu=self.settingsMenu,background = self.menuButonFontColor )
		self.settingsMenu.add_cascade(label=" Canvas size ", menu=self.submenuCanvas,background = self.menuButonFontColor)
		self.settingsMenu.add_cascade(label=" Theme ", menu=self.submenuTheme,background = self.menuButonFontColor)
		self.submenuTheme.add_command(label=" White   ", command=self.whiteTheme)
		self.submenuTheme.add_command(label=" Dark   ", command=self.darkTheme)
		
		self.submenuCanvas.add_command(label=" 600 x 600 ", command= lambda : self.resizeCanvas(600,600) )
		self.submenuCanvas.add_command(label=" 700 x 700 ", command= lambda : self.resizeCanvas(700,700) )
		self.submenuCanvas.add_command(label=" 800 x 800 ", command= lambda : self.resizeCanvas(800,800) )


		self.helpMenu = Menu(self.barMenu, tearoff=0)
		self.helpMenu.add_command(label=" Topological map ", command=self.NewFile)
		self.helpMenu.add_command(label=" User guide ", command=self.OpenFile)
		self.helpMenu.add_command(label=" ROS nodes ", command=self.root.quit)
		self.barMenu.add_cascade(label="Help", menu=self.helpMenu,background = self.menuButonFontColor)
			
	
		self.content   = Frame(self.root)
		self.frame     = Frame(self.content,borderwidth = 5, relief = "flat", width = 600, height = 900 ,background = self.backgroundColor)
		self.rightMenu = Frame(self.content,borderwidth = 5, relief = "flat", width = 300, height = 900 ,background = self.backgroundColor)
		self.w = Canvas(self.frame, width = self.canvasX, height = self.canvasY, bg=self.canvasColor)
		self.w.pack()
		
		self.headLineFont = Font( family = 'Helvetica' ,size = 12, weight = 'bold')
		self.lineFont     = Font( family = 'Helvetica' ,size = 10, weight = 'bold')
		self.buttonFont   = Font( family = 'Helvetica' ,size = 8, weight = 'bold')


		self.lableEnvironment   = Label(self.rightMenu ,text = "Settings"     ,background = self.backgroundColor ,foreground = self.titlesColor ,font = self.headLineFont)
		self.labelFile          = Label(self.rightMenu ,text = "Environment:"           ,background = self.backgroundColor ,font = self.lineFont)
		self.labelSteps         = Label(self.rightMenu ,text = "Steps:"          ,background = self.backgroundColor ,font = self.lineFont)
		self.labelBehavior		= Label(self.rightMenu ,text = "Behavior:"          ,background = self.backgroundColor ,font = self.lineFont)
		self.labelLightX        = Label(self.rightMenu ,text = "Light X:"          ,background = self.backgroundColor ,font = self.lineFont)
		self.labelLightY        = Label(self.rightMenu ,text = "Light Y:"          ,background = self.backgroundColor ,font = self.lineFont)
		self.labelStepsExcec        = Label(self.rightMenu ,text = "Steps:"          ,background = self.backgroundColor ,font = self.lineFont)
		self.labelConfiguration = Label(self.rightMenu ,text = "Configurations:" ,background = self.backgroundColor ,font = self.lineFont)
			
		self.entryFile  = Entry(self.rightMenu ,width = 15 ,foreground = self.entryforegroundColor ,background = self.entrybackgroudColor )
		self.entryFile.bind('<Return>', self.map_change)
		self.entrySteps = Entry(self.rightMenu ,width = 15 ,foreground = self.entryforegroundColor ,background = self.entrybackgroudColor )

		self.buttonBehaviorLess          = Button(self.rightMenu ,width = 1, foreground = self.buttonFontColor, background = self.buttonColor , font = self.buttonFont ,text = "<" ,command = self.behavioLess)
		self.entryBehavior    	         = Entry(self.rightMenu ,width = 4 ,foreground = self.entryforegroundColor ,background = self.entrybackgroudColor ,justify='center' )
		self.buttonBehaviorMore          = Button(self.rightMenu ,width = 1, foreground = self.buttonFontColor, background = self.buttonColor , font = self.buttonFont, text = ">" ,command = self.behavioMore)
		
		self.entryLightX = Label(self.rightMenu ,text = "Click Right" ,background = self.backgroundColor ,font = self.lineFont ,justify='center')
		self.entryLightY = Label(self.rightMenu ,text = "Click Right" ,background = self.backgroundColor ,font = self.lineFont ,justify='center')
		self.entryStepsExcec = Label(self.rightMenu ,text = "0" ,background = self.backgroundColor ,font = self.lineFont ,justify='center')
		self.entryFile.insert ( 0, 'random_2' )
		self.entrySteps.insert( 0, '100' )
		self.entryBehavior.insert ( 0, '4' )

		self.buttonMapLess = Button(self.rightMenu ,width = 5, foreground = self.buttonFontColor, background = self.buttonColor , font = self.buttonFont ,text = "Zoom Out" ,command = self.mapLess)
		self.buttonMapMore = Button(self.rightMenu ,width = 5, foreground = self.buttonFontColor, background = self.buttonColor , font = self.buttonFont, text = "Zoom In " ,command = self.mapMore)
		
		##### Rigth menu widgets declaration

		# Environment

		self.varFaster    = IntVar()
		self.varShowSensors = IntVar()
		self.varAddNoise    = IntVar()
		self.varLoadObjects    = IntVar()
		

		self.checkFaster    = Checkbutton(self.rightMenu ,text = 'Fast Mode'    ,variable = self.varFaster    ,onvalue = 1 ,offvalue = 0 ,background = self.backgroundColor)
		self.checkShowSensors = Checkbutton(self.rightMenu ,text = 'Show Sensors' ,variable = self.varShowSensors ,onvalue = 1 ,offvalue = 0 ,background = self.backgroundColor)
		self.checkAddNoise    = Checkbutton(self.rightMenu ,text = 'Add Noise'    ,variable = self.varAddNoise    ,onvalue = 1 ,offvalue = 0 ,background = self.backgroundColor)
		self.checkLoadObjects    = Checkbutton(self.rightMenu ,text = 'Load Objects'    ,variable = self.varLoadObjects    ,onvalue = 1 ,offvalue = 0 ,background = self.backgroundColor,command = self.read_objects)
		

		self.checkFaster      .deselect()
		self.checkShowSensors .select()
		self.checkAddNoise    .deselect()
		self.checkLoadObjects    .deselect()
	
		# Robot 

		self.lableRobot     = Label(self.rightMenu ,text = "Robot"              ,background = self.backgroundColor ,foreground = self.titlesColor ,font = self.headLineFont )
		self.labelPoseX     = Label(self.rightMenu ,text = "Pose X:"            ,background = self.backgroundColor ,font = self.lineFont)
		self.labelPoseY     = Label(self.rightMenu ,text = "Pose Y:"            ,background = self.backgroundColor ,font = self.lineFont)
		self.labelAngle     = Label(self.rightMenu ,text = "Angle:"             ,background = self.backgroundColor ,font = self.lineFont)
		self.labelRadio     = Label(self.rightMenu ,text = "Radio:"             ,background = self.backgroundColor ,font = self.lineFont)
		self.labelAdvance   = Label(self.rightMenu ,text = "Magnitude Advance:" ,background = self.backgroundColor ,font = self.lineFont)
		self.labelTurnAngle = Label(self.rightMenu ,text = "Turn Angle:"        ,background = self.backgroundColor ,font = self.lineFont)


		self.entryRobot     = Entry(self.rightMenu, width = 8 ,background = self.entrybackgroudColor  ,foreground = self.entryforegroundColor)
		self.entryPoseX     = Entry(self.rightMenu, width = 8 ,background = self.entrybackgroudColor  ,foreground = self.entryforegroundColor)
		self.entryPoseY     = Entry(self.rightMenu, width = 8 ,background = self.entrybackgroudColor  ,foreground = self.entryforegroundColor)
		self.entryAngle     = Entry(self.rightMenu, width = 8 ,background = self.entrybackgroudColor  ,foreground = self.entryforegroundColor)
		self.buttonSetZero  = Button(self.rightMenu ,width = 8, text = "Angle Zero", foreground = self.buttonFontColor ,background = self.buttonColor, font = self.buttonFont, command = self.set_zero_angle )
		self.entryRadio     = Entry(self.rightMenu, width = 8 ,background = self.entrybackgroudColor  ,foreground = self.entryforegroundColor)
		self.entryAdvance   = Entry(self.rightMenu, width = 8 ,background = self.entrybackgroudColor  ,foreground = self.entryforegroundColor)
		self.entryTurnAngle = Entry(self.rightMenu, width = 8 ,background = self.entrybackgroudColor  ,foreground = self.entryforegroundColor)

		self.entryPoseX     .insert ( 0, '0.5' )
		self.entryPoseY     .insert ( 0, '200' )
		self.entryAngle     .insert ( 0, '0.0' )
		self.entryAngle.bind('<Return>', self.set_angle)
		self.entryRadio     .insert ( 0, '0.03' )
		self.entryAdvance   .insert ( 0, '0.04' )
		self.entryTurnAngle .insert ( 0, '0.7857' )

		self.labelVelocity = Label(self.rightMenu ,text = "Execution velocity:"        ,background = self.backgroundColor ,font = self.lineFont)
		self.sliderVelocity =Scale(self.rightMenu, from_=1, to=3, orient=HORIZONTAL ,length=200 ,background = self.backgroundColor ,font = self.lineFont)
		

		# Sensors

		self.lableSensors     = Label(self.rightMenu, text = "Sensors"       ,background = self.backgroundColor ,foreground = self.titlesColor ,font = self.headLineFont)
		self.labelNumSensors  = Label(self.rightMenu, text = "Num Sensors:"  ,background = self.backgroundColor ,font = self.lineFont)
		self.labelOrigin      = Label(self.rightMenu, text = "Origin angle:" ,background = self.backgroundColor ,font = self.lineFont)
		self.labelRange       = Label(self.rightMenu, text = "Range:"        ,background = self.backgroundColor ,font = self.lineFont)
		self.labelValue       = Label(self.rightMenu, text = "Value:"        ,background = self.backgroundColor ,font = self.lineFont)

		self.entryNumSensors = Entry(self.rightMenu, width = 8 ,background = self.entrybackgroudColor ,foreground = self.entryforegroundColor)
		self.entryOrigin     = Entry(self.rightMenu, width = 8 ,background = self.entrybackgroudColor ,foreground = self.entryforegroundColor)
		self.entryRange      = Entry(self.rightMenu, width = 8 ,background = self.entrybackgroudColor ,foreground = self.entryforegroundColor)
		self.entryValue      = Entry(self.rightMenu, width = 8 ,background = self.entrybackgroudColor ,foreground = self.entryforegroundColor)

		self.entryNumSensors   .insert ( 0, '20')
		self.entryOrigin       .insert ( 0, '-1.5707' )
		self.entryRange        .insert ( 0, '3.1415' )
		self.entryValue        .insert ( 0, '0.05' )

		# buttons

		self.lableSimulator      = Label (self.rightMenu ,text = "Simulator" ,background = self.backgroundColor ,foreground = self.titlesColor ,font = self.headLineFont)
		self.buttonRviz         = Button(self.rightMenu ,width = 20, text = "Open Rviz", foreground = self.buttonFontColor ,background = self.buttonColor, font = self.buttonFont, command = self.start_rviz )
		self.buttonPlotTopological= Button(self.rightMenu ,width = 20, text = "Plot Topological", foreground = self.buttonFontColor ,background = self.buttonColor, font = self.buttonFont ,command = self.print_topological_map  )
		self.buttonLastSimulation= Button(self.rightMenu ,width = 20, text = "Run last simulation" ,state="disabled", foreground = self.buttonFontColor ,background = self.buttonColor , font = self.buttonFont ,command = self.rewindF  )
		self.buttonRunSimulation = Button(self.rightMenu ,width = 20, text = "Run simulation", foreground = self.buttonFontColor ,background = self.buttonColor,font = self.buttonFont,command = lambda: self.s_t_simulation(True) )
		self.buttonStop          = Button(self.rightMenu ,width = 20, text = "Stop", foreground = self.buttonFontColor ,background = self.buttonColor, font = self.buttonFont, command = lambda: self.s_t_simulation(False) )

		self.lableTurtleBot = Label(self.rightMenu, text = "Real robot" ,background = self.backgroundColor ,foreground = self.titlesColor ,font = self.headLineFont)
		self.varTurtleBot   = IntVar()
		self.checkTurtleBot = Checkbutton(self.rightMenu ,text = 'Use TurtleBot' ,variable = self.varTurtleBot ,onvalue = 1 ,offvalue = 0 ,background = self.backgroundColor, command = self.use_real_robot )
		

		#### Right menu widgets grid			

		# Environment
		self.lableEnvironment  .grid(column = 0 ,row = 0 ,sticky = (N, W) ,padx = (5,5))
		self.labelFile         .grid(column = 0 ,row = 1 ,sticky = (N, W) ,padx = (10,5))
		self.labelSteps        .grid(column = 0 ,row = 2 ,sticky = (N, W) ,padx = (10,5))
		self.labelBehavior     .grid(column = 0 ,row = 3 ,sticky = (N, W) ,padx = (10,5))
		self.labelLightX       .grid(column = 0 ,row = 4 ,sticky = (N, W) ,padx = (10,5))
		self.labelLightY       .grid(column = 0 ,row = 5 ,sticky = (N, W) ,padx = (10,5))
		self.labelStepsExcec   .grid(column = 0 ,row = 6 ,sticky = (N, W) ,padx = (10,5))

		self.labelConfiguration.grid(column = 0 ,row = 7 ,sticky = (N, W) ,padx = (10,5))

		self.entryFile       .grid(column = 1 ,row = 1 ,columnspan = 2 ,sticky = (N, W) ,padx = 5)
		self.entrySteps		 .grid(column = 1 ,row = 2 ,columnspan = 2 ,sticky = (N, W) ,padx = 5)	
		
		self.buttonBehaviorLess.grid(column = 1 ,row = 3 ,columnspan = 1 ,sticky = (N, W) ,padx = 5)
		self.entryBehavior   .grid(column = 1 ,row = 3 ,columnspan = 1  ,padx = 5)
		self.buttonBehaviorMore.grid(column = 1 ,row = 3 ,columnspan = 1 ,sticky = (N, E) ,padx = 5)
		
		self.entryLightX.grid(column = 1 ,row = 4 ,columnspan = 2 ,sticky = (N, W) ,padx = 5)
		self.entryLightY.grid(column = 1 ,row = 5 ,columnspan = 2 ,sticky = (N, W) ,padx = 5)
		self.entryStepsExcec.grid(column = 1 ,row = 6 ,columnspan = 2 ,sticky = (N, W) ,padx = 5)
		

		self.checkFaster   .grid(column = 1 ,row = 7  ,sticky = (N, W) ,padx = 5)
		self.checkShowSensors.grid(column = 1 ,row = 8  ,sticky = (N, W) ,padx = 5)
		self.checkAddNoise   .grid(column = 1 ,row = 9  ,sticky = (N, W) ,padx = 5)
		self.checkLoadObjects   .grid(column = 1 ,row = 10  ,sticky = (N, W) ,padx = 5)
		
		# Robot

		self.lableRobot     .grid(column = 4 ,row = 0 ,sticky = (N, W) ,padx = (5,5))     
		self.labelPoseX     .grid(column = 4 ,row = 1 ,sticky = (N, W) ,padx = (10,5))
		self.labelPoseY     .grid(column = 4 ,row = 2 ,sticky = (N, W) ,padx = (10,5))
		self.labelAngle     .grid(column = 4 ,row = 3 ,sticky = (N, W) ,padx = (10,5))
		self.labelRadio     .grid(column = 4 ,row = 5 ,sticky = (N, W) ,padx = (10,5))
		self.labelAdvance   .grid(column = 4 ,row = 6 ,sticky = (N, W) ,padx = (10,5))
		self.labelTurnAngle .grid(column = 4 ,row = 7 ,sticky = (N, W) ,padx = (10,5))
		self.labelVelocity	.grid(column = 4 ,row = 8 ,sticky = (N, W) ,padx = (10,5))
		
		self.sliderVelocity .grid(column = 4 ,row = 9 ,columnspan = 2 ,rowspan = 2 ,sticky = (N, W), padx = 5)

		self.entryPoseX     .grid(column = 5 ,row = 1 ,columnspan = 2 ,sticky = (N, W), padx = 5)
		self.entryPoseY     .grid(column = 5 ,row = 2 ,columnspan = 2 ,sticky = (N, W), padx = 5)
		self.entryAngle     .grid(column = 5 ,row = 3 ,columnspan = 2 ,sticky = (N, W), padx = 5)
		self.buttonSetZero  .grid(column = 5 ,row = 4 ,columnspan = 2 ,sticky = (N, W), padx = 5)
		self.entryRadio     .grid(column = 5 ,row = 5 ,columnspan = 2 ,sticky = (N, W), padx = 5)
		self.entryAdvance   .grid(column = 5 ,row = 6 ,columnspan = 2 ,sticky = (N, W), padx = 5)
		self.entryTurnAngle .grid(column = 5 ,row = 7 ,columnspan = 2 ,sticky = (N, W), padx = 5)
		
		# Sensors

		self.lableSensors       .grid(column = 0 ,row = 12  ,sticky = (N, W) ,padx = (5,5))     
		self.labelNumSensors    .grid(column = 0 ,row = 13  ,sticky = (N, W) ,padx = (10,5))
		self.labelOrigin        .grid(column = 0 ,row = 14  ,sticky = (N, W) ,padx = (10,5))
		self.labelRange         .grid(column = 0 ,row = 15 ,sticky = (N, W) ,padx = (10,5))
		self.labelValue         .grid(column = 0 ,row = 16 ,sticky = (N, W) ,padx = (10,5))

		self.entryNumSensors    .grid(column = 1 ,row = 13  ,columnspan=2 ,sticky = (N, W) ,padx = 5)
		self.entryOrigin        .grid(column = 1 ,row = 14  ,columnspan=2 ,sticky = (N, W) ,padx = 5)
		self.entryRange         .grid(column = 1 ,row = 15 ,columnspan=2 ,sticky = (N, W) ,padx = 5)
		self.entryValue         .grid(column = 1 ,row = 16 ,columnspan=2 ,sticky = (N, W) ,padx = 5)

		self.lableTurtleBot.grid(column = 0 ,row = 17 ,columnspan=2 ,sticky = (N, W) ,padx = 5)
		self.checkTurtleBot.grid(column = 1 ,row = 18 ,columnspan=2 ,sticky = (N, W) ,padx = 5)

		self.buttonMapLess.grid(column = 1 ,row = 19 ,columnspan=1 ,sticky = (N, W) ,padx = 5)
		self.buttonMapMore.grid(column = 1 ,row = 19 ,columnspan=1 ,sticky = (N, E) ,padx = 5)

		# buttons

		self.lableSimulator     .grid(column = 4 ,row = 12  ,sticky = (N, W) ,padx = (5,5))
		self.buttonRviz   .grid(column = 4 ,row = 14  ,sticky = (N, W) ,padx = (10,5))
		self.buttonPlotTopological   .grid(column = 4 ,row = 15  ,sticky = (N, W) ,padx = (10,5))
		self.buttonLastSimulation   .grid(column = 4 ,row = 16  ,sticky = (N, W) ,padx = (10,5))
		self.buttonRunSimulation.grid(column = 4 ,row = 17 ,sticky = (N, W) ,padx = (10,5))
		self.buttonStop         .grid(column = 4 ,row = 18 ,sticky = (N, W) ,padx = (10,5))

		self.content  .grid(column = 0 ,row = 0 ,sticky = (N, S, E, W))
		self.frame    .grid(column = 0 ,row = 0 ,columnspan = 3 ,rowspan = 2 ,sticky = (N, S, E, W))
		self.rightMenu.grid(column = 3 ,row = 0 ,columnspan = 3 ,rowspan = 2 ,sticky = (N, S, E, W))

		self.root.columnconfigure(0, weight=1)
		self.root.rowconfigure(0, weight=1)
		self.content.columnconfigure(0, weight = 3)
		self.content.columnconfigure(1, weight = 3)
		self.content.columnconfigure(2, weight = 3)
		self.content.columnconfigure(3, weight = 1)
		self.content.columnconfigure(4, weight = 1)
		self.content.rowconfigure(1, weight = 1)

		self.gif2 = PhotoImage( file = self.rospack.get_path('simulator')+'/src/gui/light.png')
		self.gif2.zoom(50, 50)

		self.a = IntVar(value=3)
		self.a.trace("w", self.move_robot)
		
		self.b = IntVar(value=3)
		self.b.trace("w", self.print_graph)

		self.c = IntVar(value=3)
		self.c.trace("w", self.print_hokuyo_values)

		self.d = IntVar(value=3)
		self.d.trace("w", self.object_interaction)

		self.d = IntVar(value=3)
		self.d.trace("w", self.print_real)

		self.w.bind("<Button-3>", self.right_click)
		self.w.bind("<Button-1>", self.left_click)

		self.buttonMapLess.configure(state="disabled")
		self.buttonMapMore.configure(state="disabled")


		
	def run(self):	
		self.gui_init()
		self.read_map()
		self.plot_robot2()
		self.root.mainloop()
