


class environment():


	def print_topological_map(self): # It plots  the topological map of the current map  and  show  "please wait" message
		wait_bg=w.create_rectangle(self.canvasX/2-30-120 ,self.canvasY/2-50 ,self.canvasX/2-30+120 ,self.canvasY/2+50 ,fill="white")
		wait = w.create_text(self.canvasX/2-30,self.canvasY/2,fill="darkblue",font="Calibri 20 bold",
	                       text="PLEASE WAIT ...")
		self.w.update()
		self.print_topological_map_lines()
		self.w.delete(wait)
		self.w.delete(wait_bg)
		self.plot_robot()

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

