#!/usr/bin/env python
#
#
# RRT Implementation in Python
# by Mohamed Elbanhawi (September 2013)
# Version 3.0 alpha - December 2013
#
# This is a two dimensional implementation of RRT algorithm
#
#
# 
#
# 
#--------------------------------------Libraries------------------------------------------
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math
import random
#--------------------------------------Classes--------------------------------------------
#Environment and Obstacles
class env3d:

#environment class is defined by obstacle vertices and boundaries
	def __init__(self, x,y,z,xmin,xmax,ymin,ymax,zmin,zmax):
		self.x = x
		self.y = y
		self.z = z
		self.xmin=xmin
		self.xmax=xmax
		self.ymin=ymin
		self.ymax=ymax
		self.zmin=zmin
		self.zmax=zmax
		
#when obstacles are sensed
	def obs_add(self,ox,oy,oz):
		self.x += ox
		self.y += oy
		self.z += oz
				
#Collision checking for a path
	def inobstacle(self,x1,y1,x2,y2):
		c=1 #assume no collision
		obs_num = len(self.x)/4 #four vertices for each rectangular obstacle
		for i in range(1,obs_num+1):
			xomin=self.x[4*(i-1)]
			xomax=self.x[4*(i-1)+2]
			yomin=self.y[4*(i-1)]
			yomax=self.y[4*(i-1)+1]
			for j in range(0,101):
				u=j/100.0
				x=x1*u+x2*(1-u)
				y=y1*u+y2*(1-u)
				if (x>=xomin) and (x<=xomax) and (y>=yomin) and (y<=yomax):
					c=0
					break
			if c==0: break	
		return c

#check if newly added sample is in the free configuration space
	def isfree(self):
		n= G.number_of_nodes()-1
		(x,y)= (G.x[n], G.y[n]) 
		obs_num = len(self.x)/4 #four vertices for each rectangular obstacle
		for i in range(1,obs_num+1):
			xomin=self.x[4*(i-1)]
			xomax=self.x[4*(i-1)+2]
			yomin=self.y[4*(i-1)]
			yomax=self.y[4*(i-1)+1]
			if (x>=xomin) and (x<=xomax) and (y>=yomin) and (y<=yomax):
				G.remove_node(n)
				return 0
				break	
				
#check if current node is in goal region
	def ingoal(self):
		n= G.number_of_nodes()-1
		(x,y,z)= (G.x[n], G.y[n],G.z[n]) 
		if (x>=xgmin) and (x<=xgmax) and (y>=ygmin) and (y<=ygmax) and (z>=zgmin) and (z<=zgmax) :
			return 1
		else:
			return 0
			
#check for a specific node
	def isfree_xy(self,x,y): 
		obs_num = len(self.x)/4 #four vertices for each rectangular obstacle
		for i in range(1,obs_num+1):
			xomin=self.x[4*(i-1)]
			xomax=self.x[4*(i-1)+2]
			yomin=self.y[4*(i-1)]
			yomax=self.y[4*(i-1)+1]
			if (x>=xomin) and (x<=xomax) and (y>=yomin) and (y<=yomax):
				return 0
				break
				
#draw the edges of a 3d cuboid			
	def cubedraw(self,obsx,obsy,obzl,obzh,k):
		x = obsx
		y = obsy
		zl = [obzl,obzl,obzl,obzl,obzl]
		zh = [obzh,obzh,obzh,obzh,obzh]
		
		ax.plot(x, y, zl,k)
		ax.plot(x,y,zh,k)
		for i in range (0,len(x)-1):
			obx = [x[i],x[i]]
			oby = [y[i],y[i]]
			obz = [zl[i],zh[i]]
			ax.plot(obx,oby,obz,k)
						
#Sensing
#Check for obstacles given the robot's current position and the sensor 
#Only hidden obstacles are checked, others are known a-priori

	def sense (self,x,y,r):
		obs_num = len(hvx)/4 #four vertices for each rectangular obstacle
		for i in range(1,obs_num+1):
			xomin=hvx[4*(i-1)]-r
			xomax=hvx[4*(i-1)+2]+r
			yomin=hvy[4*(i-1)]-r
			yomax=hvy[4*(i-1)+1]+r
			if (x>=xomin) and (x<=xomax) and (y>=yomin) and (y<=yomax):
			
#if the robot is within the sensing range of the obstacle, add it to visibile list
				hx_i= [hvx[4*(i-1)],hvx[4*(i-1)],hvx[4*(i-1)+2],hvx[4*(i-1)+2]]
				hy_i= [hvy[4*(i-1)],hvy[4*(i-1)+1],hvy[4*(i-1)+1],hvy[4*(i-1)]]
				
				self.obs_add(hx_i,hy_i)
#add point where new obstacle detected
				return 1
#-----------------------------------------------------------------------------------------
class RRT3d:
	def __init__(self,nstart):
		(x,y,z)=nstart
		self.x=[]
		self.y=[]
		self.z=[]
		self.parent=[]
		self.x.append(x)
		self.y.append(y)
		self.z.append(z)
		#first node is the only node whose parent is itself
		self.parent.append(0)
	
	#get metric value (current metric is euclidean distance)
	def metric(self,n1,n2):
		(x1,y1,z1)= (self.x[n1],self.y[n1],self.z[n1])
		(x2,y2,z2)= (self.x[n2],self.y[n2],self.z[n2])
		x1=float(x1)
		y1=float(y1)
		x2=float(x2)
		y2=float(y2)
		z1=float(z1)
		z2=float(z2)
		px=(x1-x2)**(2)
		py=(y1-y2)**(2)
		pz=(z1-z2)**(2)
		metric = (px+py+pz)**(0.5)
		return metric
			
	#expand a random point
	#calls subroutines to find nearest node and connect it
	def expand (self):
		#add random node
		x = random.uniform (E.xmin, E.xmax)
		y = random.uniform (E.ymin, E.ymax)
		z = random.uniform (E.zmin, E.zmax)
		n= self.number_of_nodes() #new node number
		self.add_node(n,x,y,z)
		if E.isfree()!=0:
			#find nearest node
			nnear = self.near(n)
			#find new node based on step size
			self.step(nnear,n)
			#connect the random node with its nearest node
			self.connect(nnear,n)
		
	def bias (self):
		#add random node
		n= self.number_of_nodes() #new node
		self.add_node(n,xg,yg,zg) #test goal region
		#find nearest node
		nnear = self.near(n)
		#find new node based on step size
		self.step(nnear,n)
		#connect the random node with its nearest node
		self.connect(nnear,n)
	
	#nearest node
	def near(self,n):
		#find a near node
		dmin = self.metric(0,n)
		nnear = 0
		for i in range(0,n):
			if self.metric(i,n) < dmin:
				dmin=self.metric(i,n)
				nnear = i
		return nnear
		
#step size
	def step(self,nnear,nrand):
		d = self.metric(nnear,nrand)
		if d>dmax:
			u=dmax/d
			(xnear,ynear,znear)= (self.x[nnear],self.y[nnear],self.z[nnear])
			(xrand,yrand,zrand)= (self.x[nrand],self.y[nrand],self.z[nrand]) 
			(px,py,pz)=(xrand-xnear,yrand-ynear,zrand-znear)
			theta = math.atan2(py,px)
			x=xnear+dmax*math.cos(theta)
			y=ynear+dmax*math.sin(theta)
			alpha = math.atan2(pz,y)
			z=znear+dmax*math.sin(alpha)
			self.remove_node(nrand)
			self.add_node(nrand,x,y,z) #this is a new node between rand and near

#connect two nodes (local planner)
	def connect(self,n1,n2):
		(x1,y1,z1)= (self.x[n1],self.y[n1],self.z[n1])
		(x2,y2,z2)= (self.x[n2],self.y[n2],self.z[n2])
		n= G.number_of_nodes()-1
		#subdivide path into 100 small segments and ensure each segment is collision free
		if E.inobstacle(x1,y1,x2,y2)==0:
			self.remove_node(n2)
		else:
			self.add_edge(n1,n2)
			
#connect two trees (Boundary Valued Problem)
	def BVP_to(self,A):
		#attempt to connect this node
		n1=self.number_of_nodes()-1
		(x1,y1,z1)= (self.x[n1],self.y[n1],self.z[n1])
		c=0 #assume no connection
		num=A.number_of_nodes()
		for i in range (0,num-1):
			(x2,y2,z2)= (A.x[n2],A.y[n2],A.z[n2])
			if E.inobstacle(x1,y1,x2,y2)==1:
				self.add_node(n1+1,x2,y2)
				self.add_edge(n1,n1+1)
				self.BVPnode=n1+1
				A.BVPnode=i
				c=1
				break
		return c					

#add node
	def add_node(self,n,x,y,z):
		self.x.insert(n, x)
		self.y.insert(n, y)
		self.z.insert(n, z)

#remove node
	def remove_node(self,n):
		self.x.pop(n)
		self.y.pop(n)
		self.z.pop(n)

#add edge
	def add_edge(self,parent,child):
		self.parent.insert(child,parent)
		
#remove node		
	def remove_edge(self,n):
		self.parent.pop(n)
		
#clear
	def clear(self,nstart):
		(x,y)=nstart
		self.x=[]
		self.y=[]
		self.z=[]
		self.parent=[]
		self.x.append(x)
		self.y.append(y)
		self.z.append(z)
		#first node is the only node whose parent is itself
		self.parent.append(0)
		
#number of nodes
	def number_of_nodes(self):
		return len(self.x)
		
#path to goal
	def path_to_goal(self):
		#find goal state
		for i in range (0,G.number_of_nodes()):
			(x,y,z)= (self.x[i],self.y[i],self.z[i])
			if (x>=xgmin) and (x<=xgmax) and (y>=ygmin) and (y<=ygmax) and (z>=zgmin) and (z<=zgmax) :
				self.goalstate = i
				break
		#add goal state to and its parent node to the path	
		self.path=[]
		self.path.append(i)
		newpos=self.parent[i]
		#keep adding parents	
		while (newpos!=0):
			self.path.append(newpos)
			newpos=self.parent[newpos]	
		#add start state
		self.path.append(0)
		
	def prun(self):	
		#initial query nodes in the path
		#we already know 0-1 is collision free
		#start by checking 0-2
		s=0
		e=2
		self.tpath=[]
		self.tpath.append(self.path[s])
		for e in range(len(self.path)-1):
			(x1,y1,z1)=(self.x[self.path[s]],self.y[self.path[s]],self.z[self.path[s]])
			(x2,y2,z2)=(self.x[self.path[e]],self.y[self.path[e]],self.z[self.path[e]])
			if E.inobstacle(x1,y1,x2,y2)==0: #CC is detected
				c=0
				self.tpath.append(self.path[e-1])
				s=e-1
		self.tpath.append(self.path[-1])			
	
	def waypoints(self):
	#subdivide path into small waypoints
	#in future can be replaced with B-spline planning
		self.wayx=[]
		self.wayy=[]
		self.wayz=[]
		self.newstart=[]
		for i in range (0,len(self.tpath)-1):
			(x1,y1,z1)=(self.x[self.tpath[i]],self.y[self.tpath[i]],self.z[self.tpath[i]])
			(x2,y2,z2)=(self.x[self.tpath[i+1]],self.y[self.tpath[i+1]],self.z[self.tpath[i+1]])
			for j in range (0,101):
				dt=j/100.0
				x=x1*(dt)+x2*(1-dt)
				y=y1*(dt)+y2*(1-dt)
				z=z1*(dt)+z2*(1-dt)
				self.wayx.append(x)
				self.wayy.append(y)
				self.wayz.append(z)
				#measurement update
				E.sense(x,y,20)
				#collision after update
				if E.isfree_xy(x,y)==0:
					#point before collision is used for generating new plan
					self.newstart.append(i*101+j-10)
					break
					
	def sense(self):
		while(len(self.newstart)!=0):
			#first observation state
			cn=self.newstart[0]
			cx=self.wayx[cn]
			cy=self.wayy[cn]
			cz=self.wayz[cn]
			self.clear((cx,cy,cz))
			#balance between extending and biasing	
			for i in range(0,nmax):
				if i%10!=0: self.expand()
				else: self.bias()
			#check if sample is in goal, if so STOP!		
				if E.ingoal()==1:
					break
			
			cn=self.newstart[0]
			cx=self.wayx[cn]
			cy=self.wayy[cn]
			cz=self.wayz[cn]
			plt.plot(cx,cy,cz,'yo',markersize=75,alpha=0.3)
			
			#find path in RRT
			self.path_to_goal()
			self.prun()
			#display initial plan under limited sensing
			draw()
			#execute
			self.waypoints()
					
	#draw tree
	def showtree(self,k):
		for i in range (0,self.number_of_nodes()):
			par=self.parent[i]
			x=[self.x[i],self.x[par]]
			y=[self.y[i],self.y[par]]
			z=[self.z[i],self.z[par]]
			ax.plot(x,y,z,k,lw=0.5)
			
	#draw path 
	def showpath(self,k):
		for i in range (len(self.path)-1):
			 n1=self.path[i]
			 n2=self.path[i+1]
			 x=[self.x[n1],self.x[n2]]
			 y=[self.y[n1],self.y[n2]]
			 z=[self.z[n1],self.z[n2]]
			 ax.plot(x,y,z,k,lw=1,markersize=3)
			 
	#draw path to be executed
	def showtpath(self,k):
		for i in range (len(self.tpath)-1):
			 n1=self.tpath[i]
			 n2=self.tpath[i+1]
			 x=[self.x[n1],self.x[n2]]
			 y=[self.y[n1],self.y[n2]]
			 z=[self.z[n1],self.z[n2]]
			 ax.plot(x,y,z,k,lw=2,markersize=5)
			
#--------------------------------------Global Definitions---------------------------------
#node limit
nmax = 5000

#goal region
xg=5
yg=5
zg=5
epsilon=5
xgmin=xg-epsilon
xgmax=xg+epsilon
ygmin=yg-epsilon
ygmax=yg+epsilon
zgmin=zg-epsilon
zgmax=zg+epsilon


#extend step size
dmax = 5
#start the root of the tree
nstart =(100,100,100) 

#specify vertices for rectangular obstacles (each object has four vertices)
#obstacles known a priori
vx= [40,40,60,60,70,70,80,80,40,40,60,60]
vy= [52,100,100,52,40,60,60,40, 0,48,48, 0]
vz = [0,100]
#hidden obstacle
hvx= [15,15,25,25,25,25,35,35]
hvy= [15,30,30,15,40,60,60,40]

#create an RRT tree with a start node
G=RRT3d(nstart)

#environment instance
E=env3d(vx,vy,vz,0,100,0,100,0,100)
	
#draw setup
fig = plt.figure()
ax = fig.gca(projection='3d')

#--------------------------------------Functions------------------------------------------
#draw trees and environment
def draw ():
	#draw 
	
	#goal region
	gx=[xgmin,xgmin,xgmax,xgmax,xgmin]
	gy=[ygmin,ygmax,ygmax,ygmin,ygmin]
	E.cubedraw(gx,gy,xg-epsilon,xg+epsilon,'g')
		
	#draw tree
	G.showtree('0.45')
		 
	#draw path
	G.showpath('ro-')
	G.showtpath('g*-')
		 
	#draw obstacles
	num = len(E.x)/4
	for i in range(1,num+1):
		obx=[E.x[4*(i-1)],E.x[4*(i-1)+1],E.x[4*(i-1)+2],E.x[4*(i-1)+3],E.x[4*(i-1)]]
		oby=[E.y[4*(i-1)],E.y[4*(i-1)+1],E.y[4*(i-1)+2],E.y[4*(i-1)+3],E.y[4*(i-1)]]
		E.cubedraw(obx,oby,E.zmin,E.zmax,'k')
	
		
	#draw  hidden obstacles (if they exist)
	obs_num = len(hvx)/4
	for i in range(1,obs_num+1):
		obsx=[hvx[4*(i-1)],hvx[4*(i-1)+1],hvx[4*(i-1)+2],hvx[4*(i-1)+3],hvx[4*(i-1)]]
		obsy=[hvy[4*(i-1)],hvy[4*(i-1)+1],hvy[4*(i-1)+2],hvy[4*(i-1)+3],hvy[4*(i-1)]]
		E.cubedraw(obx,oby,E.zmin,E.zmax,'k--')
	plt.show()


#--------------------------------------RRT Implementation---------------------------------
def main():
	#balance between extending and biasing	
	for i in range(0,nmax):
		if i%10!=0: G.expand()
		else: G.bias()
	#check if sample is in goal, if so STOP!		
		if E.ingoal()==1:
			print 'found'
			break
	G.path_to_goal()
	G.prun()
		
	#display initial plan under limited sensing
	draw()
	
	

	
	
# run main when RRT is called
if __name__ == '__main__':
    main()