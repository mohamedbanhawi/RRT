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
import matplotlib.pyplot as plt
import math
import random
#--------------------------------------Classes--------------------------------------------
#Environment and Obstacles
class env:

#environment class is defined by obstacle vertices and boundaries
	def __init__(self, x,y,xmin,xmax,ymin,ymax):
		self.x = x
		self.y = y
		self.xmin=xmin
		self.xmax=xmax
		self.ymin=ymin
		self.ymax=ymax
		
#when obstacles are sensed
	def obs_add(self,ox,oy):
		self.x += ox
		self.y += oy
				
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
		(x,y)= (G.x[n], G.y[n]) 
		if (x>=xgmin) and (x<=xgmax) and (y>=ygmin) and (y<=ygmax):
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
			elif (x<E.xmin) and (x>E.xmax) and (y<E.ymin) and (y>E.ymax):
				return 0
				break
						
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
class RRT:
	def __init__(self,nstart):
		(x,y,theta)=nstart
		self.x=[]
		self.y=[]
		self.theta=[]
		self.parent=[]
		self.x.append(x)
		self.y.append(y)
		self.theta.append(theta)
		self.edgex=[]
		self.edgey=[]
		self.edgex.append(x)
		self.edgey.append(y)
		
		#first node is the only node whose parent is itself
		self.parent.append(0)
		
		#define control and time space
		self.uSpeed = [2, 1, 0.5, 0.25 ]
		self.uSteer = [math.pi/-6.0,math.pi/-12.0,math.pi/-18.0, 0.0, math.pi/6.0,math.pi/12.0,math.pi/18.0]
		self.ut = 1
	
	#get metric value (euclidean distance)
	def metric(self,n1,n2):
		(x1,y1)= (self.x[n1],self.y[n1])
		(x2,y2)= (self.x[n2],self.y[n2])
		x1=float(x1)
		y1=float(y1)
		x2=float(x2)
		y2=float(y2)
		px=(x1-x2)**(2)
		py=(y1-y2)**(2)
		metric = (px+py)**(0.5)
		return metric
	
	#expand a random point
	#calls subroutines to find nearest node and connect it
	def expand (self):
		#add random node
		x = random.uniform (E.xmin, E.xmax)
		y = random.uniform (E.ymin, E.ymax)
		theta = random.uniform (0, math.pi)
		n= self.number_of_nodes() #new node number
		self.add_node(n,x,y,theta)
		if E.isfree()!=0:
			#find nearest node
			nnear = self.near(n)
			#find new node based on the transition equation
			self.step(nnear,n)
		
	def bias (self):
		#add random node
		n= self.number_of_nodes() #new node
		self.add_node(n,xg,yg,0.0) #test goal region
		#find nearest node
		nnear = self.near(n)
		#find new node based on the transition equations
		self.step(nnear,n)
	
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
	
#state transition 
	def step(self,nnear,nrand):
		(xn,yn,thetan) = (self.x[nnear],self.y[nnear],self.theta[nnear])
		(xran,yran,thetaran) = (self.x[nrand],self.y[nrand],self.theta[nrand])
		
		#compute all reachable states
		xr=[]
		yr=[]
		thetar=[]
		usp=[]
		ust=[]
		for i in self.uSpeed:
			for j in self.uSteer:
				usp.append(i)
				ust.append(j)
				(x,y,theta)=self.trajectory(xn,yn,thetan,j,i)
				xr.append(x)
				yr.append(y)
				thetar.append(theta)
				
		#find a nearest reachable from nnear to nrand
		dmin = ((((xran-xr[0][-1])**2)+((yran-yr[0][-1])**2))**(0.5))
		near = 0
		for i in range(1,len(xr)):
			d = ((((xran-xr[i][-1])**2)+((yran-yr[i][-1])**2))**(0.5))
			if d < dmin:
				dmin= d
				near = i
		self.remove_node(nrand)
		
		#collision detection
		cd = False
		for i in range(0,len(xr[near])):
			if E.isfree_xy(xr[near][i],yr[near][i])==0:
				cd= True
				break
		if cd==False:
			self.add_node(nrand,xr[near][-1],yr[near][-1],thetar[near][-1])
			self.add_edge(nnear,nrand,xr[near],yr[near])
			
#generate trajectory by integrating equations of motion			
	def trajectory(self,xi,yi,thetai,ust,usp):
		(x,y,theta)=([],[],[])
		x.append(xi)
		y.append(yi)
		theta.append(thetai)
		dt=0.01
		for i in range(1,int(self.ut/dt)):
			theta.append(theta[i-1]+usp*math.tan(ust)/1.9*dt)
			x.append(x[i-1]+usp*math.cos(theta[i-1])*dt)
			y.append(y[i-1]+usp*math.sin(theta[i-1])*dt)	
		return (x,y,theta)
		
#add node
	def add_node(self,n,x,y,theta):
		self.x.insert(n, x)
		self.y.insert(n, y)
		self.theta.insert(n,theta)

#remove node
	def remove_node(self,n):
		self.x.pop(n)
		self.y.pop(n)
		self.theta.pop(n)

#add edge
	def add_edge(self,parent,child,x,y):
		self.parent.insert(child,parent)
		self.edgex.append(x)
		self.edgey.append(y)
		
#remove node		
	def remove_edge(self,n):
		self.parent.pop(n)
		
#clear
	def clear(self,nstart):
		(x,y,theta)=nstart
		self.x=[]
		self.y=[]
		self.theta=[]
		self.parent=[]
		self.x.append(x)
		self.y.append(y)
		self.theta.append(theta)
		#first node is the only node whose parent is itself
		self.parent.append(0)
		self.theta.append(theta)
		self.edgex=[]
		self.edgey=[]
		
#number of nodes
	def number_of_nodes(self):
		return len(self.x)
		
#path to goal
	def path_to_goal(self):
		#find goal state
		for i in range (0,G.number_of_nodes()):
			(x,y)= (self.x[i],self.y[i])
			if (x>=xgmin) and (x<=xgmax) and (y>=ygmin) and (y<=ygmax):
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
			(x1,y1)=(G.x[self.path[s]],G.y[self.path[s]])
			(x2,y2)=(G.x[self.path[e]],G.y[self.path[e]])
			if E.inobstacle(x1,y1,x2,y2)==0: #CC is detected
				c=0
				self.tpath.append(self.path[e-1])
				s=e-1
		self.tpath.append(self.path[-1])			
	
	def execute(self):
	
	#subdivide path into small waypoints
	#in future can be replaced with B-spline planning
		self.wayx=[]
		self.wayy=[]
		self.newstart=[]
		for i in range (0,len(self.tpath)-1):
			(x1,y1)=(G.x[self.tpath[i]],G.y[self.tpath[i]])
			(x2,y2)=(G.x[self.tpath[i+1]],G.y[self.tpath[i+1]])
			for j in range (0,101):
				dt=j/100.0
				x=x1*(1-dt)+x2*dt
				y=y1*(1-dt)+y2*dt
				self.wayx.append(x)
				self.wayy.append(y)
	
	def showtree(self,k):			
		for i in range (1,G.number_of_nodes()):
			plt.plot(G.edgex[i],G.edgey[i],k,lw=0.5)
			
	def showpath(self,k):
		for i in range (len(self.path)-1):
			 n1=self.path[i]
			 plt.plot(G.edgex[n1],G.edgey[n1],k,lw=0.5)

#--------------------------------------Global Definitions---------------------------------
#node limit
nmax = 2000

#goal region
xg=5.0
yg=5.0
epsilon=5.0
xgmin=xg-epsilon
xgmax=xg+epsilon
ygmin=yg-epsilon
ygmax=yg+epsilon


#start the root of the tree
nstart =(100.0,100.0,math.pi) 

#specify vertices for rectangular obstacles (each object has four vertices)
#obstacles known a priori
vx= [40,40,60,60,70,70,80,80,40,40,60,60]
vy= [52,100,100,52,40,60,60,40, 0,48,48, 0]
#hidden
hvx= []
hvy= []

#create an RRT tree with a start node
G=RRT(nstart)

#environment instance
E=env(vx,vy,0,100,0,100)
	

#--------------------------------------Functions------------------------------------------
#draw trees and environment
def draw ():
	#draw boundary
	plt.plot([0,0,100,100,0],[0,100,100,0,0],'k',lw=0.5)
	
	#goal region
	plt.plot([xgmin,xgmin,xgmax,xgmax,xgmin],[ygmin,ygmax,ygmax,ygmin,ygmin],'g',lw=2)
	
	#draw tree
	G.showtree('0.45')
		 
	#draw path
	G.showpath('r')
	
	#draw obstacles
	num = len(E.x)/4
	for i in range(1,num+1):
		plt.plot([E.x[4*(i-1)],E.x[4*(i-1)+1],E.x[4*(i-1)+2],
		E.x[4*(i-1)+3],E.x[4*(i-1)]],[E.y[4*(i-1)],E.y[4*(i-1)+1],
		E.y[4*(i-1)+2],E.y[4*(i-1)+3],E.y[4*(i-1)]],'k',lw=2)	

	plt.show()

#--------------------------------------RRT Implementation---------------------------------
def main():
	#balance between extending and biasing	
	for i in range(0,nmax):
		if i%10!=0: G.expand()
		else: G.bias()
		if E.ingoal()==1:
			break
	plt.text(45, 103, 'Loops: %d' %(i+1))
	G.path_to_goal()
		
	#display initial plan under limited sensing
	draw()
	
	
# run main when RRT is called
if __name__ == '__main__':
    main()