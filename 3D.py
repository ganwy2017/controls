import numpy as np
from math import *
from pylab import *
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from visual import *
from visual.graph import *

def dist_calc(x1, y1, z1, x2, y2, z2):
    dx = x1-x2
    dy = y1-y2
    dz = z1-z2
    dist = sqrt(dx**2 + dy**2)
    gamma = atan2(dz,dist)
    psi = atan2(y2-y1,x2-x1)
    dist_fin = sqrt(dx**2+dy**2+dz**2)
    return dist_fin, psi, gamma

def attractive(d,psi,gamma,s,r,alpha):
    if d < r:
        delx = 0
        dely = 0
        delz = 0
    elif r <= d <= s+r:
        delx = alpha*(d-r)*cos(psi)*cos(gamma)
        dely = alpha*(d-r)*sin(psi)*cos(gamma)
        delz = alpha*(d-r)*(-sin(gamma))
    elif d > s + r:
        delx = alpha*s*cos(psi)*cos(gamma)
        dely = alpha*s*sin(psi)*cos(gamma)
        delz = alpha*s*(-sin(gamma))
    return delx, dely, delz

def repulsive(d,psi,gamma,s,r,beta):
    if d < r:
        delx = -np.sign(cos(psi)*cos(gamma))*float("inf")
        dely = -np.sign(cos(psi)*cos(gamma))*float("inf")
        delz = -np.sign(-sin(gamma))*float("inf")
    elif r <= d <=s+r:
        delx = -beta*(s+r-d)*cos(psi)*cos(gamma)
        dely = -beta*(s+r-d)*sin(psi)*cos(gamma)
        delz = -beta*(s+r-d)*(-sin(gamma))
    elif d > s + r:
        delx = 0
        dely = 0
        delz = 0
    return delx, dely,delz

def draw_robot(delx,dely,delz,robot_x,robot_y,robot_z):
	#Ideally you should be using OOP so self for coordinates will be useful
    robot_x = robot_x + delx
    robot_y = robot_y + dely
    robot_z = robot_z + delz
    return robot_x, robot_y, robot_z

def obstacle_maker(x,y,z,psi,gamma,vel):
    x = x + vel*cos(psi)*cos(gamma)
    y = y + vel*sin(psi)*cos(gamma)
    z = z + vel*(-sin(gamma))
    return x, y, z



def main():
    print __doc__
    iter_num = 0
    MapSize = 500
    MaxSteps = 700
    xpos, ypos, zpos = 0,0,0
    xgoal, ygoal,zgoal = 500,500,500
    ObstacleNumber = 10
    RadiusOfInfluence = 50 #If obstacles are 20, decrease RadiusOfInfluence
    RadiusofObstacle = 2
    alpha = 1
    beta = 0.8
    AttractivePotential = 10
    RadiusOfGoal = 1
    xobs = MapSize*np.random.rand(ObstacleNumber)
    yobs = MapSize*np.random.rand(ObstacleNumber)
    zobs = MapSize*np.random.rand(ObstacleNumber)
    
    # We have to print obstacle, goal and initial position together
    robot_x = xpos
    robot_y = ypos
    robot_z = zpos
    GoalError_x = xgoal - robot_x
    GoalError_y = ygoal - robot_y
    GoalError_z = zgoal - robot_z
    robot_pose=(robot_x,robot_y,robot_z)
    robot_vel = robot_pose

    arr = arrow(pos=robot_pose,color=color.cyan,velocity=robot_vel)
    

    traj = []
    fig = plt.gcf()

    TotalError = np.linalg.norm([GoalError_x,GoalError_y,GoalError_z]) 
    #print TotalError
    while(iter_num < MaxSteps) & (TotalError>1.1):
    	rep_mat = []
        for i in range(ObstacleNumber):
            xobs[i],yobs[i],zobs[i] = obstacle_maker(xobs[i],yobs[i],zobs[i],-pi*i/(ObstacleNumber),-pi*i/(ObstacleNumber),0)
        #print xobs,yobs,zobs
    	
        for i in range(len(xobs)):
    		[dist, psi, gamma] = dist_calc(robot_x,robot_y,robot_z,xobs[i],yobs[i],zobs[i])
    		rep_mat.append([dist, psi, gamma])
    	#print rep_mat

    	field_rep = []
    	for i in range(ObstacleNumber):
    		[delx,dely,delz] = repulsive(rep_mat[i][0], rep_mat[i][1], rep_mat[i][2], RadiusOfInfluence, RadiusofObstacle, beta)
    		field_rep.append([delx,dely,delz])
    	#print field_rep

    	att_mat = []

    	[dist, psi, gamma] = dist_calc(robot_x,robot_y,robot_z,xgoal,ygoal,zgoal)
    	att_mat = [dist, psi, gamma]
    	#print att_mat

    	[delx, dely, delz] = attractive(att_mat[0],att_mat[1],att_mat[2],AttractivePotential,RadiusOfGoal,alpha)
    	final_field = [delx,dely,delz]
    	field_rep.append(final_field)
    	final_field = field_rep
        print final_field

    	fin = []

    	for column in range(3):
    		a = sum(row[column] for row in final_field)
    		fin.append(a)
    	#print fin

    	robot_x, robot_y,robot_z= draw_robot(fin[0],fin[1],fin[2],robot_x,robot_y,robot_z)
    	#print robot_x, robot_y
        traj.append([robot_x,robot_y,robot_z])     

    	GoalError_x = xgoal - robot_x
        GoalError_y = ygoal - robot_y
        GoalError_z = zgoal - robot_z
        TotalError = np.linalg.norm([GoalError_x,GoalError_y,GoalError_z])
        #print TotalError

        ax = Axes3D(plt.gcf())
        #plt.plot(xobs,yobs,'bo',robot_x,robot_y,'ro')
        #plt.plot(xpos,zpos,'rx',xgoal,zgoal,'g^',xobs,zobs,'bo',robot_x,robot_z,'ro')
        ax.scatter(xpos,ypos,zpos,c='b',marker='o')
        ax.scatter(xgoal,ygoal,zgoal,c='g',marker='^')
        ax.scatter(xobs,yobs,zobs,c='r',marker='*')
        ax.plot3D([robot_x], [robot_y],robot_z,c='b',marker='o')
        for i in range(iter_num):
            ax.scatter(traj[i][0], traj[i][1], traj[i][2],c='g',marker='^')

    	plt.axis([-50,550,-50,550])
        #plt.setp(plt.gca(),rotation=90)
        ax.set_zlim(-50,550)
        ax.azim=-145
        ax.elev=35
        rate(100)
        arr.pos.x = robot_x
        arr.pos.y = robot_y
        arr.pos.z = robot_z
    	
        plt.ion()
        plt.pause(0.000001)
        iter_num = iter_num + 1
       
if __name__ == '__main__':
    main()