import numpy as np
from math import *
import matplotlib.pyplot as plt
import time
from pylab import *
from matplotlib.patches import FancyArrowPatch

def dist_calc(x1, y1, x2, y2):
    global dx, dy, theta
    dx = x1-x2
    dy = y1-y2
    dist = linalg.norm([dx,dy])
    theta = atan2(y2-y1,x2-x1)
    return dist, theta

def attractive(d,theta,s,r,alpha):
    if d < r:
        delx = 0
        dely = 0
    elif r <=d <=s+r:
        delx = alpha*(d-r)*cos(theta)
        dely = alpha*(d-r)*sin(theta)
    elif d > s + r:
        delx = alpha*s*cos(theta)
        dely = alpha*s*sin(theta)
    return delx, dely

def repulsive(d,theta,s,r,beta):
    if d < r:
        delx = -np.sign(cos(theta))*float("inf")
        dely = -np.sign(cos(theta))*float("inf")
    elif r <=d <=s+r:
        delx = -beta*(s+r-d)*cos(theta+pi/2)
        dely = -beta*(s+r-d)*sin(theta+pi/2)
    elif d > s + r:
        delx = 0
        dely = 0
    return delx, dely

def draw_robot(delx,dely,robot_x,robot_y):
    #Ideally you should be using OOP so self for coordinates will be useful
    robot_x = robot_x  + delx
    robot_y = robot_y +  dely
    return robot_x, robot_y

def obstacle_maker(x,y,psi,vel):
    x = x + vel*0.2*cos(psi)
    y = y + vel*0.2*sin(psi)
    return x, y
    

def main():
    print __doc__
    iter_num = 0
    MapSize = 500
    MaxSteps = 700
    xpos, ypos = 0,0
    xgoal, ygoal = 500,500
    ObstacleNumber = 5
    RadiusOfInfluence = 40
    RadiusofObstacle = 2
    alpha = 1
    beta = 0.8
    AttractivePotential = 10
    RadiusOfGoal = 1
    xobs = (MapSize/2)*np.random.rand(ObstacleNumber)
    yobs = (MapSize/2)*np.random.rand(ObstacleNumber)
    # We have to print obstacle, goal and initial position together
    robot_x = xpos
    robot_y = ypos
    robot_v = 0
    robot_theta = 0
    GoalError_x = xgoal - robot_x
    GoalError_y = ygoal - robot_y
    traj = []
    fig, ax = plt.subplots()

    TotalError = np.linalg.norm([GoalError_x,GoalError_y]) 
    while(iter_num < MaxSteps) & (TotalError>1.1):
        rep_mat = []

        for i in range(ObstacleNumber):
            xobs[i],yobs[i] = obstacle_maker(xobs[i],yobs[i],(pi*i)/(ObstacleNumber),0)



        for i in range(len(xobs)):
            [dist, theta] = dist_calc(robot_x,robot_y,xobs[i],yobs[i])
            rep_mat.append([dist, theta])
        #print rep_mat

        field_rep = []
        for i in range(ObstacleNumber):
            [delx, dely] = repulsive(rep_mat[i][0], rep_mat[i][1], RadiusOfInfluence, RadiusofObstacle, beta)
            field_rep.append([delx,dely])
        print field_rep

        att_mat = []

        [dist, theta] = dist_calc(robot_x,robot_y,xgoal,ygoal)
        att_mat = [dist, theta]

        [delx, dely] = attractive(att_mat[0],att_mat[1],AttractivePotential,RadiusOfGoal,alpha)
        final_field = [delx,dely]
        field_rep.append(final_field)
        final_field = field_rep

        fin = []

        for column in range(2):
            a = sum(row[column] for row in final_field)
            fin.append(a)

        robot_x, robot_y= draw_robot(fin[0],fin[1],robot_x,robot_y)

        traj.append([robot_x,robot_y])
        GoalError_x = xgoal - robot_x
        GoalError_y = ygoal - robot_y
        TotalError = np.linalg.norm([GoalError_x,GoalError_y])

        plt.plot(xpos,ypos,'rx',xgoal,ygoal,'go',robot_x,robot_y,'ro')
        plt.axis([-50,550,-50,550])
        plt.hold(True)

        for i in range(iter_num):
            plt.plot(traj[i][0], traj[i][1],c='r',marker='^')


        plt.plot(xobs,yobs,'b*')
        plt.hold(False)
        plt.draw()

        
        plt.pause(0.01)
        iter_num = iter_num + 1
       
if __name__ == '__main__':
    main()
