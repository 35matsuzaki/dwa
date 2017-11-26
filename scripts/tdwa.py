#!/usr/bin/python
# -*- coding: utf-8 -*-

import random
import math
import copy
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

x = [0.0, 0.0, 0.0, 0.0, 0.0] #robot initial state [x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]

fig = plt.figure()
ax1 = fig.add_subplot(121)
ax2 = fig.add_subplot(122)

goal = [10, 10] #goal position [x(m),y(m)]

goalR  = 1.0
path=[[]]
obstacle = [
    [5, 9, 1],
    [8, 8, 1],
    [8, 9, 1],
    [7, 9, 1],
]  # [x,y,size]
obstacleR = 1.0 #obstacle tolerance
dt = 0.1 #duration time [s]
# preT = 0.3 #prediction time #TODO use relative velocity

def toRadian(degree):
    return degree/180*math.pi

def toDegree(radian):
    return radian/math.pi*180

def drange(begin, end, step):
    n = begin
    while n <= end: #FIXME while n+step < end
        yield n
        n += step

model=[0.7,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1.0)];
#model:Kinematic
#[max_velocity[m/s],max_angular_velocity[rad/s],max_acc[m/ss], max_angular_acc[rad/ss],
# vel_resolution[m/s],angular_vel_resolution[rad/s]]

evalParam=[0.1, 0.5, 0.2, 3.0];
#[heading,dist,velocity,predictSimTime]

area=[-2,15,-2,15]
#range of demo area
area_dw=[-1,1,-0.5,1.5]

def DynamicWindowApproach(x):
    Vr, preVr = CalcDynamicWindow(x)
    evalDB, trajDB = Evaluation(x, Vr)
    if evalDB.size == 0:
        print "no path to goal!!"
        u=[0,0]
        return u, trajDB
    evalDB = NormalizeEval(evalDB)

    feval=[] # final Evaluation
    for i in range(len(evalDB)):
        # print "eval:",i,evalDB[i]
        #evalDB[i]: [vt,ot,heading,dist,vel]
        feval.append(evalParam[0]*evalDB[i][2] + evalParam[1]*evalDB[i][3] + evalParam[2]*evalDB[i][4])
        # print "Evaluation Value: ", feval[-1]
        # print "[heading, obdist, vel=", [evalParam[0]*evalDB[i][2], evalParam[1]*evalDB[i][3], evalParam[2]*evalDB[i][4]]

    maxIndex = feval.index(max(feval))
    u = [evalDB[maxIndex][0],evalDB[maxIndex][1]]

    # print "max_eval:",maxIndex,evalDB[maxIndex]
    # print "u:" , u

    return u, trajDB

def Evaluation(x, Vr):
    evalDB= np.array([[0,0,0,0,0]])
    trajDB= [[]]#trajectory[[]] database

    for vt in drange(Vr[0], Vr[1], model[4]):
        for ot in drange(Vr[2], Vr[3], model[5]):
            # print vt, ot
            #predict trajectory
            xt, traj = GenerateTrajectory(x, vt, ot, evalParam[3])
            #calculate earch cost function
            heading=CalcHeadingEval(xt)
            dist=CalcDistEval(xt)
            vel=vt

            eval=np.array([[vt, ot, heading, dist, vel]])
            evalDB=np.concatenate((evalDB,eval), axis=0)
            trajDB.append(traj)


    evalDB = np.delete(evalDB,0,0)#delete initial
    del trajDB[0]#delete initial

    return evalDB, trajDB

def NormalizeEval(EvalDB):
    #EvalDB=np.array[earch trajectory][[vt,ot,heading,dist,vel]]
    sumArray=np.sum(EvalDB, axis=0)
    if sumArray[2] != 0:
        EvalDB=EvalDB /[1,1,sumArray[2],1,1]
    if sumArray[3] != 0:
        EvalDB=EvalDB /[1,1,1,sumArray[3],1]
    if sumArray[4] != 0:
        EvalDB=EvalDB /[1,1,1,1,sumArray[4]]
    return EvalDB

def GenerateTrajectory(x,vt,ot,evaldt):
    time=0.0
    u=[vt, ot]
    traj = [[]]
    traj.append(x)
    while time<=evaldt:
        time=time+dt
        x=f(x,u)
        traj.append(x)
        #TODO collision check here?

    del traj[0] #delete initial
    return x, traj

def CalcBreakingDist(vel):
    #TODO use this function
    stopDist=0
    while vel>0:
        stopDist=stopDist+vel*dt;
        vel=vel-model[2]*dt;
    return stopDist

# def CalcDistEval(x):
#     dist = 2.0 #FIXME
#     for i in range(len(obstacle)):
#         disttmp=math.sqrt((obstacle[i][0] - x[0])*(obstacle[i][0] - x[0]) + (obstacle[i][1] - x[1])*(obstacle[i][1] - x[1]) ) - obstacleR
#         #TODO calculate robot and obstacle size
#         if dist>disttmp:
#             dist=disttmp
#
#     return dist

def CalcDistEval(x):
    dist = 2.0 #FIXME
    predictObstacle = list(obstacle)
    ob_vel = -0.8
    preT = evalParam[3]
    # preT = 0.0
    for i in range(len(obstacle)):
        predictObstacle[i] = [predictObstacle[i][0]+preT*ob_vel, predictObstacle[i][1]+preT*ob_vel, predictObstacle[i][2]]
        disttmp=math.sqrt((predictObstacle[i][0] - x[0])*(predictObstacle[i][0] - x[0]) + (predictObstacle[i][1] - x[1])*(predictObstacle[i][1] - x[1]) ) - obstacleR

        #TODO calculate robot and obstacle size
        if dist>disttmp:
            dist=disttmp

    return dist

def CalcObstacleEval(x,preT):
    minDist = 2.0 #FIXME
    ob_vel = -0.8
    predictTime = 1.0 #TODO use relative velocity between robot and an obstacle
    duration = dt
    predictObstacle = list(obstacle)
    #for t in dragne(0.0, predictTime, duration):
    for i in range(len(predictObstacle)):
        predictObstacle[i] = [predictObstacle[i][0]+preT*ob_vel, predictObstacle[i][1]+preT*ob_vel, predictObstacle[i][2]]
        disttmp=math.sqrt((predictObstacle[i][0] - x[0])*(predictObstacle[i][0] - x[0]) + (predictObstacle[i][1] - x[1])*(predictObstacle[i][1] - x[1]) ) - obstacleR
        #TODO calculate robot and obstacle size
        if minDist>disttmp:
            minDist=disttmp



def CalcHeadingEval(x):#TODO change degree to radian
    #calculate goal heading evaluation
    theta=toDegree(x[2])
    goalTheta=toDegree(math.atan2(goal[1]-x[1], goal[0]-x[0]))

    if goalTheta > theta:
        targetTheta=goalTheta-theta
    else:
        targetTheta=theta-goalTheta
    heading=180.0-targetTheta

    #calculate goal distance evaluation
    dist = (goal[0] - x[0])*(goal[0] - x[0]) + (goal[1] - x[1])*(goal[1] - x[1])
    # dist = math.sqrt((goal[0] - x[0])*(goal[0] - x[0]) + (goal[1] - x[1])*(goal[1] - x[1]))
    # dist = distStartTOGoal - math.sqrt(dist)

    return heading+1.0/dist #TODO use some function to calculate dist cost

def CalcDynamicWindow(x):
    #TODO collision check
    #window1: input range
    Vs=[0, model[0], -model[1], model[1]];#TODO add backward to Vs[0]
    #window2: kinematic constraints
    Vd=[x[3]-model[2]*dt, x[3]+model[2]*dt, x[4]-model[3]*dt, x[4]+model[3]*dt]
    #window: intersection window1 and window2
    Vr=[max(Vs[0],Vd[0]), min(Vs[1],Vd[1]), max(Vs[2],Vd[2]), min(Vs[3],Vd[3])]

    preT = 0.3
    preVd=[x[3]-model[2]*preT, x[3]+model[2]*preT, x[4]-model[3]*preT, x[4]+model[3]*preT]
    preVr=[max(Vs[0],preVd[0]), min(Vs[1],preVd[1]), max(Vs[2],preVd[2]), min(Vs[3],preVd[3])]


    DrawDynamicWindow(Vs, Vd, Vr, preVd, preVr, x)

    return Vr, preVr

def f(x,u):
    #x = [x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
    #u = [v(m/s),w(rad/s)]
    x_ = list(x)
    x_[0] = x[0] + dt * math.cos(x[2]) * u[0]
    x_[1] = x[1] + dt * math.sin(x[2]) * u[0]
    x_[2] = x[2] + dt * u[1]
    x_[3] = u[0]
    x_[4] = u[1]

    return x_

def MoveObstacle():
    global  obstacle
    ob_vel = -0.8
    for i in range(len(obstacle)):
        obstacle[i] = [obstacle[i][0]+dt*ob_vel, obstacle[i][1]+dt*ob_vel, obstacle[i][2]]


def DrawDynamicWindow(Vs, Vd, Vr, preVd, preVr, x):
    ax2.cla()
    VsPoly = plt.Polygon(((Vs[2],Vs[0]),(Vs[2],Vs[1]),(Vs[3],Vs[1]),(Vs[3],Vs[0])),fc="red",alpha=0.2)
    ax2.add_patch(VsPoly)

    VdPoly = plt.Polygon(((Vd[2],Vd[0]),(Vd[2],Vd[1]),(Vd[3],Vd[1]),(Vd[3],Vd[0])),fc="lime",alpha=0.7)
    ax2.add_patch(VdPoly)

    VrPoly = plt.Polygon(((Vr[2],Vr[0]),(Vr[2],Vr[1]),(Vr[3],Vr[1]),(Vr[3],Vr[0])),fc="blue",alpha=0.7)
    ax2.add_patch(VrPoly)

    #preVdPoly = plt.Polygon(((preVd[2],preVd[0]),(preVd[2],preVd[1]),(preVd[3],preVd[1]),(preVd[3],preVd[0])),fc="lime",alpha=0.2)
    #ax2.add_patch(preVdPoly)

    #preVrPoly = plt.Polygon(((preVr[2],preVr[0]),(preVr[2],preVr[1]),(preVr[3],preVr[1]),(preVr[3],preVr[0])),fc="blue",alpha=0.2)
    #ax2.add_patch(preVrPoly)


    #plot robot input
    ax2.plot(x[4],x[3], 'ro')

    ax2.text(Vs[2]-0.05, Vs[1]+0.01,'Vs')
    ax2.text(Vd[2]-0.05, Vd[1]+0.01,'Vd')
    ax2.text(Vr[2]-0.05, Vr[1]+0.01,'Vr')
    #ax2.text(preVd[2]-0.05, preVd[1]+0.01,'preVd')
    #ax2.text(preVr[2]-0.05, preVr[1]+0.01,'preVr')

    ax2.set_title('Dynamic Window')
    ax2.set_xlabel('w [rad/s]')
    ax2.set_ylabel('v [m/s]')
    ax2.axis(area_dw)
    ax2.grid(True)
    ax2.set_aspect('equal', adjustable='box')
    plt.tight_layout()

def DrawGraph(x, trajDB, path):
    ax1.cla()
    ax1.plot([ox for (ox,oy,size) in obstacle],[oy for (ox,oy,size) in obstacle], "ok", ms=size * 20)
    predictObstacle = list(obstacle)
    ob_vel = -0.8
    for i in range(len(predictObstacle)):
        predictObstacle[i] = [predictObstacle[i][0]+evalParam[3]*ob_vel, predictObstacle[i][1]+evalParam[3]*ob_vel, predictObstacle[i][2]]
        ax1.plot(predictObstacle[i][0],predictObstacle[i][1], 'bo', ms=10, alpha=0.5)


    ax1.plot(goal[0],goal[1], 'ro', ms=goalR*20 )
    ax1.plot(x[0],x[1], 'bo', ms=10)

    #plot trajectory
    for i in range(len(trajDB)):
        ax1.plot([point[0] for point in trajDB[i]],[point[1] for point in trajDB[i]], 'g')
        ax1.plot(point[0],point[1], 'r,') #plot end points

    #plot passed path
    ax1.plot([point[0] for point in path],[point[1] for point in path], 'y')

    ax1.set_title('Demo')
    ax1.set_xlabel('x [m]')
    ax1.set_ylabel('y [m]')
    ax1.axis(area)
    ax1.grid(True)
    ax1.set_aspect('equal', adjustable='box')
    # plt.pause(0.01)

def run(count):
    global x, path

    if count == 0:
        del path[0] #delete initial

    path.append(x)
    # for i in range(5000):
    # print i,x
    u,trajDB=DynamicWindowApproach(x)
    x=f(x,u)#move robot using kinematic
    path.append(x)
    MoveObstacle()

    if math.sqrt((goal[0] - x[0])*(goal[0] - x[0])+(goal[1] - x[1])*(goal[1] - x[1])) < 0.5:
        print "Arrive Goal"
    DrawGraph(x, trajDB, path)




if __name__ == '__main__':

    ani = animation.FuncAnimation(fig, run, interval=10)
    plt.show()
