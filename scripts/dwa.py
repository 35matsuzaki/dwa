#!/usr/bin/python
# -*- coding: utf-8 -*-

import random
import math
import copy
import matplotlib.pyplot as plt
import numpy as np

goal = [10, 10] #goal position [x(m),y(m)]
goalR  = 1.0
obstacle = [
    # [0, 2, 1],
    # [4, 2, 1],
    # [4, 4, 1],
    # [5, 4, 1],
    # [5, 5, 1],
    # [5, 6, 1],
    [5, 9, 1],
    [8, 8, 1],
    [8, 9, 1],
    [7, 9, 1],
]  # [x,y,size]
obstacleR = 1.0 #obstacle tolerance
dt = 0.1 #duration time [s]

def toRadian(degree):
    return degree/180*math.pi

def toDegree(radian):
    return radian/math.pi*180

def drange(begin, end, step):
    n = begin
    while n <= end: #FIXME while n+step < end
        yield n
        n += step

model=[1.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1.0)];
#model:Kinematic
#[max_velocity[m/s],max_angular_velocity[rad/s],max_acc[m/ss], max_angular_acc[rad/ss],
# vel_resolution[m/s],angular_vel_resolution[rad/s]]

evalParam=[0.1, 0.5, 0.1, 3.0];
#[heading,dist,velocity,predictSimTime]

area=[-2,15,-2,15]
#range of sim area

def DynamicWindowApproach(x):
    Vr = CalcDynamicWindow(x)
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
            vel=math.fabs(vt)

            eval=np.array([[vt, ot, heading, dist, vel]])
            evalDB=np.concatenate((evalDB,eval), axis=0)
            trajDB.append(traj)


    evalDB = np.delete(evalDB,0,0)#delete initial
    del trajDB[0]#delete initial

    return evalDB, trajDB

def NormalizeEval(EvalDB):
    #EvalDB=np.array[earch trajector][[vt,ot,heading,dist,vel]]
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

def CalcDistEval(x):
    dist = 2 #FIXME
    for i in range(len(obstacle)):
        disttmp=math.sqrt((obstacle[i][0] - x[0])*(obstacle[i][0] - x[0]) + (obstacle[i][1] - x[1])*(obstacle[i][1] - x[1]) ) - obstacleR
        #TODO calculate robot and obstacle size
        if dist>disttmp:
            dist=disttmp
    return dist

def CalcHeadingEval(x):#TODO change degree to radian
    theta=toDegree(x[2])
    goalTheta=toDegree(math.atan2(goal[1]-x[1], goal[0]-x[0]))

    if goalTheta > theta:
        targetTheta=goalTheta-theta
    else:
        targetTheta=theta-goalTheta
    heading=180.0-targetTheta

    return heading

def CalcDynamicWindow(x):
    #TODO collision check
    #window1: input range
    Vs=[0, model[0], -model[1], model[1]];#TODO add backward to Vs[0]
    #window2: kinematic constraints
    Vd=[x[3]-model[2]*dt, x[3]+model[2]*dt, x[4]-model[3]*dt, x[4]+model[3]*dt]
    #window: intersection window1 and window2
    Vr=[max(Vs[0],Vd[0]), min(Vs[1],Vd[1]), max(Vs[2],Vd[2]), min(Vs[3],Vd[3])]
    # print "x:", x
    # print "Vr:", Vr

    return Vr

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


def DrawGraph(x, trajDB):
    plt.clf()
    plt.plot([ox for (ox,oy,size) in obstacle],[oy for (ox,oy,size) in obstacle], "ok", ms=size * 20)
    plt.plot(goal[0],goal[1], 'ro', ms=goalR*20 )
    plt.plot(x[0],x[1], 'bo', ms=10)

    for i in range(len(trajDB)):
        plt.plot([point[0] for point in trajDB[i]],[point[1] for point in trajDB[i]], 'g')

    plt.axis(area)
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.pause(0.01)



if __name__ == '__main__':
    x = [0, 0, math.pi/2, 0, 0] #robot initial state [x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]

    # TestDraw(x)
    result=[[]]
    result.append(x)
    del result[0] #delete initial
    for i in range(5000):
        # print i,x
        u,trajDB=DynamicWindowApproach(x)
        x=f(x,u)#move robot using kinematic
        result.append(x)
        MoveObstacle()

        if math.sqrt((goal[0] - x[0])*(goal[0] - x[0])+(goal[1] - x[1])*(goal[1] - x[1])) < 0.5:
            print "Arrive Goal"
            break
        DrawGraph(x, trajDB)

    plt.show()
