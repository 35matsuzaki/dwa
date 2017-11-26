#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

import math
import numpy as np

x = [0.0, 0.0, 0.0, 0.0, 0.0] #robot initial state [x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]

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
hz = 10 #rospy.Rate(hz) [hz]
dt = 1.0/hz #duration time [s]

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
#[max_velocity[m/s],max_angular_velocity[rad/s],max_acc[m/ss], max_angular_acc[rad/ss],
# vel_resolution[m/s],angular_vel_resolution[rad/s]]

evalParam=[0.1, 0.5, 0.2, 3.0];
#[heading,dist,velocity,predictSimTime]

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
    duration = 0.1
    while time<=evaldt:
        time=time+duration
        x=f(x,u)
        traj.append(x)

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

    return heading+1.0/dist

def CalcDynamicWindow(x):
    #TODO collision check
    #window1: input range
    Vs=[0, model[0], -model[1], model[1]];#TODO add backward to Vs[0]
    # Vs=[-model[0], model[0], -model[1], model[1]];#TODO add backward to Vs[0]

    #window2: kinematic constraints
    Vd=[x[3]-model[2]*dt, x[3]+model[2]*dt, x[4]-model[3]*dt, x[4]+model[3]*dt]
    # Vd=[x[3]-model[2]*dt*4, x[3]+model[2]*dt, x[4]-model[3]*dt, x[4]+model[3]*dt]

    #window: intersection window1 and window2
    Vr=[max(Vs[0],Vd[0]), min(Vs[1],Vd[1]), max(Vs[2],Vd[2]), min(Vs[3],Vd[3])]

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


def run():
    global x, path

    # pub = rospy.Publisher('/cmd_vel_src/nav', Twist, queue_size=1)
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
        u,trajDB=DynamicWindowApproach(x)
        x=f(x,u)#move robot using kinematic
        path.append(x)
        MoveObstacle()
        msg = Twist()

        if math.sqrt((goal[0] - x[0])*(goal[0] - x[0])+(goal[1] - x[1])*(goal[1] - x[1])) < 0.5:
            rospy.loginfo("Goal reached: %s" % x)
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            break

        msg.linear.x = u[0]
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = u[1]
        pub.publish(msg)
        rate.sleep()
        rospy.loginfo("Current state: %s" % x)



if __name__ == '__main__':
    run()
