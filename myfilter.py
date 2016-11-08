#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue May 10 13:09:38 2016

@author: Li zh
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

numOfGroup1=6
numOfGroup2=6

times=20
step=times*100
S10=[1.0,1.0]
S20=[2.0,2.0]
A=np.array([[0,0.5],[0.8,0]])
B=np.array([[0,-0.3],[0.1,0.0]])
C=np.array([[0.0,-0.1],[0.1,0.0]])
D=np.array([[-0.16,0.0],[0.1,0.0]])
radius=20.0
def targetDynamical(para,t):
    d1=np.dot(A,np.array([np.cos(para[0]),np.sin(para[1])]))+np.dot(C,para[2:4])
    d2=np.dot(B,np.array([np.cos(para[2]),np.sin(para[3])]))+np.dot(D,para[0:2])
    return np.concatenate((d1,d2))

def targetDynamical1(para):
    d1=np.dot(A,np.array([np.cos(para[0]),np.sin(para[1])]))+np.dot(C,para[2:4])
    return d1

def targetDynamical2(para):
    d2=np.dot(B,np.array([np.cos(para[0]),np.sin(para[1])]))+np.dot(D,para[2:4])
    return d2

class sensor():
    tag=1
    def __init__(self,flag):
        self.x=float(10*(np.random.random())-5)
        self.y=float(10*(np.random.random())-5)
        self.dx=float(10*(np.random.random())-5)
        self.dy=float(10*(np.random.random())-5)
        self.flag=flag
        self.tag=sensor.tag
        self.record=np.array([np.array([0.0,0.0]) for i in range(step)])
        self.record[0]=np.array([self.x,self.y])
        self.record_vel=np.array([np.array([0.0,0.0]) for i in range(step)])
        self.record_vel[0]=np.array([self.dx,self.dy])
        if self.flag==1:
            self.dynamic=targetDynamical1
        else:
            self.dynamic=targetDynamical2
        self.i=0
        sensor.tag+=1

    def myfilter(self,group,step,target,local):
        thissum=np.array([0.0,0.0])
        du=np.array([0.0,0.0])
        othersum=np.array([0.0,0.0])
        num=0
        for agent in group:
            if agent.flag==self.flag:
                if self.cal_distance(agent.x,agent.y):                        #noise=np.random.randn(2,1)*5
                    temp=agent.record_vel[self.i]-np.array([self.dx,self.dy])
                    thissum+=np.array(temp)
            else:
                if self.cal_distance(agent.x,agent.y):
                    othersum+=agent.record_vel[self.i]
                    num+=1

        if num!=0:
            othersum/=num
            du=self.dynamic(np.concatenate((np.array([self.dx,self.dy]),othersum)))

        if self.flag==1:
            observe1=2*np.array(target[0:2])+np.random.randn(1,2)*10
            observe2=2*np.array(local[0:2])+np.random.randn(1,2)*10
        else:
            observe1=2*np.array(target[2:4])+np.random.randn(1,2)*10
            observe2=2*np.array(local[2:4])+np.random.randn(1,2)*10
        temp1=[observe1[0,0]-2*self.dx,observe1[0,1]-2*self.dy]
        temp2=[observe2[0,0]-2*self.x,observe2[0,1]-2*self.y]

        ret=du+thissum+np.array(temp1)+np.array(temp2)
        self.update(ret)
        return ret


    def cal_distance(self,tx,ty):
        dw=np.sqrt((self.x-tx)**2+(self.y-ty)**2)
        if dw<=radius:
            return 1
        else:
            return 0

    def update(self,du):
        self.i+=1
        self.dx+=du[0]*0.01
        self.dy+=du[1]*0.01
        self.x+=self.dx*0.01
        self.y+=self.dy*0.01
        self.record[self.i]=np.array([self.x,self.y])
        self.record_vel[self.i]=np.array([self.dx,self.dy])


if __name__=="__main__":
    time_range=np.arange(0,times,0.01)
    track=odeint(targetDynamical,S10+S20,time_range)
    local=np.array([[20.0,0.0,30.0,-20.0] for i in range(step)])
    for i in range(step-1):
        local[i+1]=local[i]+0.01*track[i]
    group1=[sensor(1) for i in range(numOfGroup1)]
    group2=[sensor(2) for i in range(numOfGroup2)]
    wholeGroup=group1+group2

    for i in range(step-1):
        for agent in wholeGroup:
            agent.myfilter(wholeGroup,i,track[i+1],local[i+1])
    plt.plot(local[:,0],local[:,1],color='g')
    plt.plot(local[:,2],local[:,3],color='b')
    ansx=[wholeGroup[i].x for i in range(numOfGroup1)]
    ansy=[wholeGroup[i].y for i in range(numOfGroup1)]
    plt.scatter(ansx,ansy,color='r')

    init_x=[wholeGroup[i].record[0,0] for i in range(numOfGroup1)]
    init_y=[wholeGroup[i].record[0,1] for i in range(numOfGroup1)]
    plt.scatter(init_x,init_y,color='r')
    #plt.plot(wholeGroup[0].record[:,0],wholeGroup[0].record[:,1])
    plt.show()

    plt.figure()
    plt.plot(time_range,track[:,3])
    for i in range(numOfGroup2):
        plt.plot(time_range,wholeGroup[numOfGroup1+i].record_vel[:,1])
    plt.show()
