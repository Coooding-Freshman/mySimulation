#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue May 10 13:09:38 2016

@author: Li zh
"""

import copy
import numpy as np
import matplotlib.pyplot as plt
#from scipy.integrate import odeint

numOfGroup1=4
numOfGroup2=4

times=10
step=times*100
S10=[1.0,1.0]
S20=[2.0,2.0]
A=np.array([[0,0.5],[0.5,0]])
B=np.array([[0,-0.05],[-0.05,0.0]])
C=np.array([[0.0,0.01],[0.01,0.0]])
D=np.array([[0.005,0.0],[0.0,0.01]])
sigma=0.8
xx=[[0,1,0,1],[1,0,0,0],[0,0,0,1],[1,0,1,0]]
yy=xx
xy=np.array([[0.5,0.5,0,0],[0,0.5,0.5,0],[0,0,0.5,0.5],[0.5,0,0,0.5]])
yx=np.transpose(xy)
h=1
H=h*np.eye(2)
q=1


def updateA(t):
    A[0,0]=0.99#+0.02*np.sin(t/100)
    A[0,1]=0.0015#*np.cos(t/100)
    A[1,0]=0.001#+0.001*np.cos(t/100)
    A[1,1]=0.97 #+0.02*np.sin(t/100)
    C[0,0]=0.0012#+np.cos(t/100)/50
    C[1,1]=0.0015#-np.sin(t/100)/100

def updateB(t):
    B[0,0]=0.98#+0.01*np.cos(t/100)
    B[0,1]=0.02
    B[1,0]=0.02
    B[1,1]=0.95#+0.01*np.cos(t/100)
    D[0,0]=0.001#+np.sin(t/100)/60
    D[1,1]=0.001#-np.cos(t/100)/90

def targetDynamical(para,t):
    updateA(t)
    updateB(t)
    x_k1=targetDynamical1(para)
    x_k2=targetDynamical2(para)
    return np.concatenate((x_k1,x_k2))

def targetDynamical1(para):
    ret=np.dot(A,para[0:2])+np.dot(C,para[2:4])
    return ret

def targetDynamical2(para):
    ret=np.dot(B,para[2:4])+np.dot(D,para[0:2])
    return ret

class sensor():
    tag=0
    def __init__(self,flag):
        if flag==1:
            self.x=float(20*(np.random.random()-0.5))+100
            self.y=float(20*(np.random.random()-0.5))+80
        else:
            self.x=float(20*(np.random.random()-0.5))+30
            self.y=float(20*(np.random.random()-0.5))+50
        self.flag=flag                                                      #属于哪个组的标示
        self.tag=sensor.tag                                                 #组内的号码
        self.record=np.array([np.array([0.0,0.0]) for i in range(step)])
        self.record[0]=np.array([self.x,self.y])
        self.send=np.array([np.array([0.0,0.0]) for i in range(step)])
        self.send[0]=self.record[0]
        if self.flag==1:
            self.dynamic=targetDynamical1
        else:
            self.dynamic=targetDynamical2
        self.i=0
        self.k=np.eye(2)*0.25
        self.sendk=copy.deepcopy(self.k)
        self.p=40*np.eye(2)
        self.sendp=copy.deepcopy(self.p)
        self.pij=np.array([40*np.array(np.eye(2)) for i in range(numOfGroup1)])
        sensor.tag+=1
        self.counter=0

    def myfilter(self,group,step,location):
        updateA(step)
        updateB(step)
        thissum=np.array([0.0,0.0])
        othersum=np.array([0.0,0.0])
        for agent in group:
            if agent.flag==self.flag and self.cal_distance(agent):
                temp=agent.send[self.i]-self.record[self.i]
                thissum+=temp
            if agent.flag!=self.flag:
                othersum+=agent.send[self.i]*self.cal_distance(agent)

        if self.flag==1:
            du=self.dynamic(np.concatenate((np.array([self.x,self.y]),othersum)))
        else:
            du=self.dynamic(np.concatenate((othersum,np.array([self.x,self.y]))))
       # change 0.25 to K
        if self.flag==1:
            observe=h*np.array(location[0:2])+np.random.randn(1,2)*q
        else:
            observe=h*np.array(location[2:4])+np.random.randn(1,2)*q
        temp=[observe[0,0]-h*self.x,observe[0,1]-h*self.y]

        self.k=0.5*np.eye(2)
        #if self.i<50:
        #    print observe,du,thissum,temp
        if self.flag==1:
            ret=np.dot(self.k,np.array(temp))+du+0.02*np.dot(A,thissum)#+0.25*np.array(temp)
        else:
            ret=np.dot(self.k,np.array(temp))+du+0.02*np.dot(B,thissum)#+0.25*np.array(temp) self.update(ret)
        if self.flag==1 and self.tag==1:
            print ret-observe[0:2]
        self.update(ret)
        self._updataPK(group)
        return ret

    def _updataPK(self,group):
        sump=np.array([[0.0,0.0],[0.0,0.0]])
        sumpij=np.array([[0.0,0.0],[0.0,0.0]])
        add_term=0
        g_ij=0
        if self.flag==1:
            add_term=np.dot(A-np.dot(self.k,H),A.transpose())+np.dot(A,(A-np.dot(self.k,H).transpose()))
        if self.flag==2:
            add_term=np.dot(B-np.dot(self.k,H),B.transpose())+np.dot(B,(B-np.dot(self.k,H).transpose()))
        for agent in group:
            if agent.flag != self.flag and self.flag==1:
                sump+=xy[self.tag][agent.tag]*agent.sendp
                sumpij+=xy[self.tag][agent.tag]*self.pij[agent.tag]
                if xy[self.tag][agent.tag]!=0:
                    temp1=A-np.dot(self.k,H)
                    temp2=B-np.dot(agent.sendk,H)
                    self.pij[agent.tag]=np.dot(np.dot(temp1,self.pij[agent.tag]),temp2)+np.dot(np.dot(C,self.pij[agent.tag]),D)+sigma
            if agent.flag != self.flag and self.flag==2:
                sump+=yx[self.tag][agent.tag]*agent.sendp
                sumpij+=yx[self.tag][agent.tag]*self.pij[agent.tag]
                if yx[self.tag][agent.tag]!=0:
                    temp1=B-np.dot(self.k,H)
                    temp2=A-np.dot(agent.sendk,H)
                    self.pij[agent.tag]=np.dot(np.dot(temp1,self.pij[agent.tag]),temp2)+np.dot(np.dot(D,self.pij[agent.tag]),C)+sigma
            if agent.flag==self.flag and self.cal_distance(agent):
                g_ij+=0.02
        ansp=np.zeros((2,2))
        ansk=np.zeros((2,2))
        inv=np.dot(np.dot(H,self.p),H)+np.eye(2)*q
        if self.flag==1:
            ansk=(np.dot(np.dot(A,self.p),H)+np.dot(np.dot(C,sumpij),H)+sigma*g_ij*np.dot(A,H))*np.linalg.inv(inv)
            temp=A-np.dot(self.k,H)
            ansp=np.dot(np.dot(temp,self.p),temp.transpose())+np.dot(self.k,self.k)*q+np.dot(np.dot(C,sump),C.transpose())+g_ij*add_term
        else:
            ansk=(np.dot(np.dot(B,self.p),H)+np.dot(np.dot(D,sumpij),H)+sigma*g_ij*np.dot(B,H))*np.linalg.inv(inv)
            temp=B-np.dot(self.k,H)
            ansp=np.dot(np.dot(temp,self.p),temp.transpose())+np.dot(self.k,self.k)*q+np.dot(np.dot(D,sump),D.transpose())+g_ij*add_term
        self.k=ansk
        if self.tag==1 and self.flag==1:
        #    print self.pij
            print self.p
        self.p=ansp

    def cal_distance(self,agent):
        if self.flag==agent.flag:
            if self.flag==1:
                return xx[self.tag][agent.tag]
            else:
                return yy[self.tag][agent.tag]
        else:
            if self.flag==1:
                return xy[self.tag][agent.tag]
            else:
                return yx[self.tag][agent.tag]

    def _tigger_function(self, old, new):
        if sum((old-new)**2)>=sigma:
            self.counter+=1
            return True
        else:
            return False

    def update(self,du):
        self.i+=1
        if self._tigger_function(self.send[self.i-1],du):
            self.send[self.i]=du
            self.sendp=copy.deepcopy(self.p)
            self.sendk=copy.deepcopy(self.k)
        else:
            self.send[self.i]=self.send[self.i-1]
        self.x=du[0]
        self.y=du[1]
        self.record[self.i]=du

if __name__=="__main__":
    time_range=np.arange(0,times,0.01)
    location=np.array([[100.0,80.0,30.0,50.0] for i in range(step)])
    for i in range(step-1):
        location[i+1]=targetDynamical(location[i],i)
    #print location
    group1=[sensor(1) for i in range(numOfGroup1)]
    sensor.tag=0
    group2=[sensor(2) for i in range(numOfGroup2)]
    wholeGroup=group1+group2

    #print location[step-1]
    for i in range(step-1):
        for agent in wholeGroup:
            agent.myfilter(wholeGroup,i,location[i+1])
    #print group1[0].counter
    #print location[step-1]
    #print '------------'
    #print group1[0].record
    #print '------------'
    #print group1[0].send
    plt.plot(location[1:1000,0],location[1:1000,1],color='g',label="location of x")
    plt.plot(location[1:1000,2],location[1:1000,3],color='b',label="location of y")
#    plt.plot(group1[0].record[:,0],group1[0].record[:,1])
    for i in range(numOfGroup1):
        plt.plot(group1[i].record[:1000,0],group1[i].record[:1000,1])
    for i in range(numOfGroup2):
        plt.plot(group2[i].record[:1000,0],group2[i].record[:1000,1])
    print location[step-1]
    ansx=[wholeGroup[i].record[0,0] for i in range(numOfGroup1)]
    ansy=[wholeGroup[i].record[0,1] for i in range(numOfGroup1)]
    plt.scatter(ansx,ansy,color='r')

    init_x=[wholeGroup[i+4].record[0,0] for i in range(numOfGroup2)]
    init_y=[wholeGroup[i+4].record[0,1] for i in range(numOfGroup2)]
    plt.scatter(init_x,init_y,color='r')
    plt.title("location of targets and sensors")
    plt.legend(loc="upper left")
    #plt.plot(wholeGroup[0].record[:,0],wholeGroup[0].record[:,1])
    plt.show()
    error=np.array([0.0 for i in range(step)])

    dic={0:r"$\sigma=0.4$",1:r"$\sigma=0.6$",2:r"$\sigma=0.8$"}
    plt.figure("MSE")
    plt.title(r"mean square error at {}".format(dic[2]))
    plt.ylim((0,10))
    plt.xlabel("steps")
    plt.ylabel("MSE")
    for j in range(step):
        error[j]=sum([sum((group1[i].record[j]-location[j,0:2])**2) for i in range(numOfGroup1)])/numOfGroup1
    plt.plot(range(step),error,label="x-group")
    for j in range(step):
        error[j]=sum([sum((group2[i].record[j]-location[j,2:4])**2) for i in range(numOfGroup2)])/numOfGroup2
    plt.plot(range(step),error,label="y-group")
    plt.legend()
    plt.show()
    tigger_counter=[]
    for i in range(numOfGroup1+numOfGroup2):
        tigger_counter.append(wholeGroup[i].counter)
    plt.figure("triggered_times")
    plt.title("triggered_times")
    plt.grid()
    plt.xlabel("Node Number")
    plt.ylabel("Times")
    plt.plot(range(1,numOfGroup1+numOfGroup2+1),tigger_counter)
    plt.show()
#    plt.figure()
#    plt.plot(time_range,track[:,3])
#    for i in range(numOfGroup2):
#        plt.plot(time_range,wholeGroup[numOfGroup1+i].record[:,1])
#    plt.show()
