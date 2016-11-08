# -*- coding: utf-8 -*-
"""
Created on Fri May 13 03:44:14 2016

@author: Administrator
"""

import numpy as np
import scipy.integrate as integrate
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import myfilter as mf

adjA=[[0,0,0,0]for i in range(4)]
adjB=[[0,0,0,0]for i in range(4)]
c_1=2
c_2=2
times=mf.times
steps=times*100
radius=mf.radius

a=5
b=10
c=(b-a)/np.sqrt(4*a*b)
h=0.5
alpha=10
deserve=5
def rho(z):
    if z<=h:
        return 1
    elif  z<=1:
        return (1+np.cos(np.pi*(z-h)/(1-h)))/2
    else:
        return 0
sigma=lambda z:z/np.sqrt(1+z**2)
phi=lambda z:(sigma(z+c)*(a+b)+(a-b))/2
phi_alpha=lambda z:rho(z/alpha)*phi(z-deserve)
def artificial_potential_fun(distance):
    return integrate.quad(phi_alpha,deserve,distance)

#*******************************Agent_Class*********************************
class agent():
    tag=1
    def __init__(self,fg):
        """flag=0 in A group flag=1 in B group"""
        self.x=10*np.random.randn()-5
        self.y=10*np.random.randn()-5
        self.dx=0#10*np.random.randn()-5
        self.dy=0#10*np.random.randn()-5
        self.flag=fg
        self.tag=agent.tag
        self.i=0
        self.record=np.array([[0.0,0.0] for i in range(steps)])
        self.record_vel=np.array([[0.0,0.0] for i in range(steps)])
        self.record[0]=np.array([self.x,self.y])
        self.record_vel[0]=np.array([self.x,self.y])
        self.sensor=mf.sensor(fg)
        self.control=np.array([0.0,0.0])
        self.adj=[]
        agent.tag+=1

    def calculate_control(self,group,target,local):
        """a function that can calculate the control force
        calculate_control(self,groupA,groupB,rx,rdx)
        groupA should be a list record agent in group A
        groupB ~
        rx&&rdx shoule be a list with 2*1 dimension
        rx[0] is coordinate of x
        ry[1] is coordinate of y
        """
        gradient_part = np.array([0.0,0.0])
        consensus_part = np.array([0.0,0.0])
        feedback_part = np.array([0.0,0.0])
        self.cal_adj(group)

        #part I:  calculate force processed by potential function
        for agent in group:
            if not(self is agent):
                dis=np.sqrt((self.x-agent.record[self.i][0])**2+(self.y-agent.record[self.i][1])**2)
                force=phi_alpha(dis)
                gradient_part+=force*np.array([agent.record[self.i][0]-self.x,agent.record[self.i][1]-self.y])/dis

        #part II: calculate force that lead to consensus
        for myNeighbor in self.adj:
            consensus_part+=np.array([myNeighbor.record_vel[self.i][0]-self.dx,myNeighbor.record_vel[self.i][1]-self.dy])



        #part III: claculate force that process by feedback from target
        self.sensor.myfilter([member.sensor for member in self.adj],self.i,target,local)
        feedback_part+=-c_1*np.array([self.x-self.sensor.x,self.y-self.sensor.y])\
                        -c_2*np.array([self.dx-self.sensor.dx,self.dy-self.sensor.dy])

        ret=feedback_part+consensus_part+gradient_part
        self.control=ret
        self.update_state()
        return  ret

    def update_state(self):
        self.i+=1
        self.dx+=self.control[0]*0.01
        self.dy+=self.control[1]*0.01
        self.x+=self.dx*0.01
        self.y+=self.dy*0.01
        self.record[self.i]=np.array([self.x,self.y])
        self.record_vel[self.i]=np.array([self.dx,self.dy])

    def cal_adj(self,group):
        self.adj=[]
        for g in group:
            if(np.sqrt((self.x-g.x)**2+(self.y-g.y)**2)<radius):
                self.adj.append(g)
        if group[0] not in self.adj:
            self.adj.append(group[0])
        if group[mf.numOfGroup1] not in self.adj:
            self.adj.append(group[mf.numOfGroup1])



#*******************************Agent_Class*********************************
if __name__=="__main__":
    end=steps-1
    time_range=np.arange(0,times,0.01)
    track=odeint(mf.targetDynamical,mf.S10+mf.S20,time_range)
    local=np.array([[20.0,0.0,30.0,-20.0] for i in range(steps)])
    for i in range(end):
        local[i+1]=local[i]+track[i]*0.01
    group1=[agent(1) for i in range(mf.numOfGroup1)]
    group2=[agent(2) for i in range(mf.numOfGroup2)]
    wholeGroup=group1+group2

    for i in range(end):
        for follower in wholeGroup:
            follower.calculate_control(wholeGroup,track[i+1],local[i+1])

    mp={0:'first',1:'second'}
    for i in range(2):
        plt.figure("Velocity of group x {0} dimension".format(mp[i]))
        for agent in group1:
            plt.plot(time_range,agent.record_vel[:,i])
        temp,=plt.plot(time_range,track[:,i],label='target x\'s velocity trajectory'.format(mp[i]))
        plt.legend()
        plt.title("The agent's {0} dimension velocity in x group".format(mp[i]))
        plt.show()

    for i in range(2):
        plt.figure("Velocity of group y {0} dimension".format(mp[i]))
        for agent in group2:
            plt.plot(time_range,agent.record_vel[:,i])
        plt.plot(time_range,track[:,i+2],label='target y\'s velocity trajectory'.format(mp[i]))
        plt.legend()
        plt.title("The agent's {0} dimension velocity in y group".format(mp[i]))
        plt.show()

    for s in [5,15,20]:
        end=s*100-1
        plt.figure("flocking in {0} s".format(s))
        follower1=plt.scatter([agent.record[end,0] for agent in group1],[agent.record[end,1] for agent in group1],color='b')
        follower2=plt.scatter([agent.record[end,0] for agent in group2],[agent.record[end,1] for agent in group2],color='g')
        for agent in wholeGroup:
            lens=np.sqrt(agent.record_vel[end,0]**2+agent.record_vel[end,1]**2)
            plt.arrow(agent.record[end,0],agent.record[end,1],agent.record_vel[end,0]/lens,agent.record_vel[end,1]/lens,head_width=0.5,head_length=0.5,fc='k',ec='k')
        plt.plot(local[:,0],local[:,1],label='Trajectory of target x')
        plt.plot(local[:,2],local[:,3],label='Trajectory of target y')
        target1=plt.scatter(local[end,0],local[end,1],color='r',marker='^',s=40)
        plt.scatter(local[end,2],local[end,3],color='r',marker='^',s=40)
        for i in range(2):
            lens=np.sqrt(track[end,i*2]**2+track[end,i*2+1])
            plt.arrow(local[end,i*2],local[end,i*2+1],track[end,i*2]/lens,track[end,i*2+1]/lens,head_width=0.5,head_length=0.5,fc='k',ec='k')
        #plt.legend((follower1,follower2,target1),("agent in x group","agent in y group","target"),loc='upper left')
        plt.legend(loc="upper left")
        plt.show()
