#!/usr/local/bin/python
#-*- coding: UTF-8 -*-

#Author: Li Zhenghao

import numpy as np
from scipy.special import gamma
import matplotlib.pyplot as plt

## 可调的参数 维度，分数阶数，智能体数量, 邻接矩阵,步数...######
dimension = 2
alpha = 0.7
amount = 5
adj = np.array([[0, 1, 0, 0, 0], [1, 0, 0, 1, 0], [1, 0, 0, 0, 1],
                [0, 0, 1, 0, 0], [0, 0, 0, 1, 0]])
h = 0.01  #仿真步长
time = 5  #仿真时间
steps = int(time / h)
k = [1, 3, 2]
init_state = np.array([
    np.array([0.0, 0.0]), np.array([1.0, 2.0]), np.array([3.0, 4.0]),
    np.array([5.0, 6.0]), np.array([7.0, 8.0])
])
firstGroup = [0, 1]  #一阶个体

##############################参数区终止###########################


class agent(object):
    """智能体类，通过调用step一步一步向前迭代 """

    def __init__(self, gammaValue, tags, network):
        """初始化需要gamma值，每一个智能体有自己的编号(tags=0:amount-1)
           每个个体的历史数据记录在self.record之中"""
        #        self.states=np.array([20*np.random.random()-10 for i in range(dimension)])
        self.states = init_state[tags]
        self.gamma = gammaValue
        self.tags = tags
        self.record = np.array(
            [np.array([0.0 for i in range(dimension)]) for j in range(steps)])
        self.record[0] += self.states
        self.omega = np.array([0.0 for i in range(dimension)])
        self.record_omega = np.array(
            [np.array([0.0 for i in range(dimension)]) for j in range(steps)])
        self.record_omega[0] += self.omega
        self.network = network
        self.gain = 0
        if self.tags in firstGroup:
            self.gain = k[0]
        else:
            self.gain = k[1]
        self.index = 1

    def next(self):
        u = self._communication()
        u -= k[2] * self.omega
        self._update(u)

    def _update(self, u):
        """分数阶积分\frac{1}{\gamma{\alpha}}\interga_{t_0}^{t}(t-\tao)^{\alpha-1}f(\tao)d\tao
        对于数值系统t-t_0=h f(\tao)=f(t_0),由此求出数值解"""
        if self.tags in firstGroup:
            self.states += u * (h**alpha) / (alpha * self.gamma)
        else:
            self.omega += u * (h**alpha) / (alpha * self.gamma)
            self.states += self.omega * (h**alpha) / (alpha * self.gamma)
        self.record[self.index] += self.states
        self.record_omega[self.index] += self.omega
        self.index += 1

    def _communication(self):
        u = np.array([0.0 for i in range(dimension)])
        for neighbor in self.network.nodes:
            u += adj[self.tags][neighbor.tags] * (
                neighbor.record[self.index - 1] - self.states)
        return self.gain * u


class Network(object):
    """通过for in 一个Network不断向前迭代，所有的节点信息储存在Network中 """

    def __init__(self):
        """创建节点，self.index是当前步数 """
        self.gamma = gamma(alpha)
        self.nodes = [agent(self.gamma, i, self) for i in range(amount)]
        self.index = 1

    def __iter__(self):
        return self

    def next(self):
        """不断向前迭代自己的节点，超过steps停止"""
        if self.index >= steps:
            raise StopIteration("Steps have completed!")
        for agent in self.nodes:
            agent.next()
        self.index += 1
        return self.index


def plotNetwork(network):
    mp = {0: "first", 1: "second", 2: "third"}
    for d in range(dimension):
        plt.figure("States_of_the_{}_dimension".format(mp[d]))
        plt.title("States of the {} dimension".format(mp[d]))
        for node in network.nodes:
            plt.plot(range(steps), node.record[:, d])

#        plt.plot(range(steps),)
        plt.xlabel("step")
        plt.ylabel("states")
        plt.show()

    for d in range(dimension):
        plt.figure("Omega_of_the_{}_dimension".format(mp[d]))
        for node in network.nodes:
            plt.plot(range(steps), node.record_omega[:, d])
        plt.show()

if __name__ == "__main__":
    network = Network()
    for step in network:
        print step
    plotNetwork(network)
