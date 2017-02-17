#!/usr/local/bin/python
#-*- coding: UTF-8 -*-
#2017-02-05  Author: Li Zhenghao

import numpy as np
from scipy.special import gamma
import matplotlib.pyplot as plt

##### 可调的参数 维度，分数阶数，智能体数量, 邻接矩阵, 步数...######
dimension = 3
alpha = 0.7
amount = 5
adj = np.array([[0, 1, 0, 0, 0], [1, 0, 0, 1, 0], [1, 0, 0, 0, 1],
                [0, 0, 1, 0, 0], [0, 0, 0, 1, 0]])
matrixB = np.array([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]])
h = 0.01  #仿真步长
time = 20  #仿真时间
steps = int(time / 0.01)
k = [2, 1, 0.5, 0, 0]

##############################参数区终止###########################

dynamicFunction = lambda x: np.sin(x)


# TODO sgn有问题向量标量?
def sgnfunction(val):
    ret = np.array([0.0] * 3)
    for i, v in enumerate(val):
        if v > 0:
            ret[i] = 1
        elif v < 0:
            ret[i] = -1
        else:
            ret[i] = 0
    return ret


class agent(object):
    """智能体类，通过调用step一步一步向前迭代 """

    def __init__(self, mult, tags, network):
        """初始化需要gamma值，每一个智能体有自己的编号(tags=0:amount-1)
           每个个体的历史数据记录在self.record之中"""
        self.states = np.array(
            [20 * np.random.random() - 10 for i in range(dimension)])
        self.init = self.states
        self.mult = mult
        self.tags = tags
        self.record = np.array([[0.0] * dimension for j in range(steps)])
        self.record[0] += self.states
        self.network = network
        self.leader = network.leader
        self.silding = 0
        self.index = 1

    def next(self):
        # TODO 问一下她matrixB应该是一个向量就可以了
        u = self._communication() + matrixB[self.tags][self.tags] * (
            self.leader[self.index - 1] - self.states)

        val = [0.0] * dimension
        if k[self.tags] != 0:
            self.silding = self.silding + self.mult * (-u + dynamicFunction(
                self.states) - dynamicFunction(self.leader[self.index - 1]))
            val = self.states - self.leader[
                self.index - 1] + self.init - self.leader[0] - self.silding
        u -= k[self.tags] * sgnfunction(val)
        self._update(u)

    def _update(self, val):
        """分数阶积分\frac{1}{\gamma{\alpha}}\interga_{t_0}^{t}(t-\tao)^{\alpha-1}f(\tao)d\tao
        对于数值系统t-t_0=h f(\tao)=f(t_0),由此求出数值解"""
        val += dynamicFunction(self.states)
        self.states += val * self.mult
        self.record[self.index] += self.states
        self.index += 1

    def _communication(self):
        u = np.array([0.0 for i in range(dimension)])
        for neighbor in self.network.nodes:
            u += adj[self.tags][neighbor.tags] * (
                neighbor.record[self.index - 1] - self.states)
        return u


class Network(object):
    """通过for in 一个Network不断向前迭代，所有的节点信息储存在Network中 """

    def __init__(self):
        """创建节点，self.index是当前步数, 计算目标的路径 """
        self.gamma = gamma(alpha)
        self.mult = h**alpha / self.gamma #(alpha * self.gamma)
        self.leadertrajectory()
        self.nodes = [agent(self.mult, i, self) for i in range(amount)]
        self.index = 1

    def leadertrajectory(self):
        self.x0 = np.random.rand(1, dimension) * 5
        record = np.array([[0.0] * dimension for i in range(steps)])
        record[0] = self.x0
        for i in range(steps - 1):
            val = dynamicFunction(record[i])
            record[i + 1] = record[i] + val * self.mult
        self.leader = record

    def __iter__(self):
        return self

    def next(self):
        """不断向前迭代自己的节点，超过steps停止"""
        if self.index >= steps:
            raise StopIteration("Steps have completed!")
        for agent in self.nodes:
            next(agent)
        self.index += 1
        return self.index


def plotNetwork(network):
    mp = {0: "first", 1: "second", 2: "third"}
    for d in range(dimension):
        plt.figure("States_of_the_{}_dimension".format(mp[d]))
        plt.plot(range(steps), network.leader[:, d], color='r')
        for node in network.nodes:
            plt.plot(range(steps), node.record[:, d])
        plt.show()


#    for d in range(dimension):
#        plt.figure("Omega_of_the_{}_dimension".format(mp[d]))
#        for node in network.nodes:
#            plt.plot(range(steps), node.record_omega[:, d])
#        plt.show()

if __name__ == "__main__":
    network = Network()
    for step in network:
        print(step)
    plotNetwork(network)
