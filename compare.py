#!/usr/local/bin/python
import bad_flocking as bad
from flocking import *

if __name__ == "__main__":
    end = steps - 1
    time_range = np.arange(0, times, 0.01)
    track = odeint(mf.targetDynamical, mf.S10 + mf.S20, time_range)
    location = np.array([[20.0, 0.0, 30.0, -20.0] for i in range(steps)])
    for i in range(end):
        location[i + 1] = location[i] + track[i] * 0.01
    group1 = [agent(1) for i in range(mf.numOfGroup1)]
    group2 = [agent(2) for i in range(mf.numOfGroup2)]
    wholeGroup = group1 + group2

    for i in range(end):
        for follower in wholeGroup:
            follower.calculate_control(wholeGroup, track[i + 1], location[i +
                                                                          1])

    badgroup1 = [bad.agent(1) for i in range(mf.numOfGroup1)]
    badgroup2 = [bad.agent(2) for i in range(mf.numOfGroup2)]
    badwholeGroup = badgroup1 + badgroup2

    for i in range(end):
        for follower in badwholeGroup:
            follower.calculate_control(badwholeGroup, track[i + 1],
                                       location[i + 1])

    error = [0.0 for i in range(steps)]
    for i in range(steps):
        for follower in wholeGroup:
            error[i] += np.linalg.norm(follower.record_vel[i] - track[i, (
                follower.flag - 1) * 2:(follower.flag - 1) * 2 + 2])**2
        error[i] /= 4

    baderror = [0.0 for i in range(steps)]
    for i in range(steps):
        for follower in badwholeGroup:
            baderror[i] += np.linalg.norm(follower.record_vel[i] - track[i, (
                follower.flag - 1) * 2:(follower.flag - 1) * 2 + 2])**2
        baderror[i] /= 4

    plt.title("Error of sensors' velocity")
    plt.plot(
        time_range[500:], error[500:], label="The proposed flocking algorithm")
    plt.plot(
        time_range[500:], baderror[500:], label="Ordinary flocking algorithm")
    plt.legend(loc="upper right")
    plt.show()

    goodConditionA = np.array([[0.0, 0.0] for i in range(steps)])
    for i in range(steps):
        for follower in group1:
            goodConditionA[i] += follower.record[i]
        goodConditionA[i] /= mf.numOfGroup1

    goodConditionB = np.array([[0.0, 0.0] for i in range(steps)])
    for i in range(steps):
        for follower in group2:
            goodConditionB[i] += follower.record[i]
        goodConditionB[i] /= mf.numOfGroup2

    plt.figure()
    plt.title("Flocking centre")
    plt.plot(location[:, 0], location[:, 1], color = 'deepskyblue', label = 'Trajectory of target')
    plt.plot(location[:, 2], location[:, 3], color = 'deepskyblue')
    plt.plot(goodConditionA[200:, 0], goodConditionA[200:, 1], color = 'coral', label = 'Trajectory of flock\'s centre')
    plt.plot(goodConditionB[200:, 0], goodConditionB[200:, 1], color = 'coral')
    plt.legend(loc = 'upper left')
    plt.show()

    badConditionA = np.array([[0.0, 0.0] for i in range(steps)])
    for i in range(steps):
        for follower in badgroup1:
            badConditionA[i] += follower.record[i]
        badConditionA[i] /= mf.numOfGroup1

    badConditionB = np.array([[0.0, 0.0] for i in range(steps)])
    for i in range(steps):
        for follower in badgroup2:
            badConditionB[i] += follower.record[i]
        badConditionB[i] /= mf.numOfGroup2

    plt.figure()
    plt.title("Flocking centre")
    plt.plot(location[:, 0], location[:, 1], color = 'deepskyblue', label = 'Trajectory of target')
    plt.plot(location[:, 2], location[:, 3], color = 'deepskyblue')
    plt.plot(badConditionA[200:, 0], badConditionA[200:, 1], color = 'coral', label = 'Trajectory of flock\'s centre')
    plt.plot(badConditionB[200:, 0], badConditionB[200:, 1], color = 'coral')
    plt.legend(loc = 'upper left')
    plt.show()
