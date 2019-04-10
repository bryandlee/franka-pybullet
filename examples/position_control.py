import sys
sys.path.append('../src')

import time
from math import sin

from panda import Panda

duration = 30
stepsize = 1e-3

robot = Panda(stepsize)
robot.setControlMode("position")
target_pos = robot.target_pos

for i in range(int(duration/stepsize)):
    t = robot.t
    robot.setTargetPositions(target_pos)
    robot.step()

    pos, vel = robot.getJointStates()

    time.sleep(robot.stepsize)