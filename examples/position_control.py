import sys
sys.path.append('../src')

import pybullet as p
import time
from math import sin

from panda_ball import Panda


duration = 3000
stepsize = 1e-3

robot = Panda(stepsize)
robot.setControlMode("position")

for i in range(int(duration/stepsize)):
    if i%1000 == 0:
        print("Simulation time: {:.3f}".format(robot.t))
 
    # if i%3000 == 0:
    #     robot.reset()
    #     robot.resetBall()
    #     robot.setControlMode("position")
    #     pos, vel = robot.getJointStates()
    #     target_pos = pos

    ball_pos, ball_ori = robot.getBallStates()
    target_task_pos = ball_pos
    target_task_pos[2] += .5

    target_pos = robot.solveInverseKinematics(ball_pos,[1,0,0,0])
    robot.setTargetPositions(target_pos)

    robot.step()


    time.sleep(robot.stepsize)