import pybullet as p

class Panda:
    def __init__(self):
        self.t = 0.0
        self.stepsize = 1e-3
        self.RealTime = 0

        self.position_control_gain_p = 0.01
        self.position_control_gain_d = 1.0
        self.max_torque = 100

        p.connect(p.GUI)
        self.reset()

    def reset(self):
        self.t = 0.0
        
        # simulator
        p.resetSimulation()
        p.setRealTimeSimulation(self.RealTime)
        p.setTimeStep(self.stepsize)
        p.setGravity(0, 0, -9.81)

        # ground plane
        p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, 0)

        # panda robot
        self.robot = p.loadURDF("panda/panda.urdf", useFixedBase=True)
        self.resetControl()

    def step(self):
        self.t += self.stepsize
        p.stepSimulation()

    def resetControl(self):
        for i in range(11):
            p.setJointMotorControl2(self.robot, i, controlMode=p.VELOCITY_CONTROL, force=0)

    def setTargetPosition(self, joint, pos):
        p.setJointMotorControl2(bodyUniqueId   = self.robot, 
                                jointIndex     = joint, 
                                controlMode    = p.POSITION_CONTROL,
                                targetPosition = pos,
                                targetVelocity = 0,
                                force          = self.max_torque,
                                positionGain   = self.position_control_gain_p,
                                velocityGain   = self.position_control_gain_d)
            
    def setTargetTorque(self, joint, torque):
        p.setJointMotorControl2(bodyUniqueId = self.robot, 
                                jointIndex   = joint,
                                controlMode  = p.TORQUE_CONTROL,
                                force        = torque)
