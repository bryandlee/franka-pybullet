import pybullet as p

class Panda:
    def __init__(self, stepsize=1e-3):
        self.t = 0.0
        self.stepsize = stepsize
        self.realtime = 0

        self.control_mode = "torque" 

        self.dof = 7
        self.joints = [0,1,2,3,4,5,6]

        self.position_control_gain_p = [0.01,0.01,0.01,0.01,0.01,0.01,0.01]
        self.position_control_gain_d = [1.0,1.0,1.0,1.0,1.0,1.0,1.0]
        self.max_torque = [100,100,100,100,100,100,100]

        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5, cameraYaw=30, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.5])

        self.reset()

    def reset(self):
        self.t = 0.0

        # simulator
        p.resetSimulation()
        p.setRealTimeSimulation(self.realtime)
        p.setTimeStep(self.stepsize)
        p.setGravity(0,0,-9.81)

        # ground plane
        p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0,0)

        # panda robot
        self.robot = p.loadURDF("panda/panda.urdf",
                                useFixedBase=True,
                                flags=p.URDF_USE_SELF_COLLISION)
        
        self.joint_min = []
        self.joint_max = []
        self.target_pos = []
        self.target_torque = [0,0,0,0,0,0,0]

        for j in range(self.dof):
            joint_info = p.getJointInfo(self.robot, j)
            self.joint_min.append(joint_info[8])
            self.joint_max.append(joint_info[9])
            self.target_pos.append((self.joint_min[j] + self.joint_max[j])/2.0)
            
            p.resetJointState(self.robot,j,targetValue=self.target_pos[j])

        self.resetController()

    def step(self):
        self.t += self.stepsize
        p.stepSimulation()

    def resetController(self):
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0,0,0,0,0,0,0])

    def setControlMode(self, mode):
        if mode == "position":
            self.control_mode = "position"
        elif mode == "torque":
            if self.control_mode != "torque":
                self.resetController()
            self.control_mode = "torque"
        else:
            raise Exception('wrong control mode')

    def setTargetPositions(self, target_pos):
        self.target_pos = target_pos
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=self.target_pos,
                                    forces=self.max_torque,
                                    positionGains=self.position_control_gain_p,
                                    velocityGains=self.position_control_gain_d)

    def setTargetTorques(self, target_torque):
        self.target_torque = target_torque
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=self.target_torque)

    def getJointStates(self):
        joint_states = p.getJointStates(self.robot, self.joints)
        joint_pos = [x[0] for x in joint_states]
        joint_vel = [x[1] for x in joint_states]
        return joint_pos, joint_vel 

