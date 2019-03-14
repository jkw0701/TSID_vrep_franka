import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
import os

class RobotState:
    def __init__(self):
        filename = str(os.path.dirname(os.path.abspath(__file__)))
        pkg = filename + '/../Model'
        urdf = pkg + '/franka_description/urdf/franka_no_gripper.urdf'
        self.robot = RobotWrapper.BuildFromURDF(urdf,[pkg,])
        self.srdf = pkg + '/srdf/ur5.srdf'
        self.model = self.robot.model
        self.data = self.robot.data

    def setWorldSE3(self, world):
        self.world = world

    def updateKinematics(self, q, qdot):
        self.q = q
        self.qdot = qdot
        self.robot.forwardKinematics(q)

    def placement(self, joint_name):
        index = self.model.getJointId(joint_name)
        return self.data.oMi[index]
       
