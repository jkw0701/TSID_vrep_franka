import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
import os
import numpy as np


class RobotState:
    def __init__(self):
        filename = str(os.path.dirname(os.path.abspath(__file__)))
        pkg = filename + '/../Model'
        urdf = pkg + '/franka_description/urdf/franka_no_gripper.urdf'
        self.robot = RobotWrapper.BuildFromURDF(urdf,[pkg,])
        self.srdf = pkg + '/srdf/ur5.srdf'
        self.model = self.robot.model
        self.data = self.robot.data
        self.Rot_ee = np.matrix(np.zeros((6, 6)))


    def setWorldSE3(self, world):
        self.world = world

    def updateKinematics(self, q, qdot):
        self.q = q
        self.qdot = qdot
        self.robot.forwardKinematics(q, qdot)

    def placement(self, joint_name):
        index = self.model.getJointId(joint_name)
        return self.data.oMi[index] # Transformation matrix

       
    def jointJacobian(self, q, frame_id):
        J = self.robot.jointJacobian(q, frame_id)
        # local to global Jacobian
        self.Rot_ee[0:3,0:3] = self.data.oMi[7].rotation.T
        self.Rot_ee[3:6,3:6] = self.data.oMi[7].rotation.T
        J = self.Rot_ee*J
        return J 

    # frame_id does not matter
    def frameJacobian(self, q, frame_id): # local frame
        for index in range(15, 21):
            print(index)
            J = self.robot.frameJacobian(q, index)
            print(J)
        return J 

    def jointPosition(self, joint_id):
        return self.data.oMi[joint_id]
 
    # data.oMi[index] : only joints' position and orientation (base ~ joint7)
    # frame index : univese - root joint - link 0 - joint 1 - link 1 ...
    def framePosition(self, q, index):
        f = self.model.frames[index]
        return self.robot.framePlacement(q, index)
 
    def computeAllTerms(self, data, q, v):
        se3.computeAllTerms(self.m_model, data, q, v)
         
    def mass(self, data):
        return data.M

    def nonLinearEffects(self, data):
        return data.nle    

    def getPhi(self, Rotd, Rot):
        s1 = Rot[0:3,0]
        s1d = Rotd[0:3,0]
        s2 = Rot[0:3,1]
        s2d = Rotd[0:3,1]
        s3 = Rot[0:3,2]
        s3d = Rotd[0:3,2]
        delphi = -0.5*( np.cross(s1.T,s1d.T) + np.cross(s2.T,s2d.T) + np.cross(s3.T, s3d.T) )
        #print(delphi)

        return np.asmatrix(delphi)    