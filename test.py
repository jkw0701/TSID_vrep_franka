from Bridge import connection 
from Bridge import configuration as cf
from Bridge import kbhit
import numpy as np
import pinocchio as se3


vrep = connection.clientsever()
client = vrep.client

def simulationStepStarted(msg):
    simTime=msg[1][b'simulationTime'];
    
def simulationStepDone(msg):
    simTime=msg[1][b'simulationTime'];
    vrep.doNextStep=True

vrep.getMotorHandle()

client.simxSynchronous(True)
client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))
client.simxStartSimulation(client.simxDefaultPublisher())

is_simulation_run = True
exit_flag = False
is_first = True
kbd = kbhit.KBHit()
qdes = np.array(np.zeros(7))
err = np.matrix(np.zeros(6)).T
DT = 0.001


from Controller import robot
robot = robot.RobotState()
robot.updateKinematics(np.matrix(np.zeros(7)).T, np.matrix(np.zeros(7)).T)
robot.placement('panda_joint7') # urdf index

i=0
while not exit_flag:    
    if kbd.kbhit():
        key = kbd.getch()
        if key == 'q':
            is_simulation_run = False
            exit_flag = True
        elif key == '\t': # TAB
            if is_simulation_run:
                print "SIMULATION PAUSE"
                is_simulation_run = False
            else:
                print "SIMULATION RUN"
                is_simulation_run = True
        elif key == 'i':
            print "Initial Posture"
            qdes = np.array(np.zeros(cf.dof)) 
        elif key == 'h':
            print "Home Posture"
            qdes = np.array([0.1, -1.57, 0.3, 0.4, 0.5, 0.6, 0.7, 0.7]) 
        elif key == 't':
            print "Task Space Control"
            qdes = np.array([0.1, -1.57, 0.3, 0.4, 0.5, 0.6, 0.7, 0.7]) 


    if is_simulation_run:
        if vrep.doNextStep: # for synchronize      
            vrep.doNextStep=False  
            q, qdot = vrep.getMotorState() #Read Device

            x = robot.data.oMi[7].translation
            Rot = robot.data.oMi[7].rotation
            xdes = np.matrix([0.0, 0.5542, 0.7315]).T
            Rot_d = np.identity(3)

            delphi = robot.getPhi(Rot_d, Rot)

            err[0:3] = 400*(xdes - x)
            err[3:6] = -100.0*delphi.T
            J = robot.jointJacobian(np.asmatrix(q).T, 7)
            print(J)

            v = np.linalg.pinv(J)*err
            qdes = se3.integrate(robot.model,np.asmatrix(q).T,v*DT)

            if not i % 10:        
                print('error = %s' % err.T)
                
            i += 1

            robot.updateKinematics(np.asmatrix(q).T, np.asmatrix(qdot).T)
            vrep.setMotorState(np.asarray(qdes).T)   
            
            client.simxSynchronousTrigger()

    client.simxSpinOnce()
    

client.simxStopSimulation(client.simxDefaultPublisher())
print "Simulation finished"

#TODO: How to terminate server???