from Bridge import connection 
from Bridge import configuration as cf
from Bridge import kbhit
import numpy as np

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
qdes = []

from Controller import robot
robot = robot.RobotState()
robot.updateKinematics(np.matrix(np.zeros(7)).T, np.matrix(np.zeros(7)).T)
robot.placement('panda_joint3') # urdf index

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

    if is_simulation_run:
        if vrep.doNextStep: # for synchronize      
            vrep.doNextStep=False  
            q, qdot = vrep.getMotorState() #Read Device
            
            if is_first:
                qdes = q
                is_first = False 

            qdes = np.array(qdes) 
            vrep.setMotorState(qdes)   
            
            client.simxSynchronousTrigger()

    client.simxSpinOnce()
    

client.simxStopSimulation(client.simxDefaultPublisher())
print "Simulation finished"

#TODO: How to terminate server???