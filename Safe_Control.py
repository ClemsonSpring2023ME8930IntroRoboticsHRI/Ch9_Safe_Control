# -*- coding: utf-8 -*-
"""
Author: Miao Yu
"""

"""
Before Running the command it is expected from you to read through the
command syntax from python API for CoppeliaSim. 
"""
# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.


import math
import sim
import time
import numpy as np
from qpsolvers import solve_qp

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
else:
        print ('Failed connecting to remote API server')
        print ('Program ended')

res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
else:
        print ('Remote API function call returned with error code: ',res)

time.sleep(2)

#Getting object handlle
opMode=sim.simx_opmode_blocking
errorCode,base=sim.simxGetObjectHandle(clientID,"Base",opMode)
errorCode,link1=sim.simxGetObjectHandle(clientID,"Link1",opMode)
errorCode,link2=sim.simxGetObjectHandle(clientID,"Link2",opMode)
errorCode,link3=sim.simxGetObjectHandle(clientID,"Link3",opMode)
errorCode,dummy=sim.simxGetObjectHandle(clientID,"Dummy",opMode)
errorCode,target=sim.simxGetObjectHandle(clientID,"Target",opMode)
errorCode,obstacle=sim.simxGetObjectHandle(clientID,"Obstacle",opMode)

errorCode,J1=sim.simxGetObjectHandle(clientID,"Revolute_joint1",opMode)
errorCode,J2=sim.simxGetObjectHandle(clientID,"Revolute_joint2",opMode)
errorCode,J3=sim.simxGetObjectHandle(clientID,"Revolute_joint3",opMode)

#Getting tip and target positions
errorCode, tip_position= sim.simxGetObjectPosition(clientID,dummy,base,sim.simx_opmode_oneshot_wait)
errorCode, target_position= sim.simxGetObjectPosition(clientID,target,base,sim.simx_opmode_oneshot_wait)
errorCode, obstacle_position= sim.simxGetObjectPosition(clientID,obstacle,base,sim.simx_opmode_oneshot_wait)


"""
Note that the coppeliasim coordinate system isn't configured according to
the DH convention. Hence, observe that the formula is changed. ye in 
coppeliasim is xe according to the DH convention and similarly, ze is ye
according to DH convention. The actual calculation doesn't change just the 
naming of variable changes to resemble the coordinate system in coppeliasim
"""
a1 = 0.8; #Link 1 length
a2 = 0.6; #Link 2 length
a3 = 0.4; #Link 3 length
ye = target_position[1]; #Target Position
ze = target_position[2]; #Target Position
ze = ze- 0.425;
ro = 0.47; # radius of the obstacle
yo = obstacle_position[1]; #Obstacle Position
zo = obstacle_position[2]; #Obstacle Position
zo = zo;
g = 70; #end-effector orientation with respect to horizontal axis
g = math.radians(g)

#Finding joint 3 position
y3 = ye - a3*math.cos(g)
z3 = ze - a3*math.sin(g)

#Calculate the inverse kinematics for configuration that end-effector reaches the target
theta1a = (math.atan(z3/y3)-math.acos((y3**2+z3**2+a1**2-a2**2)/(2*a1*math.sqrt(y3**2+z3**2))))
theta2a = (math.pi - math.acos((a1**2+a2**2-y3**2-z3**2)/(2*a1*a2)))
theta3a = (g) - theta1a - theta2a 


"""
Simulation
"""

"""
Part I: conventional control method, cannot guarantee collision avoidance
"""
for i in range(1,20+1):
  q1 = sim.simxGetJointPosition(clientID, J1, opMode)[1] # joint 1 angle
  q2 = sim.simxGetJointPosition(clientID, J2, opMode)[1] # joint 2 angle
  q3 = sim.simxGetJointPosition(clientID, J3, opMode)[1] # joint 3 angle
  # print("angle 1=",q1)
  # print("angle 2=",q2)
  # print("angle 3=",q3)
  kp1 = 0.1
  kp2 = 0.1
  kp3 = 0.18

  # Desired Lyapunov-based controller
  #joint 1
  k1 = -kp1*(q1-theta1a);
  #joint 2  
  k2 = -kp2*(q2-theta2a);
  #joint 3  
  k3 = -kp3*(q3-theta3a);
  
  u1 = k1;
  u2 = k2;
  u3 = k3;
  
  # Pass the control command to the manipulator
  errorcode = sim.simxSetJointTargetVelocity(clientID, J1, u1, sim.simx_opmode_oneshot_wait)
  errorcode = sim.simxSetJointTargetVelocity(clientID,J2,u2,sim.simx_opmode_oneshot_wait)
  errorcode = sim.simxSetJointTargetVelocity(clientID,J3,u3,sim.simx_opmode_oneshot_wait)    

"""
Reset the initial robot configuration
"""
errorcode = sim.simxSetJointTargetVelocity(clientID, J1, 0, sim.simx_opmode_oneshot_wait)
errorcode = sim.simxSetJointTargetVelocity(clientID,J2,0,sim.simx_opmode_oneshot_wait)
errorcode = sim.simxSetJointTargetVelocity(clientID,J3,0,sim.simx_opmode_oneshot_wait)

errorcode = sim.simxSetJointPosition(clientID, J1, 0, sim.simx_opmode_oneshot_wait)
errorcode = sim.simxSetJointPosition(clientID,J2,0,sim.simx_opmode_oneshot_wait)
errorcode = sim.simxSetJointPosition(clientID,J3,0,sim.simx_opmode_oneshot_wait)    


"""
Part II: CBF-based method, guarantees collision avoidance
"""
for i in range(1,30+1):
  q1 = sim.simxGetJointPosition(clientID, J1, opMode)[1] # joint 1 angle
  q2 = sim.simxGetJointPosition(clientID, J2, opMode)[1] # joint 2 angle
  q3 = sim.simxGetJointPosition(clientID, J3, opMode)[1] # joint 3 angle
  print("angle 1=",q1)
  print("angle 2=",q2)
  print("angle 3=",q3)
  kp1 = 0.1
  kp2 = 0.1
  kp3 = 0.18

  # Desired Lyapunov-based controller
  #joint 1
  k1 = -kp1*(q1-theta1a);
  #joint 2  
  k2 = -kp2*(q2-theta2a);
  #joint 3  
  k3 = -kp3*(q3-theta3a);
  
  # Solving QP problem
  P = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]])
  q = np.array([-k1, -k2, -k3])
  D = (a1*math.cos(q1)+a2*math.cos(q1+q2)+a3*math.cos(q1+q2+q3)-yo)**2+(a1*math.sin(q1)+a2*math.sin(q1+q2)+a3*math.sin(q1+q2+q3)-zo)**2-ro**2;
  G = np.array([-(2*(a1*math.cos(q1)+a2*math.cos(q1+q2)+a3*math.cos(q1+q2+q3)-yo)*(-a1*math.sin(q1)-a2*math.sin(q1+q2)-a3*math.sin(q1+q2+q3))+2*(a1*math.sin(q1)+a2*math.sin(q1+q2)+a3*math.sin(q1+q2+q3)-yo)*(a1*math.cos(q1)+a2*math.cos(q1+q2)+a3*math.cos(q1+q2+q3)))/D**2,\
                -(2*(a1*math.cos(q1)+a2*math.cos(q1+q2)+a3*math.cos(q1+q2+q3)-yo)*(-a2*math.sin(q1+q2)-a3*math.sin(q1+q2+q3))+2*(a1*math.sin(q1)+a2*math.sin(q1+q2)+a3*math.sin(q1+q2+q3)-yo)*(a2*math.cos(q1+q2)+a3*math.cos(q1+q2+q3)))/D**2,\
                    -(2*(a1*math.cos(q1)+a2*math.cos(q1+q2)+a3*math.cos(q1+q2+q3)-yo)*(-a3*math.sin(q1+q2+q3))+2*(a1*math.sin(q1)+a2*math.sin(q1+q2)+a3*math.sin(q1+q2+q3)-yo)*(a3*math.cos(q1+q2+q3)))/D**2]);
  h = np.array([(ye-yo)**2+(ze-zo)**2-ro**2]);
  A = np.array([0.0, 0.0, 0.0]);
  b = np.array([0.0]);
  
  u = solve_qp(P, q, G.T, h, A, b, solver="daqp");
  u1 = u[0];
  u2 = u[1];
  u3 = u[2];
  
  # Pass the control command to the manipulator
  errorcode = sim.simxSetJointTargetVelocity(clientID, J1, u1, sim.simx_opmode_oneshot_wait)
  errorcode = sim.simxSetJointTargetVelocity(clientID,J2,u2,sim.simx_opmode_oneshot_wait)
  errorcode = sim.simxSetJointTargetVelocity(clientID,J3,u3,sim.simx_opmode_oneshot_wait)    

errorcode = sim.simxSetJointTargetVelocity(clientID, J1, 0, sim.simx_opmode_oneshot_wait)
errorcode = sim.simxSetJointTargetVelocity(clientID,J2,0,sim.simx_opmode_oneshot_wait)
errorcode = sim.simxSetJointTargetVelocity(clientID,J3,0,sim.simx_opmode_oneshot_wait)    

sim.simxFinish(clientID)