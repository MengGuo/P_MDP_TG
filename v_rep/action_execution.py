#!/usr/bin/env python

import vrep

from math import sqrt, cos, sin, radians, atan2
from math import pi as PI
from numpy import floor, random

import sys

# import workspace and robot model
from load_model import WS_d, initial_state, motion_mdp_edges


def connect_vrep():
    print 'Start to connect vrep'
    # Close eventual old connections
    vrep.simxFinish(-1)
    # Connect to V-REP remote server
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1:
        print 'Connected to remote API server'
        # Communication operating mode with the remote API : wait for its answer before continuing (blocking mode)
        # http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm
        opmode = vrep.simx_opmode_oneshot_wait
        # Try to retrieve motors and robot handlers
        # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectHandle
        ret_l, LMotorHandle = vrep.simxGetObjectHandle(clientID, "K3_leftWheelMotor", opmode)
        ret_r, RMotorHandle = vrep.simxGetObjectHandle(clientID, "K3_rightWheelMotor", opmode)
        ret_a, RobotHandle = vrep.simxGetObjectHandle(clientID, "K3_robot", opmode)
        # If handlers are OK, execute simulation
        if (ret_l == 0) and (ret_r == 0) and (ret_a == 0):
            # Start the simulation
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxStartSimulation
            vrep.simxStartSimulation(clientID, opmode)
            print "----- Simulation started -----"
            # Getting the robot position
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectPosition
            pret, RobotPos = vrep.simxGetObjectPosition(clientID, RobotHandle, -1, vrep.simx_opmode_streaming)
            print "robot position: (x = " + str(RobotPos[0]) + ", y = " + str(RobotPos[1]) + ")"
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectOrientation
            oret, RobotOrient = vrep.simxGetObjectOrientation(clientID, RobotHandle, -1, vrep.simx_opmode_streaming)
            print "robot orientation: (x = " + str(RobotOrient[0]) + ", y = " + str(RobotOrient[1]) +", z = " + str(RobotOrient[2]) + ")"
            raw_pose_data = [RobotPos[0], RobotPos[1], RobotOrient[0]]
            # motor control
            next_action = 'TR'
            linearVelo, angularVelo, goal_pose = action_execution(raw_pose_data, next_action)
            l_ang_v, r_ang_v = dif_drive(linearVelo, angularVelo)
            dist_bound = 0.1
            ang_bound = 0.3
            while (distance(raw_pose_data, goal_pose)>= dist_bound) or (abs(raw_pose_data[2]-goal_pose[2])>=ang_bound):
                pret, RobotPos = vrep.simxGetObjectPosition(clientID, RobotHandle, -1, vrep.simx_opmode_buffer)
                oret, RobotOrient = vrep.simxGetObjectOrientation(clientID, RobotHandle, -1, vrep.simx_opmode_buffer)
                raw_pose_data = [RobotPos[0], RobotPos[1], RobotOrient[0]]                
                vrep.simxSetJointTargetVelocity(clientID, LMotorHandle, l_ang_v, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, RMotorHandle, r_ang_v, vrep.simx_opmode_streaming)
            # End the simulation, wait to be sure V-REP had the time to stop it entirely
            vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
            time.sleep(1)            
        # Close the connection to V-REP remote server
        # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxFinish
        vrep.simxFinish(clientID)
    else:
        print 'Failed connecting to remote API server'
    print ('End connection')


def dif_drive(linearVelo, angularVelo):
    R = 0.008
    D = 0.053
    if abs(linearVelo) >0:
        l_ang_v = linearVelo/(2*PI*R)
        r_ang_v = l_ang_v
    else:
        r_ang_v = angularVelo*D*0.5/R
        l_ang_v = -r_ang_v
    return l_ang_v, r_ang_v

    
def distance(pose1, pose2):
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)

    
def action_execution(raw_pose_data, next_action):
    ##### action execution
    t = 0
    grid = WS_d*2    # grid size
    cell_pose_data = Raw_To_Cell_Pose(raw_pose_data, grid)
    print 'Current cell pose %s' %str(cell_pose_data)
    ####
    print 'Next action %s' %str(next_action)
    goal_pose = Find_Goal(cell_pose_data, grid, next_action)
    print 'Goal pose: %s' %str(goal_pose)
    linearVelo, angularVelo = Find_Control(next_action)
    return linearVelo, angularVelo, goal_pose
    


def Find_Goal(cell_pose, grid, action_name):
    # decide goal state and control input
    [x, y, orientation] = cell_pose
    P =[]
    G = []
    if action_name == 'FR':
        # create uncertainty in the FR action
        P = [0.1, 0.8, 0.1]
        #if -0.25*PI <= theta <= 0.25*PI:
        if orientation == 'E':
            G = [(grid, -grid, 0), (grid, 0, 0), (grid, grid, 0)]
        #elif 0.25*PI <= theta <= 0.75*PI:
        elif orientation == 'N':
            G = [(-grid, grid, 0), (0, grid, 0), (grid, grid, 0)]
        #elif (0.75*PI <= theta <= 1.0*PI) or (-1.0*PI <= theta <= -0.75*PI):
        elif orientation == 'W':
            G = [(-grid, -grid, 0), (-grid, 0, 0), (-grid, grid, 0)]
        #elif -0.75*PI <= theta <= -0.25*PI:
        elif orientation == 'S':
            G = [(-grid, -grid, 0), (0, -grid, 0), (grid, -grid, 0)]
    elif action_name == 'BK':
        # create uncertainty in the BK action
        P = [0.25, 0.25, 0.5]
        #P = [0.15, 0.7, 0.15]
        #if -0.25*PI <= theta <= 0.25*PI:
        if orientation == 'E':
            G = [(-grid, -grid, 0), (-grid, 0, 0), (-grid, grid, 0)]
        #elif 0.25*PI <= theta <= 0.75*PI:
        elif orientation == 'N':
            G = [(-grid, -grid, 0), (0, -grid, 0), (grid, -grid, 0)]
        #elif (0.75*PI <= theta <= 1.0*PI) or (-1.0*PI <= theta <= -0.75*PI):
        elif orientation == 'W':
            G = [(grid, -grid, 0), (grid, 0, 0), (grid, grid, 0)]
        #elif -0.75*PI <= theta <= -0.25*PI:
        elif orientation == 'S':
            G = [(grid, grid, 0), (0, grid, 0), (-grid, grid, 0)]
    elif action_name == 'TR':
        # create uncertainty in the TR action
        P = [0.05, 0.9, 0.05]
        G = [(0, 0, 0), (0, 0, -0.5*PI), (0, 0, -1.0*PI)]
    elif action_name == 'TL':
        # create uncertainty in the TL action        
        P = [0.05, 0.9, 0.05]
        G = [(0, 0, 0), (0, 0, 0.5*PI), (0, 0, 1.0*PI)]
    elif action_name == 'ST':
        # create uncertainty in the ST action        
        P = [1.0]
        G = [(0, 0, 0)]
    # find possible goal
    rdn = random.random()
    pc = 0
    for k, p in enumerate(P):
        pc += p
        if pc > rdn:
            break
    G_pose = G[k]
    if orientation == 'E':
        start_pose = [x, y, 0]
    elif orientation == 'N':
        start_pose = [x, y, 0.5*PI]
    elif orientation == 'S':
        start_pose = [x, y, -0.5*PI]
    elif orientation == 'W':
        start_pose = [x, y, 0.98*PI]
    print '----------------------------------------'
    print 'start_pose: %s, G_pose: %s' %(str(start_pose), str(G_pose))
    print '----------------------------------------'
    goal_pose = [start_pose[i]+G_pose[i] for i in xrange(0,3)]
    if goal_pose[2] > 1.0*PI:
        goal_pose[2] -= 2.0*PI
    if goal_pose[2] < -1.0*PI:
        goal_pose[2] += 2.0*PI
    return goal_pose



def  Find_Control(action_name):
    # turn-and-forward-turn controller
    LINEAR_V = 0.2     #m/s
    ANGULAR_V = 0.35   #rad/s
    print 'Action to perform: %s' %action_name
    if action_name == 'FR':
        linear_V = LINEAR_V
        angular_V = 0.0
    elif action_name == 'BK':
        linear_V = -LINEAR_V
        angular_V = 0.0
    elif action_name == 'TR':
        linear_V = 0.0
        angular_V = -ANGULAR_V
    elif action_name == 'TL':
        linear_V = 0.0
        angular_V = ANGULAR_V
    return linear_V, angular_V



def Raw_To_Cell_Pose(raw_pose, grid):
    [raw_x, raw_y, raw_theta] = raw_pose
    cell_x = floor(raw_x/grid)*grid + grid*0.5
    cell_y = floor(raw_y/grid)*grid + grid*0.5
    if -0.25*PI <= raw_theta <= 0.25*PI:
        orientation = 'E'
    elif 0.25*PI <= raw_theta <= 0.75*PI:
        orientation = 'N'
    elif (0.75*PI <= raw_theta <= 1.0*PI) or (-1.0*PI <= raw_theta <= -0.75*PI):
        orientation = 'W'    
    elif -0.75*PI <= raw_theta <= -0.25*PI:
        orientation = 'S'        
    return cell_x, cell_y, orientation



if __name__ == '__main__':
    ###############
    connect_vrep()
