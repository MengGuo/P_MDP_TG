#!/usr/bin/env python

import vrep

from math import sqrt, cos, sin, radians, atan2, radians
from math import pi as PI
from numpy import floor, random

import sys
import time

# import workspace and robot model
from load_model import WS_d, grid, initial_state, motion_mdp_edges
from load_model import best_plan, prod_dra_edges, x_max, x_min, y_max, y_min


def reset_vrep():
    print('Start to connect vrep')
    # Close eventual old connections
    vrep.simxFinish(-1)
    # Connect to V-REP remote server
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    opmode = vrep.simx_opmode_oneshot_wait
    # Try to retrieve motors and robot handlers
    # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectHandle
    ret_l, LMotorHandle = vrep.simxGetObjectHandle(
        clientID, "Pioneer_p3dx_leftMotor", opmode)
    ret_r, RMotorHandle = vrep.simxGetObjectHandle(
        clientID, "Pioneer_p3dx_rightMotor", opmode)
    ret_a, RobotHandle = vrep.simxGetObjectHandle(
        clientID, "Pioneer_p3dx", opmode)
    vrep.simxSetJointTargetVelocity(
        clientID, LMotorHandle, 0, vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetVelocity(
        clientID, RMotorHandle, 0, vrep.simx_opmode_blocking)
    time.sleep(1)
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    vrep.simxFinish(clientID)
    print('Connection to vrep reset-ed!')


def connect_vrep(T=100):
    # ----------------
    print('Start to connect vrep')
    # Close eventual old connections
    vrep.simxFinish(-1)
    # Connect to V-REP remote server
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1:
        print('Connected to remote API server')
        # Communication operating mode with the remote API : wait for its answer before continuing (blocking mode)
        # http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm
        opmode = vrep.simx_opmode_oneshot_wait
        # Try to retrieve motors and robot handlers
        # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectHandle
        ret_l, LMotorHandle = vrep.simxGetObjectHandle(
            clientID, "Pioneer_p3dx_leftMotor", opmode)
        ret_r, RMotorHandle = vrep.simxGetObjectHandle(
            clientID, "Pioneer_p3dx_rightMotor", opmode)
        ret_a, RobotHandle = vrep.simxGetObjectHandle(
            clientID, "Pioneer_p3dx", opmode)
        # If handlers are OK, execute simulation
        if (ret_l == 0) and (ret_r == 0) and (ret_a == 0):
            # Start the simulation
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxStartSimulation
            vrep.simxStartSimulation(clientID, opmode)
            print("----- Simulation started -----")
            # Getting the robot position
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectPosition
            pret, RobotPos = vrep.simxGetObjectPosition(
                clientID, RobotHandle, -1, vrep.simx_opmode_blocking)
            print("robot initial position: (x = " +
                  str(RobotPos[0]) + ", y = " + str(RobotPos[1]) + ")")
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectOrientation
            oret, RobotOrient = vrep.simxGetObjectOrientation(
                clientID, RobotHandle, -1, vrep.simx_opmode_blocking)
            print("robot initial orientation: (a = " + str(RobotOrient[0]) + ", b = " + str(
                RobotOrient[1]) + ", g = " + str(RobotOrient[2]) + ")")
            raw_pose_data = shift_raw_pose(
                [RobotPos[0], RobotPos[1], RobotOrient[2]])
            cell_pose_data = Raw_To_Cell_Pose(raw_pose_data, grid)
            # --------------------
            if cell_pose_data != set(initial_state).pop()[0]:
                print('Make sure the robot starts from the initial state')
            current_state = set(initial_state).pop()
            # --------------------
            t = 0
            X = []
            U = []
            # motor control
            while (t <= T):
                print('-------------')
                # print 'current_state:', str(current_state)
                X.append(tuple(current_state))
                next_action_name, next_segment = Find_Action(
                    best_plan, current_state)
                next_actstr = r''
                for s in next_action_name:
                    next_actstr += s
                print('=====Next action: %s at stage %s=====' %
                      (next_actstr, str(t)))
                raw_pose_data = execute_action(
                    clientID, RobotHandle, LMotorHandle, RMotorHandle, raw_pose_data, next_actstr)
                t += 1
                print('Action %s done!' % next_actstr)
                cell_pose_data = Raw_To_Cell_Pose(raw_pose_data, grid)
                # print 'Robot cell pose: %s' %str(cell_pose_data)
                next_state = Find_Next_State(prod_dra_edges, tuple(
                    current_state), next_action_name, cell_pose_data)
                current_state = tuple(next_state)
                print('Next state %s' % str(current_state))
            print('System trajectory:', X)
            print('Control actions:', U)
            pickle.dump((X, U), open("v_rep_results_X_U.p", "wb"))
        # End the simulation, wait to be sure V-REP had the time to stop it entirely
        vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
        time.sleep(1)
        # Close the connection to V-REP remote server
        # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxFinish
        vrep.simxFinish(clientID)
    else:
        print('Failed connecting to remote API server')
    print('End connection')


def execute_action(clientID, RobotHandle, LMotorHandle, RMotorHandle, raw_pose_data, next_action):
    linearVelo, angularVelo, goal_pose = compute_control(
        raw_pose_data, next_action)
    l_ang_v, r_ang_v = dif_drive(linearVelo, angularVelo)
    # ----------
    dist_bound = 0.15
    ang_bound = 0.1*PI
    # retDia, DialogHandle, uiHandle = vrep.simxDisplayDialog(clientID, 'Current Action', next_action, vrep.sim_dlgstyle_message, 'None', None, None, vrep.simx_opmode_blocking)
    # ----------
    while (distance(raw_pose_data, goal_pose) >= dist_bound) or (abs(raw_pose_data[2]-goal_pose[2]) >= ang_bound):
        # print '--line_distance--', distance(raw_pose_data, goal_pose)
        # print '--angle distance--', abs(raw_pose_data[2]-goal_pose[2])
        pret, RobotPos = vrep.simxGetObjectPosition(
            clientID, RobotHandle, -1, vrep.simx_opmode_blocking)
        # print "robot position: (x = " + str(RobotPos[0]) + ", y = " + str(RobotPos[1]) + ")"
        oret, RobotOrient = vrep.simxGetObjectOrientation(
            clientID, RobotHandle, -1, vrep.simx_opmode_blocking)
        # print "robot orientation: (a = " + str(RobotOrient[0]) + ", b = " + str(RobotOrient[1]) +", g = " + str(RobotOrient[2]) + ")"
        raw_pose_data = shift_raw_pose(
            [RobotPos[0], RobotPos[1], RobotOrient[2]])
        # ------------------------------
        linearVelo, angularVelo = Find_Control(
            raw_pose_data, next_action, goal_pose)
        l_ang_v, r_ang_v = dif_drive(linearVelo, angularVelo)
        # print 'raw_pose_data', raw_pose_data
        vrep.simxSetJointTargetVelocity(
            clientID, LMotorHandle, l_ang_v, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(
            clientID, RMotorHandle, r_ang_v, vrep.simx_opmode_streaming)
        # print 'distance_x_y:%s; angle_dif:%s' %(str(distance(raw_pose_data, goal_pose)),str(abs(raw_pose_data[2]-goal_pose[2])))
    print('Goal pose reached!')
    # vrep.simxSetJointTargetVelocity(clientID, LMotorHandle, 0, vrep.simx_opmode_blocking)
    # vrep.simxSetJointTargetVelocity(clientID, RMotorHandle, 0, vrep.simx_opmode_blocking)
    # time.sleep(0.5)
    # retEnd = vrep.simxEndDialog(clientID, DialogHandle, vrep.simx_opmode_blocking)
    return raw_pose_data


def dif_drive(linearVelo, angularVelo):
    R = 0.008
    D = 0.053
    if abs(linearVelo) > 0:
        l_ang_v = linearVelo/(2*PI*R)
        r_ang_v = l_ang_v
    else:
        r_ang_v = angularVelo*D*0.5/R
        l_ang_v = -r_ang_v
    return l_ang_v, r_ang_v


def distance(pose1, pose2):
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)


def compute_control(raw_pose_data, next_action):
    # action execution
    t = 0
    cell_pose_data = Raw_To_Cell_Pose(raw_pose_data, grid)
    # print 'Current cell pose %s' %str(cell_pose_data)
    ####
    # print 'Next action %s' %str(next_action)
    goal_pose = Find_Goal(cell_pose_data, grid, next_action)
    # print 'Goal pose: %s' %str(goal_pose)
    linearVelo, angularVelo = Find_Control(
        raw_pose_data, next_action, goal_pose)
    return linearVelo, angularVelo, goal_pose


def Find_Goal(cell_pose, grid, action_name):
    # decide goal state and control input
    [x, y, orientation] = cell_pose
    P = []
    G = []
    if action_name == 'FR':
        # create uncertainty in the FR action
        P = [0.1, 0.8, 0.1]
        # if -0.25*PI <= theta <= 0.25*PI:
        if orientation == 'E':
            G = [(grid, -grid, 0), (grid, 0, 0), (grid, grid, 0)]
        # elif 0.25*PI <= theta <= 0.75*PI:
        elif orientation == 'N':
            G = [(-grid, grid, 0), (0, grid, 0), (grid, grid, 0)]
        # elif (0.75*PI <= theta <= 1.0*PI) or (-1.0*PI <= theta <= -0.75*PI):
        elif orientation == 'W':
            G = [(-grid, -grid, 0), (-grid, 0, 0), (-grid, grid, 0)]
        # elif -0.75*PI <= theta <= -0.25*PI:
        elif orientation == 'S':
            G = [(-grid, -grid, 0), (0, -grid, 0), (grid, -grid, 0)]
    elif action_name == 'BK':
        # create uncertainty in the BK action
        P = [0.25, 0.25, 0.5]
        #P = [0.15, 0.7, 0.15]
        # if -0.25*PI <= theta <= 0.25*PI:
        if orientation == 'E':
            G = [(-grid, -grid, 0), (-grid, 0, 0), (-grid, grid, 0)]
        # elif 0.25*PI <= theta <= 0.75*PI:
        elif orientation == 'N':
            G = [(-grid, -grid, 0), (0, -grid, 0), (grid, -grid, 0)]
        # elif (0.75*PI <= theta <= 1.0*PI) or (-1.0*PI <= theta <= -0.75*PI):
        elif orientation == 'W':
            G = [(grid, -grid, 0), (grid, 0, 0), (grid, grid, 0)]
        # elif -0.75*PI <= theta <= -0.25*PI:
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
    # print '----------------------------------------'
    # print 'start_pose: %s, relative_G_pose: %s' %(str(start_pose), str(G_pose))
    # print '----------------------------------------'
    goal_pose = [start_pose[i]+G_pose[i] for i in range(0, 3)]
    if goal_pose[2] > 1.0*PI:
        goal_pose[2] -= 2.0*PI
    if goal_pose[2] < -1.0*PI:
        goal_pose[2] += 2.0*PI
    # confined by workspace size
    if goal_pose[0] <= x_min:
        goal_pose[0] = x_min
    if goal_pose[0] >= x_max:
        goal_pose[0] = x_max
    if goal_pose[1] <= y_min:
        goal_pose[1] = y_min
    if goal_pose[1] >= y_max:
        goal_pose[1] = y_max
    return goal_pose


def Find_Control(raw_pose, action_name, goal_pose):
    # turn-and-forward-turn controller
    LINEAR_V = 0.05  # m/s
    ANGULAR_V = 0.08  # rad/s
    angular_V = 0.0
    linear_V = 0.0
    d_bound = 0.1
    theta_bound = 0.12*PI
    # if random.random()>0.9:
    # print 'Action to perform: %s' %action_name
    # print 'raw_pose', raw_pose
    # print 'Goal_pose', goal_pose
    [s_x, s_y, s_theta] = raw_pose
    [g_x, g_y, g_theta] = goal_pose
    if action_name != 'BK':
        face = atan2(g_y-s_y, g_x-s_x)
    else:
        face = atan2(g_y-s_y, g_x-s_x) + PI
        if face > PI:
            face -= 2*PI
        elif face < -PI:
            face += 2*PI
    theta_dif = s_theta-face
    orientation_dif = s_theta-g_theta
    # print 'line_distance', distance((s_x,s_y), (g_x,g_y))
    # print 'orientation dif', orientation_dif
    # print 'theta dif', theta_dif
    if (((action_name == 'FR') or (action_name == 'BK'))):
        if (distance((s_x, s_y), (g_x, g_y)) > d_bound) and (abs(theta_dif) > theta_bound):
            if ((0 < theta_dif < 1.0*PI) or (-2.0*PI < theta_dif < -1.0*PI)):
                angular_V = -ANGULAR_V
            else:
                angular_V = ANGULAR_V
            # print 'turning to forward'
        elif (distance((s_x, s_y), (g_x, g_y)) > d_bound) and (abs(theta_dif) <= theta_bound):
            if action_name != 'BK':
                linear_V = LINEAR_V
            if action_name == 'BK':
                linear_V = -LINEAR_V
            # print 'forward'
        else:
            if ((0 < orientation_dif < 1.0*PI) or (-2.0*PI < orientation_dif < -1.0*PI)):
                angular_V = -ANGULAR_V
            else:
                angular_V = ANGULAR_V
            # print 'turnning to face'
    else:
        if ((0 < orientation_dif < 1.0*PI) or (-2.0*PI < orientation_dif < -1.0*PI)):
            angular_V = -ANGULAR_V
        else:
            angular_V = ANGULAR_V
        # print 'turnning to turn'
    return linear_V, angular_V


def Raw_To_Cell_Pose(raw_pose, grid):
    [raw_x, raw_y, raw_theta] = raw_pose
    cell_x = floor(raw_x/grid)*grid + grid*0.5
    cell_y = floor(raw_y/grid)*grid + grid*0.5
    if cell_x <= x_min:
        cell_x = x_min
    if cell_x >= x_max:
        cell_x = x_max
    if cell_y <= y_min:
        cell_y = y_min
    if cell_y >= y_max:
        cell_y = y_max
    if -0.25*PI <= raw_theta <= 0.25*PI:
        orientation = 'E'
    elif 0.25*PI <= raw_theta <= 0.75*PI:
        orientation = 'N'
    elif (0.75*PI <= raw_theta <= 1.0*PI) or (-1.0*PI <= raw_theta <= -0.75*PI):
        orientation = 'W'
    elif -0.75*PI <= raw_theta <= -0.25*PI:
        orientation = 'S'
    return cell_x, cell_y, orientation


def Find_Action(best_plan, prod_state):
    # choose action according to the optimal policy
    # best_plan = [plan_prefix, prefix_cost, prefix_risk, y_in_sf], [plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd], plan_bad]
    print('------Choose action according to the optimal policy------')
    print('current_state', prod_state)
    plan_prefix = best_plan[0][0]
    plan_suffix = best_plan[1][0]
    plan_bad = best_plan[3]
    if (prod_state in plan_prefix):
        print('In prefix')
        U = plan_prefix[prod_state][0]
        P = plan_prefix[prod_state][1]
        next_segment = 0
    elif (prod_state in plan_suffix):
        print('In suffix')
        U = plan_suffix[prod_state][0]
        P = plan_suffix[prod_state][1]
        next_segment = 1
    elif (prod_state in plan_bad):
        print('In bad states')
        U = plan_bad[prod_state][0]
        P = plan_bad[prod_state][1]
        next_segment = 2
    print('U:%s, P:%s' % (str(U), str(P)))
    rdn = random.random()
    pc = 0
    for k, p in enumerate(P):
        pc += p
        if pc > rdn:
            break
    print('Action Chosen: %s' % str(U[k]))
    next_action_name = U[k]
    return next_action_name, next_segment


def Find_Next_State(prod_dra_edges, current_state, next_action_name, cell_pose):
    S = []
    P = []
    k = -1
    for (f_state, t_state) in prod_dra_edges.keys():
        if f_state == current_state:
            # print 'edge:', (f_state, t_state)
            prop = prod_dra_edges[(f_state, t_state)]
            # print 'prop of edge:', prop
            # print 'next_action_name', next_action_name
            if ((next_action_name in list(prop.keys())) and (t_state[0] == tuple(cell_pose))):
                S.append(t_state)
                P.append(prop[next_action_name][0])
                # print 'S:', S
                # print 'P:', P
    rdn = random.random()
    pc = 0
    for k, p in enumerate(P):
        pc += p
        if pc > rdn:
            break
    if k >= 0:
        next_state = tuple(S[k])
    else:
        print('----------------------------------------')
        print('check your plan, NO next state can be found!!!')
        print('----------------------------------------')
        next_state = current_state
    return next_state


def shift_raw_pose(raw_pose):
    # ----------------
    X_SHIFT = 1.5
    Y_SHIFT = 1.5
    new_pose = [raw_pose[0]+X_SHIFT, raw_pose[1]+Y_SHIFT, raw_pose[2]]
    return new_pose


if __name__ == '__main__':
    ###############
    if sys.argv[1] == "connect":
        connect_vrep()
    elif sys.argv[1] == "reset":
        reset_vrep()
