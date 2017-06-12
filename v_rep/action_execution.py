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
        ret_l, LMotorHandle = vrep.simxGetObjectHandle(clientID, "LeftMotor", opmode)
        ret_r, RMotorHandle = vrep.simxGetObjectHandle(clientID, "RightMotor", opmode)
        ret_a, RobotHandle = vrep.simxGetObjectHandle(clientID, "Khepera", opmode)
        # If handlers are OK, execute simulation
        if (ret_l == 0) and (ret_r == 0) and (ret_a == 0):
            # Start the simulation
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxStartSimulation
            vrep.simxStartSimulation(clientID, opmode)
            print "----- Simulation started -----"
            # Getting the robot position
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectPosition
            pret, RobotPos = vrep.simxGetObjectPosition(clientID, RobotHandle, -1, vrep.simx_opmode_streaming)
            print "robot position: (x = " + str(robotPos[0]) + ", y = " + str(robotPos[1]) + ")"
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectOrientation
            oret, RobotOrient = vrep.simxGetObjectOrientation(clientID, RobotHandle, -1, vrep.simx_opmode_streaming)
            print "robot orientation: (x = " + str(robotOrient[0]) + ", y = " + str(robotOrient[1]) +", z = " + str(robotOrient[2]) + ")"
            # motor control
            vrep.simxSetJointTargetVelocity(clientID, LMotorHandle, l_ang_v, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, RMotorHandle, r_ang_v, vrep.simx_opmode_streaming)            
            # Get the robot position after the movement sequence
            pret, RobotPos = vrep.simxGetObjectPosition(clientID, RobotHandle, -1, vrep.simx_opmode_buffer)
            # End the simulation, wait to be sure V-REP had the time to stop it entirely
            vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
            time.sleep(1)            
        # Close the connection to V-REP remote server
        # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxFinish
        vrep.simxFinish(clientID)
    else:
        print 'Failed connecting to remote API server'
    print ('End connection')



def distance(pose1, pose2):
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)

    
def action_execution():
    ######
    edges = motion_mdp_edges.keys()
    states = [e[0] for e in edges] + [e[1] for e in edges]
    states_x = [s[0] for s in states]
    states_y = [s[1] for s in states]
    x_max = max(states_x)
    x_min = min(states_x)
    y_max = max(states_y)
    y_min = min(states_y)
    print 'xy_max', x_max, x_min, y_max, y_min
    ###### publish to
    confirmation_pub = rospy.Publisher('action_done', confirmation, queue_size=100)
    cell_pose_pub = rospy.Publisher('robot_cell_pose', cell_pose, queue_size=100)
    control_pub = rospy.Publisher('%s/cmd_vel' %robot_name, geometry_msgs.msg.Twist, queue_size=100)
    ###### subscribe to
    rospy.Subscriber('next_action', action, next_action_callback)
    rospy.Subscriber('robot_raw_pose', raw_pose, raw_pose_callback)
    ##### action execution
    t = 0
    grid = WS_d*2    # grid size
    raw_pose_data = None
    rospy.sleep(1.0)
    if raw_pose_data:
        cell_pose_data = Raw_To_Cell_Pose(raw_pose_data, grid)
    else:
        cell_pose_data = set(initial_state).pop()[0]
    cell_pose_msg = cell_pose()
    cell_pose_msg.x = cell_pose_data[0]
    cell_pose_msg.y = cell_pose_data[1]
    cell_pose_msg.orientation = cell_pose_data[2]
    count_t = 0
    # print 'cell_pose_data', cell_pose_data
    while count_t < 0.5: # publish for 1s
        cell_pose_pub.publish(cell_pose_msg)
        rospy.sleep(0.05)
        count_t += 0.05
    print 'Robot cell pose published: (%s, %s, %s)' %(str(cell_pose_data[0]), str(cell_pose_data[1]), str(cell_pose_data[2]))
    next_action_data = [-1, 'None']
    #### 
    while not rospy.is_shutdown():
        reached = False
        if next_action_data[0] == t:
            print 'Next action %s received at time %s' %(next_action_data[1], next_action_data[0])
            action_name = next_action_data[1]
            start_pose = raw_pose_data[:]
            goal_pose = Find_Goal(cell_pose_data, grid, action_name)
            print 'Goal pose: %s' %str(goal_pose)
            while not rospy.is_shutdown():
                if not reached:
                    # send control msg
                    # print 'Navigation to goal'
                    linearVelo, angularVelo, reached = Find_Control(raw_pose_data, action_name, goal_pose)
                    control_msg = geometry_msgs.msg.Twist()
                    control_msg.linear.x = linearVelo
                    control_msg.angular.z = angularVelo
                    control_pub.publish(control_msg)
                    if random.random()>0.8:
                        print 'Goal pose: %s' %str(goal_pose)
                        print 'Control cmds published: %s' %str((linearVelo, angularVelo))
                    rospy.sleep(0.5) # TODO: check how fast the input can change
                    cell_pose_data = Raw_To_Cell_Pose(raw_pose_data, grid)
                    cell_pose_msg = cell_pose()
                    cell_pose_msg.x = cell_pose_data[0]
                    cell_pose_msg.y = cell_pose_data[1]
                    cell_pose_msg.orientation = cell_pose_data[2]
                    cell_pose_pub.publish(cell_pose_msg)
                else:
                    break
            print '===========Goal pose reached: %s==========' %(str(goal_pose))
            # send confirmation msg 
            confirmation_msg = confirmation()
            confirmation_msg.header = t
            confirmation_msg.name = action_name
            confirmation_msg.done = 1
            count_t = 0 
            while count_t < 1: # publish for 1s
                confirmation_pub.publish(confirmation_msg)
                rospy.sleep(0.05)
                count_t += 0.05
            print 'Confirmation for action %s at time %s sent' %(str(action_name), str(t))
            t += 1
        else:
            rospy.sleep(0.05)



def Find_Goal(cell_pose, grid, action_name):
    global x_min, x_max, y_min, y_max
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



def  Find_Control(raw_pose, action_name, goal_pose):
    # turn-and-forward-turn controller
    LINEAR_V = 0.2     #m/s
    ANGULAR_V = 0.35   #rad/s
    angular_V = 0.0
    linear_V = 0.0
    d_bound = 0.1
    theta_bound = 0.15*PI
    reached = False
    if random.random()>0.8:
        print 'Action to perform: %s' %action_name
        print 'raw_pose', raw_pose
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
    if (((action_name =='FR') or (action_name =='BK')) and (distance((s_x,s_y), (g_x,g_y)) > d_bound) and (abs(theta_dif) > theta_bound)):
        if random.random()>0.8:
            print 'Turn to face the goal, distance error: %s, face error: %s' %(str(distance((s_x,s_y), (g_x,g_y))), str(-theta_dif))
        if ((0 < theta_dif < 1.0*PI) or (-2.0*PI < theta_dif < -1.0*PI)):
            angular_V = -ANGULAR_V 
        else:
            angular_V = ANGULAR_V 
    elif ((distance((s_x,s_y), (g_x,g_y)) > d_bound) and (abs(theta_dif) < theta_bound)):
        if random.random()>0.8:        
            print 'Forward to the goal, distance error: %s, face error: %s' %(str(distance((s_x,s_y), (g_x,g_y))), str(theta_dif))
        if action_name != 'BK':
            linear_V = LINEAR_V
        if action_name == 'BK':
            linear_V = -LINEAR_V            
    #elif ((distance((s_x,s_y), (g_x,g_y)) < d_bound) and (abs(orientation_dif) > theta_bound)):
    elif ((abs(orientation_dif) > theta_bound)):            
        if random.random()>0.8:        
            print 'Turn to right orientation, distance error: %s, orientation error: %s' %(str(distance((s_x,s_y), (g_x,g_y))), str(-orientation_dif))
        if ((0 < orientation_dif < 1.0*PI) or (-2.0*PI < orientation_dif < -1.0*PI)):
            angular_V = -ANGULAR_V 
        else:
            angular_V = ANGULAR_V
    else:
        print 'Goal reached!'
        print 'distance', distance((s_x,s_y), (g_x,g_y))
        print 'orientation error', orientation_dif
        reached = True
    return linear_V, angular_V, reached



def Raw_To_Cell_Pose(raw_pose, grid):
    global x_min, x_max, y_min, y_max
    [raw_x, raw_y, raw_theta] = raw_pose
    cell_x = floor(raw_x/grid)*grid + grid*0.5
    cell_y = floor(raw_y/grid)*grid + grid*0.5
    if cell_x<= x_min:
        cell_x = x_min
    if cell_x>= x_max:
        cell_x = x_max
    if cell_y<= y_min:
        cell_y = y_min
    if cell_y>= y_max:
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



if __name__ == '__main__':
    ###############
    try:
        action_execution(robot_name='Brain5')
    except rospy.ROSInterruptException:
        pass
