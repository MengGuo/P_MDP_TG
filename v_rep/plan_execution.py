#!/usr/bin/env python
import roslib
import numpy
import Queue
roslib.load_manifest('mdp_tg')
import rospy
from mdp_tg.msg import action, confirmation, cell_pose, status
from math import sqrt, cos, sin, radians
import numpy
import random

import sys


from load_model import best_plan, prod_dra_edges, initial_state

import pickle


def confirm_callback(data):
    global confirm_data    
    header = data.header
    name = data.name
    done = data.done
    confirm_data = [header, name, done]
    if random.random() > 0.8:
        print 'Confirmation received: %s' %str(confirm_data)

    
def cell_pose_callback(data):
    global cell_pose_data
    x = data.x
    y = data.y
    orientation = data.orientation
    cell_pose_data = [x, y, orientation]
    # print 'Robot position received: %s' %str(cell_pose_data)      

    
def plan_execution():
    global confirm_data
    global cell_pose_data
    rospy.init_node('plan_execution')
    print 'Plan execution node started!'
    ###### publish to
    action_pub = rospy.Publisher('next_action', action, queue_size=100)
    status_pub = rospy.Publisher('robot_status', status, queue_size=100)
    ###### subscribe to
    rospy.Subscriber('action_done', confirmation, confirm_callback)
    rospy.Subscriber('robot_cell_pose', cell_pose, cell_pose_callback)
    ##### plan execution
    cell_pose_data = None
    confirm_data = [-1, 'None', 0]
    rospy.sleep(0.5)
    if not cell_pose_data:
        cell_pose_data = set(initial_state).pop()[0]
    elif cell_pose_data != set(initial_state).pop()[0]:
        print 'Make sure the robot starts from the initial state'
    current_state = set(initial_state).pop()
    t = 0
    X = []
    U = []
    while not rospy.is_shutdown():
        try:
            print '-------------'
            print 'current_state:', str(current_state)
            X.append(tuple(current_state))
            #
            next_action_msg = action()
            next_action_name, next_segment = Find_Action(best_plan,current_state)
            next_actstr = r''
            for s in next_action_name:
                next_actstr += s
            next_action_msg.header = t
            next_action_msg.name = next_actstr
            print '=====Next action: %s at stage %s=====' %(next_actstr, str(t))
            U.append(str(next_action_name))
            #
            status_msg = status()
            labelset = set(current_state[1])
            if labelset:
                status_msg.label = labelset.pop()
            else:
                status_msg.label = 'free'
            status_msg.action = next_actstr
            status_msg.segment = next_segment
            while not rospy.is_shutdown():
                if not ((confirm_data[0] == t) and (confirm_data[1]==next_actstr) and (confirm_data[2]>0)):
                    try:
                        action_pub.publish(next_action_msg)
                        status_pub.publish(status_msg)
                        rospy.sleep(0.5)
                    except rospy.ROSInterruptException:
                        pass
                else:
                    break
            #    
            rospy.sleep(1)
            t += 1
            print 'Action %s done!' %next_actstr
            print 'Robot cell pose: %s' %str(cell_pose_data)
            next_state = Find_Next_State(prod_dra_edges, tuple(current_state), next_action_name, cell_pose_data)
            current_state = tuple(next_state)
            print 'Next state %s' %str(current_state)
        except rospy.ROSInterruptException:
            print 'Node closing down'
            pass
    print 'System trajectory:', X
    print 'Control actions:', U
    pickle.dump((X,U), open("results_X_U.p","wb"))        


def Find_Action(best_plan, prod_state):
    # choose action according to the optimal policy
    # best_plan = [plan_prefix, prefix_cost, prefix_risk, y_in_sf], [plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd], plan_bad]
    print '------Choose action according to the optimal policy------'
    print 'current_state', prod_state
    plan_prefix = best_plan[0][0]
    plan_suffix = best_plan[1][0]
    plan_bad = best_plan[3]
    if (prod_state in plan_prefix):
        print 'In prefix'
        U = plan_prefix[prod_state][0]
        P = plan_prefix[prod_state][1]
        next_segment = 0
    elif (prod_state in plan_suffix):
        print 'In suffix'
        U = plan_suffix[prod_state][0]
        P = plan_suffix[prod_state][1]
        next_segment = 1        
    elif (prod_state in plan_bad):
        print 'In bad states'        
        U = plan_bad[prod_state][0]
        P = plan_bad[prod_state][1]
        next_segment = 2
    print 'U:%s, P:%s' %(str(U), str(P))
    rdn = random.random()
    pc = 0
    for k, p in enumerate(P):
        pc += p
        if pc>rdn:
            break
    print 'Action Chosen: %s' %str(U[k])
    next_action_name = U[k]        
    return next_action_name, next_segment


def Find_Next_State(prod_dra_edges, current_state, next_action_name, cell_pose):
    S = []
    P = []
    k = -1
    for (f_state, t_state) in prod_dra_edges.iterkeys():
        if f_state == current_state:
            # print 'edge:', (f_state, t_state)
            prop = prod_dra_edges[(f_state, t_state)]
            # print 'prop of edge:', prop
            # print 'next_action_name', next_action_name
            if ((next_action_name in prop.keys()) and (t_state[0] == tuple(cell_pose))):
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
        print '----------------------------------------'
        print 'check your plan, NO next state can be found!!!'
        print '----------------------------------------'        
        next_state = current_state
    return next_state

    

if __name__ == '__main__':
    ###############
    try:
        plan_execution()
    except rospy.ROSInterruptException:
        pass
