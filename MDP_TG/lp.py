# -*- coding: utf-8 -*-

from networkx.classes.digraph import DiGraph
from networkx import single_source_shortest_path
from gurobipy import *

from collections import defaultdict
import random


def syn_full_plan(prod_mdp, gamma, alpha=1):
    #----Optimal plan synthesis, total cost over plan prefix and suffix----
    print "==========[Optimal full plan synthesis start]=========="
    Plan = []
    for l, S_fi in enumerate(prod_mdp.Sf):
        print "---for one S_fi---"
        plan = []
        for k, MEC in enumerate(S_fi):
            plan_prefix, prefix_cost, prefix_risk, y_in_sf, Sr, Sd = syn_plan_prefix(prod_mdp, MEC, gamma)
            print "Best plan prefix obtained, cost: %s, risk %s" %(str(prefix_cost), str(prefix_risk))
            if y_in_sf:
                plan_suffix, suffix_cost, suffix_risk = syn_plan_suffix(prod_mdp, MEC, y_in_sf)
                print "Best plan suffix obtained, cost: %s, risk %s" %(str(suffix_cost), str(suffix_risk))
            else:
                plan_suffix = None
            if plan_prefix  and plan_suffix:
                plan.append([[plan_prefix, prefix_cost, prefix_risk, y_in_sf], [plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd]])
        if plan:
            best_k_plan = min(plan, key=lambda p: p[0][1] + alpha*p[1][1])
            Plan.append(best_k_plan)
        else: 
            "No valid found!"
    if Plan:
        print "========================="
        print " || Final compilation  ||"
        print "========================="
        best_all_plan = min(Plan, key=lambda p: p[0][1] + alpha*p[1][1])
        print 'Best plan prefix obtained for %s states in Sr' %str(len(best_all_plan[0][0]))
        print 'cost: %s; risk: %s '%(best_all_plan[0][1], best_all_plan[0][2])
        print 'Best plan suffix obtained for %s states in Sf' %str(len(best_all_plan[1][0]))
        print 'cost: %s; risk: %s '%(best_all_plan[1][1], best_all_plan[1][2])
        plan_bad = syn_plan_bad(prod_mdp, best_all_plan[2])
        print 'Plan for bad states obtained for %s states in Sd' %str(len(best_all_plan[2][3]))
        best_all_plan.append(plan_bad)
        return best_all_plan
    else:
        print "No valid plan found"
        return None


def syn_full_plan_rex(prod_mdp, gamma, d, alpha=1):
    #----Relaxed optimal plan synthesis, total cost over plan prefix and suffix----    
    print "==========[Relaxed full plan synthesis start]"
    Plan = []
    for l, S_fi in enumerate(prod_mdp.Sf):
        print "---for one S_fi---"
        plan = []
        for k, MEC in enumerate(S_fi):
            plan_prefix, prefix_cost, prefix_risk, y_in_sf, Sr, Sd = syn_plan_prefix(prod_mdp, MEC, gamma)
            print "Best plan prefix obtained, cost: %s, risk %s" %(str(prefix_cost), str(prefix_risk))
            if y_in_sf:
                plan_suffix, suffix_cost, suffix_risk = syn_plan_suffix_rex(prod_mdp, MEC, d, y_in_sf)
                print "Best plan suffix obtained, cost: %s, risk %s" %(str(suffix_cost), str(suffix_risk))
            else:
                plan_suffix = None
            if plan_prefix  and plan_suffix:
                plan.append([[plan_prefix, prefix_cost, prefix_risk, y_in_sf], [plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd]])
        if plan:
            best_k_plan = min(plan, key=lambda p: p[0][1] + alpha*p[1][1])
            Plan.append(best_k_plan)
        else: 
            "No valid found!"
    if Plan:
        print "========================="
        print " || Final compilation  ||"
        print "========================="
        best_all_plan = min(Plan, key=lambda p: p[0][1] + alpha*p[1][1])
        print 'Best plan prefix obtained for %s states in Sr' %str(len(best_all_plan[0][0]))
        print 'cost: %s; risk: %s '%(best_all_plan[0][1], best_all_plan[0][2])
        print 'Best plan suffix obtained for %s states in Sf' %str(len(best_all_plan[1][0]))
        print 'cost: %s; risk: %s '%(best_all_plan[1][1], best_all_plan[1][2])
        plan_bad = syn_plan_bad(prod_mdp, best_all_plan[2])
        print 'Plan for bad states obtained for %s states in Sd' %str(len(best_all_plan[2][3]))
        best_all_plan.append(plan_bad)
        return best_all_plan
    else:
        print "No valid plan found"
        return None

            

def syn_plan_prefix(prod_mdp, MEC, gamma):
    #----Synthesize optimal plan prefix to reach accepting MEC or SCC----
    #----with bounded risk and minimal expected total cost----
    print "===========[plan prefix synthesis starts]==========="    
    sf = MEC[0]
    ip = MEC[1] #force convergence to ip
    delta = 1.0
    for init_node in prod_mdp.graph['initial']:
        path_init = single_source_shortest_path(prod_mdp, init_node)
        print 'Reachable from init size: %s' %len(path_init.keys())
        if not set(path_init.keys()).intersection(sf):
            print "Initial node can not reach sf"
            return None, None, None, None
        Sn = set(path_init.keys()).difference(sf)
        #----find bad states that can not reach MEC
        simple_digraph = DiGraph()
        simple_digraph.add_edges_from(((v,u) for u,v in prod_mdp.edges()))
        path = single_source_shortest_path(simple_digraph, random.sample(ip,1)[0])
        reachable_set = set(path.keys())
        print 'States that can reach sf, size: %s' %str(len(reachable_set))
        Sd = Sn.difference(reachable_set)
        Sr = Sn.intersection(reachable_set)
        # #--------------
        print 'Sn size: %s; Sd inside size: %s; Sr inside size: %s' %(len(Sn),len(Sd), len(Sr))
        # ---------solve lp------------
        print '-----'
        print 'Gurobi starts now'
        print '-----'
        try:
        #if True:
            Y = defaultdict(float)
            model = Model('plan_prefix')
            # create variables
            for s in Sr:
                for u in prod_mdp.node[s]['act'].copy():
                    Y[(s,u)] = model.addVar(vtype=GRB.CONTINUOUS,lb=0, name='y[(%s, %s)]' %(s, u))
            model.update()
            print 'Variables added'
            # set objective
            obj = 0
            for s in Sr:                
                for t in prod_mdp.successors_iter(s):
                    prop = prod_mdp.edge[s][t]['prop'].copy()
                    for u in prop.iterkeys():
                        pe = prop[u][0]
                        ce = prop[u][1]
                        obj += Y[(s,u)]*pe*ce
            model.setObjective(obj, GRB.MINIMIZE)
            print 'Objective function set'
            # add constraints
            #------------------------------
            y_to_sd = 0.0
            y_to_sf = 0.0
            for s in Sr:
                for t in prod_mdp.successors_iter(s):
                    if t in Sd:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys(): 
                            pe = prop[u][0]
                            y_to_sd += Y[(s,u)]*pe
                    elif t in sf:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys(): 
                            pe = prop[u][0]
                            y_to_sf += Y[(s,u)]*pe
            model.addConstr(y_to_sf+y_to_sd >= delta, 'sum_out')
            model.addConstr(y_to_sf >= (1.0-gamma)*(y_to_sf+y_to_sd), 'risk')            
            print 'Risk constraint added'
            #--------------------
            for t in Sr:
                node_y_in = 0.0
                node_y_out = 0.0
                for u in prod_mdp.node[t]['act']:   
                    node_y_out += Y[(t,u)]
                for f in prod_mdp.predecessors_iter(t):
                    if f in Sr:
                        prop = prod_mdp.edge[f][t]['prop'].copy()
                        for uf in prop.iterkeys():
                            node_y_in += Y[(f,uf)]*prop[uf][0]
                if t == init_node:
                    model.addConstr(node_y_out == 1.0 + node_y_in, 'init_node_flow_balance')

                else:
                    model.addConstr(node_y_out == node_y_in, 'middle_node_flow_balance')
            print 'Initial node flow balanced'
            print 'Middle node flow balanced'
            #----------------------
            # solve
            print '--optimization starts--'
            model.optimize()
            # print '--variables value--'
            # for v in model.getVars():
            #     print v.varName, v.x
            # print 'obj:', model.objVal
            #------------------------------
            # compute plan prefix given the LP solution
            plan_prefix = dict()
            for s in Sr:
                norm = 0
                U = []
                P = []
                U_total = prod_mdp.node[s]['act'].copy()          
                for u in U_total:
                    norm += Y[(s,u)].X
                for u in U_total:
                    U.append(u)
                    if norm > 0.01:
                        P.append(Y[(s,u)].X/norm)
                    else:
                        P.append(1.0/len(U_total))
                plan_prefix[s] = [U, P]
            print "----Prefix plan generated"
            cost = model.objval
            print "----Prefix cost computed"
            # compute the risk given the plan prefix
            risk = 0.0
            y_to_sd = 0.0
            y_to_sf = 0.0
            for s in Sr:
                for t in prod_mdp.successors_iter(s):
                    if t in Sd:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys(): 
                            pe = prop[u][0]
                            y_to_sd += Y[(s,u)].X*pe
                    elif t in sf:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys(): 
                            pe = prop[u][0]
                            y_to_sf += Y[(s,u)].X*pe
            if (y_to_sd+y_to_sf) >0:
                risk = y_to_sd/(y_to_sd+y_to_sf)
            print 'y_to_sd: %s; y_to_sd+y_to_sf: %s' %(y_to_sd, y_to_sd+y_to_sf)
            print "----Prefix risk computed: %s" %str(risk)
            # compute the input flow to the suffix
            y_in_sf = dict()
            for s in Sn:
                for t in prod_mdp.successors_iter(s):
                    if t in sf:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys():
                            pe = prop[u][0]
                            if t not in y_in_sf:
                                y_in_sf[t] = Y[(s,u)].X*pe
                            else:
                                y_in_sf[t] += Y[(s,u)].X*pe
            # normalize the input flow
            y_total = 0.0
            for s, y in y_in_sf.iteritems():
                y_total += y
            print 'actual y_total: %s' %str(y_total)
            if y_total >0:
                for s in y_in_sf.iterkeys():
                    y_in_sf[s] = y_in_sf[s]/y_total
                print "----Y in Sf computed and normalized"
            #print y_in_sf
            return plan_prefix, cost, risk, y_in_sf, Sr, Sd
        except GurobiError:
            print "Gurobi Error reported"
            return None, None, None, None                        


def syn_plan_suffix(prod_mdp, MEC, y_in_sf):
    #----Synthesize optimal plan suffix to stay within the accepting MEC----
    #----with minimal expected total cost of accepting cyclic paths----
    print "===========[plan suffix synthesis starts]"    
    sf = MEC[0]
    ip = MEC[1]
    act = MEC[2].copy()
    delta = 1.0
    gamma = 0.0
    for init_node in prod_mdp.graph['initial']:
        paths = single_source_shortest_path(prod_mdp, init_node)
        Sn = set(paths.keys()).intersection(sf)
        print 'Sf size: %s' %len(sf)
        print 'reachable sf size: %s' %len(Sn)
        print 'Ip size: %s' %len(ip)
        print 'Ip and sf intersection size: %s'%len(Sn.intersection(ip))
        # ---------solve lp------------
        print '------'
        print 'Gurobi starts now'
        print '------'
        try:
            Y = defaultdict(float)
            model = Model('plan_suffix')
            # create variables
            for s in Sn:
                for u in act[s]:
                    Y[(s,u)] = model.addVar(vtype=GRB.CONTINUOUS,lb=0, name='y[(%s, %s)]' %(s, u))
            model.update()
            print 'Variables added'
            # set objective
            obj = 0
            for s in Sn:
                for u in act[s]:
                    for t in prod_mdp.successors_iter(s):
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        if u in prop.keys():
                            pe = prop[u][0]
                            ce = prop[u][1]
                            obj += Y[(s,u)]*pe*ce
            model.setObjective(obj, GRB.MINIMIZE)
            print 'Objective added'
            # add constraints
            #--------------------
            for s in Sn:
                constr3 = 0
                constr4 = 0
                for u in act[s]:
                    constr3 += Y[(s,u)]
                for f in prod_mdp.predecessors_iter(s):
                    if (f in Sn) and (s not in ip):
                        prop = prod_mdp.edge[f][s]['prop'].copy()
                        for uf in act[f]:
                            if uf in prop.keys():  
                                constr4 += Y[(f,uf)]*prop[uf][0]
                            else:
                                constr4 += Y[(f,uf)]*0.00
                    if (f in Sn) and (s in ip) and (f != s):
                        prop = prod_mdp.edge[f][s]['prop'].copy()
                        for uf in act[f]:
                            if uf in prop.keys():  
                                constr4 += Y[(f,uf)]*prop[uf][0]
                            else:
                                constr4 += Y[(f,uf)]*0.00
                if (s in y_in_sf.keys()) and (s not in ip):
                    model.addConstr(constr3 == constr4 + y_in_sf[s], 'balance_with_y_in')
                if (s in y_in_sf.keys()) and (s in ip):
                    model.addConstr(constr3 == y_in_sf[s], 'balance_with_y_in')
                if (s not in y_in_sf.keys()) and (s not in ip):
                    model.addConstr(constr3 == constr4, 'balance')
            print 'Balance condition added'
            print 'Initial sf condition added'            
            #--------------------
            y_to_ip = 0.0
            y_out = 0.0
            for s in Sn:
                for t in prod_mdp.successors_iter(s):
                    if t not in Sn:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_out += Y[(s,u)]*pe
                    elif t in ip:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_to_ip += Y[(s,u)]*pe
            model.addConstr(y_to_ip+y_out >= delta, 'sum_out')
            model.addConstr(y_to_ip >= (1.0-gamma)*(y_to_ip+y_out), 'risk')
            print 'Risk constraint added'
            #------------------------------
            # solve
            model.optimize()
            # print '--variables value--'
            # for v in model.getVars():
            #     print v.varName, v.x
            # print 'obj:', model.objVal
            #------------------------------
            # compute optimal plan suffix given the LP solution
            plan_suffix = dict()
            for s in Sn:
                norm = 0
                U = []
                P = []
                for u in act[s]:
                    norm += Y[(s,u)].X
                for u in act[s]:
                    U.append(u)
                    if norm > 0.01:
                        P.append(Y[(s,u)].X/norm)
                    else:
                        P.append(1.0/len(act[s])) 
                plan_suffix[s] = [U, P]
            print "----Suffix plan added"
            cost = model.objval
            print "----Suffix cost computed"
            # compute risk given the plan suffix
            risk = 0.0
            y_to_ip = 0.0
            y_out = 0.0
            for s in Sn:
                for t in prod_mdp.successors_iter(s):
                    if t not in Sn:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_out += Y[(s,u)].X*pe
                    elif t in ip:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_to_ip += Y[(s,u)].X*pe
            if (y_to_ip+y_out)>0:
                risk = y_out/(y_to_ip+y_out)
            print 'y_out: %s; y_to_ip+y_out: %s' %(y_out, y_to_ip+y_out)                
            print "----Suffix risk computed"
            return plan_suffix, cost, risk
        except GurobiError:
            print "Gurobi Error reported"
            return None, None, None




def syn_plan_suffix_rex(prod_mdp, MEC, d, y_in_sf):
    #----Synthesize optimal plan suffix to stay within the accepting SCC----
    #----with minimal expected total cost of accepting cyclic paths----
    #----and penalty over the risk in the suffix 
    print "===========[plan suffix synthesis starts]"    
    sf = MEC[0]
    ip = MEC[1]
    act = MEC[2].copy()
    delta = 1.0
    for init_node in prod_mdp.graph['initial']:
        paths = single_source_shortest_path(prod_mdp, init_node)
        Sn = set(paths.keys()).intersection(sf)
        print 'Sf size: %s' %len(sf)
        print 'reachable sf size: %s' %len(Sn)
        print 'Ip size: %s' %len(ip)
        print 'Ip and sf intersection size: %s'%len(Sn.intersection(ip))
        # ---------solve lp------------
        print '------'
        print 'Gurobi starts now'
        print '------'
        try:
            Y = defaultdict(float)
            model = Model('plan_suffix')
            # create variables
            for s in Sn:
                for u in act[s]:
                    Y[(s,u)] = model.addVar(vtype=GRB.CONTINUOUS,lb=0, name='y[(%s, %s)]' %(s, u))
            model.update()
            print 'Variables added'
            # set objective
            obj = 0
            for s in Sn:
                for u in act[s]:
                    for t in prod_mdp.successors_iter(s):
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        if t in Sn:
                            if u in prop.keys():
                                pe = prop[u][0]
                                ce = prop[u][1]
                                obj += Y[(s,u)]*pe*ce
                        else:
                            if u in prop.keys():
                                obj += Y[(s,u)]*d
            model.setObjective(obj, GRB.MINIMIZE)
            print 'Objective added'
            # add constraints
            #--------------------
            for s in Sn:
                constr3 = 0
                constr4 = 0
                for u in act[s]:
                    constr3 += Y[(s,u)]
                for f in prod_mdp.predecessors_iter(s):
                    if (f in Sn) and (s not in ip):
                        prop = prod_mdp.edge[f][s]['prop'].copy()
                        for uf in act[f]:
                            if uf in prop.keys():  
                                constr4 += Y[(f,uf)]*prop[uf][0]
                            else:
                                constr4 += Y[(f,uf)]*0.00
                    if (f in Sn) and (s in ip) and (f != s):
                        prop = prod_mdp.edge[f][s]['prop'].copy()
                        for uf in act[f]:
                            if uf in prop.keys():  
                                constr4 += Y[(f,uf)]*prop[uf][0]
                            else:
                                constr4 += Y[(f,uf)]*0.00
                if (s in y_in_sf.keys()) and (s not in ip):
                    model.addConstr(constr3 == constr4 + y_in_sf[s], 'balance_with_y_in')
                if (s in y_in_sf.keys()) and (s in ip):
                    model.addConstr(constr3 == y_in_sf[s], 'balance_with_y_in')
                if (s not in y_in_sf.keys()) and (s not in ip):
                    model.addConstr(constr3 == constr4, 'balance')
            print 'Balance condition added'
            print 'Initial sf condition added'            
            #--------------------
            y_to_ip = 0.0
            y_out = 0.0
            for s in Sn:
                for t in prod_mdp.successors_iter(s):
                    if t not in Sn:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_out += Y[(s,u)]*pe
                    elif t in ip:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_to_ip += Y[(s,u)]*pe
            model.addConstr(y_to_ip+y_out >= delta, 'sum_out')
            print 'Risk constraint added'
            #--------------------
            #------------------------------
            # solve
            model.optimize()
            # print '--variables value--'
            # for v in model.getVars():
            #     print v.varName, v.x
            # print 'obj:', model.objVal
            #------------------------------
            # compute optimal plan suffix given the LP solution
            plan_suffix = dict()
            for s in Sn:
                norm = 0
                U = []
                P = []
                for u in act[s]:
                    norm += Y[(s,u)].X
                for u in act[s]:
                    U.append(u)
                    if norm > 0.01:
                        P.append(Y[(s,u)].X/norm)
                    else:
                        P.append(1.0/len(act[s])) 
                plan_suffix[s] = [U, P]
            print "----Suffix plan added"
            cost = model.objval
            print "----Suffix cost computed"
            # compute the risk in the plan suffix
            risk = 0.0
            y_to_ip = 0.0
            y_out = 0.0
            for s in Sn:
                for t in prod_mdp.successors_iter(s):
                    if t not in Sn:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_out += Y[(s,u)].X*pe
                    elif t in ip:
                        prop = prod_mdp.edge[s][t]['prop'].copy()
                        for u in prop.iterkeys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_to_ip += Y[(s,u)].X*pe
            if (y_to_ip+y_out)>0:
                risk = y_out/(y_to_ip+y_out)
            print 'y_out: %s; y_to_ip+y_out: %s' %(y_out, y_to_ip+y_out)                
            print "----Suffix risk computed"
            return plan_suffix, cost, risk
        except GurobiError:
            print "Gurobi Error reported"
            return None, None, None       

        
def syn_plan_bad(prod_mdp, state_types):
    Sf = state_types[0]
    Sr = state_types[2]
    Sd = state_types[3]
    plan_bad = dict()
    for sd in Sd:
        # print 'Sd size',len(Sd)
        # print 'Sf size',len(Sf)
        # print 'Sr size',len(Sr)
        (xf, lf, qf) = sd
        Ud = prod_mdp.node[sd]['act'].copy()
        proj_cost = dict()
        postqf = prod_mdp.graph['dra'].successors(qf)
        # print 'postqf size',len(postqf)
        for xt in prod_mdp.graph['mdp'].successors_iter(xf):
            if xt != xf:
                prop = prod_mdp.graph['mdp'].edge[xf][xt]['prop']
                for u in prop.iterkeys():
                    prob_edge = prop[u][0]
                    cost = prop[u][1]
                    label = prod_mdp.graph['mdp'].node[xt]['label']
                    for lt in label.iterkeys():
                        prob_label = label[lt]
                        dist = dict()
                        for qt in postqf:
                            if (xt,lt,qt) in Sf.union(Sr):
                                dist[qt] = prod_mdp.graph['dra'].check_distance_for_dra_edge(lf,qf,qt)
                        if dist.keys():
                            qt = min(dist.keys(), key = lambda q: dist[q])
                            if u not in proj_cost.keys():
                                proj_cost[u] = 0
                            else:
                                proj_cost[u] += prob_edge*prob_label*dist[qt]
        # policy for bad states
        U = []
        P = []                                
        if proj_cost.keys():
            # print 'sd',sd
            u_star = min(proj_cost.keys(), key = lambda u: proj_cost[u])
            for u in Ud:
                U.append(u)
                if u == u_star:
                    P.append(1)
                else:
                    P.append(0)
        else:
            for u in Ud:
                U.append(u)
                P.append(1.0/len(Ud))
        plan_bad[sd] = [U, P]
    return plan_bad        

        
def act_by_plan(prod_mdp, best_plan, prod_state):
    #----choose the randomized action by the optimal policy---- 
    #recall that {best_plan = [plan_prefix, prefix_cost, prefix_risk, y_in_sf],
    #[plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd], plan_bad]}
    plan_prefix = best_plan[0][0]
    plan_suffix = best_plan[1][0]
    plan_bad = best_plan[3]
    if (prod_state in plan_prefix):
        #print 'In prefix'
        U = plan_prefix[prod_state][0]
        P = plan_prefix[prod_state][1]
        rdn = random.random()
        pc = 0
        for k, p in enumerate(P):
            pc += p
            if pc>rdn:
                break
        #print 'action chosen: %s' %str(U[k])
        return U[k], 0
    elif (prod_state in plan_suffix):
        #print 'In suffix'
        U = plan_suffix[prod_state][0]
        P = plan_suffix[prod_state][1]
        rdn = random.random()
        pc = 0
        for k, p in enumerate(P):
            pc += p
            if pc>rdn:
                break
        #print 'action chosen: %s' %str(U[k])
        if prod_state in best_plan[2][1]:
            return U[k], 10
        else:
            return U[k], 1
    elif (prod_state in plan_bad):
        #print 'In bad states'        
        U = plan_bad[prod_state][0]
        P = plan_bad[prod_state][1]
        rdn = random.random()
        pc = 0
        for k, p in enumerate(P):
            pc += p
            if pc>rdn:
                break
        #print 'action chosen: %s' %str(U[k])
        return U[k], 2
    

def rd_act_by_plan(prod_mdp, best_plan, prod_state, I):
    #----choose the randomized action by the optimal policy----
    #optimal prefix with round-robin as the plan suffix
    #recall that {best_plan = [plan_prefix, prefix_cost, prefix_risk, y_in_sf],
    #[plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd], plan_bad]}
    plan_prefix = best_plan[0][0]
    plan_suffix = best_plan[1][0]
    plan_bad = best_plan[3]
    if (prod_state in plan_prefix):
        #print 'In prefix'
        U = plan_prefix[prod_state][0]
        P = plan_prefix[prod_state][1]
        rdn = random.random()
        pc = 0
        for k, p in enumerate(P):
            pc += p
            if pc>rdn:
                break
        #print 'action chosen: %s' %str(U[k])
        return U[k], 0, I
    elif (prod_state in plan_suffix):
        #print 'In suffix'
        U = plan_suffix[prod_state][0]
        #print 'action chosen: %s' %str(U[k])
        if I[prod_state] <= (len(U)-1):
            u = U[I[prod_state]]
            I[prod_state] += 1
        if I[prod_state] == len(U):
            I[prod_state] = 0
            u = U[I[prod_state]]
        if prod_state in best_plan[2][1]:
            return u, 10, I
        else:
            return u, 1, I
    elif (prod_state in plan_bad):
        #print 'In bad states'        
        U = plan_bad[prod_state][0]
        P = plan_bad[prod_state][1]
        rdn = random.random()
        pc = 0
        for k, p in enumerate(P):
            pc += p
            if pc>rdn:
                break
        #print 'action chosen: %s' %str(U[k])
        return U[k], 2, I

    
        
