# -*- Coding: utf-8 -*-

from networkx.classes.digraph import DiGraph
from networkx import single_source_shortest_path
from ortools.linear_solver import pywraplp

from collections import defaultdict
import random


def syn_full_plan_comb(prod_mdp, gamma, alpha=1):
    # ----Optimal plan synthesis, total cost over plan prefix and suffix----
    print("==========[Optimal full plan synthesis start]==========")
    Plan = []
    for l, S_fi in enumerate(prod_mdp.Sf):
        print("---for one S_fi---")
        plan_fi = syn_plan_comb(prod_mdp, S_fi, gamma, alpha)
        if plan_fi:
            Plan.append(plan_fi)
        else:
            print("No valid plan found in S_fi!")
    if Plan:
        print("=========================")
        print(" || Final compilation  ||")
        print("=========================")
        best_all_plan = min(Plan, key=lambda p: p[0])
        best_all_plan = list(best_all_plan)
        print('Balanced cost: %s, prefix cost:%s, prefix risk: %s, suffix cost:%s, suffix risk:%s' % (
            best_all_plan[0], best_all_plan[2], best_all_plan[3], best_all_plan[5], best_all_plan[6]))
        plan_bad = syn_plan_bad(prod_mdp, best_all_plan[7])
        print('Plan for bad states obtained for %s states in Sd' %
              str(len(best_all_plan[7][3])))
        new_best_all_plan = [[best_all_plan[1], best_all_plan[2], best_all_plan[3]], [
            best_all_plan[4], best_all_plan[5], best_all_plan[6]], best_all_plan[7]]
        new_best_all_plan.append(plan_bad)
        return new_best_all_plan
    else:
        print("No valid plan found")
        return None


def syn_full_plan(prod_mdp, gamma, alpha=1):
    # ----Optimal plan synthesis, total cost over plan prefix and suffix----
    print("==========[Optimal full plan synthesis start]==========")
    Plan = []
    for l, S_fi in enumerate(prod_mdp.Sf):
        print("---for one S_fi---")
        plan = []
        for k, MEC in enumerate(S_fi):
            plan_prefix, prefix_cost, prefix_risk, y_in_sf, Sr, Sd = syn_plan_prefix(
                prod_mdp, MEC, gamma)
            print("Best plan prefix obtained, cost: %s, risk %s" %
                  (str(prefix_cost), str(prefix_risk)))
            if y_in_sf:
                plan_suffix, suffix_cost, suffix_risk = syn_plan_suffix(
                    prod_mdp, MEC, y_in_sf)
                print("Best plan suffix obtained, cost: %s, risk %s" %
                      (str(suffix_cost), str(suffix_risk)))
            else:
                plan_suffix = None
            if plan_prefix and plan_suffix:
                plan.append([[plan_prefix, prefix_cost, prefix_risk, y_in_sf], [
                            plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd]])
        if plan:
            best_k_plan = min(plan, key=lambda p: p[0][1] + alpha*p[1][1])
            Plan.append(best_k_plan)
        else:
            print("No valid found!")
    if Plan:
        print("=========================")
        print(" || Final compilation  ||")
        print("=========================")
        best_all_plan = min(Plan, key=lambda p: p[0][1] + alpha*p[1][1])
        print('Best plan prefix obtained for %s states in Sr' %
              str(len(best_all_plan[0][0])))
        print('cost: %s; risk: %s ' %
              (best_all_plan[0][1], best_all_plan[0][2]))
        print('Best plan suffix obtained for %s states in Sf' %
              str(len(best_all_plan[1][0])))
        print('cost: %s; risk: %s ' %
              (best_all_plan[1][1], best_all_plan[1][2]))
        print('Total cost:%s' %
              (best_all_plan[0][1] + alpha*best_all_plan[1][1]))
        plan_bad = syn_plan_bad(prod_mdp, best_all_plan[2])
        print('Plan for bad states obtained for %s states in Sd' %
              str(len(best_all_plan[2][3])))
        best_all_plan.append(plan_bad)
        return best_all_plan
    else:
        print("No valid plan found")
        return None


def syn_full_plan_rex(prod_mdp, gamma, d, alpha=1):
    # ----Relaxed optimal plan synthesis, total cost over plan prefix and suffix----
    print("==========[Relaxed full plan synthesis start]")
    Plan = []
    for l, S_fi in enumerate(prod_mdp.Sf):
        print("---for one S_fi---")
        plan = []
        for k, MEC in enumerate(S_fi):
            plan_prefix, prefix_cost, prefix_risk, y_in_sf, Sr, Sd = syn_plan_prefix(
                prod_mdp, MEC, gamma)
            print("Best plan prefix obtained, cost: %s, risk %s" %
                  (str(prefix_cost), str(prefix_risk)))
            if y_in_sf:
                plan_suffix, suffix_cost, suffix_risk = syn_plan_suffix_rex(
                    prod_mdp, MEC, d, y_in_sf)
                print("Best plan suffix obtained, cost: %s, risk %s" %
                      (str(suffix_cost), str(suffix_risk)))
            else:
                plan_suffix = None
            if plan_prefix and plan_suffix:
                plan.append([[plan_prefix, prefix_cost, prefix_risk, y_in_sf], [
                            plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd]])
        if plan:
            best_k_plan = min(plan, key=lambda p: p[0][1] + alpha*p[1][1])
            Plan.append(best_k_plan)
        else:
            "No valid found!"
    if Plan:
        print("=========================")
        print(" || Final compilation  ||")
        print("=========================")
        best_all_plan = min(Plan, key=lambda p: p[0][1] + alpha*p[1][1])
        print('Best plan prefix obtained for %s states in Sr' %
              str(len(best_all_plan[0][0])))
        print('cost: %s; risk: %s ' %
              (best_all_plan[0][1], best_all_plan[0][2]))
        print('Best plan suffix obtained for %s states in Sf' %
              str(len(best_all_plan[1][0])))
        print('cost: %s; risk: %s ' %
              (best_all_plan[1][1], best_all_plan[1][2]))
        plan_bad = syn_plan_bad(prod_mdp, best_all_plan[2])
        print('Plan for bad states obtained for %s states in Sd' %
              str(len(best_all_plan[2][3])))
        best_all_plan.append(plan_bad)
        return best_all_plan
    else:
        print("No valid plan found")
        return None


def syn_plan_prefix(prod_mdp, MEC, gamma):
    # ----Synthesize optimal plan prefix to reach accepting MEC or SCC----
    # ----with bounded risk and minimal expected total cost----
    print("===========[plan prefix synthesis starts]===========")
    sf = MEC[0]
    ip = MEC[1]  # force convergence to ip
    delta = 0.01
    for init_node in prod_mdp.graph['initial']:
        path_init = single_source_shortest_path(prod_mdp, init_node)
        print('Reachable from init size: %s' % len(list(path_init.keys())))
        if not set(path_init.keys()).intersection(sf):
            print("Initial node can not reach sf")
            return None, None, None, None, None, None
        Sn = set(path_init.keys()).difference(sf)
        # ----find bad states that can not reach MEC
        simple_digraph = DiGraph()
        simple_digraph.add_edges_from(((v, u) for u, v in prod_mdp.edges()))
        path = single_source_shortest_path(
            simple_digraph, random.sample(ip, 1)[0])
        reachable_set = set(path.keys())
        print('States that can reach sf, size: %s' % str(len(reachable_set)))
        Sd = Sn.difference(reachable_set)
        Sr = Sn.intersection(reachable_set)
        # #--------------
        print('Sn size: %s; Sd inside size: %s; Sr inside size: %s' %
              (len(Sn), len(Sd), len(Sr)))
        # ---------solve lp------------
        print('-----')
        print('ORtools for prefix starts now')
        print('-----')
        try:
            # if True:
            Y = defaultdict(float)
            prefix_solver = pywraplp.Solver.CreateSolver('GLOP')
            # create variables
            for s in Sr:
                for u in prod_mdp.nodes[s]['act'].copy():
                    Y[(s, u)] = prefix_solver.NumVar(
                        0, 1000, 'y[(%s, %s)]' % (s, u))
            print('Variables added')
            # set objective
            obj = 0
            for s in Sr:
                for t in prod_mdp.successors(s):
                    prop = prod_mdp[s][t]['prop'].copy()
                    for u in prop.keys():
                        pe = prop[u][0]
                        ce = prop[u][1]
                        obj += Y[(s, u)]*pe*ce
            prefix_solver.Minimize(obj)
            print('Objective function set')
            # add constraints
            # ------------------------------
            y_to_sd = 0.0
            y_to_sf = 0.0
            for s in Sr:
                for t in prod_mdp.successors(s):
                    if t in Sd:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            pe = prop[u][0]
                            y_to_sd += Y[(s, u)]*pe
                    elif t in sf:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            pe = prop[u][0]
                            y_to_sf += Y[(s, u)]*pe
            # prefix_solver.Add(y_to_sf+y_to_sd >= delta)
            prefix_solver.Add(y_to_sf >= (1.0-gamma-delta)*(y_to_sf+y_to_sd))
            print('Risk constraint added')
            # --------------------
            for t in Sr:
                node_y_in = 0.0
                node_y_out = 0.0
                for u in prod_mdp.nodes[t]['act']:
                    node_y_out += Y[(t, u)]
                for f in prod_mdp.predecessors(t):
                    if f in Sr:
                        prop = prod_mdp[f][t]['prop'].copy()
                        for uf in prop.keys():
                            node_y_in += Y[(f, uf)]*prop[uf][0]
                if t == init_node:
                    prefix_solver.Add(node_y_out == 1.0 + node_y_in)
                else:
                    prefix_solver.Add(node_y_out == node_y_in)
            print('Initial node flow balanced')
            print('Middle node flow balanced')
            # ----------------------
            # solve
            print('--optimization starts--')
            status = prefix_solver.Solve()
            if status == pywraplp.Solver.OPTIMAL:
                print('Solution:')
                print('Objective value =', prefix_solver.Objective().Value())
                print('Advanced usage:')
                print('Problem solved in %f milliseconds' %
                      prefix_solver.wall_time())
                print('Problem solved in %d iterations' %
                      prefix_solver.iterations())
            else:
                print('The problem does not have an optimal solution.')
                return None, None, None, None, None, None
            # plan
            plan_prefix = dict()
            for s in Sr:
                norm = 0
                U = []
                P = []
                U_total = prod_mdp.nodes[s]['act'].copy()
                for u in U_total:
                    norm += Y[(s, u)].solution_value()
                for u in U_total:
                    U.append(u)
                    if norm > 0.01:
                        P.append(Y[(s, u)].solution_value()/norm)
                    else:
                        P.append(1.0/len(U_total))
                plan_prefix[s] = [U, P]
            print("----Prefix plan generated")
            cost = prefix_solver.Objective().Value()
            print("----Prefix cost computed, cost: %.2f" % cost)
            # compute the risk given the plan prefix
            risk = 0.0
            y_to_sd = 0.0
            y_to_sf = 0.0
            for s in Sr:
                for t in prod_mdp.successors(s):
                    if t in Sd:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            pe = prop[u][0]
                            y_to_sd += Y[(s, u)].solution_value()*pe
                    elif t in sf:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            pe = prop[u][0]
                            y_to_sf += Y[(s, u)].solution_value()*pe
            if (y_to_sd+y_to_sf) > 0:
                risk = y_to_sd/(y_to_sd+y_to_sf)
            print('y_to_sd: %s; y_to_sf: %s, y_to_sd+y_to_sf: %s' %
                  (y_to_sd, y_to_sf, y_to_sd+y_to_sf))
            print("----Prefix risk computed: %s" % str(risk))
            # compute the input flow to the suffix
            y_in_sf = dict()
            for s in Sn:
                for t in prod_mdp.successors(s):
                    if t in sf:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            pe = prop[u][0]
                            if t not in y_in_sf:
                                y_in_sf[t] = Y[(s, u)].solution_value()*pe
                            else:
                                y_in_sf[t] += Y[(s, u)].solution_value()*pe
            # normalize the input flow
            y_total = 0.0
            for s, y in y_in_sf.items():
                y_total += y
            print('actual y_total: %s' % str(y_total))
            if y_total > 0:
                for s in y_in_sf.keys():
                    y_in_sf[s] = y_in_sf[s]/y_total
                print("----Y in Sf computed and normalized")
            # print y_in_sf
            return plan_prefix, cost, risk, y_in_sf, Sr, Sd
        except:
            print("ORTools Error reported")
            return None, None, None, None, None, None


def syn_plan_suffix(prod_mdp, MEC, y_in_sf):
    # ----Synthesize optimal plan suffix to stay within the accepting MEC----
    # ----with minimal expected total cost of accepting cyclic paths----
    print("===========[plan suffix synthesis starts]")
    sf = MEC[0]
    ip = MEC[1]
    act = MEC[2].copy()
    delta = 0.01
    gamma = 0.00
    for init_node in prod_mdp.graph['initial']:
        paths = single_source_shortest_path(prod_mdp, init_node)
        Sn = set(paths.keys()).intersection(sf)
        print('Sf size: %s' % len(sf))
        print('reachable sf size: %s' % len(Sn))
        print('Ip size: %s' % len(ip))
        print('Ip and sf intersection size: %s' % len(Sn.intersection(ip)))
        # ---------solve lp------------
        print('------')
        print('ORtools for suffix starts now')
        print('------')
        try:
            Y = defaultdict(float)
            suffix_solver = pywraplp.Solver.CreateSolver('GLOP')
            # create variables
            for s in Sn:
                for u in act[s]:
                    Y[(s, u)] = suffix_solver.NumVar(0, 1000, 'y[(%s, %s)]' % (s, u))
            print('Variables added: %d' % len(Y))
            # set objective
            obj = 0
            for s in Sn:
                for u in act[s]:
                    for t in prod_mdp.successors(s):
                        prop = prod_mdp[s][t]['prop'].copy()
                        if u in list(prop.keys()):
                            pe = prop[u][0]
                            ce = prop[u][1]
                            obj += Y[(s, u)]*pe*ce
            suffix_solver.Minimize(obj)
            print('Objective added')
            # add constraints
            # --------------------
            for s in Sn:
                constr3 = 0
                constr4 = 0
                for u in act[s]:
                    constr3 += Y[(s, u)]
                for f in prod_mdp.predecessors(s):
                    if (f in Sn) and (s not in ip):
                        prop = prod_mdp[f][s]['prop'].copy()
                        for uf in act[f]:
                            if uf in list(prop.keys()):
                                constr4 += Y[(f, uf)]*prop[uf][0]
                            else:
                                constr4 += Y[(f, uf)]*0.00
                    if (f in Sn) and (s in ip) and (f != s):
                        prop = prod_mdp[f][s]['prop'].copy()
                        for uf in act[f]:
                            if uf in list(prop.keys()):
                                constr4 += Y[(f, uf)]*prop[uf][0]
                            else:
                                constr4 += Y[(f, uf)]*0.00
                if (s in list(y_in_sf.keys())) and (s not in ip):
                    suffix_solver.Add(constr3 == constr4 + y_in_sf[s])
                if (s in list(y_in_sf.keys())) and (s in ip):
                    suffix_solver.Add(constr3 == y_in_sf[s])
                if (s not in list(y_in_sf.keys())) and (s not in ip):
                    suffix_solver.Add(constr3 == constr4)
            print('Balance condition added')
            print('Initial sf condition added')
            # --------------------
            y_to_ip = 0.0
            y_out = 0.0
            for s in Sn:
                for t in prod_mdp.successors(s):
                    if t not in Sn:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_out += Y[(s, u)]*pe
                    elif t in ip:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_to_ip += Y[(s, u)]*pe
            # suffix_solver.Add(y_to_ip+y_out >= delta)
            suffix_solver.Add(y_to_ip >= (1.0-gamma-delta)*(y_to_ip+y_out))
            print('Risk constraint added')
            # ------------------------------
            # solve
            print('--optimization for suffix starts--')
            status = suffix_solver.Solve()
            if status == pywraplp.Solver.OPTIMAL:
                print('Solution:')
                print('Objective value =', suffix_solver.Objective().Value())
                print('Advanced usage:')
                print('Problem solved in %f milliseconds' %
                      suffix_solver.wall_time())
                print('Problem solved in %d iterations' %
                      suffix_solver.iterations())
            else:
                print('The problem does not have an optimal solution.')
                return None, None, None
            # compute optimal plan suffix given the LP solution
            plan_suffix = dict()
            for s in Sn:
                norm = 0
                U = []
                P = []
                for u in act[s]:
                    norm += Y[(s, u)].solution_value()
                for u in act[s]:
                    U.append(u)
                    if norm > 0.01:
                        P.append(Y[(s, u)].solution_value()/norm)
                    else:
                        P.append(1.0/len(act[s]))
                plan_suffix[s] = [U, P]
            print("----Suffix plan added")
            cost = suffix_solver.Objective().Value()
            print("----Suffix cost computed")
            # compute risk given the plan suffix
            risk = 0.0
            y_to_ip = 0.0
            y_out = 0.0
            for s in Sn:
                for t in prod_mdp.successors(s):
                    if t not in Sn:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_out += Y[(s, u)].solution_value()*pe
                    elif t in ip:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_to_ip += Y[(s, u)].solution_value()*pe
            if (y_to_ip+y_out) > 0:
                risk = y_out/(y_to_ip+y_out)
            print('y_out: %s; y_to_ip+y_out: %s' % (y_out, y_to_ip+y_out))
            print("----Suffix risk computed")
            return plan_suffix, cost, risk
        except:
            print("ORtools Error reported")
            return None, None, None


def syn_plan_suffix_rex(prod_mdp, MEC, d, y_in_sf):
    # ----Synthesize optimal plan suffix to stay within the accepting SCC----
    # ----with minimal expected total cost of accepting cyclic paths----
    # ----and penalty over the risk in the suffix
    print("===========[plan suffix synthesis starts]")
    sf = MEC[0]
    ip = MEC[1]
    act = MEC[2].copy()
    delta = 1.0
    for init_node in prod_mdp.graph['initial']:
        paths = single_source_shortest_path(prod_mdp, init_node)
        Sn = set(paths.keys()).intersection(sf)
        print('Sf size: %s' % len(sf))
        print('reachable sf size: %s' % len(Sn))
        print('Ip size: %s' % len(ip))
        print('Ip and sf intersection size: %s' % len(Sn.intersection(ip)))
        # ---------solve lp------------
        print('------')
        print('ORtools for suffix rex starts now')
        print('------')
        try:
            Y = defaultdict(float)
            suffix_rex_solver = pywraplp.Solver.CreateSolver('GLOP')
            # create variables
            for s in Sn:
                for u in act[s]:
                    Y[(s, u)] = suffix_rex_solver.NumVar(0, 1000, 'y[(%s, %s)]' % (s, u))
            print('Variables added: %d' % len(Y))
            # set objective
            obj = 0
            for s in Sn:
                for u in act[s]:
                    for t in prod_mdp.successors(s):
                        prop = prod_mdp[s][t]['prop'].copy()
                        if t in Sn:
                            if u in list(prop.keys()):
                                pe = prop[u][0]
                                ce = prop[u][1]
                                obj += Y[(s, u)]*pe*ce
                        else:
                            if u in list(prop.keys()):
                                obj += Y[(s, u)]*d
            suffix_rex_solver.Minimize(obj)
            print('Objective added')
            # add constraints
            # --------------------
            for s in Sn:
                constr3 = 0
                constr4 = 0
                for u in act[s]:
                    constr3 += Y[(s, u)]
                for f in prod_mdp.predecessors(s):
                    if (f in Sn) and (s not in ip):
                        prop = prod_mdp[f][s]['prop'].copy()
                        for uf in act[f]:
                            if uf in list(prop.keys()):
                                constr4 += Y[(f, uf)]*prop[uf][0]
                            else:
                                constr4 += Y[(f, uf)]*0.00
                    if (f in Sn) and (s in ip) and (f != s):
                        prop = prod_mdp[f][s]['prop'].copy()
                        for uf in act[f]:
                            if uf in list(prop.keys()):
                                constr4 += Y[(f, uf)]*prop[uf][0]
                            else:
                                constr4 += Y[(f, uf)]*0.00
                if (s in list(y_in_sf.keys())) and (s not in ip):
                    suffix_rex_solver.Add(constr3 == constr4 + y_in_sf[s])
                if (s in list(y_in_sf.keys())) and (s in ip):
                    suffix_rex_solver.Add(constr3 == y_in_sf[s])
                if (s not in list(y_in_sf.keys())) and (s not in ip):
                    suffix_rex_solver.Add(constr3 == constr4)
            print('Balance condition added')
            print('Initial sf condition added')
            # --------------------
            y_to_ip = 0.0
            y_out = 0.0
            for s in Sn:
                for t in prod_mdp.successors(s):
                    if t not in Sn:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_out += Y[(s, u)]*pe
                    elif t in ip:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_to_ip += Y[(s, u)]*pe
            suffix_rex_solver.Add(y_to_ip+y_out >= delta)
            print('Risk constraint added')
            # --------------------
            # ------------------------------
            # solve
            print('--optimization for suffix rex starts')
            status = suffix_rex_solver.Solve()
            if status == pywraplp.Solver.OPTIMAL:
                print('Solution:')
                print('Objective value =', suffix_rex_solver.Objective().Value())
                print('Advanced usage:')
                print('Problem solved in %f milliseconds' %
                      suffix_rex_solver.wall_time())
                print('Problem solved in %d iterations' %
                      suffix_rex_solver.iterations())
            else:
                print('The problem does not have an optimal solution.')
                return None, None, None
            # ------------------------------
            # compute optimal plan suffix given the LP solution
            plan_suffix = dict()
            for s in Sn:
                norm = 0
                U = []
                P = []
                for u in act[s]:
                    norm += Y[(s, u)].solution_value()
                for u in act[s]:
                    U.append(u)
                    if norm > 0.01:
                        P.append(Y[(s, u)].solution_value()/norm)
                    else:
                        P.append(1.0/len(act[s]))
                plan_suffix[s] = [U, P]
            print("----Suffix plan added")
            cost = suffix_rex_solver.Objective().Value()
            print("----Suffix cost computed")
            # compute the risk in the plan suffix
            risk = 0.0
            y_to_ip = 0.0
            y_out = 0.0
            for s in Sn:
                for t in prod_mdp.successors(s):
                    if t not in Sn:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_out += Y[(s, u)].solution_value()*pe
                    elif t in ip:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            if u in act[s]:
                                pe = prop[u][0]
                                y_to_ip += Y[(s, u)].solution_value()*pe
            if (y_to_ip+y_out) > 0:
                risk = y_out/(y_to_ip+y_out)
            print('y_out: %s; y_to_ip+y_out: %s' % (y_out, y_to_ip+y_out))
            print("----Suffix risk computed")
            return plan_suffix, cost, risk
        except:
            print("ORtools Error reported")
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
        Ud = prod_mdp.nodes[sd]['act'].copy()
        proj_cost = dict()
        postqf = prod_mdp.graph['dra'].successors(qf)
        # print 'postqf size',len(postqf)
        for xt in prod_mdp.graph['mdp'].successors(xf):
            if xt != xf:
                prop = prod_mdp.graph['mdp'][xf][xt]['prop']
                for u in prop.keys():
                    prob_edge = prop[u][0]
                    cost = prop[u][1]
                    label = prod_mdp.graph['mdp'].nodes[xt]['label']
                    for lt in label.keys():
                        prob_label = label[lt]
                        dist = dict()
                        for qt in postqf:
                            if (xt, lt, qt) in Sf.union(Sr):
                                dist[qt] = prod_mdp.graph['dra'].check_distance_for_dra_edge(
                                    lf, qf, qt)
                        if list(dist.keys()):
                            qt = min(list(dist.keys()), key=lambda q: dist[q])
                            if u not in list(proj_cost.keys()):
                                proj_cost[u] = 0
                            else:
                                proj_cost[u] += prob_edge*prob_label*dist[qt]
        # policy for bad states
        U = []
        P = []
        if list(proj_cost.keys()):
            # print 'sd',sd
            u_star = min(list(proj_cost.keys()), key=lambda u: proj_cost[u])
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
    # ----choose the randomized action by the optimal policy----
    # recall that {best_plan = [plan_prefix, prefix_cost, prefix_risk, y_in_sf],
    # [plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd], plan_bad]}
    plan_prefix = best_plan[0][0]
    plan_suffix = best_plan[1][0]
    plan_bad = best_plan[3]
    if (prod_state in plan_prefix):
        # print 'In prefix'
        U = plan_prefix[prod_state][0]
        P = plan_prefix[prod_state][1]
        rdn = random.random()
        pc = 0
        for k, p in enumerate(P):
            pc += p
            if pc > rdn:
                break
        # print 'action chosen: %s' %str(U[k])
        return U[k], 0
    elif (prod_state in plan_suffix):
        # print 'In suffix'
        U = plan_suffix[prod_state][0]
        P = plan_suffix[prod_state][1]
        rdn = random.random()
        pc = 0
        for k, p in enumerate(P):
            pc += p
            if pc > rdn:
                break
        # print 'action chosen: %s' %str(U[k])
        if prod_state in best_plan[2][1]:
            return U[k], 10
        else:
            return U[k], 1
    elif (prod_state in plan_bad):
        # print 'In bad states'
        U = plan_bad[prod_state][0]
        P = plan_bad[prod_state][1]
        rdn = random.random()
        pc = 0
        for k, p in enumerate(P):
            pc += p
            if pc > rdn:
                break
        # print 'action chosen: %s' %str(U[k])
        return U[k], 2


def rd_act_by_plan(prod_mdp, best_plan, prod_state, I):
    # ----choose the randomized action by the optimal policy----
    # optimal prefix with round-robin as the plan suffix
    # recall that {best_plan = [plan_prefix, prefix_cost, prefix_risk, y_in_sf],
    # [plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd], plan_bad]}
    plan_prefix = best_plan[0][0]
    plan_suffix = best_plan[1][0]
    plan_bad = best_plan[3]
    if (prod_state in plan_prefix):
        # print 'In prefix'
        U = plan_prefix[prod_state][0]
        P = plan_prefix[prod_state][1]
        rdn = random.random()
        pc = 0
        for k, p in enumerate(P):
            pc += p
            if pc > rdn:
                break
        # print 'action chosen: %s' %str(U[k])
        return U[k], 0, I
    elif (prod_state in plan_suffix):
        # print 'In suffix'
        U = plan_suffix[prod_state][0]
        # print 'action chosen: %s' %str(U[k])
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
        # print 'In bad states'
        U = plan_bad[prod_state][0]
        P = plan_bad[prod_state][1]
        rdn = random.random()
        pc = 0
        for k, p in enumerate(P):
            pc += p
            if pc > rdn:
                break
        # print 'action chosen: %s' %str(U[k])
        return U[k], 2, I


def syn_plan_comb(prod_mdp, S_fi, gamma, alpha):
    # ----Synthesize combined optimal plan prefix and suffix
    # ----with bounded risk and minimal mean total cost----
    print("===========[plan prefix synthesis starts]===========")
    Sf = set()
    for mec in S_fi:
        Sf.update(mec[0])  # mec = (sf, ip, act)
    delta = 1.0
    for init_node in prod_mdp.graph['initial']:
        path_init = single_source_shortest_path(prod_mdp, init_node)
        print('Reachable from init size: %s' % len(list(path_init.keys())))
        if not set(path_init.keys()).intersection(Sf):
            print("Initial node can not reach Sf")
            return None, None, None, None, None, None, None, None
        Sn = set(path_init.keys()).difference(Sf)
        # ----find bad states that can not reach MEC
        simple_digraph = DiGraph()
        simple_digraph.add_edges_from(((v, u) for u, v in prod_mdp.edges()))
        reachable_set = set()
        ip_set = set()
        for mec in S_fi:
            ip = mec[1]
            path = single_source_shortest_path(
                simple_digraph, random.sample(ip, 1)[0])
            reachable_set.update(set(path.keys()))
            ip_set.update(ip)
        print('States that can reach Sf, size: %s' % str(len(reachable_set)))
        Sd = Sn.difference(reachable_set)
        Sr = Sn.intersection(reachable_set)
        # #--------------
        print('Sn size: %s; Sd inside size: %s; Sr inside size: %s' %
              (len(Sn), len(Sd), len(Sr)))
        # ---------solve lp------------
        print('-----')
        print('ORtools starts now')
        print('-----')
        try:
            comb_solver = pywraplp.Solver.CreateSolver('GLOP')
            # --------------------
            # prefix variable
            Y = defaultdict(float)
            # create variables
            for s in Sr:
                for u in prod_mdp.nodes[s]['act'].copy():
                    Y[(s, u)] = comb_solver.NumVar(
                        0, 1000, 'y[(%s, %s)]' % (s, u))
            print('Prefix Y variables added')
            # --------------------
            # suffix variables
            Z = defaultdict(float)
            for mec in S_fi:
                sf = mec[0]
                ip = mec[1]
                act = mec[2].copy()
                for s in sf:
                    for u in act[s]:
                        Z[(s, u)] = comb_solver.NumVar(
                            0, 1000, name='z[(%s, %s)]' % (s, u))
            print('Suffix Z variables added')
            # set objective
            obj = 0
            for s in Sr:
                for t in prod_mdp.successors(s):
                    prop = prod_mdp[s][t]['prop'].copy()
                    for u in prop.keys():
                        pe = prop[u][0]
                        ce = prop[u][1]
                        obj += alpha*Y[(s, u)]*pe*ce
            for mec in S_fi:
                sf = mec[0]
                ip = mec[1]
                act = mec[2].copy()
                for s in sf:
                    for u in act[s]:
                        for t in prod_mdp.successors(s):
                            prop = prod_mdp[s][t]['prop'].copy()
                            if u in list(prop.keys()):
                                pe = prop[u][0]
                                ce = prop[u][1]
                                obj += (1.0-alpha)*Z[(s, u)]*pe*ce
            comb_solver.Minimize(obj)
            print('alpha*Prefix + (1.0-alpha)*Suffix cost Objective function set')
            # add constraints
            # ------------------------------
            y_to_sd = 0.0
            y_to_sf = 0.0
            for s in Sr:
                for t in prod_mdp.successors(s):
                    if t in Sd:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            pe = prop[u][0]
                            y_to_sd += Y[(s, u)]*pe
                    elif t in sf:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            pe = prop[u][0]
                            y_to_sf += Y[(s, u)]*pe
            comb_solver.Add(y_to_sf+y_to_sd >= delta)
            comb_solver.Add(y_to_sf >= (1.0-gamma)*(y_to_sf+y_to_sd))
            print('Prefix risk constraint added')
            # --------------------
            for t in Sr:
                node_y_in = 0.0
                node_y_out = 0.0
                for u in prod_mdp.nodes[t]['act']:
                    node_y_out += Y[(t, u)]
                for f in prod_mdp.predecessors(t):
                    if f in Sr:
                        prop = prod_mdp[f][t]['prop'].copy()
                        for uf in prop.keys():
                            node_y_in += Y[(f, uf)]*prop[uf][0]
                if t == init_node:
                    comb_solver.Add(node_y_out == 1.0 + node_y_in)
                else:
                    comb_solver.Add(node_y_out == node_y_in)
            print('Prefix initial node flow balanced')
            print('Prefix middle node flow balanced')
            # ----------------------
            for mec in S_fi:
                sf = mec[0]
                ip = mec[1]
                act = mec[2].copy()
                for s in sf:
                    constr3 = 0
                    constr4 = 0
                    y_in_sf = 0
                    for u in act[s]:
                        constr3 += Z[(s, u)]
                    for f in prod_mdp.predecessors(s):
                        if (f in sf) and (s not in ip):
                            prop = prod_mdp[f][s]['prop'].copy()
                            for uf in act[f]:
                                if uf in list(prop.keys()):
                                    constr4 += Z[(f, uf)]*prop[uf][0]
                                else:
                                    constr4 += Z[(f, uf)]*0.00
                        if (f in sf) and (s in ip) and (f != s):
                            prop = prod_mdp[f][s]['prop'].copy()
                            for uf in act[f]:
                                if uf in list(prop.keys()):
                                    constr4 += Z[(f, uf)]*prop[uf][0]
                                else:
                                    constr4 += Z[(f, uf)]*0.00
                        elif (f not in sf):
                            prop = prod_mdp[f][s]['prop'].copy()
                            for uf in prop.keys():
                                pe = prop[uf][0]
                                y_in_sf += Y[(f, uf)]*pe
                    if (s not in ip):
                        comb_solver.Add(constr3 == constr4 + y_in_sf)
                    elif (s in ip):
                        comb_solver.Add(constr3 == y_in_sf)
                print('Suffix balance condition added')
                print('Suffix initial y_in_sf condition added')
                # --------------------
                y_to_ip = 0.0
                y_out = 0.0
                for s in sf:
                    for t in prod_mdp.successors(s):
                        if t not in sf:
                            prop = prod_mdp[s][t]['prop'].copy()
                            for u in prop.keys():
                                if u in act[s]:
                                    pe = prop[u][0]
                                    y_out += Z[(s, u)]*pe
                        elif t in ip:
                            prop = prod_mdp[s][t]['prop'].copy()
                            for u in prop.keys():
                                if u in act[s]:
                                    pe = prop[u][0]
                                    y_to_ip += Z[(s, u)]*pe
                comb_solver.Add(y_to_ip+y_out >= delta)
                comb_solver.Add(y_to_ip >= (1.0-gamma)*(y_to_ip+y_out))
                print('Suffix risk constraint added')
            # ----------------------
            # solve
            print('--optimization starts--')
            status = comb_solver.Solve()
            if status == pywraplp.Solver.OPTIMAL:
                print('Solution:')
                print('Objective value =', suffix_solver.Objective().Value())
                print('\nAdvanced usage:')
                print('Problem solved in %f milliseconds' %
                      suffix_solver.wall_time())
                print('Problem solved in %d iterations' %
                      suffix_solver.iterations())
            else:
                print('The problem does not have an optimal solution.')
                return None, None, None, None, None, None, None, None
            # ------------------------------
            # compute plan prefix given the LP solution
            plan_prefix = dict()
            for s in Sr:
                norm = 0
                U = []
                P = []
                U_total = prod_mdp.nodes[s]['act'].copy()
                for u in U_total:
                    norm += Y[(s, u)].solution_value()
                for u in U_total:
                    U.append(u)
                    if norm > 0.01:
                        P.append(Y[(s, u)].solution_value()/norm)
                    else:
                        P.append(1.0/len(U_total))
                plan_prefix[s] = [U, P]
            print("----Prefix plan generated")
            # --------------------
            # --------------------
            # compute the risk given the plan prefix
            cost_pre = 0.0
            risk_pre = 0.0
            y_to_sd = 0.0
            y_to_sf = 0.0
            for s in Sr:
                for t in prod_mdp.successors(s):
                    prop = prod_mdp[s][t]['prop'].copy()
                    for u in prop.keys():
                        pe = prop[u][0]
                        ce = prop[u][1]
                        cost_pre += Y[(s, u)].solution_value()*pe*ce
                    if t in Sd:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            pe = prop[u][0]
                            y_to_sd += Y[(s, u)].solution_value()*pe
                    elif t in sf:
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            pe = prop[u][0]
                            y_to_sf += Y[(s, u)].solution_value()*pe
            if (y_to_sd+y_to_sf) > 0:
                risk_pre = y_to_sd/(y_to_sd+y_to_sf)
            print('y_to_sd: %s; y_to_sd+y_to_sf: %s' %
                  (y_to_sd, y_to_sd+y_to_sf))
            print("----Prefix risk computed: %s" % str(risk_pre))
            print("----Prefix cost computed: %s" % str(cost_pre))
            # --------------------
            # compute optimal plan suffix given the LP solution
            plan_suffix = dict()
            for mec in S_fi:
                sf = mec[0]
                ip = mec[1]
                act = mec[2].copy()
                for s in sf:
                    norm = 0
                    U = []
                    P = []
                    for u in act[s]:
                        norm += Z[(s, u)].solution_value()
                    for u in act[s]:
                        U.append(u)
                        if norm > 0.01:
                            P.append(Z[(s, u)].solution_value()/norm)
                        else:
                            P.append(1.0/len(act[s]))
                    plan_suffix[s] = [U, P]
            print("----Suffix plan added")
            # compute risk given the plan suffix
            Risk_suf = []
            Cost_suf = []
            for mec in S_fi:
                risk_suf = 0.0
                cost_suf = 0.0
                y_to_ip = 0.0
                y_out = 0.0
                sf = mec[0]
                ip = mec[1]
                act = mec[2].copy()
                for s in sf:
                    for t in prod_mdp.successors(s):
                        prop = prod_mdp[s][t]['prop'].copy()
                        for u in prop.keys():
                            if u in act[s]:
                                pe = prop[u][0]
                                ce = prop[u][1]
                                cost_suf += Z[(s, u)].solution_value()*pe*ce
                        if (t not in sf):
                            prop = prod_mdp[s][t]['prop'].copy()
                            for u in prop.keys():
                                if u in act[s]:
                                    pe = prop[u][0]
                                    y_out += Z[(s, u)].solution_value()*pe
                        elif (t in ip):
                            prop = prod_mdp[s][t]['prop'].copy()
                            for u in prop.keys():
                                if u in act[s]:
                                    pe = prop[u][0]
                                    y_to_ip += Z[(s, u)].solution_value()*pe
                if (y_to_ip+y_out) > 0:
                    risk_suf = y_out/(y_to_ip+y_out)
                Risk_suf.append(risk_suf)
                Cost_suf.append(cost_suf)
                print('one AMEC: y_out: %s; y_to_ip+y_out: %s' %
                      (y_out, y_to_ip+y_out))
            print("----Suffix risk computed: %s" % str(Risk_suf))
            print("----Suffix cost computed: %s" % str(Cost_suf))
            total_cost = model.objval
            print("----alpha*Prefix + (1-alpha)*Suffix cost computed")
            return total_cost, plan_prefix, cost_pre, risk_pre, plan_suffix, Cost_suf, Risk_suf, [Sf, ip_set, Sr, Sd]
        except:
            print("ORtools Error reported")
            return None, None, None, None, None, None, None, None
