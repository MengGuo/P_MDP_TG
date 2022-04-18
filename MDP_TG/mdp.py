# -*- coding: utf-8 -*-

from math import sqrt
from networkx.classes.digraph import DiGraph
from networkx import strongly_connected_components_recursive

# ----------------------------------


class Motion_MDP(DiGraph):
    # ----construct probabilistic-labeled MDP----
    def __init__(self, node_dict, edge_dict, U, initial_node, initial_label):
        DiGraph.__init__(self, name='motion_mdp',
                         init_state=initial_node, init_label=initial_label)
        for (n, prob_label) in node_dict.items():
            self.add_node(n, label=prob_label, act=set())
        print("-------Motion MDP Initialized-------")
        self.add_edges(edge_dict, U)
        print("%s states and %s edges" %
              (str(len(self.nodes())), str(len(self.edges()))))
        self.unify_mdp()

    def add_edges(self, edge_dict, U):
        self.graph['U'] = set()
        for u in U:
            self.graph['U'].add(tuple(u))
        for edge, attri in edge_dict.items():
            f_node = edge[0]
            u = edge[1]
            t_node = edge[2]
            if (f_node, t_node) in self.edges():
                prop = self[f_node][t_node]['prop']
                prop[tuple(u)] = [attri[0], attri[1]]
            else:
                prob_cost = dict()
                #--- prob, cost
                prob_cost[tuple(u)] = [attri[0], attri[1]]
                self.add_edge(f_node, t_node, prop=prob_cost)
        # ----
        for f_node in self.nodes():
            Us = set()
            for t_node in self.successors(f_node):
                prop = self[f_node][t_node]['prop']
                Us.update(set(prop.keys()))
            if Us:
                self.nodes[f_node]['act'] = Us.copy()
            else:
                print('Isolated state')
        print("-------Motion MDP Constructed-------")

    def unify_mdp(self):
        # ----verify the probability sums up to 1----
        for f_node in self.nodes():
            for u in self.nodes[f_node]['act']:
                sum_prob = 0
                N = 0
                for t_node in self.successors(f_node):
                    prop = self[f_node][t_node]['prop']
                    if u in list(prop.keys()):
                        sum_prob += prop[u][0]
                        N += 1
                if sum_prob < 1.0:
                    to_add = (1.0-sum_prob)/N
                    for t_node in self.successors(f_node):
                        prop = self[f_node][t_node]['prop']
                        if u in list(prop.keys()):
                            prop[u][0] += to_add
        print('Unify MDP Done')

# --------------------------------


def find_MECs(mdp, Sneg):
    # ----implementation of Alg.47 P866 of Baier08----
    print('Remaining states size', len(Sneg))
    U = mdp.graph['U']
    A = dict()
    for s in Sneg:
        A[s] = mdp.nodes[s]['act'].copy()
        if not A[s]:
            print("Isolated state")
    MEC = set()
    MECnew = set()
    MECnew.add(frozenset(Sneg))
    # ----
    k = 0
    while MEC != MECnew:
        print("<============iteration %s============>" % k)
        k += 1
        MEC = MECnew
        MECnew = set()
        print("MEC size: %s" % len(MEC))
        print("MECnew size: %s" % len(MECnew))
        for T in MEC:
            R = set()
            T_temp = set(T)
            simple_digraph = DiGraph()
            for s_f in T_temp:
                if s_f not in simple_digraph:
                    simple_digraph.add_node(s_f)
                for s_t in mdp.successors(s_f):
                    if s_t in T_temp:
                        simple_digraph.add_edge(s_f, s_t)
            print("SubGraph of one MEC: %s states and %s edges" % (
                str(len(simple_digraph.nodes())), str(len(simple_digraph.edges()))))
            i = 0
            for Scc in strongly_connected_components_recursive(simple_digraph):
                i += 1
                if (len(Scc) >= 1):
                    for s in Scc:
                        U_to_remove = set()
                        for u in A[s]:
                            for t in mdp.successors(s):
                                if ((u in list(mdp[s][t]['prop'].keys())) and (t not in Scc)):
                                    U_to_remove.add(u)
                        A[s].difference_update(U_to_remove)
                        if not A[s]:
                            R.add(s)
            while R:
                s = R.pop()
                T_temp.remove(s)
                for f in mdp.predecessors(s):
                    if f in T_temp:
                        A[f].difference_update(
                            set(mdp[f][s]['prop'].keys()))
                        if not A[f]:
                            R.add(f)
            j = 0
            for Scc in strongly_connected_components_recursive(simple_digraph):
                j += 1
                if (len(Scc) >= 1):
                    common = set(Scc).intersection(T_temp)
                    if common:
                        MECnew.add(frozenset(common))
    # ---------------
    print('Final MEC and MECnew size:', len(MEC))
    return MEC, A


def find_SCCs(mdp, Sneg):
    # ----simply find strongly connected components----
    print('Remaining states size', len(Sneg))
    SCC = set()
    simple_digraph = DiGraph()
    A = dict()
    for s in mdp.nodes():
        A[s] = mdp.nodes[s]['act'].copy()
    for s_f in Sneg:
        if s_f not in simple_digraph:
            simple_digraph.add_node(s_f)
        for s_t in mdp.successors(s_f):
            if s_t in Sneg:
                simple_digraph.add_edge(s_f, s_t)
    print("SubGraph of one Sf: %s states and %s edges" %
          (str(len(simple_digraph.nodes())), str(len(simple_digraph.edges()))))
    for scc in strongly_connected_components_recursive(simple_digraph):
        SCC.add(frozenset(scc))
    return SCC, A
