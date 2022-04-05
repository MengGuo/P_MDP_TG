from MDP_TG.mdp import Motion_MDP
from MDP_TG.dra import Dra, Product_Dra
from MDP_TG.lp import syn_full_plan, syn_full_plan_rex, syn_full_plan_comb
from MDP_TG.vis import visualize_run, visualize_world_paths, visualize_world, run_movie, analyze_events, compute_suffix_mean_cost


import time

t0 = time.time()

# -------- real example -------
WS_node_dict = {
    # base stations
    (9.0, 1.0): {frozenset(['base1', 'base']): 1.0,
                 frozenset(): 0.0, },
    (9.0, 9.0): {frozenset(['base2', 'base']): 1.0,
                 frozenset(): 0.0, },
    (1.0, 9.0): {frozenset(['base3', 'base']): 1.0,
                 frozenset(): 0.0, },
    (3.0, 3.0): {frozenset(): 1.0, },
    (3.0, 5.0): {frozenset(): 1.0, },
    (7.0, 5.0): {frozenset(): 1.0, },
    (3.0, 9.0): {frozenset(): 1.0, },
    (3.0, 7.0): {frozenset(): 1.0, },
    (1.0, 5.0): {frozenset(['supply', ]): 0.8,
                 frozenset(): 0.2, },
    (5.0, 3.0): {frozenset(['supply', ]): 0.2,
                 frozenset(): 0.8, },
    (9.0, 5.0): {frozenset(['supply', ]): 0.5,
                 frozenset(): 0.5, },
    (5.0, 9.0): {frozenset(['supply', ]): 0.5,
                 frozenset(): 0.5, },
    (1.0, 3.0): {frozenset(): 1.0, },
    (1.0, 1.0): {frozenset(): 1.0, },
    (1.0, 7.0): {frozenset(): 1.0, },
    (3.0, 1.0): {frozenset(): 1.0, },
    (5.0, 1.0): {frozenset(): 1.0, },
    (7.0, 1.0): {frozenset(): 1.0, },
    (5.0, 1.0): {frozenset(['obstacle', ]): 0.99,
                 frozenset(): 0.01, },
    (7.0, 3.0): {frozenset(): 1.0, },
    (5.0, 5.0): {frozenset(): 1.0, },
    (5.0, 7.0): {frozenset(): 1.0, },
    (7.0, 7.0): {frozenset(): 1.0, },
    (7.0, 9.0): {frozenset(): 1.0, },
    (9.0, 3.0): {frozenset(): 1.0, },
    (9.0, 7.0): {frozenset(): 1.0, },
}

# ----
# visualize_world(WS_d, WS_node_dict, 'world')
# t1 = time.time()
# print 'visualize world done, time: %s' %str(t1-t0)
# ------------------------------------
robot_nodes = dict()
for loc, prop in WS_node_dict.items():
    for d in ['N', 'S', 'E', 'W']:
        robot_nodes[(loc[0], loc[1], d)] = prop
# ------------------------------------
initial_node = (1.0, 1.0, 'N')
initial_label = frozenset()

U = [tuple('FR'), tuple('BK'), tuple('TR'), tuple('TL'), tuple('ST')]
C = [2, 4, 3, 3, 1]
COST = dict()
for k, u in enumerate(U):
    COST[U[k]] = C[k]
P_FR = [0.1, 0.8, 0.1]
P_BK = [0.15, 0.7, 0.15]
P_TR = [0.05, 0.9, 0.05]
P_TL = [0.05, 0.9, 0.05]
P_ST = [0.005, 0.99, 0.005]
# -------------
robot_edges = dict()
for fnode in robot_nodes.keys():
    fx = fnode[0]
    fy = fnode[1]
    fd = fnode[2]
    # action FR
    u = U[0]
    c = C[0]
    if fd == 'N':
        t_nodes = [(fx-2, fy+2, fd), (fx, fy+2, fd), (fx+2, fy+2, fd)]
    if fd == 'S':
        t_nodes = [(fx-2, fy-2, fd), (fx, fy-2, fd), (fx+2, fy-2, fd)]
    if fd == 'E':
        t_nodes = [(fx+2, fy-2, fd), (fx+2, fy, fd), (fx+2, fy+2, fd)]
    if fd == 'W':
        t_nodes = [(fx-2, fy-2, fd), (fx-2, fy, fd), (fx-2, fy+2, fd)]
    for k, tnode in enumerate(t_nodes):
        if tnode in list(robot_nodes.keys()):
            robot_edges[(fnode, u, tnode)] = (P_FR[k], c)
    # action BK
    u = U[1]
    c = C[1]
    if fd == 'N':
        t_nodes = [(fx-2, fy-2, fd), (fx, fy-2, fd), (fx+2, fy-2, fd)]
    if fd == 'S':
        t_nodes = [(fx-2, fy+2, fd), (fx, fy+2, fd), (fx+2, fy+2, fd)]
    if fd == 'E':
        t_nodes = [(fx-2, fy-2, fd), (fx-2, fy, fd), (fx-2, fy+2, fd)]
    if fd == 'W':
        t_nodes = [(fx+2, fy-2, fd), (fx+2, fy, fd), (fx+2, fy+2, fd)]
    for k, tnode in enumerate(t_nodes):
        if tnode in list(robot_nodes.keys()):
            robot_edges[(fnode, u, tnode)] = (P_BK[k], c)
    # action TR
    u = U[2]
    c = C[2]
    if fd == 'N':
        t_nodes = [(fx, fy, 'N'), (fx, fy, 'E'), (fx, fy, 'S')]
    if fd == 'S':
        t_nodes = [(fx, fy, 'S'), (fx, fy, 'W'), (fx, fy, 'N')]
    if fd == 'E':
        t_nodes = [(fx, fy, 'E'), (fx, fy, 'S'), (fx, fy, 'W')]
    if fd == 'W':
        t_nodes = [(fx, fy, 'W'), (fx, fy, 'N'), (fx, fy, 'E')]
    for k, tnode in enumerate(t_nodes):
        if tnode in list(robot_nodes.keys()):
            robot_edges[(fnode, u, tnode)] = (P_TR[k], c)
    # action TL
    u = U[3]
    c = C[3]
    if fd == 'S':
        t_nodes = [(fx, fy, 'S'), (fx, fy, 'E'), (fx, fy, 'N')]
    if fd == 'N':
        t_nodes = [(fx, fy, 'N'), (fx, fy, 'W'), (fx, fy, 'S')]
    if fd == 'W':
        t_nodes = [(fx, fy, 'W'), (fx, fy, 'S'), (fx, fy, 'E')]
    if fd == 'E':
        t_nodes = [(fx, fy, 'E'), (fx, fy, 'N'), (fx, fy, 'W')]
    for k, tnode in enumerate(t_nodes):
        if tnode in list(robot_nodes.keys()):
            robot_edges[(fnode, u, tnode)] = (P_TL[k], c)
    # action ST
    u = U[4]
    c = C[4]
    if fd == 'S':
        t_nodes = [(fx, fy, 'W'), (fx, fy, 'S'), (fx, fy, 'E')]
    if fd == 'N':
        t_nodes = [(fx, fy, 'W'), (fx, fy, 'N'), (fx, fy, 'E')]
    if fd == 'W':
        t_nodes = [(fx, fy, 'S'), (fx, fy, 'W'), (fx, fy, 'N')]
    if fd == 'E':
        t_nodes = [(fx, fy, 'N'), (fx, fy, 'E'), (fx, fy, 'S')]
    for k, tnode in enumerate(t_nodes):
        if tnode in list(robot_nodes.keys()):
            robot_edges[(fnode, u, tnode)] = (P_ST[k], c)
# ----
motion_mdp = Motion_MDP(robot_nodes, robot_edges, U,
                        initial_node, initial_label)
t2 = time.time()
print('MDP done, time: %s' % str(t2-t0))

# ----
ordered_reach = '& F G base3 & F base1 & F base2 & F base3 G ! obstacle'
all_base = '& G F base1 & G F base2 & G F base3 G ! obstacle'
order1 = 'G i supply X U ! supply base'
order2 = 'G i base X U ! base supply'
order = '& %s %s' % (order1, order2)
task1 = '& %s & G ! obstacle %s' % (all_base, order2)
task2 = '& %s G F supply' % all_base
task3 = '& %s %s' % (all_base, order2)
dra = Dra(task1)
t3 = time.time()
print('DRA done, time: %s' % str(t3-t2))

# ----
prod_dra = Product_Dra(motion_mdp, dra)
# prod_dra.dotify()
t41 = time.time()
print('Product DRA done, time: %s' % str(t41-t3))
# ----
prod_dra.compute_S_f()
t42 = time.time()
print('Compute accepting MEC done, time: %s' % str(t42-t41))

# ------
gamma = 0.0
alpha = 0.0
best_all_plan = syn_full_plan_comb(prod_dra, gamma, alpha)
t5 = time.time()
print('Plan synthesis done, time: %s' % str(t5-t42))

# ----------------------------------------
print("----------------------------------------")
print("||||||||Simulation start||||||||||||||||")
print("----------------------------------------")
total_T = 500
state_seq = [initial_node, ]
label_seq = [initial_label, ]
N = 100
n = 0
print("Try %s simulations of length %s" % (str(N), str(total_T)))

XX = []
LL = []
UU = []
MM = []
PP = []

while (n < N):
    # print '=======simulation %s starts=======' %str(n)
    X, L, U, M, PX = prod_dra.execution(
        best_all_plan, total_T, state_seq, label_seq)
    # print 'Product State trajectory: %s' %str(PX)
    # print 'State trajectory: %s' %str(X)
    # print 'Label trajectory: %s' %str(L)
    # print 'Control Actions: %s' %str(U)
    # print 'Marker sequence: %s' %str(M)
    # print '=======simulation %s ends=======' %str(n)
    XX.append(X)
    LL.append(L)
    UU.append(U)
    MM.append(M)
    PP.append(PX)
    #run_movie(motion_mdp, WS_d, WS_node_dict, X, L, U, M)
    n += 1

t6 = time.time()
print('MC simulation done, time: %s' % str(t6-t5))

# # visualize_world_paths(WS_d, WS_node_dict, XX, LL, UU, MM, 'GFabc')
# # t7 = time.time()
# # print 'Visualize paths done, time: %s' %str(t7-t6)

# analyze_events(MM, LL)

mean_cost = compute_suffix_mean_cost(UU, MM, COST)
print('mean_total_cost_suffix:%s, alpha:%s' % (str(mean_cost), str(alpha)))
