from MDP_TG.mdp import Motion_MDP
from MDP_TG.dra import Dra, Product_Dra
from MDP_TG.lp import syn_full_plan
from MDP_TG.vis import visualize_run


# -------- toy example -------
node_dict = {
    's1': {frozenset(['a', ]): 0.9,
           frozenset(): 0.1, },
    's2': {frozenset(['a', 'b']): 0.6,
           frozenset(): 0.4, },
    's3': {frozenset(['b']): 0.5,
           frozenset(['c']): 0.4,
           frozenset(): 0.1, },
    's4': {frozenset(['a', ]): 0.3,
           frozenset(['a', 'c']): 0.4,
           frozenset(): 0.3, }, }

initial_node = 's1'
initial_label = set()

U = [tuple('st'), tuple('fr'), tuple('bc')]
C = [1, 3, 4]
qf = 0.8
qb = 0.7
qf1 = 0.4
qf2 = 0.5
qs = 0.99
edge_dict = {
    # s1
    ('s1', U[1], 's2'): (qf, C[1]),
    ('s1', U[1], 's1'): (1-qf, C[1]),
    ('s1', U[0], 's1'): (qs, C[0]),
    ('s1', U[0], 's2'): (1-qs, C[0]),
    # s2
    ('s2', U[2], 's1'): (qb, C[2]),
    ('s2', U[2], 's2'): (1-qb, C[2]),
    ('s2', U[0], 's2'): (qs, C[0]),
    ('s2', U[0], 's3'): (1-qs, C[0]),
    ('s2', U[1], 's3'): (qf, C[1]),
    ('s2', U[1], 's2'): (1-qf, C[1]),
    # s3
    ('s3', U[2], 's2'): (qb, C[2]),
    ('s3', U[2], 's3'): (1-qb, C[2]),
    ('s3', U[0], 's3'): (qs, C[0]),
    ('s3', U[0], 's4'): (1-qs, C[0]),
    ('s3', U[1], 's4'): (qf, C[1]),
    ('s3', U[1], 's3'): (1-qf, C[1]),
    # s4
    ('s4', U[2], 's3'): (qb, C[2]),
    ('s4', U[2], 's4'): (1-qb, C[2]),
    ('s4', U[0], 's4'): (qs, C[0]),
    ('s4', U[0], 's1'): (1-qs, C[0]),
    ('s4', U[1], 's2'): (qf2, C[1]),
    ('s4', U[1], 's1'): (qf1, C[1]),
    ('s4', U[1], 's4'): (1-qf1-qf2, C[1]),
}

# ----
motion_mdp = Motion_MDP(node_dict, edge_dict, U, initial_node, initial_label)

# ----
seq_formula1 = "& F a & F b F c"
seq_formula2 = "F & a F & b F c"
sur_formula1 = "& G F a & G F b G F c"
sur_formula2 = "& G F a G F b"
seq_sur_formula1 = "G F i a F i b F c"
seq_sur_formula2 = "G F & a F & b F c"
dra = Dra(sur_formula1)

# ----
prod_dra = Product_Dra(motion_mdp, dra)
prod_dra.compute_S_f()
# prod_dra.dotify()

allowed_risk = 0.0
best_all_plan = syn_full_plan(prod_dra, allowed_risk)

# ----------------------------------------
print("----------------------------------------")
print("||||||||Simulation start||||||||||||||||")
print("----------------------------------------")
total_T = 20
state_seq = [initial_node, ]
label_seq = [initial_label, ]
N = 5
n = 0
print("Try %s simulations of length %s" % (str(N), str(total_T)))

XX = []
LL = []
UU = []
MM = []
PP = []

while (n < N):
    print('=======simulation %s starts=======' % str(n))
    X, L, U, M, PX = prod_dra.execution(
        best_all_plan, total_T, state_seq, label_seq)
    # print 'Product State trajectory: %s' %str(PX)
    # print 'State trajectory: %s' %str(X)
    # print 'Label trajectory: %s' %str(L)
    # print 'Control Actions: %s' %str(U)
    # print 'Marker sequence: %s' %str(M)
    print('=======simulation %s ends=======' % str(n))
    XX.append(X)
    LL.append(L)
    UU.append(U)
    MM.append(M)
    PP.append(PX)
    n += 1

#visualize_run(XX, LL, UU, MM, 'surv_result')
