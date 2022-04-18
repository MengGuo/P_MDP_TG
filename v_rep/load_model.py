import pickle

best_plan = pickle.load(open('best_plan.p', 'rb'))

prod_dra_edges, initial_state = pickle.load(open('prod_dra_edges.p', 'rb'))

WS_node_dict, WS_d = pickle.load(open('ws_model.p', 'rb'))

motion_mdp_edges = pickle.load(open('motion_mdp_edges.p', 'rb'))


WS_d = 0.25
grid = WS_d*2    # grid size

######
edges = list(motion_mdp_edges.keys())
states = [e[0] for e in edges] + [e[1] for e in edges]
states_x = [s[0] for s in states]
states_y = [s[1] for s in states]
x_max = max(states_x)
x_min = min(states_x)
y_max = max(states_y)
y_min = min(states_y)


# prod_dra_edges, over attribute 'prop'

# motion_mdp_edges, over attribute 'prop'

# best_plan = [plan_prefix, prefix_cost, prefix_risk, y_in_sf], [plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd], plan_bad]
