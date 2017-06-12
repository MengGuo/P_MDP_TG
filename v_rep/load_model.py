import pickle

best_plan = pickle.load(open('best_plan.p', 'rb'))

prod_dra_edges, initial_state  = pickle.load(open('prod_dra_edges.p', 'rb'))

WS_node_dict, WS_d = pickle.load(open('ws_model.p', 'rb'))

motion_mdp_edges = pickle.load(open('motion_mdp_edges.p', 'rb'))





# prod_dra_edges, over attribute 'prop'

# motion_mdp_edges, over attribute 'prop'

# best_plan = [plan_prefix, prefix_cost, prefix_risk, y_in_sf], [plan_suffix, suffix_cost, suffix_risk], [MEC[0], MEC[1], Sr, Sd], plan_bad]
