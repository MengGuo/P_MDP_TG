from MDP_TG.mdp import Motion_MDP
from MDP_TG.dra import Dra, Product_Dra
import networkx

import time
import pickle

t0 = time.time()

# ----
all_base = '& G F base1 & G F base2 G F base3'
order1 = 'G i supply X U ! supply base'
order2 = 'G i base X U ! base supply'
order = '& %s %s' % (order1, order2)
task1 = '& %s & G ! obstacle %s' % (all_base, order2)
task2 = '& %s G F supply' % all_base
task3 = '& %s %s' % (all_base, order2)
dra = Dra(all_base)
t1 = time.time()
print('------------------------------')
print('DRA done, time: %s' % str(t1-t0))


print('------------------------------')
t2 = time.time()
clean_dra = dict()
clean_dra['name'] = 'dra_all_base'
clean_dra['init'] = dra.graph['initial']
clean_dra['states'] = dra.nodes()
clean_dra['symbols'] = dra.graph['symbols']
clean_dra['edge_guard'] = networkx.get_edge_attributes(dra, 'guard_string')
clean_dra['accept'] = dra.graph['accept']

pickle.dump(clean_dra, open('nx_dra_model.p', 'wb'))
print('------------------------------')
print('Save to clean pickle data, time: %s' % str(t2-t1))
