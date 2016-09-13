P_MDP_TG
========

Planner for Markov Decision Processes with Temporal Goals 

-----
Description
-----
this package contains the implementation for policy synthesis algorithms given a probabilistically-labeled Markov Decision Process (MDP) (as the robot motion model) and a Linear Temporal Logic (LTL) formula (as the robot task). It outputs a stationary  and finite-memory policy consists of plan prefix and plan suffix, such that the controlled robot behavior fulfills the task with a given lower-bounded risk and minimizes the expected total cost. 

-----
Features
-----
* Allows probabilistic labels on MDP states.
* Tunable trade-off between risk and expected total cost in the plan prefix.
* Linear programs for solving constrained stochastic shortest path (SSP).
* Optimization over both plan prefix and suffix.
* Relaxed policy generation for cases where no accepting end components (AECs) exist.
* Interface between LTL formula, Buchi Automaton, Deterministic Robin Automaton and NetworkX graph objects.
* Computing maximal accepting end components (MAEC) of MDPs.

<p align="center">  
  <img src="https://github.com/MengGuo/P_MDP_TG/blob/master/MDP_TG/figures/risk.png" width="800"/>
</p>

----
Dependence
----
* install python packages like Networkx, ply
* ltl2ba executable, from http://www.lsv.ens-cachan.fr/%7Egastin/ltl2ba/download.php
* ltl2dstar executable, from http://www.ltl2dstar.de
* Gurobi solver for linear programs, from http://www.gurobi.com