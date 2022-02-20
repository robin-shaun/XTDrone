#!/bin/bash
while(true)
do
    python3 ego_swarm_goal.py iris 0 9 7 0.7&
    python3 ego_swarm_goal.py iris 1 9 5 0.7&
    python3 ego_swarm_goal.py iris 2 9 3 0.7&
    python3 ego_swarm_goal.py iris 3 9 1 0.7&
    python3 ego_swarm_goal.py iris 4 9 -1 0.7&
    python3 ego_swarm_goal.py iris 5 9 -3 0.7&
    python3 ego_swarm_goal.py iris 6 9 -5 0.7&
    python3 ego_swarm_goal.py iris 7 9 -7 0.7
done