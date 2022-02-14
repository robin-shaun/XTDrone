#!/bin/bash
while(true)
do
    python3 ego_swarm_goal.py iris 0 -7 -9 0.7&
    python3 ego_swarm_goal.py iris 1 3 -7 0.7&
    python3 ego_swarm_goal.py iris 2 -5 -2 0.7&
    python3 ego_swarm_goal.py iris 3 -2 3 0.7
done