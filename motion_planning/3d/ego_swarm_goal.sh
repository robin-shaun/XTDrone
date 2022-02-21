#!/bin/bash
while(true)
do
    python3 ego_swarm_goal.py iris 0 9 5 0.7&
    python3 ego_swarm_goal.py iris 1 9 1.2 0.7&
    python3 ego_swarm_goal.py iris 2 9 -1.2 0.7&
    python3 ego_swarm_goal.py iris 3 9 -4 0.7
done