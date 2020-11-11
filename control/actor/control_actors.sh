#!/bin/bash
python control_actor.py 0 -34 34 5 &
python control_actor.py 1 -38 -36 5 &
python control_actor.py 2 2 -2 5 &
python control_actor.py 3 -2 2 5 &
python control_actor.py 4 -2 -2 5 &
python control_actor.py 5 0 0 5

