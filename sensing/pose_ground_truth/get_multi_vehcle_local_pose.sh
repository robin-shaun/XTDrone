#!/bin/bash
iris_num=0
typhoon_h480_num=0
solo_num=0
plane_num=3
rover_num=0
standard_vtol_num=0
tiltrotor_num=0
tailsitter_num=0


python2.7 get_local_pose.py iris $iris_num&
python2.7 get_local_pose.py typhoon_h480 $typhoon_h480_num&
python2.7 get_local_pose.py solo $solo_num&
python2.7 get_local_pose.py plane $plane_num&
python2.7 get_local_pose.py rover $rover_num&
python2.7 get_local_pose.py standard_vtol $standard_vtol_num&
python2.7 get_local_pose.py tiltrotor $tiltrotor_num&
python2.7 get_local_pose.py tailsitter $tailsitter_num
