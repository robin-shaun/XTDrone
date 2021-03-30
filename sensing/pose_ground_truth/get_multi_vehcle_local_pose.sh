#!/bin/bash
iris_num=3
typhoon_h480_num=2
solo_num=0
plane_num=0
rover_num=0
standard_vtol_num=0
tiltrotor_num=0
tailsitter_num=0


python get_local_pose.py iris $iris_num&
python get_local_pose.py typhoon_h480 $typhoon_h480_num&
python get_local_pose.py solo $solo_num&
python get_local_pose.py plane $plane_num&
python get_local_pose.py rover $rover_num&
python get_local_pose.py standard_vtol $standard_vtol_num&
python get_local_pose.py tiltrotor $tiltrotor_num&
python get_local_pose.py tailsitter $tailsitter_num