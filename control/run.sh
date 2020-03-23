#!/bin/bash
python leader.py $1 &
uav_num=2
while(( $uav_num<= $1 )) 
do
    python follower_vel_control.py $uav_num $1&
    #echo $uav_num
    let "uav_num++"
done