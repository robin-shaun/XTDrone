#!/bin/bash
python leader.py $1 $2 &
uav_num=1
while(( $uav_num< $2 )) 
do
    python follower_consensus.py $1 $uav_num $2&
    let "uav_num++"
done
