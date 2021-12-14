#!/bin/bash
python leader.py $1 $2 &
python avoid.py $1 $2 $3 &
uav_id=1
while(( $uav_id< $2 )) 
do
    python follower_consensus.py $1 $uav_id $2 $3 &
    let "uav_id++"
done
