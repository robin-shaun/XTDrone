#!/bin/bash
python leader.py $1 $2 &
python avoid.py $1 $2 vel &
uav_id=1
while(( $uav_id< $2 )) 
do
    python follower.py $1 $uav_id $2 &
    let "uav_id++"
done
