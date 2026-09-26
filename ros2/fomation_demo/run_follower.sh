#!/bin/bash
python3 leader.py $1 $2&
uav_id=1
while(($uav_id<$2))
do
    python3 follower.py $1 $uav_id $2&
    let "uav_id++"
done
   