#!/bin/bash
# -*- coding: UTF-8 -*-
python my_leader.py $1 $2 & #第一架飞机是 引领，其余飞机是 跟随。$1是飞机类型如iris，$2是数量
uav_num=1
while(( $uav_num< $2 )) 
do
    python my_follower.py $1 $uav_num $2&
    let "uav_num++"
done
