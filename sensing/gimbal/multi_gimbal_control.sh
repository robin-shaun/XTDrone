#!/bin/bash
iris_num=0
typhoon_h480_num=6
solo_num=0
plane_num=0
rover_num=0
standard_vtol_num=0
tiltrotor_num=0
tailsitter_num=0

vehicle_num=0
while(( $vehicle_num< iris_num)) 
do
    python gimbal_control.py iris $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< typhoon_h480_num)) 
do
    python gimbal_control.py typhoon_h480 $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< solo_num)) 
do
    python gimbal_control.py solo $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< plane_num)) 
do
    python gimbal_control.py $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< rover_num)) 
do
    python gimbal_control.py $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< standard_vtol_num)) 
do
    python gimbal_control.py standard_vtol $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< tiltrotor_num)) 
do
    python gimbal_control.py tiltrotor $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< tailsitter_num)) 
do
    python gimbal_control.py tailsitter $vehicle_num&
    let "vehicle_num++"
done
