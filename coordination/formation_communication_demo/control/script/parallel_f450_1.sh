#!/bin/bash
#ssh nvidia@parallel_f450_1
#tmux
cmd=$(which tmux) # tmux path
session=f450_1 # session name
winName=uav

if [ -z $cmd ]; then
    echo "You need to install tmux."
    exit 1
fi

$cmd kill-session -t $session

$cmd has -t $session

if [ $? != 0 ]; then
    #new session, window name is "env"
    $cmd new -s $session -d -n $winName
    $cmd splitw -v -p 50 -t $winName
    $cmd splitw -h -p 50 -t $winName
    $cmd selectp -t 0
    $cmd splitw -h -p 50 -t $winName
    $cmd selectp -t 1
    $cmd splitw -v -p 50 -t $winName
fi

$cmd selectp -t 0
$cmd send-keys "roslaunch mavros real_fly_1.launch" C-m
$cmd selectp -t 1
$cmd send-keys "sleep 1;roslaunch vrpn_client_ros sample.launch" C-m
$cmd selectp -t 2
$cmd send-keys "sleep 5;cd parallel-simulation/src/control/script/;python vicon_transfer.py" C-m
$cmd selectp -t 3
$cmd send-keys "sleep 10;cd parallel-simulation/;source devel/setup.bash;cd src/control/script/" C-m
$cmd selectp -t 4
$cmd send-keys "sleep 5;rostopic echo /UAV0/mavros/local_position/pose" C-m


$cmd att -t $session

