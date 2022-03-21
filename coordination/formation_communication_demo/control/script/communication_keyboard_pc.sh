#!/bin/bash
#ssh nvidia@parallel_f450_1
#tmux
cmd=$(which tmux) # tmux path
session=f450 # session name
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

fi

$cmd selectp -t 0
$cmd send-keys "roslaunch px4 iris20_exam1.launch" C-m
$cmd selectp -t 1
$cmd send-keys "sleep 1;python multirotor_keyboard_control iris 20 vel" C-m
$cmd selectp -t 2
$cmd send-keys "sleep 5;python communication_verify_1.py" C-m
$cmd selectp -t 3
# edit number and python file each time
$cmd send-keys "sleep 10;mpiexec -n 20 python sim_addleader_1.py " C-m
$cmd att -t $session

