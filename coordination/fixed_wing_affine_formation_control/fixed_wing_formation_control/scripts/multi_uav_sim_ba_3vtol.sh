#本脚本启动的是三机仿真，不能行间注释，否则无法使用

#!/bin/bash
gnome-terminal --window  -e 'bash -c "cd ~/catkin_ws; catkin build; exec bash"' \
--tab -e 'bash -c "sleep 2; source ~/.bashrc; roslaunch px4 fixed_wing_formation_control_leader.launch;  exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun fixed_wing_formation_control pack_fw0_states; exec bash"' \
--tab -e 'bash -c "sleep 11; rosrun fixed_wing_formation_control pack_fw1_states; exec bash"' \
--tab -e 'bash -c "sleep 12; rosrun fixed_wing_formation_control pack_fw2_states; exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control follower0_main; exec bash"' \
--tab -e 'bash -c "sleep 9; rosrun fixed_wing_formation_control follower1_main; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun fixed_wing_formation_control follower2_main; exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control switch_fw0_mode; exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control switch_fw1_mode; exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control switch_fw2_mode; exec bash"' \

