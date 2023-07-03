#本脚本启动的是三机仿真

#bash
gnome-terminal --window  -e 'bash -c "cd ~/catkin_ws; catkin build; exec bash"' \
--tab -e 'bash -c "sleep 2; source ~/.bashrc; roslaunch px4 fixed_wing_formation_control.launch;  exec bash"' \
--tab -e 'bash -c "sleep 4; rosrun fixed_wing_formation_control pack_leader_states; exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun fixed_wing_formation_control vir_sim_leader;  exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun fixed_wing_formation_control pack_fw1_states;exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun fixed_wing_formation_control pack_fw2_states;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control follower1_main;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control follower2_main;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control switch_fw1_mode;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control switch_fw2_mode;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control switch_leader_mode;exec bash"'\ 

