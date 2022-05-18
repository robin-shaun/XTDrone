# Required: rrtbot_description package

read -p "URDF from file:"
rosrun gazebo_ros spawn_model -file `rospack find rrbot_description`/urdf/rrbot.xml -urdf -y 1 -model rrbot1 -robot_namespace robot2

read -p "URDF from parameter server using roslaunch and xacro:"
roslaunch parameter_server_test.launch

read -p "SDF from local model database:"
rosrun gazebo_ros spawn_model -file `echo $GAZEBO_MODEL_PATH`/coke_can/model.sdf -sdf -model coke_can1 -y 0.2 -x -0.3

read -p "SDF from local model database with Deprecated tag:"
rosrun gazebo_ros spawn_model -file `echo $GAZEBO_MODEL_PATH`/coke_can/model.sdf -gazebo -model coke_can2 -y 1.2 -x -0.3

read -p "SDF from the online model database:"
rosrun gazebo_ros spawn_model -database coke_can -sdf -model coke_can3 -y 2.2 -x -0.3


