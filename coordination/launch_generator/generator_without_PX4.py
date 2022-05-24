from jinja2 import Template

class Body_Param:

    def __init__(self,x,y,vehicle,ID) -> None:
        self.x = x
        self.y = y
        self.vehicle = vehicle
        self.ID = ID
    


body_temp_str = '''
<group ns="{{ params.vehicle }}_{{params.ID}}">
    <!-- vehicle pose -->
    <arg name="x" default="{{params.x}}"/>
    <arg name="y" default="{{params.y}}"/>
    <arg name="z" default="0.2"/>
    <arg name="R" default="0"/>
    <arg name="P" default="0"/>
    <arg name="Y" default="0"/>
    <arg name="vehicle" default="{{ params.vehicle }}"/>
    <arg name="ID" default="{{params.ID}}"/>
    <!-- generate sdf vehicle model -->
    <arg name="model_description" default="$(find px4)/Tools/sitl_gazebo/models/$(arg vehicle)/$(arg vehicle).sdf"/>
    <!-- spawn vehicle -->
    <node name="$(arg vehicle)_$(arg ID)_spawn" output="screen" pkg="gazebo_ros" type="spawn_model" args="-sdf -file $(arg model_description) -model $(arg vehicle)_$(arg ID) -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)"/>
</group>

'''

welcome_msg = """
                      ,----,                                                      
                    ,/   .`|                                                      
 ,--,     ,--,    ,`   .'  :   ,---,                                              
 |'. \   / .`|  ;    ;     / .'  .' `\                                            
 ; \ `\ /' / ;.'___,/    ,',---.'     \   __  ,-.   ,---.        ,---,            
 `. \  /  / .'|    :     | |   |  .`\  |,' ,'/ /|  '   ,'\   ,-+-. /  |           
  \  \/  / ./ ;    |.';  ; :   : |  '  |'  | |' | /   /   | ,--.'|'   |   ,---.   
   \  \.'  /  `----'  |  | |   ' '  ;  :|  |   ,'.   ; ,. :|   |  ,"' |  /     \  
    \  ;  ;       '   :  ; '   | ;  .  |'  :  /  '   | |: :|   | /  | | /    /  | 
   / \  \  \      |   |  ' |   | :  |  '|  | '   '   | .; :|   | |  | |.    ' / | 
  ;  /\  \  \     '   :  | '   : | /  ; ;  : |   |   :    ||   | |  |/ '   ;   /| 
./__;  \  ;  \    ;   |.'  |   | '` ,/  |  , ;    \   \  / |   | |--'  '   |  / | 
|   : / \  \  ;   '---'    ;   :  .'     ---'      `----'  |   |/      |   :    | 
;   |/   \  ' |            |   ,.'                         '---'        \   \  /  
`---'     `--`             '---'                                         `----'   
                                                                                  
Welcome to use the XTDrone multi-vehicle launch file generator!
"""

input_msg=  """
0.  rover
1.  plane
2.  typhoon_h480
3.  solo
4.  iris
5.  tiltrotor
6.  tailsitter
7.  standard_vtol
8.  plane_gimbal
Enter the TYPE_ID to add a vehicle type.
Then enter f to generate!
(defalt type is "iris")
  """

head_temp_str = """<?xml version="1.0"?>
<launch>
    <!-- launches Gazebo environment and spawns vehicle -->
    <!-- vehicle model and world -->
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/outdoor2.world"/>
    <!-- gazebo configs -->
    <arg name="gui" default="true"/>
    <arg name="debug" default="false"/>
    <arg name="verbose" default="false"/>
    <arg name="paused" default="false"/>
    <!-- Gazebo sim -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="gui" value="$(arg gui)"/>
        <arg name="world_name" value="$(arg world)"/>
        <arg name="debug" value="$(arg debug)"/>
        <arg name="verbose" value="$(arg verbose)"/>
        <arg name="paused" value="$(arg paused)"/>
    </include>
"""

TYPE_ID_DICT = {
    "rover_with_lidar_stereo" : 0,
    "plane" : 1,
    "typhoon_h480" : 2,
    "solo_stereo_camera" : 3,
    "iris_stereo_camera" : 4,
    "tiltrotor" : 5,
    "tailsitter" : 6,
    "standard_vtol" : 7,
    "plane_gimbal" : 8
}
ID_TYPE_DICT = {v : k for k, v in TYPE_ID_DICT.items()}
print(welcome_msg)
TYPE_ID = 4
num_of_all = 0
num_of_type = [0]*8
row_of_type = [0]*8

while TYPE_ID != 'f':
    TYPE_ID = input(input_msg)
    if TYPE_ID>='0' and  TYPE_ID<='7':
        num_of_type[int(TYPE_ID)] = int( input("Enter the num of "+ID_TYPE_DICT[int(TYPE_ID)]+" :" ) )
        row_of_type[int(TYPE_ID)] = int( input("Enter the row num of "+ID_TYPE_DICT[int(TYPE_ID)]+" :" ) )
    elif TYPE_ID == 'f':
        for i in range(7):
            if  num_of_type[i] != 0:
                print(ID_TYPE_DICT[i]+' num : '+str(num_of_type[i]))
        print('generating........') 
    else:
        print("error!please enter a id_in_all between 0 to 7!")

sum_of_row = sum(row_of_type) 

body_template= Template(body_temp_str)

with open('multi_vehicle.launch','w') as f:
    f.write(head_temp_str)
    row_in_all = 0
    row_in_all = 0
    id_in_all = 0
    for type_id in range(8):
        type_num = num_of_type[type_id]
        row_in_type = row_of_type[type_id]
        sdf_name = ID_TYPE_DICT[type_id]    
        if type_num > 0:
            for id_in_type in range(0,type_num):
                x = (row_in_all*3 +(id_in_type%row_in_type )*3)
                y = (( id_in_type//row_in_type +1)*3  ) 
                body_lines_param = Body_Param(x,y,sdf_name,id_in_type)
                body_lines = body_template.render(params=body_lines_param)
                f.write(body_lines)
                id_in_all+=1           
            row_in_all += row_in_type 

    f.write('</launch>\n')
    f.write('<!--the launch file is generated by XTDrone multi-vehicle generator.py  -->')
    print ("all down!")
