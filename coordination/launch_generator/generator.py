import sys

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
Enter the TYPE_ID to add a vehicle type.
Then enter f to generate!
(defalt type is "iris")
  """

TYPE_ID_DICT = {
    "rover_with_lidar_stereo" : 0,
    "plane" : 1,
    "typhoon_h480_stereo" : 2,
    "solo_stereo_camera" : 3,
    "iris_stereo_camera" : 4,
    "tiltrotor" : 5,
    "tailsitter" : 6,
    "standard_vtol" : 7,
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
        print("error!please enter a id_in_allber between 0 to 7!")

sum_of_row = sum(row_of_type)    



with open('launch_head_1.11','r') as f:
    launch_head=[]
    launch_head=f.read()
with open('launch_temp_1.11','r') as f:
    launch_lines=[]
    for line in f.readlines():
         launch_lines.append(line)


with open('multi_vehicle.launch','w') as f:
    f.write(launch_head)
    row_in_all = 0
    id_in_all = 0
    for type_id in range(8):
        type_num = num_of_type[type_id]
        row_in_type = row_of_type[type_id]
        sdf_name = ID_TYPE_DICT[type_id]

        # For example,
        # While "iris_stereo_camera" is the model name,
        # and we only need the "iris" to publish those topics.
        
        if "h480" in sdf_name or "vtol" in sdf_name:
            type_name = sdf_name.split('_')[0]+'_'+sdf_name.split('_')[1]
        else:
            type_name = sdf_name.split('_')[0]

        if type_num > 0:
            
            for id_in_type in range(0,type_num):
                offboard_local=34580+id_in_all
                offboard_remote=24540+id_in_all
                SITL=18570+id_in_all
                TCP=4560+id_in_all
                for line in launch_lines:
                    if "<!-- UAV" in line:
                        f.write("     <!-- "+type_name+("_%d -->\n" %id_in_type) )
                    elif "<group ns" in line:
                        f.write('''     <group ns="%s_%d">\n''' %(type_name,id_in_type)  )
                    elif '''<arg name="ID" value="0"/>''' in line:
                        f.write('''            <arg name="ID" value="%d"/>\n''' %id_in_all)
                    elif '''<arg name="ID_in_group" value="0"/>''' in line:
                        f.write('''            <arg name="ID_in_group" value="%d"/>\n''' %id_in_type)
                    elif "udp://:" in line:
                        f.write('''            <arg name="fcu_url" default="udp://:%d@localhost:%d"/>\n''' %(offboard_remote,offboard_local))
                    elif "mavlink_udp_port" in line:
                        f.write('''            <arg name="mavlink_udp_port" value="%d"/>\n'''%SITL)
                    elif "mavlink_tcp_port" in line:
                        f.write('''            <arg name="mavlink_tcp_port" value="%d"/>\n'''%TCP)
                    elif '''<arg name="vehicle" value=''' in line:
                        f.write('''            <arg name="vehicle" value="%s"/>\n'''%type_name)
                    elif '''<arg name="sdf" value=''' in line:
                        f.write('''            <arg name="sdf" value="%s"/>\n'''%sdf_name)    
                    elif '''name="x"''' in line:
                        f.write('''            <arg name="x" value="%d"/>\n'''%(row_in_all*3 +(id_in_type%row_in_type )*3 ))
                    elif '''name="y"''' in line:
                        f.write('''            <arg name="y" value="%d"/>\n'''  %(( id_in_type//row_in_type +1)*3  ) )
                    else:
                        f.write('%s' %line) 
                f.write("\n")
                id_in_all+=1

            row_in_all += row_in_type    
                
                

    f.write('</launch>\n')
    f.write('<!--the launch file is generated by XTDrone multi-vehicle generator.py  -->')
    print ("all down!")


    
