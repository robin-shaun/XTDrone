#The number of drones
import sys
uav_num=int(sys.argv[1])

welcome_msg = 
"""
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


ekf_dir="./ekf2_config/"
launch_dir="./launch/"
with open('launch_head','r') as f:
    launch_head=[]
    launch_head=f.read()
with open('launch_temp','r') as f:
    launch_lines=[]
    for line in f.readlines():
         launch_lines.append(line)
with open('launch_head_1.9','r') as f:
    launch_head_1_9=[]
    launch_head_1_9=f.read()
with open('launch_temp_1.9','r') as f:
    launch_lines_1_9=[]
    for line in f.readlines():
         launch_lines_1_9.append(line)
with open('ekf2_temp','r') as f:
    ekf2lines=[]
    for line in f.readlines():
        if line!='\n':
         ekf2lines.append(line)

with open('launch_head_1.11','r') as f:
    launch_head_1_11=[]
    launch_head_1_11=f.read()
with open('launch_temp_1.11','r') as f:
    launch_lines_1_11=[]
    for line in f.readlines():
         launch_lines_1_11.append(line)


for num in range(1,uav_num+1):
    iris_name="iris_"+str(num)
    mavlink_1=34570-1+num*2
    mavlink_2=mavlink_1+1
    onboard=14540+num-1
    SITL=24560+(num-1)*2
    with open(ekf_dir+iris_name,'w') as f:
        for line in ekf2lines:
            if "MAV_SYS_ID" in line:
                line ="param set MAV_SYS_ID "+str(num)             
                f.write('%s\n' %line)
            elif "SITL_UDP_PRT" in line:
                line ="param set SITL_UDP_PRT "+str(SITL)
                f.write('%s\n' %line)
            elif "mavlink" in line:
                continue
            elif "replay trystart" in line:
                continue
            elif "logger start" in line:
                continue
            else:
                f.write('%s' %line)
        f.write("mavlink start -x -u %d -r 4000000\n" %mavlink_1)
        f.write("mavlink start -x -u %d -r 4000000 -m onboard -o %d\n" %(mavlink_2,onboard))
        f.write("mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u %d\n" %mavlink_1)
        f.write("mavlink stream -r 50 -s LOCAL_POSITION_NED -u %d\n" %mavlink_1)
        f.write("mavlink stream -r 50 -s GLOBAL_POSITION_INT -u %d\n" %mavlink_1)
        f.write("mavlink stream -r 50 -s ATTITUDE -u %d\n" %mavlink_1)
        f.write("mavlink stream -r 50 -s ATTITUDE_QUATERNION -u %d\n" %mavlink_1)
        f.write("mavlink stream -r 50 -s ATTITUDE_TARGET -u %d\n" %mavlink_1)    
        f.write("mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u %d\n" %mavlink_1)    
        f.write("mavlink stream -r 20 -s RC_CHANNELS -u %d\n" %mavlink_1)    
        f.write("mavlink stream -r 250 -s HIGHRES_IMU -u %d\n" %mavlink_1)    
        f.write("mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u %d\n" %mavlink_1)    
        f.write("logger start -e -t\n") 
        f.write("mavlink boot_complete\n") 
        f.write("replay trystart\n")
        print iris_name," down"

with open('./launch/multi_uav.launch','w') as f:
    f.write(launch_head)
    for num in range(1,uav_num+1):
        mavlink_1=34570-1+num*2
        mavlink_2=mavlink_1+1
        onboard=14540+num-1
        SITL=24560+(num-1)*2
        for line in launch_lines:
            if "<!-- UAV" in line:
                f.write("     <!-- UAV%d-->\n" %num)
            elif "<group ns" in line:
                f.write('''     <group ns="uav%d">\n''' %num)
            elif '''<arg name="ID" value="1"/>''' in line:
                f.write('''        <arg name="ID" value="%d"/>\n''' %num)
            elif "udp://:" in line:
                f.write('''        <arg name="fcu_url" default="udp://:%d@127.0.0.1:%d"/>\n''' %(onboard,mavlink_2))
            elif "rcS" in line:
                f.write('''            <arg name="rcS" value="$(find px4)/posix-configs/SITL/init/$(arg est)/$(arg vehicle)_%d"/>\n''' %num)
            elif "mavlink_udp_port" in line:
                f.write('''            <arg name="mavlink_udp_port" value="%d"/>\n'''%SITL)
            elif '''name="x"''' in line:
                f.write('''            <arg name="x" value="0"/>\n''')
            elif '''name="y"''' in line:
                f.write('''            <arg name="y" value="%d"/>\n''' %( (2*(num%2)-1 )*(num-1) ) )
            else:
                f.write('%s' %line)
        f.write("\n")
            
    f.write('</launch>')
    print ".launch for 1.8  down"

# 生成1.9版本的launch文件
with open('./launch_1.9/multi_uav.launch','w') as f:
    f.write(launch_head_1_9)
    for num in range(1,uav_num):
        mavlink_1=34570-1+num*2
        mavlink_2=mavlink_1+1
        onboard=14540+num
        SITL=24560+(num)*2
        TCP=4560+num
        for line in launch_lines_1_9:
            if "<!-- UAV" in line:
                f.write("     <!-- UAV%d-->\n" %num)
            elif "<group ns" in line:
                f.write('''     <group ns="uav%d">\n''' %num)
            elif '''<arg name="ID" value="1"/>''' in line:
                f.write('''        <arg name="ID" value="%d"/>\n''' %num)
            elif "udp://:" in line:
                f.write('''        <arg name="fcu_url" default="udp://:%d@127.0.0.1:%d"/>\n''' %(onboard,mavlink_2))
            elif "mavlink_udp_port" in line:
                f.write('''            <arg name="mavlink_udp_port" value="%d"/>\n'''%SITL)
            elif "mavlink_tcp_port" in line:
                f.write('''            <arg name="mavlink_tcp_port" value="%d"/>\n'''%TCP)
            elif '''name="x"''' in line:
                f.write('''            <arg name="x" value="0"/>\n''')
            elif '''name="y"''' in line:
                f.write('''            <arg name="y" value="%d"/>\n''' %( (2*(num%2)-1 )*(num) ) )
            else:
                f.write('%s' %line)
        f.write("\n")
            
    f.write('</launch>')
    print ".launch for 1.9 down"      


# 生成1.9版本的launch文件
vehicle_name = []
with open('./launch_1.9/multi_uav.launch','w') as f:
    f.write(launch_head_1_11)
    for num in range(1,uav_num):
        mavlink_1=34570-1+num*2
        mavlink_2=mavlink_1+1
        onboard=14540+num
        SITL=24560+(num)*2
        TCP=4560+num
        for line in launch_lines_1_9:
            if "<!-- UAV" in line:
                f.write("     <!-- UAV%d-->\n" %num)
            elif "<group ns" in line:
                f.write('''     <group ns="%s_%d">\n''' %(vehicle_name[num],num))
            elif '''<arg name="ID" value="1"/>''' in line:
                f.write('''        <arg name="ID" value="%d"/>\n''' %num)
            elif "udp://:" in line:
                f.write('''        <arg name="fcu_url" default="udp://:%d@127.0.0.1:%d"/>\n''' %(onboard,mavlink_2))
            elif "mavlink_udp_port" in line:
                f.write('''            <arg name="mavlink_udp_port" value="%d"/>\n'''%SITL)
            elif "mavlink_tcp_port" in line:
                f.write('''            <arg name="mavlink_tcp_port" value="%d"/>\n'''%TCP)
            elif '''name="x"''' in line:
                f.write('''            <arg name="x" value="0"/>\n''')
            elif '''name="y"''' in line:
                f.write('''            <arg name="y" value="%d"/>\n''' %( (2*(num%2)-1 )*(num) ) )
            else:
                f.write('%s' %line)
        f.write("\n")
            
    f.write('</launch>')
    print ".launch for 1.9 down"   




       
    

