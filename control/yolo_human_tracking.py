import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import sys 
from pyquaternion import Quaternion
from darknet_ros_msgs.msg import BoundingBoxes
import math

def darknet_callback(data):
    global find_cnt, twist, cmd, target_height_mask, target_height,theta, get_time
    for target in data.bounding_boxes:
        if(target.id==0):
            print('find human')
            z = height / math.sin(theta)
            u = (target.xmax+target.xmin)/2
            v = (target.ymax+target.ymin)/2
            u_ = u-u_center
            v_ = v-v_center
            u_velocity = -Kp_xy*u_
            v_velocity = -Kp_xy*v_
            x_velocity = v_velocity*z/(v_*math.cos(theta)+fy*math.sin(theta))
            y_velocity = (z*u_velocity-u_*math.cos(theta)*x_velocity)/fx
            twist.linear.x = x_velocity
            twist.linear.y = y_velocity
            twist.linear.z = Kp_z*(target_height-height)
            cmd = ''
            find_cnt = find_cnt + 1
            get_time = False
    
        
def local_pose_callback(data):
    global height, target_height, target_set
    height = data.pose.position.z    
    if not target_set:
        target_height = height     
        target_set = True    

def cam_pose_callback(data):
    global theta
    q = Quaternion(data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z)
    theta = q.yaw_pitch_roll[1]
            
if __name__ == "__main__":
    height = 0  
    target_height = 0
    target_set = False
    find_cnt = 0
    find_cnt_last = 0
    not_find_time = 0
    get_time = False
    twist = Twist()
    cmd = String()
    theta = 0
    u_center=640/2 
    v_center=360/2
    fx = 205.46963709898583
    fy = 205.46963709898583
    Kp_xy = 0.5
    Kp_z = 1
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    rospy.init_node('yolo_human_tracking')
    rospy.Subscriber("/uav_"+vehicle_id+"/darknet_ros/bounding_boxes", BoundingBoxes, darknet_callback,queue_size=1)
    rospy.Subscriber(vehicle_type+'_'+vehicle_id+"/mavros/local_position/pose", PoseStamped, local_pose_callback,queue_size=1)
    rospy.Subscriber('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cam_pose', PoseStamped, cam_pose_callback,queue_size=1)
    cmd_vel_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd_vel_flu', Twist, queue_size=1)
    cmd_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd', String, queue_size=1)
    rate = rospy.Rate(60) 
    while not rospy.is_shutdown():
        rate.sleep()
        cmd_vel_pub.publish(twist)
        cmd_pub.publish(cmd)
        if find_cnt - find_cnt_last == 0:
            if not get_time:
                not_find_time = rospy.get_time()
                get_time = True
            if rospy.get_time() - not_find_time > 2.0:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                cmd = 'HOVER'
                print(cmd)
                get_time = False
        find_cnt_last = find_cnt