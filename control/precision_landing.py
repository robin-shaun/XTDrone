import rospy
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import TransformListener, Buffer
import sys

def local_pose_callback(data):
    global local_pose
    local_pose = data

if __name__ == '__main__':
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    rospy.init_node(vehicle_type+'_'+vehicle_id+'_precision_landing')
    tfBuffer = Buffer()
    tflistener = TransformListener(tfBuffer)
    cmd_vel_enu = Twist()   
    local_pose = PoseStamped()
    Kp = 1.0
    land_vel = 0.5
    rospy.Subscriber(vehicle_type+'_'+vehicle_id+"/mavros/local_position/pose", PoseStamped, local_pose_callback,queue_size=1)
    cmd_vel_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd_vel_enu', Twist, queue_size=1)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            tfstamped = tfBuffer.lookup_transform('map', 'tag_'+vehicle_id, rospy.Time(0))
        except:
            continue
        # print('tf:',tfstamped.transform.translation.x)
        # print(local_pose.pose.position.x)
        cmd_vel_enu.linear.x = Kp * (tfstamped.transform.translation.x - local_pose.pose.position.x)
        cmd_vel_enu.linear.y = Kp * (tfstamped.transform.translation.y - local_pose.pose.position.y)
        cmd_vel_enu.linear.z = -land_vel
        # print(cmd_vel_enu)
        cmd_vel_pub.publish(cmd_vel_enu)
        rate.sleep()

