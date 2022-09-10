import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import MountControl
from mavros_msgs.srv import MountConfigure
from gazebo_msgs.srv import GetLinkState
import sys
import std_msgs.msg

if __name__ == "__main__":
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    rospy.init_node('gimbal_control'+'_'+vehicle_type+'_'+vehicle_id)
    mountCnt = rospy.Publisher(vehicle_type+'_'+vehicle_id+'/mavros/mount_control/command', MountControl, queue_size=1)
    mountConfig = rospy.ServiceProxy(vehicle_type+'_'+vehicle_id+'/mavros/mount_control/configure', MountConfigure)
    rate=rospy.Rate(30)
    gimbal_pitch_ = -60
    gimbal_yaw_ = 0.0
    gimbal_roll_ = 0.0
    srvheader=std_msgs.msg.Header()
    srvheader.stamp=rospy.Time.now()
    srvheader.frame_id="map"
    cam_pose_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cam_pose', PoseStamped, queue_size=1)
    cam_pose = PoseStamped()
    gazeboLinkstate = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)

    mountConfig(header=srvheader,mode=2,stabilize_roll=0,stabilize_yaw=0,stabilize_pitch=0)
    print(vehicle_type+'_'+vehicle_id+': Gimbal control')
    while not rospy.is_shutdown():
        msg = MountControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.mode = 2
        msg.pitch = gimbal_pitch_
        msg.roll = gimbal_roll_
        msg.yaw = gimbal_yaw_
        mountCnt.publish(msg)
        try:
            response = gazeboLinkstate(vehicle_type+'_'+vehicle_id+'::cgo3_camera_link','ground_plane::link')
        except:
            print("Gazebo model state service call failed")
        cam_pose.header.stamp = rospy.Time.now()
        cam_pose.pose = response.link_state.pose
        cam_pose_pub.publish(cam_pose)
        rate.sleep()
