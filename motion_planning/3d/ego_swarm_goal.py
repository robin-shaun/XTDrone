import rospy
from geometry_msgs.msg import PoseStamped
import sys

vehicle_type = sys.argv[1]
vehicle_id = sys.argv[2]
goal_point = PoseStamped()

def goal_publisher(): 
    rospy.init_node(vehicle_type+"_"+vehicle_id+'_ego_swarm_goal')
    goal_pub = rospy.Publisher(vehicle_type+"_"+vehicle_id+"/move_base_simple/goal", PoseStamped, queue_size=1)
    rate = rospy.Rate(20) 

    while not rospy.is_shutdown():
        goal_point=PoseStamped()
        goal_point.header.frame_id = 'map'
        goal_point.pose.position.x=float(sys.argv[3])
        goal_point.pose.position.y=float(sys.argv[4])
        goal_point.pose.position.z=float(sys.argv[5])
        goal_point.pose.orientation.x=0
        goal_point.pose.orientation.y=0
        goal_point.pose.orientation.z=0
        goal_point.pose.orientation.w=1  

        goal_point.header.stamp = rospy.Time.now()
        goal_pub.publish(goal_point) 
        try:
            rate.sleep()
        except:
            continue

if __name__ == '__main__':
    try:
        goal_publisher()
    except rospy.ROSInterruptException:
        pass