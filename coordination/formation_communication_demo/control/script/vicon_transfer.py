import rospy
from geometry_msgs.msg import PoseStamped

def callback(data):
    #rospy.loginfo(str(data.pose.position.x)+','+str(data.pose.position.y)+','+str(data.pose.position.z))
    global posx, posy, posz, qx, qy, qz, qw
    posx = data.pose.position.x
    posy = data.pose.position.y
    posz = data.pose.position.z
    qx = data.pose.orientation.x
    qy = data.pose.orientation.y
    qz = data.pose.orientation.z
    qw = data.pose.orientation.w

posx, posy, posz, qx, qy ,qz ,qw = 0, 0, 0, 0, 0, 0, 1
rospy.init_node('transfer_f450_1')
rospy.Subscriber("/vrpn_client_node/parallel_f450_1/pose", PoseStamped, callback)
 # spin() simply keeps python from exiting until this node is stopped
position_pub = rospy.Publisher("/UAV0/mavros/vision_pose/pose", PoseStamped, queue_size=2) #creates a publisher object
setpoint_msg = PoseStamped()
rate = rospy.Rate(50) #the rate will take care of updating the publisher at a specified speed - 50hz in this case
#loop_count = 0

while True:
    setpoint_msg.header.stamp = rospy.Time.now()
    #setpoint_msg.header.seq = loop_count
    #setpoint_msg.header.frame_id = '/world'
    #rospy.loginfo(str(posx)+','+str(posy)+','+str(posz))

    setpoint_msg.pose.position.x = posx
    setpoint_msg.pose.position.y = posy
    setpoint_msg.pose.position.z = posz
    
    setpoint_msg.pose.orientation.x = qx
    setpoint_msg.pose.orientation.y = qy
    setpoint_msg.pose.orientation.z = qz
    setpoint_msg.pose.orientation.w = qw 
    position_pub.publish(setpoint_msg) 
    #loop_count = loop_count + 1
    rate.sleep()
#creates a message object of type pose stamped #http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html

