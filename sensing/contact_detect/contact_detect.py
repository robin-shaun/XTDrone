import rospy
from gazebo_msgs.msg import ContactsState
import sys
a = ContactsState()
vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2])

def gazebo_contact_state_callback(msg, id):
    if (len(msg.states) > 0):
        print(vehicle_type + "_" + str(id) + " contact " + msg.states[0].collision2_name)
        print("One contact position is " + str([msg.states[0].contact_positions[0].x, msg.states[0].contact_positions[0].y, msg.states[0].contact_positions[0].z]))

if __name__ == '__main__':
    rospy.init_node(vehicle_type + "_contact_detect")
    gazebo_contacts_state_sub = [[] for i in range(vehicle_num)]
    for i in range(vehicle_num):
        gazebo_contacts_state_sub[i] = rospy.Subscriber(vehicle_type+"_" + str(i) + "/benchmarker/collision", ContactsState, gazebo_contact_state_callback, i, queue_size=1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
