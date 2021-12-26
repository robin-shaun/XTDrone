import rclpy
import time
import sys
import numpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32MultiArray
from geometry_msgs.msg import PoseStamped, Pose, Twist,Vector3

if sys.argv[2] == '6':
    from formation_dict import formation_dict_6 as formation_dict
elif sys.argv[2] == '9':
    from formation_dict import formation_dict_9 as formation_dict
elif sys.argv[2] == '18':
    from formation_dict import formation_dict_18 as formation_dict
else:
    print("Only 6, 9 and 18 UAVs are supported.")

class Leader(Node):
    def __init__(self,uav_type,leader_id,uav_num):
        self.id=leader_id
        self.pose=PoseStamped()
        self.cmd_vel_enu=Twist()
        self.uav_num=uav_num
        self.avoid_vel=Vector3()
        self.formation_config='waiting'
        self.origin_formation=formation_dict["origin"]
        self.new_formation =self.origin_formation
        self.adj_matrix=None
        self.communication_topology=None
        self.changed_id=numpy.arange(0,self.uav_num - 1)
        self.target_height_recorded=False
        self.cmd=String()
        self.timer_period=1/200
        self.Kz=0.5

        super().__init__('leader')
        self.pose_sub = self.create_subscription(PoseStamped,uav_type+'_'+str(self.id)+"/mavros/local_position/pose",self.pose_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist,"/xtdrone/leader/cmd_vel_flu",self.cmd_vel_callback,10)
        self.avoid_vel_sub = self.create_subscription(Vector3,"/xtdrone/"+uav_type+'_'+str(self.id)+"/avoid_vel",self.avoid_vel_callback,10)
        self.leader_cmd_sub = self.create_subscription(String,"/xtdrone/leader/cmd",self.cmd_callback,10)

        self.pose_pub =self.create_publisher(PoseStamped ,"/xtdrone/leader/pose",10)
        self.formation_pattern_pub =self.create_publisher(Float32MultiArray,'/xtdrone/formation_pattern',10)
        self.communication_topology_pub =self.create_publisher(Int32MultiArray,'/xtdrone/communication_topology',10)
        self.vel_enu_pub =self.create_publisher(Twist,'/xtdrone/'+uav_type+'_'+str(self.id)+'/cmd_vel_enu',10)
        self.cmd_pub =self.create_publisher(String,'/xtdrone/'+uav_type+'_'+str(self.id)+'/cmd',10)
        
        self.timer=self.create_timer(self.timer_period,self.timer_callback)

    def pose_callback(self, msg):
        self.pose = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel_enu = msg

    def cmd_callback(self, msg):
        if msg.data in formation_dict.keys():
            self.formation_config = msg.data
            print("Formation pattern: ", self.formation_config)
            # These variables are determined for KM algorithm
            self.adj_matrix = self.build_graph(self.origin_formation, formation_dict[self.formation_config])
            self.label_left = numpy.max(self.adj_matrix, axis=1)  # init label for the left set
            self.label_right = numpy.array([0] * (self.uav_num - 1))  # init label for the right set
            self.match_right = numpy.array([-1] * (self.uav_num - 1))
            self.visit_left = numpy.array([0] * (self.uav_num - 1))
            self.visit_right = numpy.array([0] * (self.uav_num - 1))
            self.slack_right = numpy.array([100] * (self.uav_num - 1))
            self.changed_id = self.KM()
            # Get a new formation pattern of UAVs based on KM.
            self.new_formation = self.get_new_formation(self.changed_id, formation_dict[self.formation_config])
            self.communication_topology = self.get_communication_topology(self.new_formation)
            self.orig_formation = self.new_formation
        else:
            self.cmd = msg

    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg

    # 'build_graph',  'find_path' and 'KM' functions are all determined for KM algorithm.
    # A graph of UAVs is established based on distances between them in 'build_graph' function.
    def build_graph(self, orig_formation, change_formation):
        distance = [[0 for i in range(self.uav_num - 1)] for j in range(self.uav_num - 1)]
        for i in range(self.uav_num - 1):
            for j in range(self.uav_num - 1):
                distance[i][j] = numpy.linalg.norm(orig_formation[:, i] - change_formation[:, j])
                distance[i][j] = int(50 - distance[i][j])
        return distance

    # Determine whether a path has been found.
    def find_path(self, i):
        self.visit_left[i] = True
        for j, match_weight in enumerate(self.adj_matrix[i], start=0):
            if self.visit_right[j]:
                continue
            gap = self.label_left[i] + self.label_right[j] - match_weight
            if gap == 0:
                self.visit_right[j] = True
                if self.match_right[j] == -1 or self.find_path(self.match_right[j]):
                    self.match_right[j] = i
                    return True
            else:
                self.slack_right[j] = min(gap, self.slack_right[j])
        return False

    # Main body of KM algorithm.
    def KM(self):
        for i in range(self.uav_num - 1):
            self.slack_right = numpy.array([100] * (self.uav_num - 1))
            while True:
                self.visit_left = numpy.array([0] * (self.uav_num - 1))
                self.visit_right = numpy.array([0] * (self.uav_num - 1))
                if self.find_path(i):
                    break
                d = numpy.inf
                for j, slack in enumerate(self.slack_right):
                    if not self.visit_right[j]:
                        d = min(d, slack)
                for k in range(self.uav_num - 1):
                    if self.visit_left[k]:
                        self.label_left[k] -= d
                    if self.visit_right[k]:
                        self.label_right[k] += d
                    else:
                        self.slack_right[k] -= d
        return self.match_right

    def get_new_formation(self, changed_id, change_formation):
        new_formation = numpy.zeros((3, self.uav_num - 1),dtype=float)
        position = numpy.zeros((3, self.uav_num - 1),dtype=float)
        changed_id = [i + 1 for i in changed_id]
        for i in range(0, self.uav_num - 1):
            position[:, i] = change_formation[:, i]

        for i in range(0, self.uav_num - 1):
            for j in range(0, self.uav_num - 1):
                if (j + 1) == changed_id[i]:
                    new_formation[:, i] = position[:, j]
        return new_formation

    def get_communication_topology(self, rel_posi):

        c_num = int((self.uav_num) / 2)
        min_num_index_list = [0] * c_num

        comm = [[] for i in range(self.uav_num)]
        communication = numpy.ones((self.uav_num, self.uav_num)) * 0
        nodes_next = []
        node_flag = [self.uav_num - 1]
        node_mid_flag = []

        rel_d = [0] * (self.uav_num - 1)

        for i in range(0, self.uav_num - 1):
            rel_d[i] = pow(rel_posi[0][i], 2) + pow(rel_posi[1][i], 2) + pow(rel_posi[2][i], 2)

        c = numpy.copy(rel_d)
        c.sort()
        count = 0

        for j in range(0,c_num):
            for i in range(0,self.uav_num-1):
                if rel_d[i] == c[j]:
                    if not i in node_mid_flag:
                        min_num_index_list[count] = i
                        node_mid_flag.append(i)
                        count = count + 1
                        if count == c_num:
                            break
            if count == c_num:
                break

        for j in range(0, c_num):
            nodes_next.append(min_num_index_list[j])

            comm[self.uav_num - 1].append(min_num_index_list[j])

        size_ = len(node_flag)

        while (nodes_next != []) and (size_ < (self.uav_num - 1)):

            next_node = nodes_next[0]
            nodes_next = nodes_next[1:]
            min_num_index_list = [0] * c_num
            node_mid_flag = []
            rel_d = [0] * (self.uav_num - 1)
            for i in range(0, self.uav_num - 1):

                if i == next_node or i in node_flag:

                    rel_d[i] = 2000
                else:

                    rel_d[i] = pow((rel_posi[0][i] - rel_posi[0][next_node]), 2) + pow(
                        (rel_posi[1][i] - rel_posi[1][next_node]), 2) + pow((rel_posi[2][i] - rel_posi[2][next_node]),
                                                                            2)
            c = numpy.copy(rel_d)
            c.sort()
            count = 0

            for j in range(0, c_num):
                for i in range(0, self.uav_num - 1):
                    if rel_d[i] == c[j]:
                        if not i in node_mid_flag:
                            min_num_index_list[count] = i
                            node_mid_flag.append(i)
                            count = count + 1
                            if count == c_num:
                                break
                if count == c_num:
                    break
            node_flag.append(next_node)

            size_ = len(node_flag)

            for j in range(0, c_num):

                if min_num_index_list[j] in node_flag:

                    nodes_next = nodes_next

                else:
                    if min_num_index_list[j] in nodes_next:
                        nodes_next = nodes_next
                    else:
                        nodes_next.append(min_num_index_list[j])

                    comm[next_node].append(min_num_index_list[j])

        for i in range(0, self.uav_num):
            for j in range(0, self.uav_num - 1):
                if i == 0:
                    if j in comm[self.uav_num - 1]:
                        communication[j + 1][i] = 1
                    else:
                        communication[j + 1][i] = 0
                else:
                    if j in comm[i - 1] and i < (j+1):
                        communication[j + 1][i] = 1
                    else:
                        communication[j + 1][i] = 0
            
        for i in range(1, self.uav_num):  # 防止某个无人机掉队
            if sum(communication[i]) == 0:
                communication[i][0] = 1
        return communication


    def timer_callback(self):
        #while True:
        # self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.avoid_vel.x
        # self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.avoid_vel.y
        # self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.avoid_vel.z
        self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x 
        self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y
        self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z
        formation_pattern=Float32MultiArray()
        data1=self.new_formation.flatten().tolist()
        data2=[]
        for i in data1:
            j=float(i)
            data2.append(j)
        formation_pattern.data=data2
        self.formation_pattern_pub.publish(formation_pattern)
        if(not self.communication_topology is None):
            communication_topology = Int32MultiArray()
            topology1=self.communication_topology.flatten().tolist()
            topology2=[]
            for i in topology1:
                j=int(i)
                topology2.append(j)
            communication_topology.data =topology2
            self.communication_topology_pub.publish(communication_topology)
        self.vel_enu_pub.publish(self.cmd_vel_enu)
        self.pose_pub.publish(self.pose)
        self.cmd_pub.publish(self.cmd)


def main():
    rclpy.init()
    leader=Leader(sys.argv[1],0,int(sys.argv[2]))
    rclpy.spin(leader)
    rclpy.shutdown()

if __name__ == '__main__':
    main()  

