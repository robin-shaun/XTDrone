import numpy as np
import matplotlib.pyplot as plt
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from control.msg import UAVDataMsg, AllUAVData, CommVerify, UavComm
import copy

class communi_verify:
    def __init__(self):
        self.uav_num = 20
        self.comm_matrix = CommVerify()
        for i in range(self.uav_num):
            self.comm_matrix.transMatrix.append(UavComm()) # if [a,b] = 1, a is a receiver of b
        self.leader_id = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]  # all
        #self.leader_id = [0,13,17] # part
        #self.leader_id = [0,1,2,3,5,9,13,17] # select
        #self.leader_id = [0,2,3,8,12,13,14,15,16,17,18,19] #random
        self.leader_num = len(self.leader_id)
        self.follower_num = self.uav_num-self.leader_num
        self.local_pose = [PoseStamped() for i in range(self.uav_num)]
        self.distance_all = [[] for i in range(self.uav_num)]
        self.sinr_all = [[] for i in range(self.uav_num)]
        self.C = [[] for i in range(self.uav_num)]   #C[a][b] is the capacity of a receiver of b
        # parameters
        self.rate = 15
        self.M = 128*self.uav_num
        self.R = self.M*self.rate
        self.B = 1e6
        self.W_noise = 1e-9
        self.epsilon = 2
        self.Pw = 0.5
        self.count = 0
        # subscribers
        self.all_uav_data_sub = rospy.Subscriber("/xtdrone_gcs/all_uav_data", AllUAVData,
                                                     self.all_uav_data_callback)
        # publishers
        self.comm_verify_pub = rospy.Publisher('/communication_verify', CommVerify, queue_size=10)

    def working(self):
        rospy.init_node("communication_verify")
        rate = rospy.Rate(self.rate)
        trans_total = self.leader_num**2+self.leader_num

        while rospy.is_shutdown() is False:
            trans_success = 0
            self.count += 1
            self.distance_all = [[] for i in range(self.uav_num)]
            self.sinr_all = [[] for i in range(self.uav_num)]
            self.C = [[] for i in range(self.uav_num)]
            #calculate the H function of each UAV and other leaders
            for id in range(self.uav_num): # UAV0 is a best leader
                if id > 0:
                    for transmitter_id in range(len(self.leader_id)):
                        if id == self.leader_id[transmitter_id]:
                            self.distance_all[id].append(0)
                        else:
                            self.distance_all[id].append(self.sinr_h_function(self.local_pose[id],self.local_pose[self.leader_id[transmitter_id]]))
            #calculate the SINR of each pair
            for id in range(self.uav_num):  # UAV0 is a best leader
                transmatrix = UavComm()
                if id > 0:
                    for transmitter_id in range(len(self.leader_id)):
                        sum_dis = sum(self.distance_all[id]) - self.distance_all[id][transmitter_id]
                        sinr = self.Pw*self.distance_all[id][transmitter_id]/(self.W_noise+self.Pw*sum_dis)
                        self.sinr_all[id].append(sinr)
                        capacity = self.B * np.log2(1+sinr)
                        self.C[id].append(capacity)
                        if capacity > self.R:
                            trans_success += 1
                            transmatrix.transMatrix.append(1)
                        else:
                            transmatrix.transMatrix.append(0)
                self.comm_matrix.transMatrix[id]= transmatrix

            if self.count % 30 == 0:  # 2s
                # print("comm_matrix", self.comm_matrix.transMatrix)
                # print("R: ", self.R)
                # print("C: ", self.C[17])
                print(self.comm_matrix.transMatrix[1].transMatrix)

            self.comm_matrix.header.stamp = rospy.Time.now()
            self.comm_matrix.leader = self.leader_id
            self.comm_verify_pub.publish(self.comm_matrix)
            rate.sleep()

    def all_uav_data_callback(self, msg):
        all_uav_data_real = msg
        for i in range(self.uav_num):
            self.local_pose[i] = copy.deepcopy(all_uav_data_real.data[i])

    def sinr_h_function(self, pos1, pos2):
        distance = (pos1.pose.position.x - pos2.pose.position.x) ** 2 + (
                                                                        pos1.pose.position.y - pos2.pose.position.y) ** 2 + (
                                                                                                                            pos1.pose.position.z - pos2.pose.position.z) ** 2
        g = 1/(1+distance)
        return g

if __name__ == "__main__":
    verify = communi_verify()
    verify.working()



