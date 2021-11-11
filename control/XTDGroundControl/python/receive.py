from PyQt5.QtWidgets import QApplication,QMainWindow
from PyQt5.QtCore import *
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_msgs.msg import String
from multiprocessing import Process,Queue
from nav_msgs.msg import Odometry


class Ros2Gui(QThread):
    update_text = pyqtSignal(str)
    plot_array = pyqtSignal(list)
    def __init__(self, multi_select, multi_num, multi_type):
        super(Ros2Gui, self).__init__()
        rospy.init_node('multirotor_pyqt5_receiver')
        self.multirotor_num = 0
        self.multirotor_type = multi_type
        self.map = map
        for i in multi_select:
            self.multirotor_num = self.multirotor_num + multi_num[i]
        self.multi_odom_groundtruth_sub = [None] * self.multirotor_num
        self.local_vel_sub = [None] * self.multirotor_num
        self.leader_cmd_vel_sub = [None]
        self.local_pose = [PoseStamped() for i in range(self.multirotor_num)]
        self.local_vel = [TwistStamped() for i in range(self.multirotor_num)]
        self.plot_all = [[[], []]for i in range(self.multirotor_num)]
        self.n = 0
        self.count = 0
        self.vel = Twist()
        self.time_map_x = 1
        self.time_map_y = 875/587
        #self.q_flag = flag
        count = 0
        for i in multi_select:
            for k in range(multi_num[i]):
                self.multi_odom_groundtruth_sub[count] = rospy.Subscriber('/xtdrone/'+self.multirotor_type[count]+'_'+str(k)+'/ground_truth/odom', Odometry, self.odm_groundtruth_callback, count,queue_size=1)
                self.local_vel_sub[count] = rospy.Subscriber(
                    self.multirotor_type[count] + '_' + str(k) + "/mavros/local_position/velocity_body", TwistStamped, self.local_vel_callback, count,queue_size=1)
                count += 1
                

    def run(self):
        print('RUN Thread')
        flag_get = False
        rate = rospy.Rate(30)
        while True:
            self.n = self.n+1
            self.text_all = ''
            self.text = [' ' for i in range(self.multirotor_num)]
            if self.n % 30 == 0:
                self.count = self.count+1
                if self.multirotor_num < 10:
                    for id in range(self.multirotor_num):
                        # if self.q_local_pose[id].empty():
                        self.text[id] = str(self.multirotor_type[id])+' pose:\n'+'uav'+str(id)+':\n'+'x:'+"%s"%(self.local_pose[id].pose.position.x)+'    '+'y:'\
                                        +"%s"%(self.local_pose[id].pose.position.y)+'    '+'z:'+"%s"%(self.local_pose[id].pose.position.z)\
                                        +'\n'+str(self.multirotor_type[id])+' vel:\n'+'uav'+str(id)+':\n'+'x:'+"%s"%(self.local_vel[id].twist.linear.x)+'    '+'y:'\
                                        +"%s"%(self.local_vel[id].twist.linear.y)+'    '+'z:'+"%s"%(self.local_vel[id].twist.linear.z)+'\n'
                        self.text_all = self.text_all + self.text[id]
                        if self.n > 0:
                            self.plot_all[id][0].append(self.local_pose[id].pose.position.x*self.time_map_x)
                            self.plot_all[id][1].append(self.local_pose[id].pose.position.y*self.time_map_y)
                    self.plot_array.emit(self.plot_all)
                    self.update_text.emit(self.text_all)

            rate.sleep()

    def odm_groundtruth_callback(self, msg, i):
        self.local_pose[i] = msg.pose

    def local_vel_callback(self, msg, id_1):
        self.local_vel[id_1] = msg

    def hhhhhhh(self, msg):
        self.vel = msg
