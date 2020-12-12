from PyQt5.QtWidgets import QApplication,QMainWindow
from PyQt5.QtCore import *
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_msgs.msg import String
from multiprocessing import Process,Queue


class Ros2Gui(QThread):
    update_text = pyqtSignal(str)
    plot_array = pyqtSignal(list)
    def __init__(self, multi_num, multi_type):
        super(Ros2Gui, self).__init__()
        rospy.init_node('multirotor_pyqt5_receiver')
        self.multirotor_num = multi_num
        self.multirotor_type = multi_type
        self.local_pose_sub = [None] * self.multirotor_num
        self.local_vel_sub = [None] * self.multirotor_num
        self.leader_cmd_vel_sub = [None]
        self.local_pose = [None] * self.multirotor_num
        self.local_vel = [None] * self.multirotor_num
        self.local_pose = [PoseStamped() for i in range(self.multirotor_num)]
        self.local_vel = [TwistStamped() for i in range(self.multirotor_num)]
        self.plot_all = [[[0], [0]]for i in range(self.multirotor_num)]
        self.n = 0
        self.count = 0
        self.vel = Twist()
        #self.q_flag = flag
        for id_1 in range(self.multirotor_num):
            self.local_pose_sub[id_1] = rospy.Subscriber(
                self.multirotor_type + '_' + str(id_1) + "/mavros/local_position/pose", PoseStamped, self.local_pose_callback, id_1)
            self.local_vel_sub[id_1] = rospy.Subscriber(
                self.multirotor_type + '_' + str(id_1) + "/mavros/local_position/velocity_body", TwistStamped, self.local_vel_callback, id_1)

    def run(self):
        print('RUN Thread')
        flag_get = False
        rate = rospy.Rate(30)
        while True:
            self.n = self.n+1
            self.text_all = ''
            self.text = [' ' for i in range(self.multirotor_num)]
            if self.n % 10 == 0:
                self.count = self.count+1
                if self.multirotor_num < 10:
                    for id in range(self.multirotor_num):
                        # if self.q_local_pose[id].empty():
                        self.text[id] = 'uav pose:\n'+'uav'+str(id)+':\n'+'x:'+"%s"%(self.local_pose[id].pose.position.x)+'    '+'y:'\
                                        +"%s"%(self.local_pose[id].pose.position.y)+'    '+'z:'+"%s"%(self.local_pose[id].pose.position.z)\
                                        +'\n'+'uav vel:\n'+'uav'+str(id)+':\n'+'x:'+"%s"%(self.local_vel[id].twist.linear.x)+'    '+'y:'\
                                        +"%s"%(self.local_vel[id].twist.linear.y)+'    '+'z:'+"%s"%(self.local_vel[id].twist.linear.z)+'\n'
                        self.text_all += self.text_all + self.text[id]
                        self.plot_all[id][0].append(self.local_pose[id].pose.position.x)
                        self.plot_all[id][1].append(self.local_pose[id].pose.position.y)
                    self.plot_array.emit(self.plot_all)
                    self.update_text.emit(self.text_all)
            if self.n % 50 == 0:
                self.n = 0
                print 'receiver_run'

            rate.sleep()

    def local_pose_callback(self, msg, id_1):
        self.local_pose[id_1] = msg

    def local_vel_callback(self, msg, id_1):
        self.local_vel[id_1] = msg

    def hhhhhhh(self, msg):
        self.vel = msg
