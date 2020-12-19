from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy
import xtd_ui
import rospy
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from std_msgs.msg import String
from multiprocessing import Process,Queue
from PyQt5.QtCore import *
from receive import Ros2Gui
from PIL import Image
import random

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from plotcanvas import PlotCanvas

class Gui2Ros(QMainWindow,xtd_ui.Ui_MainWindow):
    def __init__(self):
        super(Gui2Ros, self).__init__()
        self.setupUi(self)
        self.map = 'indoor1'
        self.comboBox_maps.currentIndexChanged.connect(self.initplot)
        self.button_run.clicked.connect(self.startrun)
        self.close_flag = False
        self.local_pose = PoseStamped()
        self.local_vel = Twist()
        self.m = PlotCanvas(self, self.map)
        self.m.move(180, 0)
        self.flag = 0
        # rospy.init_node('multirotor_pyqt5_control')

    def initplot(self):
        self.map = self.comboBox_maps.currentText()
        self.m.canvas_update(self.map)

    def startrun(self):
        print 'start run!'
        self.init_controller()
        self.pSend2ros = Process(target=self.run_process)
        self.pSend2ros.start()
        self.text_thread = Ros2Gui(self.multirotor_select, self.multirotor_num, self.multi_type)
        self.text_thread.update_text.connect(self.display)
        self.text_thread.plot_array.connect(self.plot)
        self.text_thread.start()
        # self.pSend2ros = Process(target=self.run_process)
        # self.pSend2ros.start()

    def init_controller(self):
        
        self.text_show_info.setPlainText('data')
        self.multi_num = 0
        self.multi_type = []
        counnnt = 0
        print self.multirotor_select
        for j in self.multirotor_select:
            self.multi_num = self.multi_num + self.multirotor_num[j]   
            for id_1 in range(self.multirotor_num[j]):
                self.multi_type.append(self.multirotor_type[j])
                counnnt+=1
        self.color_plot = ['' for i in range(self.multi_num)]
        for i in range(self.multi_num):
            color_R = hex(random.randint(16,255))
            color_G = hex(random.randint(16,255))
            color_B = hex(random.randint(16,255))
            self.color_plot[i] = '#'+str(color_R)+str(color_G)+str(color_B) 
            self.color_plot[i] = self.color_plot[i].replace('0x','')
        

    #publish messages to ros nodes like a keyboard
    def run_process(self):
        rospy.init_node('multirotor_pyqt5_control')
        counnnt = 0
        if self.control_type == 'vel':
            self.multi_cmd_vel_flu_pub = [None] * self.multi_num
            self.multi_cmd_pub = [None] * self.multi_num
            for i in self.multirotor_select:
                for k in range(self.multirotor_num[i]):
                    if i == 7:
                        self.multi_cmd_vel_flu_pub[counnnt] = rospy.Publisher('/ugv_' + str(k) + '/cmd_vel', Twist, queue_size=10)
                        self.multi_cmd_pub[counnnt] = rospy.Publisher('/ugv_' + str(k) + + '/cmd', String,queue_size=10)      
                    else:                    
                        self.multi_cmd_vel_flu_pub[counnnt] = rospy.Publisher('/xtdrone/' + self.multi_type[counnnt] + '_' + str(k) + '/cmd_vel_flu', Twist, queue_size=10)
                        self.multi_cmd_pub[counnnt] = rospy.Publisher('/xtdrone/' + self.multi_type[counnnt] + '_' + str(k) + '/cmd', String,queue_size=10)
                    counnnt += 1
                self.leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=10)
                self.leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=10)

        else:
            self.multi_cmd_accel_flu_pub = [None] * self.multi_num
            self.multi_cmd_pub = [None] * self.multi_num
            for i in self.multirotor_select:
                for k in range(self.multirotor_num[i]):
                    self.multi_cmd_accel_flu_pub[i] = rospy.Publisher(
                        '/xtdrone/' + self.multi_type[counnnt] + '_' + str(k) + '/cmd_accel_flu', Twist, queue_size=10)
                    self.multi_cmd_pub[i] = rospy.Publisher('/xtdrone/' + self.multi_type[counnnt] + '_' + str(k) + '/cmd',
                                                            String,
                                                            queue_size=10)
                    counnnt = 0
            self.leader_cmd_accel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_accel_flu", Twist, queue_size=10)
            self.leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=10)
        self.twist = [Twist() for i in range (self.multi_num)]
        self.cmd = ['' for i in range (self.multi_num)]
        self.ctrl_leader = True 
        self.cmd_vel_mask = False
        for j in range(self.multi_num):
            self.twist[j].angular.x = 0.0
            self.twist[j].angular.y = 0.0
        last_forward = [0.0 for i in range(self.multi_num)]
        last_upward = [0.0 for i in range(self.multi_num)]
        last_leftward = [0.0 for i in range(self.multi_num)]
        last_orientation = [0.0 for i in range(self.multi_num)]
        last_ctrl_leader = False
        last_cmd_vel_mask = False
        last_multirotor_get_control = [0 for i in range(self.multi_num)]
        last_forward_all = 0.0
        last_upward_all = 0.0
        last_leftward_all = 0.0
        last_orientation_all = 0.0

        num = 0
        rate = rospy.Rate(30)
        check_stop_flag = False
        print('StartRun!')
        start_flag = False
        flag = False
        time = 0
        while True:
            if not start_flag:
                flag = self.q_start_control_flag.get()
            if flag: 
                time += 1
                start_flag = True
                num += 1
                if self.q_multirotor_get_control.empty():
                    multirotor_get_control = last_multirotor_get_control
                else:
                    multirotor_get_control = self.q_multirotor_get_control.get()
                    last_multirotor_get_control = multirotor_get_control
                if self.q_forward.empty():
                    for i in range(self.multi_num):
                        if multirotor_get_control[i]:
                            self.twist[i].linear.x = last_forward[i]
                else:
                    forward = self.q_forward.get()
                    for i in range(self.multi_num):
                        if multirotor_get_control[i]:
                            self.twist[i].linear.x = forward
                            last_forward[i] = self.twist[i].linear.x
                if self.q_upward.empty():
                    for i in range(self.multi_num):
                        if multirotor_get_control[i]:
                            self.twist[i].linear.z = last_upward[i]
                else:
                    upward = self.q_upward.get()
                    for i in range(self.multi_num):
                        if multirotor_get_control[i]:
                            self.twist[i].linear.z = upward
                            last_upward[i] = self.twist[i].linear.z
                if self.q_leftward.empty():
                    for i in range(self.multi_num):
                        if multirotor_get_control[i]:
                            self.twist[i].linear.y = last_leftward[i]
                else:
                    leftward = self.q_leftward.get()
                    for i in range(self.multi_num):
                        if multirotor_get_control[i]:
                            self.twist[i].linear.y = leftward
                            last_leftward[i] = self.twist[i].linear.y
                if self.q_orientation.empty():
                    for i in range(self.multi_num):
                        if multirotor_get_control[i]:
                            self.twist[i].angular.z = last_orientation[i]
                else:
                    orientation = self.q_orientation.get()
                    for i in range(self.multi_num):
                        if multirotor_get_control[i]:
                            self.twist[i].angular.z = orientation
                            last_orientation[i] = self.twist[i].angular.z
                if self.q_cmd.empty():
                    for i in range(self.multi_num):
                        if multirotor_get_control[i]:
                            self.cmd[i] = ''
                else:
                    cmd = self.q_cmd.get()
                    for i in range(self.multi_num):
                        if multirotor_get_control[i]:
                            self.cmd[i] = cmd
                            print(self.cmd[i])
                if self.q_ctrl_leader.empty():
                    self.ctrl_leader = last_ctrl_leader
                else:
                    self.ctrl_leader = self.q_ctrl_leader.get()
                    last_ctrl_leader = self.ctrl_leader
                
                if self.q_cmd_vel_mask.empty():
                    self.cmd_vel_mask = last_cmd_vel_mask
                else:
                    self.cmd_vel_mask = self.q_cmd_vel_mask.get()
                    last_cmd_vel_mask = self.cmd_vel_mask
                if self.q_stop_flag.empty():
                    pass
                else:
                    check_stop_flag = self.q_stop_flag.get()
                    if check_stop_flag:
                        for i in range(self.multi_num):
                            self.cmd[i] = 'AUTO.RTL'
                if self.ctrl_leader:
                    for i in range(self.multi_num):
                        if multirotor_get_control[i]:
                            if self.control_type == 'vel':
                                self.leader_cmd_vel_flu_pub.publish(self.twist[i])
                            else:
                                self.leader_cmd_accel_flu_pub.publish(self.twist[i])
                            self.leader_cmd_pub.publish(self.cmd[i])
                            break
                else:
                    for i in range(self.multi_num):
                        if not self.cmd_vel_mask:
                            if self.control_type == 'vel':
                                self.multi_cmd_vel_flu_pub[i].publish(self.twist[i])
                            else:
                                self.multi_cmd_accel_flu_pub[i].publish(self.twist[i])
                        self.multi_cmd_pub[i].publish(self.cmd[i])
            else:
                print 'shut down!'
            rate.sleep()
            
            if check_stop_flag:
                self.q_stop_flag.put(True)
                rospy.signal_shutdown('STOP!')
                break
        

    def display(self, data):
        self.text_show_info.setPlainText(data)

    def plot(self, data):
         for i in range(self.multi_num):
            self.m.ax.plot(data[i][0], data[i][1], color = self.color_plot[i])
            # self.m.canvas_update(self.map)
            self.m.draw()























