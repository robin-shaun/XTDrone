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
        self.cmd = ''
        self.ctrl_leader = True
        self.cmd_vel_mask = False
        self.close_flag = False
        self.local_pose = PoseStamped()
        self.local_vel = Twist()
        self.m = PlotCanvas(self, self.map)
        self.m.move(180, 0)

    def initplot(self):
        self.map = self.comboBox_maps.currentText()
        self.m.canvas_update(self.map)

    def startrun(self):
        print 'start run!'
        self.color_plot = ['' for i in range(self.multirotor_num)]
        for i in range(self.multirotor_num):
            color_R = hex(random.randint(16,255))
            color_G = hex(random.randint(16,255))
            color_B = hex(random.randint(16,255))
            self.color_plot[i] = '#'+str(color_R)+str(color_G)+str(color_B) 
            self.color_plot[i] = self.color_plot[i].replace('0x','')
        self.pSend2ros = Process(target=self.run_process)
        self.pSend2ros.start()
        self.text_thread = Ros2Gui(self.multirotor_num, self.multirotor_type)
        self.text_thread.update_text.connect(self.display)
        self.text_thread.plot_array.connect(self.plot)
        self.text_thread.start()

    #publish messages to ros nodes like a keyboard
    def run_process(self):
        rospy.init_node('multirotor_pyqt5_control')
        self.text_show_info.setPlainText('data')
        if self.control_type == 'vel':
            self.multi_cmd_vel_flu_pub = [None] * self.multirotor_num
            self.multi_cmd_pub = [None] * self.multirotor_num
            for i in range(self.multirotor_num):
                self.multi_cmd_vel_flu_pub[i] = rospy.Publisher(
                    '/xtdrone/' + self.multirotor_type + '_' + str(i) + '/cmd_vel_flu',
                    Twist, queue_size=10)
                self.multi_cmd_pub[i] = rospy.Publisher('/xtdrone/' + self.multirotor_type + '_' + str(i) + '/cmd',
                                                        String,
                                                        queue_size=10)
            self.leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=10)
            self.leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=10)

        else:
            self.multi_cmd_accel_flu_pub = [None] * self.multirotor_num
            self.multi_cmd_pub = [None] * self.multirotor_num
            for i in range(self.multirotor_num):
                self.multi_cmd_accel_flu_pub[i] = rospy.Publisher(
                    '/xtdrone/' + self.multirotor_type + '_' + str(i) + '/cmd_accel_flu', Twist, queue_size=10)
                self.multi_cmd_pub[i] = rospy.Publisher('/xtdrone/' + self.multirotor_type + '_' + str(i) + '/cmd',
                                                        String,
                                                        queue_size=10)
            self.leader_cmd_accel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_accel_flu", Twist, queue_size=10)
            self.leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=10)
        last_forward = 0.0
        last_upward = 0.0
        last_leftward = 0.0
        last_orientation = 0.0
        last_ctrl_leader = False
        last_cmd_vel_mask = False
        num = 0
        rate = rospy.Rate(30)
        check_stop_flag = False
        print('StartRun!')
        while True:
            num += 1
            #print(check_stop_flag)
            if self.q_forward.empty():
                self.twist.linear.x = last_forward
            else:
                self.twist.linear.x = self.q_forward.get()
                last_forward = self.twist.linear.x
            if self.q_upward.empty():
                self.twist.linear.z = last_upward
            else:
                self.twist.linear.z = self.q_upward.get()
                last_upward = self.twist.linear.z
            if self.q_leftward.empty():
                self.twist.linear.y = last_leftward
            else:
                self.twist.linear.y = self.q_leftward.get()
                last_leftward = self.twist.linear.y
            if self.q_orientation.empty():
                self.twist.angular.z = last_orientation
            else:
                self.twist.angular.z = self.q_orientation.get()
                last_orientation = self.twist.angular.z
            if self.q_cmd.empty():
                self.cmd = ''
            else:
                self.cmd = self.q_cmd.get()
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
                    self.cmd = 'AUTO.RTL'
            for i in range(self.multirotor_num):
                if self.ctrl_leader:
                    if self.control_type == 'vel':
                        self.leader_cmd_vel_flu_pub.publish(self.twist)
                    else:
                        self.leader_cmd_accel_flu_pub.publish(self.twist)
                    self.leader_cmd_pub.publish(self.cmd)

                else:
                    if not self.cmd_vel_mask:
                        if self.control_type == 'vel':
                            self.multi_cmd_vel_flu_pub[i].publish(self.twist)
                        else:
                            self.multi_cmd_accel_flu_pub[i].publish(self.twist)
                    self.multi_cmd_pub[i].publish(self.cmd)
            rate.sleep()
            if check_stop_flag:
                self.q_stop_flag.put(True)
                rospy.signal_shutdown('STOP!')
                break

    def display(self, data):
        self.text_show_info.setPlainText(data)

    def plot(self, data):
         for i in range(self.multirotor_num):
            self.m.ax.plot(data[i][0], data[i][1], color = self.color_plot[i])
            # self.m.canvas_update(self.map)
            self.m.draw()























