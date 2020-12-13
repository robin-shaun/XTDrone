# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'first_version.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
import numpy
from geometry_msgs.msg import Twist

from multiprocessing import Process, Queue
import sys
from plotcanvas import PlotCanvas

class Ui_MainWindow(object):
    def __init__(self):
        self.multirotor_type = 'iris'
        self.multirotor_num = 1
        self.control_type = 'vel'
        self.task = 'Flying'
        self.map = 'indoor1'
        self.initial_point = numpy.array([0.0,0.0])
        self.terminal_point = numpy.array([0.0,0.0])
        self.formation2 = 'formation1'
        self.control_type = 'vel'
        self.twist = Twist()
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.cmd2 = 'control the leader'
        self.q_cmd = Queue()
        self.q_ctrl_leader = Queue()
        self.q_forward = Queue()
        self.q_upward = Queue()
        self.q_leftward = Queue()
        self.q_orientation = Queue()
        self.q_stop_flag = Queue()
        self.q_cmd_vel_mask = Queue()
        self.q_close_flag = Queue()


    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1142, 798)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMaximumSize(QtCore.QSize(16777215, 16777215))
        MainWindow.setSizeIncrement(QtCore.QSize(1, 1))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("images/xt.ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        MainWindow.setIconSize(QtCore.QSize(32, 32))
        MainWindow.setToolButtonStyle(QtCore.Qt.ToolButtonIconOnly)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(170, 500, 710, 271))
        self.frame.setStyleSheet("border-color: rgb(46, 52, 54);\n"
"background-color: rgb(238, 238, 236);")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setLineWidth(1)
        self.frame.setObjectName("frame")
        self.label_control_board_2 = QtWidgets.QLabel(self.frame)
        self.label_control_board_2.setGeometry(QtCore.QRect(70, 10, 471, 31))
        self.label_control_board_2.setStyleSheet("\n"
"color: rgb(32, 74, 135);\n"
"font: 57 23pt \"Ubuntu\";")
        self.label_control_board_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_control_board_2.setObjectName("label_control_board_2")
        self.layoutWidget = QtWidgets.QWidget(self.frame)
        self.layoutWidget.setGeometry(QtCore.QRect(40, 60, 331, 198))
        self.layoutWidget.setObjectName("layoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.layoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_go_back_2 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_go_back_2.sizePolicy().hasHeightForWidth())
        self.label_go_back_2.setSizePolicy(sizePolicy)
        self.label_go_back_2.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.label_go_back_2.setObjectName("label_go_back_2")
        self.verticalLayout.addWidget(self.label_go_back_2)
        self.box_go_and_back_2 = QtWidgets.QDoubleSpinBox(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.box_go_and_back_2.sizePolicy().hasHeightForWidth())
        self.box_go_and_back_2.setSizePolicy(sizePolicy)
        self.box_go_and_back_2.setMaximumSize(QtCore.QSize(120, 40))
        self.box_go_and_back_2.setSizeIncrement(QtCore.QSize(0, 30))
        self.box_go_and_back_2.setStyleSheet("color: rgb(32, 74, 135);\n"
"font: 57 12pt \"Ubuntu\";")
        self.box_go_and_back_2.setMinimum(-10.0)
        self.box_go_and_back_2.setMaximum(10.0)
        self.box_go_and_back_2.setSingleStep(0.01)
        self.box_go_and_back_2.setProperty("value", 0.0)
        self.box_go_and_back_2.setObjectName("box_go_and_back_2")
        self.verticalLayout.addWidget(self.box_go_and_back_2)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_orientation = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_orientation.sizePolicy().hasHeightForWidth())
        self.label_orientation.setSizePolicy(sizePolicy)
        self.label_orientation.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.label_orientation.setObjectName("label_orientation")
        self.verticalLayout_4.addWidget(self.label_orientation)
        self.box_orientation = QtWidgets.QDoubleSpinBox(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.box_orientation.sizePolicy().hasHeightForWidth())
        self.box_orientation.setSizePolicy(sizePolicy)
        self.box_orientation.setMaximumSize(QtCore.QSize(120, 40))
        self.box_orientation.setStyleSheet("color: rgb(32, 74, 135);\n"
"font: 57 12pt \"Ubuntu\";")
        self.box_orientation.setMinimum(-0.5)
        self.box_orientation.setMaximum(0.5)
        self.box_orientation.setSingleStep(0.01)
        self.box_orientation.setObjectName("box_orientation")
        self.verticalLayout_4.addWidget(self.box_orientation)
        self.gridLayout.addLayout(self.verticalLayout_4, 1, 1, 1, 1)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_left_right_2 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_left_right_2.sizePolicy().hasHeightForWidth())
        self.label_left_right_2.setSizePolicy(sizePolicy)
        self.label_left_right_2.setStyleSheet("color: rgb(32, 74, 135);\n"
"font: 57 12pt \"Ubuntu\";")
        self.label_left_right_2.setObjectName("label_left_right_2")
        self.verticalLayout_2.addWidget(self.label_left_right_2)
        self.box_left_and_right_2 = QtWidgets.QDoubleSpinBox(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.box_left_and_right_2.sizePolicy().hasHeightForWidth())
        self.box_left_and_right_2.setSizePolicy(sizePolicy)
        self.box_left_and_right_2.setMaximumSize(QtCore.QSize(120, 40))
        self.box_left_and_right_2.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.box_left_and_right_2.setMinimum(-10.0)
        self.box_left_and_right_2.setMaximum(10.0)
        self.box_left_and_right_2.setSingleStep(0.01)
        self.box_left_and_right_2.setObjectName("box_left_and_right_2")
        self.verticalLayout_2.addWidget(self.box_left_and_right_2)
        self.gridLayout.addLayout(self.verticalLayout_2, 1, 0, 1, 1)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_up_down_2 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_up_down_2.sizePolicy().hasHeightForWidth())
        self.label_up_down_2.setSizePolicy(sizePolicy)
        self.label_up_down_2.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.label_up_down_2.setObjectName("label_up_down_2")
        self.verticalLayout_3.addWidget(self.label_up_down_2)
        self.box_up_and_down_2 = QtWidgets.QDoubleSpinBox(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.box_up_and_down_2.sizePolicy().hasHeightForWidth())
        self.box_up_and_down_2.setSizePolicy(sizePolicy)
        self.box_up_and_down_2.setMaximumSize(QtCore.QSize(120, 40))
        self.box_up_and_down_2.setStyleSheet("color: rgb(32, 74, 135);\n"
"font: 57 12pt \"Ubuntu\";")
        self.box_up_and_down_2.setMinimum(-10.0)
        self.box_up_and_down_2.setMaximum(10.0)
        self.box_up_and_down_2.setSingleStep(0.01)
        self.box_up_and_down_2.setObjectName("box_up_and_down_2")
        self.verticalLayout_3.addWidget(self.box_up_and_down_2)
        self.gridLayout.addLayout(self.verticalLayout_3, 0, 1, 1, 1)
        self.text_control_tips = QtWidgets.QTextBrowser(self.layoutWidget)
        self.text_control_tips.setStyleSheet("")
        self.text_control_tips.setLineWidth(0)
        self.text_control_tips.setOpenLinks(False)
        self.text_control_tips.setObjectName("text_control_tips")
        self.gridLayout.addWidget(self.text_control_tips, 2, 0, 1, 2)
        self.layoutWidget1 = QtWidgets.QWidget(self.frame)
        self.layoutWidget1.setGeometry(QtCore.QRect(430, 60, 234, 191))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.layoutWidget1)
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_send_command = QtWidgets.QLabel(self.layoutWidget1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_send_command.sizePolicy().hasHeightForWidth())
        self.label_send_command.setSizePolicy(sizePolicy)
        self.label_send_command.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.label_send_command.setAlignment(QtCore.Qt.AlignCenter)
        self.label_send_command.setObjectName("label_send_command")
        self.verticalLayout_5.addWidget(self.label_send_command)
        self.box_command = QtWidgets.QComboBox(self.layoutWidget1)
        self.box_command.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.box_command.setObjectName("box_command")
        self.box_command.addItem("")
        self.box_command.addItem("")
        self.box_command.addItem("")
        self.box_command.addItem("")
        self.box_command.addItem("")
        self.box_command.addItem("")
        self.box_command.addItem("")
        self.box_command.addItem("")
        self.box_command.addItem("")
        self.verticalLayout_5.addWidget(self.box_command)
        self.verticalLayout_7.addLayout(self.verticalLayout_5)
        self.label_choose_formation = QtWidgets.QLabel(self.layoutWidget1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_choose_formation.sizePolicy().hasHeightForWidth())
        self.label_choose_formation.setSizePolicy(sizePolicy)
        self.label_choose_formation.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.label_choose_formation.setAlignment(QtCore.Qt.AlignCenter)
        self.label_choose_formation.setObjectName("label_choose_formation")
        self.verticalLayout_7.addWidget(self.label_choose_formation)
        self.box_formation = QtWidgets.QComboBox(self.layoutWidget1)
        self.box_formation.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.box_formation.setObjectName("box_formation")
        self.box_formation.addItem("")
        self.box_formation.addItem("")
        self.box_formation.addItem("")
        self.box_formation.addItem("")
        self.verticalLayout_7.addWidget(self.box_formation)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_7.addItem(spacerItem)
        self.button_stop = QtWidgets.QPushButton(self.layoutWidget1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.button_stop.sizePolicy().hasHeightForWidth())
        self.button_stop.setSizePolicy(sizePolicy)
        self.button_stop.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);\n"
"border-bottom-color: rgb(186, 189, 182);\n"
"")
        self.button_stop.setCheckable(True)
        self.button_stop.setObjectName("button_stop")
        self.verticalLayout_7.addWidget(self.button_stop)
        self.gridLayout_4.addLayout(self.verticalLayout_7, 1, 0, 1, 1)
        self.frame_3 = QtWidgets.QFrame(self.centralwidget)
        self.frame_3.setGeometry(QtCore.QRect(170, 0, 710, 470))
        self.frame_3.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.frame_4 = QtWidgets.QFrame(self.centralwidget)
        self.frame_4.setGeometry(QtCore.QRect(880, 0, 261, 771))
        self.frame_4.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.layoutWidget2 = QtWidgets.QWidget(self.frame_4)
        self.layoutWidget2.setGeometry(QtCore.QRect(10, 0, 241, 761))
        self.layoutWidget2.setObjectName("layoutWidget2")
        self.verticalLayout_13 = QtWidgets.QVBoxLayout(self.layoutWidget2)
        self.verticalLayout_13.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_13.setObjectName("verticalLayout_13")
        self.label_control_board_4 = QtWidgets.QLabel(self.layoutWidget2)
        self.label_control_board_4.setStyleSheet("\n"
"color: rgb(32, 74, 135);\n"
"font: 57 23pt \"Ubuntu\";")
        self.label_control_board_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_control_board_4.setObjectName("label_control_board_4")
        self.verticalLayout_13.addWidget(self.label_control_board_4)
        self.text_show_info = QtWidgets.QTextBrowser(self.layoutWidget2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(2)
        sizePolicy.setHeightForWidth(self.text_show_info.sizePolicy().hasHeightForWidth())
        self.text_show_info.setSizePolicy(sizePolicy)
        self.text_show_info.setObjectName("text_show_info")
        self.verticalLayout_13.addWidget(self.text_show_info)
        self.more_topics = QtWidgets.QPushButton(self.layoutWidget2)
        self.more_topics.setObjectName("more_topics")
        self.verticalLayout_13.addWidget(self.more_topics)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_13.addLayout(self.horizontalLayout_2)
        self.label_website = QtWidgets.QLabel(self.layoutWidget2)
        self.label_website.setTextFormat(QtCore.Qt.AutoText)
        self.label_website.setAlignment(QtCore.Qt.AlignCenter)
        self.label_website.setOpenExternalLinks(True)
        self.label_website.setObjectName("label_website")
        self.verticalLayout_13.addWidget(self.label_website)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_13.addItem(spacerItem1)
        self.frame_5 = QtWidgets.QFrame(self.centralwidget)
        self.frame_5.setGeometry(QtCore.QRect(0, -10, 181, 781))
        self.frame_5.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.comboBox_controltype = QtWidgets.QComboBox(self.frame_5)
        self.comboBox_controltype.setGeometry(QtCore.QRect(30, 700, 101, 25))
        self.comboBox_controltype.setStyleSheet("color: rgb(32, 74, 135);\n"
"font: 57 12pt \"Ubuntu\";")
        self.comboBox_controltype.setObjectName("comboBox_controltype")
        self.comboBox_controltype.addItem("")
        self.comboBox_controltype.addItem("")
        self.Box_uavnumber = QtWidgets.QSpinBox(self.frame_5)
        self.Box_uavnumber.setGeometry(QtCore.QRect(30, 620, 101, 31))
        self.Box_uavnumber.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.Box_uavnumber.setMinimum(1)
        self.Box_uavnumber.setObjectName("Box_uavnumber")
        self.label_uavnumber = QtWidgets.QLabel(self.frame_5)
        self.label_uavnumber.setEnabled(True)
        self.label_uavnumber.setGeometry(QtCore.QRect(10, 590, 121, 19))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_uavnumber.sizePolicy().hasHeightForWidth())
        self.label_uavnumber.setSizePolicy(sizePolicy)
        self.label_uavnumber.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.label_uavnumber.setObjectName("label_uavnumber")
        self.label_uavnumber_2 = QtWidgets.QLabel(self.frame_5)
        self.label_uavnumber_2.setEnabled(True)
        self.label_uavnumber_2.setGeometry(QtCore.QRect(10, 670, 101, 19))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_uavnumber_2.sizePolicy().hasHeightForWidth())
        self.label_uavnumber_2.setSizePolicy(sizePolicy)
        self.label_uavnumber_2.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.label_uavnumber_2.setObjectName("label_uavnumber_2")
        self.button_run = QtWidgets.QPushButton(self.frame_5)
        self.button_run.setGeometry(QtCore.QRect(20, 740, 141, 27))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.button_run.sizePolicy().hasHeightForWidth())
        self.button_run.setSizePolicy(sizePolicy)
        self.button_run.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);\n"
"border-bottom-color: rgb(186, 189, 182);\n"
"")
        self.button_run.setCheckable(True)
        self.button_run.setObjectName("button_run")
        self.label_uavnumber_3 = QtWidgets.QLabel(self.frame_5)
        self.label_uavnumber_3.setEnabled(True)
        self.label_uavnumber_3.setGeometry(QtCore.QRect(10, 450, 129, 19))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_uavnumber_3.sizePolicy().hasHeightForWidth())
        self.label_uavnumber_3.setSizePolicy(sizePolicy)
        self.label_uavnumber_3.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.label_uavnumber_3.setObjectName("label_uavnumber_3")
        self.label_control_board_5 = QtWidgets.QLabel(self.frame_5)
        self.label_control_board_5.setGeometry(QtCore.QRect(10, 20, 151, 35))
        self.label_control_board_5.setStyleSheet("\n"
"color: rgb(32, 74, 135);\n"
"font: 57 23pt \"Ubuntu\";")
        self.label_control_board_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_control_board_5.setObjectName("label_control_board_5")
        self.label_uavnumber_4 = QtWidgets.QLabel(self.frame_5)
        self.label_uavnumber_4.setEnabled(True)
        self.label_uavnumber_4.setGeometry(QtCore.QRect(10, 520, 121, 19))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_uavnumber_4.sizePolicy().hasHeightForWidth())
        self.label_uavnumber_4.setSizePolicy(sizePolicy)
        self.label_uavnumber_4.setStyleSheet("font: 57 12pt \"Ubuntu\";\n"
"color: rgb(32, 74, 135);")
        self.label_uavnumber_4.setObjectName("label_uavnumber_4")
        self.comboBox_maps = QtWidgets.QComboBox(self.frame_5)
        self.comboBox_maps.setGeometry(QtCore.QRect(30, 550, 101, 25))
        self.comboBox_maps.setStyleSheet("color: rgb(32, 74, 135);\n"
"font: 57 12pt \"Ubuntu\";")
        self.comboBox_maps.setObjectName("comboBox_maps")
        self.comboBox_maps.addItem("")
        self.comboBox_maps.addItem("")
        self.comboBox_maps.addItem("")
        self.comboBox_maps.addItem("")
        self.comboBox_maps.addItem("")
        self.comboBox_maps.addItem("")
        self.comboBox_maps.addItem("")
        self.comboBox_vehicle_types = QtWidgets.QComboBox(self.frame_5)
        self.comboBox_vehicle_types.setGeometry(QtCore.QRect(30, 480, 101, 25))
        self.comboBox_vehicle_types.setStyleSheet("color: rgb(32, 74, 135);\n"
"font: 57 12pt \"Ubuntu\";")
        self.comboBox_vehicle_types.setObjectName("comboBox_vehicle_types")
        self.comboBox_vehicle_types.addItem("")
        self.comboBox_vehicle_types.addItem("")
        self.comboBox_vehicle_types.addItem("")
        self.comboBox_vehicle_types.addItem("")
        self.comboBox_vehicle_types.addItem("")
        self.comboBox_vehicle_types.addItem("")
        self.comboBox_vehicle_types.addItem("")
        self.comboBox_vehicle_types.addItem("")
        self.line_4 = QtWidgets.QFrame(self.centralwidget)
        self.line_4.setGeometry(QtCore.QRect(420, 471, 16, 27))
        self.line_4.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_4.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_4.setObjectName("line_4")
        self.line_5 = QtWidgets.QFrame(self.centralwidget)
        self.line_5.setGeometry(QtCore.QRect(558, 471, 16, 27))
        self.line_5.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_5.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_5.setObjectName("line_5")
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(170, 470, 411, 29))
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.button_world = QtWidgets.QPushButton(self.widget)
        self.button_world.setStyleSheet("background-color: rgb(238, 238, 236);\n"
"color: rgb(32, 74, 135);\n"
"font: 57 12pt \"Ubuntu\";")
        self.button_world.setCheckable(True)
        self.button_world.setObjectName("button_world")
        self.horizontalLayout.addWidget(self.button_world)
        self.button_waypoint = QtWidgets.QPushButton(self.widget)
        self.button_waypoint.setStyleSheet("background-color: rgb(238, 238, 236);\n"
"color: rgb(32, 74, 135);\n"
"font: 57 12pt \"Ubuntu\";")
        self.button_waypoint.setCheckable(True)
        self.button_waypoint.setObjectName("button_waypoint")
        self.horizontalLayout.addWidget(self.button_waypoint)
        self.button_plot = QtWidgets.QPushButton(self.widget)
        self.button_plot.setStyleSheet("background-color: rgb(238, 238, 236);\n"
"color: rgb(32, 74, 135);\n"
"font: 57 12pt \"Ubuntu\";")
        self.button_plot.setCheckable(True)
        self.button_plot.setObjectName("button_plot")
        self.horizontalLayout.addWidget(self.button_plot)
        self.frame_3.raise_()
        self.frame.raise_()
        self.frame_4.raise_()
        self.layoutWidget.raise_()
        self.button_plot.raise_()
        self.frame_5.raise_()
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionbasic_settings = QtWidgets.QAction(MainWindow)
        self.actionbasic_settings.setObjectName("actionbasic_settings")
        self.actionmulti_comfiguration = QtWidgets.QAction(MainWindow)
        self.actionmulti_comfiguration.setObjectName("actionmulti_comfiguration")
        self.actionselect_worlds = QtWidgets.QAction(MainWindow)
        self.actionselect_worlds.setObjectName("actionselect_worlds")

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        MainWindow.setTabOrder(self.box_command, self.box_orientation)
        MainWindow.setTabOrder(self.box_orientation, self.Box_uavnumber)
        MainWindow.setTabOrder(self.Box_uavnumber, self.comboBox_controltype)
        MainWindow.setTabOrder(self.comboBox_controltype, self.text_control_tips)
        MainWindow.setTabOrder(self.text_control_tips, self.box_go_and_back_2)
        MainWindow.setTabOrder(self.box_go_and_back_2, self.box_up_and_down_2)
        MainWindow.setTabOrder(self.box_up_and_down_2, self.box_formation)
        MainWindow.setTabOrder(self.box_formation, self.button_waypoint)
        MainWindow.setTabOrder(self.button_waypoint, self.text_show_info)
        MainWindow.setTabOrder(self.text_show_info, self.button_run)
        MainWindow.setTabOrder(self.button_run, self.button_stop)
        MainWindow.setTabOrder(self.button_stop, self.box_left_and_right_2)

        self.retranslateUi(MainWindow)
        #self.backend = backendthread()
        self.Set_Slot()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "XTDGroundControl"))
        self.label_control_board_2.setText(_translate("MainWindow", "control board"))
        self.label_go_back_2.setText(_translate("MainWindow", "go and back"))
        self.box_go_and_back_2.setToolTip(_translate("MainWindow", "maximum:+-10.00"))
        self.label_orientation.setText(_translate("MainWindow", "orientation"))
        self.box_orientation.setToolTip(_translate("MainWindow", "maximum:+-0.50"))
        self.label_left_right_2.setText(_translate("MainWindow", "left and right"))
        self.box_left_and_right_2.setToolTip(_translate("MainWindow", "maximum:+-10.00"))
        self.label_up_down_2.setText(_translate("MainWindow", "up and down"))
        self.box_up_and_down_2.setToolTip(_translate("MainWindow", "maximum:+-10.00"))
        self.text_control_tips.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#204a87;\">Tips: When use the </span><span style=\" font-size:12pt; font-weight:600; color:#204a87;\">offboard mode </span><span style=\" font-size:12pt; color:#204a87;\">to take off, the upforward speed must be greater than 0.3 m/s (by default)</span></p></body></html>"))
        self.label_send_command.setText(_translate("MainWindow", "send command"))
        self.box_command.setItemText(0, _translate("MainWindow", "control the leader"))
        self.box_command.setItemText(1, _translate("MainWindow", "control all"))
        self.box_command.setItemText(2, _translate("MainWindow", "arm"))
        self.box_command.setItemText(3, _translate("MainWindow", "disarm"))
        self.box_command.setItemText(4, _translate("MainWindow", "return home"))
        self.box_command.setItemText(5, _translate("MainWindow", "take off"))
        self.box_command.setItemText(6, _translate("MainWindow", "land"))
        self.box_command.setItemText(7, _translate("MainWindow", "offboard"))
        self.box_command.setItemText(8, _translate("MainWindow", "hover"))
        self.label_choose_formation.setText(_translate("MainWindow", "choose formation"))
        self.box_formation.setItemText(0, _translate("MainWindow", "waiting"))
        self.box_formation.setItemText(1, _translate("MainWindow", "formation1"))
        self.box_formation.setItemText(2, _translate("MainWindow", "formation2"))
        self.box_formation.setItemText(3, _translate("MainWindow", "formation3"))
        self.button_stop.setText(_translate("MainWindow", "STOP!"))
        self.label_control_board_4.setText(_translate("MainWindow", "vehicle states"))
        self.more_topics.setText(_translate("MainWindow", "more topics"))
        self.label_website.setToolTip(_translate("MainWindow", "XTDrone wiki"))
        self.label_website.setText(_translate("MainWindow", "<a href= https://yuque.com/xtdrone>more information</a>"))
        self.comboBox_controltype.setItemText(0, _translate("MainWindow", "vel"))
        self.comboBox_controltype.setItemText(1, _translate("MainWindow", "accel"))
        self.label_uavnumber.setText(_translate("MainWindow", "vehicle number:"))
        self.label_uavnumber_2.setText(_translate("MainWindow", "control type:"))
        self.button_run.setText(_translate("MainWindow", "RUN!"))
        self.label_uavnumber_3.setText(_translate("MainWindow", "vehicle types:"))
        self.label_control_board_5.setText(_translate("MainWindow", "Settings"))
        self.label_uavnumber_4.setText(_translate("MainWindow", "maps:"))
        self.comboBox_maps.setItemText(0, _translate("MainWindow", "indoor1"))
        self.comboBox_maps.setItemText(1, _translate("MainWindow", "indoor2"))
        self.comboBox_maps.setItemText(2, _translate("MainWindow", "indoor3"))
        self.comboBox_maps.setItemText(3, _translate("MainWindow", "outdoor1"))
        self.comboBox_maps.setItemText(4, _translate("MainWindow", "outdoor2"))
        self.comboBox_maps.setItemText(5, _translate("MainWindow", "outdoor3"))
        self.comboBox_maps.setItemText(6, _translate("MainWindow", "robocup"))
        self.comboBox_vehicle_types.setItemText(0, _translate("MainWindow", "iris"))
        self.comboBox_vehicle_types.setItemText(1, _translate("MainWindow", "solo"))
        self.comboBox_vehicle_types.setItemText(2, _translate("MainWindow", "typhoon_h480"))
        self.comboBox_vehicle_types.setItemText(3, _translate("MainWindow", "tailsitter"))
        self.comboBox_vehicle_types.setItemText(4, _translate("MainWindow", "quadplane"))
        self.comboBox_vehicle_types.setItemText(5, _translate("MainWindow", "tiltrotor"))
        self.comboBox_vehicle_types.setItemText(6, _translate("MainWindow", "plane"))
        self.comboBox_vehicle_types.setItemText(7, _translate("MainWindow", "rovor"))
        self.button_world.setText(_translate("MainWindow", "world"))
        self.button_waypoint.setText(_translate("MainWindow", "way point"))
        self.button_plot.setText(_translate("MainWindow", "plots"))
        self.actionbasic_settings.setText(_translate("MainWindow", "basic settings"))
        self.actionmulti_comfiguration.setText(_translate("MainWindow", "multi comfiguration"))
        self.actionselect_worlds.setText(_translate("MainWindow", "select worlds"))

    def Set_Slot(self):

        self.button_stop.setEnabled(False)
        self.button_run.setEnabled(True)
        self.button_run.clicked.connect(self.on_button_run_clicked)
        self.button_stop.clicked.connect(self.on_button_stop_clicked)
        self.button_world.clicked.connect(self.on_button_world_clicked)
        self.button_waypoint.clicked.connect(self.on_button_waypoint_clicked)

        self.box_go_and_back_2.valueChanged.connect(self.valueChange_forward_exp_vel)
        self.box_go_and_back_2.setReadOnly(True)
        self.box_up_and_down_2.valueChanged.connect(self.valueChange_upward_exp_vel)
        self.box_up_and_down_2.setReadOnly(True)
        self.box_left_and_right_2.valueChanged.connect(self.valueChange_leftward_exp_vel)
        self.box_left_and_right_2.setReadOnly(True)
        self.box_orientation.valueChanged.connect(self.valueChange_orientation_exp)
        self.box_orientation.setReadOnly(True)
        self.box_command.currentIndexChanged.connect(self.get_command)
        self.box_formation.currentIndexChanged.connect(self.get_formation)
        self.comboBox_maps.currentIndexChanged.connect(self.initplot)
        
    def initplot(self):
        self.map = self.comboBox_maps.currentText()
        self.m = PlotCanvas(self, map=self.map)
        self.m.move(170, 0)

    def on_button_waypoint_clicked(self):
        if self.button_world.isChecked():
            self.button_world.toggle()

    def on_button_world_clicked(self):
        if self.button_waypoint.isChecked():
            self.button_waypoint.toggle()

    def on_button_run_clicked(self):
        self.map = str(self.comboBox_maps.currentText())
        self.multirotor_num = int(self.Box_uavnumber.value())
        self.multirotor_type = str(self.comboBox_vehicle_types.currentText())
        self.control_type = str(self.comboBox_controltype.currentText())
        if self.multirotor_num != 6 and self.multirotor_num != 9 and self.multirotor_num != 18:
            self.box_formation.setVisible(False)
        self.box_go_and_back_2.setReadOnly(False)
        self.box_up_and_down_2.setReadOnly(False)
        self.box_left_and_right_2.setReadOnly(False)
        self.box_orientation.setReadOnly(False)
        self.button_stop.setEnabled(True)
        stop_flag = False
        self.q_stop_flag.put(stop_flag)
        self.button_run.setEnabled(False)

    def on_button_stop_clicked(self):
        self.box_go_and_back_2.setProperty("value", 0.0)
        self.box_up_and_down_2.setProperty("value", 0.0)
        self.box_left_and_right_2.setProperty("value", 0.0)
        self.box_orientation.setProperty("value", 0.0)
        self.box_go_and_back_2.setReadOnly(True)
        self.box_up_and_down_2.setReadOnly(True)
        self.box_left_and_right_2.setReadOnly(True)
        self.box_orientation.setReadOnly(True)
        self.button_run.setEnabled(True)
        self.button_stop.setEnabled(False)
        stop_flag = True
        self.q_stop_flag.put(stop_flag)
        self.button_stop.toggle()
        self.button_run.toggle()

    def valueChange_forward_exp_vel(self):
        forward_exp = self.box_go_and_back_2.value()
        #if forward_exp
        self.q_forward.put(forward_exp)
        #print self.box_go_and_back_2.value()
    def valueChange_upward_exp_vel(self):
        upward_exp = self.box_up_and_down_2.value()
        print(upward_exp)
        self.q_upward.put(upward_exp)
        #print('upward_exp:',self.twist.linear.z)
    def valueChange_leftward_exp_vel(self):
        leftward_exp = self.box_left_and_right_2.value()
        self.q_leftward.put(leftward_exp)
        #print('left_ward_exp:',self.twist.linear.y)
    def valueChange_orientation_exp(self):
        orientation_exp = self.box_orientation.value()
        self.q_orientation.put(orientation_exp )
        #print('orientation_exp:',self.twist.angular.z)

   

    def get_command(self):
        self.cmd2 = str(self.box_command.currentText())
        print('cmd2:' + self.cmd2)
        if self.cmd2 == 'control all':
            cmd = ''
            ctrl_leader = False
            self.q_cmd.put(cmd)
            self.q_ctrl_leader.put(ctrl_leader)
        elif self.cmd2 == 'arm':
            cmd = 'ARM'
            self.q_cmd.put(cmd)
        elif self.cmd2 == 'disarm':
            cmd = 'DISARM'
            self.q_cmd.put(cmd)
        elif self.cmd2 == 'return home':
            cmd = 'AUTO.RTL'
            self.q_cmd.put(cmd)
        elif self.cmd2 == 'take off':
            cmd = 'AUTO.TAKEOFF'
            self.q_cmd.put(cmd)
        elif self.cmd2 == 'land':
            cmd = 'AUTO.LAND'
            self.q_cmd.put(cmd)
        elif self.cmd2 == 'offboard':
            cmd = 'OFFBOARD'
            self.q_cmd.put(cmd)
        elif self.cmd2 == 'hover':
            cmd = 'HOVER'
            self.q_cmd.put(cmd)
            cmd_vel_mask = False
            self.q_cmd_vel_mask.put(cmd_vel_mask)
            self.box_go_and_back_2.setProperty("value", 0.0)
            self.box_up_and_down_2.setProperty("value", 0.0)
            self.box_left_and_right_2.setProperty("value", 0.0)
            self.box_orientation.setProperty("value", 0.0)
        else:
            cmd = ''
            ctrl_leader = True
            self.q_cmd.put(cmd)
            self.q_ctrl_leader.put(ctrl_leader)
        #print('cmd:'+self.cmd)

    def get_formation(self):
        self.formation2 = str(self.box_formation.currentText())
        cmd_vel_mask = True
        self.q_cmd_vel_mask.put(cmd_vel_mask)
        if self.multirotor_num == 6:
            if self.formation2 == 'formation1':
                formation = 'T'
                self.q_cmd.put(formation)
            elif self.formation2 == 'formation2':
                formation = 'diamond'
                self.q_cmd.put(formation)
            elif self.formation2 == 'formation3':
                formation = 'triangle'
                self.q_cmd.put(formation)
            else:
                formation = 'waiting'
                self.q_cmd.put(formation)
        elif self.multirotor_num == 9:
            if self.formation2 == 'formation1':
                formation = 'cube'
                self.q_cmd.put(formation)
            elif self.formation2 == 'formation2':
                formation = 'pyramid'
                self.q_cmd.put(formation)
            elif self.formation2 == 'formation3':
                formation = 'triangle'
                self.q_cmd.put(formation)
            else:
                formation = 'waiting'
                self.q_cmd.put(formation)
        else:
            if self.formation2 == 'formation1':
                formation = 'cuboid'
                self.q_cmd.put(formation)
            elif self.formation2 == 'formation2':
                formation = 'sphere'
                self.q_cmd.put(formation)
            elif self.formation2 == 'formation3':
                formation = 'diamond'
                self.q_cmd.put(formation)
            else:
                formation = 'waiting'
                self.q_cmd.put(formation)
        #print(self.formation)


if __name__== '__main__':
    app = QApplication(sys.argv)
    mainWindow = QMainWindow()          # creat a window object
    #ui = first_version.Ui_MainWindow()    # use the class of my ui window
    ui = Ui_MainWindow()
    ui.setupUi(mainWindow)              # send the window to my ui
    mainWindow.show()
    #pipe1, = multiprocessing.Pipe()
    sys.exit(app.exec_())               # main circulation





